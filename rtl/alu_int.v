// =============================================================================
// alu_int.v — Integer Execution Unit
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Handles all RandomX integer instructions:
//   IADD_RS, IADD_M, ISUB_R, ISUB_M, IMUL_R, IMUL_M,
//   IMULH_R, IMULH_M, ISMULH_R, ISMULH_M, IMUL_RCP,
//   INEG_R, IXOR_R, IXOR_M, IROR_R, IROL_R, ISWAP_R,
//   CBRANCH, ISTORE
//
// All operations are 64-bit. Results feed back to the register file.
//
// TODO: IMUL_RCP (reciprocal multiply) requires a lookup table or division
//       unit for the 64-bit modular inverse — currently stubbed.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module alu_int (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         en,          // Execute enable (instruction valid)

    // Opcode encoding (matches RandomX VM instruction decoder output)
    input  wire [5:0]   opcode,

    // Source operands
    input  wire [63:0]  src_a,       // Destination register value (read-modify-write)
    input  wire [63:0]  src_b,       // Source register or immediate

    // For IADD_RS: shift amount (2 bits) and condition modifiers
    input  wire [1:0]   shift_amt,   // shift for IADD_RS
    input  wire [63:0]  imm32_sext,  // Sign-extended 32-bit immediate

    // mod byte fields (from instruction decoder)
    input  wire [3:0]   cond,        // mod.cond: CBRANCH condition / ISTORE L3 select
    input  wire         mem_is_l1,   // mod.mem != 0: ISTORE targets L1 (else L2)

    // Result
    output reg  [63:0]  result,
    output reg          result_valid,

    // CBRANCH: branch taken signal and target (PC update)
    output reg          branch_taken,

    // ISTORE: memory write request
    output reg          mem_wr_en,
    output reg  [63:0]  mem_wr_addr,
    output reg  [63:0]  mem_wr_data,
    output reg  [1:0]   mem_wr_level // L0/L1/L2 scratchpad level
);

// ---------------------------------------------------------------------------
// Opcode constants (aligned with RandomX spec ISA table)
// ---------------------------------------------------------------------------
localparam OP_IADD_RS   = 6'd0;
localparam OP_IADD_M    = 6'd1;
localparam OP_ISUB_R    = 6'd2;
localparam OP_ISUB_M    = 6'd3;
localparam OP_IMUL_R    = 6'd4;
localparam OP_IMUL_M    = 6'd5;
localparam OP_IMULH_R   = 6'd6;
localparam OP_IMULH_M   = 6'd7;
localparam OP_ISMULH_R  = 6'd8;
localparam OP_ISMULH_M  = 6'd9;
localparam OP_IMUL_RCP  = 6'd10;
localparam OP_INEG_R    = 6'd11;
localparam OP_IXOR_R    = 6'd12;
localparam OP_IXOR_M    = 6'd13;
localparam OP_IROR_R    = 6'd14;
localparam OP_IROL_R    = 6'd15;
localparam OP_ISWAP_R   = 6'd16;
localparam OP_CBRANCH   = 6'd17;
localparam OP_ISTORE    = 6'd18;

// ---------------------------------------------------------------------------
// 128-bit multiplication (unsigned) for IMULH
// ---------------------------------------------------------------------------
wire [127:0] mul128_u = {64'b0, src_a} * {64'b0, src_b};

// ---------------------------------------------------------------------------
// 128-bit multiplication (signed) for ISMULH — use 2's complement
// ---------------------------------------------------------------------------
wire [63:0]  src_a_neg = ~src_a + 64'd1;
wire [63:0]  src_b_neg = ~src_b + 64'd1;
wire         a_neg     = src_a[63];
wire         b_neg     = src_b[63];
wire [63:0]  abs_a     = a_neg ? src_a_neg : src_a;
wire [63:0]  abs_b     = b_neg ? src_b_neg : src_b;
wire [127:0] mul128_s  = {64'b0, abs_a} * {64'b0, abs_b};
wire [63:0]  ismulh_res_pos = mul128_s[127:64];
wire [63:0]  ismulh_res_neg = ~ismulh_res_pos + 64'd1;
wire [63:0]  ismulh_res = (a_neg ^ b_neg) ? ismulh_res_neg : ismulh_res_pos;

// ---------------------------------------------------------------------------
// Rotate operations
// ---------------------------------------------------------------------------
wire [5:0]   rot_amt   = src_b[5:0];
wire [63:0]  ror_res   = (src_a >> rot_amt) | (src_a << (6'd63 - rot_amt + 6'd1));
wire [63:0]  rol_res   = (src_a << rot_amt) | (src_a >> (6'd63 - rot_amt + 6'd1));

// ---------------------------------------------------------------------------
// CBRANCH condition logic (RandomX spec 5.5.10)
//   shift = mod.cond + ConditionOffset(8)
//   imm   = imm32_sext | (1 << shift); imm &= ~(1 << (shift-1))
//   dst  += imm; branch if (dst & (255 << shift)) == 0
// ---------------------------------------------------------------------------
wire [4:0]  cb_shift = {1'b0, cond} + 5'd8;          // 8..23
wire [63:0] cb_imm   = (imm32_sext | (64'd1 << cb_shift))
                     & ~(64'd1 << (cb_shift - 5'd1));
wire [63:0] cb_res   = src_a + cb_imm;
wire        cb_taken = ((cb_res & (64'hFF << cb_shift)) == 64'b0);

// ---------------------------------------------------------------------------
// Combinational result MUX
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        result       <= 64'b0;
        result_valid <= 1'b0;
        branch_taken <= 1'b0;
        mem_wr_en    <= 1'b0;
        mem_wr_addr  <= 64'b0;
        mem_wr_data  <= 64'b0;
        mem_wr_level <= 2'd2;
    end else begin
        result_valid <= 1'b0;
        branch_taken <= 1'b0;
        mem_wr_en    <= 1'b0;

        if (en) begin
            result_valid <= 1'b1;
            case (opcode)
                OP_IADD_RS: // dst = dst + (src << shift) + imm32
                    result <= src_a + (src_b << shift_amt) + imm32_sext;

                OP_IADD_M:  // dst = dst + mem[src+imm] (src_b = loaded mem value)
                    result <= src_a + src_b;

                OP_ISUB_R:  // dst = dst - src
                    result <= src_a - src_b;

                OP_ISUB_M:  // dst = dst - mem[src+imm]
                    result <= src_a - src_b;

                OP_IMUL_R:  // dst = dst * src (low 64)
                    result <= src_a * src_b;

                OP_IMUL_M:
                    result <= src_a * src_b;

                OP_IMULH_R: // dst = high64(dst * src) unsigned
                    result <= mul128_u[127:64];

                OP_IMULH_M:
                    result <= mul128_u[127:64];

                OP_ISMULH_R: // dst = high64(dst * src) signed
                    result <= ismulh_res;

                OP_ISMULH_M:
                    result <= ismulh_res;

                OP_IMUL_RCP: begin
                    // TODO: Compute 2^128 / src_b (64-bit reciprocal multiply)
                    // Requires division unit or precomputed LUT — skeleton only
                    result <= src_a; // placeholder
                end

                OP_INEG_R:  // dst = -dst
                    result <= ~src_a + 64'd1;

                OP_IXOR_R:  // dst = dst ^ src
                    result <= src_a ^ src_b;

                OP_IXOR_M:
                    result <= src_a ^ src_b;

                OP_IROR_R:  // dst = ror(dst, src[5:0])
                    result <= ror_res;

                OP_IROL_R:  // dst = rol(dst, src[5:0])
                    result <= rol_res;

                OP_ISWAP_R: begin
                    // ISWAP writes src_b to dst and src_a to src register
                    // The VM handles the second write; result = src_b
                    result <= src_b;
                end

                OP_CBRANCH: begin
                    // dst += modified imm; branch if condition byte is zero
                    result       <= cb_res;
                    branch_taken <= cb_taken;
                end

                OP_ISTORE: begin
                    // mem[dst + imm32] = src  (masked to scratchpad level)
                    result_valid <= 1'b0; // no register write
                    mem_wr_en    <= 1'b1;
                    mem_wr_addr  <= src_a + imm32_sext;
                    mem_wr_data  <= src_b;
                    // Spec: mod.cond >= 14 → L3, else mod.mem selects L1/L2
                    mem_wr_level <= (cond >= 4'd14) ? 2'd2
                                                    : (mem_is_l1 ? 2'd0 : 2'd1);
                end

                default:
                    result <= src_a; // NOP — pass through
            endcase
        end
    end
end

endmodule
