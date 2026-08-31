// =============================================================================
// superscalar_hash.v — SuperscalarHash execution unit
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// SuperscalarHash is used during dataset item generation.
// It executes a randomly generated "superscalar program" of integer
// instructions over 8 x 64-bit registers (r0..r7), producing a deterministic
// hash of the dataset item index combined with cache data.
//
// Instruction set (RandomX spec §6.2 / superscalar.hpp), all operands 64-bit:
//   ISUB_R   : dst = dst - src
//   IXOR_R   : dst = dst ^ src
//   IADD_RS  : dst = dst + (src << mod_shift)
//   IMUL_R   : dst = dst * src                       (low 64 bits)
//   IROR_C   : dst = ror(dst, imm32 % 64)
//   IADD_C7/8/9 : dst = dst + signExtend2sCompl(imm32)
//   IXOR_C7/8/9 : dst = dst ^ signExtend2sCompl(imm32)
//   IMULH_R  : dst = hi64_unsigned(dst * src)
//   ISMULH_R : dst = hi64_signed(dst * src)
//   IMUL_RCP : dst = dst * reciprocal(imm32)
// (The C7/C8/C9 variants only differ in their x86 encoding length, the
//  arithmetic behaviour is identical.)
//
// Instruction word encoding used by this core (filled by the program
// generator / cache-init logic):
//   [63:56] opcode   — SuperscalarInstructionType (see SS_* localparams)
//   [55:53] dst      — destination register index (r0..r7)
//   [52:50] src      — source register index (r0..r7), unused by *_C / IROR_C
//   [49:48] mod_shift— shift amount for IADD_RS (0..3)
//   [47:32] reserved — must be zero
//   [31: 0] imm32    — immediate (zero-extended for IROR_C / IMUL_RCP,
//                      sign-extended for IADD_C* / IXOR_C*)
//
// Micro-architecture: strictly in-order, one instruction at a time.
// Each instruction is issued to alu_int and retired (register writeback)
// before the next one is fetched, which removes all RAW/WAW hazards of the
// registered ALU output. IMUL_RCP first runs the reciprocal unit
// (restoring division, 64+bsr(divisor) cycles) and then issues a plain
// 64x64 multiply.
//
// TODO: superscalar scheduling (multiple parallel execution ports) — the
//       result is bit-identical, this is a throughput optimisation only.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module superscalar_hash (
    input  wire         clk,
    input  wire         rst_n,

    // Start: load program and initial registers, begin execution
    input  wire         start,

    // Program memory write port (filled by cache-init logic)
    input  wire         prog_wr_en,
    input  wire [11:0]  prog_wr_addr,  // up to 4096 instructions
    input  wire [63:0]  prog_wr_data,  // encoded instruction word

    // Program base address inside the program buffer (execution starts here).
    // Allows several SuperscalarHash programs (e.g. the 8 programs used by
    // dataset generation) to share one program buffer.
    input  wire [11:0]  prog_base,

    // Program length (number of instructions to execute)
    input  wire [11:0]  prog_len,

    // Initial register values (from cache data XOR)
    input  wire [63:0]  init_r0, init_r1, init_r2, init_r3,
    input  wire [63:0]  init_r4, init_r5, init_r6, init_r7,

    // Output register values
    output reg  [63:0]  out_r0, out_r1, out_r2, out_r3,
    output reg  [63:0]  out_r4, out_r5, out_r6, out_r7,

    // Status
    output reg          busy,
    // Done pulse
    output reg          done
);

// ---------------------------------------------------------------------------
// Program buffer — stores encoded SuperscalarHash instructions
// ---------------------------------------------------------------------------
reg [63:0] prog_mem [0:4095];

always @(posedge clk) begin
    if (prog_wr_en)
        prog_mem[prog_wr_addr] <= prog_wr_data;
end

// ---------------------------------------------------------------------------
// Register file
// ---------------------------------------------------------------------------
reg [63:0] rf [0:7]; // r0..r7

// ---------------------------------------------------------------------------
// Instruction decode fields
// ---------------------------------------------------------------------------
reg  [63:0] cur_instr;
wire [7:0]  ss_opcode  = cur_instr[63:56];
wire [2:0]  ss_dst_idx = cur_instr[55:53];
wire [2:0]  ss_src_idx = cur_instr[52:50];
wire [1:0]  ss_shift   = cur_instr[49:48];
wire [31:0] ss_imm32   = cur_instr[31:0];
wire [63:0] ss_imm_sx  = {{32{ss_imm32[31]}}, ss_imm32};  // sign-extended
wire [63:0] ss_imm_zx  = {32'b0, ss_imm32};               // zero-extended

// Register file read mux
wire [63:0] rf_dst = rf[ss_dst_idx];
wire [63:0] rf_src = rf[ss_src_idx];

// ---------------------------------------------------------------------------
// SuperscalarHash opcodes (values match RandomX SuperscalarInstructionType)
// ---------------------------------------------------------------------------
localparam SS_ISUB_R   = 8'd0;
localparam SS_IXOR_R   = 8'd1;
localparam SS_IADD_RS  = 8'd2;
localparam SS_IMUL_R   = 8'd3;
localparam SS_IROR_C   = 8'd4;
localparam SS_IADD_C7  = 8'd5;
localparam SS_IXOR_C7  = 8'd6;
localparam SS_IADD_C8  = 8'd7;
localparam SS_IXOR_C8  = 8'd8;
localparam SS_IADD_C9  = 8'd9;
localparam SS_IXOR_C9  = 8'd10;
localparam SS_IMULH_R  = 8'd11;
localparam SS_ISMULH_R = 8'd12;
localparam SS_IMUL_RCP = 8'd13;

// alu_int opcode encoding (see rtl/alu_int.v)
localparam ALU_IADD_RS  = 6'd0;
localparam ALU_ISUB_R   = 6'd2;
localparam ALU_IMUL_R   = 6'd4;
localparam ALU_IMULH_R  = 6'd6;
localparam ALU_ISMULH_R = 6'd8;
localparam ALU_IXOR_R   = 6'd12;
localparam ALU_IROR_R   = 6'd14;

// ---------------------------------------------------------------------------
// Integer ALU (shared datapath with the VM execution unit)
// ---------------------------------------------------------------------------
reg  [5:0]  alu_opcode;
reg  [63:0] alu_src_a, alu_src_b, alu_imm;
reg  [1:0]  alu_shift;
reg         alu_en;
wire [63:0] alu_result;
wire        alu_valid;

alu_int u_alu (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (alu_en),
    .opcode       (alu_opcode),
    .src_a        (alu_src_a),
    .src_b        (alu_src_b),
    .shift_amt    (alu_shift),
    .imm32_sext   (alu_imm),
    .cond         (4'd0),        // no CBRANCH/ISTORE in SuperscalarHash
    .mem_is_l1    (1'b0),
    .result       (alu_result),
    .result_valid (alu_valid),
    .busy         (),
    .branch_taken (),         // not used in SuperscalarHash
    .mem_wr_en    (),         // not used in SuperscalarHash
    .mem_wr_addr  (),
    .mem_wr_data  (),
    .mem_wr_level ()
);

// ---------------------------------------------------------------------------
// IMUL_RCP reciprocal unit (RandomX spec §5.5.11 / reciprocal.c)
// Shared restoring-division unit, see rtl/recip.v.
// ---------------------------------------------------------------------------
reg         rcp_start;
reg  [63:0] rcp_divisor;
wire [63:0] rcp_quot;
wire        rcp_valid;

recip u_recip (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (rcp_start),
    .divisor  (rcp_divisor),
    .quotient (rcp_quot),
    .valid    (rcp_valid),
    .busy     ()
);

// ---------------------------------------------------------------------------
// Control FSM
// ---------------------------------------------------------------------------
localparam ST_IDLE  = 3'd0;
localparam ST_FETCH = 3'd1;
localparam ST_DECODE= 3'd2;
localparam ST_RCP   = 3'd3;
localparam ST_ISSUE = 3'd4;
localparam ST_WB    = 3'd5;
localparam ST_DONE  = 3'd6;

reg [2:0]  state;
// 13 bits: prog_base + prog_len may reach 4096 (program 7 with 512
// instructions), which must be representable so the fetch loop terminates.
reg [12:0] pc;      // absolute index into prog_mem
reg [2:0]  wb_dst;      // writeback destination register

integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= ST_IDLE;
        pc          <= 13'd0;
        alu_en      <= 1'b0;
        busy        <= 1'b0;
        done        <= 1'b0;
        cur_instr   <= 64'b0;
        alu_opcode  <= 6'd0;
        alu_src_a   <= 64'b0;
        alu_src_b   <= 64'b0;
        alu_shift   <= 2'd0;
        alu_imm     <= 64'b0;
        wb_dst      <= 3'd0;
        rcp_divisor <= 64'b0;
        rcp_start   <= 1'b0;
        for (i = 0; i < 8; i = i + 1)
            rf[i] <= 64'b0;
        out_r0 <= 64'b0; out_r1 <= 64'b0; out_r2 <= 64'b0; out_r3 <= 64'b0;
        out_r4 <= 64'b0; out_r5 <= 64'b0; out_r6 <= 64'b0; out_r7 <= 64'b0;
    end else begin
        alu_en    <= 1'b0;
        done      <= 1'b0;
        rcp_start <= 1'b0;

        case (state)
            // -----------------------------------------------------------
            ST_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    rf[0] <= init_r0; rf[1] <= init_r1;
                    rf[2] <= init_r2; rf[3] <= init_r3;
                    rf[4] <= init_r4; rf[5] <= init_r5;
                    rf[6] <= init_r6; rf[7] <= init_r7;
                    pc    <= {1'b0, prog_base};
                    busy  <= 1'b1;
                    state <= ST_FETCH;
                end
            end

            // -----------------------------------------------------------
            ST_FETCH: begin
                if (pc >= ({1'b0, prog_base} + {1'b0, prog_len})) begin
                    state <= ST_DONE;
                end else begin
                    cur_instr <= prog_mem[pc[11:0]];
                    pc        <= pc + 13'd1;
                    state     <= ST_DECODE;
                end
            end

            // -----------------------------------------------------------
            // Decode: map the SuperscalarHash opcode onto the alu_int ISA.
            // Register operands are sampled here, one instruction is in
            // flight at a time so no forwarding is required.
            // -----------------------------------------------------------
            ST_DECODE: begin
                alu_src_a <= rf_dst;
                alu_src_b <= rf_src;
                alu_shift <= 2'd0;
                alu_imm   <= 64'b0;
                wb_dst    <= ss_dst_idx;
                state     <= ST_ISSUE;

                case (ss_opcode)
                    SS_ISUB_R: begin
                        alu_opcode <= ALU_ISUB_R;
                    end
                    SS_IXOR_R: begin
                        alu_opcode <= ALU_IXOR_R;
                    end
                    SS_IADD_RS: begin
                        alu_opcode <= ALU_IADD_RS;
                        alu_shift  <= ss_shift;
                    end
                    SS_IMUL_R: begin
                        alu_opcode <= ALU_IMUL_R;
                    end
                    SS_IROR_C: begin
                        // rotate right by a constant: reuse IROR_R, the ALU
                        // only looks at the low 6 bits of src_b
                        alu_opcode <= ALU_IROR_R;
                        alu_src_b  <= ss_imm_zx;
                    end
                    SS_IADD_C7, SS_IADD_C8, SS_IADD_C9: begin
                        // dst = dst + sext(imm32): IADD_RS with src = 0
                        alu_opcode <= ALU_IADD_RS;
                        alu_src_b  <= 64'b0;
                        alu_imm    <= ss_imm_sx;
                    end
                    SS_IXOR_C7, SS_IXOR_C8, SS_IXOR_C9: begin
                        alu_opcode <= ALU_IXOR_R;
                        alu_src_b  <= ss_imm_sx;
                    end
                    SS_IMULH_R: begin
                        alu_opcode <= ALU_IMULH_R;
                    end
                    SS_ISMULH_R: begin
                        alu_opcode <= ALU_ISMULH_R;
                    end
                    SS_IMUL_RCP: begin
                        // multiply by reciprocal(imm32) — compute it first
                        alu_opcode  <= ALU_IMUL_R;
                        rcp_divisor <= ss_imm_zx;
                        rcp_start   <= 1'b1;
                        state       <= ST_RCP;
                    end
                    default: begin
                        // unknown opcode behaves as a NOP (dst unchanged)
                        alu_opcode <= 6'd63;
                    end
                endcase
            end

            // -----------------------------------------------------------
            // Reciprocal computation (rtl/recip.v)
            // -----------------------------------------------------------
            ST_RCP: begin
                if (rcp_valid) begin
                    alu_src_b <= rcp_quot;
                    state     <= ST_ISSUE;
                end
            end

            // -----------------------------------------------------------
            ST_ISSUE: begin
                alu_en <= 1'b1;
                state  <= ST_WB;
            end

            // -----------------------------------------------------------
            // Writeback: alu_int registers its output, wait for result_valid
            // -----------------------------------------------------------
            ST_WB: begin
                if (alu_valid) begin
                    rf[wb_dst] <= alu_result;
                    state      <= ST_FETCH;
                end
            end

            // -----------------------------------------------------------
            ST_DONE: begin
                out_r0 <= rf[0]; out_r1 <= rf[1];
                out_r2 <= rf[2]; out_r3 <= rf[3];
                out_r4 <= rf[4]; out_r5 <= rf[5];
                out_r6 <= rf[6]; out_r7 <= rf[7];
                done   <= 1'b1;
                busy   <= 1'b0;
                state  <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
