// =============================================================================
// superscalar_hash.v — SuperscalarHash skeleton
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// SuperscalarHash is used during dataset item generation.
// It executes a randomly generated "superscalar program" of integer instructions
// over 8 × 64-bit registers (r0..r7), producing a deterministic hash of
// the dataset item index combined with cache data.
//
// Each SuperscalarHash program:
//   - Up to 3170 instructions (RANDOMX_SUPERSCALAR_MAX_LATENCY * SLOTS)
//   - Instruction types: IADD_RS, IADD_C7/C8/C9, ISUB_R, IMUL_R, UMULH_R,
//                        SMULH_R, IMUL_RCP, IROR_C, IXOR_R, ISWAP_R
//   - Executed in-order but with superscalar latency scheduling
//
// This skeleton:
//   - Provides an instruction buffer (program_mem)
//   - Implements a simple in-order sequential execution FSM
//   - Drives alu_int for integer operations
//   - Produces 8 × 64-bit output register file
//
// TODO: Implement the superscalar scheduling (parallel execution ports).
// TODO: Implement full instruction decode for all SuperscalarHash opcodes.
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

    // Program length (number of instructions to execute)
    input  wire [11:0]  prog_len,

    // Initial register values (from cache data XOR)
    input  wire [63:0]  init_r0, init_r1, init_r2, init_r3,
    input  wire [63:0]  init_r4, init_r5, init_r6, init_r7,

    // Output register values
    output reg  [63:0]  out_r0, out_r1, out_r2, out_r3,
    output reg  [63:0]  out_r4, out_r5, out_r6, out_r7,

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
// Instruction decode fields (from 64-bit instruction word)
// RandomX SuperscalarHash instruction encoding (simplified):
//   [63:56] = opcode
//   [55:53] = dst register index
//   [52:50] = src register index
//   [49:32] = immediate (18-bit, sign-extended to 64)
//   [31: 0] = additional immediate
// TODO: Verify exact bit encoding against RandomX spec §7
// ---------------------------------------------------------------------------
reg [63:0] cur_instr;
wire [7:0]  ss_opcode  = cur_instr[63:56];
wire [2:0]  ss_dst_idx = cur_instr[55:53];
wire [2:0]  ss_src_idx = cur_instr[52:50];
wire [63:0] ss_imm     = {{46{cur_instr[49]}}, cur_instr[49:32]};

// Register file read mux
wire [63:0] rf_dst = rf[ss_dst_idx];
wire [63:0] rf_src = rf[ss_src_idx];

// ALU control
reg  [5:0]  alu_opcode;
reg  [63:0] alu_src_a, alu_src_b, alu_imm;
reg  [1:0]  alu_shift;
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
    .result       (alu_result),
    .result_valid (alu_valid),
    .branch_taken (),         // not used in SuperscalarHash
    .mem_wr_en    (),         // not used in SuperscalarHash
    .mem_wr_addr  (),
    .mem_wr_data  (),
    .mem_wr_level ()
);

// FSM
localparam ST_IDLE  = 2'd0;
localparam ST_FETCH = 2'd1;
localparam ST_EXEC  = 2'd2;
localparam ST_DONE  = 2'd3;

reg [1:0]  state;
reg [11:0] pc;
reg        alu_en;
reg [2:0]  wb_dst;  // writeback destination register
reg        wb_en;   // writeback enable

// Opcode mappings for SuperscalarHash (TODO: match spec table exactly)
localparam SS_IADD_RS  = 8'd0;
localparam SS_IADD_C9  = 8'd1;
localparam SS_ISUB_R   = 8'd2;
localparam SS_IMUL_R   = 8'd3;
localparam SS_UMULH_R  = 8'd4;
localparam SS_SMULH_R  = 8'd5;
localparam SS_IMUL_RCP = 8'd6;
localparam SS_IROR_C   = 8'd7;
localparam SS_IXOR_R   = 8'd8;
localparam SS_ISWAP_R  = 8'd9;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= ST_IDLE;
        pc         <= 12'd0;
        alu_en     <= 1'b0;
        wb_en      <= 1'b0;
        done       <= 1'b0;
        cur_instr  <= 64'b0;
        alu_opcode <= 6'd0;
        alu_src_a  <= 64'b0;
        alu_src_b  <= 64'b0;
        alu_shift  <= 2'd0;
        alu_imm    <= 64'b0;
        wb_dst     <= 3'd0;
        {rf[0],rf[1],rf[2],rf[3],rf[4],rf[5],rf[6],rf[7]} <= {8{64'b0}};
        {out_r0,out_r1,out_r2,out_r3,out_r4,out_r5,out_r6,out_r7} <= {8{64'b0}};
    end else begin
        alu_en <= 1'b0;
        wb_en  <= 1'b0;
        done   <= 1'b0;

        // Writeback stage
        if (wb_en && alu_valid) begin
            rf[wb_dst] <= alu_result;
        end

        case (state)
            ST_IDLE: begin
                if (start) begin
                    // Load initial registers
                    rf[0] <= init_r0; rf[1] <= init_r1;
                    rf[2] <= init_r2; rf[3] <= init_r3;
                    rf[4] <= init_r4; rf[5] <= init_r5;
                    rf[6] <= init_r6; rf[7] <= init_r7;
                    pc    <= 12'd0;
                    state <= ST_FETCH;
                end
            end

            ST_FETCH: begin
                if (pc >= prog_len) begin
                    state <= ST_DONE;
                end else begin
                    cur_instr <= prog_mem[pc];
                    state     <= ST_EXEC;
                end
            end

            ST_EXEC: begin
                // Decode and dispatch to ALU
                // TODO: Complete full opcode mapping
                case (ss_opcode)
                    SS_IADD_RS: begin alu_opcode <= 6'd0; alu_shift <= cur_instr[51:50]; end
                    SS_ISUB_R:  begin alu_opcode <= 6'd2; alu_shift <= 2'd0; end
                    SS_IMUL_R:  begin alu_opcode <= 6'd4; alu_shift <= 2'd0; end
                    SS_UMULH_R: begin alu_opcode <= 6'd6; alu_shift <= 2'd0; end
                    SS_SMULH_R: begin alu_opcode <= 6'd8; alu_shift <= 2'd0; end
                    SS_IMUL_RCP:begin alu_opcode <= 6'd10; alu_shift <= 2'd0; end
                    SS_IXOR_R:  begin alu_opcode <= 6'd12; alu_shift <= 2'd0; end
                    default:    begin alu_opcode <= 6'd0; alu_shift <= 2'd0; end
                endcase
                alu_src_a <= rf_dst;
                alu_src_b <= rf_src;
                alu_imm   <= ss_imm;
                alu_en    <= 1'b1;
                wb_dst    <= ss_dst_idx;
                wb_en     <= 1'b1;
                pc        <= pc + 12'd1;
                state     <= ST_FETCH;
            end

            ST_DONE: begin
                out_r0 <= rf[0]; out_r1 <= rf[1];
                out_r2 <= rf[2]; out_r3 <= rf[3];
                out_r4 <= rf[4]; out_r5 <= rf[5];
                out_r6 <= rf[6]; out_r7 <= rf[7];
                done   <= 1'b1;
                state  <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
