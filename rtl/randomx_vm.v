// =============================================================================
// randomx_vm.v — RandomX Virtual Machine
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Executes one RandomX program (256 instructions) for ITERATIONS loop
// iterations, following the VM loop of the RandomX specification §4.6.2:
//
//   spAddr0 = mx, spAddr1 = ma                          (before the loop)
//   repeat ITERATIONS times:
//     spMix    = r[readReg0] ^ r[readReg1]
//     spAddr0 ^= spMix       ; spAddr0 &= ScratchpadL3Mask64
//     spAddr1 ^= spMix >> 32 ; spAddr1 &= ScratchpadL3Mask64
//     r[i] ^= load64(spAddr0 + 8*i)                     i = 0..7
//     f[i]  = cvt_i32x2(load64(spAddr1 + 8*i))          i = 0..3
//     e[i]  = mask(cvt_i32x2(load64(spAddr1 + 8*(4+i))))i = 0..3
//     execute the 256-instruction program
//     mx ^= r[readReg2] ^ r[readReg3] ; mx &= CacheLineAlignMask
//     r[i] ^= datasetItem(datasetOffset + ma)[i]        i = 0..7
//     swap(ma, mx)
//     store64(spAddr1 + 8*i , r[i])                     i = 0..7
//     store128(spAddr0 + 16*i, f[i] ^ e[i])             i = 0..3
//     spAddr0 = spAddr1 = 0
//
// Register file:
//   r[0..7]  — 8 × 64-bit integer registers
//   f[0..3]  — 4 × 128-bit FP pair registers (lo/hi = two doubles)
//   e[0..3]  — 4 × 128-bit FP pair registers (always positive)
//   a[0..3]  — 4 × 128-bit FP pair registers (read-only, from program entropy)
//
// Program configuration (spec §4.6.4) is derived from the 16 entropy words
// written through the `cfg_wr_*` port before `start`:
//   entropy[ 0.. 7] → a registers (getSmallPositiveFloatBits)
//   entropy[ 8]     → ma (masked with CacheLineAlignMask)
//   entropy[10]     → mx
//   entropy[12]     → readReg0..readReg3 selection bits
//   entropy[13]     → datasetOffset = (e13 % (DatasetExtraItems+1)) * 64
//   entropy[14/15]  → eMask0 / eMask1 (getFloatMask)
//
// Instruction word layout used by this VM (256 × 64-bit program buffer):
//   [63:56] = opcode (see OPC_* below, aligned with alu_int.v numbering)
//   [55:52] = dst  ([2:0] for integer, [1:0]/[2] for FP registers)
//   [51:48] = src
//   [47:32] = mod  ([1:0] = mod.mem, [3:2] = mod.shift, [7:4] = mod.cond)
//   [31: 0] = imm32
//
// All 29 RandomX instructions are decoded:
//   integer      : IADD_RS IADD_M ISUB_R ISUB_M IMUL_R IMUL_M IMULH_R IMULH_M
//                  ISMULH_R ISMULH_M IMUL_RCP INEG_R IXOR_R IXOR_M IROR_R
//                  IROL_R ISWAP_R
//   control/mem  : CBRANCH ISTORE CFROUND NOP
//   floating pt  : FADD_R FADD_M FSUB_R FSUB_M FSCAL_R FMUL_R FDIV_M FSQRT_R
//                  FSWAP_R
// Memory operands use the spec addressing rules: `src == dst` selects the L3
// mask with an immediate-only address, otherwise mod.mem selects L1 or L2.
// Every FP instruction is executed twice (low and high half of the register
// pair) through the shared fpu_double unit.
//
// CBRANCH uses a compile pre-pass (ST_COMPILE) that computes per-instruction
// branch targets from register usage (spec §5.5.10); taken branches redirect
// `ic` to target+1.
//
// Final step (only when `do_final` is set, i.e. the last program of the
// 8-program chain): every 64-byte block of the scratchpad is streamed through
// AesHash1R and the resulting 64-byte digest replaces the `a` registers, as in
// getFinalResult() of the reference implementation. The caller then applies
// Blake2b over the whole 256-byte register file exposed on `regfile_out`.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module randomx_vm #(
    // Number of VM loop iterations per program (spec: 2048).
    parameter ITERATIONS = 2048,
    // Number of 64-bit scratchpad words folded by the final hash step
    // (2 MiB / 8 B = 262144). Simulation builds may shrink this to match the
    // reduced scratchpad depth of scratchpad_mem.
    parameter SP_WORDS   = 262144
) (
    input  wire          clk,
    input  wire          rst_n,

    // Start pulse: begin VM execution with loaded program and configuration
    input  wire          start,

    // Program memory write port (filled externally before start)
    input  wire          prog_wr_en,
    input  wire [7:0]    prog_wr_addr,  // 256-instruction program
    input  wire [63:0]   prog_wr_data,

    // Program configuration entropy write port (16 × 64-bit, before start)
    input  wire          cfg_wr_en,
    input  wire [3:0]    cfg_wr_addr,
    input  wire [63:0]   cfg_wr_data,

    // Scratchpad interface (to scratchpad_mem)
    output reg           sp_rd_en,
    output reg  [20:0]   sp_rd_addr,
    output reg  [1:0]    sp_rd_level,
    input  wire [63:0]   sp_rd_data,
    input  wire          sp_rd_valid,
    output reg           sp_wr_en,
    output reg  [20:0]   sp_wr_addr,
    output reg  [1:0]    sp_wr_level,
    output reg  [63:0]   sp_wr_data,

    // Dataset interface (to hbm_dataset_if)
    output reg           ds_req_valid,
    output reg  [31:0]   ds_req_idx,
    input  wire          ds_req_ready,
    input  wire [511:0]  ds_resp_data,
    input  wire          ds_resp_valid,
    output reg           ds_resp_ready,

    // Run the final AesHash1R pass over the scratchpad after this program
    // (set for the last program of the chain only)
    input  wire          do_final,

    // AesHash1R streaming interface (spec §3.5)
    output reg           aes_start,
    output reg           aes_blk_valid,
    output reg           aes_blk_last,
    output reg  [511:0]  aes_data_in,
    input  wire [511:0]  aes_hash_out,
    input  wire          aes_hash_valid,

    // Full 256-byte register file (r0..r7, f, e, a) in reference byte order
    output wire [2047:0] regfile_out,

    // Final 512-bit AesHash1R digest (valid when do_final was set)
    output reg  [511:0]  hash_out,
    output reg           done
);

// ---------------------------------------------------------------------------
// Constants (spec §1.2 / §4.6)
// ---------------------------------------------------------------------------
localparam [63:0] CACHE_LINE_ALIGN_MASK = 64'h000000007FFFFFC0;
localparam [20:0] SP_L3_MASK64          = 21'h1FFFC0; // 64-byte aligned L3
localparam [63:0] SP_L1_MASK            = 64'h0000000000003FF8;
localparam [63:0] SP_L2_MASK            = 64'h000000000003FFF8;
localparam [63:0] SP_L3_MASK            = 64'h00000000001FFFF8;
// (1 << (52 + dynamicExponentBits(4))) - 1
localparam [63:0] DYN_MANT_MASK         = 64'h00FFFFFFFFFFFFFF;
// DatasetExtraItems + 1 = 524288 + 1
localparam [20:0] DS_EXTRA_MOD          = 21'd524289;

// Last iteration index / last folded scratchpad word
localparam [31:0] ITER_LAST = ITERATIONS - 1;
localparam [31:0] FOLD_LAST = SP_WORDS - 1;   // last 64-bit word index

// ---------------------------------------------------------------------------
// VM opcode encoding (integer opcodes match alu_int.v)
// ---------------------------------------------------------------------------
localparam OPC_IADD_RS  = 8'd0;
localparam OPC_IADD_M   = 8'd1;
localparam OPC_ISUB_R   = 8'd2;
localparam OPC_ISUB_M   = 8'd3;
localparam OPC_IMUL_R   = 8'd4;
localparam OPC_IMUL_M   = 8'd5;
localparam OPC_IMULH_R  = 8'd6;
localparam OPC_IMULH_M  = 8'd7;
localparam OPC_ISMULH_R = 8'd8;
localparam OPC_ISMULH_M = 8'd9;
localparam OPC_IMUL_RCP = 8'd10;
localparam OPC_INEG_R   = 8'd11;
localparam OPC_IXOR_R   = 8'd12;
localparam OPC_IXOR_M   = 8'd13;
localparam OPC_IROR_R   = 8'd14;
localparam OPC_IROL_R   = 8'd15;
localparam OPC_ISWAP_R  = 8'd16;
localparam OPC_CBRANCH  = 8'd17;
localparam OPC_ISTORE   = 8'd18;
localparam OPC_FADD_R   = 8'd19;
localparam OPC_FADD_M   = 8'd20;
localparam OPC_FSUB_R   = 8'd21;
localparam OPC_FSUB_M   = 8'd22;
localparam OPC_FSCAL_R  = 8'd23;
localparam OPC_FMUL_R   = 8'd24;
localparam OPC_FDIV_M   = 8'd25;
localparam OPC_FSQRT_R  = 8'd26;
localparam OPC_FSWAP_R  = 8'd27;
localparam OPC_CFROUND  = 8'd28;

// fpu_double opcodes
localparam FP_FADD  = 5'd0;
localparam FP_FSUB  = 5'd2;
localparam FP_FMUL  = 5'd4;
localparam FP_FDIV  = 5'd5;
localparam FP_FSQRT = 5'd6;
localparam FP_FSCAL = 5'd7;

// ---------------------------------------------------------------------------
// Program buffer — 256 × 64-bit instruction words
// ---------------------------------------------------------------------------
reg [63:0] prog_mem [0:255];

always @(posedge clk) begin
    if (prog_wr_en)
        prog_mem[prog_wr_addr] <= prog_wr_data;
end

// ---------------------------------------------------------------------------
// Program configuration entropy — 16 × 64-bit words
// ---------------------------------------------------------------------------
reg [63:0] entropy [0:15];

always @(posedge clk) begin
    if (cfg_wr_en)
        entropy[cfg_wr_addr] <= cfg_wr_data;
end

// ---------------------------------------------------------------------------
// Register file
// ---------------------------------------------------------------------------
reg [63:0]  r [0:7];           // integer registers r0..r7
reg [63:0]  f_lo [0:3];        // FP f registers low half
reg [63:0]  f_hi [0:3];        // FP f registers high half
reg [63:0]  e_lo [0:3];        // FP e registers low half
reg [63:0]  e_hi [0:3];        // FP e registers high half
reg [63:0]  a_lo [0:3];        // FP a registers low half (const)
reg [63:0]  a_hi [0:3];        // FP a registers high half (const)

// Reference RegisterFile layout (256 bytes): r[8], f[4][2], e[4][2], a[4][2].
// Byte 0 of the struct maps to bit 0, so the concatenation runs high -> low.
assign regfile_out = {
    a_hi[3], a_lo[3], a_hi[2], a_lo[2], a_hi[1], a_lo[1], a_hi[0], a_lo[0],
    e_hi[3], e_lo[3], e_hi[2], e_lo[2], e_hi[1], e_lo[1], e_hi[0], e_lo[0],
    f_hi[3], f_lo[3], f_hi[2], f_lo[2], f_hi[1], f_lo[1], f_hi[0], f_lo[0],
    r[7], r[6], r[5], r[4], r[3], r[2], r[1], r[0]
};
reg [63:0]  ma;                // Memory address register (dataset ptr)
reg [63:0]  mx;                // Memory mix register
reg [1:0]   fprc;              // FP rounding mode control
reg [7:0]   ic;                // Instruction counter
reg [31:0]  iter_cnt;          // Loop iteration counter

// Program configuration
reg [2:0]   read_reg0, read_reg1, read_reg2, read_reg3;
reg [31:0]  ds_off_items;      // datasetOffset expressed in 64-byte items
reg [63:0]  e_mask0, e_mask1;

// Scratchpad pointers for the current iteration
reg [20:0]  sp_addr0, sp_addr1;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

// (the helper functions below use blocking assignments on function-local
//  variables, which is legal but flagged by Verilator when they are called
//  from a clocked process)
/* verilator lint_off BLKSEQ */

// Convert a signed 32-bit integer to an IEEE-754 double (exact, no rounding)
function [63:0] cvt_i32;
    input [31:0] v;
    reg          sgn;
    reg [31:0]   mag;
    reg [5:0]    msb;
    reg [83:0]   shifted;
    integer      k;
    begin
        sgn = v[31];
        mag = sgn ? (~v + 32'd1) : v;
        if (mag == 32'd0) begin
            cvt_i32 = 64'd0;
        end else begin
            msb = 6'd0;
            for (k = 0; k < 32; k = k + 1)
                if (mag[k]) msb = k[5:0];
            shifted = {52'b0, mag} << (6'd52 - msb);
            cvt_i32 = {sgn, (11'd1023 + {5'd0, msb}), shifted[51:0]};
        end
    end
endfunction

// getSmallPositiveFloatBits(): exponent = (x >> 59) + 1023, mantissa = x[51:0]
function [63:0] small_pos_float;
    input [63:0] v;
    begin
        small_pos_float = {1'b0, (11'd1023 + {6'd0, v[63:59]}), v[51:0]};
    end
endfunction

// getFloatMask(): (x & mask22bit) | staticExponent(x)
function [63:0] float_mask;
    input [63:0] v;
    begin
        float_mask = {1'b0, (11'd1023 | {v[63:60], 7'd0}), 30'd0, v[21:0]};
    end
endfunction

// Apply the e-register exponent/mantissa mask
function [63:0] mask_e;
    input [63:0] v;
    input [63:0] m;
    begin
        mask_e = (v & DYN_MANT_MASK) | m;
    end
endfunction

/* verilator lint_on BLKSEQ */

// ---------------------------------------------------------------------------
// Instruction decode
// ---------------------------------------------------------------------------
wire [63:0] cur_instr  = prog_mem[ic];
wire [7:0]  op         = cur_instr[63:56];
wire [3:0]  dst_idx    = cur_instr[55:52];
wire [3:0]  src_idx    = cur_instr[51:48];
wire [15:0] mod        = cur_instr[47:32];
wire [31:0] imm32      = cur_instr[31: 0];
wire [63:0] imm64_sext = {{32{imm32[31]}}, imm32};

wire [2:0]  dst_r      = dst_idx[2:0];
wire [2:0]  src_r      = src_idx[2:0];
wire [1:0]  dst_f      = dst_idx[1:0];
wire [1:0]  src_f      = src_idx[1:0];
wire        dst_is_e   = dst_idx[2];      // FSWAP_R may target f or e

wire        mod_mem    = (mod[1:0] != 2'b00);
wire [1:0]  mod_shift  = mod[3:2];

wire [63:0] r_src      = r[src_r];
wire [63:0] r_dst      = r[dst_r];

// Memory operand addressing (spec §5.5): src == dst → immediate-only, L3
wire        same_reg   = (dst_r == src_r);
wire [63:0] mem_addr_r = same_reg ? imm64_sext : (r_src + imm64_sext);
wire [63:0] mem_mask   = same_reg ? SP_L3_MASK
                                  : (mod_mem ? SP_L1_MASK : SP_L2_MASK);
wire [63:0] mem_masked = mem_addr_r & mem_mask;
wire [20:0] mem_addr   = mem_masked[20:0];
wire [1:0]  mem_level  = same_reg ? 2'd2 : (mod_mem ? 2'd0 : 2'd1);

// CFROUND: fprc = ror(r[src], imm32 % 64) & 3
wire [127:0] cfr_ror   = {r_src, r_src} >> imm32[5:0];

// ---------------------------------------------------------------------------
// ALU instance
// ---------------------------------------------------------------------------
reg  [5:0]  alu_op;
reg  [63:0] alu_a, alu_b, alu_imm;
reg  [1:0]  alu_shift;
reg         alu_en;
wire [63:0] alu_result;
wire        alu_valid;
wire        branch_taken;
wire        alu_mem_wr;
wire [63:0] alu_mem_addr, alu_mem_data;
wire [1:0]  alu_mem_level;

alu_int u_alu (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (alu_en),
    .opcode       (alu_op),
    .src_a        (alu_a),
    .src_b        (alu_b),
    .shift_amt    (alu_shift),
    .imm32_sext   (alu_imm),
    .cond         (mod[7:4]),
    .mem_is_l1    (mod_mem),
    .result       (alu_result),
    .result_valid (alu_valid),
    .busy         (),
    .branch_taken (branch_taken),
    .mem_wr_en    (alu_mem_wr),
    .mem_wr_addr  (alu_mem_addr),
    .mem_wr_data  (alu_mem_data),
    .mem_wr_level (alu_mem_level)
);

// ---------------------------------------------------------------------------
// FPU instance
// ---------------------------------------------------------------------------
reg  [4:0]  fpu_op;
reg  [63:0] fpu_a, fpu_b;
reg         fpu_en;
wire [63:0] fpu_result;
wire        fpu_valid;
wire        fpu_busy;

fpu_double u_fpu (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (fpu_en),
    .opcode       (fpu_op),
    .src_a        (fpu_a),
    .src_b        (fpu_b),
    .round_mode   (fprc),
    .result       (fpu_result),
    .result_valid (fpu_valid),
    .busy         (fpu_busy)
);

// ---------------------------------------------------------------------------
// Loop variable for reset / bulk initialization
// ---------------------------------------------------------------------------
integer i;

// ---------------------------------------------------------------------------
// FSM states
// ---------------------------------------------------------------------------
localparam ST_IDLE      = 5'd0;
localparam ST_MOD       = 5'd1;   // datasetOffset modulo (64 cycles)
localparam ST_COMPILE   = 5'd2;   // CBRANCH target pre-pass
localparam ST_LOOP      = 5'd3;   // iteration start: spAddr update
localparam ST_LD_R      = 5'd4;   // scratchpad → r registers
localparam ST_LD_R_W    = 5'd5;
localparam ST_LD_F      = 5'd6;   // scratchpad → f/e registers
localparam ST_LD_F_W    = 5'd7;
localparam ST_FETCH     = 5'd8;
localparam ST_DECODE    = 5'd9;
localparam ST_MEM_RD    = 5'd10;  // scratchpad operand load
localparam ST_INT_WAIT  = 5'd11;  // wait for registered ALU result
localparam ST_BR_WAIT   = 5'd12;  // wait for CBRANCH ALU result
localparam ST_FP_ISSUE  = 5'd13;  // issue one half of an FP instruction
localparam ST_FP_WAIT   = 5'd14;
localparam ST_WB        = 5'd15;  // advance instruction counter
localparam ST_MX        = 5'd16;  // mx update
localparam ST_DS_REQ    = 5'd17;  // dataset item request
localparam ST_DS_WAIT   = 5'd18;
localparam ST_ST_R      = 5'd19;  // r registers → scratchpad
localparam ST_ST_F      = 5'd20;  // f ^ e → scratchpad
localparam ST_FIN_RD    = 5'd21;  // final AesHash1R: scratchpad read
localparam ST_FIN_W     = 5'd22;  // final AesHash1R: absorb 64-byte block
localparam ST_FIN_HASH  = 5'd23;  // final AesHash1R: wait for the digest
localparam ST_DONE      = 5'd24;

reg [4:0] state;

// Execution bookkeeping
reg [2:0]  wb_dst;
reg        wb_int_en;
reg [3:0]  seq_i;          // scratchpad load/store element counter
reg        mem_for_fp;     // pending scratchpad load feeds the FPU
reg [1:0]  fp_dst;
reg        fp_is_e;
reg [4:0]  fp_opc;
reg [63:0] fp_b_lo, fp_b_hi;
reg        fp_half;        // 0 = low half, 1 = high half
reg [63:0] acc [0:7];      // final hash XOR accumulator
reg [31:0] fold_cnt;

// datasetOffset modulo state
reg [63:0] mod_sh;
reg [20:0] mod_rem;
reg [6:0]  mod_cnt;
wire [20:0] mod_shifted = {mod_rem[19:0], mod_sh[63]};
wire [20:0] mod_next    = (mod_shifted >= DS_EXTRA_MOD)
                        ? (mod_shifted - DS_EXTRA_MOD) : mod_shifted;

// ---------------------------------------------------------------------------
// CBRANCH target table — filled by the ST_COMPILE pre-pass
// target[j] = index of last instruction before j writing j's dst register
// (or 0xFF if none / clobbered by a previous CBRANCH); jump goes to target+1
// ---------------------------------------------------------------------------
reg [7:0] branch_target [0:255];
reg [7:0] reg_usage [0:7];
reg [7:0] cc;                    // compile pass counter
reg [7:0] cb_target_q;           // latched target of in-flight CBRANCH

wire [63:0] c_instr = prog_mem[cc];
wire [7:0]  c_op    = c_instr[63:56];
wire [2:0]  c_dst   = c_instr[54:52];
wire [2:0]  c_src   = c_instr[50:48];

// Convenience: current FP destination register value for the active half
wire [63:0] fp_a_cur = fp_is_e ? (fp_half ? e_hi[fp_dst] : e_lo[fp_dst])
                               : (fp_half ? f_hi[fp_dst] : f_lo[fp_dst]);

// Scratchpad element addresses (spAddr0 / spAddr1 + 8 * seq_i)
wire [20:0] sp0_addr = sp_addr0 + {14'd0, seq_i[2:0], 3'd0};
wire [20:0] sp1_addr = sp_addr1 + {14'd0, seq_i[2:0], 3'd0};

// Register mixing values (spec §4.6.2)
wire [63:0] sp_mix = r[read_reg0] ^ r[read_reg1];
wire [63:0] mx_mix = r[read_reg2] ^ r[read_reg3];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 8; i = i + 1) r[i]    <= 64'b0;
        for (i = 0; i < 4; i = i + 1) f_lo[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) f_hi[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) e_lo[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) e_hi[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) a_lo[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) a_hi[i] <= 64'b0;
        for (i = 0; i < 8; i = i + 1) acc[i]  <= 64'b0;
        ma           <= 64'b0;
        mx           <= 64'b0;
        fprc         <= 2'b00;
        ic           <= 8'd0;
        iter_cnt     <= 32'd0;
        state        <= ST_IDLE;
        alu_en       <= 1'b0;
        fpu_en       <= 1'b0;
        wb_int_en    <= 1'b0;
        wb_dst       <= 3'd0;
        sp_rd_en     <= 1'b0;
        sp_rd_addr   <= 21'b0;
        sp_rd_level  <= 2'd2;
        sp_wr_en     <= 1'b0;
        sp_wr_addr   <= 21'b0;
        sp_wr_level  <= 2'd2;
        sp_wr_data   <= 64'b0;
        ds_req_valid <= 1'b0;
        ds_req_idx   <= 32'b0;
        ds_resp_ready<= 1'b0;
        aes_start    <= 1'b0;
        aes_blk_valid<= 1'b0;
        aes_blk_last <= 1'b0;
        aes_data_in  <= 512'b0;
        hash_out     <= 512'b0;
        done         <= 1'b0;
        alu_op       <= 6'd0;
        alu_a        <= 64'b0;
        alu_b        <= 64'b0;
        alu_imm      <= 64'b0;
        alu_shift    <= 2'd0;
        fpu_op       <= 5'd0;
        fpu_a        <= 64'b0;
        fpu_b        <= 64'b0;
        fp_dst       <= 2'd0;
        fp_is_e      <= 1'b0;
        fp_opc       <= 5'd0;
        fp_b_lo      <= 64'b0;
        fp_b_hi      <= 64'b0;
        fp_half      <= 1'b0;
        mem_for_fp   <= 1'b0;
        seq_i        <= 4'd0;
        fold_cnt     <= 32'd0;
        cc           <= 8'd0;
        cb_target_q  <= 8'd0;
        mod_sh       <= 64'b0;
        mod_rem      <= 21'd0;
        mod_cnt      <= 7'd0;
        read_reg0    <= 3'd0;
        read_reg1    <= 3'd2;
        read_reg2    <= 3'd4;
        read_reg3    <= 3'd6;
        ds_off_items <= 32'd0;
        e_mask0      <= 64'b0;
        e_mask1      <= 64'b0;
        sp_addr0     <= 21'b0;
        sp_addr1     <= 21'b0;
    end else begin
        alu_en    <= 1'b0;
        fpu_en    <= 1'b0;
        done          <= 1'b0;
        aes_start     <= 1'b0;
        aes_blk_valid <= 1'b0;
        aes_blk_last  <= 1'b0;
        sp_rd_en  <= 1'b0;
        sp_wr_en  <= 1'b0;

        // Scratchpad writeback from ALU (ISTORE)
        if (alu_mem_wr) begin
            sp_wr_en    <= 1'b1;
            sp_wr_addr  <= alu_mem_addr[20:0];
            sp_wr_level <= alu_mem_level;
            sp_wr_data  <= alu_mem_data;
        end

        // Integer writeback
        if (alu_valid && wb_int_en) begin
            r[wb_dst] <= alu_result;
            wb_int_en <= 1'b0;
        end

        case (state)
            ST_IDLE: begin
                if (start) begin
                    // ---- Program configuration (spec §4.6.4) ----
                    for (i = 0; i < 4; i = i + 1) begin
                        a_lo[i] <= small_pos_float(entropy[2*i]);
                        a_hi[i] <= small_pos_float(entropy[2*i+1]);
                    end
                    for (i = 0; i < 8; i = i + 1) r[i]    <= 64'b0;
                    for (i = 0; i < 4; i = i + 1) f_lo[i] <= 64'b0;
                    for (i = 0; i < 4; i = i + 1) f_hi[i] <= 64'b0;
                    for (i = 0; i < 4; i = i + 1) e_lo[i] <= 64'b0;
                    for (i = 0; i < 4; i = i + 1) e_hi[i] <= 64'b0;

                    ma        <= entropy[8] & CACHE_LINE_ALIGN_MASK;
                    mx        <= entropy[10];
                    read_reg0 <= {2'b00, entropy[12][0]};
                    read_reg1 <= {2'b01, entropy[12][1]};
                    read_reg2 <= {2'b10, entropy[12][2]};
                    read_reg3 <= {2'b11, entropy[12][3]};
                    e_mask0   <= float_mask(entropy[14]);
                    e_mask1   <= float_mask(entropy[15]);

                    // spAddr0 = mx, spAddr1 = ma (masking commutes with the
                    // XOR applied at the start of every iteration)
                    sp_addr0  <= entropy[10][20:0] & SP_L3_MASK64;
                    // (CacheLineAlignMask and ScratchpadL3Mask64 agree on
                    //  bits [20:0], so a single mask is sufficient here)
                    sp_addr1  <= entropy[8][20:0] & SP_L3_MASK64;

                    // datasetOffset = (entropy[13] % (DatasetExtraItems+1)) * 64
                    mod_sh    <= entropy[13];
                    mod_rem   <= 21'd0;
                    mod_cnt   <= 7'd0;

                    fprc      <= 2'b00;
                    iter_cnt  <= 32'd0;
                    state     <= ST_MOD;
                end
            end

            // ---- datasetOffset modulo: 64-cycle restoring division ----
            ST_MOD: begin
                mod_rem <= mod_next;
                mod_sh  <= {mod_sh[62:0], 1'b0};
                if (mod_cnt == 7'd63) begin
                    ds_off_items <= {11'd0, mod_next};
                    cc           <= 8'd0;
                    for (i = 0; i < 8; i = i + 1) reg_usage[i] <= 8'hFF;
                    state        <= ST_COMPILE;
                end else begin
                    mod_cnt <= mod_cnt + 7'd1;
                end
            end

            ST_COMPILE: begin
                // One instruction per cycle: track integer register usage and
                // record CBRANCH targets (RandomX spec §5.5.10)
                if (c_op == OPC_CBRANCH) begin
                    branch_target[cc] <= reg_usage[c_dst];
                    for (i = 0; i < 8; i = i + 1) reg_usage[i] <= cc;
                end else if (c_op <= OPC_ISWAP_R) begin
                    reg_usage[c_dst] <= cc;
                    if (c_op == OPC_ISWAP_R)
                        reg_usage[c_src] <= cc;
                end
                if (cc == 8'd255) begin
                    cc    <= 8'd0;
                    state <= ST_LOOP;
                end else begin
                    cc <= cc + 8'd1;
                end
            end

            // ---- Iteration start: scratchpad pointer mixing ----
            ST_LOOP: begin
                sp_addr0 <= (sp_addr0 ^ sp_mix[20: 0]) & SP_L3_MASK64;
                sp_addr1 <= (sp_addr1 ^ sp_mix[52:32]) & SP_L3_MASK64;
                seq_i    <= 4'd0;
                state    <= ST_LD_R;
            end

            // ---- r[i] ^= load64(spAddr0 + 8*i) ----
            ST_LD_R: begin
                sp_rd_en    <= 1'b1;
                sp_rd_addr  <= sp0_addr;
                sp_rd_level <= 2'd2;
                state       <= ST_LD_R_W;
            end

            ST_LD_R_W: begin
                if (sp_rd_valid) begin
                    r[seq_i[2:0]] <= r[seq_i[2:0]] ^ sp_rd_data;
                    if (seq_i == 4'd7) begin
                        seq_i <= 4'd0;
                        state <= ST_LD_F;
                    end else begin
                        seq_i <= seq_i + 4'd1;
                        state <= ST_LD_R;
                    end
                end
            end

            // ---- f[i] / e[i] = cvt(load64(spAddr1 + 8*i)) ----
            ST_LD_F: begin
                sp_rd_en    <= 1'b1;
                sp_rd_addr  <= sp1_addr;
                sp_rd_level <= 2'd2;
                state       <= ST_LD_F_W;
            end

            ST_LD_F_W: begin
                if (sp_rd_valid) begin
                    if (seq_i[2] == 1'b0) begin
                        f_lo[seq_i[1:0]] <= cvt_i32(sp_rd_data[31: 0]);
                        f_hi[seq_i[1:0]] <= cvt_i32(sp_rd_data[63:32]);
                    end else begin
                        e_lo[seq_i[1:0]] <= mask_e(cvt_i32(sp_rd_data[31: 0]),
                                                   e_mask0);
                        e_hi[seq_i[1:0]] <= mask_e(cvt_i32(sp_rd_data[63:32]),
                                                   e_mask1);
                    end
                    if (seq_i == 4'd7) begin
                        seq_i <= 4'd0;
                        ic    <= 8'd0;
                        state <= ST_FETCH;
                    end else begin
                        seq_i <= seq_i + 4'd1;
                        state <= ST_LD_F;
                    end
                end
            end

            ST_FETCH: begin
                // Instruction available combinationally from prog_mem
                state <= ST_DECODE;
            end

            ST_DECODE: begin
                case (op)
                    // ---- Integer register-register instructions ----
                    OPC_IADD_RS,
                    OPC_ISUB_R,
                    OPC_IMUL_R,
                    OPC_IMULH_R,
                    OPC_ISMULH_R,
                    OPC_INEG_R,
                    OPC_IXOR_R,
                    OPC_IROR_R,
                    OPC_IROL_R: begin
                        alu_op    <= op[5:0];
                        alu_a     <= r_dst;
                        alu_b     <= r_src;
                        alu_shift <= mod_shift;
                        alu_imm   <= imm64_sext;
                        alu_en    <= 1'b1;
                        wb_dst    <= dst_r;
                        wb_int_en <= 1'b1;
                        state     <= ST_INT_WAIT;
                    end

                    // IMUL_RCP: dst *= reciprocal(imm32)
                    OPC_IMUL_RCP: begin
                        alu_op    <= op[5:0];
                        alu_a     <= r_dst;
                        alu_b     <= {32'b0, imm32};
                        alu_shift <= 2'd0;
                        alu_imm   <= imm64_sext;
                        alu_en    <= 1'b1;
                        wb_dst    <= dst_r;
                        wb_int_en <= 1'b1;
                        state     <= ST_INT_WAIT;
                    end

                    // ---- Integer memory instructions ----
                    OPC_IADD_M,
                    OPC_ISUB_M,
                    OPC_IMUL_M,
                    OPC_IMULH_M,
                    OPC_ISMULH_M,
                    OPC_IXOR_M: begin
                        sp_rd_en    <= 1'b1;
                        sp_rd_addr  <= mem_addr;
                        sp_rd_level <= mem_level;
                        mem_for_fp  <= 1'b0;
                        state       <= ST_MEM_RD;
                    end

                    // ---- ISWAP_R: exchange two integer registers ----
                    OPC_ISWAP_R: begin
                        r[dst_r] <= r_src;
                        r[src_r] <= r_dst;
                        state    <= ST_WB;
                    end

                    // ---- CBRANCH ----
                    OPC_CBRANCH: begin
                        alu_op      <= 6'd17;   // alu_int OP_CBRANCH
                        alu_a       <= r_dst;
                        alu_b       <= 64'b0;
                        alu_imm     <= imm64_sext;
                        alu_shift   <= 2'd0;
                        alu_en      <= 1'b1;
                        wb_dst      <= dst_r;
                        wb_int_en   <= 1'b1;
                        cb_target_q <= branch_target[ic];
                        state       <= ST_BR_WAIT;
                    end

                    // ---- ISTORE ----
                    OPC_ISTORE: begin
                        alu_op    <= 6'd18;     // alu_int OP_ISTORE
                        alu_a     <= r_dst;
                        alu_b     <= r_src;
                        alu_imm   <= imm64_sext;
                        alu_shift <= 2'd0;
                        alu_en    <= 1'b1;
                        state     <= ST_INT_WAIT;
                    end

                    // ---- FP register instructions ----
                    OPC_FADD_R, OPC_FSUB_R: begin
                        fp_opc  <= (op == OPC_FADD_R) ? FP_FADD : FP_FSUB;
                        fp_dst  <= dst_f;
                        fp_is_e <= 1'b0;
                        fp_b_lo <= a_lo[src_f];
                        fp_b_hi <= a_hi[src_f];
                        fp_half <= 1'b0;
                        state   <= ST_FP_ISSUE;
                    end

                    OPC_FSCAL_R: begin
                        fp_opc  <= FP_FSCAL;
                        fp_dst  <= dst_f;
                        fp_is_e <= 1'b0;
                        fp_b_lo <= 64'b0;
                        fp_b_hi <= 64'b0;
                        fp_half <= 1'b0;
                        state   <= ST_FP_ISSUE;
                    end

                    OPC_FMUL_R: begin
                        fp_opc  <= FP_FMUL;
                        fp_dst  <= dst_f;
                        fp_is_e <= 1'b1;
                        fp_b_lo <= a_lo[src_f];
                        fp_b_hi <= a_hi[src_f];
                        fp_half <= 1'b0;
                        state   <= ST_FP_ISSUE;
                    end

                    OPC_FSQRT_R: begin
                        fp_opc  <= FP_FSQRT;
                        fp_dst  <= dst_f;
                        fp_is_e <= 1'b1;
                        fp_b_lo <= 64'b0;
                        fp_b_hi <= 64'b0;
                        fp_half <= 1'b0;
                        state   <= ST_FP_ISSUE;
                    end

                    // ---- FP memory instructions ----
                    OPC_FADD_M, OPC_FSUB_M, OPC_FDIV_M: begin
                        sp_rd_en    <= 1'b1;
                        sp_rd_addr  <= mem_addr;
                        sp_rd_level <= mem_level;
                        mem_for_fp  <= 1'b1;
                        fp_dst      <= dst_f;
                        fp_is_e     <= (op == OPC_FDIV_M);
                        fp_opc      <= (op == OPC_FADD_M) ? FP_FADD :
                                       (op == OPC_FSUB_M) ? FP_FSUB : FP_FDIV;
                        state       <= ST_MEM_RD;
                    end

                    // ---- FSWAP_R: exchange the halves of an FP register ----
                    OPC_FSWAP_R: begin
                        if (dst_is_e) begin
                            e_lo[dst_f] <= e_hi[dst_f];
                            e_hi[dst_f] <= e_lo[dst_f];
                        end else begin
                            f_lo[dst_f] <= f_hi[dst_f];
                            f_hi[dst_f] <= f_lo[dst_f];
                        end
                        state <= ST_WB;
                    end

                    // ---- CFROUND: fprc = ror(r[src], imm32 % 64) & 3 ----
                    OPC_CFROUND: begin
                        fprc  <= cfr_ror[1:0];
                        state <= ST_WB;
                    end

                    default: begin
                        // NOP / unknown opcode
                        state <= ST_WB;
                    end
                endcase
            end

            // ---- Scratchpad operand load ----
            ST_MEM_RD: begin
                if (sp_rd_valid) begin
                    if (mem_for_fp) begin
                        if (fp_opc == FP_FDIV) begin
                            fp_b_lo <= mask_e(cvt_i32(sp_rd_data[31: 0]),
                                              e_mask0);
                            fp_b_hi <= mask_e(cvt_i32(sp_rd_data[63:32]),
                                              e_mask1);
                        end else begin
                            fp_b_lo <= cvt_i32(sp_rd_data[31: 0]);
                            fp_b_hi <= cvt_i32(sp_rd_data[63:32]);
                        end
                        fp_half <= 1'b0;
                        state   <= ST_FP_ISSUE;
                    end else begin
                        alu_op    <= op[5:0];
                        alu_a     <= r_dst;
                        alu_b     <= sp_rd_data;
                        alu_imm   <= imm64_sext;
                        alu_shift <= 2'd0;
                        alu_en    <= 1'b1;
                        wb_dst    <= dst_r;
                        wb_int_en <= 1'b1;
                        state     <= ST_INT_WAIT;
                    end
                end
            end

            ST_INT_WAIT: begin
                // Registered ALU result (writeback handled above)
                if (alu_valid || alu_mem_wr)
                    state <= ST_WB;
            end

            ST_BR_WAIT: begin
                if (alu_valid) begin
                    if (branch_taken) begin
                        // Jump back to target+1 (0xFF+1 wraps to 0)
                        ic    <= cb_target_q + 8'd1;
                        state <= ST_FETCH;
                    end else if (ic == 8'd255) begin
                        state <= ST_MX;
                    end else begin
                        ic    <= ic + 8'd1;
                        state <= ST_FETCH;
                    end
                end
            end

            // ---- FP execution: low half then high half ----
            ST_FP_ISSUE: begin
                fpu_op <= fp_opc;
                fpu_a  <= fp_a_cur;
                fpu_b  <= fp_half ? fp_b_hi : fp_b_lo;
                fpu_en <= 1'b1;
                state  <= ST_FP_WAIT;
            end

            ST_FP_WAIT: begin
                if (fpu_valid) begin
                    if (fp_is_e) begin
                        if (fp_half) e_hi[fp_dst] <= fpu_result;
                        else         e_lo[fp_dst] <= fpu_result;
                    end else begin
                        if (fp_half) f_hi[fp_dst] <= fpu_result;
                        else         f_lo[fp_dst] <= fpu_result;
                    end
                    if (!fp_half) begin
                        fp_half <= 1'b1;
                        state   <= ST_FP_ISSUE;
                    end else begin
                        state   <= ST_WB;
                    end
                end
            end

            ST_WB: begin
                if (ic == 8'd255) begin
                    state <= ST_MX;
                end else begin
                    ic    <= ic + 8'd1;
                    state <= ST_FETCH;
                end
            end

            // ---- End of program: mx update and dataset read ----
            ST_MX: begin
                mx    <= (mx ^ mx_mix) & CACHE_LINE_ALIGN_MASK;
                state <= ST_DS_REQ;
            end

            ST_DS_REQ: begin
                // Dataset item index = (datasetOffset + ma) / 64
                ds_req_valid <= 1'b1;
                ds_req_idx   <= ds_off_items + {7'd0, ma[30:6]};
                if (ds_req_valid && ds_req_ready) begin
                    ds_req_valid  <= 1'b0;
                    ds_resp_ready <= 1'b1;
                    state         <= ST_DS_WAIT;
                end
            end

            ST_DS_WAIT: begin
                if (ds_resp_valid) begin
                    for (i = 0; i < 8; i = i + 1)
                        r[i] <= r[i] ^ ds_resp_data[64*i +: 64];
                    ds_resp_ready <= 1'b0;
                    // swap(ma, mx)
                    ma            <= mx;
                    mx            <= ma;
                    seq_i         <= 4'd0;
                    state         <= ST_ST_R;
                end
            end

            // ---- store64(spAddr1 + 8*i, r[i]) ----
            ST_ST_R: begin
                sp_wr_en    <= 1'b1;
                sp_wr_addr  <= sp1_addr;
                sp_wr_level <= 2'd2;
                sp_wr_data  <= r[seq_i[2:0]];
                if (seq_i == 4'd7) begin
                    seq_i <= 4'd0;
                    state <= ST_ST_F;
                end else begin
                    seq_i <= seq_i + 4'd1;
                end
            end

            // ---- store128(spAddr0 + 16*i, f[i] ^ e[i]) ----
            ST_ST_F: begin
                sp_wr_en    <= 1'b1;
                sp_wr_addr  <= sp0_addr;
                sp_wr_level <= 2'd2;
                sp_wr_data  <= seq_i[0] ? (f_hi[seq_i[2:1]] ^ e_hi[seq_i[2:1]])
                                        : (f_lo[seq_i[2:1]] ^ e_lo[seq_i[2:1]]);
                if (seq_i == 4'd7) begin
                    seq_i    <= 4'd0;
                    sp_addr0 <= 21'd0;
                    sp_addr1 <= 21'd0;
                    if (iter_cnt == ITER_LAST) begin
                        fold_cnt <= 32'd0;
                        state    <= do_final ? ST_FIN_RD : ST_DONE;
                    end else begin
                        iter_cnt <= iter_cnt + 32'd1;
                        state    <= ST_LOOP;
                    end
                end else begin
                    seq_i <= seq_i + 4'd1;
                end
            end

            // ---- Final hash: stream every 64-byte scratchpad block
            //      through AesHash1R (spec §3.5 / getFinalResult) ----
            ST_FIN_RD: begin
                sp_rd_en    <= 1'b1;
                sp_rd_addr  <= {fold_cnt[17:0], 3'd0};
                sp_rd_level <= 2'd2;
                state       <= ST_FIN_W;
            end

            ST_FIN_W: begin
                if (sp_rd_valid) begin
                    acc[fold_cnt[2:0]] <= sp_rd_data;
                    if (fold_cnt[2:0] == 3'd7) begin
                        // 8 words collected: absorb one 64-byte block. The
                        // first block also (re)loads the fixed initial state.
                        aes_start     <= (fold_cnt[31:3] == 29'd0);
                        aes_blk_valid <= 1'b1;
                        aes_blk_last  <= (fold_cnt == FOLD_LAST);
                        aes_data_in   <= {sp_rd_data, acc[6], acc[5], acc[4],
                                          acc[3], acc[2], acc[1], acc[0]};
                    end
                    if (fold_cnt == FOLD_LAST) begin
                        state <= ST_FIN_HASH;
                    end else begin
                        fold_cnt <= fold_cnt + 32'd1;
                        state    <= ST_FIN_RD;
                    end
                end
            end

            ST_FIN_HASH: begin
                if (aes_hash_valid) begin
                    hash_out <= aes_hash_out;
                    // getFinalResult() writes the digest into the a registers
                    a_lo[0] <= aes_hash_out[ 63:  0];
                    a_hi[0] <= aes_hash_out[127: 64];
                    a_lo[1] <= aes_hash_out[191:128];
                    a_hi[1] <= aes_hash_out[255:192];
                    a_lo[2] <= aes_hash_out[319:256];
                    a_hi[2] <= aes_hash_out[383:320];
                    a_lo[3] <= aes_hash_out[447:384];
                    a_hi[3] <= aes_hash_out[511:448];
                    state   <= ST_DONE;
                end
            end

            ST_DONE: begin
                done  <= 1'b1;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
