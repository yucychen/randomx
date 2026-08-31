// =============================================================================
// tb_randomx_vm.v — Self-checking testbench for randomx_vm
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Loads a directed 256-instruction program that exercises one instruction of
// (almost) every RandomX instruction class and checks the architectural state
// of the VM after the program has finished:
//
//   idx  instruction                       expected effect
//   ---  --------------------------------  ------------------------------
//    0   IADD_RS r0, r0, imm=5             r0 = 5
//    1   IADD_RS r1, r0<<1, imm=3          r1 = 13
//    2   ISUB_R  r1, r0                    r1 = 8
//    3   IXOR_R  r2, r1                    r2 = 8
//    4   IROR_R  r2, r0                    r2 = ror(8,5) = 2^62
//    5   ISWAP_R r3, r2                    r3 = 2^62, r2 = 0
//    6   IADD_RS r4, r2, imm=7             r4 = 7
//    7   IMUL_R  r4, r0                    r4 = 35
//    8   IADD_RS r5, r2, imm=9             r5 = 9
//    9   INEG_R  r5                        r5 = -9
//   10   IADD_RS r7, r2, imm=0x100         r7 = 0x100
//   11   ISTORE  L1[r7+0] = r0             scratchpad[0x100] = 5
//   12   IADD_M  r6 += L1[r7+0]            r6 = 5
//   13   IADD_RS r6, r2, imm=0xFEFB        r6 = 0xFF00
//   14   CBRANCH r6 (cond=0)               taken once → r6 = 0x10100
//   15   FADD_R  f0, a0                    f0 = a0
//   16   FSCAL_R f0                        f0 ^= 0x80F0000000000000
//   17   FMUL_R  e0, a0                    e0 = 1.0 * a0
//   18   FMUL_R  e1, a0                    e1 = 1.0 * a0
//   19   FSQRT_R e1                        e1 = sqrt(e1)
//   20   FSWAP_R e1                        e1 halves exchanged
//   21   CFROUND r0, imm=0                 fprc = r0 & 3 = 1
//   22.. NOP
//
// The program entropy is chosen so that a0 = (4.0, 1.0), the e-register mask
// yields 1.0 and all scratchpad pointers start at 0.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module tb_randomx_vm;

// ---------------------------------------------------------------------------
// Clock / reset
// ---------------------------------------------------------------------------
reg clk;
reg rst_n;

initial clk = 1'b0;
always #1.667 clk = ~clk;

// ---------------------------------------------------------------------------
// DUT signals
// ---------------------------------------------------------------------------
reg          start;
reg          prog_wr_en;
reg  [7:0]   prog_wr_addr;
reg  [63:0]  prog_wr_data;
reg          cfg_wr_en;
reg  [3:0]   cfg_wr_addr;
reg  [63:0]  cfg_wr_data;

wire         sp_rd_en,  sp_wr_en;
wire [20:0]  sp_rd_addr, sp_wr_addr;
wire [1:0]   sp_rd_level, sp_wr_level;
wire [63:0]  sp_rd_data, sp_wr_data;
wire         sp_rd_valid;

wire         ds_req_valid;
wire [31:0]  ds_req_idx;
reg          ds_req_ready;
reg  [511:0] ds_resp_data;
reg          ds_resp_valid;
wire         ds_resp_ready;

wire         aes_start;
wire         aes_blk_valid;
wire         aes_blk_last;
wire [511:0] aes_data_in;
wire [2047:0] regfile_out;
wire [511:0] aes_hash_out;
wire         aes_hash_valid;

wire [511:0] hash_out;
wire         done;

integer errors;
integer k;
integer timeout;

// ---------------------------------------------------------------------------
// DUT
// ---------------------------------------------------------------------------
randomx_vm #(
    .ITERATIONS (1),
    .SP_WORDS   (64)
) u_dut (
    .clk           (clk),
    .rst_n         (rst_n),
    .start         (start),
    .prog_wr_en    (prog_wr_en),
    .prog_wr_addr  (prog_wr_addr),
    .prog_wr_data  (prog_wr_data),
    .cfg_wr_en     (cfg_wr_en),
    .cfg_wr_addr   (cfg_wr_addr),
    .cfg_wr_data   (cfg_wr_data),
    .sp_rd_en      (sp_rd_en),
    .sp_rd_addr    (sp_rd_addr),
    .sp_rd_level   (sp_rd_level),
    .sp_rd_data    (sp_rd_data),
    .sp_rd_valid   (sp_rd_valid),
    .sp_wr_en      (sp_wr_en),
    .sp_wr_addr    (sp_wr_addr),
    .sp_wr_level   (sp_wr_level),
    .sp_wr_data    (sp_wr_data),
    .ds_req_valid  (ds_req_valid),
    .ds_req_idx    (ds_req_idx),
    .ds_req_ready  (ds_req_ready),
    .ds_resp_data  (ds_resp_data),
    .ds_resp_valid (ds_resp_valid),
    .ds_resp_ready (ds_resp_ready),
    .do_final      (1'b1),
    .aes_start     (aes_start),
    .aes_blk_valid (aes_blk_valid),
    .aes_blk_last  (aes_blk_last),
    .aes_data_in   (aes_data_in),
    .regfile_out   (regfile_out),
    .aes_hash_out  (aes_hash_out),
    .aes_hash_valid(aes_hash_valid),
    .hash_out      (hash_out),
    .done          (done)
);

scratchpad_mem u_sp (
    .clk      (clk),
    .rst_n    (rst_n),
    .wr_en    (sp_wr_en),
    .wr_addr  (sp_wr_addr),
    .wr_data  (sp_wr_data),
    .wr_level (sp_wr_level),
    .rd_en    (sp_rd_en),
    .rd_addr  (sp_rd_addr),
    .rd_level (sp_rd_level),
    .rd_data  (sp_rd_data),
    .rd_valid (sp_rd_valid)
);

aes_hash1r u_aes (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (aes_start),
    .blk_valid(aes_blk_valid),
    .blk_last (aes_blk_last),
    .data_in  (aes_data_in),
    .hash_out (aes_hash_out),
    .busy     (),
    .valid    (aes_hash_valid)
);

// ---------------------------------------------------------------------------
// Dataset model: always ready, returns an all-zero 64-byte item
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ds_resp_valid <= 1'b0;
    end else begin
        if (ds_req_valid && ds_req_ready)
            ds_resp_valid <= 1'b1;
        else if (ds_resp_valid && ds_resp_ready)
            ds_resp_valid <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// Helper tasks
// ---------------------------------------------------------------------------
task wr_instr;
    input [7:0]  idx;
    input [7:0]  opcode;
    input [3:0]  dst;
    input [3:0]  src;
    input [15:0] modb;
    input [31:0] imm;
    begin
        @(posedge clk);
        #0.1;
        prog_wr_en   = 1'b1;
        prog_wr_addr = idx;
        prog_wr_data = {opcode, dst, src, modb, imm};
        @(posedge clk);
        #0.1;
        prog_wr_en   = 1'b0;
    end
endtask

task wr_cfg;
    input [3:0]  idx;
    input [63:0] data;
    begin
        @(posedge clk);
        #0.1;
        cfg_wr_en   = 1'b1;
        cfg_wr_addr = idx;
        cfg_wr_data = data;
        @(posedge clk);
        #0.1;
        cfg_wr_en   = 1'b0;
    end
endtask

task check64;
    input [255:0] name;
    input [63:0]  got;
    input [63:0]  exp;
    begin
        if (got !== exp) begin
            $display("[TB] FAIL: %0s = 0x%016h, expected 0x%016h",
                     name, got, exp);
            errors = errors + 1;
        end else begin
            $display("[TB] OK  : %0s = 0x%016h", name, got);
        end
    end
endtask

// ---------------------------------------------------------------------------
// Test sequence
// ---------------------------------------------------------------------------
// The final AesHash1R pass overwrites the a registers, so latch their
// entropy-derived initial values while the program is still running.
reg [63:0] a0_lo_init, a0_hi_init;
reg        a0_latched;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        a0_latched <= 1'b0;
        a0_lo_init <= 64'b0;
        a0_hi_init <= 64'b0;
    end else if (!a0_latched && u_dut.a_lo[0] !== 64'b0) begin
        a0_lo_init <= u_dut.a_lo[0];
        a0_hi_init <= u_dut.a_hi[0];
        a0_latched <= 1'b1;
    end
end

initial begin
    $dumpfile("tb_randomx_vm.vcd");
    $dumpvars(0, tb_randomx_vm);
end

initial begin
    errors        = 0;
    rst_n         = 1'b0;
    start         = 1'b0;
    prog_wr_en    = 1'b0;
    prog_wr_addr  = 8'b0;
    prog_wr_data  = 64'b0;
    cfg_wr_en     = 1'b0;
    cfg_wr_addr   = 4'b0;
    cfg_wr_data   = 64'b0;
    ds_req_ready  = 1'b1;
    ds_resp_data  = 512'b0;

    // Scratchpad starts as X in simulation — clear it
    for (k = 0; k < 4096; k = k + 1)
        u_sp.scratchpad[k] = 64'b0;

    repeat (10) @(posedge clk);
    #0.1;
    rst_n = 1'b1;
    repeat (5) @(posedge clk);

    // ---- Program configuration entropy ----
    // entropy[0] → a0.lo = 4.0, entropy[1] → a0.hi = 1.0
    // entropy[8] → ma = 0x200 (spAddr1), entropy[10] → mx = 0 (spAddr0)
    for (k = 0; k < 16; k = k + 1)
        wr_cfg(k[3:0], 64'd0);
    wr_cfg(4'd0, 64'h1000000000000000);
    wr_cfg(4'd8, 64'h0000000000000200);

    // ---- Program ----
    wr_instr(8'd0,  8'd0,  4'd0, 4'd0, 16'h0000, 32'h00000005); // IADD_RS
    wr_instr(8'd1,  8'd0,  4'd1, 4'd0, 16'h0004, 32'h00000003); // IADD_RS <<1
    wr_instr(8'd2,  8'd2,  4'd1, 4'd0, 16'h0000, 32'h00000000); // ISUB_R
    wr_instr(8'd3,  8'd12, 4'd2, 4'd1, 16'h0000, 32'h00000000); // IXOR_R
    wr_instr(8'd4,  8'd14, 4'd2, 4'd0, 16'h0000, 32'h00000000); // IROR_R
    wr_instr(8'd5,  8'd16, 4'd3, 4'd2, 16'h0000, 32'h00000000); // ISWAP_R
    wr_instr(8'd6,  8'd0,  4'd4, 4'd2, 16'h0000, 32'h00000007); // IADD_RS
    wr_instr(8'd7,  8'd4,  4'd4, 4'd0, 16'h0000, 32'h00000000); // IMUL_R
    wr_instr(8'd8,  8'd0,  4'd5, 4'd2, 16'h0000, 32'h00000009); // IADD_RS
    wr_instr(8'd9,  8'd11, 4'd5, 4'd0, 16'h0000, 32'h00000000); // INEG_R
    wr_instr(8'd10, 8'd0,  4'd7, 4'd2, 16'h0000, 32'h00000100); // IADD_RS
    wr_instr(8'd11, 8'd18, 4'd7, 4'd0, 16'h0001, 32'h00000000); // ISTORE L1
    wr_instr(8'd12, 8'd1,  4'd6, 4'd7, 16'h0001, 32'h00000000); // IADD_M  L1
    wr_instr(8'd13, 8'd0,  4'd6, 4'd2, 16'h0000, 32'h0000FEFB); // IADD_RS
    wr_instr(8'd14, 8'd17, 4'd6, 4'd0, 16'h0000, 32'h00000000); // CBRANCH
    wr_instr(8'd15, 8'd19, 4'd0, 4'd0, 16'h0000, 32'h00000000); // FADD_R
    wr_instr(8'd16, 8'd23, 4'd0, 4'd0, 16'h0000, 32'h00000000); // FSCAL_R
    wr_instr(8'd17, 8'd24, 4'd0, 4'd0, 16'h0000, 32'h00000000); // FMUL_R
    wr_instr(8'd18, 8'd24, 4'd1, 4'd0, 16'h0000, 32'h00000000); // FMUL_R
    wr_instr(8'd19, 8'd26, 4'd1, 4'd0, 16'h0000, 32'h00000000); // FSQRT_R
    wr_instr(8'd20, 8'd27, 4'd5, 4'd0, 16'h0000, 32'h00000000); // FSWAP_R e1
    wr_instr(8'd21, 8'd28, 4'd0, 4'd0, 16'h0000, 32'h00000000); // CFROUND
    for (k = 22; k < 256; k = k + 1)
        wr_instr(k[7:0], 8'd29, 4'd0, 4'd0, 16'h0000, 32'h00000000); // NOP

    // ---- Run ----
    @(posedge clk);
    #0.1;
    start = 1'b1;
    @(posedge clk);
    #0.1;
    start = 1'b0;

    timeout = 0;
    while (done !== 1'b1 && timeout < 200000) begin
        @(posedge clk);
        timeout = timeout + 1;
    end

    if (done !== 1'b1) begin
        $display("[TB] FAIL: VM did not assert done (timeout)");
        errors = errors + 1;
    end else begin
        $display("[TB] VM done after %0d cycles, hash[63:0] = 0x%016h",
                 timeout, hash_out[63:0]);
    end

    // ---- Architectural state checks ----
    check64("r0",    u_dut.r[0],    64'h0000000000000005);
    check64("r1",    u_dut.r[1],    64'h0000000000000008);
    check64("r2",    u_dut.r[2],    64'h0000000000000000);
    check64("r3",    u_dut.r[3],    64'h4000000000000000);
    check64("r4",    u_dut.r[4],    64'h0000000000000023);
    check64("r5",    u_dut.r[5],    64'hFFFFFFFFFFFFFFF7);
    check64("r6",    u_dut.r[6],    64'h0000000000010100);
    check64("r7",    u_dut.r[7],    64'h0000000000000100);
    // a0 = (4.0, 1.0) from the program entropy
    check64("a0.lo init", a0_lo_init, 64'h4010000000000000);
    check64("a0.hi init", a0_hi_init, 64'h3FF0000000000000);
    // getFinalResult() overwrites the a registers with the 64-byte AesHash1R
    // digest of the scratchpad, so a0..a3 must now mirror `hash_out`
    // (their entropy-derived initial values are checked above).
    check64("a0.lo = digest", u_dut.a_lo[0], hash_out[ 63:  0]);
    check64("a0.hi = digest", u_dut.a_hi[0], hash_out[127: 64]);
    check64("a3.hi = digest", u_dut.a_hi[3], hash_out[511:448]);
    // regfile_out must expose the reference RegisterFile byte layout
    check64("regfile r0",   regfile_out[  63:   0], u_dut.r[0]);
    check64("regfile r7",   regfile_out[ 511: 448], u_dut.r[7]);
    check64("regfile f0lo", regfile_out[ 575: 512], u_dut.f_lo[0]);
    check64("regfile e0lo", regfile_out[1087:1024], u_dut.e_lo[0]);
    check64("regfile a0lo", regfile_out[1599:1536], u_dut.a_lo[0]);
    check64("regfile a3hi", regfile_out[2047:1984], u_dut.a_hi[3]);
    // f0 = a0 with FSCAL applied
    check64("f0.lo", u_dut.f_lo[0], 64'hC0E0000000000000);
    check64("f0.hi", u_dut.f_hi[0], 64'hBF00000000000000);
    // e0 = 1.0 * a0
    check64("e0.lo", u_dut.e_lo[0], 64'h4010000000000000);
    check64("e0.hi", u_dut.e_hi[0], 64'h3FF0000000000000);
    // e1 = sqrt(1.0 * a0) then halves swapped → (1.0, 2.0)
    check64("e1.lo", u_dut.e_lo[1], 64'h3FF0000000000000);
    check64("e1.hi", u_dut.e_hi[1], 64'h4000000000000000);
    // e2 untouched by the program: the e-mask of an all-zero word is 1.0
    check64("e2.lo", u_dut.e_lo[2], 64'h3FF0000000000000);
    // CFROUND: fprc = r0 & 3
    check64("fprc",  {62'b0, u_dut.fprc}, 64'h0000000000000001);
    // ISTORE / IADD_M round trip: scratchpad word 0x100 holds r0
    check64("sp[0x100]", u_sp.scratchpad[12'h020], 64'h0000000000000005);
    // r registers written back to the scratchpad at spAddr1 = 0x200
    check64("sp[0x200]", u_sp.scratchpad[12'h040], 64'h0000000000000005);
    check64("sp[0x208]", u_sp.scratchpad[12'h041], 64'h0000000000000008);
    // f[0] ^ e[0] written back to the scratchpad at spAddr0 = 0
    check64("sp[0x00]",  u_sp.scratchpad[12'h000],
            64'hC0E0000000000000 ^ 64'h4010000000000000);

    if (errors == 0)
        $display("[TB] tb_randomx_vm: ALL CHECKS PASSED");
    else
        $display("[TB] tb_randomx_vm: %0d FAIL(s)", errors);

    #50;
    $finish;
end

// Safety watchdog
initial begin
    #5000000;
    $display("[TB] FAIL: watchdog timeout");
    $finish;
end

endmodule
