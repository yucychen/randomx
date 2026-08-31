// =============================================================================
// tb_prog_gen.v — Self-checking testbench for prog_gen (AesGenerator4R program)
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// A RandomX program is 2176 bytes of AesGenerator4R output (spec §4.3):
//   bytes    0 .. 127  → 16 × 64-bit program configuration entropy words
//   bytes  128 .. 2175 → 256 × 64-bit instruction words
//
// The testbench builds an independent golden stream by driving its own
// aes_gen4r instance for 34 blocks from the same seed, then checks that
// prog_gen:
//   * writes exactly 16 cfg words at addresses 0..15 in order,
//   * writes exactly 256 instruction words at addresses 0..255 in order,
//   * emits the AesGenerator4R byte stream in order (cfg first, then instrs),
//   * exports the post-program generator state on `state_out` so the caller
//     can chain the next program of the hash,
//   * asserts busy for the whole run and pulses done exactly once.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module tb_prog_gen;

reg          clk = 1'b0;
reg          rst_n = 1'b0;
reg          start;
reg  [511:0] seed_in;

wire         prog_wr_en;
wire [7:0]   prog_wr_addr;
wire [63:0]  prog_wr_data;
wire         cfg_wr_en;
wire [3:0]   cfg_wr_addr;
wire [63:0]  cfg_wr_data;
wire [511:0] state_out;
wire         busy;
wire         done;

integer      errors = 0;

always #5 clk = ~clk;

prog_gen u_dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (start),
    .seed_in      (seed_in),
    .prog_wr_en   (prog_wr_en),
    .prog_wr_addr (prog_wr_addr),
    .prog_wr_data (prog_wr_data),
    .cfg_wr_en    (cfg_wr_en),
    .cfg_wr_addr  (cfg_wr_addr),
    .cfg_wr_data  (cfg_wr_data),
    .state_out    (state_out),
    .busy         (busy),
    .done         (done)
);

// ---------------------------------------------------------------------------
// Golden model: an independent aes_gen4r driven for 34 blocks
// ---------------------------------------------------------------------------
localparam TOTAL_BLOCKS = 34;

reg          g_start;
reg  [511:0] g_state;
wire [511:0] g_out;
wire         g_valid;

aes_gen4r u_gold (
    .clk       (clk),
    .rst_n     (rst_n),
    .start     (g_start),
    .state_in  (g_state),
    .state_out (g_out),
    .valid     (g_valid)
);

reg [63:0]  gold_word [0:TOTAL_BLOCKS*8-1];
reg [511:0] gold_state;

// ---------------------------------------------------------------------------
// Capture of the DUT write streams
// ---------------------------------------------------------------------------
reg [63:0]  cap_cfg   [0:15];
reg [63:0]  cap_instr [0:255];
integer     n_cfg   = 0;
integer     n_instr = 0;
integer     busy_low_during_run = 0;
integer     done_pulses = 0;

always @(posedge clk) begin
    if (rst_n && cfg_wr_en) begin
        if (cfg_wr_addr !== n_cfg[3:0]) begin
            $display("[TB] FAIL: cfg write #%0d at addr %0d (expected %0d)",
                     n_cfg, cfg_wr_addr, n_cfg);
            errors = errors + 1;
        end
        if (n_cfg < 16) cap_cfg[n_cfg] = cfg_wr_data;
        n_cfg = n_cfg + 1;
    end
    if (rst_n && prog_wr_en) begin
        if (prog_wr_addr !== n_instr[7:0]) begin
            $display("[TB] FAIL: instr write #%0d at addr %0d (expected %0d)",
                     n_instr, prog_wr_addr, n_instr);
            errors = errors + 1;
        end
        if (n_instr < 256) cap_instr[n_instr] = prog_wr_data;
        n_instr = n_instr + 1;
    end
    if (rst_n && done) done_pulses = done_pulses + 1;
end

// ---------------------------------------------------------------------------
integer i, b, w, timeout;

initial begin
    $dumpfile("tb_prog_gen.vcd");
    $dumpvars(0, tb_prog_gen);

    start   = 1'b0;
    g_start = 1'b0;
    g_state = 512'b0;
    seed_in = {64'h0F0E0D0C0B0A0908, 64'h0706050403020100,
               64'h1F1E1D1C1B1A1918, 64'h1716151413121110,
               64'h2F2E2D2C2B2A2928, 64'h2726252423222120,
               64'h3F3E3D3C3B3A3938, 64'h3736353433323130};

    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // ---- Build the golden stream ----
    g_state = seed_in;
    for (b = 0; b < TOTAL_BLOCKS; b = b + 1) begin
        @(negedge clk);
        g_start = 1'b1;
        @(negedge clk);
        g_start = 1'b0;
        timeout = 0;
        while (g_valid !== 1'b1 && timeout < 100) begin
            @(negedge clk);
            timeout = timeout + 1;
        end
        if (g_valid !== 1'b1) begin
            $display("[TB] FAIL: golden aes_gen4r block %0d timed out", b);
            errors = errors + 1;
        end
        for (w = 0; w < 8; w = w + 1)
            gold_word[b*8 + w] = g_out[w*64 +: 64];
        g_state = g_out;
    end
    gold_state = g_state;

    // ---- Run the DUT ----
    @(negedge clk);
    start = 1'b1;
    @(negedge clk);
    start = 1'b0;

    timeout = 0;
    while (done !== 1'b1 && timeout < 20000) begin
        @(posedge clk);
        // busy drops in the same cycle as the done pulse
        if (busy !== 1'b1 && done !== 1'b1 && timeout > 2) busy_low_during_run = 1;
        timeout = timeout + 1;
    end

    if (done !== 1'b1) begin
        $display("[TB] FAIL: prog_gen did not assert done (timeout)");
        errors = errors + 1;
    end else begin
        $display("[TB] prog_gen done after %0d cycles", timeout);
    end

    @(posedge clk);
    @(posedge clk);

    // ---- Checks ----
    if (n_cfg !== 16) begin
        $display("[TB] FAIL: %0d cfg writes, expected 16", n_cfg);
        errors = errors + 1;
    end else $display("[TB] PASS: 16 cfg entropy words written");

    if (n_instr !== 256) begin
        $display("[TB] FAIL: %0d instruction writes, expected 256", n_instr);
        errors = errors + 1;
    end else $display("[TB] PASS: 256 instruction words written");

    if (busy_low_during_run !== 0) begin
        $display("[TB] FAIL: busy deasserted while the program was generating");
        errors = errors + 1;
    end else $display("[TB] PASS: busy held for the whole run");

    if (done_pulses !== 1) begin
        $display("[TB] FAIL: done pulsed %0d times, expected 1", done_pulses);
        errors = errors + 1;
    end else $display("[TB] PASS: done pulsed exactly once");

    // cfg words are the first 16 words of the AesGenerator4R stream
    for (i = 0; i < 16; i = i + 1) begin
        if (cap_cfg[i] !== gold_word[i]) begin
            $display("[TB] FAIL: cfg[%0d] = 0x%016h, expected 0x%016h",
                     i, cap_cfg[i], gold_word[i]);
            errors = errors + 1;
        end
    end

    // instruction words follow immediately (word offset 16)
    for (i = 0; i < 256; i = i + 1) begin
        if (cap_instr[i] !== gold_word[16 + i]) begin
            $display("[TB] FAIL: instr[%0d] = 0x%016h, expected 0x%016h",
                     i, cap_instr[i], gold_word[16 + i]);
            errors = errors + 1;
        end
    end

    if (state_out !== gold_state) begin
        $display("[TB] FAIL: state_out mismatch");
        $display("        got 0x%0h", state_out);
        $display("        exp 0x%0h", gold_state);
        errors = errors + 1;
    end else $display("[TB] PASS: state_out matches the chained generator state");

    if (errors == 0)
        $display("[TB] tb_prog_gen: ALL TESTS PASSED");
    else
        $display("[TB] tb_prog_gen: %0d FAIL(s)", errors);
    $finish;
end

initial begin
    #2000000;
    $display("[TB] FAIL: tb_prog_gen timeout");
    $finish;
end

endmodule
