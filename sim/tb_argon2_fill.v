// =============================================================================
// tb_argon2_fill.v — Argon2d cache fill testbench
//
// Verifies argon2_fill (together with blake2b_core and a behavioural 1 KiB
// block cache memory) against golden vectors produced by the Argon2 reference
// implementation, using the RandomX flavour of Argon2d
// (type = Argon2d, version = 0x13, lanes = 1, salt = "RandomX\x03", no tag)
// with the memory cost reduced so the simulation stays short:
//
//   case 1: m = 8 blocks, t = 3 passes, 43-byte key  (exercises multi-pass
//           XOR mode and a key that is not a multiple of the block size)
//   case 2: m = 32 blocks, t = 1 pass,  64-byte key  (exercises all four
//           slices with more than two blocks per segment)
//
// The expected memory images (one 1 KiB block per line, most significant byte
// first) live in sim/argon2_expected.hex and sim/argon2_expected_m32.hex.
//
// Run:
//   iverilog -g2001 -DSIMULATION -o tb_argon2_fill.vvp \
//       rtl/blake2b_core.v rtl/argon2_fill.v sim/tb_argon2_fill.v
//   vvp tb_argon2_fill.vvp
// =============================================================================

`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Test harness: argon2_fill + shared Blake2b core + behavioural cache memory
// ---------------------------------------------------------------------------
module argon2_harness #(
    parameter ARGON_M    = 8,
    parameter ARGON_T    = 3,
    parameter KEY_BYTES  = 64,
    // Blocks actually filled = m_cost after the reference normalisation
    parameter MEM_BLOCKS = 8
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [KEY_BYTES*8-1:0] key,
    input  wire [15:0] key_len,
    output wire        fill_done
);

wire          cache_wr_en;
wire [31:0]   cache_wr_addr;
wire [8191:0] cache_wr_data;
wire          cache_rd_en;
wire [31:0]   cache_rd_addr;
reg  [8191:0] cache_rd_data;
reg           cache_rd_valid;

wire          b2b_start, b2b_init, b2b_last;
wire [1023:0] b2b_msg;
wire [127:0]  b2b_byte_cnt;
wire [511:0]  b2b_h_in;
wire [511:0]  b2b_h_out;
wire          b2b_busy, b2b_done;

// Cache memory: one cycle read latency, always ready for writes
reg [8191:0] mem [0:MEM_BLOCKS-1];

always @(posedge clk) begin
    if (!rst_n) begin
        cache_rd_valid <= 1'b0;
    end else begin
        cache_rd_valid <= 1'b0;
        if (cache_rd_en && cache_rd_addr < MEM_BLOCKS) begin
            cache_rd_data  <= mem[cache_rd_addr];
            cache_rd_valid <= 1'b1;
        end
        if (cache_wr_en && cache_wr_addr < MEM_BLOCKS)
            mem[cache_wr_addr] <= cache_wr_data;
    end
end

blake2b_core u_blake2b (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (b2b_start),
    .init       (b2b_init),
    .last_block (b2b_last),
    .msg_block  (b2b_msg),
    .byte_count (b2b_byte_cnt),
    .h_in       (b2b_h_in),
    .h_out      (b2b_h_out),
    .busy       (b2b_busy),
    .done       (b2b_done)
);

argon2_fill #(
    .ARGON_M   (ARGON_M),
    .ARGON_T   (ARGON_T),
    .KEY_BYTES (KEY_BYTES)
) dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .start          (start),
    .key            (key),
    .key_len        (key_len),
    .cache_wr_en    (cache_wr_en),
    .cache_wr_addr  (cache_wr_addr),
    .cache_wr_data  (cache_wr_data),
    .cache_wr_rdy   (1'b1),
    .cache_rd_en    (cache_rd_en),
    .cache_rd_addr  (cache_rd_addr),
    .cache_rd_data  (cache_rd_data),
    .cache_rd_valid (cache_rd_valid),
    .done           (fill_done),
    .b2b_start      (b2b_start),
    .b2b_init       (b2b_init),
    .b2b_msg        (b2b_msg),
    .b2b_byte_cnt   (b2b_byte_cnt),
    .b2b_h_in       (b2b_h_in),
    .b2b_last       (b2b_last),
    .b2b_h_out      (b2b_h_out),
    .b2b_busy       (b2b_busy),
    .b2b_done       (b2b_done)
);

endmodule


module tb_argon2_fill;

localparam M1 = 8;    // case 1: memory blocks
localparam T1 = 3;    // case 1: passes
localparam M2 = 32;   // case 2: memory blocks
localparam T2 = 1;    // case 2: passes
// case 3: m_cost = 6 (normalised to 8 by the reference), t = 5, 100-byte key
localparam M3  = 6;
localparam T3  = 5;
localparam MB3 = 8;   // blocks actually filled after normalisation
localparam KB3 = 100; // key bytes

reg clk;
reg rst_n;
reg start1, start2, start3;
reg [511:0] key1, key2;
reg [KB3*8-1:0] key3;

wire done1, done2, done3;

integer errors;
integer cycles;
integer i;

reg [8191:0] expected1 [0:M1-1];
reg [8191:0] expected2 [0:M2-1];
reg [8191:0] expected3 [0:MB3-1];

argon2_harness #(.ARGON_M(M1), .ARGON_T(T1), .MEM_BLOCKS(M1)) h1 (
    .clk (clk), .rst_n (rst_n), .start (start1),
    .key (key1), .key_len (16'd43), .fill_done (done1)
);

argon2_harness #(.ARGON_M(M2), .ARGON_T(T2), .MEM_BLOCKS(M2)) h2 (
    .clk (clk), .rst_n (rst_n), .start (start2),
    .key (key2), .key_len (16'd64), .fill_done (done2)
);

argon2_harness #(.ARGON_M(M3), .ARGON_T(T3), .KEY_BYTES(KB3),
                 .MEM_BLOCKS(MB3)) h3 (
    .clk (clk), .rst_n (rst_n), .start (start3),
    .key (key3), .key_len (KB3[15:0]), .fill_done (done3)
);

always #5 clk = ~clk;

function sel_done;
    input [1:0] sel;
    begin
        case (sel)
            2'd0:    sel_done = done1;
            2'd1:    sel_done = done2;
            default: sel_done = done3;
        endcase
    end
endfunction

// Wait for `flag` to be asserted, counting cycles
task wait_done;
    input [79:0] name;
    input [1:0]  flag_sel;
    begin
        cycles = 0;
        while (!sel_done(flag_sel) && cycles < 4000000) begin
            @(posedge clk);
            cycles = cycles + 1;
        end
        if (!sel_done(flag_sel)) begin
            $display("FAIL: %0s did not finish (%0d cycles)", name, cycles);
            errors = errors + 1;
        end else begin
            $display("INFO: %0s completed in %0d cycles", name, cycles);
        end
    end
endtask

initial begin
    clk    = 1'b0;
    rst_n  = 1'b0;
    start1 = 1'b0;
    start2 = 1'b0;
    start3 = 1'b0;
    errors = 0;

    // key bytes 00 01 02 ... (byte 0 is the least significant byte)
    key1 = 512'b0;
    key2 = 512'b0;
    for (i = 0; i < 43; i = i + 1) key1[i*8 +: 8] = i[7:0];
    for (i = 0; i < 64; i = i + 1) key2[i*8 +: 8] = i[7:0];
    key3 = {KB3*8{1'b0}};
    for (i = 0; i < KB3; i = i + 1) key3[i*8 +: 8] = (i * 7 + 3) % 256;

    for (i = 0; i < M1; i = i + 1) h1.mem[i] = 8192'b0;
    for (i = 0; i < M2; i = i + 1) h2.mem[i] = 8192'b0;
    for (i = 0; i < MB3; i = i + 1) h3.mem[i] = 8192'b0;

    $readmemh("sim/argon2_expected.hex",     expected1);
    $readmemh("sim/argon2_expected_m32.hex", expected2);
    $readmemh("sim/argon2_expected_k100.hex", expected3);

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // --- Case 1: m = 8, t = 3, 43-byte key ---------------------------------
    @(posedge clk); #1; start1 = 1'b1;
    @(posedge clk); #1; start1 = 1'b0;
    wait_done("m=8 t=3", 2'd0);

    for (i = 0; i < M1; i = i + 1) begin
        if (h1.mem[i] !== expected1[i]) begin
            $display("FAIL: case 1 block %0d mismatch", i);
            $display("      got      [63:0] = %016h", h1.mem[i][63:0]);
            $display("      expected [63:0] = %016h", expected1[i][63:0]);
            errors = errors + 1;
        end
    end
    if (errors == 0)
        $display("PASS: case 1 (m=8, t=3, 43-byte key) matches Argon2d reference");

    // --- Case 2: m = 32, t = 1, 64-byte key --------------------------------
    @(posedge clk); #1; start2 = 1'b1;
    @(posedge clk); #1; start2 = 1'b0;
    wait_done("m=32 t=1", 2'd1);

    for (i = 0; i < M2; i = i + 1) begin
        if (h2.mem[i] !== expected2[i]) begin
            $display("FAIL: case 2 block %0d mismatch", i);
            $display("      got      [63:0] = %016h", h2.mem[i][63:0]);
            $display("      expected [63:0] = %016h", expected2[i][63:0]);
            errors = errors + 1;
        end
    end
    if (errors == 0)
        $display("PASS: case 2 (m=32, t=1, 64-byte key) matches Argon2d reference");

    // --- Case 3: m = 6 (→ 8), t = 5, 100-byte key (multi-block H0) ---------
    @(posedge clk); #1; start3 = 1'b1;
    @(posedge clk); #1; start3 = 1'b0;
    wait_done("m=6 t=5 k=100", 2'd2);

    for (i = 0; i < MB3; i = i + 1) begin
        if (h3.mem[i] !== expected3[i]) begin
            $display("FAIL: case 3 block %0d mismatch", i);
            $display("      got      [63:0] = %016h", h3.mem[i][63:0]);
            $display("      expected [63:0] = %016h", expected3[i][63:0]);
            errors = errors + 1;
        end
    end
    if (errors == 0)
        $display("PASS: case 3 (m=6->8, t=5, 100-byte key) matches Argon2d reference");

    if (errors == 0)
        $display("=== tb_argon2_fill: ALL TESTS PASSED ===");
    else
        $display("=== tb_argon2_fill: %0d FAILURE(S) ===", errors);

    $finish;
end

endmodule
