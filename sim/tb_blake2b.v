// =============================================================================
// tb_blake2b.v — Blake2b-512 compression core testbench
// Verifies blake2b_core against the RFC 7693 Appendix A test vector:
//   Blake2b-512("abc") =
//   BA80A53F981C4D0D 6A2797B69F12F6E9 4C212F14685AC4B7 4B12BB6FDBFFA2D1
//   7D87C5392AAB792D C252D5DE4533CC95 18D38AA8DBF1925A B92386EDD4009923
//
// Run:
//   iverilog -g2001 -o tb_blake2b.vvp rtl/blake2b_core.v sim/tb_blake2b.v
//   vvp tb_blake2b.vvp
// =============================================================================

`timescale 1ns/1ps

module tb_blake2b;

reg           clk;
reg           rst_n;
reg           start;
reg           last_block;
reg  [1023:0] msg_block;
reg  [127:0]  byte_count;
reg  [511:0]  h_in;
wire [511:0]  h_out;
wire          done;

blake2b_core dut (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (start),
    .last_block (last_block),
    .msg_block  (msg_block),
    .byte_count (byte_count),
    .h_in       (h_in),
    .h_out      (h_out),
    .done       (done)
);

always #5 clk = ~clk;

// Blake2b IV (SHA-512 IV)
localparam [63:0] IV0 = 64'h6a09e667f3bcc908;
localparam [63:0] IV1 = 64'hbb67ae8584caa73b;
localparam [63:0] IV2 = 64'h3c6ef372fe94f82b;
localparam [63:0] IV3 = 64'ha54ff53a5f1d36f1;
localparam [63:0] IV4 = 64'h510e527fade682d1;
localparam [63:0] IV5 = 64'h9b05688c2b3e6c1f;
localparam [63:0] IV6 = 64'h1f83d9abfb41bd6b;
localparam [63:0] IV7 = 64'h5be0cd19137e2179;

// Expected digest words (little-endian 64-bit words of the digest bytes)
localparam [63:0] E0 = 64'h0d4d1c983fa580ba;
localparam [63:0] E1 = 64'he9f6129fb697276a;
localparam [63:0] E2 = 64'hb7c45a68142f214c;
localparam [63:0] E3 = 64'hd1a2ffdb6fbb124b;
localparam [63:0] E4 = 64'h2d79ab2a39c5877d;
localparam [63:0] E5 = 64'h95cc3345ded552c2;
localparam [63:0] E6 = 64'h5a92f1dba88ad318;
localparam [63:0] E7 = 64'h239900d4ed8623b9;

integer errors;
integer i;

task check_word;
    input [2:0]  idx;
    input [63:0] got;
    input [63:0] expect_val;
    begin
        if (got !== expect_val) begin
            $display("FAIL: h[%0d] = %016h, expected %016h", idx, got, expect_val);
            errors = errors + 1;
        end
    end
endtask

initial begin
    clk        = 1'b0;
    rst_n      = 1'b0;
    start      = 1'b0;
    last_block = 1'b0;
    msg_block  = 1024'b0;
    byte_count = 128'b0;
    h_in       = 512'b0;
    errors     = 0;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // Blake2b-512, no key: h0 = IV0 ^ 0x01010040 (depth=1, fanout=1, digest=64)
    h_in = {IV7, IV6, IV5, IV4, IV3, IV2, IV1, IV0 ^ 64'h0000000001010040};

    // Message "abc": bytes 61 62 63, zero-padded to 128 bytes
    msg_block         = 1024'b0;
    msg_block[63:0]   = 64'h0000000000636261;
    byte_count        = 128'd3;
    last_block        = 1'b1;

    @(posedge clk);
    start = 1'b1;
    @(posedge clk);
    start = 1'b0;

    // Wait for done (12 rounds * 8 steps = 96 cycles + margin)
    i = 0;
    while (!done && i < 300) begin
        @(posedge clk);
        i = i + 1;
    end

    if (!done) begin
        $display("FAIL: blake2b_core did not complete within %0d cycles", i);
        $finish;
    end

    check_word(3'd0, h_out[ 63:  0], E0);
    check_word(3'd1, h_out[127: 64], E1);
    check_word(3'd2, h_out[191:128], E2);
    check_word(3'd3, h_out[255:192], E3);
    check_word(3'd4, h_out[319:256], E4);
    check_word(3'd5, h_out[383:320], E5);
    check_word(3'd6, h_out[447:384], E6);
    check_word(3'd7, h_out[511:448], E7);

    if (errors == 0)
        $display("PASS: blake2b_core matches RFC 7693 test vector (abc)");
    else
        $display("FAIL: %0d word mismatches", errors);

    $finish;
end

endmodule
