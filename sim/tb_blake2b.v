// =============================================================================
// tb_blake2b.v — Blake2b-512 compression core testbench
// Verifies blake2b_core against RFC 7693 / reference test vectors:
//   1. Blake2b-512("abc") using an externally supplied parameter-block IV
//   2. Blake2b-512("abc") using the core's internal `init` IV generation
//   3. Blake2b-512("")     (empty message, single padded block)
//   4. Blake2b-512(200-byte message) — two-block streaming (chained h)
//   5. `busy` handshake: `start` is ignored while a compression is running
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
reg           init;
reg           last_block;
reg  [1023:0] msg_block;
reg  [127:0]  byte_count;
reg  [511:0]  h_in;
wire [511:0]  h_out;
wire          busy;
wire          done;

blake2b_core dut (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (start),
    .init       (init),
    .last_block (last_block),
    .msg_block  (msg_block),
    .byte_count (byte_count),
    .h_in       (h_in),
    .h_out      (h_out),
    .busy       (busy),
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

// Unkeyed Blake2b-512 initial chaining value
localparam [511:0] H_PARAM = {IV7, IV6, IV5, IV4, IV3, IV2, IV1,
                              IV0 ^ 64'h0000000001010040};

integer errors;
integer cycles;

// Expected digests (little-endian 64-bit words of the digest bytes)
reg [511:0] exp_abc;
reg [511:0] exp_empty;
reg [511:0] exp_200;

// 200-byte test message split into two 128-byte blocks (zero padded)
reg [1023:0] msg200_b0;
reg [1023:0] msg200_b1;
reg [511:0]  h_mid;

task check_digest;
    input [639:0] name;
    input [511:0] got;
    input [511:0] expect_val;
    integer w;
    begin
        if (got !== expect_val) begin
            $display("FAIL: %0s", name);
            for (w = 0; w < 8; w = w + 1)
                if (got[w*64 +: 64] !== expect_val[w*64 +: 64])
                    $display("      h[%0d] = %016h, expected %016h",
                             w, got[w*64 +: 64], expect_val[w*64 +: 64]);
            errors = errors + 1;
        end else begin
            $display("PASS: %0s", name);
        end
    end
endtask

// Run one compression and wait for done
task compress;
    input          use_init;
    input          is_last;
    input [1023:0] blk;
    input [127:0]  cnt;
    input [511:0]  chain;
    begin
        @(posedge clk); #1;
        init       = use_init;
        last_block = is_last;
        msg_block  = blk;
        byte_count = cnt;
        h_in       = chain;
        start      = 1'b1;
        @(posedge clk); #1;
        start = 1'b0;
        cycles = 0;
        while (!done && cycles < 300) begin
            @(posedge clk);
            cycles = cycles + 1;
        end
        if (!done) begin
            $display("FAIL: blake2b_core did not complete within %0d cycles", cycles);
            errors = errors + 1;
        end
    end
endtask

initial begin
    clk        = 1'b0;
    rst_n      = 1'b0;
    start      = 1'b0;
    init       = 1'b0;
    last_block = 1'b0;
    msg_block  = 1024'b0;
    byte_count = 128'b0;
    h_in       = 512'b0;
    errors     = 0;

    exp_abc   = {64'h239900d4ed8623b9, 64'h5a92f1dba88ad318, 64'h95cc3345ded552c2, 64'h2d79ab2a39c5877d, 64'hd1a2ffdb6fbb124b, 64'hb7c45a68142f214c, 64'he9f6129fb697276a, 64'h0d4d1c983fa580ba};
    exp_empty = {64'hcee29bfe1a706fd5, 64'h55b748145b683a90, 64'h4bb04e9344648913, 64'h5358eeaf31105ed2, 64'h19541ff717e2868a, 64'h614758e140472f91, 64'h72d2522585fdc6c6, 64'h03590142f7026a78};
    exp_200   = {64'h9ee0ca914e5c1fdc, 64'h52bcfa0b1ffcc22f, 64'h7174d6b20b728074, 64'hd80adba17d503250, 64'hbef9d43c4b9376d4, 64'h1bf9b6badba6d2fa, 64'he82c332ca75becb1, 64'hcffae0f60ee882dd};
    msg200_b0 = 1024'h7c756e676059524b443d362f28211a130c05fef7f0e9e2dbd4cdc6bfb8b1aaa39c958e878079726b645d564f48413a332c251e17100902fbf4ede6dfd8d1cac3bcb5aea7a099928b847d766f68615a534c453e373029221b140d06fff8f1eae3dcd5cec7c0b9b2aba49d968f88817a736c655e575049423b342d261f18110a03;
    msg200_b1 = 1024'h0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000746d665f58514a433c352e272019120b04fdf6efe8e1dad3ccc5beb7b0a9a29b948d867f78716a635c554e474039322b241d160f0801faf3ece5ded7d0c9c2bbb4ada69f98918a83;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // --- Test 1: "abc" with externally supplied IV ---
    msg_block       = 1024'b0;
    msg_block[63:0] = 64'h0000000000636261;
    compress(1'b0, 1'b1, {960'b0, 64'h0000000000636261}, 128'd3, H_PARAM);
    check_digest("blake2b-512(\"abc\") with external IV", h_out, exp_abc);

    // --- Test 2: same message, IV generated internally via `init` ---
    compress(1'b1, 1'b1, {960'b0, 64'h0000000000636261}, 128'd3, 512'hdead_beef);
    check_digest("blake2b-512(\"abc\") with init IV", h_out, exp_abc);

    // --- Test 3: empty message (single all-zero block, t = 0) ---
    compress(1'b1, 1'b1, 1024'b0, 128'd0, 512'b0);
    check_digest("blake2b-512(\"\")", h_out, exp_empty);

    // --- Test 4: 200-byte message → two blocks, chained ---
    compress(1'b1, 1'b0, msg200_b0, 128'd128, 512'b0);
    h_mid = h_out;
    compress(1'b0, 1'b1, msg200_b1, 128'd200, h_mid);
    check_digest("blake2b-512(200-byte message, 2 blocks)", h_out, exp_200);

    // --- Test 5: start is ignored while busy ---
    @(posedge clk); #1;
    init       = 1'b1;
    last_block = 1'b1;
    msg_block  = {960'b0, 64'h0000000000636261};
    byte_count = 128'd3;
    h_in       = 512'b0;
    start      = 1'b1;
    @(posedge clk); #1;
    if (!busy) begin
        $display("FAIL: busy not asserted after start");
        errors = errors + 1;
    end
    // Keep start asserted for a few cycles: must not restart the compression
    repeat (5) @(posedge clk); #1;
    start      = 1'b0;
    msg_block  = 1024'b0;     // corrupt inputs — must be ignored
    byte_count = 128'd0;
    cycles = 0;
    while (!done && cycles < 300) begin
        @(posedge clk);
        cycles = cycles + 1;
    end
    check_digest("start ignored while busy", h_out, exp_abc);
    if (busy) begin
        $display("FAIL: busy still asserted after done");
        errors = errors + 1;
    end

    if (errors == 0)
        $display("ALL TESTS PASSED: blake2b_core matches the reference vectors");
    else
        $display("FAILED: %0d test(s) failed", errors);

    $finish;
end

endmodule
