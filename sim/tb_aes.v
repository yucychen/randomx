// =============================================================================
// tb_aes.v - self-checking testbench for the RandomX AES primitives
//
// Golden vectors were produced with a software reference model of the x86
// AESENC/AESDEC round functions (validated against the FIPS-197 AES-128
// test vector) and the fillAes1Rx4 / fillAes4Rx4 / hashAes1Rx4 routines of
// the RandomX reference implementation.
//
// Build: iverilog -g2005 -o tb_aes sim/tb_aes.v rtl/aes_round.v \
//                 rtl/aes_gen1r.v rtl/aes_gen4r.v rtl/aes_hash1r.v
// =============================================================================

`timescale 1ns/1ps

module tb_aes;

reg clk = 1'b0;
reg rst_n = 1'b0;
always #5 clk = ~clk;

integer errors = 0;

task check512;
    input [255:0] name;
    input [511:0] got;
    input [511:0] exp;
    begin
        if (got !== exp) begin
            $display("FAIL %0s", name);
            $display("  got = %h", got);
            $display("  exp = %h", exp);
            errors = errors + 1;
        end else begin
            $display("PASS %0s", name);
        end
    end
endtask

task check128;
    input [255:0] name;
    input [127:0] got;
    input [127:0] exp;
    begin
        if (got !== exp) begin
            $display("FAIL %0s got=%h exp=%h", name, got, exp);
            errors = errors + 1;
        end else begin
            $display("PASS %0s", name);
        end
    end
endtask

// ---------------------------------------------------------------------------
// aes_round: single round in both directions
// ---------------------------------------------------------------------------
localparam [127:0] R_STATE = 128'h0123456789abcdeffedcba9876543210;
localparam [127:0] R_KEY   = 128'h000102030405060708090a0b0c0d0e0f;

reg  rnd_last, rnd_dec;
wire [127:0] rnd_out;

aes_round u_rnd (
    .state_in  (R_STATE),
    .round_key (R_KEY),
    .last_round(rnd_last),
    .dec       (rnd_dec),
    .state_out (rnd_out)
);

// ---------------------------------------------------------------------------
// AesGenerator1R / AesGenerator4R
// ---------------------------------------------------------------------------
localparam [511:0] SEED = 512'h0f0e0d0c0b0a090807060504030201001f1e1d1c1b1a191817161514131211102f2e2d2c2b2a292827262524232221203f3e3d3c3b3a39383736353433323130;

reg          g1_start, g4_start;
reg  [511:0] g1_state, g4_state;
wire [511:0] g1_out, g4_out;
wire         g1_valid, g4_valid;

aes_gen1r u_g1 (.clk(clk), .rst_n(rst_n), .start(g1_start),
                .state_in(g1_state), .state_out(g1_out), .valid(g1_valid));

aes_gen4r u_g4 (.clk(clk), .rst_n(rst_n), .start(g4_start),
                .state_in(g4_state), .state_out(g4_out), .valid(g4_valid));

// ---------------------------------------------------------------------------
// AesHash1R
// ---------------------------------------------------------------------------
reg          h_start, h_blk_valid, h_blk_last;
reg  [511:0] h_data;
wire [511:0] h_out;
wire         h_valid, h_busy;

aes_hash1r u_h (.clk(clk), .rst_n(rst_n), .start(h_start),
                .blk_valid(h_blk_valid), .blk_last(h_blk_last),
                .data_in(h_data), .hash_out(h_out), .busy(h_busy),
                .valid(h_valid));

// ---------------------------------------------------------------------------
// Message blocks for the AesHash1R streaming test
// ---------------------------------------------------------------------------
localparam [511:0] BLK0 = 512'h0f0e0d0c0b0a090807060504030201001f1e1d1c1b1a191817161514131211102f2e2d2c2b2a292827262524232221203f3e3d3c3b3a39383736353433323130;
localparam [511:0] BLK1 = 512'h2d2a2724211e1b1815120f0c090603005d5a5754514e4b4845423f3c393633308d8a8784817e7b7875726f6c69666360bdbab7b4b1aeaba8a5a29f9c99969390;
localparam [511:0] BLK2 = 512'hf0f1f2f3f4f5f6f7f8f9fafbfcfdfeffe0e1e2e3e4e5e6e7e8e9eaebecedeeefd0d1d2d3d4d5d6d7d8d9dadbdcdddedfc0c1c2c3c4c5c6c7c8c9cacbcccdcecf;

task absorb;
    input [511:0] blk;
    input         first;
    input         last;
    begin
        @(posedge clk);
        #0.1;
        h_start     = first;
        h_blk_valid = 1'b1;
        h_blk_last  = last;
        h_data      = blk;
        @(posedge clk);
        #0.1;
        h_start     = 1'b0;
        h_blk_valid = 1'b0;
        h_blk_last  = 1'b0;
    end
endtask

integer i;

initial begin
    g1_start    = 1'b0;
    g4_start    = 1'b0;
    g1_state    = 512'b0;
    g4_state    = 512'b0;
    h_start     = 1'b0;
    h_blk_valid = 1'b0;
    h_blk_last  = 1'b0;
    h_data      = 512'b0;
    rnd_last    = 1'b0;
    rnd_dec     = 1'b0;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // --- aes_round ---
    rnd_dec = 1'b0; rnd_last = 1'b0; #1;
    check128("aes_round AESENC", rnd_out, 128'h6442f7565d22de8b29f67f452773ed99);
    rnd_dec = 1'b0; rnd_last = 1'b1; #1;
    check128("aes_round AESENCLAST", rnd_out, 128'ha7872186bf2568d8302fb74d706ffac5);
    rnd_dec = 1'b1; rnd_last = 1'b0; #1;
    check128("aes_round AESDEC", rnd_out, 128'ha94508f21f867eb27a3a36f5cd72872e);
    rnd_dec = 1'b1; rnd_last = 1'b1; #1;
    check128("aes_round AESDECLAST", rnd_out, 128'h0f9282090df8c666fa3babe900036673);

    // --- AesGenerator1R: three chained 64-byte blocks ---
    g1_state = SEED;
    for (i = 0; i < 3; i = i + 1) begin
        @(posedge clk); #0.1;
        g1_start = 1'b1;
        @(posedge clk); #0.1;
        g1_start = 1'b0;
        while (!g1_valid) @(posedge clk);
        #0.1;
        g1_state = g1_out;
        case (i)
            0: check512("aes_gen1r block 0", g1_out, 512'h15370d73776c9538e0895c502fb3eb5f945d2b6d87d22799f3c80459514599556dd0bfb683bb2e3c06f163825f1a47c3c992e1d0ed1ec9016c4028e633d7ffbf);
            1: check512("aes_gen1r block 1", g1_out, 512'h7c21d0237c03a868ded3962b88217367fd01f3f81e9fc80d633e9cf92d8fd02b1d39dd644528ba6075e25fc77622759b6e9d2ba05befc29e7ed911656dc877a0);
            2: check512("aes_gen1r block 2", g1_out, 512'hea4308e46ec2e872153f87e1353ca2e44be92b4abc036d6d9f3d3ec609ec1c5c11e663820fb5a99543344ab7f938c6f59e05112dcd866efda759d5cbbc70a0fc);
        endcase
    end

    // --- AesGenerator4R: two chained 64-byte blocks ---
    g4_state = SEED;
    for (i = 0; i < 2; i = i + 1) begin
        @(posedge clk); #0.1;
        g4_start = 1'b1;
        @(posedge clk); #0.1;
        g4_start = 1'b0;
        while (!g4_valid) @(posedge clk);
        #0.1;
        g4_state = g4_out;
        case (i)
            0: check512("aes_gen4r block 0", g4_out, 512'h1b38bc4eb12b283045ca376df4127c56cb156a9072669fdce75ade709c7edc3ccc3a383063beef8349059c43d82b85ff5db674811ae46ec6657e3df67329257f);
            1: check512("aes_gen4r block 1", g4_out, 512'h8f962a7c899ffc4216794f2b6db4be26db910c12cd4aedd01f7ac7f6859337883a196170a5750d2cf2f3d915e1d5db61b80693e83f00cee24eae1d62a7b6494d);
        endcase
    end

    // --- AesHash1R: single 64-byte block ---
    absorb(BLK0, 1'b1, 1'b1);
    while (!h_valid) @(posedge clk);
    #0.1;
    check512("aes_hash1r 1 block", h_out, 512'h354c9f82be94efe7080198f3070a70acbfffd43c32dc0313c059d2ac6657fc8891e137146d8ae7d52d490141d8936c03217d1cbae39b45a44b0a2a243245f2e4);

    // --- AesHash1R: three 64-byte blocks ---
    absorb(BLK0, 1'b1, 1'b0);
    absorb(BLK1, 1'b0, 1'b0);
    absorb(BLK2, 1'b0, 1'b1);
    while (!h_valid) @(posedge clk);
    #0.1;
    check512("aes_hash1r 3 blocks", h_out, 512'h3f830be90feb4c58dba903552a2cbd4b2e2e6ecea44bf4f1b139362de400ae116cd3456bde1dccd86cffdba422a5c27597fceabb440b912309dc607e6e243943);

    if (errors == 0)
        $display("=== tb_aes: ALL TESTS PASSED ===");
    else
        $display("=== tb_aes: %0d FAILURE(S) ===", errors);

    $finish;
end

initial begin
    #200000;
    $display("ERROR: tb_aes timeout");
    $finish;
end

endmodule
