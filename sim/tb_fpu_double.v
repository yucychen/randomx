// =============================================================================
// tb_fpu_double.v — Unit test for the IEEE 754 double-precision FPU
//
// Covers FADD / FSUB / FMUL / FDIV / FSQRT / FSCAL / FSWAP with:
//   - reference values for normal operands in all four rounding modes
//   - special values (NaN / +-Inf / +-0)
//   - subnormal operands and gradual underflow
//   - overflow behaviour per rounding mode
//
// Self-checking: prints FAIL on mismatch and finishes with a summary.
// =============================================================================

`timescale 1ns/1ps

module tb_fpu_double;

localparam FP_FADD_R  = 5'd0;
localparam FP_FSUB_R  = 5'd2;
localparam FP_FMUL_E  = 5'd4;
localparam FP_FDIV_M  = 5'd5;
localparam FP_FSQRT_R = 5'd6;
localparam FP_FSCAL_R = 5'd7;
localparam FP_FSWAP_R = 5'd8;

localparam RM_NEAR = 2'b00;
localparam RM_DOWN = 2'b01;
localparam RM_UP   = 2'b10;
localparam RM_ZERO = 2'b11;

reg         clk = 1'b0;
reg         rst_n = 1'b0;
reg         en = 1'b0;
reg  [4:0]  opcode = 5'd0;
reg  [63:0] src_a = 64'd0;
reg  [63:0] src_b = 64'd0;
reg  [1:0]  round_mode = 2'b00;
wire [63:0] result;
wire        result_valid;
wire        busy;

integer errors = 0;
integer checks = 0;

fpu_double dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (en),
    .opcode       (opcode),
    .src_a        (src_a),
    .src_b        (src_b),
    .round_mode   (round_mode),
    .result       (result),
    .result_valid (result_valid),
    .busy         (busy)
);

always #5 clk = ~clk;

// Issue one operation and compare against the expected encoding
task check;
    input [4:0]   op;
    input [1:0]   rm;
    input [63:0]  a;
    input [63:0]  b;
    input [63:0]  expected;
    input [255:0] name;
    integer       guard;
    begin
        @(negedge clk);
        opcode     = op;
        round_mode = rm;
        src_a      = a;
        src_b      = b;
        en         = 1'b1;
        @(negedge clk);
        en = 1'b0;
        guard = 0;
        while (!result_valid && guard < 200) begin
            @(negedge clk);
            guard = guard + 1;
        end
        checks = checks + 1;
        if (!result_valid) begin
            errors = errors + 1;
            $display("FAIL %0s: timeout waiting for result_valid", name);
        end else if (result !== expected) begin
            errors = errors + 1;
            $display("FAIL %0s: a=%h b=%h rm=%0d got=%h exp=%h",
                     name, a, b, rm, result, expected);
        end
    end
endtask

initial begin
    repeat (2) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);

    // ---------------- FADD ----------------
    check(FP_FADD_R, RM_NEAR, 64'h3FF0000000000000, 64'h3FF0000000000000,
          64'h4000000000000000, "fadd 1+1=2");
    check(FP_FADD_R, RM_NEAR, 64'h3FF0000000000000, 64'hBFF0000000000000,
          64'h0000000000000000, "fadd 1-1=+0");
    check(FP_FADD_R, RM_DOWN, 64'h3FF0000000000000, 64'hBFF0000000000000,
          64'h8000000000000000, "fadd 1-1=-0 (down)");
    check(FP_FADD_R, RM_NEAR, 64'h3FF0000000000000, 64'h3CB0000000000000,
          64'h3FF0000000000001, "fadd 1+2^-52");
    // 1 + 2^-53 : tie -> round to even (stays 1.0)
    check(FP_FADD_R, RM_NEAR, 64'h3FF0000000000000, 64'h3CA0000000000000,
          64'h3FF0000000000000, "fadd 1+2^-53 tie-even");
    check(FP_FADD_R, RM_UP,   64'h3FF0000000000000, 64'h3CA0000000000000,
          64'h3FF0000000000001, "fadd 1+2^-53 up");
    check(FP_FADD_R, RM_NEAR, 64'h4008000000000000, 64'h4010000000000000,
          64'h401C000000000000, "fadd 3+4=7");
    // subnormal + subnormal
    check(FP_FADD_R, RM_NEAR, 64'h0000000000000001, 64'h0000000000000001,
          64'h0000000000000002, "fadd subnormal");
    // largest normal + largest normal -> +Inf
    check(FP_FADD_R, RM_NEAR, 64'h7FEFFFFFFFFFFFFF, 64'h7FEFFFFFFFFFFFFF,
          64'h7FF0000000000000, "fadd overflow -> inf");
    check(FP_FADD_R, RM_ZERO, 64'h7FEFFFFFFFFFFFFF, 64'h7FEFFFFFFFFFFFFF,
          64'h7FEFFFFFFFFFFFFF, "fadd overflow -> max (zero)");
    // specials
    check(FP_FADD_R, RM_NEAR, 64'h7FF0000000000000, 64'hFFF0000000000000,
          64'h7FF8000000000000, "fadd inf-inf = nan");
    check(FP_FADD_R, RM_NEAR, 64'h7FF8000000000000, 64'h3FF0000000000000,
          64'h7FF8000000000000, "fadd nan");
    check(FP_FADD_R, RM_NEAR, 64'h7FF0000000000000, 64'h3FF0000000000000,
          64'h7FF0000000000000, "fadd inf+1");

    // ---------------- FSUB ----------------
    check(FP_FSUB_R, RM_NEAR, 64'h4010000000000000, 64'h3FF0000000000000,
          64'h4008000000000000, "fsub 4-1=3");
    check(FP_FSUB_R, RM_NEAR, 64'h3FF0000000000000, 64'h3FF0000000000000,
          64'h0000000000000000, "fsub 1-1=+0");
    check(FP_FSUB_R, RM_NEAR, 64'h3FF0000000000000, 64'h4000000000000000,
          64'hBFF0000000000000, "fsub 1-2=-1");
    check(FP_FSUB_R, RM_NEAR, 64'h0010000000000000, 64'h0000000000000001,
          64'h000FFFFFFFFFFFFF, "fsub normal-subnormal");

    // ---------------- FMUL ----------------
    check(FP_FMUL_E, RM_NEAR, 64'h4000000000000000, 64'h4008000000000000,
          64'h4018000000000000, "fmul 2*3=6");
    check(FP_FMUL_E, RM_NEAR, 64'hBFF0000000000000, 64'h4008000000000000,
          64'hC008000000000000, "fmul -1*3=-3");
    check(FP_FMUL_E, RM_NEAR, 64'h3FF0000000000001, 64'h3FF0000000000001,
          64'h3FF0000000000002, "fmul (1+ulp)^2");
    check(FP_FMUL_E, RM_NEAR, 64'h0000000000000000, 64'h7FF0000000000000,
          64'h7FF8000000000000, "fmul 0*inf = nan");
    check(FP_FMUL_E, RM_NEAR, 64'hBFF0000000000000, 64'h0000000000000000,
          64'h8000000000000000, "fmul -1*0 = -0");
    check(FP_FMUL_E, RM_NEAR, 64'h0000000000000001, 64'h3FE0000000000000,
          64'h0000000000000000, "fmul subnormal underflow to +0");
    check(FP_FMUL_E, RM_NEAR, 64'h7FE0000000000000, 64'h4000000000000000,
          64'h7FF0000000000000, "fmul overflow -> inf");

    // ---------------- FDIV ----------------
    check(FP_FDIV_M, RM_NEAR, 64'h4018000000000000, 64'h4008000000000000,
          64'h4000000000000000, "fdiv 6/3=2");
    check(FP_FDIV_M, RM_NEAR, 64'h3FF0000000000000, 64'h4008000000000000,
          64'h3FD5555555555555, "fdiv 1/3 near");
    check(FP_FDIV_M, RM_UP,   64'h3FF0000000000000, 64'h4008000000000000,
          64'h3FD5555555555556, "fdiv 1/3 up");
    check(FP_FDIV_M, RM_ZERO, 64'h3FF0000000000000, 64'h4008000000000000,
          64'h3FD5555555555555, "fdiv 1/3 zero");
    check(FP_FDIV_M, RM_NEAR, 64'h3FF0000000000000, 64'h0000000000000000,
          64'h7FF0000000000000, "fdiv 1/0 = inf");
    check(FP_FDIV_M, RM_NEAR, 64'h0000000000000000, 64'h0000000000000000,
          64'h7FF8000000000000, "fdiv 0/0 = nan");
    check(FP_FDIV_M, RM_NEAR, 64'h7FF0000000000000, 64'h7FF0000000000000,
          64'h7FF8000000000000, "fdiv inf/inf = nan");
    check(FP_FDIV_M, RM_NEAR, 64'h3FF0000000000000, 64'hFFF0000000000000,
          64'h8000000000000000, "fdiv 1/-inf = -0");
    check(FP_FDIV_M, RM_NEAR, 64'hC010000000000000, 64'h4000000000000000,
          64'hC000000000000000, "fdiv -4/2 = -2");

    // ---------------- FSQRT ----------------
    check(FP_FSQRT_R, RM_NEAR, 64'h4010000000000000, 64'h0,
          64'h4000000000000000, "fsqrt 4=2");
    check(FP_FSQRT_R, RM_NEAR, 64'h4022000000000000, 64'h0,
          64'h4008000000000000, "fsqrt 9=3");
    check(FP_FSQRT_R, RM_NEAR, 64'h4000000000000000, 64'h0,
          64'h3FF6A09E667F3BCD, "fsqrt 2 near");
    check(FP_FSQRT_R, RM_ZERO, 64'h4000000000000000, 64'h0,
          64'h3FF6A09E667F3BCC, "fsqrt 2 zero");
    check(FP_FSQRT_R, RM_UP,   64'h4000000000000000, 64'h0,
          64'h3FF6A09E667F3BCD, "fsqrt 2 up");
    check(FP_FSQRT_R, RM_NEAR, 64'h0000000000000000, 64'h0,
          64'h0000000000000000, "fsqrt +0");
    check(FP_FSQRT_R, RM_NEAR, 64'h8000000000000000, 64'h0,
          64'h8000000000000000, "fsqrt -0");
    check(FP_FSQRT_R, RM_NEAR, 64'hBFF0000000000000, 64'h0,
          64'h7FF8000000000000, "fsqrt -1 = nan");
    check(FP_FSQRT_R, RM_NEAR, 64'h7FF0000000000000, 64'h0,
          64'h7FF0000000000000, "fsqrt inf");

    // ---------------- FSCAL / FSWAP ----------------
    check(FP_FSCAL_R, RM_NEAR, 64'h3FF0000000000000, 64'h0,
          64'hBF00000000000000, "fscal 1.0");
    check(FP_FSCAL_R, RM_NEAR, 64'h0123456789ABCDEF, 64'h0,
          64'h81D3456789ABCDEF, "fscal pattern");
    check(FP_FSWAP_R, RM_NEAR, 64'h1111111122222222, 64'h3333333344444444,
          64'h3333333344444444, "fswap");

    // ---------------- busy handshake ----------------
    @(negedge clk);
    opcode     = FP_FDIV_M;
    round_mode = RM_NEAR;
    src_a      = 64'h4018000000000000;
    src_b      = 64'h4008000000000000;
    en         = 1'b1;
    @(negedge clk);
    en = 1'b0;
    @(negedge clk);
    checks = checks + 1;
    if (!busy) begin
        errors = errors + 1;
        $display("FAIL busy: expected busy high during FDIV");
    end
    while (!result_valid) @(negedge clk);
    @(negedge clk);
    checks = checks + 1;
    if (busy) begin
        errors = errors + 1;
        $display("FAIL busy: expected busy low after FDIV completes");
    end

    $display("tb_fpu_double: %0d checks, %0d errors", checks, errors);
    if (errors == 0)
        $display("tb_fpu_double: PASS");
    $finish;
end

endmodule
