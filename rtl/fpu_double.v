// =============================================================================
// fpu_double.v — IEEE 754 Double-precision FP Unit
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Handles RandomX floating-point instructions:
//   FADD_R, FADD_M, FSUB_R, FSUB_M,
//   FMUL_E, FDIV_M, FSQRT_R, FSCAL_R, FSWAP_R
//
// RandomX uses IEEE 754 double precision (64-bit) with a dynamic rounding
// mode held in the FPRC register (`round_mode` input).
//
// IMPLEMENTATION STATUS:
//   - FADD / FSUB : IEEE 754 add/sub, 1 cycle  (align / add / normalize / round)
//   - FMUL        : IEEE 754 multiply, 1 cycle (106-bit product)
//   - FDIV        : IEEE 754 divide, multi-cycle restoring division (57 cycles)
//   - FSQRT       : IEEE 754 square root, multi-cycle restoring sqrt (57 cycles)
//   - FSCAL       : bit manipulation (dst.u ^= 0x80F0000000000000)
//   - FSWAP       : returns the opposite half of the 128-bit register pair
//
// All operations support the four IEEE rounding modes, subnormal operands and
// results (gradual underflow), and the special values (NaN / Inf / +-0).
// A quiet NaN (0x7FF8000000000000) is produced for invalid operations.
//
// Handshake: assert `en` for one cycle with a valid opcode. `result_valid`
// pulses for one cycle when `result` is available (next cycle for the
// single-cycle ops, after the iteration count for FDIV/FSQRT). `busy` is high
// while a multi-cycle operation is in flight; `en` must not be asserted then.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module fpu_double (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         en,

    // Opcode
    input  wire [4:0]   opcode,

    // Operands (IEEE 754 double-precision 64-bit)
    input  wire [63:0]  src_a,
    input  wire [63:0]  src_b,

    // Rounding mode (from FPRC, 2-bit, maps to IEEE 754 rounding modes)
    // 00=nearest-even, 01=toward -inf, 10=toward +inf, 11=toward zero
    input  wire [1:0]   round_mode,

    // Result
    output reg  [63:0]  result,
    output reg          result_valid,
    output wire         busy
);

// ---------------------------------------------------------------------------
// Opcode constants
// ---------------------------------------------------------------------------
localparam FP_FADD_R  = 5'd0;
localparam FP_FADD_M  = 5'd1;
localparam FP_FSUB_R  = 5'd2;
localparam FP_FSUB_M  = 5'd3;
localparam FP_FMUL_E  = 5'd4;
localparam FP_FDIV_M  = 5'd5;
localparam FP_FSQRT_R = 5'd6;
localparam FP_FSCAL_R = 5'd7;
localparam FP_FSWAP_R = 5'd8;

localparam [63:0] QNAN     = 64'h7FF8000000000000;
localparam [63:0] POS_INF  = 64'h7FF0000000000000;
localparam [63:0] MAX_NORM = 64'h7FEFFFFFFFFFFFFF;

// Rounding modes
localparam RM_NEAR = 2'b00;
localparam RM_DOWN = 2'b01;
localparam RM_UP   = 2'b10;
localparam RM_ZERO = 2'b11;

// ---------------------------------------------------------------------------
// IEEE 754 Double decomposition
// ---------------------------------------------------------------------------
wire        a_sign = src_a[63];
wire [10:0] a_exp  = src_a[62:52];
wire [51:0] a_mant = src_a[51:0];

wire        b_sign = src_b[63];
wire [10:0] b_exp  = src_b[62:52];
wire [51:0] b_mant = src_b[51:0];

wire a_is_zero = (a_exp == 11'd0)    && (a_mant == 52'd0);
wire b_is_zero = (b_exp == 11'd0)    && (b_mant == 52'd0);
wire a_is_inf  = (a_exp == 11'h7FF)  && (a_mant == 52'd0);
wire b_is_inf  = (b_exp == 11'h7FF)  && (b_mant == 52'd0);
wire a_is_nan  = (a_exp == 11'h7FF)  && (a_mant != 52'd0);
wire b_is_nan  = (b_exp == 11'h7FF)  && (b_mant != 52'd0);
wire a_is_spec = (a_exp == 11'h7FF);
wire b_is_spec = (b_exp == 11'h7FF);

// ---------------------------------------------------------------------------
// Count leading zeros of a 53-bit value (returns 53 when the value is zero)
// ---------------------------------------------------------------------------
function [5:0] clz53;
    input [52:0] v;
    integer k;
    begin
        clz53 = 6'd53;
        for (k = 0; k < 53; k = k + 1)
            if (v[k]) clz53 = 6'd52 - k[5:0];
    end
endfunction

// Count leading zeros of a 57-bit value (returns 57 when the value is zero)
function [6:0] clz57;
    input [56:0] v;
    integer k;
    begin
        clz57 = 7'd57;
        for (k = 0; k < 57; k = k + 1)
            if (v[k]) clz57 = 7'd56 - k[6:0];
    end
endfunction

// ---------------------------------------------------------------------------
// Operand normalization: returns the significand (53-bit, MSB = hidden bit)
// and the unbiased-friendly *biased* exponent as a signed value. Subnormal
// inputs are normalized (exponent may become <= 0). Zero yields sig = 0.
// ---------------------------------------------------------------------------
function [52:0] norm_sig;
    input [10:0] e;
    input [51:0] m;
    reg   [5:0]  sh;
    begin
        if (e != 11'd0)
            norm_sig = {1'b1, m};
        else begin
            sh = clz53({1'b0, m});
            norm_sig = (m == 52'd0) ? 53'd0 : ({1'b0, m} << sh);
        end
    end
endfunction

function signed [13:0] norm_exp;
    input [10:0] e;
    input [51:0] m;
    reg   [5:0]  sh;
    begin
        if (e != 11'd0)
            norm_exp = {3'b000, e};
        else begin
            sh = clz53({1'b0, m});
            norm_exp = 14'sd1 - $signed({8'd0, sh});
        end
    end
endfunction

wire [52:0]        sig_a = norm_sig(a_exp, a_mant);
wire [52:0]        sig_b = norm_sig(b_exp, b_mant);
wire signed [13:0] exp_a = norm_exp(a_exp, a_mant);
wire signed [13:0] exp_b = norm_exp(b_exp, b_mant);

// ---------------------------------------------------------------------------
// Normalize (subnormal shift), round and pack.
//   sign : result sign
//   expi : biased exponent of the value sig[52].sig[51:0] x 2^(expi-1023)
//   sigi : 53-bit significand (leading bit at position 52)
//   g    : guard bit (one below the LSB of sigi)
//   s    : sticky bit (OR of everything below the guard bit)
// ---------------------------------------------------------------------------
function [63:0] round_pack;
    input               sign;
    input signed [13:0] expi;
    input        [52:0] sigi;
    input               g;
    input               s;
    input        [1:0]  rm;

    reg signed [13:0] e;
    reg        [53:0] sg;
    reg               gg, ss, inc;
    reg signed [13:0] k;
    reg signed [13:0] j;
    begin
        e  = expi;
        sg = {1'b0, sigi};
        gg = g;
        ss = s;

        // Gradual underflow: shift right until the biased exponent is 1
        if (e < 14'sd1) begin
            k = 14'sd1 - e;
            if (k > 14'sd56) begin
                ss = ss | gg | (|sg);
                sg = 54'd0;
                gg = 1'b0;
            end else begin
                for (j = 14'sd0; j < 14'sd56; j = j + 14'sd1) begin
                    if (j < k) begin
                        ss = ss | gg;
                        gg = sg[0];
                        sg = sg >> 1;
                    end
                end
            end
            e = 14'sd1;
        end

        // Rounding decision
        case (rm)
            RM_NEAR: inc = gg & (ss | sg[0]);
            RM_DOWN: inc = sign & (gg | ss);
            RM_UP:   inc = (~sign) & (gg | ss);
            default: inc = 1'b0;   // RM_ZERO
        endcase

        if (inc) sg = sg + 54'd1;

        // Carry out of the significand -> renormalize
        if (sg[53]) begin
            sg = sg >> 1;
            e  = e + 14'sd1;
        end

        if (sg == 54'd0) begin
            // Exact zero (underflow); sign is preserved
            round_pack = {sign, 63'd0};
        end else if (e >= 14'sd2047) begin
            // Overflow: infinity, except for the modes that round toward the
            // representable maximum magnitude
            if ((rm == RM_ZERO) || (rm == RM_DOWN && !sign) ||
                (rm == RM_UP   &&  sign))
                round_pack = {sign, MAX_NORM[62:0]};
            else
                round_pack = {sign, POS_INF[62:0]};
        end else begin
            // sg[52] = 1 -> normal (exponent e); sg[52] = 0 -> subnormal (e==1
            // is encoded as a zero exponent field)
            round_pack = {sign, (sg[52] ? e[10:0] : 11'd0), sg[51:0]};
        end
    end
endfunction

// ---------------------------------------------------------------------------
// FSCAL_R: dst.u ^= 0x80F0000000000000  (RandomX spec 4.6.6)
// Bit 63 (sign) and bits [55:52] (low 4 bits of the exponent field) are XORed.
// ---------------------------------------------------------------------------
wire [63:0] fscal_result = {~a_sign, a_exp ^ 11'h00F, a_mant};

// ---------------------------------------------------------------------------
// FSWAP_R: exchange the two halves of a 128-bit FP register pair. src_a is the
// half being written back, src_b is the opposite half, so the new value of the
// written half is simply src_b.
// ---------------------------------------------------------------------------
wire [63:0] fswap_result = src_b;

// ---------------------------------------------------------------------------
// FADD / FSUB (combinational)
// ---------------------------------------------------------------------------
wire        sub_op   = (opcode == FP_FSUB_R) || (opcode == FP_FSUB_M);
wire        eff_bsgn = b_sign ^ sub_op;                 // effective sign of B

// Order operands so that A' has the larger magnitude
wire a_ge_b = (exp_a > exp_b) || ((exp_a == exp_b) && (sig_a >= sig_b));

wire signed [13:0] add_exp_l = a_ge_b ? exp_a : exp_b;  // large
wire signed [13:0] add_exp_s = a_ge_b ? exp_b : exp_a;  // small
wire [52:0]        add_sig_l = a_ge_b ? sig_a : sig_b;
wire [52:0]        add_sig_s = a_ge_b ? sig_b : sig_a;
wire               add_sgn_l = a_ge_b ? a_sign  : eff_bsgn;
wire               add_sgn_s = a_ge_b ? eff_bsgn : a_sign;
wire               eff_sub   = add_sgn_l ^ add_sgn_s;

wire signed [13:0] exp_diff_s = add_exp_l - add_exp_s;
wire [6:0]         shamt      = (exp_diff_s > 14'sd57) ? 7'd57 : exp_diff_s[6:0];

// Significands scaled by 8 (3 spare LSBs: guard / round / sticky room)
wire [56:0] add_l_ext = {1'b0, add_sig_l, 3'b000};
wire [56:0] add_s_ext = {1'b0, add_sig_s, 3'b000};
wire [56:0] add_s_shf = add_s_ext >> shamt;
wire        add_stky  = |(add_s_ext & ~(({57{1'b1}}) << shamt));

// For subtraction the discarded bits make the small operand slightly larger,
// hence the extra -1 with a sticky of 1 (value = (l - s - 1) + (1 - eps)).
wire [56:0] add_sum = eff_sub ? (add_l_ext - add_s_shf - {56'd0, add_stky})
                              : (add_l_ext + add_s_shf);

wire [6:0]         add_lz    = clz57(add_sum);
wire [6:0]         add_shl   = (add_lz >= 7'd1) ? (add_lz - 7'd1) : 7'd0;
wire [56:0]        add_norm  = add_sum[56] ? (add_sum >> 1) : (add_sum << add_shl);
wire signed [13:0] add_exp_n = add_sum[56] ? (add_exp_l + 14'sd1)
                                           : (add_exp_l - $signed({7'd0, add_shl}));

wire add_g = add_norm[2];
wire add_s = add_sum[56] ? (add_norm[1] | add_norm[0] | add_sum[0] | add_stky)
                         : (add_norm[1] | add_norm[0] | add_stky);

// Zero result sign: exact cancellation yields +0 (-0 when rounding down)
wire        add_zero_sgn = (round_mode == RM_DOWN) ? 1'b1 : 1'b0;
wire [63:0] add_generic  = (add_sum == 57'd0)
                             ? {add_zero_sgn, 63'd0}
                             : round_pack(add_sgn_l, add_exp_n, add_norm[55:3],
                                          add_g, add_s, round_mode);

wire [63:0] addsub_result =
      (a_is_nan | b_is_nan)                     ? QNAN :
      (a_is_inf & b_is_inf & (a_sign ^ eff_bsgn))? QNAN :
       a_is_inf                                 ? {a_sign,   POS_INF[62:0]} :
       b_is_inf                                 ? {eff_bsgn, POS_INF[62:0]} :
      (a_is_zero & b_is_zero)                   ? {(a_sign & eff_bsgn) |
                                                   ((a_sign | eff_bsgn) &
                                                    (round_mode == RM_DOWN)), 63'd0} :
       a_is_zero                                ? {eff_bsgn, src_b[62:0]} :
       b_is_zero                                ? src_a :
                                                  add_generic;

// ---------------------------------------------------------------------------
// FMUL (combinational)
// ---------------------------------------------------------------------------
wire         mul_sign = a_sign ^ b_sign;
wire [105:0] mul_prod = sig_a * sig_b;

wire signed [13:0] mul_exp_raw = exp_a + exp_b - 14'sd1023;
wire signed [13:0] mul_exp_n   = mul_prod[105] ? (mul_exp_raw + 14'sd1) : mul_exp_raw;
wire [52:0]        mul_sig     = mul_prod[105] ? mul_prod[105:53] : mul_prod[104:52];
wire               mul_g       = mul_prod[105] ? mul_prod[52] : mul_prod[51];
wire               mul_s       = mul_prod[105] ? (|mul_prod[51:0]) : (|mul_prod[50:0]);

wire [63:0] mul_generic = round_pack(mul_sign, mul_exp_n, mul_sig,
                                     mul_g, mul_s, round_mode);

wire [63:0] mul_result =
      (a_is_nan | b_is_nan)                        ? QNAN :
      ((a_is_inf & b_is_zero) | (b_is_inf & a_is_zero)) ? QNAN :
      (a_is_inf | b_is_inf)                        ? {mul_sign, POS_INF[62:0]} :
      (a_is_zero | b_is_zero)                      ? {mul_sign, 63'd0} :
                                                     mul_generic;

// ---------------------------------------------------------------------------
// FDIV / FSQRT — iterative (restoring) implementation
// ---------------------------------------------------------------------------
localparam DIV_ITERS  = 7'd56;   // q = floor(sig_a * 2^56 / sig_b)
localparam SQRT_ITERS = 7'd56;   // 56 root bits

reg               it_active;
reg               it_is_sqrt;
reg  [6:0]        it_cnt;
reg               it_sign;
reg signed [13:0] it_exp;
reg  [56:0]       it_q;          // quotient / root accumulator
reg  [59:0]       it_rem;        // remainder
reg  [52:0]       it_div;        // divisor (FDIV)
reg  [111:0]      it_rad;        // radicand (FSQRT)
reg  [63:0]       it_special;
reg               it_use_special;

assign busy = it_active;

// Divide special cases / exponent seed
wire        div_sign = a_sign ^ b_sign;
wire [63:0] div_special =
      (a_is_nan | b_is_nan)              ? QNAN :
      (a_is_inf & b_is_inf)              ? QNAN :
      (a_is_zero & b_is_zero)            ? QNAN :
       a_is_inf                          ? {div_sign, POS_INF[62:0]} :
       b_is_zero                         ? {div_sign, POS_INF[62:0]} :  // x/0
       b_is_inf                          ? {div_sign, 63'd0} :
                                           {div_sign, 63'd0};           // 0/x
wire div_has_special = a_is_spec | b_is_spec | a_is_zero | b_is_zero;

// Square root special cases
wire        sqrt_neg     = a_sign & ~a_is_zero;
wire [63:0] sqrt_special = a_is_nan            ? QNAN :
                           sqrt_neg            ? QNAN :
                           a_is_inf            ? POS_INF :
                                                 {a_sign, 63'd0};  // +-0
wire sqrt_has_special = a_is_spec | a_is_zero | sqrt_neg;

// sqrt operand pre-scaling: make the unbiased exponent even
wire signed [13:0] sqrt_e   = exp_a - 14'sd1023;
wire [53:0]        sqrt_m   = sqrt_e[0] ? {sig_a, 1'b0} : {1'b0, sig_a};
wire signed [13:0] sqrt_exp = (sqrt_e >>> 1) + 14'sd1023;

// Iteration datapath (next-state values)
wire [59:0] div_rem_shl = {it_rem[58:0], 1'b0};
wire [59:0] div_dvsr     = {7'd0, it_div};
wire        div_ge       = (div_rem_shl >= div_dvsr);

wire [59:0] sqrt_rem_shl = {it_rem[57:0], it_rad[111:110]};
wire [59:0] sqrt_trial   = {1'b0, it_q[56:0], 2'b01};
wire        sqrt_ge      = (sqrt_rem_shl >= sqrt_trial);

// Result assembly for the iterative operations.
//   FDIV : q = floor(sig_a * 2^56 / sig_b) in [2^55, 2^57)
//   FSQRT: q = floor(sqrt(sqrt_m * 2^58)) in [2^55, 2^56)
wire        it_msb = ~it_is_sqrt & it_q[56];
wire [52:0] it_sig = it_msb ? it_q[56:4] : it_q[55:3];
wire        it_g   = it_msb ? it_q[3]    : it_q[2];
wire        it_st  = (it_msb ? (|it_q[2:0]) : (|it_q[1:0])) | (|it_rem);
wire signed [13:0] it_exp_f = (it_is_sqrt || it_msb) ? it_exp : (it_exp - 14'sd1);
wire [63:0] it_result = round_pack(it_sign, it_exp_f, it_sig, it_g, it_st, round_mode);

// ---------------------------------------------------------------------------
// Control / output
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        result         <= 64'b0;
        result_valid   <= 1'b0;
        it_active      <= 1'b0;
        it_is_sqrt     <= 1'b0;
        it_cnt         <= 7'd0;
        it_sign        <= 1'b0;
        it_exp         <= 14'sd0;
        it_q           <= 57'd0;
        it_rem         <= 60'd0;
        it_div         <= 53'd0;
        it_rad         <= 112'd0;
        it_special     <= 64'd0;
        it_use_special <= 1'b0;
    end else begin
        result_valid <= 1'b0;

        if (it_active) begin
            if (it_use_special) begin
                result         <= it_special;
                result_valid   <= 1'b1;
                it_active      <= 1'b0;
                it_use_special <= 1'b0;
            end else if (it_cnt == 7'd0) begin
                result       <= it_result;
                result_valid <= 1'b1;
                it_active    <= 1'b0;
            end else begin
                if (it_is_sqrt) begin
                    it_rad <= {it_rad[109:0], 2'b00};
                    if (sqrt_ge) begin
                        it_rem <= sqrt_rem_shl - sqrt_trial;
                        it_q   <= {it_q[55:0], 1'b1};
                    end else begin
                        it_rem <= sqrt_rem_shl;
                        it_q   <= {it_q[55:0], 1'b0};
                    end
                end else begin
                    if (div_ge) begin
                        it_rem <= div_rem_shl - div_dvsr;
                        it_q   <= {it_q[55:0], 1'b1};
                    end else begin
                        it_rem <= div_rem_shl;
                        it_q   <= {it_q[55:0], 1'b0};
                    end
                end
                it_cnt <= it_cnt - 7'd1;
            end
        end else if (en) begin
            case (opcode)
                FP_FDIV_M: begin
                    it_active      <= 1'b1;
                    it_is_sqrt     <= 1'b0;
                    it_use_special <= div_has_special;
                    it_special     <= div_special;
                    it_cnt         <= DIV_ITERS;
                    it_sign        <= div_sign;
                    it_exp         <= exp_a - exp_b + 14'sd1023;
                    // Pre-scale: sig_a/sig_b < 2, so one conditional subtract
                    // produces the integer bit of the quotient.
                    it_q           <= (sig_a >= sig_b) ? 57'd1 : 57'd0;
                    it_rem         <= (sig_a >= sig_b) ? {7'd0, sig_a - sig_b}
                                                       : {7'd0, sig_a};
                    it_div         <= sig_b;
                end

                FP_FSQRT_R: begin
                    it_active      <= 1'b1;
                    it_is_sqrt     <= 1'b1;
                    it_use_special <= sqrt_has_special;
                    it_special     <= sqrt_special;
                    it_cnt         <= SQRT_ITERS;
                    it_sign        <= 1'b0;
                    it_exp         <= sqrt_exp;
                    it_q           <= 57'd0;
                    it_rem         <= 60'd0;
                    it_rad         <= {sqrt_m, 58'd0};
                end

                default: begin
                    result_valid <= 1'b1;
                    case (opcode)
                        FP_FADD_R,
                        FP_FADD_M,
                        FP_FSUB_R,
                        FP_FSUB_M:  result <= addsub_result;
                        FP_FMUL_E:  result <= mul_result;
                        FP_FSCAL_R: result <= fscal_result;
                        FP_FSWAP_R: result <= fswap_result;
                        default:    result <= src_a;
                    endcase
                end
            endcase
        end
    end
end

endmodule
