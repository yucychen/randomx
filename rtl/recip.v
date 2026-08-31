// =============================================================================
// recip.v — IMUL_RCP reciprocal unit
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX spec §5.5.11 (reference: randomx_reciprocal() in reciprocal.c):
//
//   quotient  = 2^63 / divisor, remainder = 2^63 % divisor
//   repeat bsr(divisor) times:      (bsr = number of significant bits)
//       if (2*remainder >= divisor) { quotient = 2*quotient + 1;
//                                     remainder = 2*remainder - divisor; }
//       else                        { quotient = 2*quotient;
//                                     remainder = 2*remainder; }
//
// Both loops are identical restoring-division steps, therefore the whole
// computation is a single restoring division of 2^(63+bsr) by the divisor,
// executed one bit per clock cycle (64 + bsr cycles, so at most 96 for the
// 32-bit divisors RandomX uses).
//
// Handshake: pulse `start` with `divisor`; `valid` pulses for one cycle when
// `quotient` holds the result. `busy` is high while the division runs and
// `start` is ignored during that time.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module recip (
    input  wire        clk,
    input  wire        rst_n,
    // Start pulse; `divisor` is sampled on the same edge
    input  wire        start,
    input  wire [63:0] divisor,
    // Result
    output reg  [63:0] quotient,
    output reg         valid,
    output wire        busy
);

reg  [63:0] div_r;
reg  [64:0] rem;
reg  [63:0] quot;
reg  [7:0]  cnt;        // remaining division steps
reg         first;
reg         running;

assign busy = running;

// bsr: index of the highest set bit of the divisor + 1 (1..64)
reg  [6:0] bsr;
integer    b;
always @(*) begin
    bsr = 7'd0;
    for (b = 0; b < 64; b = b + 1)
        if (div_r[b])
            bsr = b[6:0] + 7'd1;
end

// The dividend is 2^(63+bsr): a single 1 bit shifted in on the first step,
// followed by zeros.
wire [64:0] rem_shift = {rem[63:0], 1'b0} | {64'b0, first};
wire        ge        = (rem_shift >= {1'b0, div_r});

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        div_r    <= 64'b0;
        rem      <= 65'b0;
        quot     <= 64'b0;
        cnt      <= 8'd0;
        first    <= 1'b0;
        running  <= 1'b0;
        quotient <= 64'b0;
        valid    <= 1'b0;
    end else begin
        valid <= 1'b0;

        if (!running) begin
            if (start) begin
                div_r   <= divisor;
                rem     <= 65'b0;
                quot    <= 64'b0;
                first   <= 1'b1;
                running <= 1'b1;
            end
        end else if (first) begin
            // First cycle: `bsr` of the freshly loaded divisor is now valid.
            if (div_r == 64'b0) begin
                // Division by zero cannot occur in valid programs; return 0
                // instead of looping forever.
                quotient <= 64'b0;
                valid    <= 1'b1;
                running  <= 1'b0;
            end else begin
                cnt   <= {1'b0, bsr} + 8'd63;   // 64 + bsr - 1 more steps
                first <= 1'b0;
                rem   <= ge ? (rem_shift - {1'b0, div_r}) : rem_shift;
                quot  <= {quot[62:0], ge};
            end
        end else begin
            rem  <= ge ? (rem_shift - {1'b0, div_r}) : rem_shift;
            quot <= {quot[62:0], ge};
            if (cnt == 8'd1) begin
                quotient <= {quot[62:0], ge};
                valid    <= 1'b1;
                running  <= 1'b0;
            end else begin
                cnt <= cnt - 8'd1;
            end
        end
    end
end

endmodule
