// =============================================================================
// aes_gen4r.v — AesGenerator4R
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX spec §3.4 (reference: fillAes4Rx4() in RandomX aes_hash.cpp).
// The 512-bit state is split into 4 × 128-bit lanes and every 64 bytes of
// output costs four AES rounds per lane:
//
//   round r (r = 0..3):
//     state0 = aesdec(state0, key[r])
//     state1 = aesenc(state1, key[r])
//     state2 = aesdec(state2, key[4+r])
//     state3 = aesenc(state3, key[4+r])
//
// The resulting state IS the output block; the caller feeds `state_out` back
// into `state_in` to produce the next 64 bytes.
//
// Implementation: 4-stage sequential FSM (one AES round per cycle per lane).
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module aes_gen4r (
    input  wire         clk,
    input  wire         rst_n,
    // Start pulse: load state_in and begin 4-round sequence
    input  wire         start,
    // 512-bit input state (4 × 128-bit lanes)
    input  wire [511:0] state_in,
    // 512-bit output state after 4 AES rounds per lane
    output reg  [511:0] state_out,
    // valid pulses 4 cycles after start
    output reg          valid
);

// ---------------------------------------------------------------------------
// Round keys — AES_GEN_4R_KEY0..7 from the RandomX reference implementation.
// Written as {word3, word2, word1, word0} so byte 0 of the little-endian
// 16-byte key lands in bits [7:0], matching _mm_set_epi32().
//   key0..key3 = Blake2b-512("RandomX AesGenerator4R keys 0-3")
//   key4..key7 = Blake2b-512("RandomX AesGenerator4R keys 4-7")
// ---------------------------------------------------------------------------
localparam [127:0] RK0 = {32'h99e5d23f, 32'h2f546d2b, 32'hd1833ddb, 32'h6421aadd};
localparam [127:0] RK1 = {32'ha5dfcde5, 32'h06f79d53, 32'hb6913f55, 32'hb20e3450};
localparam [127:0] RK2 = {32'h171c02bf, 32'h0aa4679f, 32'h515e7baf, 32'h5c3ed904};
localparam [127:0] RK3 = {32'hd8ded291, 32'hcd673785, 32'he78f5d08, 32'h85623763};
localparam [127:0] RK4 = {32'h229effb4, 32'h3d518b6d, 32'he3d6a7a6, 32'hb5826f73};
localparam [127:0] RK5 = {32'hb272b7d2, 32'he9024d4e, 32'h9c10b3d9, 32'hc7566bf3};
localparam [127:0] RK6 = {32'hf63befa7, 32'h2ba9660a, 32'hf765a38b, 32'hf273c9e7};
localparam [127:0] RK7 = {32'hc0b0762d, 32'h0c06d1fd, 32'h915839de, 32'h7a7cd609};

// ---------------------------------------------------------------------------
// Internal state registers — hold the 4 lanes between rounds
// ---------------------------------------------------------------------------
reg [127:0] lane0, lane1, lane2, lane3;
reg [1:0]   round_cnt; // 0..3 counts AES rounds completed
reg         running;

// ---------------------------------------------------------------------------
// Round-key MUX: lanes 0/1 walk key0..key3, lanes 2/3 walk key4..key7
// ---------------------------------------------------------------------------
reg [127:0] rk_lo, rk_hi;

always @(*) begin
    case (round_cnt)
        2'd0: begin rk_lo = RK0; rk_hi = RK4; end
        2'd1: begin rk_lo = RK1; rk_hi = RK5; end
        2'd2: begin rk_lo = RK2; rk_hi = RK6; end
        2'd3: begin rk_lo = RK3; rk_hi = RK7; end
        default: begin rk_lo = 128'b0; rk_hi = 128'b0; end
    endcase
end

// ---------------------------------------------------------------------------
// AES round combinational outputs for each lane
// ---------------------------------------------------------------------------
wire [127:0] out0, out1, out2, out3;

aes_round u_rnd0 (.state_in(lane0), .round_key(rk_lo), .last_round(1'b0), .dec(1'b1), .state_out(out0));
aes_round u_rnd1 (.state_in(lane1), .round_key(rk_lo), .last_round(1'b0), .dec(1'b0), .state_out(out1));
aes_round u_rnd2 (.state_in(lane2), .round_key(rk_hi), .last_round(1'b0), .dec(1'b1), .state_out(out2));
aes_round u_rnd3 (.state_in(lane3), .round_key(rk_hi), .last_round(1'b0), .dec(1'b0), .state_out(out3));

// ---------------------------------------------------------------------------
// FSM: 4-round counter
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lane0     <= 128'b0;
        lane1     <= 128'b0;
        lane2     <= 128'b0;
        lane3     <= 128'b0;
        round_cnt <= 2'd0;
        running   <= 1'b0;
        state_out <= 512'b0;
        valid     <= 1'b0;
    end else begin
        valid <= 1'b0;

        if (start) begin
            // Load input lanes and begin
            lane0     <= state_in[127:  0];
            lane1     <= state_in[255:128];
            lane2     <= state_in[383:256];
            lane3     <= state_in[511:384];
            round_cnt <= 2'd0;
            running   <= 1'b1;
        end else if (running) begin
            // Apply one AES round to all lanes
            lane0     <= out0;
            lane1     <= out1;
            lane2     <= out2;
            lane3     <= out3;
            round_cnt <= round_cnt + 2'd1;

            if (round_cnt == 2'd3) begin
                // Final round done
                running   <= 1'b0;
                state_out <= {out3, out2, out1, out0};
                valid     <= 1'b1;
            end
        end
    end
end

endmodule
