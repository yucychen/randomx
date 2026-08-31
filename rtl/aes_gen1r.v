// =============================================================================
// aes_gen1r.v — AesGenerator1R
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX spec §3.3 (reference: fillAes1Rx4() in RandomX aes_hash.cpp).
// The 512-bit state is split into 4 × 128-bit lanes; every 64 bytes of output
// costs one AES round per lane:
//
//   state0 = aesdec(state0, key0)
//   state1 = aesenc(state1, key1)
//   state2 = aesdec(state2, key2)
//   state3 = aesenc(state3, key3)
//
// The updated state IS the output block, so the caller feeds `state_out` back
// into `state_in` to produce the next 64 bytes.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module aes_gen1r (
    input  wire         clk,
    input  wire         rst_n,
    // Start pulse: load state_in and begin
    input  wire         start,
    // 512-bit input state (4 × 128-bit lanes)
    input  wire [511:0] state_in,
    // 512-bit output state (updated after 1 AES round per lane)
    output reg  [511:0] state_out,
    // output valid pulse
    output reg          valid
);

// ---------------------------------------------------------------------------
// Round keys — AES_GEN_1R_KEY0..3 from the RandomX reference implementation.
// Each key is written as {word3, word2, word1, word0} so that byte 0 of the
// little-endian 16-byte key lands in bits [7:0], matching _mm_set_epi32().
//   key0..key3 = Blake2b-512("RandomX AesGenerator1R keys")
// ---------------------------------------------------------------------------
localparam [127:0] RK0 = {32'hb4f44917, 32'hdbb5552b, 32'h62716609, 32'h6daca553};
localparam [127:0] RK1 = {32'h0da1dc4e, 32'h1725d378, 32'h846a710d, 32'h6d7caf07};
localparam [127:0] RK2 = {32'h3e20e345, 32'hf4c0794f, 32'h9f947ec6, 32'h3f1262f1};
localparam [127:0] RK3 = {32'h49169154, 32'h16314c88, 32'hb1ba317c, 32'h6aef8135};

// Wires for AES round outputs per lane
wire [127:0] lane_in  [0:3];
wire [127:0] lane_out [0:3];

assign lane_in[0] = state_in[127:  0];
assign lane_in[1] = state_in[255:128];
assign lane_in[2] = state_in[383:256];
assign lane_in[3] = state_in[511:384];

// Lanes 0/2 use the decryption round, lanes 1/3 the encryption round.
aes_round u_rnd0 (
    .state_in  (lane_in[0]),
    .round_key (RK0),
    .last_round(1'b0),
    .dec       (1'b1),
    .state_out (lane_out[0])
);

aes_round u_rnd1 (
    .state_in  (lane_in[1]),
    .round_key (RK1),
    .last_round(1'b0),
    .dec       (1'b0),
    .state_out (lane_out[1])
);

aes_round u_rnd2 (
    .state_in  (lane_in[2]),
    .round_key (RK2),
    .last_round(1'b0),
    .dec       (1'b1),
    .state_out (lane_out[2])
);

aes_round u_rnd3 (
    .state_in  (lane_in[3]),
    .round_key (RK3),
    .last_round(1'b0),
    .dec       (1'b0),
    .state_out (lane_out[3])
);

// ---------------------------------------------------------------------------
// Register outputs
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state_out <= 512'b0;
        valid     <= 1'b0;
    end else begin
        valid <= 1'b0;
        if (start) begin
            state_out <= {lane_out[3], lane_out[2], lane_out[1], lane_out[0]};
            valid     <= 1'b1;
        end
    end
end

endmodule
