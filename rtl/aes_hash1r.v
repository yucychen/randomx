// =============================================================================
// aes_hash1r.v — AesHash1R
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX spec §3.5 (reference: hashAes1Rx4() in RandomX aes_hash.cpp).
// The message is absorbed 64 bytes at a time into a 512-bit state made of four
// 128-bit lanes; the message block itself is used as the AES round key:
//
//   state0 = aesenc(state0, in0)
//   state1 = aesdec(state1, in1)
//   state2 = aesenc(state2, in2)
//   state3 = aesdec(state3, in3)
//
// After the last block two extra rounds with the fixed keys xkey0 and xkey1
// are applied to achieve full diffusion, then the four lanes form the digest.
//
// Handshake:
//   start     - pulse with the first block; reloads the fixed initial state
//   blk_valid - pulse: absorb `data_in` (must not overlap with `busy`)
//   blk_last  - qualifier on blk_valid: run the two xkey rounds afterwards
//   valid     - pulses when `hash_out` holds the final 512-bit digest
// A single-block hash therefore asserts start, blk_valid and blk_last together.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module aes_hash1r (
    input  wire         clk,
    input  wire         rst_n,
    // Start a new hash (reloads the fixed initial state)
    input  wire         start,
    // Absorb one 64-byte block
    input  wire         blk_valid,
    // Marks the final block of the message
    input  wire         blk_last,
    // 512-bit input data (64 bytes)
    input  wire [511:0] data_in,
    // 512-bit hash output
    output reg  [511:0] hash_out,
    // High while the finishing rounds are in flight
    output wire         busy,
    // Valid pulse
    output reg          valid
);

// ---------------------------------------------------------------------------
// Fixed constants from the RandomX reference implementation, written as
// {word3, word2, word1, word0} so byte 0 lands in bits [7:0] (_mm_set_epi32).
//   state0..state3 = Blake2b-512("RandomX AesHash1R state")
//   xkey0, xkey1   = Blake2b-256("RandomX AesHash1R xkeys")
// ---------------------------------------------------------------------------
localparam [127:0] ST0 = {32'hd7983aad, 32'hcc82db47, 32'h9fa856de, 32'h92b52c0d};
localparam [127:0] ST1 = {32'hace78057, 32'hf59e125a, 32'h15c7b798, 32'h338d996e};
localparam [127:0] ST2 = {32'he8a07ce4, 32'h5079506b, 32'hae62c7d0, 32'h6a770017};
localparam [127:0] ST3 = {32'h7e994948, 32'h79a10005, 32'h07ad828d, 32'h630a240c};

localparam [127:0] XKEY0 = {32'h06890201, 32'h90dc56bf, 32'h8b24949f, 32'hf6fa8389};
localparam [127:0] XKEY1 = {32'hed18f99b, 32'hee1043c6, 32'h51f4e03c, 32'h61b263d1};

// Lane registers
reg [127:0] lane0, lane1, lane2, lane3;

// Finishing-round FSM
localparam ST_ABSORB = 2'd0;
localparam ST_XKEY0  = 2'd1;
localparam ST_XKEY1  = 2'd2;

reg [1:0] state;

assign busy = (state != ST_ABSORB);

// ---------------------------------------------------------------------------
// Round input MUX: `start` bypasses the lane registers with the fixed state so
// that start+blk_valid in the same cycle absorbs the first block directly.
// ---------------------------------------------------------------------------
wire [127:0] in0 = start ? ST0 : lane0;
wire [127:0] in1 = start ? ST1 : lane1;
wire [127:0] in2 = start ? ST2 : lane2;
wire [127:0] in3 = start ? ST3 : lane3;

// ---------------------------------------------------------------------------
// Round key MUX: the message block during absorption, xkey0/xkey1 afterwards.
// Every lane uses the same key; lanes 0/2 encrypt and lanes 1/3 decrypt.
// ---------------------------------------------------------------------------
reg [127:0] rk0, rk1, rk2, rk3;
always @(*) begin
    case (state)
        ST_XKEY0: begin rk0 = XKEY0; rk1 = XKEY0; rk2 = XKEY0; rk3 = XKEY0; end
        ST_XKEY1: begin rk0 = XKEY1; rk1 = XKEY1; rk2 = XKEY1; rk3 = XKEY1; end
        default: begin
            rk0 = data_in[127:  0];
            rk1 = data_in[255:128];
            rk2 = data_in[383:256];
            rk3 = data_in[511:384];
        end
    endcase
end

wire [127:0] out0, out1, out2, out3;

aes_round u_h0 (.state_in(in0), .round_key(rk0), .last_round(1'b0), .dec(1'b0), .state_out(out0));
aes_round u_h1 (.state_in(in1), .round_key(rk1), .last_round(1'b0), .dec(1'b1), .state_out(out1));
aes_round u_h2 (.state_in(in2), .round_key(rk2), .last_round(1'b0), .dec(1'b0), .state_out(out2));
aes_round u_h3 (.state_in(in3), .round_key(rk3), .last_round(1'b0), .dec(1'b1), .state_out(out3));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lane0    <= 128'b0;
        lane1    <= 128'b0;
        lane2    <= 128'b0;
        lane3    <= 128'b0;
        state    <= ST_ABSORB;
        hash_out <= 512'b0;
        valid    <= 1'b0;
    end else begin
        valid <= 1'b0;

        case (state)
            ST_ABSORB: begin
                if (blk_valid) begin
                    lane0 <= out0;
                    lane1 <= out1;
                    lane2 <= out2;
                    lane3 <= out3;
                    if (blk_last)
                        state <= ST_XKEY0;
                end
            end

            ST_XKEY0: begin
                lane0 <= out0;
                lane1 <= out1;
                lane2 <= out2;
                lane3 <= out3;
                state <= ST_XKEY1;
            end

            ST_XKEY1: begin
                lane0    <= out0;
                lane1    <= out1;
                lane2    <= out2;
                lane3    <= out3;
                hash_out <= {out3, out2, out1, out0};
                valid    <= 1'b1;
                state    <= ST_ABSORB;
            end

            default: state <= ST_ABSORB;
        endcase
    end
end

endmodule
