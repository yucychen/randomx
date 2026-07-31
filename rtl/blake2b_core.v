// =============================================================================
// blake2b_core.v — Blake2b-512 compression core
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Blake2b-512 is used for:
//   - Seed expansion → Cache (Argon2d)
//   - Header hash
//   - Final output hash
//
// Implements the complete compression function F:
//   full sigma message schedule, 12 rounds, 4 G-functions evaluated in
//   parallel per cycle (one half-round per cycle → 24 cycles per block),
//   finalization h[i] ^= v[i] ^ v[i+8].
// Verified against the RFC 7693 test vectors.
//
// Usage:
//   - Assert `start` for one cycle together with `msg_block`, `byte_count`
//     (total bytes hashed *including* this block) and `last_block`.
//   - `h_in` supplies the incoming chaining value. Assert `init` instead to
//     let the core start from the Blake2b parameter block IV
//     (unkeyed, digest length = DIGEST_BYTES, fanout = depth = 1); `h_in` is
//     then ignored, which allows a single-block message to be hashed without
//     the caller having to know the IV.
//   - `busy` is high while a compression is in flight; `start` is ignored
//     while busy. `done` pulses for one cycle when `h_out` is valid.
//
// Verilog-2001 compliant, no vendor IP.
// =============================================================================

`timescale 1ns/1ps

module blake2b_core #(
    parameter DIGEST_BYTES = 64   // digest length used for the parameter block
) (
    input  wire          clk,
    input  wire          rst_n,
    // Control
    input  wire          start,        // Begin compression (ignored while busy)
    input  wire          init,         // Start from parameter-block IV (h_in ignored)
    input  wire          last_block,   // Final block flag (sets f0 = ~0)
    // Message block: 16 × 64-bit words (1024 bits)
    input  wire [1023:0] msg_block,
    // Byte counter: total bytes hashed so far
    input  wire [127:0]  byte_count,
    // Initial chaining values (h[0..7], 512 bits)
    input  wire [511:0]  h_in,
    // Output chaining values after compression
    output reg  [511:0]  h_out,
    // Status
    output wire          busy,         // High while a compression is running
    output reg           done          // One-cycle pulse when h_out is valid
);

// ---------------------------------------------------------------------------
// Blake2b constants (initialization vector = SHA-512 IV)
// ---------------------------------------------------------------------------
localparam [63:0] IV0 = 64'h6a09e667f3bcc908;
localparam [63:0] IV1 = 64'hbb67ae8584caa73b;
localparam [63:0] IV2 = 64'h3c6ef372fe94f82b;
localparam [63:0] IV3 = 64'ha54ff53a5f1d36f1;
localparam [63:0] IV4 = 64'h510e527fade682d1;
localparam [63:0] IV5 = 64'h9b05688c2b3e6c1f;
localparam [63:0] IV6 = 64'h1f83d9abfb41bd6b;
localparam [63:0] IV7 = 64'h5be0cd19137e2179;

// Parameter block for an unkeyed hash: digest_length | key_length=0 |
// fanout=1 | depth=1, all other fields zero.
localparam [63:0] PARAM0 = 64'h0000000001010000 | DIGEST_BYTES;
localparam [63:0] H_INIT0 = IV0 ^ PARAM0;

// ---------------------------------------------------------------------------
// Blake2b sigma permutation lookup — round r, position p → message index
// Full table per RFC 7693 section 2.7. Rounds 10 and 11 repeat rounds 0, 1.
// ---------------------------------------------------------------------------
function [3:0] sigma;
    input [3:0] r;
    input [3:0] p;
    reg   [3:0] rr;
    begin
        rr = (r >= 4'd10) ? (r - 4'd10) : r;  // rounds 10,11 -> 0,1
        case ({rr, p})
            // Round 0
            8'h00: sigma = 4'd0;  8'h01: sigma = 4'd1;  8'h02: sigma = 4'd2;  8'h03: sigma = 4'd3;
            8'h04: sigma = 4'd4;  8'h05: sigma = 4'd5;  8'h06: sigma = 4'd6;  8'h07: sigma = 4'd7;
            8'h08: sigma = 4'd8;  8'h09: sigma = 4'd9;  8'h0a: sigma = 4'd10; 8'h0b: sigma = 4'd11;
            8'h0c: sigma = 4'd12; 8'h0d: sigma = 4'd13; 8'h0e: sigma = 4'd14; 8'h0f: sigma = 4'd15;
            // Round 1
            8'h10: sigma = 4'd14; 8'h11: sigma = 4'd10; 8'h12: sigma = 4'd4;  8'h13: sigma = 4'd8;
            8'h14: sigma = 4'd9;  8'h15: sigma = 4'd15; 8'h16: sigma = 4'd13; 8'h17: sigma = 4'd6;
            8'h18: sigma = 4'd1;  8'h19: sigma = 4'd12; 8'h1a: sigma = 4'd0;  8'h1b: sigma = 4'd2;
            8'h1c: sigma = 4'd11; 8'h1d: sigma = 4'd7;  8'h1e: sigma = 4'd5;  8'h1f: sigma = 4'd3;
            // Round 2
            8'h20: sigma = 4'd11; 8'h21: sigma = 4'd8;  8'h22: sigma = 4'd12; 8'h23: sigma = 4'd0;
            8'h24: sigma = 4'd5;  8'h25: sigma = 4'd2;  8'h26: sigma = 4'd15; 8'h27: sigma = 4'd13;
            8'h28: sigma = 4'd10; 8'h29: sigma = 4'd14; 8'h2a: sigma = 4'd3;  8'h2b: sigma = 4'd6;
            8'h2c: sigma = 4'd7;  8'h2d: sigma = 4'd1;  8'h2e: sigma = 4'd9;  8'h2f: sigma = 4'd4;
            // Round 3
            8'h30: sigma = 4'd7;  8'h31: sigma = 4'd9;  8'h32: sigma = 4'd3;  8'h33: sigma = 4'd1;
            8'h34: sigma = 4'd13; 8'h35: sigma = 4'd12; 8'h36: sigma = 4'd11; 8'h37: sigma = 4'd14;
            8'h38: sigma = 4'd2;  8'h39: sigma = 4'd6;  8'h3a: sigma = 4'd5;  8'h3b: sigma = 4'd10;
            8'h3c: sigma = 4'd4;  8'h3d: sigma = 4'd0;  8'h3e: sigma = 4'd15; 8'h3f: sigma = 4'd8;
            // Round 4
            8'h40: sigma = 4'd9;  8'h41: sigma = 4'd0;  8'h42: sigma = 4'd5;  8'h43: sigma = 4'd7;
            8'h44: sigma = 4'd2;  8'h45: sigma = 4'd4;  8'h46: sigma = 4'd10; 8'h47: sigma = 4'd15;
            8'h48: sigma = 4'd14; 8'h49: sigma = 4'd1;  8'h4a: sigma = 4'd11; 8'h4b: sigma = 4'd12;
            8'h4c: sigma = 4'd6;  8'h4d: sigma = 4'd8;  8'h4e: sigma = 4'd3;  8'h4f: sigma = 4'd13;
            // Round 5
            8'h50: sigma = 4'd2;  8'h51: sigma = 4'd12; 8'h52: sigma = 4'd6;  8'h53: sigma = 4'd10;
            8'h54: sigma = 4'd0;  8'h55: sigma = 4'd11; 8'h56: sigma = 4'd8;  8'h57: sigma = 4'd3;
            8'h58: sigma = 4'd4;  8'h59: sigma = 4'd13; 8'h5a: sigma = 4'd7;  8'h5b: sigma = 4'd5;
            8'h5c: sigma = 4'd15; 8'h5d: sigma = 4'd14; 8'h5e: sigma = 4'd1;  8'h5f: sigma = 4'd9;
            // Round 6
            8'h60: sigma = 4'd12; 8'h61: sigma = 4'd5;  8'h62: sigma = 4'd1;  8'h63: sigma = 4'd15;
            8'h64: sigma = 4'd14; 8'h65: sigma = 4'd13; 8'h66: sigma = 4'd4;  8'h67: sigma = 4'd10;
            8'h68: sigma = 4'd0;  8'h69: sigma = 4'd7;  8'h6a: sigma = 4'd6;  8'h6b: sigma = 4'd3;
            8'h6c: sigma = 4'd9;  8'h6d: sigma = 4'd2;  8'h6e: sigma = 4'd8;  8'h6f: sigma = 4'd11;
            // Round 7
            8'h70: sigma = 4'd13; 8'h71: sigma = 4'd11; 8'h72: sigma = 4'd7;  8'h73: sigma = 4'd14;
            8'h74: sigma = 4'd12; 8'h75: sigma = 4'd1;  8'h76: sigma = 4'd3;  8'h77: sigma = 4'd9;
            8'h78: sigma = 4'd5;  8'h79: sigma = 4'd0;  8'h7a: sigma = 4'd15; 8'h7b: sigma = 4'd4;
            8'h7c: sigma = 4'd8;  8'h7d: sigma = 4'd6;  8'h7e: sigma = 4'd2;  8'h7f: sigma = 4'd10;
            // Round 8
            8'h80: sigma = 4'd6;  8'h81: sigma = 4'd15; 8'h82: sigma = 4'd14; 8'h83: sigma = 4'd9;
            8'h84: sigma = 4'd11; 8'h85: sigma = 4'd3;  8'h86: sigma = 4'd0;  8'h87: sigma = 4'd8;
            8'h88: sigma = 4'd12; 8'h89: sigma = 4'd2;  8'h8a: sigma = 4'd13; 8'h8b: sigma = 4'd7;
            8'h8c: sigma = 4'd1;  8'h8d: sigma = 4'd4;  8'h8e: sigma = 4'd10; 8'h8f: sigma = 4'd5;
            // Round 9
            8'h90: sigma = 4'd10; 8'h91: sigma = 4'd2;  8'h92: sigma = 4'd8;  8'h93: sigma = 4'd4;
            8'h94: sigma = 4'd7;  8'h95: sigma = 4'd6;  8'h96: sigma = 4'd1;  8'h97: sigma = 4'd5;
            8'h98: sigma = 4'd15; 8'h99: sigma = 4'd11; 8'h9a: sigma = 4'd9;  8'h9b: sigma = 4'd14;
            8'h9c: sigma = 4'd3;  8'h9d: sigma = 4'd12; 8'h9e: sigma = 4'd13; 8'h9f: sigma = 4'd0;
            default: sigma = 4'd0;
        endcase
    end
endfunction

// ---------------------------------------------------------------------------
// Working state vector v[0..15], chaining value h[0..7], message m[0..15]
// ---------------------------------------------------------------------------
reg [63:0] v0,  v1,  v2,  v3,  v4,  v5,  v6,  v7;
reg [63:0] v8,  v9,  v10, v11, v12, v13, v14, v15;
reg [63:0] h0,  h1,  h2,  h3,  h4,  h5,  h6,  h7;
reg [1023:0] m;   // message block, 16 x 64-bit little-endian words

// Round counter (0..11) and half-round select (0 = columns, 1 = diagonals)
reg [3:0] round;
reg       half;
reg       busy_r;

assign busy = busy_r;

// ---------------------------------------------------------------------------
// Message word selection.
// The block is kept as a single vector so that the words can be selected with
// an indexed part-select; a function call inside a continuous assignment would
// only be re-evaluated when its explicit arguments change.
// ---------------------------------------------------------------------------
// Half-round message positions: columns use sigma positions 0..7,
// diagonals use positions 8..15.
wire [3:0] pos0 = {half, 3'd0};
wire [3:0] pos1 = {half, 3'd1};
wire [3:0] pos2 = {half, 3'd2};
wire [3:0] pos3 = {half, 3'd3};
wire [3:0] pos4 = {half, 3'd4};
wire [3:0] pos5 = {half, 3'd5};
wire [3:0] pos6 = {half, 3'd6};
wire [3:0] pos7 = {half, 3'd7};

wire [9:0] mo0 = {sigma(round, pos0), 6'b0};   // word index * 64
wire [9:0] mo1 = {sigma(round, pos1), 6'b0};
wire [9:0] mo2 = {sigma(round, pos2), 6'b0};
wire [9:0] mo3 = {sigma(round, pos3), 6'b0};
wire [9:0] mo4 = {sigma(round, pos4), 6'b0};
wire [9:0] mo5 = {sigma(round, pos5), 6'b0};
wire [9:0] mo6 = {sigma(round, pos6), 6'b0};
wire [9:0] mo7 = {sigma(round, pos7), 6'b0};

wire [63:0] gm0 = m[mo0 +: 64];
wire [63:0] gm1 = m[mo1 +: 64];
wire [63:0] gm2 = m[mo2 +: 64];
wire [63:0] gm3 = m[mo3 +: 64];
wire [63:0] gm4 = m[mo4 +: 64];
wire [63:0] gm5 = m[mo5 +: 64];
wire [63:0] gm6 = m[mo6 +: 64];
wire [63:0] gm7 = m[mo7 +: 64];

// ---------------------------------------------------------------------------
// Four parallel G-functions.
//   columns  (half = 0): G(v0,v4,v8, v12) G(v1,v5,v9, v13)
//                        G(v2,v6,v10,v14) G(v3,v7,v11,v15)
//   diagonals(half = 1): G(v0,v5,v10,v15) G(v1,v6,v11,v12)
//                        G(v2,v7,v8, v13) G(v3,v4,v9, v14)
// ---------------------------------------------------------------------------
wire [63:0] a0_in = v0;
wire [63:0] a1_in = v1;
wire [63:0] a2_in = v2;
wire [63:0] a3_in = v3;
wire [63:0] b0_in = half ? v5 : v4;
wire [63:0] b1_in = half ? v6 : v5;
wire [63:0] b2_in = half ? v7 : v6;
wire [63:0] b3_in = half ? v4 : v7;
wire [63:0] c0_in = half ? v10 : v8;
wire [63:0] c1_in = half ? v11 : v9;
wire [63:0] c2_in = half ? v8  : v10;
wire [63:0] c3_in = half ? v9  : v11;
wire [63:0] d0_in = half ? v15 : v12;
wire [63:0] d1_in = half ? v12 : v13;
wire [63:0] d2_in = half ? v13 : v14;
wire [63:0] d3_in = half ? v14 : v15;

wire [63:0] a0_out, b0_out, c0_out, d0_out;
wire [63:0] a1_out, b1_out, c1_out, d1_out;
wire [63:0] a2_out, b2_out, c2_out, d2_out;
wire [63:0] a3_out, b3_out, c3_out, d3_out;

blake2b_g u_g0 (
    .a_in (a0_in), .b_in (b0_in), .c_in (c0_in), .d_in (d0_in),
    .mx   (gm0),   .my   (gm1),
    .a_out(a0_out),.b_out(b0_out),.c_out(c0_out),.d_out(d0_out)
);
blake2b_g u_g1 (
    .a_in (a1_in), .b_in (b1_in), .c_in (c1_in), .d_in (d1_in),
    .mx   (gm2),   .my   (gm3),
    .a_out(a1_out),.b_out(b1_out),.c_out(c1_out),.d_out(d1_out)
);
blake2b_g u_g2 (
    .a_in (a2_in), .b_in (b2_in), .c_in (c2_in), .d_in (d2_in),
    .mx   (gm4),   .my   (gm5),
    .a_out(a2_out),.b_out(b2_out),.c_out(c2_out),.d_out(d2_out)
);
blake2b_g u_g3 (
    .a_in (a3_in), .b_in (b3_in), .c_in (c3_in), .d_in (d3_in),
    .mx   (gm6),   .my   (gm7),
    .a_out(a3_out),.b_out(b3_out),.c_out(c3_out),.d_out(d3_out)
);

// Next working-vector values for the current half-round
wire [63:0] v0_nxt = a0_out;
wire [63:0] v1_nxt = a1_out;
wire [63:0] v2_nxt = a2_out;
wire [63:0] v3_nxt = a3_out;
wire [63:0] v4_nxt  = half ? b3_out : b0_out;
wire [63:0] v5_nxt  = half ? b0_out : b1_out;
wire [63:0] v6_nxt  = half ? b1_out : b2_out;
wire [63:0] v7_nxt  = half ? b2_out : b3_out;
wire [63:0] v8_nxt  = half ? c2_out : c0_out;
wire [63:0] v9_nxt  = half ? c3_out : c1_out;
wire [63:0] v10_nxt = half ? c0_out : c2_out;
wire [63:0] v11_nxt = half ? c1_out : c3_out;
wire [63:0] v12_nxt = half ? d1_out : d0_out;
wire [63:0] v13_nxt = half ? d2_out : d1_out;
wire [63:0] v14_nxt = half ? d3_out : d2_out;
wire [63:0] v15_nxt = half ? d0_out : d3_out;

// Chaining value after the final half-round of the last round
wire [511:0] h_final = {
    h7 ^ v7_nxt ^ v15_nxt,
    h6 ^ v6_nxt ^ v14_nxt,
    h5 ^ v5_nxt ^ v13_nxt,
    h4 ^ v4_nxt ^ v12_nxt,
    h3 ^ v3_nxt ^ v11_nxt,
    h2 ^ v2_nxt ^ v10_nxt,
    h1 ^ v1_nxt ^ v9_nxt,
    h0 ^ v0_nxt ^ v8_nxt
};

// Starting chaining value: either supplied by the caller or the parameter IV
wire [511:0] h_start = init ? {IV7, IV6, IV5, IV4, IV3, IV2, IV1, H_INIT0} : h_in;

// ---------------------------------------------------------------------------
// FSM: 12 rounds × 2 half-rounds = 24 cycles per block
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        {v0,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15} <= {16{64'b0}};
        {h0,h1,h2,h3,h4,h5,h6,h7}                               <= {8{64'b0}};
        m      <= 1024'b0;
        round  <= 4'd0;
        half   <= 1'b0;
        busy_r <= 1'b0;
        h_out  <= 512'b0;
        done   <= 1'b0;
    end else begin
        done <= 1'b0;

        if (start && !busy_r) begin
            // Latch the incoming chaining value
            h0 <= h_start[ 63:  0]; h1 <= h_start[127: 64];
            h2 <= h_start[191:128]; h3 <= h_start[255:192];
            h4 <= h_start[319:256]; h5 <= h_start[383:320];
            h6 <= h_start[447:384]; h7 <= h_start[511:448];
            // Latch message words
            m <= msg_block;
            // Init working vector v[0..7] = h[0..7]
            v0  <= h_start[ 63:  0]; v1  <= h_start[127: 64];
            v2  <= h_start[191:128]; v3  <= h_start[255:192];
            v4  <= h_start[319:256]; v5  <= h_start[383:320];
            v6  <= h_start[447:384]; v7  <= h_start[511:448];
            // v[8..15] = IV; v12 ^= t0; v13 ^= t1; v14 ^= finalization flag
            v8  <= IV0; v9  <= IV1; v10 <= IV2; v11 <= IV3;
            v12 <= IV4 ^ byte_count[63:0];
            v13 <= IV5 ^ byte_count[127:64];
            v14 <= last_block ? (IV6 ^ 64'hffffffffffffffff) : IV6;
            v15 <= IV7;
            round  <= 4'd0;
            half   <= 1'b0;
            busy_r <= 1'b1;
        end else if (busy_r) begin
            // One half-round (4 G-functions) per cycle
            v0  <= v0_nxt;  v1  <= v1_nxt;  v2  <= v2_nxt;  v3  <= v3_nxt;
            v4  <= v4_nxt;  v5  <= v5_nxt;  v6  <= v6_nxt;  v7  <= v7_nxt;
            v8  <= v8_nxt;  v9  <= v9_nxt;  v10 <= v10_nxt; v11 <= v11_nxt;
            v12 <= v12_nxt; v13 <= v13_nxt; v14 <= v14_nxt; v15 <= v15_nxt;

            if (!half) begin
                half <= 1'b1;
            end else begin
                half <= 1'b0;
                if (round == 4'd11) begin
                    // Finalize: h_out[i] = h[i] ^ v[i] ^ v[i+8]
                    busy_r <= 1'b0;
                    done   <= 1'b1;
                    h_out  <= h_final;
                end else begin
                    round <= round + 4'd1;
                end
            end
        end
    end
end

endmodule

// =============================================================================
// blake2b_g — Blake2b mixing function G (purely combinational)
//   a = a + b + mx;  d = ror64(d ^ a, 32);  c = c + d;  b = ror64(b ^ c, 24)
//   a = a + b + my;  d = ror64(d ^ a, 16);  c = c + d;  b = ror64(b ^ c, 63)
// =============================================================================
module blake2b_g (
    input  wire [63:0] a_in,
    input  wire [63:0] b_in,
    input  wire [63:0] c_in,
    input  wire [63:0] d_in,
    input  wire [63:0] mx,
    input  wire [63:0] my,
    output wire [63:0] a_out,
    output wire [63:0] b_out,
    output wire [63:0] c_out,
    output wire [63:0] d_out
);

// Verilog-2001: no bit-select on expressions, so name every intermediate
wire [63:0] t0_a    = a_in + b_in + mx;
wire [63:0] t0_d_xr = d_in ^ t0_a;
wire [63:0] t0_d    = {t0_d_xr[31:0], t0_d_xr[63:32]};   // ror 32
wire [63:0] t0_c    = c_in + t0_d;
wire [63:0] t0_b_xr = b_in ^ t0_c;
wire [63:0] t0_b    = {t0_b_xr[23:0], t0_b_xr[63:24]};   // ror 24
wire [63:0] t1_a    = t0_a + t0_b + my;
wire [63:0] t1_d_xr = t0_d ^ t1_a;
wire [63:0] t1_d    = {t1_d_xr[15:0], t1_d_xr[63:16]};   // ror 16
wire [63:0] t1_c    = t0_c + t1_d;
wire [63:0] t1_b_xr = t0_b ^ t1_c;
wire [63:0] t1_b    = {t1_b_xr[62:0], t1_b_xr[63]};      // ror 63

assign a_out = t1_a;
assign b_out = t1_b;
assign c_out = t1_c;
assign d_out = t1_d;

endmodule
