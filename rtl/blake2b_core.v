// =============================================================================
// blake2b_core.v — Blake2b-512 hash core skeleton
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Blake2b-512 is used for:
//   - Seed expansion → Cache (Argon2d)
//   - Header hash
//   - Final output hash
//
// This skeleton implements the structural datapath:
//   G-function, message schedule, compression FSM.
// TODO: Complete the full 12-round compression loop.
//
// Verilog-2001 compliant, no vendor IP.
// =============================================================================

`timescale 1ns/1ps

module blake2b_core (
    input  wire          clk,
    input  wire          rst_n,
    // Control
    input  wire          start,        // Begin compression
    input  wire          last_block,   // Final block flag (unused skeleton)
    // Message block: 16 × 64-bit words (1024 bits)
    input  wire [1023:0] msg_block,
    // Byte counter: total bytes hashed so far
    input  wire [127:0]  byte_count,
    // Initial chaining values (h[0..7], 512 bits)
    input  wire [511:0]  h_in,
    // Output chaining values after compression
    output reg  [511:0]  h_out,
    // Done pulse
    output reg           done
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

// ---------------------------------------------------------------------------
// Blake2b sigma permutation lookup — round r, position p → message index
// Only round 0 and 1 filled; TODO: complete rounds 2–11 per spec table
// ---------------------------------------------------------------------------
function [3:0] sigma;
    input [3:0] r;
    input [3:0] p;
    case ({r, p})
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
        // Rounds 2-11: TODO — fill per Blake2b specification table
        default: sigma = 4'd0;
    endcase
endfunction

// ---------------------------------------------------------------------------
// Working state vector v[0..15] (16 × 64-bit)
// ---------------------------------------------------------------------------
reg [63:0] v0,  v1,  v2,  v3,  v4,  v5,  v6,  v7;
reg [63:0] v8,  v9,  v10, v11, v12, v13, v14, v15;
reg [63:0] h0,  h1,  h2,  h3,  h4,  h5,  h6,  h7;
reg [63:0] m0,  m1,  m2,  m3,  m4,  m5,  m6,  m7;
reg [63:0] m8,  m9,  m10, m11, m12, m13, m14, m15;

// Round counter (0..11) and G-call step within round (0..7)
reg [3:0] round;
reg [2:0] step;
reg       busy;

// ---------------------------------------------------------------------------
// G-function: a = v[a_idx] etc. — combinational for one G-call per cycle
// G(va, vb, vc, vd, mx, my):
//   va = va + vb + mx
//   vd = ror64(vd ^ va, 32)
//   vc = vc + vd
//   vb = ror64(vb ^ vc, 24)
//   va = va + vb + my
//   vd = ror64(vd ^ va, 16)
//   vc = vc + vd
//   vb = ror64(vb ^ vc, 63)
// ---------------------------------------------------------------------------
reg [63:0] gva_in, gvb_in, gvc_in, gvd_in, gmx, gmy;
wire [63:0] gva_out, gvb_out, gvc_out, gvd_out;

// G-function intermediate signals (Verilog-2001: no bit-select on expressions)
wire [63:0] g_t0_a    = gva_in + gvb_in + gmx;
wire [63:0] g_t0_d_xr = gvd_in ^ g_t0_a;
wire [63:0] g_t0_d    = {g_t0_d_xr[31:0], g_t0_d_xr[63:32]};      // ror32
wire [63:0] g_t0_c    = gvc_in + g_t0_d;
wire [63:0] g_t0_b_xr = gvb_in ^ g_t0_c;
wire [63:0] g_t0_b    = {g_t0_b_xr[23:0], g_t0_b_xr[63:24]};      // ror24
wire [63:0] g_t1_a    = g_t0_a + g_t0_b + gmy;
wire [63:0] g_t1_d_xr = g_t0_d ^ g_t1_a;
wire [63:0] g_t1_d    = {g_t1_d_xr[47:0], g_t1_d_xr[63:48]};      // ror16
wire [63:0] g_t1_c    = g_t0_c + g_t1_d;
wire [63:0] g_t1_b_xr = g_t0_b ^ g_t1_c;
wire [63:0] g_t1_b    = {g_t1_b_xr[0], g_t1_b_xr[63:1]};          // ror63

assign gva_out = g_t1_a;
assign gvb_out = g_t1_b;
assign gvc_out = g_t1_c;
assign gvd_out = g_t1_d;

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        {v0,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15} <= {16{64'b0}};
        {h0,h1,h2,h3,h4,h5,h6,h7}                                 <= {8{64'b0}};
        {m0,m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,m13,m14,m15} <= {16{64'b0}};
        {gva_in,gvb_in,gvc_in,gvd_in,gmx,gmy}                     <= {6{64'b0}};
        round  <= 4'd0;
        step   <= 3'd0;
        busy   <= 1'b0;
        h_out  <= 512'b0;
        done   <= 1'b0;
    end else begin
        done <= 1'b0;

        if (start) begin
            // Latch h_in
            h0 <= h_in[ 63:  0]; h1 <= h_in[127: 64];
            h2 <= h_in[191:128]; h3 <= h_in[255:192];
            h4 <= h_in[319:256]; h5 <= h_in[383:320];
            h6 <= h_in[447:384]; h7 <= h_in[511:448];
            // Latch message words
            m0  <= msg_block[ 63:  0]; m1  <= msg_block[127: 64];
            m2  <= msg_block[191:128]; m3  <= msg_block[255:192];
            m4  <= msg_block[319:256]; m5  <= msg_block[383:320];
            m6  <= msg_block[447:384]; m7  <= msg_block[511:448];
            m8  <= msg_block[575:512]; m9  <= msg_block[639:576];
            m10 <= msg_block[703:640]; m11 <= msg_block[767:704];
            m12 <= msg_block[831:768]; m13 <= msg_block[895:832];
            m14 <= msg_block[959:896]; m15 <= msg_block[1023:960];
            // Init working vector v[0..7] = h[0..7]
            v0  <= h_in[ 63:  0]; v1  <= h_in[127: 64];
            v2  <= h_in[191:128]; v3  <= h_in[255:192];
            v4  <= h_in[319:256]; v5  <= h_in[383:320];
            v6  <= h_in[447:384]; v7  <= h_in[511:448];
            // v[8..15] = IV; v12 ^= t0; v13 ^= t1; v14 ^= finalization
            v8  <= IV0; v9  <= IV1; v10 <= IV2; v11 <= IV3;
            v12 <= IV4 ^ byte_count[63:0];
            v13 <= IV5 ^ byte_count[127:64];
            v14 <= last_block ? (IV6 ^ 64'hffffffffffffffff) : IV6;
            v15 <= IV7;
            round <= 4'd0;
            step  <= 3'd0;
            busy  <= 1'b1;
        end else if (busy) begin
            // TODO: Implement G-function dispatch for each step within round
            // 8 G-calls per round (column + diagonal mixing)
            // Placeholder: advance counters without real computation
            if (step == 3'd7) begin
                step  <= 3'd0;
                if (round == 4'd11) begin
                    // Finalize: h_out[i] = h[i] ^ v[i] ^ v[i+8]
                    // TODO: Replace with correct XOR finalization
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    h_out <= {
                        h7 ^ v7 ^ v15, h6 ^ v6 ^ v14,
                        h5 ^ v5 ^ v13, h4 ^ v4 ^ v12,
                        h3 ^ v3 ^ v11, h2 ^ v2 ^ v10,
                        h1 ^ v1 ^ v9,  h0 ^ v0 ^ v8
                    };
                end else begin
                    round <= round + 4'd1;
                end
            end else begin
                step <= step + 3'd1;
            end
        end
    end
end

endmodule
