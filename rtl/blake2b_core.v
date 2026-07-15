// =============================================================================
// blake2b_core.v — Blake2b-512 hash core skeleton
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Blake2b-512 is used for:
//   - Seed expansion → Cache (Argon2d)
//   - Header hash
//   - Final output hash
//
// Implements the complete compression function F:
//   full sigma message schedule, 12 rounds × 8 G-calls (1 G-call per cycle),
//   finalization h[i] ^= v[i] ^ v[i+8].
// Verified against RFC 7693 test vector (Blake2b-512 of "abc").
//
// Verilog-2001 compliant, no vendor IP.
// =============================================================================

`timescale 1ns/1ps

module blake2b_core (
    input  wire          clk,
    input  wire          rst_n,
    // Control
    input  wire          start,        // Begin compression
    input  wire          last_block,   // Final block flag (sets f0 = ~0)
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
// G-call operand selection per step (column steps 0-3, diagonal steps 4-7):
//   step 0: (v0,v4,v8, v12)   step 4: (v0,v5,v10,v15)
//   step 1: (v1,v5,v9, v13)   step 5: (v1,v6,v11,v12)
//   step 2: (v2,v6,v10,v14)   step 6: (v2,v7,v8, v13)
//   step 3: (v3,v7,v11,v15)   step 7: (v3,v4,v9, v14)
reg [63:0] gva_in, gvb_in, gvc_in, gvd_in, gmx, gmy;
wire [63:0] gva_out, gvb_out, gvc_out, gvd_out;

// Message word selection: mx = m[sigma(round, 2*step)], my = m[sigma(round, 2*step+1)]
wire [3:0] mx_idx = sigma(round, {step, 1'b0});
wire [3:0] my_idx = sigma(round, {step, 1'b1});

function [63:0] msel;
    input [3:0] idx;
    case (idx)
        4'd0:  msel = m0;  4'd1:  msel = m1;  4'd2:  msel = m2;  4'd3:  msel = m3;
        4'd4:  msel = m4;  4'd5:  msel = m5;  4'd6:  msel = m6;  4'd7:  msel = m7;
        4'd8:  msel = m8;  4'd9:  msel = m9;  4'd10: msel = m10; 4'd11: msel = m11;
        4'd12: msel = m12; 4'd13: msel = m13; 4'd14: msel = m14; 4'd15: msel = m15;
    endcase
endfunction

always @(*) begin
    gmx = msel(mx_idx);
    gmy = msel(my_idx);
    case (step)
        3'd0: begin gva_in = v0; gvb_in = v4; gvc_in = v8;  gvd_in = v12; end
        3'd1: begin gva_in = v1; gvb_in = v5; gvc_in = v9;  gvd_in = v13; end
        3'd2: begin gva_in = v2; gvb_in = v6; gvc_in = v10; gvd_in = v14; end
        3'd3: begin gva_in = v3; gvb_in = v7; gvc_in = v11; gvd_in = v15; end
        3'd4: begin gva_in = v0; gvb_in = v5; gvc_in = v10; gvd_in = v15; end
        3'd5: begin gva_in = v1; gvb_in = v6; gvc_in = v11; gvd_in = v12; end
        3'd6: begin gva_in = v2; gvb_in = v7; gvc_in = v8;  gvd_in = v13; end
        3'd7: begin gva_in = v3; gvb_in = v4; gvc_in = v9;  gvd_in = v14; end
    endcase
end

// G-function intermediate signals (Verilog-2001: no bit-select on expressions)
wire [63:0] g_t0_a    = gva_in + gvb_in + gmx;
wire [63:0] g_t0_d_xr = gvd_in ^ g_t0_a;
wire [63:0] g_t0_d    = {g_t0_d_xr[31:0], g_t0_d_xr[63:32]};      // ror32
wire [63:0] g_t0_c    = gvc_in + g_t0_d;
wire [63:0] g_t0_b_xr = gvb_in ^ g_t0_c;
wire [63:0] g_t0_b    = {g_t0_b_xr[23:0], g_t0_b_xr[63:24]};      // ror24
wire [63:0] g_t1_a    = g_t0_a + g_t0_b + gmy;
wire [63:0] g_t1_d_xr = g_t0_d ^ g_t1_a;
wire [63:0] g_t1_d    = {g_t1_d_xr[15:0], g_t1_d_xr[63:16]};      // ror16
wire [63:0] g_t1_c    = g_t0_c + g_t1_d;
wire [63:0] g_t1_b_xr = g_t0_b ^ g_t1_c;
// ror63 = rotate right by 63 = rotate left by 1 = {x[62:0], x[63]}
wire [63:0] g_t1_b    = {g_t1_b_xr[62:0], g_t1_b_xr[63]};         // ror63

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
            // One G-call per cycle: write G outputs back to the selected v regs
            case (step)
                3'd0: begin v0 <= gva_out; v4 <= gvb_out; v8  <= gvc_out; v12 <= gvd_out; end
                3'd1: begin v1 <= gva_out; v5 <= gvb_out; v9  <= gvc_out; v13 <= gvd_out; end
                3'd2: begin v2 <= gva_out; v6 <= gvb_out; v10 <= gvc_out; v14 <= gvd_out; end
                3'd3: begin v3 <= gva_out; v7 <= gvb_out; v11 <= gvc_out; v15 <= gvd_out; end
                3'd4: begin v0 <= gva_out; v5 <= gvb_out; v10 <= gvc_out; v15 <= gvd_out; end
                3'd5: begin v1 <= gva_out; v6 <= gvb_out; v11 <= gvc_out; v12 <= gvd_out; end
                3'd6: begin v2 <= gva_out; v7 <= gvb_out; v8  <= gvc_out; v13 <= gvd_out; end
                3'd7: begin v3 <= gva_out; v4 <= gvb_out; v9  <= gvc_out; v14 <= gvd_out; end
            endcase

            if (step == 3'd7) begin
                step  <= 3'd0;
                if (round == 4'd11) begin
                    // Finalize: h_out[i] = h[i] ^ v[i] ^ v[i+8]
                    // Step 7 updates v3,v4,v9,v14 — use G outputs for those
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    h_out <= {
                        h7 ^ v7      ^ v15,
                        h6 ^ v6      ^ gvd_out,   // v14
                        h5 ^ v5      ^ v13,
                        h4 ^ gvb_out ^ v12,       // v4
                        h3 ^ gva_out ^ v11,       // v3
                        h2 ^ v2      ^ v10,
                        h1 ^ v1      ^ gvc_out,   // v9
                        h0 ^ v0      ^ v8
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
