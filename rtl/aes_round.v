// =============================================================================
// aes_round.v — Single AES round function (combinational)
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Implements both directions of the x86 AES-NI single-round primitives that
// RandomX is defined in terms of:
//
//   dec = 0 (AESENC): ShiftRows -> SubBytes -> MixColumns -> AddRoundKey
//   dec = 1 (AESDEC): InvShiftRows -> InvSubBytes -> InvMixColumns -> AddRoundKey
//
// SubBytes is a byte-wise map and ShiftRows a byte permutation, so they commute
// and are applied here in the (Sub, Shift) order for both directions.
//
// Used by: aes_gen1r.v, aes_gen4r.v, aes_hash1r.v
//
// Verilog-2001 compliant, no vendor IP.
// =============================================================================

`timescale 1ns/1ps

module aes_round (
    // 128-bit input state (AES block, column-major: byte[0] at bits[7:0])
    input  wire [127:0] state_in,
    // 128-bit round key
    input  wire [127:0] round_key,
    // Select: 0 = full round (Sub/Shift + MixColumns + AddRoundKey)
    //         1 = last round  (Sub/Shift + AddRoundKey, no MixColumns)
    input  wire         last_round,
    // Direction: 0 = AESENC (encryption round), 1 = AESDEC (decryption round)
    input  wire         dec,
    // 128-bit output state
    output wire [127:0] state_out
);

// ---------------------------------------------------------------------------
// AES S-box (forward substitution) — 256-entry LUT
// ---------------------------------------------------------------------------
function [7:0] sbox;
    input [7:0] x;
    case (x)
        8'h00: sbox = 8'h63; 8'h01: sbox = 8'h7c; 8'h02: sbox = 8'h77; 8'h03: sbox = 8'h7b;
        8'h04: sbox = 8'hf2; 8'h05: sbox = 8'h6b; 8'h06: sbox = 8'h6f; 8'h07: sbox = 8'hc5;
        8'h08: sbox = 8'h30; 8'h09: sbox = 8'h01; 8'h0a: sbox = 8'h67; 8'h0b: sbox = 8'h2b;
        8'h0c: sbox = 8'hfe; 8'h0d: sbox = 8'hd7; 8'h0e: sbox = 8'hab; 8'h0f: sbox = 8'h76;
        8'h10: sbox = 8'hca; 8'h11: sbox = 8'h82; 8'h12: sbox = 8'hc9; 8'h13: sbox = 8'h7d;
        8'h14: sbox = 8'hfa; 8'h15: sbox = 8'h59; 8'h16: sbox = 8'h47; 8'h17: sbox = 8'hf0;
        8'h18: sbox = 8'had; 8'h19: sbox = 8'hd4; 8'h1a: sbox = 8'ha2; 8'h1b: sbox = 8'haf;
        8'h1c: sbox = 8'h9c; 8'h1d: sbox = 8'ha4; 8'h1e: sbox = 8'h72; 8'h1f: sbox = 8'hc0;
        8'h20: sbox = 8'hb7; 8'h21: sbox = 8'hfd; 8'h22: sbox = 8'h93; 8'h23: sbox = 8'h26;
        8'h24: sbox = 8'h36; 8'h25: sbox = 8'h3f; 8'h26: sbox = 8'hf7; 8'h27: sbox = 8'hcc;
        8'h28: sbox = 8'h34; 8'h29: sbox = 8'ha5; 8'h2a: sbox = 8'he5; 8'h2b: sbox = 8'hf1;
        8'h2c: sbox = 8'h71; 8'h2d: sbox = 8'hd8; 8'h2e: sbox = 8'h31; 8'h2f: sbox = 8'h15;
        8'h30: sbox = 8'h04; 8'h31: sbox = 8'hc7; 8'h32: sbox = 8'h23; 8'h33: sbox = 8'hc3;
        8'h34: sbox = 8'h18; 8'h35: sbox = 8'h96; 8'h36: sbox = 8'h05; 8'h37: sbox = 8'h9a;
        8'h38: sbox = 8'h07; 8'h39: sbox = 8'h12; 8'h3a: sbox = 8'h80; 8'h3b: sbox = 8'he2;
        8'h3c: sbox = 8'heb; 8'h3d: sbox = 8'h27; 8'h3e: sbox = 8'hb2; 8'h3f: sbox = 8'h75;
        8'h40: sbox = 8'h09; 8'h41: sbox = 8'h83; 8'h42: sbox = 8'h2c; 8'h43: sbox = 8'h1a;
        8'h44: sbox = 8'h1b; 8'h45: sbox = 8'h6e; 8'h46: sbox = 8'h5a; 8'h47: sbox = 8'ha0;
        8'h48: sbox = 8'h52; 8'h49: sbox = 8'h3b; 8'h4a: sbox = 8'hd6; 8'h4b: sbox = 8'hb3;
        8'h4c: sbox = 8'h29; 8'h4d: sbox = 8'he3; 8'h4e: sbox = 8'h2f; 8'h4f: sbox = 8'h84;
        8'h50: sbox = 8'h53; 8'h51: sbox = 8'hd1; 8'h52: sbox = 8'h00; 8'h53: sbox = 8'hed;
        8'h54: sbox = 8'h20; 8'h55: sbox = 8'hfc; 8'h56: sbox = 8'hb1; 8'h57: sbox = 8'h5b;
        8'h58: sbox = 8'h6a; 8'h59: sbox = 8'hcb; 8'h5a: sbox = 8'hbe; 8'h5b: sbox = 8'h39;
        8'h5c: sbox = 8'h4a; 8'h5d: sbox = 8'h4c; 8'h5e: sbox = 8'h58; 8'h5f: sbox = 8'hcf;
        8'h60: sbox = 8'hd0; 8'h61: sbox = 8'hef; 8'h62: sbox = 8'haa; 8'h63: sbox = 8'hfb;
        8'h64: sbox = 8'h43; 8'h65: sbox = 8'h4d; 8'h66: sbox = 8'h33; 8'h67: sbox = 8'h85;
        8'h68: sbox = 8'h45; 8'h69: sbox = 8'hf9; 8'h6a: sbox = 8'h02; 8'h6b: sbox = 8'h7f;
        8'h6c: sbox = 8'h50; 8'h6d: sbox = 8'h3c; 8'h6e: sbox = 8'h9f; 8'h6f: sbox = 8'ha8;
        8'h70: sbox = 8'h51; 8'h71: sbox = 8'ha3; 8'h72: sbox = 8'h40; 8'h73: sbox = 8'h8f;
        8'h74: sbox = 8'h92; 8'h75: sbox = 8'h9d; 8'h76: sbox = 8'h38; 8'h77: sbox = 8'hf5;
        8'h78: sbox = 8'hbc; 8'h79: sbox = 8'hb6; 8'h7a: sbox = 8'hda; 8'h7b: sbox = 8'h21;
        8'h7c: sbox = 8'h10; 8'h7d: sbox = 8'hff; 8'h7e: sbox = 8'hf3; 8'h7f: sbox = 8'hd2;
        8'h80: sbox = 8'hcd; 8'h81: sbox = 8'h0c; 8'h82: sbox = 8'h13; 8'h83: sbox = 8'hec;
        8'h84: sbox = 8'h5f; 8'h85: sbox = 8'h97; 8'h86: sbox = 8'h44; 8'h87: sbox = 8'h17;
        8'h88: sbox = 8'hc4; 8'h89: sbox = 8'ha7; 8'h8a: sbox = 8'h7e; 8'h8b: sbox = 8'h3d;
        8'h8c: sbox = 8'h64; 8'h8d: sbox = 8'h5d; 8'h8e: sbox = 8'h19; 8'h8f: sbox = 8'h73;
        8'h90: sbox = 8'h60; 8'h91: sbox = 8'h81; 8'h92: sbox = 8'h4f; 8'h93: sbox = 8'hdc;
        8'h94: sbox = 8'h22; 8'h95: sbox = 8'h2a; 8'h96: sbox = 8'h90; 8'h97: sbox = 8'h88;
        8'h98: sbox = 8'h46; 8'h99: sbox = 8'hee; 8'h9a: sbox = 8'hb8; 8'h9b: sbox = 8'h14;
        8'h9c: sbox = 8'hde; 8'h9d: sbox = 8'h5e; 8'h9e: sbox = 8'h0b; 8'h9f: sbox = 8'hdb;
        8'ha0: sbox = 8'he0; 8'ha1: sbox = 8'h32; 8'ha2: sbox = 8'h3a; 8'ha3: sbox = 8'h0a;
        8'ha4: sbox = 8'h49; 8'ha5: sbox = 8'h06; 8'ha6: sbox = 8'h24; 8'ha7: sbox = 8'h5c;
        8'ha8: sbox = 8'hc2; 8'ha9: sbox = 8'hd3; 8'haa: sbox = 8'hac; 8'hab: sbox = 8'h62;
        8'hac: sbox = 8'h91; 8'had: sbox = 8'h95; 8'hae: sbox = 8'he4; 8'haf: sbox = 8'h79;
        8'hb0: sbox = 8'he7; 8'hb1: sbox = 8'hc8; 8'hb2: sbox = 8'h37; 8'hb3: sbox = 8'h6d;
        8'hb4: sbox = 8'h8d; 8'hb5: sbox = 8'hd5; 8'hb6: sbox = 8'h4e; 8'hb7: sbox = 8'ha9;
        8'hb8: sbox = 8'h6c; 8'hb9: sbox = 8'h56; 8'hba: sbox = 8'hf4; 8'hbb: sbox = 8'hea;
        8'hbc: sbox = 8'h65; 8'hbd: sbox = 8'h7a; 8'hbe: sbox = 8'hae; 8'hbf: sbox = 8'h08;
        8'hc0: sbox = 8'hba; 8'hc1: sbox = 8'h78; 8'hc2: sbox = 8'h25; 8'hc3: sbox = 8'h2e;
        8'hc4: sbox = 8'h1c; 8'hc5: sbox = 8'ha6; 8'hc6: sbox = 8'hb4; 8'hc7: sbox = 8'hc6;
        8'hc8: sbox = 8'he8; 8'hc9: sbox = 8'hdd; 8'hca: sbox = 8'h74; 8'hcb: sbox = 8'h1f;
        8'hcc: sbox = 8'h4b; 8'hcd: sbox = 8'hbd; 8'hce: sbox = 8'h8b; 8'hcf: sbox = 8'h8a;
        8'hd0: sbox = 8'h70; 8'hd1: sbox = 8'h3e; 8'hd2: sbox = 8'hb5; 8'hd3: sbox = 8'h66;
        8'hd4: sbox = 8'h48; 8'hd5: sbox = 8'h03; 8'hd6: sbox = 8'hf6; 8'hd7: sbox = 8'h0e;
        8'hd8: sbox = 8'h61; 8'hd9: sbox = 8'h35; 8'hda: sbox = 8'h57; 8'hdb: sbox = 8'hb9;
        8'hdc: sbox = 8'h86; 8'hdd: sbox = 8'hc1; 8'hde: sbox = 8'h1d; 8'hdf: sbox = 8'h9e;
        8'he0: sbox = 8'he1; 8'he1: sbox = 8'hf8; 8'he2: sbox = 8'h98; 8'he3: sbox = 8'h11;
        8'he4: sbox = 8'h69; 8'he5: sbox = 8'hd9; 8'he6: sbox = 8'h8e; 8'he7: sbox = 8'h94;
        8'he8: sbox = 8'h9b; 8'he9: sbox = 8'h1e; 8'hea: sbox = 8'h87; 8'heb: sbox = 8'he9;
        8'hec: sbox = 8'hce; 8'hed: sbox = 8'h55; 8'hee: sbox = 8'h28; 8'hef: sbox = 8'hdf;
        8'hf0: sbox = 8'h8c; 8'hf1: sbox = 8'ha1; 8'hf2: sbox = 8'h89; 8'hf3: sbox = 8'h0d;
        8'hf4: sbox = 8'hbf; 8'hf5: sbox = 8'he6; 8'hf6: sbox = 8'h42; 8'hf7: sbox = 8'h68;
        8'hf8: sbox = 8'h41; 8'hf9: sbox = 8'h99; 8'hfa: sbox = 8'h2d; 8'hfb: sbox = 8'h0f;
        8'hfc: sbox = 8'hb0; 8'hfd: sbox = 8'h54; 8'hfe: sbox = 8'hbb; 8'hff: sbox = 8'h16;
        default: sbox = 8'h00;
    endcase
endfunction

// ---------------------------------------------------------------------------
// AES inverse S-box (used by the AESDEC round) — 256-entry LUT
// ---------------------------------------------------------------------------
function [7:0] inv_sbox;
    input [7:0] x;
    case (x)
        8'h00: inv_sbox = 8'h52; 8'h01: inv_sbox = 8'h09; 8'h02: inv_sbox = 8'h6a; 8'h03: inv_sbox = 8'hd5;
        8'h04: inv_sbox = 8'h30; 8'h05: inv_sbox = 8'h36; 8'h06: inv_sbox = 8'ha5; 8'h07: inv_sbox = 8'h38;
        8'h08: inv_sbox = 8'hbf; 8'h09: inv_sbox = 8'h40; 8'h0a: inv_sbox = 8'ha3; 8'h0b: inv_sbox = 8'h9e;
        8'h0c: inv_sbox = 8'h81; 8'h0d: inv_sbox = 8'hf3; 8'h0e: inv_sbox = 8'hd7; 8'h0f: inv_sbox = 8'hfb;
        8'h10: inv_sbox = 8'h7c; 8'h11: inv_sbox = 8'he3; 8'h12: inv_sbox = 8'h39; 8'h13: inv_sbox = 8'h82;
        8'h14: inv_sbox = 8'h9b; 8'h15: inv_sbox = 8'h2f; 8'h16: inv_sbox = 8'hff; 8'h17: inv_sbox = 8'h87;
        8'h18: inv_sbox = 8'h34; 8'h19: inv_sbox = 8'h8e; 8'h1a: inv_sbox = 8'h43; 8'h1b: inv_sbox = 8'h44;
        8'h1c: inv_sbox = 8'hc4; 8'h1d: inv_sbox = 8'hde; 8'h1e: inv_sbox = 8'he9; 8'h1f: inv_sbox = 8'hcb;
        8'h20: inv_sbox = 8'h54; 8'h21: inv_sbox = 8'h7b; 8'h22: inv_sbox = 8'h94; 8'h23: inv_sbox = 8'h32;
        8'h24: inv_sbox = 8'ha6; 8'h25: inv_sbox = 8'hc2; 8'h26: inv_sbox = 8'h23; 8'h27: inv_sbox = 8'h3d;
        8'h28: inv_sbox = 8'hee; 8'h29: inv_sbox = 8'h4c; 8'h2a: inv_sbox = 8'h95; 8'h2b: inv_sbox = 8'h0b;
        8'h2c: inv_sbox = 8'h42; 8'h2d: inv_sbox = 8'hfa; 8'h2e: inv_sbox = 8'hc3; 8'h2f: inv_sbox = 8'h4e;
        8'h30: inv_sbox = 8'h08; 8'h31: inv_sbox = 8'h2e; 8'h32: inv_sbox = 8'ha1; 8'h33: inv_sbox = 8'h66;
        8'h34: inv_sbox = 8'h28; 8'h35: inv_sbox = 8'hd9; 8'h36: inv_sbox = 8'h24; 8'h37: inv_sbox = 8'hb2;
        8'h38: inv_sbox = 8'h76; 8'h39: inv_sbox = 8'h5b; 8'h3a: inv_sbox = 8'ha2; 8'h3b: inv_sbox = 8'h49;
        8'h3c: inv_sbox = 8'h6d; 8'h3d: inv_sbox = 8'h8b; 8'h3e: inv_sbox = 8'hd1; 8'h3f: inv_sbox = 8'h25;
        8'h40: inv_sbox = 8'h72; 8'h41: inv_sbox = 8'hf8; 8'h42: inv_sbox = 8'hf6; 8'h43: inv_sbox = 8'h64;
        8'h44: inv_sbox = 8'h86; 8'h45: inv_sbox = 8'h68; 8'h46: inv_sbox = 8'h98; 8'h47: inv_sbox = 8'h16;
        8'h48: inv_sbox = 8'hd4; 8'h49: inv_sbox = 8'ha4; 8'h4a: inv_sbox = 8'h5c; 8'h4b: inv_sbox = 8'hcc;
        8'h4c: inv_sbox = 8'h5d; 8'h4d: inv_sbox = 8'h65; 8'h4e: inv_sbox = 8'hb6; 8'h4f: inv_sbox = 8'h92;
        8'h50: inv_sbox = 8'h6c; 8'h51: inv_sbox = 8'h70; 8'h52: inv_sbox = 8'h48; 8'h53: inv_sbox = 8'h50;
        8'h54: inv_sbox = 8'hfd; 8'h55: inv_sbox = 8'hed; 8'h56: inv_sbox = 8'hb9; 8'h57: inv_sbox = 8'hda;
        8'h58: inv_sbox = 8'h5e; 8'h59: inv_sbox = 8'h15; 8'h5a: inv_sbox = 8'h46; 8'h5b: inv_sbox = 8'h57;
        8'h5c: inv_sbox = 8'ha7; 8'h5d: inv_sbox = 8'h8d; 8'h5e: inv_sbox = 8'h9d; 8'h5f: inv_sbox = 8'h84;
        8'h60: inv_sbox = 8'h90; 8'h61: inv_sbox = 8'hd8; 8'h62: inv_sbox = 8'hab; 8'h63: inv_sbox = 8'h00;
        8'h64: inv_sbox = 8'h8c; 8'h65: inv_sbox = 8'hbc; 8'h66: inv_sbox = 8'hd3; 8'h67: inv_sbox = 8'h0a;
        8'h68: inv_sbox = 8'hf7; 8'h69: inv_sbox = 8'he4; 8'h6a: inv_sbox = 8'h58; 8'h6b: inv_sbox = 8'h05;
        8'h6c: inv_sbox = 8'hb8; 8'h6d: inv_sbox = 8'hb3; 8'h6e: inv_sbox = 8'h45; 8'h6f: inv_sbox = 8'h06;
        8'h70: inv_sbox = 8'hd0; 8'h71: inv_sbox = 8'h2c; 8'h72: inv_sbox = 8'h1e; 8'h73: inv_sbox = 8'h8f;
        8'h74: inv_sbox = 8'hca; 8'h75: inv_sbox = 8'h3f; 8'h76: inv_sbox = 8'h0f; 8'h77: inv_sbox = 8'h02;
        8'h78: inv_sbox = 8'hc1; 8'h79: inv_sbox = 8'haf; 8'h7a: inv_sbox = 8'hbd; 8'h7b: inv_sbox = 8'h03;
        8'h7c: inv_sbox = 8'h01; 8'h7d: inv_sbox = 8'h13; 8'h7e: inv_sbox = 8'h8a; 8'h7f: inv_sbox = 8'h6b;
        8'h80: inv_sbox = 8'h3a; 8'h81: inv_sbox = 8'h91; 8'h82: inv_sbox = 8'h11; 8'h83: inv_sbox = 8'h41;
        8'h84: inv_sbox = 8'h4f; 8'h85: inv_sbox = 8'h67; 8'h86: inv_sbox = 8'hdc; 8'h87: inv_sbox = 8'hea;
        8'h88: inv_sbox = 8'h97; 8'h89: inv_sbox = 8'hf2; 8'h8a: inv_sbox = 8'hcf; 8'h8b: inv_sbox = 8'hce;
        8'h8c: inv_sbox = 8'hf0; 8'h8d: inv_sbox = 8'hb4; 8'h8e: inv_sbox = 8'he6; 8'h8f: inv_sbox = 8'h73;
        8'h90: inv_sbox = 8'h96; 8'h91: inv_sbox = 8'hac; 8'h92: inv_sbox = 8'h74; 8'h93: inv_sbox = 8'h22;
        8'h94: inv_sbox = 8'he7; 8'h95: inv_sbox = 8'had; 8'h96: inv_sbox = 8'h35; 8'h97: inv_sbox = 8'h85;
        8'h98: inv_sbox = 8'he2; 8'h99: inv_sbox = 8'hf9; 8'h9a: inv_sbox = 8'h37; 8'h9b: inv_sbox = 8'he8;
        8'h9c: inv_sbox = 8'h1c; 8'h9d: inv_sbox = 8'h75; 8'h9e: inv_sbox = 8'hdf; 8'h9f: inv_sbox = 8'h6e;
        8'ha0: inv_sbox = 8'h47; 8'ha1: inv_sbox = 8'hf1; 8'ha2: inv_sbox = 8'h1a; 8'ha3: inv_sbox = 8'h71;
        8'ha4: inv_sbox = 8'h1d; 8'ha5: inv_sbox = 8'h29; 8'ha6: inv_sbox = 8'hc5; 8'ha7: inv_sbox = 8'h89;
        8'ha8: inv_sbox = 8'h6f; 8'ha9: inv_sbox = 8'hb7; 8'haa: inv_sbox = 8'h62; 8'hab: inv_sbox = 8'h0e;
        8'hac: inv_sbox = 8'haa; 8'had: inv_sbox = 8'h18; 8'hae: inv_sbox = 8'hbe; 8'haf: inv_sbox = 8'h1b;
        8'hb0: inv_sbox = 8'hfc; 8'hb1: inv_sbox = 8'h56; 8'hb2: inv_sbox = 8'h3e; 8'hb3: inv_sbox = 8'h4b;
        8'hb4: inv_sbox = 8'hc6; 8'hb5: inv_sbox = 8'hd2; 8'hb6: inv_sbox = 8'h79; 8'hb7: inv_sbox = 8'h20;
        8'hb8: inv_sbox = 8'h9a; 8'hb9: inv_sbox = 8'hdb; 8'hba: inv_sbox = 8'hc0; 8'hbb: inv_sbox = 8'hfe;
        8'hbc: inv_sbox = 8'h78; 8'hbd: inv_sbox = 8'hcd; 8'hbe: inv_sbox = 8'h5a; 8'hbf: inv_sbox = 8'hf4;
        8'hc0: inv_sbox = 8'h1f; 8'hc1: inv_sbox = 8'hdd; 8'hc2: inv_sbox = 8'ha8; 8'hc3: inv_sbox = 8'h33;
        8'hc4: inv_sbox = 8'h88; 8'hc5: inv_sbox = 8'h07; 8'hc6: inv_sbox = 8'hc7; 8'hc7: inv_sbox = 8'h31;
        8'hc8: inv_sbox = 8'hb1; 8'hc9: inv_sbox = 8'h12; 8'hca: inv_sbox = 8'h10; 8'hcb: inv_sbox = 8'h59;
        8'hcc: inv_sbox = 8'h27; 8'hcd: inv_sbox = 8'h80; 8'hce: inv_sbox = 8'hec; 8'hcf: inv_sbox = 8'h5f;
        8'hd0: inv_sbox = 8'h60; 8'hd1: inv_sbox = 8'h51; 8'hd2: inv_sbox = 8'h7f; 8'hd3: inv_sbox = 8'ha9;
        8'hd4: inv_sbox = 8'h19; 8'hd5: inv_sbox = 8'hb5; 8'hd6: inv_sbox = 8'h4a; 8'hd7: inv_sbox = 8'h0d;
        8'hd8: inv_sbox = 8'h2d; 8'hd9: inv_sbox = 8'he5; 8'hda: inv_sbox = 8'h7a; 8'hdb: inv_sbox = 8'h9f;
        8'hdc: inv_sbox = 8'h93; 8'hdd: inv_sbox = 8'hc9; 8'hde: inv_sbox = 8'h9c; 8'hdf: inv_sbox = 8'hef;
        8'he0: inv_sbox = 8'ha0; 8'he1: inv_sbox = 8'he0; 8'he2: inv_sbox = 8'h3b; 8'he3: inv_sbox = 8'h4d;
        8'he4: inv_sbox = 8'hae; 8'he5: inv_sbox = 8'h2a; 8'he6: inv_sbox = 8'hf5; 8'he7: inv_sbox = 8'hb0;
        8'he8: inv_sbox = 8'hc8; 8'he9: inv_sbox = 8'heb; 8'hea: inv_sbox = 8'hbb; 8'heb: inv_sbox = 8'h3c;
        8'hec: inv_sbox = 8'h83; 8'hed: inv_sbox = 8'h53; 8'hee: inv_sbox = 8'h99; 8'hef: inv_sbox = 8'h61;
        8'hf0: inv_sbox = 8'h17; 8'hf1: inv_sbox = 8'h2b; 8'hf2: inv_sbox = 8'h04; 8'hf3: inv_sbox = 8'h7e;
        8'hf4: inv_sbox = 8'hba; 8'hf5: inv_sbox = 8'h77; 8'hf6: inv_sbox = 8'hd6; 8'hf7: inv_sbox = 8'h26;
        8'hf8: inv_sbox = 8'he1; 8'hf9: inv_sbox = 8'h69; 8'hfa: inv_sbox = 8'h14; 8'hfb: inv_sbox = 8'h63;
        8'hfc: inv_sbox = 8'h55; 8'hfd: inv_sbox = 8'h21; 8'hfe: inv_sbox = 8'h0c; 8'hff: inv_sbox = 8'h7d;
        default: inv_sbox = 8'h00;
    endcase
endfunction

// ---------------------------------------------------------------------------
// GF(2^8) multiply-by-2 (xtime)
// ---------------------------------------------------------------------------
function [7:0] xtime;
    input [7:0] a;
    xtime = (a[7]) ? ((a << 1) ^ 8'h1b) : (a << 1);
endfunction

// ---------------------------------------------------------------------------
// GF(2^8) multiply
// ---------------------------------------------------------------------------
function [7:0] gf_mul;
    input [7:0] a, b;
    reg [7:0] p, aa;
    integer i;  // integer (32-bit) prevents loop counter wrap-around
    begin
        p  = 8'h00;
        aa = a;
        for (i = 0; i < 8; i = i + 1) begin
            if (b[i]) p = p ^ aa;
            aa = xtime(aa);
        end
        gf_mul = p;
    end
endfunction

// ---------------------------------------------------------------------------
// Step 1: SubBytes / InvSubBytes — apply the (inverse) S-box to all 16 bytes
// ---------------------------------------------------------------------------
wire [7:0] sb [0:15];
genvar gi;
generate
    for (gi = 0; gi < 16; gi = gi + 1) begin : sub_bytes
        assign sb[gi] = dec ? inv_sbox(state_in[gi*8 +: 8])
                            :     sbox(state_in[gi*8 +: 8]);
    end
endgenerate

// ---------------------------------------------------------------------------
// Step 2: ShiftRows / InvShiftRows
// AES state as 4x4 byte matrix (column-major, byte[0] is row0,col0):
//   row 0: bytes  0,  4,  8, 12 - rotate by 0
//   row 1: bytes  1,  5,  9, 13 - rotate left by 1 (right by 1 when dec)
//   row 2: bytes  2,  6, 10, 14 - rotate left by 2 (right by 2 when dec)
//   row 3: bytes  3,  7, 11, 15 - rotate left by 3 (right by 3 when dec)
// ---------------------------------------------------------------------------
wire [7:0] sr [0:15];
// row 0 is never shifted
assign sr[ 0] = sb[ 0];
assign sr[ 4] = sb[ 4];
assign sr[ 8] = sb[ 8];
assign sr[12] = sb[12];
// row 1
assign sr[ 1] = dec ? sb[13] : sb[ 5];
assign sr[ 5] = dec ? sb[ 1] : sb[ 9];
assign sr[ 9] = dec ? sb[ 5] : sb[13];
assign sr[13] = dec ? sb[ 9] : sb[ 1];
// row 2 (rotate by 2 is its own inverse)
assign sr[ 2] = sb[10];
assign sr[ 6] = sb[14];
assign sr[10] = sb[ 2];
assign sr[14] = sb[ 6];
// row 3
assign sr[ 3] = dec ? sb[ 7] : sb[15];
assign sr[ 7] = dec ? sb[11] : sb[ 3];
assign sr[11] = dec ? sb[15] : sb[ 7];
assign sr[15] = dec ? sb[ 3] : sb[11];

// ---------------------------------------------------------------------------
// Step 3: MixColumns / InvMixColumns
//   forward: [2 3 1 1] MDS matrix, inverse: [14 11 13 9]
// ---------------------------------------------------------------------------
wire [7:0] mc [0:15];
genvar col;
generate
    for (col = 0; col < 4; col = col + 1) begin : mix_col
        wire [7:0] b0, b1, b2, b3;
        assign b0 = sr[col*4+0];
        assign b1 = sr[col*4+1];
        assign b2 = sr[col*4+2];
        assign b3 = sr[col*4+3];
        assign mc[col*4+0] = dec
            ? (gf_mul(8'h0e,b0)^gf_mul(8'h0b,b1)^gf_mul(8'h0d,b2)^gf_mul(8'h09,b3))
            : (gf_mul(8'h02,b0)^gf_mul(8'h03,b1)^b2^b3);
        assign mc[col*4+1] = dec
            ? (gf_mul(8'h09,b0)^gf_mul(8'h0e,b1)^gf_mul(8'h0b,b2)^gf_mul(8'h0d,b3))
            : (b0^gf_mul(8'h02,b1)^gf_mul(8'h03,b2)^b3);
        assign mc[col*4+2] = dec
            ? (gf_mul(8'h0d,b0)^gf_mul(8'h09,b1)^gf_mul(8'h0e,b2)^gf_mul(8'h0b,b3))
            : (b0^b1^gf_mul(8'h02,b2)^gf_mul(8'h03,b3));
        assign mc[col*4+3] = dec
            ? (gf_mul(8'h0b,b0)^gf_mul(8'h0d,b1)^gf_mul(8'h09,b2)^gf_mul(8'h0e,b3))
            : (gf_mul(8'h03,b0)^b1^b2^gf_mul(8'h02,b3));
    end
endgenerate

// ---------------------------------------------------------------------------
// Step 4: AddRoundKey - XOR with the round key
// MUX between with/without MixColumns based on last_round
// ---------------------------------------------------------------------------
wire [127:0] after_mc;
wire [127:0] after_sr;
genvar bi;
generate
    for (bi = 0; bi < 16; bi = bi + 1) begin : pack_out
        assign after_mc[bi*8 +: 8] = mc[bi];
        assign after_sr[bi*8 +: 8] = sr[bi];
    end
endgenerate

assign state_out = (last_round ? after_sr : after_mc) ^ round_key;

endmodule
