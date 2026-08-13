// =============================================================================
// argon2_fill.v — Argon2d Cache Fill
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX derives the Cache from the key (seed) with Argon2d
// (RFC 9106 / reference implementation, version 0x13):
//
//   lanes  = RANDOMX_ARGON_LANES      = 1
//   t_cost = RANDOMX_ARGON_ITERATIONS = 3
//   m_cost = RANDOMX_ARGON_MEMORY     = 262144 blocks of 1 KiB (256 MiB)
//   salt   = RANDOMX_ARGON_SALT       = "RandomX\x03"
//   tag    = unused (RandomX keeps the filled memory, no final tag)
//
// Implemented here:
//   1. H0 = Blake2b-512(LE32(lanes) || LE32(tagLen) || LE32(m) || LE32(t) ||
//                       LE32(version) || LE32(type=Argon2d) ||
//                       LE32(|K|) || K || LE32(|S|) || S || LE32(0) || LE32(0))
//   2. B[0] = H'(1024, H0 || LE32(0) || LE32(lane))
//      B[1] = H'(1024, H0 || LE32(1) || LE32(lane))
//      where H' is the Argon2 variable-length hash built from Blake2b-512.
//   3. For every remaining block: data-dependent (Argon2d) reference block
//      selection from J1 = LE32 word 0 of the previous block, followed by the
//      compression function G:
//         R = B[i-1] ^ B[ref]
//         Q = P applied to the eight 128-byte rows of R
//         Z = P applied to the eight 128-byte columns of Q
//         B[i] = Z ^ R              (first pass)
//         B[i] = Z ^ R ^ B[i]       (later passes, XOR mode of version 0x13)
//      P is the Argon2 BlaMka permutation (Blake2b round with the
//      64-bit multiply-add G_B function).
//   4. Multi-pass support (t > 1) including the XOR mode.
//
// The cost parameters are module parameters: ARGON_M (requested m_cost, which
// is normalised exactly like the reference implementation), ARGON_T (any
// number of passes) and KEY_BYTES (width of the key port; H0 is hashed in
// 128-byte Blake2b blocks so keys longer than 80 bytes work as well).
//
// Interfaces:
//   - One shared Blake2b-512 core (blake2b_core) driven through the b2b_* pins.
//   - A 1 KiB-wide cache memory port (read + write) towards the external
//     cache storage (URAM/HBM). Blocks are addressed by block index.
//
// Throughput: one block requires 16 permutation rounds × 2 half-rounds
// (= 32 cycles) plus the memory accesses.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module argon2_fill #(
    // Argon2d cost parameters. The defaults follow RandomX
    // (RANDOMX_ARGON_MEMORY / RANDOMX_ARGON_ITERATIONS). Reduce ARGON_M
    // through a parameter override (not through `ifdef`) when simulating.
    parameter ARGON_M = 262144,     // memory blocks (1 KiB each) = 256 MiB
    parameter ARGON_T = 3,          // passes
    // Widest key accepted by the `key` port. The Argon2 key (RandomX seed) may
    // be of any length; H0 is hashed block by block, so this only bounds the
    // width of the input port, not the algorithm.
    parameter KEY_BYTES = 64
) (
    input  wire          clk,
    input  wire          rst_n,

    // Start: begin Argon2d cache fill from key
    input  wire          start,

    // Key input (seed), little-endian byte packed: byte k is key[8*k +: 8]
    input  wire [KEY_BYTES*8-1:0] key,
    input  wire [15:0]   key_len,  // key length in bytes (0..KEY_BYTES)

    // Cache write interface (to URAM/BRAM/HBM cache storage)
    output reg           cache_wr_en,
    output reg  [31:0]   cache_wr_addr,  // block address (1024-byte blocks)
    output reg  [8191:0] cache_wr_data,  // one 1 KiB block
    input  wire          cache_wr_rdy,   // ready to accept write

    // Cache read interface (previous / reference / current block fetch)
    output reg           cache_rd_en,
    output reg  [31:0]   cache_rd_addr,
    input  wire [8191:0] cache_rd_data,
    input  wire          cache_rd_valid,

    // Done pulse — cache fully filled
    output reg           done,

    // Blake2b interface (single shared core instance)
    output reg           b2b_start,
    output reg           b2b_init,      // let the core generate the Blake2b IV
    output reg  [1023:0] b2b_msg,
    output reg  [127:0]  b2b_byte_cnt,
    output reg  [511:0]  b2b_h_in,
    output reg           b2b_last,
    input  wire [511:0]  b2b_h_out,
    input  wire          b2b_busy,
    input  wire          b2b_done
);

// ---------------------------------------------------------------------------
// Argon2d configuration constants (RandomX defaults)
// ---------------------------------------------------------------------------
localparam ARGON_P       = 1;       // parallelism (lanes)
localparam ARGON_LANES   = ARGON_P;
localparam ARGON_SEGS    = 4;       // segments (slices) per pass per lane
localparam ARGON_VERSION = 32'h13;  // Argon2 version 1.3
localparam ARGON_TYPE    = 32'd0;   // 0 = Argon2d
localparam ARGON_TAGLEN  = 32'd0;   // RandomX requests no final tag
// Memory cost normalisation, exactly as in the Argon2 reference implementation:
// m is raised to 2 * ARGON2_SYNC_POINTS * lanes and then truncated to a
// multiple of the segment count. H0 still hashes the *requested* m_cost.
localparam ARGON_M_MIN   = 2 * ARGON_SEGS * ARGON_LANES;
localparam ARGON_M_RAISED= (ARGON_M < ARGON_M_MIN) ? ARGON_M_MIN : ARGON_M;
localparam SEG_LEN       = ARGON_M_RAISED / (ARGON_LANES * ARGON_SEGS);
localparam ARGON_BLOCKS  = SEG_LEN * ARGON_LANES * ARGON_SEGS;
localparam LANE_LEN      = ARGON_BLOCKS / ARGON_LANES;

// Sized copies of the configuration used inside expressions
localparam [31:0] ARGON_LANES32 = ARGON_LANES;
localparam [31:0] ARGON_M32     = ARGON_M;
localparam [31:0] ARGON_T32     = ARGON_T;
localparam [31:0] LANE_LEN32    = LANE_LEN;
localparam [31:0] SEG_LEN32     = SEG_LEN;
localparam [31:0] LAST_PASS     = ARGON_T - 1;

// RANDOMX_ARGON_SALT = "RandomX\x03" (8 bytes, little-endian packed)
localparam [63:0] ARGON_SALT    = 64'h03586d6f646e6152;
localparam [31:0] ARGON_SALTLEN = 32'd8;

localparam BLOCK_BITS = 8192;       // 1 KiB block
localparam HP_CHUNKS  = 30;         // 30 × 32-byte chunks + one 64-byte tail
localparam [4:0] HP_LAST = HP_CHUNKS;

// H0 pre-hash message layout: 7 × LE32 parameters, the key, then
// LE32(|S|) || S || LE32(|K|) || LE32(|X|). Longer keys simply need more
// Blake2b blocks; the message is hashed 128 bytes at a time.
localparam H0_PRE_BYTES  = 28;
localparam H0_TAIL_BYTES = 20;      // 4 + 8 (salt) + 4 + 4
localparam H0_MAX_BYTES  = H0_PRE_BYTES + KEY_BYTES + H0_TAIL_BYTES;
localparam H0_BLOCKS     = (H0_MAX_BYTES + 127) / 128;
localparam H0_MSG_BITS   = H0_BLOCKS * 1024;

// FSM states
localparam ST_IDLE     = 4'd0;
localparam ST_H0       = 4'd1;   // H0 = Blake2b(params || key || salt)
localparam ST_HP_FIRST = 4'd2;   // H'(1024, H0 || LE32(i) || LE32(lane)), V1
localparam ST_HP_NEXT  = 4'd3;   // H' chain V2..V31
localparam ST_HP_WRITE = 4'd4;   // write B[0] / B[1]
localparam ST_RD_PREV  = 4'd5;   // fetch B[i-1]
localparam ST_RD_REF   = 4'd6;   // fetch B[ref]
localparam ST_RD_CUR   = 4'd7;   // fetch B[i] (XOR mode, pass > 0)
localparam ST_ROUNDS   = 4'd8;   // 8 row + 8 column permutation rounds
localparam ST_WRITE    = 4'd9;   // write B[i]
localparam ST_H0_NEXT  = 4'd11;  // further H0 message blocks (long keys)
localparam ST_DONE     = 4'd10;

reg [3:0]  state;
reg [31:0] block_idx;   // current block index (0..ARGON_BLOCKS-1)
reg [31:0] pass_cnt;    // current pass (0..t-1)
reg [1:0]  seg_cnt;     // current slice (0..3)
reg [31:0] seg_idx;     // index inside the current slice
reg [31:0] ref_idx;     // selected reference block index
reg [15:0] h0_blk;      // H0 message block counter
reg [4:0]  hp_cnt;      // H' chunk counter (0..30)
reg        hp_blk;      // which initial block is being expanded (0 or 1)

// H0 (first Blake2b output)
reg [511:0] h0;

// Working block (R → Q → Z) and the XOR accumulator (R, plus B[i] in XOR mode)
reg [BLOCK_BITS-1:0] work;
reg [BLOCK_BITS-1:0] xacc;

// Permutation round counter: 0..7 rows, 8..15 columns; half = G_B group
reg [3:0] rnd;
reg       half;

integer k;

// ---------------------------------------------------------------------------
// Argon2 G_B function (BlaMka): Blake2b round core with 64-bit multiply-add.
// Returns {d, c, b, a}.
// ---------------------------------------------------------------------------
function [63:0] rotr64;
    input [63:0] x;
    input [6:0]  n;
    begin
        rotr64 = (x >> n) | (x << (7'd64 - n));
    end
endfunction

function [255:0] gb;
    input [63:0] a_in;
    input [63:0] b_in;
    input [63:0] c_in;
    input [63:0] d_in;
    reg [63:0] a, b, c, d, mul;
    begin
        a = a_in; b = b_in; c = c_in; d = d_in;
        mul = a[31:0] * b[31:0];
        a   = a + b + {mul[62:0], 1'b0};
        d   = rotr64(d ^ a, 7'd32);          // rotr64(d ^ a, 32)
        mul = c[31:0] * d[31:0];
        c   = c + d + {mul[62:0], 1'b0};
        b   = rotr64(b ^ c, 7'd24);          // rotr64(b ^ c, 24)
        mul = a[31:0] * b[31:0];
        a   = a + b + {mul[62:0], 1'b0};
        d   = rotr64(d ^ a, 7'd16);          // rotr64(d ^ a, 16)
        mul = c[31:0] * d[31:0];
        c   = c + d + {mul[62:0], 1'b0};
        b   = rotr64(b ^ c, 7'd63);          // rotr64(b ^ c, 63)
        gb  = {d, c, b, a};
    end
endfunction

// ---------------------------------------------------------------------------
// One half of the Argon2 permutation P over 16 × 64-bit words.
//   half = 0 : G_B(v0,v4,v8,v12) G_B(v1,v5,v9,v13)
//              G_B(v2,v6,v10,v14) G_B(v3,v7,v11,v15)
//   half = 1 : G_B(v0,v5,v10,v15) G_B(v1,v6,v11,v12)
//              G_B(v2,v7,v8,v13)  G_B(v3,v4,v9,v14)
// ---------------------------------------------------------------------------
function [1023:0] p_half;
    input [1023:0] vin;
    input          sel;
    reg [63:0] v0, v1, v2,  v3,  v4,  v5,  v6,  v7;
    reg [63:0] v8, v9, v10, v11, v12, v13, v14, v15;
    reg [255:0] r0, r1, r2, r3;
    begin
        v0  = vin[  0 +: 64]; v1  = vin[ 64 +: 64];
        v2  = vin[128 +: 64]; v3  = vin[192 +: 64];
        v4  = vin[256 +: 64]; v5  = vin[320 +: 64];
        v6  = vin[384 +: 64]; v7  = vin[448 +: 64];
        v8  = vin[512 +: 64]; v9  = vin[576 +: 64];
        v10 = vin[640 +: 64]; v11 = vin[704 +: 64];
        v12 = vin[768 +: 64]; v13 = vin[832 +: 64];
        v14 = vin[896 +: 64]; v15 = vin[960 +: 64];

        if (!sel) begin
            r0 = gb(v0, v4, v8,  v12);
            r1 = gb(v1, v5, v9,  v13);
            r2 = gb(v2, v6, v10, v14);
            r3 = gb(v3, v7, v11, v15);
            v0  = r0[ 63:  0]; v4  = r0[127: 64];
            v8  = r0[191:128]; v12 = r0[255:192];
            v1  = r1[ 63:  0]; v5  = r1[127: 64];
            v9  = r1[191:128]; v13 = r1[255:192];
            v2  = r2[ 63:  0]; v6  = r2[127: 64];
            v10 = r2[191:128]; v14 = r2[255:192];
            v3  = r3[ 63:  0]; v7  = r3[127: 64];
            v11 = r3[191:128]; v15 = r3[255:192];
        end else begin
            r0 = gb(v0, v5, v10, v15);
            r1 = gb(v1, v6, v11, v12);
            r2 = gb(v2, v7, v8,  v13);
            r3 = gb(v3, v4, v9,  v14);
            v0  = r0[ 63:  0]; v5  = r0[127: 64];
            v10 = r0[191:128]; v15 = r0[255:192];
            v1  = r1[ 63:  0]; v6  = r1[127: 64];
            v11 = r1[191:128]; v12 = r1[255:192];
            v2  = r2[ 63:  0]; v7  = r2[127: 64];
            v8  = r2[191:128]; v13 = r2[255:192];
            v3  = r3[ 63:  0]; v4  = r3[127: 64];
            v9  = r3[191:128]; v14 = r3[255:192];
        end

        p_half = {v15, v14, v13, v12, v11, v10, v9, v8,
                  v7,  v6,  v5,  v4,  v3,  v2,  v1, v0};
    end
endfunction

// ---------------------------------------------------------------------------
// Round data gathering / scattering.
// Rounds 0..7  : row    r    → the eight 128-bit chunks are contiguous
//                              (bit offset r*1024 + 128*k)
// Rounds 8..15 : column c    → chunks are strided by one row
//                              (bit offset 128*c + 1024*k)
// ---------------------------------------------------------------------------
wire [2:0] rnd_grp = rnd[2:0];                 // row or column number
wire [12:0] chunk_base = rnd[3] ? {3'b0, rnd_grp, 7'b0}   // 128 * column
                                : {rnd_grp, 10'b0};      // 1024 * row
wire [12:0] chunk_step = rnd[3] ? 13'd1024 : 13'd128;

reg  [1023:0] p_in;
wire [1023:0] p_out = p_half(p_in, half);
reg  [BLOCK_BITS-1:0] work_nxt;

always @* begin
    p_in = 1024'b0;
    for (k = 0; k < 8; k = k + 1)
        p_in[k*128 +: 128] = work[(chunk_base + chunk_step * k[12:0]) +: 128];

    work_nxt = work;
    for (k = 0; k < 8; k = k + 1)
        work_nxt[(chunk_base + chunk_step * k[12:0]) +: 128] = p_out[k*128 +: 128];
end

// ---------------------------------------------------------------------------
// Argon2d reference block index (single lane ⇒ the reference lane is our own).
// J1 is the first 32-bit little-endian word of the previous block.
// ---------------------------------------------------------------------------
wire [31:0] j1 = cache_rd_data[31:0];
wire [63:0] j1_sq  = j1 * j1;
wire [31:0] rel_x  = j1_sq[63:32];
// reference area size (blocks that may be referenced)
wire [31:0] ref_area = (pass_cnt == 32'd0)
                     ? (block_idx - 32'd1)
                     : (LANE_LEN32 - SEG_LEN32 + seg_idx - 32'd1);
wire [63:0] ref_mul  = ref_area * rel_x;
wire [31:0] rel_pos  = ref_area - 32'd1 - ref_mul[63:32];
// first block of the reference area
wire [31:0] start_pos = (pass_cnt == 32'd0 || seg_cnt == 2'd3)
                      ? 32'd0
                      : (({30'b0, seg_cnt} + 32'd1) * SEG_LEN32);
wire [31:0] abs_pos   = start_pos + rel_pos;
wire [31:0] ref_next  = (abs_pos >= LANE_LEN32) ? (abs_pos - LANE_LEN32)
                                                    : abs_pos;

// Previous block index, wrapping to the last block of the lane at i == 0
wire [31:0] prev_idx = (block_idx == 32'd0) ? (LANE_LEN32 - 32'd1)
                                            : (block_idx - 32'd1);

// ---------------------------------------------------------------------------
// H0 message: Argon2 parameter encoding, key of arbitrary length, salt tail.
// The message is hashed in 128-byte Blake2b blocks (one block for keys of up
// to 80 bytes, more for longer keys).
// ---------------------------------------------------------------------------
wire [31:0]  key_bits  = {16'b0, key_len} << 3;        // key_len * 8
reg  [KEY_BYTES*8-1:0] key_mask;
wire [31:0]  tail_off  = 32'd224 + key_bits;           // 28 bytes + key
reg [H0_MSG_BITS-1:0] h0_msg;

always @* begin
    for (k = 0; k < KEY_BYTES; k = k + 1)
        key_mask[k*8 +: 8] = (k[15:0] < key_len) ? 8'hFF : 8'h00;
end

always @* begin
    h0_msg = {H0_MSG_BITS{1'b0}};
    h0_msg[  0 +: 32] = ARGON_LANES32;
    h0_msg[ 32 +: 32] = ARGON_TAGLEN;
    h0_msg[ 64 +: 32] = ARGON_M32;
    h0_msg[ 96 +: 32] = ARGON_T32;
    h0_msg[128 +: 32] = ARGON_VERSION;
    h0_msg[160 +: 32] = ARGON_TYPE;
    h0_msg[192 +: 32] = {16'b0, key_len};
    h0_msg[224 +: KEY_BYTES*8] = key & key_mask;
    h0_msg[tail_off +: 32] = ARGON_SALTLEN;              // LE32(|S|)
    h0_msg[(tail_off + 32'd32) +: 64] = ARGON_SALT;      // S
    h0_msg[(tail_off + 32'd96) +: 32] = 32'd0;           // LE32(|K|) = 0 (secret)
    h0_msg[(tail_off + 32'd128) +: 32] = 32'd0;          // LE32(|X|) = 0 (assoc.)
end

// Total H0 message length and the number of 128-byte blocks it occupies
wire [31:0] h0_len     = {16'b0, key_len} + 32'd48;   // 48 + key_len bytes
wire [31:0] h0_nblocks = (h0_len + 32'd127) >> 7;
// Bytes hashed after the current block (capped by the message length)
wire [31:0] h0_blk_end = ({16'b0, h0_blk} + 32'd1) << 7;
wire [31:0] h0_cnt_nxt = (h0_blk_end > h0_len) ? h0_len : h0_blk_end;
wire        h0_is_last = ({16'b0, h0_blk} + 32'd1) >= h0_nblocks;
// Current 128-byte slice of the H0 message (constant indices keep the
// selection width-safe for any KEY_BYTES)
reg [1023:0] h0_blk_msg;
always @* begin
    h0_blk_msg = 1024'b0;
    for (k = 0; k < H0_BLOCKS; k = k + 1)
        if (k[15:0] == h0_blk)
            h0_blk_msg = h0_msg[k*1024 +: 1024];
end

// H'(1024, H0 || LE32(block) || LE32(lane)) first input block: 76 bytes
wire [1023:0] hp_msg = {{416{1'b0}}, 32'd0, {31'b0, hp_blk}, h0, 32'd1024};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= ST_IDLE;
        block_idx      <= 32'd0;
        pass_cnt       <= 32'd0;
        seg_cnt        <= 2'd0;
        seg_idx        <= 32'd0;
        ref_idx        <= 32'd0;
        hp_cnt         <= 5'd0;
        h0_blk         <= 16'd0;
        hp_blk         <= 1'b0;
        rnd            <= 4'd0;
        half           <= 1'b0;
        done           <= 1'b0;
        cache_wr_en    <= 1'b0;
        cache_wr_addr  <= 32'b0;
        cache_wr_data  <= {BLOCK_BITS{1'b0}};
        cache_rd_en    <= 1'b0;
        cache_rd_addr  <= 32'b0;
        b2b_start      <= 1'b0;
        b2b_init       <= 1'b0;
        b2b_msg        <= 1024'b0;
        b2b_byte_cnt   <= 128'b0;
        b2b_h_in       <= 512'b0;
        b2b_last       <= 1'b0;
        h0             <= 512'b0;
        work           <= {BLOCK_BITS{1'b0}};
        xacc           <= {BLOCK_BITS{1'b0}};
    end else begin
        done        <= 1'b0;
        b2b_start   <= 1'b0;

        case (state)
            ST_IDLE: begin
                cache_wr_en <= 1'b0;
                cache_rd_en <= 1'b0;
                if (start && !b2b_busy) begin
                    // H0 = Blake2b(params || key || salt), first message block
                    b2b_msg      <= h0_blk_msg;
                    b2b_byte_cnt <= {96'b0, h0_cnt_nxt};
                    b2b_h_in     <= 512'b0;   // unused: core starts from its own IV
                    b2b_init     <= 1'b1;     // IV generated inside blake2b_core
                    b2b_last     <= h0_is_last;
                    b2b_start    <= 1'b1;
                    block_idx    <= 32'd0;
                    pass_cnt     <= 32'd0;
                    seg_cnt      <= 2'd0;
                    seg_idx      <= 32'd0;
                    hp_blk       <= 1'b0;
                    h0_blk       <= 16'd0;
                    state        <= ST_H0;
                end
            end

            ST_H0: begin
                if (b2b_done) begin
                    if (h0_is_last) begin
                        h0     <= b2b_h_out;
                        h0_blk <= 16'd0;
                        state  <= ST_HP_FIRST;
                    end else begin
                        // Chain the remaining message blocks of a long key
                        h0_blk <= h0_blk + 16'd1;
                        state  <= ST_H0_NEXT;
                    end
                end
            end

            ST_H0_NEXT: begin
                // h0_blk has advanced: issue the next 128-byte message block
                b2b_msg      <= h0_blk_msg;
                b2b_byte_cnt <= {96'b0, h0_cnt_nxt};
                b2b_h_in     <= b2b_h_out;
                b2b_init     <= 1'b0;
                b2b_last     <= h0_is_last;
                b2b_start    <= 1'b1;
                state        <= ST_H0;
            end

            ST_HP_FIRST: begin
                // V1 = Blake2b(LE32(1024) || H0 || LE32(i) || LE32(lane))
                if (b2b_done) begin
                    work[0 +: 256] <= b2b_h_out[255:0];
                    b2b_msg        <= {{512{1'b0}}, b2b_h_out};
                    b2b_byte_cnt   <= 128'd64;
                    b2b_init       <= 1'b1;
                    b2b_last       <= 1'b1;
                    b2b_start      <= 1'b1;
                    hp_cnt         <= 5'd1;
                    state          <= ST_HP_NEXT;
                end else if (!b2b_start && !b2b_busy) begin
                    b2b_msg      <= hp_msg;
                    b2b_byte_cnt <= 128'd76;
                    b2b_init     <= 1'b1;
                    b2b_last     <= 1'b1;
                    b2b_start    <= 1'b1;
                    hp_cnt       <= 5'd0;
                end
            end

            ST_HP_NEXT: begin
                // V2..V31: 32 bytes of each of V2..V30, then all 64 bytes of V31
                if (b2b_done) begin
                    if (hp_cnt == HP_LAST) begin
                        work[HP_CHUNKS*256 +: 512] <= b2b_h_out;
                        state <= ST_HP_WRITE;
                    end else begin
                        work[{hp_cnt, 8'b0} +: 256] <= b2b_h_out[255:0];
                        b2b_msg      <= {{512{1'b0}}, b2b_h_out};
                        b2b_byte_cnt <= 128'd64;
                        b2b_init     <= 1'b1;
                        b2b_last     <= 1'b1;
                        b2b_start    <= 1'b1;
                        hp_cnt       <= hp_cnt + 5'd1;
                    end
                end
            end

            ST_HP_WRITE: begin
                cache_wr_en   <= 1'b1;
                cache_wr_addr <= {31'b0, hp_blk};
                cache_wr_data <= work;
                if (cache_wr_rdy && cache_wr_en) begin
                    cache_wr_en <= 1'b0;
                    if (hp_blk == 1'b0) begin
                        hp_blk <= 1'b1;
                        state  <= ST_HP_FIRST;
                    end else begin
                        // Pass 0 starts at block 2 of slice 0
                        block_idx <= 32'd2;
                        seg_cnt   <= 2'd0;
                        seg_idx   <= 32'd2;
                        state     <= ST_RD_PREV;
                    end
                end
            end

            ST_RD_PREV: begin
                cache_rd_en   <= 1'b1;
                cache_rd_addr <= prev_idx;
                if (cache_rd_en && cache_rd_valid) begin
                    cache_rd_en <= 1'b0;
                    xacc        <= cache_rd_data;   // keep B[i-1] for the XOR
                    ref_idx     <= ref_next;
                    state       <= ST_RD_REF;
                end
            end

            ST_RD_REF: begin
                cache_rd_en   <= 1'b1;
                cache_rd_addr <= ref_idx;
                if (cache_rd_en && cache_rd_valid) begin
                    cache_rd_en <= 1'b0;
                    work        <= xacc ^ cache_rd_data;   // R = B[i-1] ^ B[ref]
                    xacc        <= xacc ^ cache_rd_data;
                    rnd         <= 4'd0;
                    half        <= 1'b0;
                    state       <= (pass_cnt == 32'd0) ? ST_ROUNDS : ST_RD_CUR;
                end
            end

            ST_RD_CUR: begin
                // XOR mode (version 0x13, pass > 0): B[i] = Z ^ R ^ B[i]
                cache_rd_en   <= 1'b1;
                cache_rd_addr <= block_idx;
                if (cache_rd_en && cache_rd_valid) begin
                    cache_rd_en <= 1'b0;
                    xacc        <= xacc ^ cache_rd_data;
                    state       <= ST_ROUNDS;
                end
            end

            ST_ROUNDS: begin
                work <= work_nxt;
                half <= ~half;
                if (half) begin
                    rnd <= rnd + 4'd1;
                    if (rnd == 4'd15)
                        state <= ST_WRITE;
                end
            end

            ST_WRITE: begin
                cache_wr_en   <= 1'b1;
                cache_wr_addr <= block_idx;
                cache_wr_data <= work ^ xacc;
                if (cache_wr_rdy && cache_wr_en) begin
                    cache_wr_en <= 1'b0;
                    if (block_idx == LANE_LEN32 - 32'd1) begin
                        // End of a pass
                        block_idx <= 32'd0;
                        seg_cnt   <= 2'd0;
                        seg_idx   <= 32'd0;
                        pass_cnt  <= pass_cnt + 32'd1;
                        state     <= (pass_cnt == LAST_PASS) ? ST_DONE
                                                                       : ST_RD_PREV;
                    end else begin
                        block_idx <= block_idx + 32'd1;
                        if (seg_idx == SEG_LEN32 - 32'd1) begin
                            seg_idx <= 32'd0;
                            seg_cnt <= seg_cnt + 2'd1;
                        end else begin
                            seg_idx <= seg_idx + 32'd1;
                        end
                        state <= ST_RD_PREV;
                    end
                end
            end

            ST_DONE: begin
                done  <= 1'b1;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
