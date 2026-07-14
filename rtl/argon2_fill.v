// =============================================================================
// argon2_fill.v — Argon2d Cache Fill Skeleton
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX uses Argon2d to fill the Cache memory from the key (seed).
// Cache size = RANDOMX_ARGON_MEMORY × 1024 bytes = 256 MB (default) — but
// on FPGA this is scaled; the cache required by the VM is typically 256 MB
// stored externally or in HBM.
//
// Argon2d Algorithm Overview (RandomX config: t=3, m=262144, p=1, hash=64):
//   1. H0 = Blake2b(params || key || '') — initial 64-byte hash
//   2. Fill first two 1KB blocks per lane from H0
//   3. For t passes: for each block, compute reference block index (data-dependent)
//      and XOR-compress Blake2b-derived blocks
//
// This skeleton provides:
//   - State machine structure for Argon2d
//   - Blake2b core interface
//   - Cache memory write interface (to external BRAM/URAM or HBM)
//
// TODO: Implement the full Argon2d compress function (G-function, XOR chain).
// TODO: Implement multi-pass (t>1) support.
// TODO: Wire to actual cache memory (HBM or large URAM).
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module argon2_fill (
    input  wire          clk,
    input  wire          rst_n,

    // Start: begin Argon2d cache fill from key
    input  wire          start,

    // Key input (seed, up to 512 bits / 64 bytes)
    input  wire [511:0]  key,
    input  wire [5:0]    key_len,  // key length in bytes (1..64)

    // Cache write interface (to URAM/BRAM cache storage)
    output reg           cache_wr_en,
    output reg  [31:0]   cache_wr_addr,  // block address (1024-byte blocks)
    output reg  [1023:0] cache_wr_data,  // one 1KB block
    input  wire          cache_wr_rdy,   // ready to accept write

    // Done pulse — cache fully filled
    output reg           done,

    // Blake2b interface (single shared core instance)
    output reg           b2b_start,
    output reg  [1023:0] b2b_msg,
    output reg  [127:0]  b2b_byte_cnt,
    output reg  [511:0]  b2b_h_in,
    output reg           b2b_last,
    input  wire [511:0]  b2b_h_out,
    input  wire          b2b_done
);

// ---------------------------------------------------------------------------
// Argon2d configuration constants (RandomX defaults)
// ARGON_M_SIM is reduced for simulation; use `iverilog -DSIMULATION` to enable
// ---------------------------------------------------------------------------
`ifdef SIMULATION
localparam ARGON_T       = 1;       // 1 pass for simulation speed
localparam ARGON_M       = 8;       // 8 blocks only (vs 262144 production)
`else
localparam ARGON_T       = 3;       // passes
localparam ARGON_M       = 262144;  // memory blocks (1 KB each)
`endif
localparam ARGON_P       = 1;       // parallelism
localparam ARGON_LANES   = 1;
localparam ARGON_SEGS    = 4;       // segments per pass per lane
localparam BLOCKS_PER_LANE = ARGON_M / ARGON_P;

// FSM states
localparam ST_IDLE      = 4'd0;
localparam ST_H0        = 4'd1;  // Compute H0 = Blake2b(params||key)
localparam ST_INIT_BLK  = 4'd2;  // Fill B[0] and B[1]
localparam ST_FILL      = 4'd3;  // Main fill loop
localparam ST_COMPRESS  = 4'd4;  // Blake2b-based block compression
localparam ST_WRITE     = 4'd5;  // Write block to cache
localparam ST_DONE      = 4'd6;

reg [3:0]  state;
reg [17:0] block_idx;  // current block index (0..ARGON_M-1)
reg [1:0]  pass_cnt;   // current pass (0..t-1)

// Segment and column counters
reg [1:0]  seg_cnt;    // 0..3 (4 segments per lane)

// H0 (first Blake2b output)
reg [511:0] h0;

// Current block being compressed
reg [1023:0] cur_block;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= ST_IDLE;
        block_idx      <= 18'd0;
        pass_cnt       <= 2'd0;
        seg_cnt        <= 2'd0;
        done           <= 1'b0;
        cache_wr_en    <= 1'b0;
        cache_wr_addr  <= 32'b0;
        cache_wr_data  <= 1024'b0;
        b2b_start      <= 1'b0;
        b2b_msg        <= 1024'b0;
        b2b_byte_cnt   <= 128'b0;
        b2b_h_in       <= 512'b0;
        b2b_last       <= 1'b0;
        h0             <= 512'b0;
        cur_block      <= 1024'b0;
    end else begin
        done        <= 1'b0;
        b2b_start   <= 1'b0;
        cache_wr_en <= 1'b0;

        case (state)
            ST_IDLE: begin
                if (start) begin
                    // Initiate H0 = Blake2b(params || key)
                    // TODO: Pack Argon2d parameter block into msg
                    b2b_msg      <= {key, 512'b0}; // placeholder packing
                    b2b_byte_cnt <= {122'b0, key_len};
                    b2b_h_in     <= 512'b0;        // IV will be set inside blake2b_core
                    b2b_last     <= 1'b1;
                    b2b_start    <= 1'b1;
                    block_idx    <= 18'd0;
                    pass_cnt     <= 2'd0;
                    state        <= ST_H0;
                end
            end

            ST_H0: begin
                if (b2b_done) begin
                    h0    <= b2b_h_out;
                    state <= ST_INIT_BLK;
                end
            end

            ST_INIT_BLK: begin
                // Fill B[0] from H0 || LE32(0) and B[1] from H0 || LE32(1)
                // TODO: Implement proper variable-length hash (Blake2b 1024-bit output)
                // For now: use h0 as seed and set first two blocks
                cache_wr_en   <= 1'b1;
                cache_wr_addr <= {14'b0, block_idx};
                cache_wr_data <= {h0, h0}; // placeholder — needs proper Argon2 init
                if (cache_wr_rdy) begin
                    block_idx <= block_idx + 18'd1;
                    if (block_idx == 18'd1) begin
                        block_idx <= 18'd2;
                        state     <= ST_FILL;
                    end
                end
            end

            ST_FILL: begin
                // TODO: Implement Argon2d reference block selection (data-dependent)
                // J1 and J2 from the previous block determine reference index
                // For skeleton: fill sequentially (NOT spec-correct)
                if (block_idx < ARGON_M) begin
                    // TODO: Compress B[ref] into cur_block using the G function
                    cur_block <= {h0, h0}; // placeholder
                    state     <= ST_COMPRESS;
                end else begin
                    pass_cnt <= pass_cnt + 2'd1;
                    if (pass_cnt == ARGON_T - 1) begin
                        state <= ST_DONE;
                    end else begin
                        block_idx <= 18'd0;
                        state     <= ST_FILL;
                    end
                end
            end

            ST_COMPRESS: begin
                // TODO: Drive Blake2b to compress current block
                // Blake2b here acts as a PRF over the block
                b2b_msg      <= cur_block;
                b2b_h_in     <= h0;
                b2b_byte_cnt <= 128'd1024;
                b2b_last     <= 1'b1;
                b2b_start    <= 1'b1;
                state        <= ST_WRITE;
            end

            ST_WRITE: begin
                if (b2b_done) begin
                    // Write XOR'd result to cache
                    // TODO: XOR with previous block content (Argon2d XOR mode)
                    cache_wr_en   <= 1'b1;
                    cache_wr_addr <= {14'b0, block_idx};
                    cache_wr_data <= {b2b_h_out, b2b_h_out}; // placeholder
                    if (cache_wr_rdy) begin
                        block_idx <= block_idx + 18'd1;
                        state     <= ST_FILL;
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
