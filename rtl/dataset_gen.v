// =============================================================================
// dataset_gen.v — RandomX Dataset item generator
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Implements `initDatasetItem` (RandomX spec §7.3 / dataset.cpp): every
// 64-byte dataset item is produced from the Argon2d cache with 8 cache
// accesses, each followed by one SuperscalarHash program:
//
//   r0 = (itemNumber + 1) * 6364136223846793005
//   r1..r7 = r0 ^ superscalarAdd1..7
//   registerValue = itemNumber
//   repeat 8 times (i = 0..7):
//       mix           = cache line at (registerValue & (CACHE_LINES-1)) * 64
//       executeSuperscalar(r, prog[i])
//       r[q]         ^= mix[q]                       (q = 0..7)
//       registerValue = r[addressRegister(prog[i])]
//   dataset[itemNumber] = r0..r7 (little endian, 64 bytes)
//
// The 8 SuperscalarHash programs are *not* generated on-chip: `generateSuperscalar`
// (spec §6.1) is a scheduling-model driven generator that is impractical in
// RTL. They are loaded through `prog_wr_*` (program i occupies the 512-word
// window at i × 512 of the shared SuperscalarHash program buffer) together
// with their length and address register (`cfg_wr_*`). With all lengths left
// at their reset value of 0 the SuperscalarHash step degenerates to a NOP and
// dataset items become the plain XOR chain of the cache lines, which keeps the
// datapath exercisable before the host has loaded any programs.
//
// Cache access uses the 1 KiB block port of `cache_hbm_if`: block index =
// line >> 4, and the 64-byte mix block is the (line & 15)-th 512-bit slice.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module dataset_gen #(
    // Number of 64-byte cache lines (RANDOMX_ARGON_MEMORY × 1024 / 64).
    // Must be a power of two so that `& (CACHE_LINES-1)` is the spec's mask.
    parameter CACHE_LINES = 4194304,
    // Number of dataset items to generate (RANDOMX_DATASET_ITEM_COUNT).
    parameter ITEM_COUNT  = 34078720,
    // Number of cache accesses per item (RANDOMX_CACHE_ACCESSES).
    parameter ACCESSES    = 8
) (
    input  wire          clk,
    input  wire          rst_n,

    // Start pulse: generate items 0 .. ITEM_COUNT-1
    input  wire          start,

    // ---- SuperscalarHash program load (before start) ----
    input  wire          prog_wr_en,
    input  wire [11:0]   prog_wr_addr,   // absolute program-buffer index
    input  wire [63:0]   prog_wr_data,
    // Per-program configuration: length and address register
    input  wire          cfg_wr_en,
    input  wire [2:0]    cfg_wr_sel,     // program index 0..7
    input  wire [11:0]   cfg_wr_len,     // instruction count
    input  wire [2:0]    cfg_wr_addr_reg,// address register of the program

    // ---- Cache read port (1 KiB blocks, cache_hbm_if) ----
    output reg           cache_rd_en,
    output reg  [31:0]   cache_rd_addr,  // block index
    input  wire [8191:0] cache_rd_data,
    input  wire          cache_rd_valid,

    // ---- Dataset write port (hbm_dataset_if) ----
    output reg           ds_wr_valid,
    output reg  [31:0]   ds_wr_item_idx,
    output reg  [511:0]  ds_wr_data,
    input  wire          ds_wr_ready,

    // ---- Status ----
    output reg           busy,
    output reg           done        // one-cycle pulse when all items are written
);

// ---------------------------------------------------------------------------
// Spec constants (dataset.cpp)
// ---------------------------------------------------------------------------
localparam [63:0] SS_MUL0 = 64'h5851F42D4C957F2D;
localparam [63:0] SS_ADD1 = 64'h810A978A59F5A1FC;
localparam [63:0] SS_ADD2 = 64'hA77099DF38C2D846;
localparam [63:0] SS_ADD3 = 64'h8126B91CBF22495C;
localparam [63:0] SS_ADD4 = 64'h494D2597179F8A62;
localparam [63:0] SS_ADD5 = 64'h9237EFB9CEAAEC0C;
localparam [63:0] SS_ADD6 = 64'h2F2A56746CE62D78;
localparam [63:0] SS_ADD7 = 64'h84853BF7B62CE54E;

// Cache line mask (spec: CacheSize / CacheLineSize - 1)
localparam [63:0] LINE_MASK   = CACHE_LINES - 1;


// ---------------------------------------------------------------------------
// Per-program configuration
// ---------------------------------------------------------------------------
reg [11:0] prog_len_r  [0:7];
reg [2:0]  prog_areg_r [0:7];

integer c;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (c = 0; c < 8; c = c + 1) begin
            prog_len_r[c]  <= 12'd0;
            prog_areg_r[c] <= 3'd0;
        end
    end else if (cfg_wr_en) begin
        prog_len_r[cfg_wr_sel]  <= cfg_wr_len;
        prog_areg_r[cfg_wr_sel] <= cfg_wr_addr_reg;
    end
end

// ---------------------------------------------------------------------------
// SuperscalarHash execution unit
// ---------------------------------------------------------------------------
reg  [2:0]  acc_idx;                  // current cache access / program index
wire [11:0] ss_prog_base = {acc_idx, 9'b0};   // program i at i × 512
wire [11:0] ss_prog_len  = prog_len_r[acc_idx];

reg         ss_start;
reg  [63:0] rf [0:7];
wire [63:0] ss_out0, ss_out1, ss_out2, ss_out3;
wire [63:0] ss_out4, ss_out5, ss_out6, ss_out7;
wire        ss_done;

superscalar_hash u_ss (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (ss_start),
    .prog_wr_en   (prog_wr_en),
    .prog_wr_addr (prog_wr_addr),
    .prog_wr_data (prog_wr_data),
    .prog_base    (ss_prog_base),
    .prog_len     (ss_prog_len),
    .init_r0      (rf[0]),
    .init_r1      (rf[1]),
    .init_r2      (rf[2]),
    .init_r3      (rf[3]),
    .init_r4      (rf[4]),
    .init_r5      (rf[5]),
    .init_r6      (rf[6]),
    .init_r7      (rf[7]),
    .out_r0       (ss_out0),
    .out_r1       (ss_out1),
    .out_r2       (ss_out2),
    .out_r3       (ss_out3),
    .out_r4       (ss_out4),
    .out_r5       (ss_out5),
    .out_r6       (ss_out6),
    .out_r7       (ss_out7),
    .busy         (),
    .done         (ss_done)
);

// ---------------------------------------------------------------------------
// Item / cache bookkeeping
// ---------------------------------------------------------------------------
reg  [31:0] item_idx;      // dataset item being generated
reg  [63:0] reg_value;     // spec `registerValue` (drives the cache access)
reg  [511:0] mix_block;    // 64-byte cache line of the current access

// Cache line selected by reg_value
wire [63:0] cache_line = reg_value & LINE_MASK;
wire [31:0] cache_blk  = cache_line[35:4];   // 16 lines (64 B) per 1 KiB block
wire [3:0]  cache_sel  = cache_line[3:0];

// 64-byte slice of the 1 KiB block returned by cache_hbm_if
wire [511:0] mix_slice = cache_rd_data[{cache_sel, 9'b0} +: 512];

// (item_idx + 1) * SS_MUL0 — a single 64×64 multiply, mapped to DSP slices.
wire [63:0] item_seed = ({32'b0, item_idx} + 64'd1) * SS_MUL0;

// ---------------------------------------------------------------------------
// Control FSM
// ---------------------------------------------------------------------------
localparam ST_IDLE     = 3'd0;
localparam ST_ITEM_INIT= 3'd1;
localparam ST_MIX_REQ  = 3'd2;
localparam ST_SS_RUN   = 3'd3;
localparam ST_MIX_XOR  = 3'd4;
localparam ST_WRITE    = 3'd5;
localparam ST_DONE     = 3'd6;

reg [2:0] state;
integer   k;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= ST_IDLE;
        busy           <= 1'b0;
        done           <= 1'b0;
        ss_start       <= 1'b0;
        cache_rd_en    <= 1'b0;
        cache_rd_addr  <= 32'b0;
        ds_wr_valid    <= 1'b0;
        ds_wr_item_idx <= 32'b0;
        ds_wr_data     <= 512'b0;
        item_idx       <= 32'b0;
        acc_idx        <= 3'd0;
        reg_value      <= 64'b0;
        mix_block      <= 512'b0;
        for (k = 0; k < 8; k = k + 1)
            rf[k] <= 64'b0;
    end else begin
        ss_start <= 1'b0;
        done     <= 1'b0;

        case (state)
            // -----------------------------------------------------------
            ST_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    item_idx <= 32'd0;
                    busy     <= 1'b1;
                    state    <= ST_ITEM_INIT;
                end
            end

            // -----------------------------------------------------------
            // Seed the register file from the item index (spec §7.3)
            // -----------------------------------------------------------
            ST_ITEM_INIT: begin
                rf[0]     <= item_seed;
                rf[1]     <= item_seed ^ SS_ADD1;
                rf[2]     <= item_seed ^ SS_ADD2;
                rf[3]     <= item_seed ^ SS_ADD3;
                rf[4]     <= item_seed ^ SS_ADD4;
                rf[5]     <= item_seed ^ SS_ADD5;
                rf[6]     <= item_seed ^ SS_ADD6;
                rf[7]     <= item_seed ^ SS_ADD7;
                reg_value <= {32'b0, item_idx};
                acc_idx   <= 3'd0;
                state     <= ST_MIX_REQ;
            end

            // -----------------------------------------------------------
            // Fetch the mix block, then run SuperscalarHash program acc_idx
            // -----------------------------------------------------------
            ST_MIX_REQ: begin
                cache_rd_en   <= 1'b1;
                cache_rd_addr <= cache_blk;
                if (cache_rd_en && cache_rd_valid) begin
                    cache_rd_en <= 1'b0;
                    mix_block   <= mix_slice;
                    ss_start    <= 1'b1;
                    state       <= ST_SS_RUN;
                end
            end

            // -----------------------------------------------------------
            ST_SS_RUN: begin
                if (ss_done) begin
                    rf[0]  <= ss_out0; rf[1] <= ss_out1;
                    rf[2]  <= ss_out2; rf[3] <= ss_out3;
                    rf[4]  <= ss_out4; rf[5] <= ss_out5;
                    rf[6]  <= ss_out6; rf[7] <= ss_out7;
                    state  <= ST_MIX_XOR;
                end
            end

            // -----------------------------------------------------------
            // r[q] ^= mix[q]; registerValue = r[addressRegister]
            // -----------------------------------------------------------
            ST_MIX_XOR: begin
                for (k = 0; k < 8; k = k + 1)
                    rf[k] <= rf[k] ^ mix_block[64*k +: 64];
                reg_value <= rf[prog_areg_r[acc_idx]] ^
                             mix_block[{prog_areg_r[acc_idx], 6'b0} +: 64];

                if ({29'b0, acc_idx} == (ACCESSES - 1)) begin
                    state <= ST_WRITE;
                end else begin
                    acc_idx <= acc_idx + 3'd1;
                    state   <= ST_MIX_REQ;
                end
            end

            // -----------------------------------------------------------
            // Store the finished 64-byte item
            // -----------------------------------------------------------
            ST_WRITE: begin
                ds_wr_valid    <= 1'b1;
                ds_wr_item_idx <= item_idx;
                ds_wr_data     <= {rf[7], rf[6], rf[5], rf[4],
                                   rf[3], rf[2], rf[1], rf[0]};
                if (ds_wr_valid && ds_wr_ready) begin
                    ds_wr_valid <= 1'b0;
                    if (item_idx == (ITEM_COUNT - 1)) begin
                        state <= ST_DONE;
                    end else begin
                        item_idx <= item_idx + 32'd1;
                        state    <= ST_ITEM_INIT;
                    end
                end
            end

            // -----------------------------------------------------------
            ST_DONE: begin
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
