// =============================================================================
// cache_hbm_if.v — Argon2d Cache Storage AXI4 Master Interface
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// The RandomX Cache is RANDOMX_ARGON_MEMORY × 1 KiB = 256 MiB, far larger than
// the on-chip URAM of the XCVU33P (~1.4 MiB), so it is stored in HBM2 next to
// the Dataset. This module converts the 1 KiB block port of `argon2_fill`
// (and of any other cache reader, e.g. the dataset generator) into AXI4
// bursts on a 256-bit HBM pseudo-channel:
//
//   1 KiB block = 1024 bytes / 32 bytes-per-beat = 32 beats (awlen/arlen = 31)
//   byte address = CACHE_BASE_ADDR + block_idx × 1024
//
// Because the block size is 1024 bytes and CACHE_BASE_ADDR is required to be
// 1 KiB aligned, no burst ever crosses the AXI 4 KiB boundary.
//
// Protocol on the block port (matches argon2_fill):
//   write : `wr_en` is held until `wr_rdy` is sampled high for one cycle
//           (the whole block has then been accepted and written to HBM).
//   read  : `rd_en` is held until `rd_valid` pulses for one cycle together
//           with the whole 1 KiB block on `rd_data`.
//   Requests are served strictly one at a time; a new request is only sampled
//   after the previous `*_en` has been deasserted, so a held enable can never
//   be interpreted as a second request.
//
// Endianness: block bit [8*k +: 8] is cache byte k, and the first AXI beat
// carries bytes 0..31, i.e. block bits [255:0] — the same little-endian
// packing used by argon2_fill and hbm_dataset_if.
//
// AXI4 signals follow the AMBA AXI4 spec (single ID, INCR bursts).
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module cache_hbm_if #(
    parameter AXI_ADDR_WIDTH  = 34,          // 16 GB address space (XCVU33P HBM)
    parameter AXI_DATA_WIDTH  = 256,         // HBM pseudo-channel bus width
    parameter AXI_ID_WIDTH    = 6,           // AXI ID width
    // Byte offset of the cache region inside HBM (must be 1 KiB aligned)
    parameter CACHE_BASE_ADDR = 34'h0_C000_0000
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // ---- 1 KiB block write port ----
    input  wire                        wr_en,
    input  wire [31:0]                 wr_addr,   // block index
    input  wire [8191:0]               wr_data,   // one 1 KiB block
    output reg                         wr_rdy,    // 1-cycle accept pulse

    // ---- 1 KiB block read port ----
    input  wire                        rd_en,
    input  wire [31:0]                 rd_addr,   // block index
    output reg  [8191:0]               rd_data,
    output reg                         rd_valid,  // 1-cycle data valid pulse

    // ---- Sticky AXI error status (cleared by reset only) ----
    output reg                         axi_err,

    // ---- AXI4 Master — Read address channel ----
    output wire [AXI_ID_WIDTH-1:0]     m_axi_arid,
    output reg  [AXI_ADDR_WIDTH-1:0]   m_axi_araddr,
    output wire [7:0]                  m_axi_arlen,
    output wire [2:0]                  m_axi_arsize,
    output wire [1:0]                  m_axi_arburst,
    output reg                         m_axi_arvalid,
    input  wire                        m_axi_arready,
    // ---- AXI4 Master — Read data channel ----
    input  wire [AXI_DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]                  m_axi_rresp,
    input  wire                        m_axi_rlast,
    input  wire                        m_axi_rvalid,
    output wire                        m_axi_rready,

    // ---- AXI4 Master — Write address channel ----
    output wire [AXI_ID_WIDTH-1:0]     m_axi_awid,
    output reg  [AXI_ADDR_WIDTH-1:0]   m_axi_awaddr,
    output wire [7:0]                  m_axi_awlen,
    output wire [2:0]                  m_axi_awsize,
    output wire [1:0]                  m_axi_awburst,
    output reg                         m_axi_awvalid,
    input  wire                        m_axi_awready,
    // ---- AXI4 Master — Write data channel ----
    output wire [AXI_DATA_WIDTH-1:0]   m_axi_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire                        m_axi_wlast,
    output reg                         m_axi_wvalid,
    input  wire                        m_axi_wready,
    // ---- AXI4 Master — Write response channel ----
    input  wire [1:0]                  m_axi_bresp,
    input  wire                        m_axi_bvalid,
    output wire                        m_axi_bready
);

// ---------------------------------------------------------------------------
// Local constants
// ---------------------------------------------------------------------------
function integer clog2;
    input integer value;
    integer v;
    begin
        clog2 = 0;
        for (v = value - 1; v > 0; v = v >> 1)
            clog2 = clog2 + 1;
    end
endfunction

localparam BLOCK_BITS      = 8192;                            // 1 KiB block
localparam BLOCK_BYTES     = BLOCK_BITS / 8;                  // 1024
localparam BEATS_PER_BLOCK = BLOCK_BITS / AXI_DATA_WIDTH;     // 32 @256-bit
localparam BEAT_CNT_W      = clog2(BEATS_PER_BLOCK + 1);
localparam BLOCK_SHIFT     = clog2(BLOCK_BYTES);              // 10 → ×1024

localparam [31:0] BURST_LEN_W = BEATS_PER_BLOCK - 1;
localparam [31:0] BEAT_SIZE_W = clog2(AXI_DATA_WIDTH/8);
localparam [7:0]  BURST_LEN   = BURST_LEN_W[7:0];
localparam [2:0]  BEAT_SIZE   = BEAT_SIZE_W[2:0];

localparam [AXI_ADDR_WIDTH-1:0] BASE_ADDR = CACHE_BASE_ADDR;

// Static AXI attributes
assign m_axi_arid    = {AXI_ID_WIDTH{1'b0}}; // single ID → in-order responses
assign m_axi_arlen   = BURST_LEN;
assign m_axi_arsize  = BEAT_SIZE;
assign m_axi_arburst = 2'b01;                // INCR
assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
assign m_axi_awlen   = BURST_LEN;
assign m_axi_awsize  = BEAT_SIZE;
assign m_axi_awburst = 2'b01;                // INCR
assign m_axi_wstrb   = {(AXI_DATA_WIDTH/8){1'b1}};
assign m_axi_rready  = 1'b1;                 // only one burst in flight
assign m_axi_bready  = 1'b1;

// ---------------------------------------------------------------------------
// Block index → byte address
// ---------------------------------------------------------------------------
function [AXI_ADDR_WIDTH-1:0] blk_to_addr;
    input [31:0] blk_idx;
    blk_to_addr = BASE_ADDR +
                  ({{(AXI_ADDR_WIDTH-32){1'b0}}, blk_idx} << BLOCK_SHIFT);
endfunction

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------
localparam ST_IDLE     = 3'd0;
localparam ST_WR_ADDR  = 3'd1;   // drive AW
localparam ST_WR_DATA  = 3'd2;   // drive W beats
localparam ST_WR_RESP  = 3'd3;   // wait for B
localparam ST_WR_ACK   = 3'd4;   // pulse wr_rdy, wait for wr_en release
localparam ST_RD_ADDR  = 3'd5;   // drive AR
localparam ST_RD_DATA  = 3'd6;   // collect R beats
localparam ST_RD_ACK   = 3'd7;   // pulse rd_valid, wait for rd_en release

reg [2:0]            state;
reg [BLOCK_BITS-1:0] wr_buf;      // latched write block (shifted out beat-wise)
reg [BLOCK_BITS-1:0] rd_buf;      // read block assembled from R beats
reg [BEAT_CNT_W-1:0] beat_cnt;    // beats already sent/received

// Lowest beat of the write buffer is driven on W; the buffer is shifted right
// by one beat after every accepted beat, so the beats leave in ascending
// byte-address order.
assign m_axi_wdata = wr_buf[AXI_DATA_WIDTH-1:0];
assign m_axi_wlast = (beat_cnt == BURST_LEN_W[BEAT_CNT_W-1:0]);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state         <= ST_IDLE;
        wr_rdy        <= 1'b0;
        rd_valid      <= 1'b0;
        rd_data       <= {BLOCK_BITS{1'b0}};
        rd_buf        <= {BLOCK_BITS{1'b0}};
        wr_buf        <= {BLOCK_BITS{1'b0}};
        beat_cnt      <= {BEAT_CNT_W{1'b0}};
        axi_err       <= 1'b0;
        m_axi_araddr  <= {AXI_ADDR_WIDTH{1'b0}};
        m_axi_arvalid <= 1'b0;
        m_axi_awaddr  <= {AXI_ADDR_WIDTH{1'b0}};
        m_axi_awvalid <= 1'b0;
        m_axi_wvalid  <= 1'b0;
    end else begin
        wr_rdy   <= 1'b0;
        rd_valid <= 1'b0;

        case (state)
            // ---- Idle: sample a new block request (writes have priority) ---
            ST_IDLE: begin
                beat_cnt <= {BEAT_CNT_W{1'b0}};
                if (wr_en) begin
                    wr_buf        <= wr_data;
                    m_axi_awaddr  <= blk_to_addr(wr_addr);
                    m_axi_awvalid <= 1'b1;
                    state         <= ST_WR_ADDR;
                end else if (rd_en) begin
                    m_axi_araddr  <= blk_to_addr(rd_addr);
                    m_axi_arvalid <= 1'b1;
                    state         <= ST_RD_ADDR;
                end
            end

            // ---- Write: AW handshake ---------------------------------------
            ST_WR_ADDR: begin
                if (m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b1;
                    state         <= ST_WR_DATA;
                end
            end

            // ---- Write: W beats --------------------------------------------
            ST_WR_DATA: begin
                if (m_axi_wready) begin
                    wr_buf <= {{AXI_DATA_WIDTH{1'b0}},
                               wr_buf[BLOCK_BITS-1:AXI_DATA_WIDTH]};
                    if (m_axi_wlast) begin
                        m_axi_wvalid <= 1'b0;
                        state        <= ST_WR_RESP;
                    end else begin
                        beat_cnt <= beat_cnt + 1'b1;
                    end
                end
            end

            // ---- Write: B response -----------------------------------------
            ST_WR_RESP: begin
                if (m_axi_bvalid) begin
                    if (m_axi_bresp != 2'b00)
                        axi_err <= 1'b1;
                    wr_rdy <= 1'b1;      // block accepted (one-cycle pulse)
                    state  <= ST_WR_ACK;
                end
            end

            // ---- Write: wait for the requester to drop wr_en ----------------
            ST_WR_ACK: begin
                if (!wr_en)
                    state <= ST_IDLE;
            end

            // ---- Read: AR handshake -----------------------------------------
            ST_RD_ADDR: begin
                if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    state         <= ST_RD_DATA;
                end
            end

            // ---- Read: R beats ----------------------------------------------
            ST_RD_DATA: begin
                if (m_axi_rvalid) begin
                    // beats arrive in ascending byte-address order → shift in
                    // from the top so beat 0 ends up in rd_buf[255:0]
                    rd_buf <= {m_axi_rdata,
                               rd_buf[BLOCK_BITS-1:AXI_DATA_WIDTH]};
                    if (m_axi_rresp != 2'b00)
                        axi_err <= 1'b1;
                    if (m_axi_rlast) begin
                        rd_data  <= {m_axi_rdata,
                                     rd_buf[BLOCK_BITS-1:AXI_DATA_WIDTH]};
                        rd_valid <= 1'b1;
                        state    <= ST_RD_ACK;
                    end else begin
                        beat_cnt <= beat_cnt + 1'b1;
                    end
                end
            end

            // ---- Read: wait for the requester to drop rd_en -----------------
            ST_RD_ACK: begin
                if (!rd_en)
                    state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
