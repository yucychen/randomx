// =============================================================================
// hbm_dataset_if.v — HBM2 Dataset AXI4 Master Interface
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// The RandomX Dataset is ~2.08 GiB (RANDOMX_DATASET_ITEM_COUNT × 64 bytes).
// On the XCVU33P the 8 GB HBM2 is accessed via AXI4 (AXI3 compatible) ports.
// Each HBM pseudo-channel provides a 256-bit wide AXI4 slave port.
//
// This module is a full AXI4 master that:
//   1. Accepts read requests from the VM (dataset item index), queued in a
//      small request FIFO so the VM can post ahead of AR issue.
//   2. Issues pipelined AXI4 AR bursts (2 beats × 256-bit per 64-byte item)
//      with up to MAX_OUTSTANDING transactions in flight (single AXI ID, so
//      responses are returned in order per the AXI4 spec).
//   3. Reassembles R beats into 64-byte items in a response FIFO and returns
//      them to the VM with valid/ready handshaking.
//   4. Accepts write requests (dataset item index + 64-byte data) from the
//      dataset generator (SuperscalarHash) and issues AXI4 AW/W bursts,
//      waiting for the B response before accepting the next write.
//
// Flow control invariant: an AR is only issued when the response FIFO has
// guaranteed space for the item (outstanding + queued responses < FIFO
// depth), so m_axi_rready can be held high without risk of overflow.
//
// AXI4 signals follow AMBA AXI4 spec (no QoS/user extensions needed).
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module hbm_dataset_if #(
    parameter AXI_ADDR_WIDTH = 34,  // 16 GB address space (XCVU33P HBM)
    parameter AXI_DATA_WIDTH = 256, // HBM pseudo-channel bus width
    parameter AXI_ID_WIDTH   = 6    // AXI ID width
) (
    input  wire                       clk,
    input  wire                       rst_n,

    // ---- VM read request interface ----
    // Request: VM wants a 64-byte item at dataset_addr (item-index)
    input  wire                       req_valid,
    input  wire [31:0]                req_item_idx,  // dataset item index
    output wire                       req_ready,

    // Response: 64-byte dataset item returned to VM
    output wire                       resp_valid,
    output wire [511:0]               resp_data,     // 64 bytes
    input  wire                       resp_ready,

    // ---- Dataset generator write request interface ----
    // Write: store a 64-byte dataset item at wr_item_idx
    input  wire                       wr_req_valid,
    input  wire [31:0]                wr_req_item_idx,
    input  wire [511:0]               wr_req_data,
    output reg                        wr_req_ready,
    output reg                        wr_done,       // 1-cycle pulse on B resp

    // ---- AXI4 Master — Read address channel ----
    output wire [AXI_ID_WIDTH-1:0]   m_axi_arid,
    output reg  [AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                 m_axi_arlen,   // burst length-1
    output wire [2:0]                 m_axi_arsize,  // beat size (log2 bytes)
    output wire [1:0]                 m_axi_arburst, // INCR = 2'b01
    output reg                        m_axi_arvalid,
    input  wire                       m_axi_arready,
    // ---- AXI4 Master — Read data channel ----
    input  wire [AXI_ID_WIDTH-1:0]   m_axi_rid,
    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]                 m_axi_rresp,
    input  wire                       m_axi_rlast,
    input  wire                       m_axi_rvalid,
    output wire                       m_axi_rready,

    // ---- AXI4 Master — Write address channel ----
    output wire [AXI_ID_WIDTH-1:0]   m_axi_awid,
    output reg  [AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]                 m_axi_awlen,
    output wire [2:0]                 m_axi_awsize,
    output wire [1:0]                 m_axi_awburst,
    output reg                        m_axi_awvalid,
    input  wire                       m_axi_awready,
    // ---- AXI4 Master — Write data channel ----
    output reg  [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output reg                        m_axi_wlast,
    output reg                        m_axi_wvalid,
    input  wire                       m_axi_wready,
    // ---- AXI4 Master — Write response channel ----
    input  wire [AXI_ID_WIDTH-1:0]   m_axi_bid,
    input  wire [1:0]                 m_axi_bresp,
    input  wire                       m_axi_bvalid,
    output reg                        m_axi_bready
);

// ---------------------------------------------------------------------------
// Constants
// Dataset item size = 64 bytes → with 256-bit bus → 2 beats per item
// ---------------------------------------------------------------------------
localparam BEATS_PER_ITEM  = 2;      // 64 bytes / 32 bytes per beat
localparam [7:0] BURST_LEN = 8'd1;   // arlen/awlen = beats - 1
localparam [2:0] BEAT_SIZE = 3'd5;   // 2^5 = 32 bytes per beat (256-bit)
localparam FIFO_DEPTH      = 4;      // request / response FIFO depth
localparam PTR_W           = 2;      // log2(FIFO_DEPTH)

// Static AXI attributes
assign m_axi_arid    = {AXI_ID_WIDTH{1'b0}}; // single ID → in-order responses
assign m_axi_arlen   = BURST_LEN;
assign m_axi_arsize  = BEAT_SIZE;
assign m_axi_arburst = 2'b01;                // INCR
assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
assign m_axi_awlen   = BURST_LEN;
assign m_axi_awsize  = BEAT_SIZE;
assign m_axi_awburst = 2'b01;                // INCR
assign m_axi_wstrb   = {(AXI_DATA_WIDTH/8){1'b1}}; // full-word writes

// ---------------------------------------------------------------------------
// Address calculation: item_idx × 64 bytes = item_idx << 6
// ---------------------------------------------------------------------------
function [AXI_ADDR_WIDTH-1:0] item_to_addr;
    input [31:0] item_idx;
    item_to_addr = {{(AXI_ADDR_WIDTH-32){1'b0}}, item_idx} << 6;
endfunction

// ===========================================================================
// READ PATH — request FIFO → pipelined AR issue → R reassembly → resp FIFO
// ===========================================================================

// ---- Request FIFO (item indices posted by the VM) ----
reg [31:0]      req_fifo [0:FIFO_DEPTH-1];
reg [PTR_W-1:0] req_wp, req_rp;
reg [PTR_W:0]   req_cnt;

wire req_push = req_valid && req_ready;

assign req_ready = (req_cnt < FIFO_DEPTH);

// ---- Response FIFO (assembled 64-byte items) ----
reg [511:0]     resp_fifo [0:FIFO_DEPTH-1];
reg [PTR_W-1:0] resp_wp, resp_rp;
reg [PTR_W:0]   resp_cnt;

assign resp_valid = (resp_cnt != 0);
assign resp_data  = resp_fifo[resp_rp];

wire resp_pop = resp_valid && resp_ready;

// ---- Outstanding read transaction tracking ----
// Only issue an AR when the response FIFO is guaranteed to have space for
// every in-flight item: outstanding + resp_cnt + queued-resp-writes < DEPTH.
reg [PTR_W:0] outstanding;

wire ar_fire   = m_axi_arvalid && m_axi_arready;
// Reserve space for: retired items in resp FIFO + in-flight bursts + a
// pending (not yet accepted) AR, so the resp FIFO can never overflow.
wire can_issue = (req_cnt != 0) &&
                 ((outstanding + resp_cnt +
                   {{PTR_W{1'b0}}, m_axi_arvalid}) < FIFO_DEPTH);

// R data is always accepted — space is reserved at AR issue time.
assign m_axi_rready = 1'b1;

// ---- R beat reassembly (2 beats per item, in order) ----
reg [255:0] rbeat_lo;   // first beat of the current item
reg         rbeat_sel;  // 0 = expecting low beat, 1 = expecting high beat

wire r_fire      = m_axi_rvalid && m_axi_rready;
wire resp_push   = r_fire && rbeat_sel; // second (last) beat completes an item
wire item_retire = r_fire && m_axi_rlast;

integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_wp        <= {PTR_W{1'b0}};
        req_rp        <= {PTR_W{1'b0}};
        req_cnt       <= {(PTR_W+1){1'b0}};
        resp_wp       <= {PTR_W{1'b0}};
        resp_rp       <= {PTR_W{1'b0}};
        resp_cnt      <= {(PTR_W+1){1'b0}};
        outstanding   <= {(PTR_W+1){1'b0}};
        m_axi_araddr  <= {AXI_ADDR_WIDTH{1'b0}};
        m_axi_arvalid <= 1'b0;
        rbeat_lo      <= 256'b0;
        rbeat_sel     <= 1'b0;
        for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
            req_fifo[i]  <= 32'b0;
            resp_fifo[i] <= 512'b0;
        end
    end else begin
        // --- Request FIFO push ---
        if (req_push) begin
            req_fifo[req_wp] <= req_item_idx;
            req_wp           <= req_wp + {{(PTR_W-1){1'b0}}, 1'b1};
        end

        // --- AR issue: pop request FIFO into AR channel ---
        if (!m_axi_arvalid || m_axi_arready) begin
            if (can_issue) begin
                m_axi_araddr  <= item_to_addr(req_fifo[req_rp]);
                m_axi_arvalid <= 1'b1;
                req_rp        <= req_rp + {{(PTR_W-1){1'b0}}, 1'b1};
            end else begin
                m_axi_arvalid <= 1'b0;
            end
        end

        // --- R beat capture / item reassembly ---
        if (r_fire) begin
            if (!rbeat_sel) begin
                rbeat_lo  <= m_axi_rdata;
                rbeat_sel <= 1'b1;
            end else begin
                resp_fifo[resp_wp] <= {m_axi_rdata, rbeat_lo};
                resp_wp            <= resp_wp + {{(PTR_W-1){1'b0}}, 1'b1};
                rbeat_sel          <= 1'b0;
            end
        end

        // --- Response FIFO pop ---
        if (resp_pop)
            resp_rp <= resp_rp + {{(PTR_W-1){1'b0}}, 1'b1};

        // --- Counters (push/pop may happen in the same cycle) ---
        case ({req_push, can_issue && (!m_axi_arvalid || m_axi_arready)})
            2'b10:   req_cnt <= req_cnt + 1'b1;
            2'b01:   req_cnt <= req_cnt - 1'b1;
            default: ; // 00 or 11: no net change
        endcase

        case ({resp_push, resp_pop})
            2'b10:   resp_cnt <= resp_cnt + 1'b1;
            2'b01:   resp_cnt <= resp_cnt - 1'b1;
            default: ;
        endcase

        case ({ar_fire, item_retire})
            2'b10:   outstanding <= outstanding + 1'b1;
            2'b01:   outstanding <= outstanding - 1'b1;
            default: ;
        endcase
    end
end

// ===========================================================================
// WRITE PATH — one item at a time: AW → W (2 beats) → B
// ===========================================================================
localparam WST_IDLE = 2'd0;
localparam WST_AW   = 2'd1;  // waiting for AW handshake
localparam WST_DATA = 2'd2;  // sending W beats
localparam WST_B    = 2'd3;  // waiting for B response

reg [1:0]   wstate;
reg [511:0] wdata_buf;
reg         wbeat_sel;  // 0 = low beat, 1 = high (last) beat

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wstate        <= WST_IDLE;
        wr_req_ready  <= 1'b1;
        wr_done       <= 1'b0;
        wdata_buf     <= 512'b0;
        wbeat_sel     <= 1'b0;
        m_axi_awaddr  <= {AXI_ADDR_WIDTH{1'b0}};
        m_axi_awvalid <= 1'b0;
        m_axi_wdata   <= {AXI_DATA_WIDTH{1'b0}};
        m_axi_wlast   <= 1'b0;
        m_axi_wvalid  <= 1'b0;
        m_axi_bready  <= 1'b0;
    end else begin
        wr_done <= 1'b0;

        case (wstate)
            WST_IDLE: begin
                if (wr_req_valid && wr_req_ready) begin
                    wr_req_ready  <= 1'b0;
                    wdata_buf     <= wr_req_data;
                    m_axi_awaddr  <= item_to_addr(wr_req_item_idx);
                    m_axi_awvalid <= 1'b1;
                    wbeat_sel     <= 1'b0;
                    wstate        <= WST_AW;
                end
            end

            WST_AW: begin
                if (m_axi_awready && m_axi_awvalid) begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wdata   <= wdata_buf[255:0];
                    m_axi_wlast   <= 1'b0;
                    m_axi_wvalid  <= 1'b1;
                    wstate        <= WST_DATA;
                end
            end

            WST_DATA: begin
                if (m_axi_wready && m_axi_wvalid) begin
                    if (!wbeat_sel) begin
                        // Low beat accepted — send high (last) beat
                        m_axi_wdata  <= wdata_buf[511:256];
                        m_axi_wlast  <= 1'b1;
                        wbeat_sel    <= 1'b1;
                    end else begin
                        // Last beat accepted — wait for write response
                        m_axi_wvalid <= 1'b0;
                        m_axi_wlast  <= 1'b0;
                        m_axi_bready <= 1'b1;
                        wstate       <= WST_B;
                    end
                end
            end

            WST_B: begin
                if (m_axi_bvalid) begin
                    m_axi_bready <= 1'b0;
                    wr_done      <= 1'b1;
                    wr_req_ready <= 1'b1;
                    wstate       <= WST_IDLE;
                end
            end

            default: wstate <= WST_IDLE;
        endcase
    end
end

endmodule
