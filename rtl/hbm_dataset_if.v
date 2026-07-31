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
//   2. Issues pipelined AXI4 AR bursts (ITEM_BYTES/beat-size beats per item)
//      with up to RD_FIFO_DEPTH transactions in flight (single AXI ID, so
//      responses are returned in order per the AXI4 spec).
//   3. Reassembles R beats into 64-byte items in a response FIFO and returns
//      them to the VM with valid/ready handshaking, together with the AXI
//      response status of the burst (resp_err).
//   4. Accepts write requests (dataset item index + 64-byte data) from the
//      dataset generator (SuperscalarHash) into a write FIFO and issues
//      pipelined AXI4 AW/W bursts with up to WR_FIFO_DEPTH outstanding write
//      transactions, so the generator is not stalled by the B round trip.
//
// Flow control invariants:
//   - An AR is only issued when the response FIFO has guaranteed space for
//     the item (outstanding + queued responses < depth), so m_axi_rready can
//     be held high without risk of overflow.
//   - A W burst is only started after the matching AW has been accepted, so
//     address/data ordering is trivially correct; entries are released from
//     the write FIFO when their last W beat is accepted.
//   - Both paths use a single AXI ID, so AXI4 guarantees in-order responses.
//
// Addressing: byte address = DATASET_BASE_ADDR + item_idx × 64.
//
// AXI4 signals follow the AMBA AXI4 spec (no QoS/user extensions needed).
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module hbm_dataset_if #(
    parameter AXI_ADDR_WIDTH    = 34,  // 16 GB address space (XCVU33P HBM)
    parameter AXI_DATA_WIDTH    = 256, // HBM pseudo-channel bus width
    parameter AXI_ID_WIDTH      = 6,   // AXI ID width
    // Byte offset of the dataset region inside HBM (must be 64-byte aligned)
    parameter DATASET_BASE_ADDR = 0,
    // Read request/response FIFO depth == max outstanding read bursts
    parameter RD_FIFO_DEPTH     = 4,   // power of two, >= 2
    // Write FIFO depth == max outstanding write bursts
    parameter WR_FIFO_DEPTH     = 4    // power of two, >= 2
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
    output wire                       resp_err,      // AXI error for this item
    input  wire                       resp_ready,

    // ---- Dataset generator write request interface ----
    // Write: store a 64-byte dataset item at wr_item_idx
    input  wire                       wr_req_valid,
    input  wire [31:0]                wr_req_item_idx,
    input  wire [511:0]               wr_req_data,
    output wire                       wr_req_ready,
    output reg                        wr_done,       // 1-cycle pulse on B resp
    output reg                        wr_err,        // qualified by wr_done

    // ---- Sticky AXI error status (cleared by reset only) ----
    output reg                        axi_err,

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
    output wire                       m_axi_bready
);

// ---------------------------------------------------------------------------
// Local helpers / constants
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

localparam ITEM_BITS      = 512;                         // 64-byte item
localparam BEATS_PER_ITEM = ITEM_BITS / AXI_DATA_WIDTH;  // 2 for 256-bit HBM
localparam BEAT_CNT_W     = (BEATS_PER_ITEM > 1) ? clog2(BEATS_PER_ITEM) : 1;
localparam [7:0] BURST_LEN = BEATS_PER_ITEM - 1;         // arlen/awlen
localparam [2:0] BEAT_SIZE = clog2(AXI_DATA_WIDTH/8);    // 5 → 32 bytes

localparam [AXI_ADDR_WIDTH-1:0] BASE_ADDR = DATASET_BASE_ADDR;

localparam RD_PTR_W = clog2(RD_FIFO_DEPTH);
localparam WR_PTR_W = clog2(WR_FIFO_DEPTH);

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
// Address calculation: DATASET_BASE_ADDR + item_idx × 64 bytes
// ---------------------------------------------------------------------------
function [AXI_ADDR_WIDTH-1:0] item_to_addr;
    input [31:0] item_idx;
    item_to_addr = BASE_ADDR +
                   ({{(AXI_ADDR_WIDTH-32){1'b0}}, item_idx} << 6);
endfunction

// ===========================================================================
// READ PATH — request FIFO → pipelined AR issue → R reassembly → resp FIFO
// ===========================================================================

// ---- Request FIFO (item indices posted by the VM) ----
reg [31:0]         req_fifo [0:RD_FIFO_DEPTH-1];
reg [RD_PTR_W-1:0] req_wp, req_rp;
reg [RD_PTR_W:0]   req_cnt;

wire req_push = req_valid && req_ready;

assign req_ready = (req_cnt < RD_FIFO_DEPTH);

// ---- Response FIFO (assembled 64-byte items) ----
reg [ITEM_BITS-1:0] resp_fifo [0:RD_FIFO_DEPTH-1];
reg                 resp_err_fifo [0:RD_FIFO_DEPTH-1];
reg [RD_PTR_W-1:0]  resp_wp, resp_rp;
reg [RD_PTR_W:0]    resp_cnt;

assign resp_valid = (resp_cnt != 0);
assign resp_data  = resp_fifo[resp_rp];
assign resp_err   = resp_err_fifo[resp_rp];

wire resp_pop = resp_valid && resp_ready;

// ---- Outstanding read transaction tracking ----
// Only issue an AR when the response FIFO is guaranteed to have space for
// every in-flight item: outstanding + resp_cnt + queued-resp-writes < DEPTH.
reg [RD_PTR_W:0] outstanding;

wire ar_fire   = m_axi_arvalid && m_axi_arready;
// Reserve space for: retired items in resp FIFO + in-flight bursts + a
// pending (not yet accepted) AR, so the resp FIFO can never overflow.
wire can_issue = (req_cnt != 0) &&
                 ((outstanding + resp_cnt +
                   {{RD_PTR_W{1'b0}}, m_axi_arvalid}) < RD_FIFO_DEPTH);
wire ar_pop    = can_issue && (!m_axi_arvalid || m_axi_arready);

// R data is always accepted — space is reserved at AR issue time.
assign m_axi_rready = 1'b1;

// ---- R beat reassembly ----
// Beats arrive LSB-first; shift them in from the top so that beat 0 ends up
// in resp_data[AXI_DATA_WIDTH-1:0]. RLAST (not a local counter) delimits the
// item, so a slave returning a different burst length cannot desynchronise
// the assembler.
reg [ITEM_BITS-1:0]  rd_shift;
reg                  rd_err_acc;

wire                 r_fire      = m_axi_rvalid && m_axi_rready;
wire                 r_beat_err  = (m_axi_rresp != 2'b00);
wire                 item_retire = r_fire && m_axi_rlast;
wire                 resp_push   = item_retire;
wire [ITEM_BITS-1:0] rd_shift_nxt = {m_axi_rdata,
                                     rd_shift[ITEM_BITS-1:AXI_DATA_WIDTH]};

integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_wp        <= {RD_PTR_W{1'b0}};
        req_rp        <= {RD_PTR_W{1'b0}};
        req_cnt       <= {(RD_PTR_W+1){1'b0}};
        resp_wp       <= {RD_PTR_W{1'b0}};
        resp_rp       <= {RD_PTR_W{1'b0}};
        resp_cnt      <= {(RD_PTR_W+1){1'b0}};
        outstanding   <= {(RD_PTR_W+1){1'b0}};
        m_axi_araddr  <= {AXI_ADDR_WIDTH{1'b0}};
        m_axi_arvalid <= 1'b0;
        rd_shift      <= {ITEM_BITS{1'b0}};
        rd_err_acc    <= 1'b0;
        for (i = 0; i < RD_FIFO_DEPTH; i = i + 1) begin
            req_fifo[i]      <= 32'b0;
            resp_fifo[i]     <= {ITEM_BITS{1'b0}};
            resp_err_fifo[i] <= 1'b0;
        end
    end else begin
        // --- Request FIFO push ---
        if (req_push) begin
            req_fifo[req_wp] <= req_item_idx;
            req_wp           <= req_wp + {{(RD_PTR_W-1){1'b0}}, 1'b1};
        end

        // --- AR issue: pop request FIFO into AR channel ---
        if (!m_axi_arvalid || m_axi_arready) begin
            if (can_issue) begin
                m_axi_araddr  <= item_to_addr(req_fifo[req_rp]);
                m_axi_arvalid <= 1'b1;
                req_rp        <= req_rp + {{(RD_PTR_W-1){1'b0}}, 1'b1};
            end else begin
                m_axi_arvalid <= 1'b0;
            end
        end

        // --- R beat capture / item reassembly ---
        if (r_fire) begin
            rd_shift <= rd_shift_nxt;
            if (m_axi_rlast) begin
                resp_fifo[resp_wp]     <= rd_shift_nxt;
                resp_err_fifo[resp_wp] <= rd_err_acc | r_beat_err;
                resp_wp                <= resp_wp +
                                          {{(RD_PTR_W-1){1'b0}}, 1'b1};
                rd_err_acc             <= 1'b0;
            end else begin
                rd_err_acc <= rd_err_acc | r_beat_err;
            end
        end

        // --- Response FIFO pop ---
        if (resp_pop)
            resp_rp <= resp_rp + {{(RD_PTR_W-1){1'b0}}, 1'b1};

        // --- Counters (push/pop may happen in the same cycle) ---
        case ({req_push, ar_pop})
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
// WRITE PATH — write FIFO → pipelined AW issue → W bursts → B retirement
//
// Three loosely coupled engines share one FIFO:
//   aw_rp : entries whose address has not been pushed onto the AW channel
//   w_rp  : entries whose AW has fired and whose data still has to be sent
//   b     : bursts whose data was sent and whose B response is outstanding
// ===========================================================================
reg [AXI_ADDR_WIDTH-1:0] wr_addr_fifo [0:WR_FIFO_DEPTH-1];
reg [ITEM_BITS-1:0]      wr_data_fifo [0:WR_FIFO_DEPTH-1];

reg [WR_PTR_W-1:0] wr_wp, aw_rp, w_rp;
reg [WR_PTR_W:0]   wr_cnt;    // entries pushed, data not yet fully sent
reg [WR_PTR_W:0]   aw_pend;   // entries whose address is not yet on AW
reg [WR_PTR_W:0]   w_pend;    // entries whose AW fired, data not yet started
reg [WR_PTR_W:0]   b_out;     // AW accepted, B not yet received

wire wr_push = wr_req_valid && wr_req_ready;

assign wr_req_ready = (wr_cnt < WR_FIFO_DEPTH) && (b_out < WR_FIFO_DEPTH);

// B responses are always accepted (b_out is bounded by WR_FIFO_DEPTH).
assign m_axi_bready = 1'b1;

wire aw_fire = m_axi_awvalid && m_axi_awready;
wire b_fire  = m_axi_bvalid  && m_axi_bready;
// Reserve one B slot for a pending (not yet accepted) AW.
wire aw_can  = (aw_pend != 0) &&
               ((b_out + {{WR_PTR_W{1'b0}}, m_axi_awvalid}) < WR_FIFO_DEPTH);
wire aw_pop  = aw_can && (!m_axi_awvalid || m_axi_awready);

// ---- W data engine ----
reg [ITEM_BITS-1:0]  w_shift;
reg [BEAT_CNT_W:0]   wbeat;     // number of beats already presented

wire w_fire       = m_axi_wvalid && m_axi_wready;
wire w_burst_last = w_fire && m_axi_wlast;
// A new burst may start when the engine is idle or is retiring its last beat
wire w_slot_free  = (!m_axi_wvalid) || w_burst_last;
wire w_start      = w_slot_free && (w_pend != 0);

wire [ITEM_BITS-1:0] w_entry     = wr_data_fifo[w_rp];
wire [ITEM_BITS-1:0] w_shift_nxt = w_shift >> AXI_DATA_WIDTH;

integer j;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_wp         <= {WR_PTR_W{1'b0}};
        aw_rp         <= {WR_PTR_W{1'b0}};
        w_rp          <= {WR_PTR_W{1'b0}};
        wr_cnt        <= {(WR_PTR_W+1){1'b0}};
        aw_pend       <= {(WR_PTR_W+1){1'b0}};
        w_pend        <= {(WR_PTR_W+1){1'b0}};
        b_out         <= {(WR_PTR_W+1){1'b0}};
        wr_done       <= 1'b0;
        wr_err        <= 1'b0;
        axi_err       <= 1'b0;
        w_shift       <= {ITEM_BITS{1'b0}};
        wbeat         <= {(BEAT_CNT_W+1){1'b0}};
        m_axi_awaddr  <= {AXI_ADDR_WIDTH{1'b0}};
        m_axi_awvalid <= 1'b0;
        m_axi_wdata   <= {AXI_DATA_WIDTH{1'b0}};
        m_axi_wlast   <= 1'b0;
        m_axi_wvalid  <= 1'b0;
        for (j = 0; j < WR_FIFO_DEPTH; j = j + 1) begin
            wr_addr_fifo[j] <= {AXI_ADDR_WIDTH{1'b0}};
            wr_data_fifo[j] <= {ITEM_BITS{1'b0}};
        end
    end else begin
        wr_done <= 1'b0;
        wr_err  <= 1'b0;

        // --- Write FIFO push ---
        if (wr_push) begin
            wr_addr_fifo[wr_wp] <= item_to_addr(wr_req_item_idx);
            wr_data_fifo[wr_wp] <= wr_req_data;
            wr_wp               <= wr_wp + {{(WR_PTR_W-1){1'b0}}, 1'b1};
        end

        // --- AW issue ---
        if (!m_axi_awvalid || m_axi_awready) begin
            if (aw_can) begin
                m_axi_awaddr  <= wr_addr_fifo[aw_rp];
                m_axi_awvalid <= 1'b1;
                aw_rp         <= aw_rp + {{(WR_PTR_W-1){1'b0}}, 1'b1};
            end else begin
                m_axi_awvalid <= 1'b0;
            end
        end

        // --- W data burst ---
        if (w_start) begin
            // Load the first beat of the next entry (back-to-back capable)
            m_axi_wdata  <= w_entry[AXI_DATA_WIDTH-1:0];
            m_axi_wvalid <= 1'b1;
            m_axi_wlast  <= (BEATS_PER_ITEM == 1);
            w_shift      <= w_entry >> AXI_DATA_WIDTH;
            wbeat        <= {{BEAT_CNT_W{1'b0}}, 1'b1};
        end else if (w_burst_last) begin
            m_axi_wvalid <= 1'b0;
            m_axi_wlast  <= 1'b0;
            wbeat        <= {(BEAT_CNT_W+1){1'b0}};
        end else if (w_fire) begin
            m_axi_wdata  <= w_shift[AXI_DATA_WIDTH-1:0];
            m_axi_wlast  <= ((wbeat + 1'b1) == BEATS_PER_ITEM);
            w_shift      <= w_shift_nxt;
            wbeat        <= wbeat + 1'b1;
        end

        if (w_start)
            w_rp <= w_rp + {{(WR_PTR_W-1){1'b0}}, 1'b1};

        // --- B response retirement ---
        if (b_fire) begin
            wr_done <= 1'b1;
            wr_err  <= (m_axi_bresp != 2'b00);
        end

        // --- Sticky AXI error status ---
        if ((b_fire && (m_axi_bresp != 2'b00)) || (r_fire && r_beat_err))
            axi_err <= 1'b1;

        // --- Counters ---
        case ({wr_push, w_burst_last})
            2'b10:   wr_cnt <= wr_cnt + 1'b1;
            2'b01:   wr_cnt <= wr_cnt - 1'b1;
            default: ;
        endcase

        case ({wr_push, aw_pop})
            2'b10:   aw_pend <= aw_pend + 1'b1;
            2'b01:   aw_pend <= aw_pend - 1'b1;
            default: ;
        endcase

        case ({aw_fire, w_start})
            2'b10:   w_pend <= w_pend + 1'b1;
            2'b01:   w_pend <= w_pend - 1'b1;
            default: ;
        endcase

        case ({aw_fire, b_fire})
            2'b10:   b_out <= b_out + 1'b1;
            2'b01:   b_out <= b_out - 1'b1;
            default: ;
        endcase
    end
end

endmodule
