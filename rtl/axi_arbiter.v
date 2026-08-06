// =============================================================================
// axi_arbiter.v — 2-Master AXI4 Arbiter (single slave port)
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// The XCVU33P HBM pseudo-channel exposes a single AXI4 slave port, but two
// masters need it:
//   M0 — cache_hbm_if   (Argon2d cache blocks, active during CACHE_INIT)
//   M1 — hbm_dataset_if (Dataset items, active during DS_GEN / VM_RUN)
//
// Arbitration rules (read and write paths arbitrate independently):
//   * A grant is exclusive: while a master has transactions in flight
//     (address accepted, response not complete) the other master cannot
//     issue an address, so responses can be routed by the recorded owner.
//     Both masters use AXI ID 0, so this exclusivity is what keeps the
//     in-order response assumption of each master valid.
//   * When nothing is in flight the arbiter picks a requesting master
//     round-robin (the master granted last has the lower priority), so
//     neither master can starve the other.
//   * The W channel follows the write address owner, so W bursts always
//     belong to the AW that was accepted by the same master.
//
// Verilog-2001 compliant, purely combinational routing + 2 small counters.
// =============================================================================

`timescale 1ns/1ps

module axi_arbiter #(
    parameter AXI_ADDR_WIDTH = 34,
    parameter AXI_DATA_WIDTH = 256,
    // Maximum number of in-flight bursts tracked per direction
    parameter MAX_OUTSTANDING = 16
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // ---------------- Master 0 ----------------
    input  wire [AXI_ADDR_WIDTH-1:0]   m0_araddr,
    input  wire [7:0]                  m0_arlen,
    input  wire [2:0]                  m0_arsize,
    input  wire [1:0]                  m0_arburst,
    input  wire                        m0_arvalid,
    output wire                        m0_arready,
    output wire [AXI_DATA_WIDTH-1:0]   m0_rdata,
    output wire [1:0]                  m0_rresp,
    output wire                        m0_rlast,
    output wire                        m0_rvalid,
    input  wire                        m0_rready,
    input  wire [AXI_ADDR_WIDTH-1:0]   m0_awaddr,
    input  wire [7:0]                  m0_awlen,
    input  wire [2:0]                  m0_awsize,
    input  wire [1:0]                  m0_awburst,
    input  wire                        m0_awvalid,
    output wire                        m0_awready,
    input  wire [AXI_DATA_WIDTH-1:0]   m0_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] m0_wstrb,
    input  wire                        m0_wlast,
    input  wire                        m0_wvalid,
    output wire                        m0_wready,
    output wire [1:0]                  m0_bresp,
    output wire                        m0_bvalid,
    input  wire                        m0_bready,

    // ---------------- Master 1 ----------------
    input  wire [AXI_ADDR_WIDTH-1:0]   m1_araddr,
    input  wire [7:0]                  m1_arlen,
    input  wire [2:0]                  m1_arsize,
    input  wire [1:0]                  m1_arburst,
    input  wire                        m1_arvalid,
    output wire                        m1_arready,
    output wire [AXI_DATA_WIDTH-1:0]   m1_rdata,
    output wire [1:0]                  m1_rresp,
    output wire                        m1_rlast,
    output wire                        m1_rvalid,
    input  wire                        m1_rready,
    input  wire [AXI_ADDR_WIDTH-1:0]   m1_awaddr,
    input  wire [7:0]                  m1_awlen,
    input  wire [2:0]                  m1_awsize,
    input  wire [1:0]                  m1_awburst,
    input  wire                        m1_awvalid,
    output wire                        m1_awready,
    input  wire [AXI_DATA_WIDTH-1:0]   m1_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] m1_wstrb,
    input  wire                        m1_wlast,
    input  wire                        m1_wvalid,
    output wire                        m1_wready,
    output wire [1:0]                  m1_bresp,
    output wire                        m1_bvalid,
    input  wire                        m1_bready,

    // ---------------- Slave (HBM) ----------------
    output wire [AXI_ADDR_WIDTH-1:0]   s_araddr,
    output wire [7:0]                  s_arlen,
    output wire [2:0]                  s_arsize,
    output wire [1:0]                  s_arburst,
    output wire                        s_arvalid,
    input  wire                        s_arready,
    input  wire [AXI_DATA_WIDTH-1:0]   s_rdata,
    input  wire [1:0]                  s_rresp,
    input  wire                        s_rlast,
    input  wire                        s_rvalid,
    output wire                        s_rready,
    output wire [AXI_ADDR_WIDTH-1:0]   s_awaddr,
    output wire [7:0]                  s_awlen,
    output wire [2:0]                  s_awsize,
    output wire [1:0]                  s_awburst,
    output wire                        s_awvalid,
    input  wire                        s_awready,
    output wire [AXI_DATA_WIDTH-1:0]   s_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0] s_wstrb,
    output wire                        s_wlast,
    output wire                        s_wvalid,
    input  wire                        s_wready,
    input  wire [1:0]                  s_bresp,
    input  wire                        s_bvalid,
    output wire                        s_bready
);

function integer clog2;
    input integer value;
    integer v;
    begin
        clog2 = 0;
        for (v = value - 1; v > 0; v = v >> 1)
            clog2 = clog2 + 1;
    end
endfunction

localparam CNT_W = clog2(MAX_OUTSTANDING + 1);

// ===========================================================================
// Read path
// ===========================================================================
reg              rd_owner;    // master currently owning the read channel
reg              rd_last;     // last granted master (round-robin state)
reg  [CNT_W-1:0] rd_out;      // outstanding read bursts

wire rd_locked  = (rd_out != {CNT_W{1'b0}});
wire rd_full    = (rd_out == MAX_OUTSTANDING[CNT_W-1:0]);
// Round-robin pick when the channel is free: prefer the master that was
// *not* served last.
wire rd_rr      = (rd_last == 1'b0) ? (m1_arvalid ? 1'b1 : 1'b0)
                                    : (m0_arvalid ? 1'b0 : 1'b1);
wire rd_sel     = rd_locked ? rd_owner : rd_rr;

assign s_araddr  = rd_sel ? m1_araddr  : m0_araddr;
assign s_arlen   = rd_sel ? m1_arlen   : m0_arlen;
assign s_arsize  = rd_sel ? m1_arsize  : m0_arsize;
assign s_arburst = rd_sel ? m1_arburst : m0_arburst;
assign s_arvalid = (rd_sel ? m1_arvalid : m0_arvalid) && !rd_full;

assign m0_arready = (rd_sel == 1'b0) && s_arready && !rd_full;
assign m1_arready = (rd_sel == 1'b1) && s_arready && !rd_full;

assign m0_rdata  = s_rdata;
assign m1_rdata  = s_rdata;
assign m0_rresp  = s_rresp;
assign m1_rresp  = s_rresp;
assign m0_rlast  = s_rlast;
assign m1_rlast  = s_rlast;
assign m0_rvalid = s_rvalid && (rd_owner == 1'b0);
assign m1_rvalid = s_rvalid && (rd_owner == 1'b1);
assign s_rready  = rd_owner ? m1_rready : m0_rready;

wire rd_ar_hs = s_arvalid && s_arready;
wire rd_r_hs  = s_rvalid  && s_rready && s_rlast;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_owner <= 1'b0;
        rd_last  <= 1'b0;
        rd_out   <= {CNT_W{1'b0}};
    end else begin
        if (rd_ar_hs) begin
            rd_owner <= rd_sel;
            rd_last  <= rd_sel;
        end
        case ({rd_ar_hs, rd_r_hs})
            2'b10:   rd_out <= rd_out + 1'b1;
            2'b01:   rd_out <= rd_out - 1'b1;
            default: rd_out <= rd_out;
        endcase
    end
end

// ===========================================================================
// Write path
// ===========================================================================
reg              wr_owner;
reg              wr_last;
reg  [CNT_W-1:0] wr_out;      // outstanding write bursts (AW issued, no B yet)

wire wr_locked = (wr_out != {CNT_W{1'b0}});
wire wr_full   = (wr_out == MAX_OUTSTANDING[CNT_W-1:0]);
wire wr_rr     = (wr_last == 1'b0) ? (m1_awvalid ? 1'b1 : 1'b0)
                                   : (m0_awvalid ? 1'b0 : 1'b1);
wire wr_sel    = wr_locked ? wr_owner : wr_rr;

assign s_awaddr  = wr_sel ? m1_awaddr  : m0_awaddr;
assign s_awlen   = wr_sel ? m1_awlen   : m0_awlen;
assign s_awsize  = wr_sel ? m1_awsize  : m0_awsize;
assign s_awburst = wr_sel ? m1_awburst : m0_awburst;
assign s_awvalid = (wr_sel ? m1_awvalid : m0_awvalid) && !wr_full;

assign m0_awready = (wr_sel == 1'b0) && s_awready && !wr_full;
assign m1_awready = (wr_sel == 1'b1) && s_awready && !wr_full;

// W follows the write address owner (locked while bursts are in flight)
assign s_wdata  = wr_owner ? m1_wdata  : m0_wdata;
assign s_wstrb  = wr_owner ? m1_wstrb  : m0_wstrb;
assign s_wlast  = wr_owner ? m1_wlast  : m0_wlast;
assign s_wvalid = wr_locked && (wr_owner ? m1_wvalid : m0_wvalid);

assign m0_wready = wr_locked && (wr_owner == 1'b0) && s_wready;
assign m1_wready = wr_locked && (wr_owner == 1'b1) && s_wready;

assign m0_bresp  = s_bresp;
assign m1_bresp  = s_bresp;
assign m0_bvalid = s_bvalid && (wr_owner == 1'b0);
assign m1_bvalid = s_bvalid && (wr_owner == 1'b1);
assign s_bready  = wr_owner ? m1_bready : m0_bready;

wire wr_aw_hs = s_awvalid && s_awready;
wire wr_b_hs  = s_bvalid  && s_bready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_owner <= 1'b0;
        wr_last  <= 1'b0;
        wr_out   <= {CNT_W{1'b0}};
    end else begin
        if (wr_aw_hs) begin
            wr_owner <= wr_sel;
            wr_last  <= wr_sel;
        end
        case ({wr_aw_hs, wr_b_hs})
            2'b10:   wr_out <= wr_out + 1'b1;
            2'b01:   wr_out <= wr_out - 1'b1;
            default: wr_out <= wr_out;
        endcase
    end
end

endmodule
