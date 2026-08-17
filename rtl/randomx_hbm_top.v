// =============================================================================
// randomx_hbm_top.v — Board-level top wrapping randomx_top for HBM2
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// `randomx_top.v` is deliberately vendor-neutral: it exposes a single AXI4
// master (34-bit address, 256-bit data, AXI ID tied to 0, INCR bursts) that
// the behavioural testbenches drive directly. This wrapper is the board-level
// layer that adapts that port to the Xilinx HBM IP:
//
//   1. Reset gating — the HBM controller reports readiness through
//      `apb_complete`. The core must not issue a single AXI transaction before
//      that, so `sys_rst_n & hbm_init_done` is synchronised to `sys_clk` and
//      used as the core reset. This is the single most common cause of a
//      design that synthesises cleanly but reads back zeros on the board.
//
//   2. Address width adaptation — the core drives 34 bits, a 4 GB HBM stack
//      AXI port takes 33 (AXI_ADDR_WIDTH). The design only ever addresses
//      DATASET_BASE = 0x0_0000_0000 (~2.08 GiB) and CACHE_BASE =
//      0x0_C000_0000 (256 MiB), so the truncated high bits are always zero.
//      That assumption is *checked in hardware*: any address with a non-zero
//      truncated bit sets the sticky `hbm_addr_err` flag, which is also
//      exported so a host can read it back.
//
//   3. Optional HBM IP instantiation — guarded by `HBM_IP`. When the macro is
//      not defined (simulation, lint, `make syntax`) the AXI port is exposed
//      at the wrapper boundary so the existing behavioural HBM models can be
//      reused. When `HBM_IP` is defined (Vivado synthesis, see
//      vivado/build.tcl) the wrapper instead drives the generated `hbm_0` and
//      `axi_protocol_converter_0` instances.
//
// Burst compatibility note: `cache_hbm_if` issues 32-beat bursts (one 1 KiB
// Argon2d block) while the HBM IP AXI slave port is AXI3-style with a 4-bit
// AWLEN/ARLEN (16 beats maximum). An AXI Protocol Converter (AXI4 slave →
// AXI3 master) sits between this wrapper and the HBM IP and splits them; see
// vivado/build.tcl. `hbm_dataset_if` only issues 2-beat bursts and is
// unaffected.
//
// Verilog-2001 compliant. No vendor IP is referenced unless `HBM_IP` is set.
// =============================================================================

`timescale 1ns/1ps

module randomx_hbm_top #(
    // Width of the HBM AXI slave address port. 33 bits = 4 GB stack.
    parameter AXI_ADDR_WIDTH = 33,
    // Reset synchroniser depth (release is synchronous, assertion is async)
    parameter RST_SYNC_STAGES = 3
) (
    input  wire         sys_clk,     // 300 MHz core clock
    input  wire         sys_rst_n,   // Active-low board reset

    // --- Control/status register interface (see randomx_top.v) ---
    input  wire         reg_wr_en,
    input  wire [7:0]   reg_wr_addr,
    input  wire [31:0]  reg_wr_data,
    input  wire         reg_rd_en,
    input  wire [7:0]   reg_rd_addr,
    output wire [31:0]  reg_rd_data,

    // Sticky flag: an AXI address did not fit in AXI_ADDR_WIDTH bits
    output reg          hbm_addr_err,

`ifdef HBM_IP
    // --- HBM IP clocks (the AXI port itself is internal in this mode) ---
    input  wire         hbm_ref_clk,   // 100 MHz HBM reference clock
    input  wire         hbm_apb_clk,   // 100 MHz APB configuration clock
    output wire         hbm_init_done, // mirrors apb_complete
    output wire         hbm_cattrip    // catastrophic temperature trip
`else
    // --- HBM AXI4 master, to be connected to the HBM IP / a bus model ---
    input  wire         hbm_init_done, // HBM controller ready (apb_complete)

    output wire [AXI_ADDR_WIDTH-1:0] hbm_axi_araddr,
    output wire [7:0]   hbm_axi_arlen,
    output wire [2:0]   hbm_axi_arsize,
    output wire [1:0]   hbm_axi_arburst,
    output wire         hbm_axi_arvalid,
    input  wire         hbm_axi_arready,
    input  wire [255:0] hbm_axi_rdata,
    input  wire [1:0]   hbm_axi_rresp,
    input  wire         hbm_axi_rlast,
    input  wire         hbm_axi_rvalid,
    output wire         hbm_axi_rready,

    output wire [AXI_ADDR_WIDTH-1:0] hbm_axi_awaddr,
    output wire [7:0]   hbm_axi_awlen,
    output wire [2:0]   hbm_axi_awsize,
    output wire [1:0]   hbm_axi_awburst,
    output wire         hbm_axi_awvalid,
    input  wire         hbm_axi_awready,
    output wire [255:0] hbm_axi_wdata,
    output wire [31:0]  hbm_axi_wstrb,
    output wire         hbm_axi_wlast,
    output wire         hbm_axi_wvalid,
    input  wire         hbm_axi_wready,
    input  wire [1:0]   hbm_axi_bresp,
    input  wire         hbm_axi_bvalid,
    output wire         hbm_axi_bready
`endif
);

// ---------------------------------------------------------------------------
// Core AXI port (34-bit address, as driven by randomx_top)
// ---------------------------------------------------------------------------
wire [33:0]  core_araddr;
wire [7:0]   core_arlen;
wire [2:0]   core_arsize;
wire [1:0]   core_arburst;
wire         core_arvalid;
wire         core_arready;
wire [255:0] core_rdata;
wire [1:0]   core_rresp;
wire         core_rlast;
wire         core_rvalid;
wire         core_rready;

wire [33:0]  core_awaddr;
wire [7:0]   core_awlen;
wire [2:0]   core_awsize;
wire [1:0]   core_awburst;
wire         core_awvalid;
wire         core_awready;
wire [255:0] core_wdata;
wire [31:0]  core_wstrb;
wire         core_wlast;
wire         core_wvalid;
wire         core_wready;
wire [1:0]   core_bresp;
wire         core_bvalid;
wire         core_bready;

// ---------------------------------------------------------------------------
// 1. Reset gating — hold the core in reset until HBM initialisation completed
//
// Asynchronous assertion, synchronous release. `hbm_init_done` comes from the
// HBM IP APB domain, so it is treated as an asynchronous input and passed
// through the same synchroniser.
// ---------------------------------------------------------------------------
wire core_rst_src_n = sys_rst_n & hbm_init_done;

reg [RST_SYNC_STAGES-1:0] rst_sync;

always @(posedge sys_clk or negedge core_rst_src_n) begin
    if (!core_rst_src_n)
        rst_sync <= {RST_SYNC_STAGES{1'b0}};
    else
        rst_sync <= {rst_sync[RST_SYNC_STAGES-2:0], 1'b1};
end

wire core_rst_n = rst_sync[RST_SYNC_STAGES-1];

// ---------------------------------------------------------------------------
// 2. Address width adaptation, with a hardware check on the truncated bits
//
// The design only addresses DATASET_BASE (0) and CACHE_BASE (0xC000_0000), so
// bits [33:AXI_ADDR_WIDTH] are always zero. Rather than silently dropping
// them, flag any violation so a mis-planned address map is caught on the
// board instead of showing up as silent data corruption.
// ---------------------------------------------------------------------------
wire [33:0] ar_hi_mask = ~(({34{1'b1}}) >> (34 - AXI_ADDR_WIDTH));
wire ar_addr_ovf = core_arvalid && |(core_araddr & ar_hi_mask);
wire aw_addr_ovf = core_awvalid && |(core_awaddr & ar_hi_mask);

always @(posedge sys_clk or negedge core_rst_n) begin
    if (!core_rst_n)
        hbm_addr_err <= 1'b0;
    else if (ar_addr_ovf || aw_addr_ovf)
        hbm_addr_err <= 1'b1;
end

wire [AXI_ADDR_WIDTH-1:0] axi_araddr = core_araddr[AXI_ADDR_WIDTH-1:0];
wire [AXI_ADDR_WIDTH-1:0] axi_awaddr = core_awaddr[AXI_ADDR_WIDTH-1:0];

// ---------------------------------------------------------------------------
// 3. RandomX core
// ---------------------------------------------------------------------------
randomx_top u_core (
    .clk           (sys_clk),
    .rst_n         (core_rst_n),
    .reg_wr_en     (reg_wr_en),
    .reg_wr_addr   (reg_wr_addr),
    .reg_wr_data   (reg_wr_data),
    .reg_rd_en     (reg_rd_en),
    .reg_rd_addr   (reg_rd_addr),
    .reg_rd_data   (reg_rd_data),
    .m_axi_araddr  (core_araddr),
    .m_axi_arlen   (core_arlen),
    .m_axi_arsize  (core_arsize),
    .m_axi_arburst (core_arburst),
    .m_axi_arvalid (core_arvalid),
    .m_axi_arready (core_arready),
    .m_axi_rdata   (core_rdata),
    .m_axi_rresp   (core_rresp),
    .m_axi_rlast   (core_rlast),
    .m_axi_rvalid  (core_rvalid),
    .m_axi_rready  (core_rready),
    .m_axi_awaddr  (core_awaddr),
    .m_axi_awlen   (core_awlen),
    .m_axi_awsize  (core_awsize),
    .m_axi_awburst (core_awburst),
    .m_axi_awvalid (core_awvalid),
    .m_axi_awready (core_awready),
    .m_axi_wdata   (core_wdata),
    .m_axi_wstrb   (core_wstrb),
    .m_axi_wlast   (core_wlast),
    .m_axi_wvalid  (core_wvalid),
    .m_axi_wready  (core_wready),
    .m_axi_bresp   (core_bresp),
    .m_axi_bvalid  (core_bvalid),
    .m_axi_bready  (core_bready)
);

`ifdef HBM_IP
// ---------------------------------------------------------------------------
// 4a. Vendor build: AXI Protocol Converter (AXI4 → AXI3) + HBM IP
//
// Both IPs are created by vivado/build.tcl. The port names below follow the
// Xilinx HBM IP (single stack, AXI port 00, Global Addressing enabled) and the
// AXI Protocol Converter defaults; re-check them against the generated IP if
// the Vivado version or the IP configuration changes.
//
// The protocol converter is what makes the 32-beat cache_hbm_if bursts legal
// on the AXI3-style HBM slave port (4-bit AWLEN/ARLEN).
// ---------------------------------------------------------------------------
wire [AXI_ADDR_WIDTH-1:0] pc_araddr;
wire [3:0]   pc_arlen;
wire [2:0]   pc_arsize;
wire [1:0]   pc_arburst;
wire         pc_arvalid;
wire         pc_arready;
wire [255:0] pc_rdata;
wire [1:0]   pc_rresp;
wire         pc_rlast;
wire         pc_rvalid;
wire         pc_rready;

wire [AXI_ADDR_WIDTH-1:0] pc_awaddr;
wire [3:0]   pc_awlen;
wire [2:0]   pc_awsize;
wire [1:0]   pc_awburst;
wire         pc_awvalid;
wire         pc_awready;
wire [255:0] pc_wdata;
wire [31:0]  pc_wstrb;
wire         pc_wlast;
wire         pc_wvalid;
wire         pc_wready;
wire [1:0]   pc_bresp;
wire         pc_bvalid;
wire         pc_bready;

axi_protocol_converter_0 u_axi4_to_axi3 (
    .aclk           (sys_clk),
    .aresetn        (core_rst_n),
    // AXI4 slave side — from the RandomX core
    .s_axi_awid     (1'b0),
    .s_axi_awaddr   (axi_awaddr),
    .s_axi_awlen    (core_awlen),
    .s_axi_awsize   (core_awsize),
    .s_axi_awburst  (core_awburst),
    .s_axi_awlock   (1'b0),
    .s_axi_awcache  (4'b0011),
    .s_axi_awprot   (3'b000),
    .s_axi_awqos    (4'b0000),
    .s_axi_awvalid  (core_awvalid),
    .s_axi_awready  (core_awready),
    .s_axi_wdata    (core_wdata),
    .s_axi_wstrb    (core_wstrb),
    .s_axi_wlast    (core_wlast),
    .s_axi_wvalid   (core_wvalid),
    .s_axi_wready   (core_wready),
    .s_axi_bid      (),
    .s_axi_bresp    (core_bresp),
    .s_axi_bvalid   (core_bvalid),
    .s_axi_bready   (core_bready),
    .s_axi_arid     (1'b0),
    .s_axi_araddr   (axi_araddr),
    .s_axi_arlen    (core_arlen),
    .s_axi_arsize   (core_arsize),
    .s_axi_arburst  (core_arburst),
    .s_axi_arlock   (1'b0),
    .s_axi_arcache  (4'b0011),
    .s_axi_arprot   (3'b000),
    .s_axi_arqos    (4'b0000),
    .s_axi_arvalid  (core_arvalid),
    .s_axi_arready  (core_arready),
    .s_axi_rid      (),
    .s_axi_rdata    (core_rdata),
    .s_axi_rresp    (core_rresp),
    .s_axi_rlast    (core_rlast),
    .s_axi_rvalid   (core_rvalid),
    .s_axi_rready   (core_rready),
    // AXI3 master side — to the HBM IP
    .m_axi_awid     (),
    .m_axi_awaddr   (pc_awaddr),
    .m_axi_awlen    (pc_awlen),
    .m_axi_awsize   (pc_awsize),
    .m_axi_awburst  (pc_awburst),
    .m_axi_awlock   (),
    .m_axi_awcache  (),
    .m_axi_awprot   (),
    .m_axi_awqos    (),
    .m_axi_awvalid  (pc_awvalid),
    .m_axi_awready  (pc_awready),
    .m_axi_wid      (),
    .m_axi_wdata    (pc_wdata),
    .m_axi_wstrb    (pc_wstrb),
    .m_axi_wlast    (pc_wlast),
    .m_axi_wvalid   (pc_wvalid),
    .m_axi_wready   (pc_wready),
    .m_axi_bid      (1'b0),
    .m_axi_bresp    (pc_bresp),
    .m_axi_bvalid   (pc_bvalid),
    .m_axi_bready   (pc_bready),
    .m_axi_arid     (),
    .m_axi_araddr   (pc_araddr),
    .m_axi_arlen    (pc_arlen),
    .m_axi_arsize   (pc_arsize),
    .m_axi_arburst  (pc_arburst),
    .m_axi_arlock   (),
    .m_axi_arcache  (),
    .m_axi_arprot   (),
    .m_axi_arqos    (),
    .m_axi_arvalid  (pc_arvalid),
    .m_axi_arready  (pc_arready),
    .m_axi_rid      (1'b0),
    .m_axi_rdata    (pc_rdata),
    .m_axi_rresp    (pc_rresp),
    .m_axi_rlast    (pc_rlast),
    .m_axi_rvalid   (pc_rvalid),
    .m_axi_rready   (pc_rready)
);

hbm_0 u_hbm (
    .HBM_REF_CLK_0    (hbm_ref_clk),
    .APB_0_PCLK       (hbm_apb_clk),
    .APB_0_PRESET_N   (sys_rst_n),
    .apb_complete_0   (hbm_init_done),
    .DRAM_0_STAT_CATTRIP (hbm_cattrip),
    .DRAM_0_STAT_TEMP    (),

    .AXI_00_ACLK      (sys_clk),
    .AXI_00_ARESET_N  (core_rst_n),

    .AXI_00_ARADDR    (pc_araddr),
    .AXI_00_ARBURST   (pc_arburst),
    .AXI_00_ARID      (6'b0),
    .AXI_00_ARLEN     (pc_arlen),
    .AXI_00_ARSIZE    (pc_arsize),
    .AXI_00_ARVALID   (pc_arvalid),
    .AXI_00_ARREADY   (pc_arready),
    .AXI_00_RDATA     (pc_rdata),
    .AXI_00_RDATA_PARITY (),
    .AXI_00_RID       (),
    .AXI_00_RLAST     (pc_rlast),
    .AXI_00_RRESP     (pc_rresp),
    .AXI_00_RVALID    (pc_rvalid),
    .AXI_00_RREADY    (pc_rready),

    .AXI_00_AWADDR    (pc_awaddr),
    .AXI_00_AWBURST   (pc_awburst),
    .AXI_00_AWID      (6'b0),
    .AXI_00_AWLEN     (pc_awlen),
    .AXI_00_AWSIZE    (pc_awsize),
    .AXI_00_AWVALID   (pc_awvalid),
    .AXI_00_AWREADY   (pc_awready),
    .AXI_00_WDATA     (pc_wdata),
    .AXI_00_WDATA_PARITY (32'b0),
    .AXI_00_WSTRB     (pc_wstrb),
    .AXI_00_WLAST     (pc_wlast),
    .AXI_00_WVALID    (pc_wvalid),
    .AXI_00_WREADY    (pc_wready),
    .AXI_00_BID       (),
    .AXI_00_BRESP     (pc_bresp),
    .AXI_00_BVALID    (pc_bvalid),
    .AXI_00_BREADY    (pc_bready)
);

`else
// ---------------------------------------------------------------------------
// 4b. Simulation / lint build: expose the adapted AXI port
// ---------------------------------------------------------------------------
assign hbm_axi_araddr  = axi_araddr;
assign hbm_axi_arlen   = core_arlen;
assign hbm_axi_arsize  = core_arsize;
assign hbm_axi_arburst = core_arburst;
assign hbm_axi_arvalid = core_arvalid;
assign core_arready    = hbm_axi_arready;
assign core_rdata      = hbm_axi_rdata;
assign core_rresp      = hbm_axi_rresp;
assign core_rlast      = hbm_axi_rlast;
assign core_rvalid     = hbm_axi_rvalid;
assign hbm_axi_rready  = core_rready;

assign hbm_axi_awaddr  = axi_awaddr;
assign hbm_axi_awlen   = core_awlen;
assign hbm_axi_awsize  = core_awsize;
assign hbm_axi_awburst = core_awburst;
assign hbm_axi_awvalid = core_awvalid;
assign core_awready    = hbm_axi_awready;
assign hbm_axi_wdata   = core_wdata;
assign hbm_axi_wstrb   = core_wstrb;
assign hbm_axi_wlast   = core_wlast;
assign hbm_axi_wvalid  = core_wvalid;
assign core_wready     = hbm_axi_wready;
assign core_bresp      = hbm_axi_bresp;
assign core_bvalid     = hbm_axi_bvalid;
assign hbm_axi_bready  = core_bready;
`endif

endmodule
