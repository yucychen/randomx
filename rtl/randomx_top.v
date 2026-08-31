// =============================================================================
// randomx_top.v — RandomX Top-level Module
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Target: Xilinx Virtex UltraScale+ XCVU33P (part xcvu33p-fsvh2104-2L-e)
// Tool:   Vivado 2022.x or later
//
// Interface: AXI-Lite style control/status registers (simplified, not full AXI)
//   Write 0x00..0x3C ← seed[511:0] (16 × 32-bit writes)
//   Write 0x40       ← control[0] = start
//   Read  0x44       → status[0] = done, status[1] = AXI error
//   Read  0x48..0x84 → hash_out[511:0] (16 × 32-bit reads)
//   Write 0x88       ← Argon2 key length in bytes (1..64, defaults to 64)
//   Write 0x8C/0x90  ← SuperscalarHash program word (low / high 32 bits)
//   Write 0x94       ← SuperscalarHash program buffer index → commits the word
//   Write 0x98       ← SuperscalarHash program config
//                      ([2:0] program, [15:4] length, [18:16] address register)
//
// Main FSM:
//   IDLE → CACHE_INIT (Argon2d) → DS_GEN (SuperscalarHash dataset items) →
//   SP_FILL (AesGenerator1R scratchpad fill) →
//   PROG_GEN (AesGenerator4R program + entropy) → VM_RUN →
//   FINAL_HASH (Blake2b-256 of the VM's AesHash1R result) → DONE
//
// All sub-modules use synchronous active-low reset (rst_n), single clock (clk).
//
// Verilog-2001 compliant, no vendor IP instantiations.
// =============================================================================

`timescale 1ns/1ps

module randomx_top (
    input  wire         clk,      // System clock (300 MHz target)
    input  wire         rst_n,    // Active-low synchronous reset

    // --- AXI-Lite style control/status register interface ---
    // (simplified: no handshake signals, single-cycle access)
    input  wire         reg_wr_en,
    input  wire [7:0]   reg_wr_addr, // byte address (word-aligned, 4-byte words)
    input  wire [31:0]  reg_wr_data,

    input  wire         reg_rd_en,
    input  wire [7:0]   reg_rd_addr,
    output reg  [31:0]  reg_rd_data,

    // --- AXI4 HBM Master interface (passed through to hbm_dataset_if) ---
    output wire [33:0]  m_axi_araddr,
    output wire [7:0]   m_axi_arlen,
    output wire [2:0]   m_axi_arsize,
    output wire [1:0]   m_axi_arburst,
    output wire         m_axi_arvalid,
    input  wire         m_axi_arready,
    input  wire [255:0] m_axi_rdata,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rlast,
    input  wire         m_axi_rvalid,
    output wire         m_axi_rready,
    // Write channels (dataset generation — driver TODO: superscalar_hash)
    output wire [33:0]  m_axi_awaddr,
    output wire [7:0]   m_axi_awlen,
    output wire [2:0]   m_axi_awsize,
    output wire [1:0]   m_axi_awburst,
    output wire         m_axi_awvalid,
    input  wire         m_axi_awready,
    output wire [255:0] m_axi_wdata,
    output wire [31:0]  m_axi_wstrb,
    output wire         m_axi_wlast,
    output wire         m_axi_wvalid,
    input  wire         m_axi_wready,
    input  wire [1:0]   m_axi_bresp,
    input  wire         m_axi_bvalid,
    output wire         m_axi_bready
);

// ===========================================================================
// Internal registers / signals
// ===========================================================================

// Seed register (512 bits = 64 bytes, written via 16 × 32-bit register writes)
reg [511:0] seed_reg;
// Argon2 key length in bytes (RandomX uses the full seed by default)
reg [15:0]  key_len_reg;

// Control / status
reg         start_pulse;
reg         busy;
wire        all_done;

// Hash output
reg [511:0] hash_out;

// Top-level FSM states
localparam FSM_IDLE       = 4'd0;
localparam FSM_CACHE_INIT = 4'd1;   // Argon2d cache fill
localparam FSM_DS_GEN     = 4'd2;   // SuperscalarHash dataset generation
localparam FSM_SP_FILL    = 4'd3;   // AesGenerator1R scratchpad fill
localparam FSM_PROG_GEN   = 4'd4;   // AesGenerator4R program + entropy
localparam FSM_VM_RUN     = 4'd5;   // RandomX VM execution
localparam FSM_FINAL_HASH = 4'd6;   // Blake2b finalization
localparam FSM_DONE       = 4'd7;

reg [3:0] fsm_state;

// ===========================================================================
// Sub-module wires
// ===========================================================================

// --- Argon2d ---
wire        argon2_done;
reg         argon2_start;
// Cache access interface (1 KiB blocks, stored in HBM via cache_hbm_if)
wire         argon2_cache_wr_en;
wire [31:0]  argon2_cache_wr_addr;
wire [8191:0] argon2_cache_wr_data;
wire         argon2_cache_wr_rdy;
wire         argon2_cache_rd_en;
wire [31:0]  argon2_cache_rd_addr;
wire [8191:0] argon2_cache_rd_data;
wire         argon2_cache_rd_valid;

// Blake2b (shared between argon2 and the final hash)
wire          b2b_done;
wire          b2b_busy;
wire [511:0]  b2b_h_out;
// Argon2d side
wire          a2_b2b_start, a2_b2b_init, a2_b2b_last;
wire [1023:0] a2_b2b_msg;
wire [127:0]  a2_b2b_byte_cnt;
wire [511:0]  a2_b2b_h_in;
// Final-hash side (driven by the top-level FSM)
reg           fh_b2b_start;
// Muxed core inputs — the two users are active in different FSM phases
wire          b2b_final_sel = (fsm_state == FSM_FINAL_HASH);
wire          b2b_start     = b2b_final_sel ? fh_b2b_start : a2_b2b_start;
wire          b2b_init      = b2b_final_sel ? 1'b0         : a2_b2b_init;
wire          b2b_last      = b2b_final_sel ? 1'b1         : a2_b2b_last;
wire [1023:0] b2b_msg       = b2b_final_sel ? {512'b0, vm_hash_out} : a2_b2b_msg;
wire [127:0]  b2b_byte_cnt  = b2b_final_sel ? 128'd64      : a2_b2b_byte_cnt;
wire [511:0]  b2b_h_in      = b2b_final_sel ? B2B256_IV    : a2_b2b_h_in;

// --- Scratchpad memory (written by the AesGenerator1R fill, then by the VM) ---
wire        sp_rd_en;
wire [20:0] sp_rd_addr;
wire [1:0]  sp_rd_level;
wire [63:0] sp_rd_data;
wire        sp_rd_valid;
wire        vm_sp_wr_en;
wire [20:0] vm_sp_wr_addr;
wire [1:0]  vm_sp_wr_level;
wire [63:0] vm_sp_wr_data;
// Scratchpad fill engine (AesGenerator1R)
reg         sp_fill_start, sp_fill_busy, sp_fill_done;
reg         fill_wr_en;
reg  [20:0] fill_wr_addr;
reg  [63:0] fill_wr_data;
wire        sp_wr_en    = sp_fill_busy ? fill_wr_en   : vm_sp_wr_en;
wire [20:0] sp_wr_addr  = sp_fill_busy ? fill_wr_addr : vm_sp_wr_addr;
wire [1:0]  sp_wr_level = sp_fill_busy ? 2'd2         : vm_sp_wr_level;
wire [63:0] sp_wr_data  = sp_fill_busy ? fill_wr_data : vm_sp_wr_data;

// --- HBM Dataset IF ---
wire        ds_req_valid, ds_req_ready;
wire [31:0] ds_req_idx;
wire [511:0] ds_resp_data;
wire        ds_resp_valid, ds_resp_ready;

// --- AES Hash ---
wire        aes_start;
wire [511:0] aes_data_in;
wire [511:0] aes_hash_out;
wire        aes_hash_valid;

// --- VM ---
wire        vm_done;
reg         vm_start;
wire [511:0] vm_hash_out;
wire        vm_aes_start;
wire [511:0] vm_aes_data_in;

// --- Program / entropy generator (AesGenerator4R) ---
reg         pg_start;
wire        pg_done;
wire        pg_prog_wr_en;
wire [7:0]  pg_prog_wr_addr;
wire [63:0] pg_prog_wr_data;
wire        pg_cfg_wr_en;
wire [3:0]  pg_cfg_wr_addr;
wire [63:0] pg_cfg_wr_data;

// --- Dataset generator (SuperscalarHash) ---
reg          dsg_start;
wire         dsg_done;
wire         dsg_busy;
wire         dsg_cache_rd_en;
wire [31:0]  dsg_cache_rd_addr;
wire         dsg_wr_valid;
wire [31:0]  dsg_wr_item_idx;
wire [511:0] dsg_wr_data;
wire         dsg_wr_ready;

// SuperscalarHash program load registers (written through the register file)
reg  [63:0] ss_prog_data;
reg         ss_prog_wr_en;
reg  [11:0] ss_prog_wr_addr;
reg         ss_cfg_wr_en;
reg  [2:0]  ss_cfg_sel;
reg  [11:0] ss_cfg_len;
reg  [2:0]  ss_cfg_addr_reg;

// Blake2b-256 parameter-block IV (digest length 32, unkeyed, fanout/depth 1).
// The shared blake2b_core is parameterised for a 64-byte digest, so the
// 32-byte parameter block is supplied through h_in for the final hash.
localparam [63:0] B2B256_H0 = 64'h6a09e667f3bcc908 ^ 64'h0000000001010020;
localparam [511:0] B2B256_IV = {64'h5be0cd19137e2179, 64'h1f83d9abfb41bd6b,
                                64'h9b05688c2b3e6c1f, 64'h510e527fade682d1,
                                64'ha54ff53a5f1d36f1, 64'h3c6ef372fe94f82b,
                                64'hbb67ae8584caa73b, B2B256_H0};

// Scratchpad size in 64-bit words (matches scratchpad_mem / randomx_vm)
`ifdef SIMULATION
localparam [20:0] SP_WORDS_TOP = 21'd4096;
localparam        DS_ITEM_COUNT = 8;
localparam        DS_CACHE_LINES = 128;      // 8 KiB cache / 64 B
`else
localparam [20:0] SP_WORDS_TOP = 21'd262144;
localparam        DS_ITEM_COUNT = 34078720;  // RANDOMX_DATASET_ITEM_COUNT
localparam        DS_CACHE_LINES = 4194304;  // 256 MiB / 64 B
`endif

// ===========================================================================
// Sub-module instantiations
// ===========================================================================

// --- Blake2b core (shared) ---
blake2b_core u_blake2b (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (b2b_start),
    .init       (b2b_init),
    .last_block (b2b_last),
    .msg_block  (b2b_msg),
    .byte_count (b2b_byte_cnt),
    .h_in       (b2b_h_in),
    .h_out      (b2b_h_out),
    .busy       (b2b_busy),
    .done       (b2b_done)
);

// --- Argon2d cache fill ---
argon2_fill #(
`ifdef SIMULATION
    // Simulation build: reduce the Argon2 memory cost to 8 blocks (8 KiB)
    .ARGON_M (8),
`else
    .ARGON_M (262144),
`endif
    .ARGON_T (3),
    .KEY_BYTES (64)
) u_argon2 (
    .clk            (clk),
    .rst_n          (rst_n),
    .start          (argon2_start),
    .key            (seed_reg),
    .key_len        (key_len_reg),
    .cache_wr_en    (argon2_cache_wr_en),
    .cache_wr_addr  (argon2_cache_wr_addr),
    .cache_wr_data  (argon2_cache_wr_data),
    .cache_wr_rdy   (argon2_cache_wr_rdy),
    .cache_rd_en    (argon2_cache_rd_en),
    .cache_rd_addr  (argon2_cache_rd_addr),
    .cache_rd_data  (argon2_cache_rd_data),
    .cache_rd_valid (argon2_cache_rd_valid),
    .done           (argon2_done),
    .b2b_start      (a2_b2b_start),
    .b2b_init       (a2_b2b_init),
    .b2b_msg        (a2_b2b_msg),
    .b2b_byte_cnt   (a2_b2b_byte_cnt),
    .b2b_h_in       (a2_b2b_h_in),
    .b2b_last       (a2_b2b_last),
    .b2b_h_out      (b2b_h_out),
    .b2b_busy       (b2b_busy),
    .b2b_done       (b2b_done)
);

// --- Scratchpad (2 MiB, URAM) ---
scratchpad_mem u_scratchpad (
    .clk      (clk),
    .rst_n    (rst_n),
    .wr_en    (sp_wr_en),
    .wr_addr  (sp_wr_addr),
    .wr_data  (sp_wr_data),
    .wr_level (sp_wr_level),
    .rd_en    (sp_rd_en),
    .rd_addr  (sp_rd_addr),
    .rd_level (sp_rd_level),
    .rd_data  (sp_rd_data),
    .rd_valid (sp_rd_valid)
);

// ===========================================================================
// HBM address map (34-bit byte addresses inside the 8 GB HBM stack)
//   0x0_0000_0000 .. 0x0_8500_0000 : Dataset (~2.08 GiB, 64-byte items)
//   0x0_C000_0000 .. 0x0_D000_0000 : Argon2d Cache (256 MiB, 1 KiB blocks)
// Both regions are accessed through one AXI4 port, shared by an arbiter.
// ===========================================================================
localparam DATASET_BASE = 34'h0_0000_0000;
localparam CACHE_BASE   = 34'h0_C000_0000;

// --- Cache storage (Argon2d 1 KiB blocks, backed by HBM) ---
wire cache_axi_err; // sticky AXI error status from the cache interface

wire [33:0]  c_axi_araddr;
wire [7:0]   c_axi_arlen;
wire [2:0]   c_axi_arsize;
wire [1:0]   c_axi_arburst;
wire         c_axi_arvalid, c_axi_arready;
wire [255:0] c_axi_rdata;
wire [1:0]   c_axi_rresp;
wire         c_axi_rlast, c_axi_rvalid, c_axi_rready;
wire [33:0]  c_axi_awaddr;
wire [7:0]   c_axi_awlen;
wire [2:0]   c_axi_awsize;
wire [1:0]   c_axi_awburst;
wire         c_axi_awvalid, c_axi_awready;
wire [255:0] c_axi_wdata;
wire [31:0]  c_axi_wstrb;
wire         c_axi_wlast, c_axi_wvalid, c_axi_wready;
wire [1:0]   c_axi_bresp;
wire         c_axi_bvalid, c_axi_bready;

// The cache read port is shared: `argon2_fill` owns it while the cache is
// being built, the dataset generator owns it afterwards. The two phases never
// overlap, so a plain ownership mux is sufficient.
wire          cache_rd_en_mux   = dsg_busy ? dsg_cache_rd_en   : argon2_cache_rd_en;
wire [31:0]   cache_rd_addr_mux = dsg_busy ? dsg_cache_rd_addr : argon2_cache_rd_addr;
wire [8191:0] cache_rd_data_mux;
wire          cache_rd_valid_mux;

assign argon2_cache_rd_data  = cache_rd_data_mux;
assign argon2_cache_rd_valid = cache_rd_valid_mux & ~dsg_busy;

cache_hbm_if #(
    .AXI_ADDR_WIDTH  (34),
    .AXI_DATA_WIDTH  (256),
    .AXI_ID_WIDTH    (6),
    .CACHE_BASE_ADDR (CACHE_BASE)
) u_cache (
    .clk            (clk),
    .rst_n          (rst_n),
    .wr_en          (argon2_cache_wr_en),
    .wr_addr        (argon2_cache_wr_addr),
    .wr_data        (argon2_cache_wr_data),
    .wr_rdy         (argon2_cache_wr_rdy),
    .rd_en          (cache_rd_en_mux),
    .rd_addr        (cache_rd_addr_mux),
    .rd_data        (cache_rd_data_mux),
    .rd_valid       (cache_rd_valid_mux),
    .axi_err        (cache_axi_err),
    .m_axi_arid     (),                // ID not connected to top
    .m_axi_araddr   (c_axi_araddr),
    .m_axi_arlen    (c_axi_arlen),
    .m_axi_arsize   (c_axi_arsize),
    .m_axi_arburst  (c_axi_arburst),
    .m_axi_arvalid  (c_axi_arvalid),
    .m_axi_arready  (c_axi_arready),
    .m_axi_rdata    (c_axi_rdata),
    .m_axi_rresp    (c_axi_rresp),
    .m_axi_rlast    (c_axi_rlast),
    .m_axi_rvalid   (c_axi_rvalid),
    .m_axi_rready   (c_axi_rready),
    .m_axi_awid     (),
    .m_axi_awaddr   (c_axi_awaddr),
    .m_axi_awlen    (c_axi_awlen),
    .m_axi_awsize   (c_axi_awsize),
    .m_axi_awburst  (c_axi_awburst),
    .m_axi_awvalid  (c_axi_awvalid),
    .m_axi_awready  (c_axi_awready),
    .m_axi_wdata    (c_axi_wdata),
    .m_axi_wstrb    (c_axi_wstrb),
    .m_axi_wlast    (c_axi_wlast),
    .m_axi_wvalid   (c_axi_wvalid),
    .m_axi_wready   (c_axi_wready),
    .m_axi_bresp    (c_axi_bresp),
    .m_axi_bvalid   (c_axi_bvalid),
    .m_axi_bready   (c_axi_bready)
);

// --- HBM AXI4 master interface (dataset) ---
wire hbm_axi_err;   // sticky AXI error status from the dataset interface

wire [33:0]  d_axi_araddr;
wire [7:0]   d_axi_arlen;
wire [2:0]   d_axi_arsize;
wire [1:0]   d_axi_arburst;
wire         d_axi_arvalid, d_axi_arready;
wire [255:0] d_axi_rdata;
wire [1:0]   d_axi_rresp;
wire         d_axi_rlast, d_axi_rvalid, d_axi_rready;
wire [33:0]  d_axi_awaddr;
wire [7:0]   d_axi_awlen;
wire [2:0]   d_axi_awsize;
wire [1:0]   d_axi_awburst;
wire         d_axi_awvalid, d_axi_awready;
wire [255:0] d_axi_wdata;
wire [31:0]  d_axi_wstrb;
wire         d_axi_wlast, d_axi_wvalid, d_axi_wready;
wire [1:0]   d_axi_bresp;
wire         d_axi_bvalid, d_axi_bready;

hbm_dataset_if #(
    .AXI_ADDR_WIDTH    (34),
    .AXI_DATA_WIDTH    (256),
    .AXI_ID_WIDTH      (6),
    .DATASET_BASE_ADDR (DATASET_BASE)
) u_hbm (
    .clk            (clk),
    .rst_n          (rst_n),
    .req_valid      (ds_req_valid),
    .req_item_idx   (ds_req_idx),
    .req_ready      (ds_req_ready),
    .resp_valid     (ds_resp_valid),
    .resp_data      (ds_resp_data),
    .resp_err       (),                // dataset read error (see axi_err)
    .resp_ready     (ds_resp_ready),
    .wr_req_valid   (dsg_wr_valid),    // driven by the dataset generator
    .wr_req_item_idx(dsg_wr_item_idx),
    .wr_req_data    (dsg_wr_data),
    .wr_req_ready   (dsg_wr_ready),
    .wr_done        (),
    .wr_err         (),
    .axi_err        (hbm_axi_err),
    .m_axi_arid     (),                // ID not connected to top
    .m_axi_araddr   (d_axi_araddr),
    .m_axi_arlen    (d_axi_arlen),
    .m_axi_arsize   (d_axi_arsize),
    .m_axi_arburst  (d_axi_arburst),
    .m_axi_arvalid  (d_axi_arvalid),
    .m_axi_arready  (d_axi_arready),
    .m_axi_rid      (6'b0),
    .m_axi_rdata    (d_axi_rdata),
    .m_axi_rresp    (d_axi_rresp),
    .m_axi_rlast    (d_axi_rlast),
    .m_axi_rvalid   (d_axi_rvalid),
    .m_axi_rready   (d_axi_rready),
    .m_axi_awid     (),
    .m_axi_awaddr   (d_axi_awaddr),
    .m_axi_awlen    (d_axi_awlen),
    .m_axi_awsize   (d_axi_awsize),
    .m_axi_awburst  (d_axi_awburst),
    .m_axi_awvalid  (d_axi_awvalid),
    .m_axi_awready  (d_axi_awready),
    .m_axi_wdata    (d_axi_wdata),
    .m_axi_wstrb    (d_axi_wstrb),
    .m_axi_wlast    (d_axi_wlast),
    .m_axi_wvalid   (d_axi_wvalid),
    .m_axi_wready   (d_axi_wready),
    .m_axi_bid      (6'b0),
    .m_axi_bresp    (d_axi_bresp),
    .m_axi_bvalid   (d_axi_bvalid),
    .m_axi_bready   (d_axi_bready)
);

// --- AXI arbiter: cache (M0) + dataset (M1) share the single HBM port ---
axi_arbiter #(
    .AXI_ADDR_WIDTH  (34),
    .AXI_DATA_WIDTH  (256),
    .MAX_OUTSTANDING (16)
) u_axi_arb (
    .clk        (clk),
    .rst_n      (rst_n),
    .m0_araddr  (c_axi_araddr),
    .m0_arlen   (c_axi_arlen),
    .m0_arsize  (c_axi_arsize),
    .m0_arburst (c_axi_arburst),
    .m0_arvalid (c_axi_arvalid),
    .m0_arready (c_axi_arready),
    .m0_rdata   (c_axi_rdata),
    .m0_rresp   (c_axi_rresp),
    .m0_rlast   (c_axi_rlast),
    .m0_rvalid  (c_axi_rvalid),
    .m0_rready  (c_axi_rready),
    .m0_awaddr  (c_axi_awaddr),
    .m0_awlen   (c_axi_awlen),
    .m0_awsize  (c_axi_awsize),
    .m0_awburst (c_axi_awburst),
    .m0_awvalid (c_axi_awvalid),
    .m0_awready (c_axi_awready),
    .m0_wdata   (c_axi_wdata),
    .m0_wstrb   (c_axi_wstrb),
    .m0_wlast   (c_axi_wlast),
    .m0_wvalid  (c_axi_wvalid),
    .m0_wready  (c_axi_wready),
    .m0_bresp   (c_axi_bresp),
    .m0_bvalid  (c_axi_bvalid),
    .m0_bready  (c_axi_bready),
    .m1_araddr  (d_axi_araddr),
    .m1_arlen   (d_axi_arlen),
    .m1_arsize  (d_axi_arsize),
    .m1_arburst (d_axi_arburst),
    .m1_arvalid (d_axi_arvalid),
    .m1_arready (d_axi_arready),
    .m1_rdata   (d_axi_rdata),
    .m1_rresp   (d_axi_rresp),
    .m1_rlast   (d_axi_rlast),
    .m1_rvalid  (d_axi_rvalid),
    .m1_rready  (d_axi_rready),
    .m1_awaddr  (d_axi_awaddr),
    .m1_awlen   (d_axi_awlen),
    .m1_awsize  (d_axi_awsize),
    .m1_awburst (d_axi_awburst),
    .m1_awvalid (d_axi_awvalid),
    .m1_awready (d_axi_awready),
    .m1_wdata   (d_axi_wdata),
    .m1_wstrb   (d_axi_wstrb),
    .m1_wlast   (d_axi_wlast),
    .m1_wvalid  (d_axi_wvalid),
    .m1_wready  (d_axi_wready),
    .m1_bresp   (d_axi_bresp),
    .m1_bvalid  (d_axi_bvalid),
    .m1_bready  (d_axi_bready),
    .s_araddr   (m_axi_araddr),
    .s_arlen    (m_axi_arlen),
    .s_arsize   (m_axi_arsize),
    .s_arburst  (m_axi_arburst),
    .s_arvalid  (m_axi_arvalid),
    .s_arready  (m_axi_arready),
    .s_rdata    (m_axi_rdata),
    .s_rresp    (m_axi_rresp),
    .s_rlast    (m_axi_rlast),
    .s_rvalid   (m_axi_rvalid),
    .s_rready   (m_axi_rready),
    .s_awaddr   (m_axi_awaddr),
    .s_awlen    (m_axi_awlen),
    .s_awsize   (m_axi_awsize),
    .s_awburst  (m_axi_awburst),
    .s_awvalid  (m_axi_awvalid),
    .s_awready  (m_axi_awready),
    .s_wdata    (m_axi_wdata),
    .s_wstrb    (m_axi_wstrb),
    .s_wlast    (m_axi_wlast),
    .s_wvalid   (m_axi_wvalid),
    .s_wready   (m_axi_wready),
    .s_bresp    (m_axi_bresp),
    .s_bvalid   (m_axi_bvalid),
    .s_bready   (m_axi_bready)
);

// --- AesHash1R ---
aes_hash1r u_aes_hash (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (vm_aes_start),
    .blk_valid(vm_aes_start),
    .blk_last (1'b1),
    .data_in  (vm_aes_data_in),
    .hash_out (aes_hash_out),
    .busy     (),
    .valid    (aes_hash_valid)
);

// --- RandomX VM ---
randomx_vm #(
`ifdef SIMULATION
    // Simulation build: 4 VM iterations and the reduced scratchpad depth of
    // scratchpad_mem (4096 × 64-bit) so the testbench finishes quickly.
    .ITERATIONS (4),
    .SP_WORDS   (4096)
`else
    .ITERATIONS (2048),
    .SP_WORDS   (262144)
`endif
) u_vm (
    .clk           (clk),
    .rst_n         (rst_n),
    .start         (vm_start),
    .prog_wr_en    (pg_prog_wr_en),
    .prog_wr_addr  (pg_prog_wr_addr),
    .prog_wr_data  (pg_prog_wr_data),
    .cfg_wr_en     (pg_cfg_wr_en),
    .cfg_wr_addr   (pg_cfg_wr_addr),
    .cfg_wr_data   (pg_cfg_wr_data),
    .sp_rd_en      (sp_rd_en),
    .sp_rd_addr    (sp_rd_addr),
    .sp_rd_level   (sp_rd_level),
    .sp_rd_data    (sp_rd_data),
    .sp_rd_valid   (sp_rd_valid),
    .sp_wr_en      (vm_sp_wr_en),
    .sp_wr_addr    (vm_sp_wr_addr),
    .sp_wr_level   (vm_sp_wr_level),
    .sp_wr_data    (vm_sp_wr_data),
    .ds_req_valid  (ds_req_valid),
    .ds_req_idx    (ds_req_idx),
    .ds_req_ready  (ds_req_ready),
    .ds_resp_data  (ds_resp_data),
    .ds_resp_valid (ds_resp_valid),
    .ds_resp_ready (ds_resp_ready),
    .aes_start     (vm_aes_start),
    .aes_data_in   (vm_aes_data_in),
    .aes_hash_out  (aes_hash_out),
    .aes_hash_valid(aes_hash_valid),
    .hash_out      (vm_hash_out),
    .done          (vm_done)
);

// --- Dataset generator (8 cache accesses + SuperscalarHash per item) ---
dataset_gen #(
    .CACHE_LINES (DS_CACHE_LINES),
    .ITEM_COUNT  (DS_ITEM_COUNT),
    .ACCESSES    (8)
) u_dsgen (
    .clk             (clk),
    .rst_n           (rst_n),
    .start           (dsg_start),
    .prog_wr_en      (ss_prog_wr_en),
    .prog_wr_addr    (ss_prog_wr_addr),
    .prog_wr_data    (ss_prog_data),
    .cfg_wr_en       (ss_cfg_wr_en),
    .cfg_wr_sel      (ss_cfg_sel),
    .cfg_wr_len      (ss_cfg_len),
    .cfg_wr_addr_reg (ss_cfg_addr_reg),
    .cache_rd_en     (dsg_cache_rd_en),
    .cache_rd_addr   (dsg_cache_rd_addr),
    .cache_rd_data   (cache_rd_data_mux),
    .cache_rd_valid  (cache_rd_valid_mux & dsg_busy),
    .ds_wr_valid     (dsg_wr_valid),
    .ds_wr_item_idx  (dsg_wr_item_idx),
    .ds_wr_data      (dsg_wr_data),
    .ds_wr_ready     (dsg_wr_ready),
    .busy            (dsg_busy),
    .done            (dsg_done)
);

// --- VM program + entropy generator (AesGenerator4R) ---
prog_gen u_prog_gen (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (pg_start),
    .seed_in      (seed_reg),
    .prog_wr_en   (pg_prog_wr_en),
    .prog_wr_addr (pg_prog_wr_addr),
    .prog_wr_data (pg_prog_wr_data),
    .cfg_wr_en    (pg_cfg_wr_en),
    .cfg_wr_addr  (pg_cfg_wr_addr),
    .cfg_wr_data  (pg_cfg_wr_data),
    .state_out    (),
    .busy         (),
    .done         (pg_done)
);

// ===========================================================================
// Scratchpad fill (AesGenerator1R, spec §4.4)
//
// The 2 MiB scratchpad is filled with AesGenerator1R output before the VM
// starts: one AES round per 64-byte block, 8 × 64-bit words written per block.
// ===========================================================================
wire [511:0] g1r_out;
wire         g1r_valid;
reg          g1r_start;
reg  [511:0] sp_fill_state;
reg  [511:0] sp_fill_blk;
reg  [20:0]  sp_fill_word;   // 64-bit word index inside the scratchpad
reg  [2:0]   sp_fill_sub;
reg  [1:0]   sp_fill_st;

localparam SPF_IDLE = 2'd0;
localparam SPF_GEN  = 2'd1;
localparam SPF_EMIT = 2'd2;
localparam SPF_DONE = 2'd3;

aes_gen1r u_gen1r (
    .clk       (clk),
    .rst_n     (rst_n),
    .start     (g1r_start),
    .state_in  (sp_fill_state),
    .state_out (g1r_out),
    .valid     (g1r_valid)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sp_fill_st    <= SPF_IDLE;
        sp_fill_busy  <= 1'b0;
        sp_fill_done  <= 1'b0;
        sp_fill_state <= 512'b0;
        sp_fill_blk   <= 512'b0;
        sp_fill_word  <= 21'd0;
        sp_fill_sub   <= 3'd0;
        g1r_start     <= 1'b0;
        fill_wr_en    <= 1'b0;
        fill_wr_addr  <= 21'd0;
        fill_wr_data  <= 64'b0;
    end else begin
        g1r_start    <= 1'b0;
        sp_fill_done <= 1'b0;
        fill_wr_en   <= 1'b0;

        case (sp_fill_st)
            SPF_IDLE: begin
                sp_fill_busy <= 1'b0;
                if (sp_fill_start) begin
                    sp_fill_state <= seed_reg;
                    g1r_start     <= 1'b1;
                    sp_fill_word  <= 21'd0;
                    sp_fill_busy  <= 1'b1;
                    sp_fill_st    <= SPF_GEN;
                end
            end

            SPF_GEN: begin
                if (g1r_valid) begin
                    sp_fill_blk   <= g1r_out;
                    sp_fill_state <= g1r_out;   // state feeds the next block
                    sp_fill_sub   <= 3'd0;
                    sp_fill_st    <= SPF_EMIT;
                end
            end

            SPF_EMIT: begin
                fill_wr_en   <= 1'b1;
                fill_wr_addr <= {sp_fill_word[17:0], 3'b0}; // byte address
                fill_wr_data <= sp_fill_blk[{sp_fill_sub, 6'b0} +: 64];

                if (sp_fill_word == (SP_WORDS_TOP - 21'd1)) begin
                    sp_fill_st <= SPF_DONE;
                end else begin
                    sp_fill_word <= sp_fill_word + 21'd1;
                    if (sp_fill_sub == 3'd7) begin
                        g1r_start  <= 1'b1;
                        sp_fill_st <= SPF_GEN;
                    end else begin
                        sp_fill_sub <= sp_fill_sub + 3'd1;
                    end
                end
            end

            SPF_DONE: begin
                // one cycle of slack so the last registered write lands
                sp_fill_done <= 1'b1;
                sp_fill_busy <= 1'b0;
                sp_fill_st   <= SPF_IDLE;
            end

            default: sp_fill_st <= SPF_IDLE;
        endcase
    end
end

// ===========================================================================
// Register interface
// ===========================================================================

// Write
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        seed_reg        <= 512'b0;
        key_len_reg     <= 16'd64;
        start_pulse     <= 1'b0;
        ss_prog_data    <= 64'b0;
        ss_prog_wr_en   <= 1'b0;
        ss_prog_wr_addr <= 12'b0;
        ss_cfg_wr_en    <= 1'b0;
        ss_cfg_sel      <= 3'b0;
        ss_cfg_len      <= 12'b0;
        ss_cfg_addr_reg <= 3'b0;
    end else begin
        start_pulse   <= 1'b0;
        ss_prog_wr_en <= 1'b0;
        ss_cfg_wr_en  <= 1'b0;
        if (reg_wr_en) begin
            casez (reg_wr_addr)
                // Seed: 0x00..0x1C (8 words × 4 bytes)
                8'h00: seed_reg[ 31:  0] <= reg_wr_data;
                8'h04: seed_reg[ 63: 32] <= reg_wr_data;
                8'h08: seed_reg[ 95: 64] <= reg_wr_data;
                8'h0C: seed_reg[127: 96] <= reg_wr_data;
                8'h10: seed_reg[159:128] <= reg_wr_data;
                8'h14: seed_reg[191:160] <= reg_wr_data;
                8'h18: seed_reg[223:192] <= reg_wr_data;
                8'h1C: seed_reg[255:224] <= reg_wr_data;
                8'h20: seed_reg[287:256] <= reg_wr_data;
                8'h24: seed_reg[319:288] <= reg_wr_data;
                8'h28: seed_reg[351:320] <= reg_wr_data;
                8'h2C: seed_reg[383:352] <= reg_wr_data;
                8'h30: seed_reg[415:384] <= reg_wr_data;
                8'h34: seed_reg[447:416] <= reg_wr_data;
                8'h38: seed_reg[479:448] <= reg_wr_data;
                8'h3C: seed_reg[511:480] <= reg_wr_data;
                // Control register: 0x40
                8'h40: start_pulse <= reg_wr_data[0];
                // Argon2 key length in bytes: 0x88 (1..64)
                8'h88: key_len_reg <= (reg_wr_data[15:0] > 16'd64) ? 16'd64
                                                                   : reg_wr_data[15:0];
                // SuperscalarHash program word (low / high half): 0x8C / 0x90
                8'h8C: ss_prog_data[31: 0] <= reg_wr_data;
                8'h90: ss_prog_data[63:32] <= reg_wr_data;
                // Program buffer index: 0x94 — commits the word above
                8'h94: begin
                    ss_prog_wr_addr <= reg_wr_data[11:0];
                    ss_prog_wr_en   <= 1'b1;
                end
                // Per-program configuration: 0x98
                //   [2:0] program index, [15:4] length, [18:16] address register
                8'h98: begin
                    ss_cfg_sel      <= reg_wr_data[2:0];
                    ss_cfg_len      <= reg_wr_data[15:4];
                    ss_cfg_addr_reg <= reg_wr_data[18:16];
                    ss_cfg_wr_en    <= 1'b1;
                end
                default: ; // ignore unknown addresses
            endcase
        end
    end
end

// Read
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        reg_rd_data <= 32'b0;
    end else if (reg_rd_en) begin
        casez (reg_rd_addr)
            // Status: 0x44
            8'h44: reg_rd_data <= {30'b0, hbm_axi_err | cache_axi_err, ~busy};
            // Hash output: 0x48..0x64 (8 × 32-bit)
            8'h48: reg_rd_data <= hash_out[ 31:  0];
            8'h4C: reg_rd_data <= hash_out[ 63: 32];
            8'h50: reg_rd_data <= hash_out[ 95: 64];
            8'h54: reg_rd_data <= hash_out[127: 96];
            8'h58: reg_rd_data <= hash_out[159:128];
            8'h5C: reg_rd_data <= hash_out[191:160];
            8'h60: reg_rd_data <= hash_out[223:192];
            8'h64: reg_rd_data <= hash_out[255:224];
            8'h68: reg_rd_data <= hash_out[287:256];
            8'h6C: reg_rd_data <= hash_out[319:288];
            8'h70: reg_rd_data <= hash_out[351:320];
            8'h74: reg_rd_data <= hash_out[383:352];
            8'h78: reg_rd_data <= hash_out[415:384];
            8'h7C: reg_rd_data <= hash_out[447:416];
            8'h80: reg_rd_data <= hash_out[479:448];
            8'h84: reg_rd_data <= hash_out[511:480];
            default: reg_rd_data <= 32'hDEADBEEF;
        endcase
    end
end

// ===========================================================================
// Top-level FSM
// ===========================================================================
assign all_done = vm_done;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm_state    <= FSM_IDLE;
        busy         <= 1'b0;
        argon2_start <= 1'b0;
        vm_start     <= 1'b0;
        dsg_start    <= 1'b0;
        sp_fill_start<= 1'b0;
        pg_start     <= 1'b0;
        fh_b2b_start <= 1'b0;
        hash_out     <= 512'b0;
    end else begin
        argon2_start  <= 1'b0;
        vm_start      <= 1'b0;
        dsg_start     <= 1'b0;
        sp_fill_start <= 1'b0;
        pg_start      <= 1'b0;
        fh_b2b_start  <= 1'b0;

        case (fsm_state)
            FSM_IDLE: begin
                if (start_pulse) begin
                    busy         <= 1'b1;
                    argon2_start <= 1'b1;
                    fsm_state    <= FSM_CACHE_INIT;
                end
            end

            FSM_CACHE_INIT: begin
                if (argon2_done) begin
                    dsg_start <= 1'b1;
                    fsm_state <= FSM_DS_GEN;
                end
            end

            // Dataset generation: SuperscalarHash over the Argon2d cache,
            // items written to HBM through hbm_dataset_if.
            FSM_DS_GEN: begin
                if (dsg_done) begin
                    sp_fill_start <= 1'b1;
                    fsm_state     <= FSM_SP_FILL;
                end
            end

            // Scratchpad fill (AesGenerator1R)
            FSM_SP_FILL: begin
                if (sp_fill_done) begin
                    pg_start  <= 1'b1;
                    fsm_state <= FSM_PROG_GEN;
                end
            end

            // Program + entropy generation (AesGenerator4R) into the VM
            FSM_PROG_GEN: begin
                if (pg_done) begin
                    vm_start  <= 1'b1;
                    fsm_state <= FSM_VM_RUN;
                end
            end

            FSM_VM_RUN: begin
                if (vm_done) begin
                    fh_b2b_start <= 1'b1;
                    fsm_state    <= FSM_FINAL_HASH;
                end
            end

            // Final hash: Blake2b-256 over the 64-byte AesHash1R result
            // produced by the VM. hash_out[255:0] holds the RandomX result.
            FSM_FINAL_HASH: begin
                if (b2b_done) begin
                    hash_out  <= {256'b0, b2b_h_out[255:0]};
                    fsm_state <= FSM_DONE;
                end
            end

            FSM_DONE: begin
                busy      <= 1'b0;
                fsm_state <= FSM_IDLE;
            end

            default: fsm_state <= FSM_IDLE;
        endcase
    end
end

endmodule
