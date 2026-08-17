// =============================================================================
// tb_randomx_hbm_top.v — Testbench for the board-level HBM wrapper
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Covers the three things the wrapper is responsible for:
//   1. Reset gating — while `hbm_init_done` is low the core must be held in
//      reset and must not drive a single AXI address valid.
//   2. Address adaptation — the 34-bit core address is truncated to
//      AXI_ADDR_WIDTH (33) bits; every address the design actually produces
//      must survive that unchanged, so `hbm_addr_err` must stay low and the
//      Argon2d cache blocks must land in the cache window.
//   3. End-to-end pass-through — with a behavioural HBM slave attached the
//      wrapped core still completes a hash and reports done.
//
// Build (see Makefile target tb_randomx_hbm_top):
//   iverilog -g2001 -Wall -DSIMULATION -o sim/build/tb_randomx_hbm_top.vvp \
//       <all rtl> rtl/randomx_hbm_top.v sim/tb_randomx_hbm_top.v
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module tb_randomx_hbm_top;

localparam AXI_ADDR_WIDTH = 33;

// ---------------------------------------------------------------------------
// Clock and reset
// ---------------------------------------------------------------------------
reg sys_clk;
reg sys_rst_n;
reg hbm_init_done;

initial sys_clk = 1'b0;
always #1.667 sys_clk = ~sys_clk;   // 300 MHz

// ---------------------------------------------------------------------------
// DUT signals
// ---------------------------------------------------------------------------
reg         reg_wr_en;
reg  [7:0]  reg_wr_addr;
reg  [31:0] reg_wr_data;
reg         reg_rd_en;
reg  [7:0]  reg_rd_addr;
wire [31:0] reg_rd_data;
wire        hbm_addr_err;

wire [AXI_ADDR_WIDTH-1:0] axi_araddr;
wire [7:0]   axi_arlen;
wire [2:0]   axi_arsize;
wire [1:0]   axi_arburst;
wire         axi_arvalid;
wire         axi_rready;
wire [AXI_ADDR_WIDTH-1:0] axi_awaddr;
wire [7:0]   axi_awlen;
wire [2:0]   axi_awsize;
wire [1:0]   axi_awburst;
wire         axi_awvalid;
wire [255:0] axi_wdata;
wire [31:0]  axi_wstrb;
wire         axi_wlast;
wire         axi_wvalid;
wire         axi_bready;

reg          axi_arready;
reg  [255:0] axi_rdata;
reg  [1:0]   axi_rresp;
reg          axi_rlast;
reg          axi_rvalid;
reg          axi_awready;
reg          axi_wready;
reg  [1:0]   axi_bresp;
reg          axi_bvalid;

randomx_hbm_top #(
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH)
) u_dut (
    .sys_clk         (sys_clk),
    .sys_rst_n       (sys_rst_n),
    .reg_wr_en       (reg_wr_en),
    .reg_wr_addr     (reg_wr_addr),
    .reg_wr_data     (reg_wr_data),
    .reg_rd_en       (reg_rd_en),
    .reg_rd_addr     (reg_rd_addr),
    .reg_rd_data     (reg_rd_data),
    .hbm_addr_err    (hbm_addr_err),
    .hbm_init_done   (hbm_init_done),
    .hbm_axi_araddr  (axi_araddr),
    .hbm_axi_arlen   (axi_arlen),
    .hbm_axi_arsize  (axi_arsize),
    .hbm_axi_arburst (axi_arburst),
    .hbm_axi_arvalid (axi_arvalid),
    .hbm_axi_arready (axi_arready),
    .hbm_axi_rdata   (axi_rdata),
    .hbm_axi_rresp   (axi_rresp),
    .hbm_axi_rlast   (axi_rlast),
    .hbm_axi_rvalid  (axi_rvalid),
    .hbm_axi_rready  (axi_rready),
    .hbm_axi_awaddr  (axi_awaddr),
    .hbm_axi_awlen   (axi_awlen),
    .hbm_axi_awsize  (axi_awsize),
    .hbm_axi_awburst (axi_awburst),
    .hbm_axi_awvalid (axi_awvalid),
    .hbm_axi_awready (axi_awready),
    .hbm_axi_wdata   (axi_wdata),
    .hbm_axi_wstrb   (axi_wstrb),
    .hbm_axi_wlast   (axi_wlast),
    .hbm_axi_wvalid  (axi_wvalid),
    .hbm_axi_wready  (axi_wready),
    .hbm_axi_bresp   (axi_bresp),
    .hbm_axi_bvalid  (axi_bvalid),
    .hbm_axi_bready  (axi_bready)
);

// ---------------------------------------------------------------------------
// Behavioural HBM AXI4 slave model (same structure as tb_randomx_top, but on
// the 33-bit adapted address bus)
// ---------------------------------------------------------------------------
localparam [AXI_ADDR_WIDTH-1:0] CACHE_BASE  = 33'h0_C000_0000;
localparam                      CACHE_WORDS = 256;    // 256 × 32 B = 8 KiB

reg [255:0] hbm_mem [0:CACHE_WORDS-1];

reg [AXI_ADDR_WIDTH-1:0] slv_wr_addr;
reg [8:0]                slv_wr_left;
reg                      slv_wr_active;
reg [AXI_ADDR_WIDTH-1:0] slv_rd_addr;
reg [8:0]                slv_rd_left;
reg                      slv_rd_active;

integer cache_writes;

function integer cache_word;
    input [AXI_ADDR_WIDTH-1:0] byte_addr;
    cache_word = (byte_addr - CACHE_BASE) >> 5;
endfunction

function in_cache;
    input [AXI_ADDR_WIDTH-1:0] byte_addr;
    in_cache = (byte_addr >= CACHE_BASE) &&
               ((byte_addr - CACHE_BASE) < (CACHE_WORDS * 32));
endfunction

integer w;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        axi_arready   <= 1'b0;
        axi_awready   <= 1'b0;
        axi_wready    <= 1'b0;
        axi_rvalid    <= 1'b0;
        axi_rlast     <= 1'b0;
        axi_rresp     <= 2'b00;
        axi_rdata     <= 256'b0;
        axi_bvalid    <= 1'b0;
        axi_bresp     <= 2'b00;
        slv_wr_active <= 1'b0;
        slv_rd_active <= 1'b0;
        slv_wr_left   <= 9'd0;
        slv_rd_left   <= 9'd0;
        slv_wr_addr   <= {AXI_ADDR_WIDTH{1'b0}};
        slv_rd_addr   <= {AXI_ADDR_WIDTH{1'b0}};
        cache_writes  <= 0;
    end else begin
        // ---- Write address ----
        if (axi_awvalid && axi_awready) begin
            slv_wr_active <= 1'b1;
            slv_wr_addr   <= axi_awaddr;
            slv_wr_left   <= {1'b0, axi_awlen} + 9'd1;
            axi_awready   <= 1'b0;
        end else begin
            axi_awready <= !slv_wr_active && !axi_awready;
        end

        // ---- Write data ----
        axi_wready <= slv_wr_active;
        if (axi_wvalid && axi_wready) begin
            if (in_cache(slv_wr_addr)) begin
                hbm_mem[cache_word(slv_wr_addr)] <= axi_wdata;
                cache_writes <= cache_writes + 1;
            end
            slv_wr_addr <= slv_wr_addr + 32;
            slv_wr_left <= slv_wr_left - 9'd1;
            if (axi_wlast) begin
                slv_wr_active <= 1'b0;
                axi_wready    <= 1'b0;
                axi_bvalid    <= 1'b1;
                axi_bresp     <= 2'b00;
            end
        end

        if (axi_bvalid && axi_bready)
            axi_bvalid <= 1'b0;

        // ---- Read address ----
        if (axi_arvalid && axi_arready) begin
            slv_rd_active <= 1'b1;
            slv_rd_addr   <= axi_araddr;
            slv_rd_left   <= {1'b0, axi_arlen} + 9'd1;
            axi_arready   <= 1'b0;
        end else begin
            axi_arready <= !slv_rd_active && !axi_arready;
        end

        // ---- Read data ----
        if (slv_rd_active && (!axi_rvalid || axi_rready)) begin
            axi_rvalid  <= 1'b1;
            axi_rdata   <= in_cache(slv_rd_addr) ?
                           hbm_mem[cache_word(slv_rd_addr)] : 256'b0;
            axi_rresp   <= 2'b00;
            axi_rlast   <= (slv_rd_left == 9'd1);
            slv_rd_addr <= slv_rd_addr + 32;
            slv_rd_left <= slv_rd_left - 9'd1;
            if (slv_rd_left == 9'd1)
                slv_rd_active <= 1'b0;
        end else if (axi_rvalid && axi_rready) begin
            axi_rvalid <= 1'b0;
            axi_rlast  <= 1'b0;
        end
    end
end

initial begin
    for (w = 0; w < CACHE_WORDS; w = w + 1) hbm_mem[w] = 256'b0;
end

// ---------------------------------------------------------------------------
// Continuous checks
// ---------------------------------------------------------------------------
integer errors;
integer gated_violations;

initial begin
    errors           = 0;
    gated_violations = 0;
end

// While HBM initialisation has not completed, no AXI address may be issued.
always @(posedge sys_clk) begin
    if (!hbm_init_done && (axi_arvalid || axi_awvalid))
        gated_violations = gated_violations + 1;
end

// The 34→33 bit truncation must never lose information.
always @(posedge sys_clk) begin
    if (hbm_addr_err) begin
        $display("FAIL: hbm_addr_err asserted — an AXI address did not fit in %0d bits",
                 AXI_ADDR_WIDTH);
        errors = errors + 1;
    end
end

// ---------------------------------------------------------------------------
// Helper tasks
// ---------------------------------------------------------------------------
task write_reg;
    input [7:0]  addr;
    input [31:0] data;
    begin
        @(posedge sys_clk);
        #0.1;
        reg_wr_en   = 1'b1;
        reg_wr_addr = addr;
        reg_wr_data = data;
        @(posedge sys_clk);
        #0.1;
        reg_wr_en   = 1'b0;
    end
endtask

task read_reg;
    input  [7:0]  addr;
    output [31:0] data;
    begin
        @(posedge sys_clk);
        #0.1;
        reg_rd_en   = 1'b1;
        reg_rd_addr = addr;
        @(posedge sys_clk);
        #0.1;
        data      = reg_rd_data;
        reg_rd_en = 1'b0;
    end
endtask

// ---------------------------------------------------------------------------
// Test sequence
// ---------------------------------------------------------------------------
integer i;
integer timeout;
reg [31:0] status;

initial begin
    sys_rst_n     = 1'b0;
    hbm_init_done = 1'b0;
    reg_wr_en     = 1'b0;
    reg_wr_addr   = 8'b0;
    reg_wr_data   = 32'b0;
    reg_rd_en     = 1'b0;
    reg_rd_addr   = 8'b0;

    repeat (10) @(posedge sys_clk);
    sys_rst_n = 1'b1;
    repeat (5) @(posedge sys_clk);

    // -----------------------------------------------------------------------
    // Test 1 — reset gating: start the core while HBM is still initialising
    // -----------------------------------------------------------------------
    for (i = 0; i < 16; i = i + 1)
        write_reg(i * 4, 32'hA5A50000 + i);
    write_reg(8'h40, 32'h1);              // start

    repeat (200) @(posedge sys_clk);

    if (gated_violations != 0) begin
        $display("FAIL: %0d AXI address handshakes issued while hbm_init_done was low",
                 gated_violations);
        errors = errors + 1;
    end else begin
        $display("PASS: core held in reset while hbm_init_done = 0 (no AXI activity)");
    end

    // -----------------------------------------------------------------------
    // Test 2 — release HBM, re-issue the request, expect completion
    // -----------------------------------------------------------------------
    hbm_init_done = 1'b1;
    repeat (20) @(posedge sys_clk);

    for (i = 0; i < 16; i = i + 1)
        write_reg(i * 4, 32'hA5A50000 + i);
    write_reg(8'h88, 32'd64);             // Argon2 key length
    write_reg(8'h40, 32'h1);              // start

    timeout = 0;
    status  = 32'b0;
    while ((status[0] !== 1'b1) && (timeout < 200000)) begin
        read_reg(8'h44, status);
        timeout = timeout + 1;
    end

    if (status[0] !== 1'b1) begin
        $display("FAIL: core did not report done within %0d register polls", timeout);
        errors = errors + 1;
    end else begin
        $display("PASS: core completed after hbm_init_done released (%0d polls)", timeout);
    end

    if (status[1] === 1'b1) begin
        $display("FAIL: status bit1 (HBM AXI error) is set");
        errors = errors + 1;
    end else begin
        $display("PASS: no AXI error reported by the core");
    end

    // -----------------------------------------------------------------------
    // Test 3 — Argon2d cache blocks reached the cache window through the
    //          adapted 33-bit address bus
    // -----------------------------------------------------------------------
    if (cache_writes == 0) begin
        $display("FAIL: no cache-window writes observed on the adapted AXI bus");
        errors = errors + 1;
    end else begin
        $display("PASS: %0d cache-window beats written through the 33-bit address bus",
                 cache_writes);
    end

    if (hbm_addr_err !== 1'b0) begin
        $display("FAIL: hbm_addr_err set at end of test");
        errors = errors + 1;
    end else begin
        $display("PASS: hbm_addr_err clear — all addresses fit in %0d bits",
                 AXI_ADDR_WIDTH);
    end

    if (errors == 0)
        $display("=== tb_randomx_hbm_top: ALL TESTS PASSED ===");
    else
        $display("=== tb_randomx_hbm_top: %0d FAILURES ===", errors);

    $finish;
end

endmodule
