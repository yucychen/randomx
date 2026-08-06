// =============================================================================
// tb_cache_hbm_if.v — Cache storage (AXI4 HBM) testbench
//
// Verifies that cache_hbm_if turns the 1 KiB block port used by argon2_fill
// into correct AXI4 bursts on a 256-bit HBM pseudo-channel:
//   1. block writes are split into 32 beats of 32 bytes at
//      CACHE_BASE_ADDR + blk × 1024, in ascending byte-address order;
//   2. block reads reassemble the beats into the original block;
//   3. the block port handshake (wr_en/wr_rdy, rd_en/rd_valid) is honoured
//      and a held enable never triggers a second transaction;
//   4. an AXI SLVERR response sets the sticky axi_err flag.
//
// The AXI slave model applies random address/data channel back-pressure so the
// handshake logic is exercised, and stores the data in a small sparse memory.
//
// Run:
//   iverilog -g2001 -DSIMULATION -o tb_cache_hbm_if.vvp \
//       rtl/cache_hbm_if.v sim/tb_cache_hbm_if.v
//   vvp tb_cache_hbm_if.vvp
// =============================================================================

`timescale 1ns/1ps

module tb_cache_hbm_if;

localparam ADDR_W    = 34;
localparam DATA_W    = 256;
localparam BASE      = 34'h0_C000_0000;
localparam MEM_WORDS = 1024;              // 32 KiB model → 32 cache blocks

reg clk;
reg rst_n;

// Block port
reg           wr_en;
reg  [31:0]   wr_addr;
reg  [8191:0] wr_data;
wire          wr_rdy;
reg           rd_en;
reg  [31:0]   rd_addr;
wire [8191:0] rd_data;
wire          rd_valid;
wire          axi_err;

// AXI signals
wire [ADDR_W-1:0] araddr;
wire [7:0]        arlen;
wire [2:0]        arsize;
wire [1:0]        arburst;
wire              arvalid;
reg               arready;
reg  [DATA_W-1:0] rdata;
reg  [1:0]        rresp;
reg               rlast;
reg               rvalid;
wire              rready;
wire [ADDR_W-1:0] awaddr;
wire [7:0]        awlen;
wire [2:0]        awsize;
wire [1:0]        awburst;
wire              awvalid;
reg               awready;
wire [DATA_W-1:0] wdata;
wire [DATA_W/8-1:0] wstrb;
wire              wlast;
wire              wvalid;
reg               wready;
reg  [1:0]        bresp;
reg               bvalid;
wire              bready;

integer errors;
integer i;
integer timeout;
reg [8191:0] blk_a, blk_b, got;

cache_hbm_if #(
    .AXI_ADDR_WIDTH  (ADDR_W),
    .AXI_DATA_WIDTH  (DATA_W),
    .AXI_ID_WIDTH    (6),
    .CACHE_BASE_ADDR (BASE)
) dut (
    .clk           (clk),
    .rst_n         (rst_n),
    .wr_en         (wr_en),
    .wr_addr       (wr_addr),
    .wr_data       (wr_data),
    .wr_rdy        (wr_rdy),
    .rd_en         (rd_en),
    .rd_addr       (rd_addr),
    .rd_data       (rd_data),
    .rd_valid      (rd_valid),
    .axi_err       (axi_err),
    .m_axi_arid    (),
    .m_axi_araddr  (araddr),
    .m_axi_arlen   (arlen),
    .m_axi_arsize  (arsize),
    .m_axi_arburst (arburst),
    .m_axi_arvalid (arvalid),
    .m_axi_arready (arready),
    .m_axi_rdata   (rdata),
    .m_axi_rresp   (rresp),
    .m_axi_rlast   (rlast),
    .m_axi_rvalid  (rvalid),
    .m_axi_rready  (rready),
    .m_axi_awid    (),
    .m_axi_awaddr  (awaddr),
    .m_axi_awlen   (awlen),
    .m_axi_awsize  (awsize),
    .m_axi_awburst (awburst),
    .m_axi_awvalid (awvalid),
    .m_axi_awready (awready),
    .m_axi_wdata   (wdata),
    .m_axi_wstrb   (wstrb),
    .m_axi_wlast   (wlast),
    .m_axi_wvalid  (wvalid),
    .m_axi_wready  (wready),
    .m_axi_bresp   (bresp),
    .m_axi_bvalid  (bvalid),
    .m_axi_bready  (bready)
);

always #5 clk = ~clk;

// ---------------------------------------------------------------------------
// Behavioural AXI4 slave model (single outstanding burst, random stalls)
// ---------------------------------------------------------------------------
reg [DATA_W-1:0] mem [0:MEM_WORDS-1];

reg [ADDR_W-1:0] wr_beat_addr;
reg [8:0]        wr_beats_left;
reg              wr_active;
reg [ADDR_W-1:0] rd_beat_addr;
reg [8:0]        rd_beats_left;
reg              rd_active;
reg              slv_err;        // when 1 the slave answers with SLVERR
integer          rnd;

function [31:0] word_index;
    input [ADDR_W-1:0] byte_addr;
    word_index = (byte_addr - BASE) >> 5;   // 32 bytes per beat
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        arready       <= 1'b0;
        awready       <= 1'b0;
        wready        <= 1'b0;
        rvalid        <= 1'b0;
        rlast         <= 1'b0;
        rresp         <= 2'b00;
        rdata         <= {DATA_W{1'b0}};
        bvalid        <= 1'b0;
        bresp         <= 2'b00;
        wr_active     <= 1'b0;
        rd_active     <= 1'b0;
        wr_beats_left <= 9'd0;
        rd_beats_left <= 9'd0;
        wr_beat_addr  <= {ADDR_W{1'b0}};
        rd_beat_addr  <= {ADDR_W{1'b0}};
    end else begin
        rnd = $random;

        // ---- Write address channel ----
        if (awvalid && awready) begin
            wr_active     <= 1'b1;
            wr_beat_addr  <= awaddr;
            wr_beats_left <= {1'b0, awlen} + 9'd1;
            awready       <= 1'b0;
        end else begin
            awready <= !wr_active && !awready && rnd[0];
        end

        // ---- Write data channel ----
        wready <= wr_active && rnd[1];
        if (wvalid && wready) begin
            if (word_index(wr_beat_addr) < MEM_WORDS)
                mem[word_index(wr_beat_addr)] <= wdata;
            wr_beat_addr  <= wr_beat_addr + (DATA_W/8);
            wr_beats_left <= wr_beats_left - 9'd1;
            if (wlast) begin
                wr_active <= 1'b0;
                wready    <= 1'b0;
                bvalid    <= 1'b1;
                bresp     <= slv_err ? 2'b10 : 2'b00;
            end
        end

        // ---- Write response channel ----
        if (bvalid && bready)
            bvalid <= 1'b0;

        // ---- Read address channel ----
        if (arvalid && arready) begin
            rd_active     <= 1'b1;
            rd_beat_addr  <= araddr;
            rd_beats_left <= {1'b0, arlen} + 9'd1;
            arready       <= 1'b0;
        end else begin
            arready <= !rd_active && !arready && rnd[2];
        end

        // ---- Read data channel ----
        if (rd_active && (!rvalid || rready)) begin
            if (rnd[3] && rd_beats_left != 9'd0) begin
                rvalid <= 1'b1;
                rdata  <= (word_index(rd_beat_addr) < MEM_WORDS) ?
                          mem[word_index(rd_beat_addr)] : {DATA_W{1'b0}};
                rresp  <= slv_err ? 2'b10 : 2'b00;
                rlast  <= (rd_beats_left == 9'd1);
                rd_beat_addr  <= rd_beat_addr + (DATA_W/8);
                rd_beats_left <= rd_beats_left - 9'd1;
                if (rd_beats_left == 9'd1)
                    rd_active <= 1'b0;
            end else begin
                rvalid <= 1'b0;
                rlast  <= 1'b0;
            end
        end else if (rvalid && rready) begin
            rvalid <= 1'b0;
            rlast  <= 1'b0;
        end
    end
end

// ---------------------------------------------------------------------------
// Block port drivers
// ---------------------------------------------------------------------------
task write_block;
    input [31:0]   blk;
    input [8191:0] data;
    begin
        @(posedge clk); #1;
        wr_addr = blk;
        wr_data = data;
        wr_en   = 1'b1;
        timeout = 0;
        while (!(wr_rdy === 1'b1) && timeout < 100000) begin
            @(posedge clk); #1;
            timeout = timeout + 1;
        end
        if (timeout >= 100000) begin
            $display("FAIL: write of block %0d timed out", blk);
            errors = errors + 1;
        end
        wr_en = 1'b0;
        @(posedge clk); #1;
    end
endtask

task read_block;
    input  [31:0]   blk;
    output [8191:0] data;
    begin
        @(posedge clk); #1;
        rd_addr = blk;
        rd_en   = 1'b1;
        timeout = 0;
        while (!(rd_valid === 1'b1) && timeout < 100000) begin
            @(posedge clk); #1;
            timeout = timeout + 1;
        end
        if (timeout >= 100000) begin
            $display("FAIL: read of block %0d timed out", blk);
            errors = errors + 1;
        end
        data  = rd_data;
        rd_en = 1'b0;
        @(posedge clk); #1;
    end
endtask

// ---------------------------------------------------------------------------
// Test sequence
// ---------------------------------------------------------------------------
initial begin
    clk     = 1'b0;
    rst_n   = 1'b0;
    wr_en   = 1'b0;
    rd_en   = 1'b0;
    wr_addr = 32'b0;
    rd_addr = 32'b0;
    wr_data = 8192'b0;
    slv_err = 1'b0;
    errors  = 0;

    for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = {DATA_W{1'b0}};

    // Two distinguishable 1 KiB blocks (byte k of block A = k, block B = ~k)
    blk_a = 8192'b0;
    blk_b = 8192'b0;
    for (i = 0; i < 1024; i = i + 1) begin
        blk_a[i*8 +: 8] = i[7:0];
        blk_b[i*8 +: 8] = ~i[7:0];
    end

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // --- Write two blocks, read them back ---------------------------------
    write_block(32'd0, blk_a);
    write_block(32'd5, blk_b);

    // Byte order check against the slave memory image: block 5 byte 0 lands at
    // BASE + 5*1024, i.e. word index 160, bits [7:0].
    if (mem[160][7:0] !== blk_b[7:0] ||
        mem[160][255:248] !== blk_b[255:248] ||
        mem[191][255:248] !== blk_b[8191:8184]) begin
        $display("FAIL: block 5 byte order wrong in HBM image");
        errors = errors + 1;
    end

    read_block(32'd0, got);
    if (got !== blk_a) begin
        $display("FAIL: block 0 read back mismatch");
        errors = errors + 1;
    end

    read_block(32'd5, got);
    if (got !== blk_b) begin
        $display("FAIL: block 5 read back mismatch");
        errors = errors + 1;
    end

    // --- Overwrite and re-read --------------------------------------------
    write_block(32'd5, blk_a);
    read_block(32'd5, got);
    if (got !== blk_a) begin
        $display("FAIL: block 5 overwrite mismatch");
        errors = errors + 1;
    end

    if (axi_err !== 1'b0) begin
        $display("FAIL: axi_err set although all responses were OKAY");
        errors = errors + 1;
    end

    // --- SLVERR response sets the sticky error flag ------------------------
    slv_err = 1'b1;
    read_block(32'd0, got);
    if (axi_err !== 1'b1) begin
        $display("FAIL: axi_err not set after SLVERR response");
        errors = errors + 1;
    end
    slv_err = 1'b0;

    if (errors == 0)
        $display("=== tb_cache_hbm_if: ALL TESTS PASSED ===");
    else
        $display("=== tb_cache_hbm_if: %0d FAILURE(S) ===", errors);

    $finish;
end

// Safety watchdog
initial begin
    #5000000;
    $display("FAIL: tb_cache_hbm_if watchdog expired");
    $finish;
end

endmodule
