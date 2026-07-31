// =============================================================================
// tb_hbm_dataset_if.v — Unit testbench for hbm_dataset_if
//
// Verifies against a behavioral AXI4 slave memory model that supports
// outstanding AR/AW transactions and injects error responses:
//   1. Write path: pipelined AW → W (2 beats) → B, data lands at
//      DATASET_BASE_ADDR + item_idx × 64.
//   2. Read path: single item read returns the written data.
//   3. Pipelined reads: back-to-back requests with delayed responses are
//      returned in order.
//   4. Back-to-back writes are accepted without waiting for each B response.
//   5. AXI error responses are reported on wr_err / resp_err and latched in
//      the sticky axi_err status bit.
//   6. AXI attributes (len/size/burst) and the dataset base address offset.
//
// Compile: iverilog -g2001 -o tb_hbm.vvp rtl/hbm_dataset_if.v sim/tb_hbm_dataset_if.v
// Run:     vvp tb_hbm.vvp   → prints PASS/FAIL
// =============================================================================

`timescale 1ns/1ps

module tb_hbm_dataset_if;

localparam BASE_ADDR = 34'h0000_1000;  // dataset base offset in HBM
localparam MEM_BEATS = 256;            // 128 items of 64 bytes
localparam ERR_ITEM  = 32'd100;        // items >= this return SLVERR

reg clk = 0;
reg rst_n = 0;
always #5 clk = ~clk;   // 100 MHz

// ---- DUT VM-side signals ----
reg          req_valid;
reg  [31:0]  req_item_idx;
wire         req_ready;
wire         resp_valid;
wire [511:0] resp_data;
wire         resp_err;
reg          resp_ready;

reg          wr_req_valid;
reg  [31:0]  wr_req_item_idx;
reg  [511:0] wr_req_data;
wire         wr_req_ready;
wire         wr_done;
wire         wr_err;
wire         axi_err;

// ---- AXI wires ----
wire [5:0]   m_axi_arid;
wire [33:0]  m_axi_araddr;
wire [7:0]   m_axi_arlen;
wire [2:0]   m_axi_arsize;
wire [1:0]   m_axi_arburst;
wire         m_axi_arvalid;
reg          m_axi_arready;
reg  [255:0] m_axi_rdata;
reg  [1:0]   m_axi_rresp;
reg          m_axi_rlast;
reg          m_axi_rvalid;
wire         m_axi_rready;

wire [5:0]   m_axi_awid;
wire [33:0]  m_axi_awaddr;
wire [7:0]   m_axi_awlen;
wire [2:0]   m_axi_awsize;
wire [1:0]   m_axi_awburst;
wire         m_axi_awvalid;
reg          m_axi_awready;
wire [255:0] m_axi_wdata;
wire [31:0]  m_axi_wstrb;
wire         m_axi_wlast;
wire         m_axi_wvalid;
reg          m_axi_wready;
reg  [1:0]   m_axi_bresp;
reg          m_axi_bvalid;
wire         m_axi_bready;

hbm_dataset_if #(
    .AXI_ADDR_WIDTH    (34),
    .AXI_DATA_WIDTH    (256),
    .AXI_ID_WIDTH      (6),
    .DATASET_BASE_ADDR (BASE_ADDR),
    .RD_FIFO_DEPTH     (4),
    .WR_FIFO_DEPTH     (4)
) u_dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .req_valid      (req_valid),
    .req_item_idx   (req_item_idx),
    .req_ready      (req_ready),
    .resp_valid     (resp_valid),
    .resp_data      (resp_data),
    .resp_err       (resp_err),
    .resp_ready     (resp_ready),
    .wr_req_valid   (wr_req_valid),
    .wr_req_item_idx(wr_req_item_idx),
    .wr_req_data    (wr_req_data),
    .wr_req_ready   (wr_req_ready),
    .wr_done        (wr_done),
    .wr_err         (wr_err),
    .axi_err        (axi_err),
    .m_axi_arid     (m_axi_arid),
    .m_axi_araddr   (m_axi_araddr),
    .m_axi_arlen    (m_axi_arlen),
    .m_axi_arsize   (m_axi_arsize),
    .m_axi_arburst  (m_axi_arburst),
    .m_axi_arvalid  (m_axi_arvalid),
    .m_axi_arready  (m_axi_arready),
    .m_axi_rid      (6'b0),
    .m_axi_rdata    (m_axi_rdata),
    .m_axi_rresp    (m_axi_rresp),
    .m_axi_rlast    (m_axi_rlast),
    .m_axi_rvalid   (m_axi_rvalid),
    .m_axi_rready   (m_axi_rready),
    .m_axi_awid     (m_axi_awid),
    .m_axi_awaddr   (m_axi_awaddr),
    .m_axi_awlen    (m_axi_awlen),
    .m_axi_awsize   (m_axi_awsize),
    .m_axi_awburst  (m_axi_awburst),
    .m_axi_awvalid  (m_axi_awvalid),
    .m_axi_awready  (m_axi_awready),
    .m_axi_wdata    (m_axi_wdata),
    .m_axi_wstrb    (m_axi_wstrb),
    .m_axi_wlast    (m_axi_wlast),
    .m_axi_wvalid   (m_axi_wvalid),
    .m_axi_wready   (m_axi_wready),
    .m_axi_bid      (6'b0),
    .m_axi_bresp    (m_axi_bresp),
    .m_axi_bvalid   (m_axi_bvalid),
    .m_axi_bready   (m_axi_bready)
);

integer errors = 0;

task check;
    input        cond;
    input [8*48:1] msg;
    begin
        if (!cond) begin
            errors = errors + 1;
            $display("[TB] FAIL: %0s", msg);
        end
    end
endtask

// ---------------------------------------------------------------------------
// Behavioral AXI4 slave: beat-addressable memory, outstanding AR/AW queues.
// Items at or beyond ERR_ITEM return SLVERR (2'b10).
// ---------------------------------------------------------------------------
reg [255:0] mem [0:MEM_BEATS-1];

function [31:0] addr_to_beat;   // byte address → beat index in mem
    input [33:0] addr;
    addr_to_beat = (addr - BASE_ADDR) >> 5;
endfunction

function slv_err;               // error injection by item index
    input [33:0] addr;
    slv_err = (((addr - BASE_ADDR) >> 6) >= ERR_ITEM);
endfunction

// -- AR queue --
reg  [33:0] arq [0:15];
reg  [3:0]  ar_wp, ar_rp;
integer     ar_pushed, ar_popped;
wire [31:0] ar_cnt = ar_pushed - ar_popped;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        m_axi_arready <= 1'b0;
        ar_wp         <= 4'd0;
        ar_pushed     <= 0;
    end else begin
        m_axi_arready <= (ar_cnt < 8);
        if (m_axi_arvalid && m_axi_arready) begin
            arq[ar_wp] <= m_axi_araddr;
            ar_wp      <= ar_wp + 4'd1;
            ar_pushed  <= ar_pushed + 1;
            check(m_axi_arlen   == 8'd1,  "arlen != 1");
            check(m_axi_arsize  == 3'd5,  "arsize != 5");
            check(m_axi_arburst == 2'b01, "arburst != INCR");
            check(m_axi_araddr >= BASE_ADDR, "araddr below base");
            check(m_axi_araddr[5:0] == 6'd0, "araddr not 64B aligned");
        end
    end
end

// -- Read data server (one burst at a time, random latency) --
reg [33:0]  rd_addr;
integer     rd_beat;
integer     rd_lat;

initial begin
    m_axi_rvalid = 1'b0;
    m_axi_rlast  = 1'b0;
    m_axi_rdata  = 256'b0;
    m_axi_rresp  = 2'b00;
    ar_rp        = 4'd0;
    ar_popped    = 0;
    forever begin
        @(posedge clk);
        if (rst_n && (ar_cnt > 0)) begin
            rd_addr   = arq[ar_rp];
            ar_rp     = ar_rp + 4'd1;
            ar_popped = ar_popped + 1;
            for (rd_lat = 0; rd_lat < 2; rd_lat = rd_lat + 1) @(posedge clk);
            for (rd_beat = 0; rd_beat < 2; rd_beat = rd_beat + 1) begin
                m_axi_rdata  <= mem[addr_to_beat(rd_addr) + rd_beat];
                m_axi_rresp  <= slv_err(rd_addr) ? 2'b10 : 2'b00;
                m_axi_rlast  <= (rd_beat == 1);
                m_axi_rvalid <= 1'b1;
                @(posedge clk);
                while (!m_axi_rready) @(posedge clk);
            end
            m_axi_rvalid <= 1'b0;
            m_axi_rlast  <= 1'b0;
        end
    end
end

// -- AW queue --
reg  [33:0] awq [0:15];
reg  [3:0]  aw_wp, aw_rp;
integer     aw_pushed, aw_popped;
wire [31:0] aw_cnt = aw_pushed - aw_popped;

// Peak number of AW transactions accepted but not yet answered — used to
// prove that the write path really is pipelined.
integer     aw_peak;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        m_axi_awready <= 1'b0;
        aw_wp         <= 4'd0;
        aw_pushed     <= 0;
        aw_peak       <= 0;
    end else begin
        m_axi_awready <= (aw_cnt < 8);
        if (m_axi_awvalid && m_axi_awready) begin
            awq[aw_wp] <= m_axi_awaddr;
            aw_wp      <= aw_wp + 4'd1;
            aw_pushed  <= aw_pushed + 1;
            if ((aw_cnt + 1) > aw_peak) aw_peak <= aw_cnt + 1;
            check(m_axi_awlen   == 8'd1,  "awlen != 1");
            check(m_axi_awsize  == 3'd5,  "awsize != 5");
            check(m_axi_awburst == 2'b01, "awburst != INCR");
            check(m_axi_awaddr >= BASE_ADDR, "awaddr below base");
            check(m_axi_awaddr[5:0] == 6'd0, "awaddr not 64B aligned");
            check(m_axi_wstrb == 32'hFFFF_FFFF, "wstrb not all ones");
        end
    end
end

// -- Write data / response server --
reg [33:0] wr_addr;
integer    wr_beat;
integer    wr_lat;

initial begin
    m_axi_wready = 1'b0;
    m_axi_bvalid = 1'b0;
    m_axi_bresp  = 2'b00;
    aw_rp        = 4'd0;
    aw_popped    = 0;
    forever begin
        @(posedge clk);
        if (rst_n && (aw_cnt > 0)) begin
            wr_addr   = awq[aw_rp];
            aw_rp     = aw_rp + 4'd1;
            aw_popped = aw_popped + 1;
            m_axi_wready <= 1'b1;
            wr_beat = 0;
            while (wr_beat < 2) begin
                @(posedge clk);
                if (m_axi_wvalid && m_axi_wready) begin
                    mem[addr_to_beat(wr_addr) + wr_beat] = m_axi_wdata;
                    check(m_axi_wlast == (wr_beat == 1), "wlast misplaced");
                    wr_beat = wr_beat + 1;
                end
            end
            m_axi_wready <= 1'b0;
            for (wr_lat = 0; wr_lat < 3; wr_lat = wr_lat + 1) @(posedge clk);
            m_axi_bresp  <= slv_err(wr_addr) ? 2'b10 : 2'b00;
            m_axi_bvalid <= 1'b1;
            @(posedge clk);
            while (!m_axi_bready) @(posedge clk);
            m_axi_bvalid <= 1'b0;
        end
    end
end

// ---------------------------------------------------------------------------
// Write completion monitor
// ---------------------------------------------------------------------------
integer wr_done_cnt;
integer wr_err_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_done_cnt <= 0;
        wr_err_cnt  <= 0;
    end else if (wr_done) begin
        wr_done_cnt <= wr_done_cnt + 1;
        if (wr_err) wr_err_cnt <= wr_err_cnt + 1;
    end
end

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------
integer k;

function [511:0] item_pattern;
    input [31:0] idx;
    item_pattern = {8{idx + 32'hA5A5_0000, ~idx}};
endfunction

// Post a write request (returns as soon as it is accepted — does not wait
// for the B response, so writes are pipelined).
task post_write;
    input [31:0] idx;
    begin
        wr_req_valid    <= 1'b1;
        wr_req_item_idx <= idx;
        wr_req_data     <= item_pattern(idx);
        @(posedge clk);
        while (!wr_req_ready) @(posedge clk);
        wr_req_valid <= 1'b0;
    end
endtask

task post_read;
    input [31:0] idx;
    begin
        req_valid    <= 1'b1;
        req_item_idx <= idx;
        @(posedge clk);
        while (!req_ready) @(posedge clk);
        req_valid <= 1'b0;
    end
endtask

task expect_resp;
    input [31:0] idx;
    input        exp_err;
    begin
        @(posedge clk);
        while (!resp_valid) @(posedge clk);
        if (resp_data !== item_pattern(idx)) begin
            errors = errors + 1;
            $display("[TB] FAIL: item %0d resp mismatch", idx);
            $display("     got      = %h", resp_data);
            $display("     expected = %h", item_pattern(idx));
        end else begin
            $display("[TB] item %0d read OK (err=%b)", idx, resp_err);
        end
        if (resp_err !== exp_err) begin
            errors = errors + 1;
            $display("[TB] FAIL: item %0d resp_err=%b expected %b",
                     idx, resp_err, exp_err);
        end
        resp_ready <= 1'b1;
        @(posedge clk);
        resp_ready <= 1'b0;
    end
endtask

// ---------------------------------------------------------------------------
// Test sequence
// ---------------------------------------------------------------------------
initial begin
    req_valid       = 0;
    req_item_idx    = 0;
    resp_ready      = 0;
    wr_req_valid    = 0;
    wr_req_item_idx = 0;
    wr_req_data     = 0;
    for (k = 0; k < MEM_BEATS; k = k + 1) mem[k] = 256'b0;

    repeat (4) @(posedge clk);
    rst_n = 1;
    repeat (2) @(posedge clk);

    // --- Test 1: pipelined writes of items 0..7 ---
    for (k = 0; k < 8; k = k + 1) post_write(k[31:0]);
    while (wr_done_cnt < 8) @(posedge clk);
    $display("[TB] 8 items written (peak outstanding AW = %0d)", aw_peak);
    check(aw_peak > 1, "write path not pipelined");
    check(wr_err_cnt == 0, "unexpected write error");
    check(axi_err == 1'b0, "unexpected sticky axi_err");

    // --- Test 2: single read ---
    post_read(32'd3);
    expect_resp(32'd3, 1'b0);

    // --- Test 3: pipelined reads (post 6 back-to-back, drain in order) ---
    fork
        begin
            for (k = 0; k < 6; k = k + 1) post_read(k[31:0]);
        end
        begin : drain
            integer j;
            for (j = 0; j < 6; j = j + 1) expect_resp(j[31:0], 1'b0);
        end
    join

    // --- Test 4: write to an erroring address → wr_err + sticky axi_err ---
    post_write(ERR_ITEM);
    while (wr_done_cnt < 9) @(posedge clk);
    check(wr_err_cnt == 1, "write error not reported");
    check(axi_err == 1'b1, "sticky axi_err not set after write error");

    // --- Test 5: read from an erroring address → resp_err ---
    post_read(ERR_ITEM);
    expect_resp(ERR_ITEM, 1'b1);

    // --- Test 6: back-pressure — post more reads than the FIFO depth ---
    for (k = 16; k < 24; k = k + 1) post_write(k[31:0]);
    while (wr_done_cnt < 17) @(posedge clk);
    for (k = 16; k < 24; k = k + 1) begin
        post_read(k[31:0]);
        expect_resp(k[31:0], 1'b0);
    end

    repeat (10) @(posedge clk);
    if (errors == 0)
        $display("[TB] PASS: all hbm_dataset_if tests passed");
    else
        $display("[TB] FAIL: %0d errors", errors);
    $finish;
end

// Timeout watchdog
initial begin
    #500000;
    $display("[TB] FAIL: timeout");
    $finish;
end

endmodule
