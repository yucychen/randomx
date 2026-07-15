// =============================================================================
// tb_hbm_dataset_if.v — Unit testbench for hbm_dataset_if
//
// Verifies against a behavioral AXI4 slave memory model:
//   1. Write path: AW → W (2 beats) → B, data lands at item_idx × 64.
//   2. Read path: single item read returns the written data.
//   3. Pipelined reads: multiple back-to-back requests with delayed
//      responses return correct data in order.
//
// Compile: iverilog -g2001 -o tb_hbm.vvp rtl/hbm_dataset_if.v sim/tb_hbm_dataset_if.v
// Run:     vvp tb_hbm.vvp   → prints PASS/FAIL
// =============================================================================

`timescale 1ns/1ps

module tb_hbm_dataset_if;

reg clk = 0;
reg rst_n = 0;
always #5 clk = ~clk;   // 100 MHz

// ---- DUT VM-side signals ----
reg          req_valid;
reg  [31:0]  req_item_idx;
wire         req_ready;
wire         resp_valid;
wire [511:0] resp_data;
reg          resp_ready;

reg          wr_req_valid;
reg  [31:0]  wr_req_item_idx;
reg  [511:0] wr_req_data;
wire         wr_req_ready;
wire         wr_done;

// ---- AXI wires ----
wire [5:0]   m_axi_arid;
wire [33:0]  m_axi_araddr;
wire [7:0]   m_axi_arlen;
wire [2:0]   m_axi_arsize;
wire [1:0]   m_axi_arburst;
wire         m_axi_arvalid;
reg          m_axi_arready;
reg  [255:0] m_axi_rdata;
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
reg          m_axi_bvalid;
wire         m_axi_bready;

hbm_dataset_if u_dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .req_valid      (req_valid),
    .req_item_idx   (req_item_idx),
    .req_ready      (req_ready),
    .resp_valid     (resp_valid),
    .resp_data      (resp_data),
    .resp_ready     (resp_ready),
    .wr_req_valid   (wr_req_valid),
    .wr_req_item_idx(wr_req_item_idx),
    .wr_req_data    (wr_req_data),
    .wr_req_ready   (wr_req_ready),
    .wr_done        (wr_done),
    .m_axi_arid     (m_axi_arid),
    .m_axi_araddr   (m_axi_araddr),
    .m_axi_arlen    (m_axi_arlen),
    .m_axi_arsize   (m_axi_arsize),
    .m_axi_arburst  (m_axi_arburst),
    .m_axi_arvalid  (m_axi_arvalid),
    .m_axi_arready  (m_axi_arready),
    .m_axi_rid      (6'b0),
    .m_axi_rdata    (m_axi_rdata),
    .m_axi_rresp    (2'b00),
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
    .m_axi_bresp    (2'b00),
    .m_axi_bvalid   (m_axi_bvalid),
    .m_axi_bready   (m_axi_bready)
);

// ---------------------------------------------------------------------------
// Behavioral AXI4 slave: 16-item (1 KiB) memory as 32 × 256-bit beats.
// Read: 2-cycle AR delay, then 2 R beats. Write: accepts AW/W, delayed B.
// ---------------------------------------------------------------------------
reg [255:0] mem [0:31];

// -- Read side --
reg [33:0] rd_addr_q;
reg [2:0]  rd_state;       // 0=idle, 1..2=delay, 3=beat0, 4=beat1
integer k;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        m_axi_arready <= 1'b0;
        m_axi_rvalid  <= 1'b0;
        m_axi_rlast   <= 1'b0;
        m_axi_rdata   <= 256'b0;
        rd_state      <= 3'd0;
        rd_addr_q     <= 34'b0;
    end else begin
        case (rd_state)
            3'd0: begin
                m_axi_arready <= 1'b1;
                if (m_axi_arvalid && m_axi_arready) begin
                    rd_addr_q     <= m_axi_araddr;
                    m_axi_arready <= 1'b0;
                    rd_state      <= 3'd1;
                end
            end
            3'd1: rd_state <= 3'd2;   // response latency
            3'd2: begin
                m_axi_rvalid <= 1'b1;
                m_axi_rlast  <= 1'b0;
                m_axi_rdata  <= mem[rd_addr_q[9:5]];
                rd_state     <= 3'd3;
            end
            3'd3: begin
                if (m_axi_rready) begin
                    m_axi_rdata <= mem[rd_addr_q[9:5] + 5'd1];
                    m_axi_rlast <= 1'b1;
                    rd_state    <= 3'd4;
                end
            end
            3'd4: begin
                if (m_axi_rready) begin
                    m_axi_rvalid <= 1'b0;
                    m_axi_rlast  <= 1'b0;
                    rd_state     <= 3'd0;
                end
            end
            default: rd_state <= 3'd0;
        endcase
    end
end

// -- Write side --
reg [33:0] wr_addr_q;
reg [1:0]  wr_beat;
reg [1:0]  wr_state;       // 0=await AW, 1=await W beats, 2=B resp

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        m_axi_awready <= 1'b0;
        m_axi_wready  <= 1'b0;
        m_axi_bvalid  <= 1'b0;
        wr_addr_q     <= 34'b0;
        wr_beat       <= 2'd0;
        wr_state      <= 2'd0;
    end else begin
        case (wr_state)
            2'd0: begin
                m_axi_awready <= 1'b1;
                if (m_axi_awvalid && m_axi_awready) begin
                    wr_addr_q     <= m_axi_awaddr;
                    wr_beat       <= 2'd0;
                    m_axi_awready <= 1'b0;
                    m_axi_wready  <= 1'b1;
                    wr_state      <= 2'd1;
                end
            end
            2'd1: begin
                if (m_axi_wvalid && m_axi_wready) begin
                    mem[wr_addr_q[9:5] + {3'b0, wr_beat}] <= m_axi_wdata;
                    wr_beat <= wr_beat + 2'd1;
                    if (m_axi_wlast) begin
                        m_axi_wready <= 1'b0;
                        m_axi_bvalid <= 1'b1;
                        wr_state     <= 2'd2;
                    end
                end
            end
            2'd2: begin
                if (m_axi_bready) begin
                    m_axi_bvalid <= 1'b0;
                    wr_state     <= 2'd0;
                end
            end
            default: wr_state <= 2'd0;
        endcase
    end
end

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------
integer errors = 0;

function [511:0] item_pattern;
    input [31:0] idx;
    item_pattern = {8{idx + 32'hA5A5_0000, ~idx}};
endfunction

task write_item;
    input [31:0] idx;
    begin
        @(posedge clk);
        while (!wr_req_ready) @(posedge clk);
        wr_req_valid    <= 1'b1;
        wr_req_item_idx <= idx;
        wr_req_data     <= item_pattern(idx);
        @(posedge clk);
        wr_req_valid <= 1'b0;
        while (!wr_done) @(posedge clk);
    end
endtask

task post_read;
    input [31:0] idx;
    begin
        @(posedge clk);
        while (!req_ready) @(posedge clk);
        req_valid    <= 1'b1;
        req_item_idx <= idx;
        @(posedge clk);
        req_valid <= 1'b0;
    end
endtask

task expect_resp;
    input [31:0] idx;
    begin
        @(posedge clk);
        while (!resp_valid) @(posedge clk);
        if (resp_data !== item_pattern(idx)) begin
            errors = errors + 1;
            $display("[TB] FAIL: item %0d resp mismatch", idx);
            $display("     got      = %h", resp_data);
            $display("     expected = %h", item_pattern(idx));
        end else begin
            $display("[TB] item %0d read OK", idx);
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
    for (k = 0; k < 32; k = k + 1) mem[k] = 256'b0;

    repeat (4) @(posedge clk);
    rst_n = 1;
    repeat (2) @(posedge clk);

    // --- Test 1: write items 0..7 ---
    for (k = 0; k < 8; k = k + 1) write_item(k[31:0]);
    $display("[TB] 8 items written");

    // --- Test 2: single read ---
    post_read(32'd3);
    expect_resp(32'd3);

    // --- Test 3: pipelined reads (post 6 back-to-back, drain in order) ---
    fork
        begin
            for (k = 0; k < 6; k = k + 1) post_read(k[31:0]);
        end
        begin : drain
            integer j;
            for (j = 0; j < 6; j = j + 1) expect_resp(j[31:0]);
        end
    join

    repeat (10) @(posedge clk);
    if (errors == 0)
        $display("[TB] PASS: all hbm_dataset_if tests passed");
    else
        $display("[TB] FAIL: %0d errors", errors);
    $finish;
end

// Timeout watchdog
initial begin
    #200000;
    $display("[TB] FAIL: timeout");
    $finish;
end

endmodule
