// =============================================================================
// tb_randomx_top.v — Basic Testbench for RandomX Top Module
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Simulation steps:
//   1. Assert reset for 10 cycles
//   2. Write a 64-byte seed via the register interface (16 × 32-bit writes)
//   3. Write control register to assert start
//   4. Wait for status.done = 1
//   5. Read back hash output registers
//   6. Dump waveform to tb_randomx_top.vcd
//
// Run with:
//   iverilog -g2001 -o sim/tb_randomx_top.vvp \
//       rtl/aes_round.v rtl/aes_gen1r.v rtl/aes_gen4r.v rtl/aes_hash1r.v \
//       rtl/blake2b_core.v rtl/scratchpad_mem.v rtl/hbm_dataset_if.v \
//       rtl/alu_int.v rtl/fpu_double.v rtl/superscalar_hash.v \
//       rtl/argon2_fill.v rtl/randomx_vm.v rtl/randomx_top.v \
//       sim/tb_randomx_top.v
//   vvp sim/tb_randomx_top.vvp
//   gtkwave tb_randomx_top.vcd
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module tb_randomx_top;

// ---------------------------------------------------------------------------
// Clock and reset
// ---------------------------------------------------------------------------
reg clk;
reg rst_n;

// 300 MHz clock → period 3.333 ns
initial clk = 1'b0;
always #1.667 clk = ~clk;

// ---------------------------------------------------------------------------
// DUT signals
// ---------------------------------------------------------------------------
reg         reg_wr_en;
reg  [7:0]  reg_wr_addr;
reg  [31:0] reg_wr_data;

reg         reg_rd_en;
reg  [7:0]  reg_rd_addr;
wire [31:0] reg_rd_data;

// AXI HBM stubs (tie off — dataset not populated in simulation)
wire [33:0] m_axi_araddr;
wire [7:0]  m_axi_arlen;
wire [2:0]  m_axi_arsize;
wire [1:0]  m_axi_arburst;
wire        m_axi_arvalid;
wire        m_axi_rready;
wire [33:0] m_axi_awaddr;
wire [7:0]  m_axi_awlen;
wire [2:0]  m_axi_awsize;
wire [1:0]  m_axi_awburst;
wire        m_axi_awvalid;
wire [255:0] m_axi_wdata;
wire [31:0] m_axi_wstrb;
wire        m_axi_wlast;
wire        m_axi_wvalid;
wire        m_axi_bready;

// ---------------------------------------------------------------------------
// DUT instantiation
// ---------------------------------------------------------------------------
randomx_top u_dut (
    .clk           (clk),
    .rst_n         (rst_n),
    .reg_wr_en     (reg_wr_en),
    .reg_wr_addr   (reg_wr_addr),
    .reg_wr_data   (reg_wr_data),
    .reg_rd_en     (reg_rd_en),
    .reg_rd_addr   (reg_rd_addr),
    .reg_rd_data   (reg_rd_data),
    // HBM — stub (arready=0, rvalid=0: DUT will stall on dataset accesses)
    .m_axi_araddr  (m_axi_araddr),
    .m_axi_arlen   (m_axi_arlen),
    .m_axi_arsize  (m_axi_arsize),
    .m_axi_arburst (m_axi_arburst),
    .m_axi_arvalid (m_axi_arvalid),
    .m_axi_arready (1'b0),
    .m_axi_rdata   (256'b0),
    .m_axi_rresp   (2'b0),
    .m_axi_rlast   (1'b0),
    .m_axi_rvalid  (1'b0),
    .m_axi_rready  (m_axi_rready),
    .m_axi_awaddr  (m_axi_awaddr),
    .m_axi_awlen   (m_axi_awlen),
    .m_axi_awsize  (m_axi_awsize),
    .m_axi_awburst (m_axi_awburst),
    .m_axi_awvalid (m_axi_awvalid),
    .m_axi_awready (1'b0),
    .m_axi_wdata   (m_axi_wdata),
    .m_axi_wstrb   (m_axi_wstrb),
    .m_axi_wlast   (m_axi_wlast),
    .m_axi_wvalid  (m_axi_wvalid),
    .m_axi_wready  (1'b0),
    .m_axi_bresp   (2'b0),
    .m_axi_bvalid  (1'b0),
    .m_axi_bready  (m_axi_bready)
);

// ---------------------------------------------------------------------------
// Waveform dump
// ---------------------------------------------------------------------------
initial begin
    $dumpfile("tb_randomx_top.vcd");
    $dumpvars(0, tb_randomx_top);
end

// ---------------------------------------------------------------------------
// Helper tasks
// ---------------------------------------------------------------------------
task write_reg;
    input [7:0] addr;
    input [31:0] data;
    begin
        @(posedge clk);
        #0.1;
        reg_wr_en   = 1'b1;
        reg_wr_addr = addr;
        reg_wr_data = data;
        @(posedge clk);
        #0.1;
        reg_wr_en   = 1'b0;
    end
endtask

task read_reg;
    input  [7:0]  addr;
    output [31:0] data;
    begin
        @(posedge clk);
        #0.1;
        reg_rd_en   = 1'b1;
        reg_rd_addr = addr;
        @(posedge clk);
        #0.1;
        data      = reg_rd_data;
        reg_rd_en = 1'b0;
    end
endtask

// ---------------------------------------------------------------------------
// Test sequence
// ---------------------------------------------------------------------------
integer timeout;
reg [31:0] status;
reg [31:0] hash_word;

initial begin
    // Initialize signals
    rst_n       = 1'b0;
    reg_wr_en   = 1'b0;
    reg_wr_addr = 8'b0;
    reg_wr_data = 32'b0;
    reg_rd_en   = 1'b0;
    reg_rd_addr = 8'b0;
    status      = 32'b0;
    hash_word   = 32'b0;

    // --- Reset phase (10 cycles) ---
    repeat(10) @(posedge clk);
    #0.1;
    rst_n = 1'b1;
    repeat(5) @(posedge clk);

    $display("[TB] Reset released at time %0t ns", $time);

    // --- Write seed (64-byte test vector: bytes 0x00..0x3F) ---
    $display("[TB] Writing seed...");
    write_reg(8'h00, 32'h03020100);
    write_reg(8'h04, 32'h07060504);
    write_reg(8'h08, 32'h0b0a0908);
    write_reg(8'h0C, 32'h0f0e0d0c);
    write_reg(8'h10, 32'h13121110);
    write_reg(8'h14, 32'h17161514);
    write_reg(8'h18, 32'h1b1a1918);
    write_reg(8'h1C, 32'h1f1e1d1c);
    write_reg(8'h20, 32'h23222120);
    write_reg(8'h24, 32'h27262524);
    write_reg(8'h28, 32'h2b2a2928);
    write_reg(8'h2C, 32'h2f2e2d2c);
    write_reg(8'h30, 32'h33323130);
    write_reg(8'h34, 32'h37363534);
    write_reg(8'h38, 32'h3b3a3938);
    write_reg(8'h3C, 32'h3f3e3d3c);

    // --- Assert start ---
    $display("[TB] Asserting start...");
    write_reg(8'h40, 32'h00000001);

    // --- Wait for done (poll status, timeout after 50000 cycles) ---
    $display("[TB] Waiting for done...");
    timeout = 0;
    status  = 32'b0;
    while (status[0] == 1'b0 && timeout < 50000) begin
        read_reg(8'h44, status);
        timeout = timeout + 1;
    end

    if (timeout >= 50000) begin
        $display("[TB] TIMEOUT waiting for done after %0d cycles", timeout);
    end else begin
        $display("[TB] Done asserted after ~%0d poll cycles at time %0t ns",
                 timeout, $time);
    end

    // --- Read back hash output ---
    $display("[TB] Hash output:");
    read_reg(8'h48, hash_word); $display("  hash[31:0]   = 0x%08h", hash_word);
    read_reg(8'h4C, hash_word); $display("  hash[63:32]  = 0x%08h", hash_word);
    read_reg(8'h50, hash_word); $display("  hash[95:64]  = 0x%08h", hash_word);
    read_reg(8'h54, hash_word); $display("  hash[127:96] = 0x%08h", hash_word);
    read_reg(8'h58, hash_word); $display("  hash[159:128]= 0x%08h", hash_word);
    read_reg(8'h5C, hash_word); $display("  hash[191:160]= 0x%08h", hash_word);
    read_reg(8'h60, hash_word); $display("  hash[223:192]= 0x%08h", hash_word);
    read_reg(8'h64, hash_word); $display("  hash[255:224]= 0x%08h", hash_word);
    read_reg(8'h68, hash_word); $display("  hash[287:256]= 0x%08h", hash_word);
    read_reg(8'h6C, hash_word); $display("  hash[319:288]= 0x%08h", hash_word);
    read_reg(8'h70, hash_word); $display("  hash[351:320]= 0x%08h", hash_word);
    read_reg(8'h74, hash_word); $display("  hash[383:352]= 0x%08h", hash_word);
    read_reg(8'h78, hash_word); $display("  hash[415:384]= 0x%08h", hash_word);
    read_reg(8'h7C, hash_word); $display("  hash[447:416]= 0x%08h", hash_word);
    read_reg(8'h80, hash_word); $display("  hash[479:448]= 0x%08h", hash_word);
    read_reg(8'h84, hash_word); $display("  hash[511:480]= 0x%08h", hash_word);

    $display("[TB] Simulation complete.");
    #100;
    $finish;
end

// Safety timeout watchdog
initial begin
    #2000000; // 2 ms simulation limit
    $display("[TB] WATCHDOG: simulation exceeded 2ms, aborting.");
    $finish;
end

endmodule
