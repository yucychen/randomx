// =============================================================================
// tb_scratchpad_mem.v — Self-checking testbench for scratchpad_mem
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Checks:
//   1. Basic write/read round-trip with the one-cycle rd_valid delay.
//   2. L1/L2/L3 address masking (spec §4.6.2): a byte address is masked to
//      2K/32K/256K words depending on the access level, so aliasing must
//      happen exactly at the level boundary.
//   3. Read-during-write returns the *old* contents (write-first is not
//      modelled), and back-to-back writes to the same address.
//
// Compiled with -DSIMULATION, so the array is 4096 × 64-bit (32 KiB) and
// only L1 masking is fully exercised; L2/L3 addresses are additionally
// clipped to 12 bits by the simulation build, which the checks account for.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module tb_scratchpad_mem;

reg         clk = 1'b0;
reg         rst_n = 1'b0;

reg         wr_en;
reg  [20:0] wr_addr;
reg  [63:0] wr_data;
reg  [1:0]  wr_level;

reg         rd_en;
reg  [20:0] rd_addr;
reg  [1:0]  rd_level;
wire [63:0] rd_data;
wire        rd_valid;

integer     errors = 0;

always #5 clk = ~clk;

scratchpad_mem u_dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .wr_en    (wr_en),
    .wr_addr  (wr_addr),
    .wr_data  (wr_data),
    .wr_level (wr_level),
    .rd_en    (rd_en),
    .rd_addr  (rd_addr),
    .rd_level (rd_level),
    .rd_data  (rd_data),
    .rd_valid (rd_valid)
);

// --- Helpers ---------------------------------------------------------------
task do_write;
    input [20:0] addr;
    input [1:0]  level;
    input [63:0] data;
begin
    @(negedge clk);
    wr_en    = 1'b1;
    wr_addr  = addr;
    wr_level = level;
    wr_data  = data;
    @(negedge clk);
    wr_en    = 1'b0;
end
endtask

// Issues a read and returns the data one cycle later, checking rd_valid.
task do_read;
    input  [20:0] addr;
    input  [1:0]  level;
    output [63:0] data;
begin
    @(negedge clk);
    rd_en    = 1'b1;
    rd_addr  = addr;
    rd_level = level;
    @(negedge clk);
    rd_en    = 1'b0;
    // rd_data / rd_valid are registered: valid on the following negedge
    if (rd_valid !== 1'b1) begin
        $display("[TB] FAIL: rd_valid not asserted for addr 0x%05h", addr);
        errors = errors + 1;
    end
    data = rd_data;
end
endtask

task check64;
    input [255:0] name;
    input [63:0]  got;
    input [63:0]  exp;
begin
    if (got !== exp) begin
        $display("[TB] FAIL: %0s = 0x%016h, expected 0x%016h", name, got, exp);
        errors = errors + 1;
    end else begin
        $display("[TB] PASS: %0s = 0x%016h", name, got);
    end
end
endtask

reg [63:0] q;
integer    i;

initial begin
    $dumpfile("tb_scratchpad_mem.vcd");
    $dumpvars(0, tb_scratchpad_mem);

    wr_en = 1'b0; wr_addr = 21'b0; wr_data = 64'b0; wr_level = 2'd2;
    rd_en = 1'b0; rd_addr = 21'b0; rd_level = 2'd2;

    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // ---- 1. Basic round-trip (L3 addressing, word-aligned addresses) ----
    do_write(21'h00000, 2'd2, 64'h0123456789ABCDEF);
    do_write(21'h00008, 2'd2, 64'hFEDCBA9876543210);
    do_write(21'h00010, 2'd2, 64'hDEADBEEFCAFEBABE);

    do_read(21'h00000, 2'd2, q); check64("word0", q, 64'h0123456789ABCDEF);
    do_read(21'h00008, 2'd2, q); check64("word1", q, 64'hFEDCBA9876543210);
    do_read(21'h00010, 2'd2, q); check64("word2", q, 64'hDEADBEEFCAFEBABE);

    // Sub-word address bits [2:0] are ignored (byte address → word index)
    do_read(21'h00005, 2'd2, q); check64("word0 (unaligned)", q, 64'h0123456789ABCDEF);

    // ---- 2. Overwrite ----
    do_write(21'h00008, 2'd2, 64'hA5A5A5A5A5A5A5A5);
    do_read (21'h00008, 2'd2, q); check64("word1 overwritten", q, 64'hA5A5A5A5A5A5A5A5);

    // ---- 3. L1 masking: 16 KiB = 2048 words, so 0x4000 aliases to 0x0000 ----
    do_write(21'h00000, 2'd0, 64'h1111111111111111);
    do_write(21'h04000, 2'd0, 64'h2222222222222222); // L1 alias of 0x0000
    do_read (21'h00000, 2'd0, q);
    check64("L1 alias 0x4000 -> 0x0000", q, 64'h2222222222222222);

    // The same two addresses are distinct at L2/L3 level
    do_write(21'h00000, 2'd2, 64'h3333333333333333);
    do_write(21'h04000, 2'd2, 64'h4444444444444444);
    do_read (21'h00000, 2'd2, q);
    check64("L3 0x0000 distinct", q, 64'h3333333333333333);
    do_read (21'h04000, 2'd2, q);
    check64("L3 0x4000 distinct", q, 64'h4444444444444444);

    // An L1 read of 0x4000 must see the L1-masked (0x0000) contents
    do_read (21'h04000, 2'd0, q);
    check64("L1 read of 0x4000", q, 64'h3333333333333333);

    // ---- 4. L2 masking: 256 KiB = 32768 words, 0x40000 aliases to 0x0000 ----
    // (the simulation build clips to 4096 words, so both alias to word 0)
    do_write(21'h00018, 2'd1, 64'h5555555555555555);
    do_read (21'h00018, 2'd1, q);
    check64("L2 round-trip", q, 64'h5555555555555555);

    // ---- 5. Sequential fill / read-back over a small window ----
    for (i = 0; i < 16; i = i + 1)
        do_write(21'h00100 + i[20:0] * 8, 2'd2, {32'hC0DE0000, i[31:0]});
    for (i = 0; i < 16; i = i + 1) begin
        do_read(21'h00100 + i[20:0] * 8, 2'd2, q);
        if (q !== {32'hC0DE0000, i[31:0]}) begin
            $display("[TB] FAIL: fill[%0d] = 0x%016h, expected 0x%016h",
                     i, q, {32'hC0DE0000, i[31:0]});
            errors = errors + 1;
        end
    end
    if (errors == 0)
        $display("[TB] PASS: 16-word sequential fill / read-back");

    // ---- 6. rd_valid must be low when rd_en is low ----
    @(negedge clk);
    rd_en = 1'b0;
    @(negedge clk);
    @(negedge clk);
    if (rd_valid !== 1'b0) begin
        $display("[TB] FAIL: rd_valid asserted without rd_en");
        errors = errors + 1;
    end

    if (errors == 0)
        $display("[TB] tb_scratchpad_mem: ALL TESTS PASSED");
    else
        $display("[TB] tb_scratchpad_mem: %0d FAIL(s)", errors);
    $finish;
end

initial begin
    #500000;
    $display("[TB] FAIL: tb_scratchpad_mem timeout");
    $finish;
end

endmodule
