// =============================================================================
// tb_alu_int.v — self-checking testbench for the integer execution unit
//
// Covers every opcode of alu_int, including the multi-cycle IMUL_RCP path
// (rtl/recip.v). Expected values follow the RandomX spec §5.5 definitions.
//
// Build: iverilog -g2005 -o tb_alu_int sim/tb_alu_int.v rtl/alu_int.v rtl/recip.v
// =============================================================================

`timescale 1ns/1ps

module tb_alu_int;

reg clk = 1'b0;
reg rst_n = 1'b0;
always #5 clk = ~clk;

reg  [5:0]  opcode;
reg  [63:0] src_a, src_b;
reg  [1:0]  shift_amt;
reg  [63:0] imm32_sext;
reg  [3:0]  cond;
reg         mem_is_l1;
reg         en;

wire [63:0] result;
wire        result_valid;
wire        busy;
wire        branch_taken;
wire        mem_wr_en;
wire [63:0] mem_wr_addr, mem_wr_data;
wire [1:0]  mem_wr_level;

alu_int u_dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (en),
    .opcode       (opcode),
    .src_a        (src_a),
    .src_b        (src_b),
    .shift_amt    (shift_amt),
    .imm32_sext   (imm32_sext),
    .cond         (cond),
    .mem_is_l1    (mem_is_l1),
    .result       (result),
    .result_valid (result_valid),
    .busy         (busy),
    .branch_taken (branch_taken),
    .mem_wr_en    (mem_wr_en),
    .mem_wr_addr  (mem_wr_addr),
    .mem_wr_data  (mem_wr_data),
    .mem_wr_level (mem_wr_level)
);

localparam OP_IADD_RS   = 6'd0;
localparam OP_IADD_M    = 6'd1;
localparam OP_ISUB_R    = 6'd2;
localparam OP_ISUB_M    = 6'd3;
localparam OP_IMUL_R    = 6'd4;
localparam OP_IMUL_M    = 6'd5;
localparam OP_IMULH_R   = 6'd6;
localparam OP_IMULH_M   = 6'd7;
localparam OP_ISMULH_R  = 6'd8;
localparam OP_ISMULH_M  = 6'd9;
localparam OP_IMUL_RCP  = 6'd10;
localparam OP_INEG_R    = 6'd11;
localparam OP_IXOR_R    = 6'd12;
localparam OP_IXOR_M    = 6'd13;
localparam OP_IROR_R    = 6'd14;
localparam OP_IROL_R    = 6'd15;
localparam OP_ISWAP_R   = 6'd16;
localparam OP_CBRANCH   = 6'd17;
localparam OP_ISTORE    = 6'd18;

integer errors = 0;
integer cyc;

// ---------------------------------------------------------------------------
// Issue one operation and wait for result_valid (IMUL_RCP takes 64+bsr cycles)
// ---------------------------------------------------------------------------
task issue;
    input [5:0]  op;
    input [63:0] a;
    input [63:0] b;
    begin
        @(posedge clk);
        #0.1;
        opcode = op;
        src_a  = a;
        src_b  = b;
        en     = 1'b1;
        @(posedge clk);
        #0.1;
        en = 1'b0;
        cyc = 0;
        while (!result_valid && cyc < 200) begin
            @(posedge clk);
            #0.1;
            cyc = cyc + 1;
        end
        if (cyc >= 200) begin
            $display("FAIL opcode %0d: result_valid never asserted", op);
            errors = errors + 1;
        end
    end
endtask

task chk;
    input [255:0] name;
    input [63:0]  got;
    input [63:0]  exp;
    begin
        if (got !== exp) begin
            $display("FAIL %0s: got=%h exp=%h", name, got, exp);
            errors = errors + 1;
        end else begin
            $display("PASS %0s = %h", name, got);
        end
    end
endtask

task chk1;
    input [255:0] name;
    input         got;
    input         exp;
    begin
        if (got !== exp) begin
            $display("FAIL %0s: got=%b exp=%b", name, got, exp);
            errors = errors + 1;
        end else begin
            $display("PASS %0s = %b", name, got);
        end
    end
endtask

initial begin
    en         = 1'b0;
    opcode     = 6'd0;
    src_a      = 64'b0;
    src_b      = 64'b0;
    shift_amt  = 2'd0;
    imm32_sext = 64'b0;
    cond       = 4'd0;
    mem_is_l1  = 1'b0;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // ---- IADD_RS: dst + (src << shift) + imm32 ----
    shift_amt  = 2'd3;
    imm32_sext = 64'h0000000000000010;
    issue(OP_IADD_RS, 64'h0000000000000100, 64'h0000000000000002);
    chk("IADD_RS", result, 64'h0000000000000100 + (64'h2 << 3) + 64'h10);
    shift_amt  = 2'd0;
    imm32_sext = 64'b0;

    // ---- wrap-around add ----
    issue(OP_IADD_M, 64'hFFFFFFFFFFFFFFFF, 64'h0000000000000002);
    chk("IADD_M wrap", result, 64'h0000000000000001);

    issue(OP_ISUB_R, 64'h0000000000000001, 64'h0000000000000002);
    chk("ISUB_R borrow", result, 64'hFFFFFFFFFFFFFFFF);

    issue(OP_ISUB_M, 64'h0123456789ABCDEF, 64'h0000000000000EF0);
    chk("ISUB_M", result, 64'h0123456789ABCDEF - 64'hEF0);

    // ---- multiplies ----
    issue(OP_IMUL_R, 64'hFFFFFFFFFFFFFFFF, 64'h0000000000000003);
    chk("IMUL_R low", result, 64'hFFFFFFFFFFFFFFFD);

    issue(OP_IMUL_M, 64'h0000000100000000, 64'h0000000100000000);
    chk("IMUL_M low", result, 64'h0000000000000000);

    issue(OP_IMULH_R, 64'hFFFFFFFFFFFFFFFF, 64'hFFFFFFFFFFFFFFFF);
    chk("IMULH_R", result, 64'hFFFFFFFFFFFFFFFE);

    issue(OP_IMULH_M, 64'h0000000100000000, 64'h0000000100000000);
    chk("IMULH_M", result, 64'h0000000000000001);

    // -1 * -1 = 1 -> high word 0
    issue(OP_ISMULH_R, 64'hFFFFFFFFFFFFFFFF, 64'hFFFFFFFFFFFFFFFF);
    chk("ISMULH_R (-1*-1)", result, 64'h0000000000000000);

    // -2^32 * 2^32 = -2^64 -> high word -1
    issue(OP_ISMULH_M, 64'hFFFFFFFF00000000, 64'h0000000100000000);
    chk("ISMULH_M (neg)", result, 64'hFFFFFFFFFFFFFFFF);

    // ---- IMUL_RCP (multi-cycle) ----
    // reciprocal(3) = 0xaaaaaaaaaaaaaaaa
    issue(OP_IMUL_RCP, 64'h0000000000000001, 64'h0000000000000003);
    chk("IMUL_RCP rcp(3)", result, 64'hAAAAAAAAAAAAAAAA);

    issue(OP_IMUL_RCP, 64'h0000000000000001, 64'h0000000000000007);
    chk("IMUL_RCP rcp(7)", result, 64'h9249249249249249);

    issue(OP_IMUL_RCP, 64'h0000000000000001, 64'h00000000FFFFFFFF);
    chk("IMUL_RCP rcp(2^32-1)", result, 64'h8000000080000000);

    issue(OP_IMUL_RCP, 64'h0000000000000001, 64'h000000007FFFFFFF);
    chk("IMUL_RCP rcp(2^31-1)", result, 64'h8000000100000002);

    issue(OP_IMUL_RCP, 64'h0000000000000001, 64'h0000000012345678);
    chk("IMUL_RCP rcp(0x12345678)", result, 64'hE10000077880003F);

    // dst is multiplied by the reciprocal
    issue(OP_IMUL_RCP, 64'h0000000000000003, 64'h0000000000000003);
    chk("IMUL_RCP 3*rcp(3)", result, 64'hAAAAAAAAAAAAAAAA * 64'd3);

    // powers of two overflow the 64-bit quotient and yield zero
    issue(OP_IMUL_RCP, 64'h0123456789ABCDEF, 64'h0000000000000002);
    chk("IMUL_RCP rcp(2)", result, 64'h0000000000000000);

    // ---- logic / rotate ----
    issue(OP_INEG_R, 64'h0000000000000001, 64'h0);
    chk("INEG_R", result, 64'hFFFFFFFFFFFFFFFF);

    issue(OP_IXOR_R, 64'hFFFF0000FFFF0000, 64'h00FF00FF00FF00FF);
    chk("IXOR_R", result, 64'hFF0000FFFF0000FF);

    issue(OP_IXOR_M, 64'h0123456789ABCDEF, 64'hFFFFFFFFFFFFFFFF);
    chk("IXOR_M", result, 64'hFEDCBA9876543210);

    issue(OP_IROR_R, 64'h0000000000000001, 64'd1);
    chk("IROR_R by 1", result, 64'h8000000000000000);

    issue(OP_IROR_R, 64'h0123456789ABCDEF, 64'd0);
    chk("IROR_R by 0", result, 64'h0123456789ABCDEF);

    issue(OP_IROL_R, 64'h8000000000000000, 64'd1);
    chk("IROL_R by 1", result, 64'h0000000000000001);

    issue(OP_IROL_R, 64'h0123456789ABCDEF, 64'd64);   // src[5:0] = 0
    chk("IROL_R by 64", result, 64'h0123456789ABCDEF);

    issue(OP_ISWAP_R, 64'hAAAAAAAAAAAAAAAA, 64'h5555555555555555);
    chk("ISWAP_R", result, 64'h5555555555555555);

    // ---- CBRANCH (spec §5.5.10) ----
    // cond = 0 -> shift = 8; imm = (0 | 1<<8) & ~(1<<7) = 0x100
    // dst = 0 -> res = 0x100, (res & (0xFF << 8)) = 0x100 != 0 -> not taken
    cond       = 4'd0;
    imm32_sext = 64'h0;
    issue(OP_CBRANCH, 64'h0, 64'h0);
    chk ("CBRANCH result", result, 64'h100);
    chk1("CBRANCH not taken", branch_taken, 1'b0);

    // dst = 0xFF00 -> res = 0x10000, condition byte (bits 8..15) is zero
    issue(OP_CBRANCH, 64'hFF00, 64'h0);
    chk ("CBRANCH result 2", result, 64'h10000);
    chk1("CBRANCH taken", branch_taken, 1'b1);

    // ---- ISTORE: no register write, level selected by cond/mem ----
    imm32_sext = 64'h0000000000000040;
    cond       = 4'd0;
    mem_is_l1  = 1'b1;
    @(posedge clk); #0.1;
    opcode = OP_ISTORE; src_a = 64'h1000; src_b = 64'hDEADBEEFCAFEBABE; en = 1'b1;
    @(posedge clk); #0.1;
    en = 1'b0;
    chk1("ISTORE mem_wr_en", mem_wr_en, 1'b1);
    chk1("ISTORE no regwrite", result_valid, 1'b0);
    chk ("ISTORE addr", mem_wr_addr, 64'h1040);
    chk ("ISTORE data", mem_wr_data, 64'hDEADBEEFCAFEBABE);
    chk ("ISTORE level L1", {62'b0, mem_wr_level}, 64'd0);

    mem_is_l1 = 1'b0;
    @(posedge clk); #0.1;
    opcode = OP_ISTORE; en = 1'b1;
    @(posedge clk); #0.1;
    en = 1'b0;
    chk("ISTORE level L2", {62'b0, mem_wr_level}, 64'd1);

    cond = 4'd14;   // mod.cond >= 14 -> L3
    @(posedge clk); #0.1;
    opcode = OP_ISTORE; en = 1'b1;
    @(posedge clk); #0.1;
    en = 1'b0;
    chk("ISTORE level L3", {62'b0, mem_wr_level}, 64'd2);

    if (errors == 0)
        $display("=== tb_alu_int: ALL TESTS PASSED ===");
    else
        $display("=== tb_alu_int: %0d FAILURE(S) ===", errors);

    $finish;
end

initial begin
    #500000;
    $display("ERROR: tb_alu_int timeout");
    $finish;
end

endmodule
