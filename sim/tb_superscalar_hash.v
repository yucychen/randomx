// =============================================================================
// tb_superscalar_hash.v — unit test for rtl/superscalar_hash.v
//
// Runs a 16-instruction program covering every SuperscalarHash opcode
// (ISUB_R, IXOR_R, IADD_RS, IMUL_R, IROR_C, IADD_C7/C8/C9, IXOR_C7/C8/C9,
//  IMULH_R, ISMULH_R, IMUL_RCP) and compares the final register file with
// reference values produced by the RandomX software model.
//
// Build & run:
//   iverilog -g2001 -o sim/tb_superscalar_hash.vvp \
//       rtl/alu_int.v rtl/superscalar_hash.v sim/tb_superscalar_hash.v
//   vvp sim/tb_superscalar_hash.vvp
// =============================================================================

`timescale 1ns/1ps

module tb_superscalar_hash;

reg         clk = 1'b0;
reg         rst_n = 1'b0;
reg         start = 1'b0;
reg         prog_wr_en = 1'b0;
reg  [11:0] prog_wr_addr = 12'd0;
reg  [63:0] prog_wr_data = 64'd0;
reg  [11:0] prog_len = 12'd0;
reg  [11:0] prog_base = 12'd0;
reg  [63:0] init_r [0:7];
wire [63:0] out_r0, out_r1, out_r2, out_r3, out_r4, out_r5, out_r6, out_r7;
wire        busy, done;

reg  [63:0] prog [0:15];
reg  [63:0] exp  [0:7];
reg  [63:0] got  [0:7];

integer i;
integer errors = 0;
integer cycles = 0;

always #5 clk = ~clk;

superscalar_hash dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (start),
    .prog_wr_en   (prog_wr_en),
    .prog_wr_addr (prog_wr_addr),
    .prog_wr_data (prog_wr_data),
    .prog_base    (prog_base),
    .prog_len     (prog_len),
    .init_r0      (init_r[0]), .init_r1 (init_r[1]),
    .init_r2      (init_r[2]), .init_r3 (init_r[3]),
    .init_r4      (init_r[4]), .init_r5 (init_r[5]),
    .init_r6      (init_r[6]), .init_r7 (init_r[7]),
    .out_r0       (out_r0), .out_r1 (out_r1),
    .out_r2       (out_r2), .out_r3 (out_r3),
    .out_r4       (out_r4), .out_r5 (out_r5),
    .out_r6       (out_r6), .out_r7 (out_r7),
    .busy         (busy),
    .done         (done)
);

// Encoding: [63:56]=opcode [55:53]=dst [52:50]=src [49:48]=mod_shift [31:0]=imm32
initial begin
    prog[ 0] = 64'h0207000000000000; // IADD_RS  r0 += r1 << 3
    prog[ 1] = 64'h0028000000000000; // ISUB_R   r1 -= r2
    prog[ 2] = 64'h014C000000000000; // IXOR_R   r2 ^= r3
    prog[ 3] = 64'h0370000000000000; // IMUL_R   r3 *= r4
    prog[ 4] = 64'h048000000000002D; // IROR_C   r4 = ror(r4, 45)
    prog[ 5] = 64'h05A0000089ABCDEF; // IADD_C7  r5 += sext(imm)
    prog[ 6] = 64'h06C0000000FF00FF; // IXOR_C7  r6 ^= sext(imm)
    prog[ 7] = 64'h070000007FFFFFFF; // IADD_C8  r0 += sext(imm)
    prog[ 8] = 64'h08200000FFFFFFFF; // IXOR_C8  r1 ^= sext(imm)
    prog[ 9] = 64'h0940000000000001; // IADD_C9  r2 += 1
    prog[10] = 64'h0A60000012345678; // IXOR_C9  r3 ^= imm
    prog[11] = 64'h0B94000000000000; // IMULH_R  r4 = hi_u(r4 * r5)
    prog[12] = 64'h0CB8000000000000; // ISMULH_R r5 = hi_s(r5 * r6)
    prog[13] = 64'h0DC000000000000B; // IMUL_RCP r6 *= reciprocal(11)
    prog[14] = 64'h0DE00000DEADBEEF; // IMUL_RCP r7 *= reciprocal(0xDEADBEEF)
    prog[15] = 64'h0CE0000000000000; // ISMULH_R r7 = hi_s(r7 * r0)

    init_r[0] = 64'h0123456789ABCDEF;
    init_r[1] = 64'h1032547698BADCFE;
    init_r[2] = 64'h23016745AB89EFCD;
    init_r[3] = 64'h32107654BA98FEDC;
    init_r[4] = 64'h45670123CDEF89AB;
    init_r[5] = 64'h54761032DCFE98BA;
    init_r[6] = 64'h67452301EFCDAB89;
    init_r[7] = 64'h76543210FEDCBA98;

    exp[0] = 64'h82B5E91CCF82B5DE;
    exp[1] = 64'h12CF12CF12CF12CE;
    exp[2] = 64'h1111111111111112;
    exp[3] = 64'h8546CF748DCDAE8C;
    exp[4] = 64'h0302312FDF3EFB24;
    exp[5] = 64'h22124FE36C7C2037;
    exp[6] = 64'h1B5FA3E77F0E4D12;
    exp[7] = 64'h199ADF62D133E707;
end

// Watchdog
always @(posedge clk) begin
    cycles = cycles + 1;
    if (cycles > 100000) begin
        $display("FAIL: timeout waiting for done");
        $finish;
    end
end

initial begin
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);

    // Load the program
    for (i = 0; i < 16; i = i + 1) begin
        @(negedge clk);
        prog_wr_en   = 1'b1;
        prog_wr_addr = i[11:0];
        prog_wr_data = prog[i];
    end
    @(negedge clk);
    prog_wr_en = 1'b0;
    prog_len   = 12'd16;

    // Start execution
    @(negedge clk);
    start = 1'b1;
    @(negedge clk);
    start = 1'b0;

    wait (done === 1'b1);
    @(posedge clk);

    got[0] = out_r0; got[1] = out_r1; got[2] = out_r2; got[3] = out_r3;
    got[4] = out_r4; got[5] = out_r5; got[6] = out_r6; got[7] = out_r7;

    for (i = 0; i < 8; i = i + 1) begin
        if (got[i] !== exp[i]) begin
            errors = errors + 1;
            $display("MISMATCH r%0d: got %016h expected %016h", i, got[i], exp[i]);
        end else begin
            $display("OK       r%0d: %016h", i, got[i]);
        end
    end

    if (errors == 0)
        $display("PASS: SuperscalarHash program executed correctly (%0d cycles)", cycles);
    else
        $display("FAIL: %0d register mismatches", errors);

    // -----------------------------------------------------------------------
    // Boundary check: the last program window (base 3584) with the maximum
    // program length (512) ends exactly at the top of the 4096-word program
    // buffer, so the fetch loop must still terminate.
    // -----------------------------------------------------------------------
    prog_base = 12'd3584;
    for (i = 0; i < 512; i = i + 1) begin
        @(negedge clk);
        prog_wr_en   = 1'b1;
        prog_wr_addr = 12'd3584 + i[11:0];
        prog_wr_data = 64'h0500000000000001; // IADD_C7  r0 += 1
    end
    @(negedge clk);
    prog_wr_en = 1'b0;
    prog_len   = 12'd512;

    @(negedge clk);
    start = 1'b1;
    @(negedge clk);
    start = 1'b0;

    wait (done === 1'b1);
    @(posedge clk);

    if (out_r0 !== (init_r[0] + 64'd512)) begin
        errors = errors + 1;
        $display("FAIL: last program window: r0 = %016h expected %016h",
                 out_r0, init_r[0] + 64'd512);
    end else begin
        $display("PASS: last program window (base 3584, len 512) terminated");
    end

    $finish;
end

endmodule
