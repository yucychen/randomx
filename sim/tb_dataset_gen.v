// =============================================================================
// tb_dataset_gen.v — self-checking testbench for dataset_gen.v
//
// Checks the dataset item generation datapath (RandomX spec §7.3):
//   - register seeding from the item index
//   - 8 cache accesses with the spec's line mask and address-register chain
//   - SuperscalarHash execution between the accesses (one program carries a
//     single IADD_C7 instruction, the others are empty)
//   - the 64-byte item written on the dataset write port
//
// The expected values are computed by a behavioural golden model driven from
// the same cache contents as the behavioural cache model.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module tb_dataset_gen;

localparam CACHE_LINES = 32;         // 2 KiB cache → 2 × 1 KiB blocks
localparam CACHE_BLOCKS = CACHE_LINES / 16;
localparam ITEM_COUNT  = 4;

// SuperscalarHash program under test: program 3 executes `r2 += imm`
localparam [2:0]  TEST_PROG   = 3'd3;
localparam [31:0] TEST_IMM    = 32'h12345678;
localparam [63:0] TEST_INSTR  = {8'd5, 3'd2, 3'd0, 2'd0, 16'd0, TEST_IMM};

localparam [63:0] SS_MUL0 = 64'h5851F42D4C957F2D;
localparam [63:0] SS_ADD1 = 64'h810A978A59F5A1FC;
localparam [63:0] SS_ADD2 = 64'hA77099DF38C2D846;
localparam [63:0] SS_ADD3 = 64'h8126B91CBF22495C;
localparam [63:0] SS_ADD4 = 64'h494D2597179F8A62;
localparam [63:0] SS_ADD5 = 64'h9237EFB9CEAAEC0C;
localparam [63:0] SS_ADD6 = 64'h2F2A56746CE62D78;
localparam [63:0] SS_ADD7 = 64'h84853BF7B62CE54E;

reg clk = 1'b0;
reg rst_n;
always #1.667 clk = ~clk;

integer errors = 0;

// ---------------------------------------------------------------------------
// DUT
// ---------------------------------------------------------------------------
reg          start;
reg          prog_wr_en;
reg  [11:0]  prog_wr_addr;
reg  [63:0]  prog_wr_data;
reg          cfg_wr_en;
reg  [2:0]   cfg_wr_sel;
reg  [11:0]  cfg_wr_len;
reg  [2:0]   cfg_wr_addr_reg;

wire         cache_rd_en;
wire [31:0]  cache_rd_addr;
reg  [8191:0] cache_rd_data;
reg          cache_rd_valid;

wire         ds_wr_valid;
wire [31:0]  ds_wr_item_idx;
wire [511:0] ds_wr_data;
reg          ds_wr_ready;

wire         busy;
wire         done;

dataset_gen #(
    .CACHE_LINES (CACHE_LINES),
    .ITEM_COUNT  (ITEM_COUNT),
    .ACCESSES    (8)
) dut (
    .clk             (clk),
    .rst_n           (rst_n),
    .start           (start),
    .prog_wr_en      (prog_wr_en),
    .prog_wr_addr    (prog_wr_addr),
    .prog_wr_data    (prog_wr_data),
    .cfg_wr_en       (cfg_wr_en),
    .cfg_wr_sel      (cfg_wr_sel),
    .cfg_wr_len      (cfg_wr_len),
    .cfg_wr_addr_reg (cfg_wr_addr_reg),
    .cache_rd_en     (cache_rd_en),
    .cache_rd_addr   (cache_rd_addr),
    .cache_rd_data   (cache_rd_data),
    .cache_rd_valid  (cache_rd_valid),
    .ds_wr_valid     (ds_wr_valid),
    .ds_wr_item_idx  (ds_wr_item_idx),
    .ds_wr_data      (ds_wr_data),
    .ds_wr_ready     (ds_wr_ready),
    .busy            (busy),
    .done            (done)
);

// ---------------------------------------------------------------------------
// Behavioural cache model — 1 KiB blocks, variable read latency
// ---------------------------------------------------------------------------
reg [63:0] cache_word [0:CACHE_LINES*8-1];   // 8 × 64-bit words per 64-byte line

integer i, j;
reg [8191:0] blk_tmp;

// Build the 1 KiB block requested by the DUT
task build_block;
    input  integer blk;
    integer w;
    begin
        blk_tmp = {8192{1'b0}};
        for (w = 0; w < 128; w = w + 1)
            blk_tmp[64*w +: 64] = cache_word[blk*128 + w];
    end
endtask

reg [2:0] cache_lat;
reg       cache_serving;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cache_rd_valid <= 1'b0;
        cache_rd_data  <= {8192{1'b0}};
        cache_lat      <= 3'd0;
        cache_serving  <= 1'b0;
    end else begin
        cache_rd_valid <= 1'b0;
        if (!cache_serving) begin
            if (cache_rd_en) begin
                cache_serving <= 1'b1;
                cache_lat     <= 3'd3;
            end
        end else if (cache_lat != 3'd0) begin
            cache_lat <= cache_lat - 3'd1;
        end else begin
            build_block(cache_rd_addr % CACHE_BLOCKS);
            cache_rd_data  <= blk_tmp;
            cache_rd_valid <= 1'b1;
            cache_serving  <= 1'b0;
        end
    end
end

// ---------------------------------------------------------------------------
// Dataset write sink (with backpressure)
// ---------------------------------------------------------------------------
reg [511:0] got_item [0:ITEM_COUNT-1];
reg         got_flag [0:ITEM_COUNT-1];
reg [3:0]   bp_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ds_wr_ready <= 1'b0;
        bp_cnt      <= 4'd0;
    end else begin
        bp_cnt      <= bp_cnt + 4'd1;
        ds_wr_ready <= bp_cnt[1];      // toggle ready to exercise the handshake
        if (ds_wr_valid && ds_wr_ready) begin
            if (ds_wr_item_idx < ITEM_COUNT) begin
                got_item[ds_wr_item_idx] <= ds_wr_data;
                got_flag[ds_wr_item_idx] <= 1'b1;
            end
        end
    end
end

// ---------------------------------------------------------------------------
// Golden model
// ---------------------------------------------------------------------------
reg [63:0] g_r [0:7];
reg [63:0] g_regval;
reg [63:0] g_seed;
reg [63:0] exp_item [0:ITEM_COUNT-1];   // only used through exp_full
reg [511:0] exp_full [0:ITEM_COUNT-1];

task golden_item;
    input integer item;
    integer acc, q;
    integer line;
    begin
        g_seed  = (item + 1) * SS_MUL0;
        g_r[0]  = g_seed;
        g_r[1]  = g_seed ^ SS_ADD1;
        g_r[2]  = g_seed ^ SS_ADD2;
        g_r[3]  = g_seed ^ SS_ADD3;
        g_r[4]  = g_seed ^ SS_ADD4;
        g_r[5]  = g_seed ^ SS_ADD5;
        g_r[6]  = g_seed ^ SS_ADD6;
        g_r[7]  = g_seed ^ SS_ADD7;
        g_regval = item;

        for (acc = 0; acc < 8; acc = acc + 1) begin
            line = g_regval & (CACHE_LINES - 1);
            // SuperscalarHash: only TEST_PROG carries an instruction
            if (acc == TEST_PROG)
                g_r[2] = g_r[2] + {{32{TEST_IMM[31]}}, TEST_IMM};
            for (q = 0; q < 8; q = q + 1)
                g_r[q] = g_r[q] ^ cache_word[line*8 + q];
            g_regval = g_r[acc];        // address register of program acc is acc
        end

        exp_full[item] = {g_r[7], g_r[6], g_r[5], g_r[4],
                          g_r[3], g_r[2], g_r[1], g_r[0]};
    end
endtask

// ---------------------------------------------------------------------------
// Stimulus
// ---------------------------------------------------------------------------
reg [63:0] lfsr;
integer    poll;

initial begin
    $dumpfile("tb_dataset_gen.vcd");
    $dumpvars(0, tb_dataset_gen);

    rst_n           = 1'b0;
    start           = 1'b0;
    prog_wr_en      = 1'b0;
    prog_wr_addr    = 12'd0;
    prog_wr_data    = 64'd0;
    cfg_wr_en       = 1'b0;
    cfg_wr_sel      = 3'd0;
    cfg_wr_len      = 12'd0;
    cfg_wr_addr_reg = 3'd0;

    // Pseudo-random cache contents
    lfsr = 64'h0123456789ABCDEF;
    for (i = 0; i < CACHE_LINES*8; i = i + 1) begin
        lfsr = {lfsr[62:0], lfsr[63] ^ lfsr[62] ^ lfsr[60] ^ lfsr[59]};
        cache_word[i] = lfsr;
    end
    for (i = 0; i < ITEM_COUNT; i = i + 1)
        got_flag[i] = 1'b0;

    repeat (10) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // ---- Load the SuperscalarHash program configuration ----
    // address register of program i = i, all programs empty except TEST_PROG
    for (i = 0; i < 8; i = i + 1) begin
        @(posedge clk);
        cfg_wr_en       <= 1'b1;
        cfg_wr_sel      <= i[2:0];
        cfg_wr_len      <= (i == TEST_PROG) ? 12'd1 : 12'd0;
        cfg_wr_addr_reg <= i[2:0];
    end
    @(posedge clk);
    cfg_wr_en <= 1'b0;

    // ---- Load the single instruction of TEST_PROG ----
    @(posedge clk);
    prog_wr_en   <= 1'b1;
    prog_wr_addr <= TEST_PROG * 512;
    prog_wr_data <= TEST_INSTR;
    @(posedge clk);
    prog_wr_en   <= 1'b0;

    // ---- Golden model ----
    for (i = 0; i < ITEM_COUNT; i = i + 1)
        golden_item(i);

    // ---- Run ----
    @(posedge clk);
    start <= 1'b1;
    @(posedge clk);
    start <= 1'b0;

    poll = 0;
    while (!done && poll < 200000) begin
        @(posedge clk);
        poll = poll + 1;
    end

    if (!done) begin
        $display("FAIL: dataset_gen did not finish (timeout)");
        errors = errors + 1;
    end else begin
        $display("PASS: dataset generation finished after %0d cycles", poll);
    end

    // ---- Check ----
    for (i = 0; i < ITEM_COUNT; i = i + 1) begin
        if (!got_flag[i]) begin
            $display("FAIL: item %0d was never written", i);
            errors = errors + 1;
        end else if (got_item[i] !== exp_full[i]) begin
            $display("FAIL: item %0d mismatch", i);
            $display("   expected %h", exp_full[i]);
            $display("   got      %h", got_item[i]);
            errors = errors + 1;
        end else begin
            $display("PASS: item %0d matches the golden model", i);
        end
    end

    if (errors == 0)
        $display("=== tb_dataset_gen: ALL TESTS PASSED ===");
    else
        $display("=== tb_dataset_gen: %0d FAILURE(S) ===", errors);

    $finish;
end

endmodule
