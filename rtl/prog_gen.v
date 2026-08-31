// =============================================================================
// prog_gen.v — RandomX program / entropy generator (AesGenerator4R)
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// A RandomX program is 2176 bytes of AesGenerator4R output (spec §4.3):
//   bytes    0 .. 127  → 16 × 64-bit program configuration entropy words
//   bytes  128 .. 2175 → 256 × 64-bit instruction words
//
// AesGenerator4R produces 64 bytes per invocation and feeds its output state
// back as the next input state, so the program is generated in 34 blocks of
// 8 × 64-bit words. Words are emitted one per cycle on the VM's program /
// configuration write ports.
//
// `state_out` returns the generator state after the last block so the caller
// can chain further programs (spec: the same generator continues across the
// 8 programs of one hash).
//
// NOTE: the AES round keys of `aes_gen4r` are still placeholders (see the
// README roadmap item "AES 轮密钥"), so the generated program is structurally
// correct but not yet bit-compatible with the reference implementation.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module prog_gen (
    input  wire          clk,
    input  wire          rst_n,

    // Start pulse: expand `seed_in` into one program
    input  wire          start,
    input  wire [511:0]  seed_in,

    // VM program buffer write port (256 instruction words)
    output reg           prog_wr_en,
    output reg  [7:0]    prog_wr_addr,
    output reg  [63:0]   prog_wr_data,

    // VM program configuration entropy write port (16 words)
    output reg           cfg_wr_en,
    output reg  [3:0]    cfg_wr_addr,
    output reg  [63:0]   cfg_wr_data,

    // Generator state after the last block (seed for the next program)
    output reg  [511:0]  state_out,

    output reg           busy,
    output reg           done        // one-cycle pulse when the program is loaded
);

// 2176 bytes / 64 bytes = 34 AesGenerator4R blocks; the first 2 blocks carry
// the program configuration entropy, the remaining 32 the instruction words.
localparam [5:0] CFG_BLOCKS   = 6'd2;
localparam [5:0] TOTAL_BLOCKS = 6'd34;

// ---------------------------------------------------------------------------
// AesGenerator4R
// ---------------------------------------------------------------------------
reg          aes_start;
reg  [511:0] aes_state;
wire [511:0] aes_out;
wire         aes_valid;

aes_gen4r u_gen4r (
    .clk       (clk),
    .rst_n     (rst_n),
    .start     (aes_start),
    .state_in  (aes_state),
    .state_out (aes_out),
    .valid     (aes_valid)
);

// ---------------------------------------------------------------------------
// Control FSM
// ---------------------------------------------------------------------------
localparam ST_IDLE  = 2'd0;
localparam ST_GEN   = 2'd1;
localparam ST_EMIT  = 2'd2;
localparam ST_DONE  = 2'd3;

reg [1:0]   state;
reg [5:0]   blk_idx;    // 0 .. TOTAL_BLOCKS-1
reg [2:0]   word_idx;   // word inside the current 64-byte block
reg [511:0] blk_data;

// Word offset of the current block inside the instruction stream
wire [5:0] instr_blk  = blk_idx - CFG_BLOCKS;      // 0 .. 31
wire [7:0] instr_base = {instr_blk[4:0], 3'b0};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= ST_IDLE;
        busy         <= 1'b0;
        done         <= 1'b0;
        aes_start    <= 1'b0;
        aes_state    <= 512'b0;
        blk_idx      <= 6'd0;
        word_idx     <= 3'd0;
        blk_data     <= 512'b0;
        state_out    <= 512'b0;
        prog_wr_en   <= 1'b0;
        prog_wr_addr <= 8'b0;
        prog_wr_data <= 64'b0;
        cfg_wr_en    <= 1'b0;
        cfg_wr_addr  <= 4'b0;
        cfg_wr_data  <= 64'b0;
    end else begin
        aes_start  <= 1'b0;
        done       <= 1'b0;
        prog_wr_en <= 1'b0;
        cfg_wr_en  <= 1'b0;

        case (state)
            // -----------------------------------------------------------
            ST_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    aes_state <= seed_in;
                    aes_start <= 1'b1;
                    blk_idx   <= 6'd0;
                    busy      <= 1'b1;
                    state     <= ST_GEN;
                end
            end

            // -----------------------------------------------------------
            ST_GEN: begin
                if (aes_valid) begin
                    blk_data  <= aes_out;
                    aes_state <= aes_out;   // output state feeds the next block
                    word_idx  <= 3'd0;
                    state     <= ST_EMIT;
                end
            end

            // -----------------------------------------------------------
            // Emit the 8 × 64-bit words of the current block
            // -----------------------------------------------------------
            ST_EMIT: begin
                if (blk_idx < CFG_BLOCKS) begin
                    cfg_wr_en   <= 1'b1;
                    cfg_wr_addr <= {blk_idx[0], word_idx};
                    cfg_wr_data <= blk_data[{word_idx, 6'b0} +: 64];
                end else begin
                    prog_wr_en   <= 1'b1;
                    prog_wr_addr <= instr_base | {5'b0, word_idx};
                    prog_wr_data <= blk_data[{word_idx, 6'b0} +: 64];
                end

                if (word_idx == 3'd7) begin
                    if (blk_idx == (TOTAL_BLOCKS - 6'd1)) begin
                        state_out <= aes_state;
                        state     <= ST_DONE;
                    end else begin
                        blk_idx   <= blk_idx + 6'd1;
                        aes_start <= 1'b1;
                        state     <= ST_GEN;
                    end
                end else begin
                    word_idx <= word_idx + 3'd1;
                end
            end

            // -----------------------------------------------------------
            ST_DONE: begin
                // one idle cycle so the last program write has landed
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
