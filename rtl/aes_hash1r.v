// =============================================================================
// aes_hash1r.v — AesHash1R
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX spec: AesHash1R hashes a 64-byte input using 4 AES rounds per lane.
// Used in the final hash step (squeezing the scratchpad into a 512-bit digest).
//
// 4 lanes × 128-bit, 4 AES rounds each (sequential FSM).
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module aes_hash1r (
    input  wire         clk,
    input  wire         rst_n,
    // Start pulse: load data_in, begin hashing
    input  wire         start,
    // 512-bit input data (64 bytes)
    input  wire [511:0] data_in,
    // 512-bit hash output
    output reg  [511:0] hash_out,
    // Valid pulse (4 cycles after start)
    output reg          valid
);

// ---------------------------------------------------------------------------
// AesHash1R uses different round keys than AesGenerator
// TODO: Set spec-correct values per RandomX spec section 3.4
// ---------------------------------------------------------------------------
localparam [127:0] RKH0 = 128'h7418aaaaf3bd5422a06f7aadb91ec0c0;
localparam [127:0] RKH1 = 128'he0b96f8dc7c849a3a16b87a6c4e2e42a;
localparam [127:0] RKH2 = 128'ha5df0a4b7e33a76e4a3c98e4c0a95d59;
localparam [127:0] RKH3 = 128'h1a41f43a7d2e3b42a60edd5bc42c28f5;

// Lane registers
reg [127:0] lane0, lane1, lane2, lane3;
reg [1:0]   round_cnt;
reg         running;

// Round key MUX
reg [127:0] rk_cur;
always @(*) begin
    case (round_cnt)
        2'd0: rk_cur = RKH0;
        2'd1: rk_cur = RKH1;
        2'd2: rk_cur = RKH2;
        2'd3: rk_cur = RKH3;
        default: rk_cur = 128'b0;
    endcase
end

// AES round instances (all lanes share the same key for AesHash1R)
wire [127:0] out0, out1, out2, out3;

aes_round u_h0 (.state_in(lane0), .round_key(rk_cur), .last_round(1'b0), .state_out(out0));
aes_round u_h1 (.state_in(lane1), .round_key(rk_cur), .last_round(1'b0), .state_out(out1));
aes_round u_h2 (.state_in(lane2), .round_key(rk_cur), .last_round(1'b0), .state_out(out2));
aes_round u_h3 (.state_in(lane3), .round_key(rk_cur), .last_round(1'b0), .state_out(out3));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lane0     <= 128'b0;
        lane1     <= 128'b0;
        lane2     <= 128'b0;
        lane3     <= 128'b0;
        round_cnt <= 2'd0;
        running   <= 1'b0;
        hash_out  <= 512'b0;
        valid     <= 1'b0;
    end else begin
        valid <= 1'b0;

        if (start) begin
            lane0     <= data_in[127:  0];
            lane1     <= data_in[255:128];
            lane2     <= data_in[383:256];
            lane3     <= data_in[511:384];
            round_cnt <= 2'd0;
            running   <= 1'b1;
        end else if (running) begin
            lane0     <= out0;
            lane1     <= out1;
            lane2     <= out2;
            lane3     <= out3;
            round_cnt <= round_cnt + 2'd1;

            if (round_cnt == 2'd3) begin
                running  <= 1'b0;
                hash_out <= {out3, out2, out1, out0};
                valid    <= 1'b1;
            end
        end
    end
end

endmodule
