// =============================================================================
// aes_gen4r.v — AesGenerator4R
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX spec: AesGenerator4R applies 4 AES rounds per 128-bit lane.
// State: 4 × 128-bit lanes (512 bits = 64 bytes).
// Generates pseudorandom data for the scratchpad fill phase.
//
// Implementation: 4-stage sequential FSM (one AES round per cycle per lane).
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module aes_gen4r (
    input  wire         clk,
    input  wire         rst_n,
    // Start pulse: load state_in and begin 4-round sequence
    input  wire         start,
    // 512-bit input state (4 × 128-bit lanes)
    input  wire [511:0] state_in,
    // 512-bit output state after 4 AES rounds per lane
    output reg  [511:0] state_out,
    // valid pulses 4 cycles after start
    output reg          valid
);

// ---------------------------------------------------------------------------
// Round keys — 8 × 128-bit keys, 2 per round stage
// TODO: derive per-lane keys from seed per RandomX spec section 3.3
// ---------------------------------------------------------------------------
localparam [127:0] RK0 = 128'h9f3169c04a1a35ba0ed30095da25baba;
localparam [127:0] RK1 = 128'hf5dba23527af0a5fca5c74a5f7d4a3ab;
localparam [127:0] RK2 = 128'h1b5af2d0a3f78cd7f3e28d56e0eae7be;
localparam [127:0] RK3 = 128'hd56fd8d3cf55b7b29a4b8be0e43b5b5f;
localparam [127:0] RK4 = 128'h78f56cb36de7b20b07af3a11c63c7699;
localparam [127:0] RK5 = 128'h0aa7ee8b4fc8ff8af89ca42b73c19e8c;
localparam [127:0] RK6 = 128'hb9e10da2fd4ad3acf9a1b0e9badb6de3;
localparam [127:0] RK7 = 128'hba36f0b5f0de6aa3a99f36d4b5c4c8ab;

// ---------------------------------------------------------------------------
// Internal state registers — hold the 4 lanes between rounds
// ---------------------------------------------------------------------------
reg [127:0] lane0, lane1, lane2, lane3;
reg [1:0]   round_cnt; // 0..3 counts AES rounds completed
reg         running;

// ---------------------------------------------------------------------------
// Round-key MUX (select based on round and lane)
// ---------------------------------------------------------------------------
reg [127:0] rk_l0, rk_l1, rk_l2, rk_l3;

always @(*) begin
    case (round_cnt)
        2'd0: begin rk_l0 = RK0; rk_l1 = RK0; rk_l2 = RK1; rk_l3 = RK1; end
        2'd1: begin rk_l0 = RK2; rk_l1 = RK2; rk_l2 = RK3; rk_l3 = RK3; end
        2'd2: begin rk_l0 = RK4; rk_l1 = RK4; rk_l2 = RK5; rk_l3 = RK5; end
        2'd3: begin rk_l0 = RK6; rk_l1 = RK6; rk_l2 = RK7; rk_l3 = RK7; end
        default: begin rk_l0 = 128'b0; rk_l1 = 128'b0; rk_l2 = 128'b0; rk_l3 = 128'b0; end
    endcase
end

// ---------------------------------------------------------------------------
// AES round combinational outputs for each lane
// ---------------------------------------------------------------------------
wire [127:0] out0, out1, out2, out3;

aes_round u_rnd0 (.state_in(lane0), .round_key(rk_l0), .last_round(1'b0), .state_out(out0));
aes_round u_rnd1 (.state_in(lane1), .round_key(rk_l1), .last_round(1'b0), .state_out(out1));
aes_round u_rnd2 (.state_in(lane2), .round_key(rk_l2), .last_round(1'b0), .state_out(out2));
aes_round u_rnd3 (.state_in(lane3), .round_key(rk_l3), .last_round(1'b0), .state_out(out3));

// ---------------------------------------------------------------------------
// FSM: 4-round counter
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lane0     <= 128'b0;
        lane1     <= 128'b0;
        lane2     <= 128'b0;
        lane3     <= 128'b0;
        round_cnt <= 2'd0;
        running   <= 1'b0;
        state_out <= 512'b0;
        valid     <= 1'b0;
    end else begin
        valid <= 1'b0;

        if (start) begin
            // Load input lanes and begin
            lane0     <= state_in[127:  0];
            lane1     <= state_in[255:128];
            lane2     <= state_in[383:256];
            lane3     <= state_in[511:384];
            round_cnt <= 2'd0;
            running   <= 1'b1;
        end else if (running) begin
            // Apply one AES round to all lanes
            lane0     <= out0;
            lane1     <= out1;
            lane2     <= out2;
            lane3     <= out3;
            round_cnt <= round_cnt + 2'd1;

            if (round_cnt == 2'd3) begin
                // Final round done
                running   <= 1'b0;
                state_out <= {out3, out2, out1, out0};
                valid     <= 1'b1;
            end
        end
    end
end

endmodule
