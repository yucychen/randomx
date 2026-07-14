// =============================================================================
// aes_gen1r.v — AesGenerator1R
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX spec: AesGenerator1R applies 1 AES round per 128-bit lane.
// State: 4 × 128-bit lanes (512 bits total = 64 bytes)
// Each clock cycle performs one AES round on all 4 lanes simultaneously.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module aes_gen1r (
    input  wire         clk,
    input  wire         rst_n,
    // Start pulse: load state_in and begin
    input  wire         start,
    // 512-bit input state (4 × 128-bit lanes)
    input  wire [511:0] state_in,
    // 512-bit output state (updated after 1 AES round per lane)
    output reg  [511:0] state_out,
    // output valid pulse
    output reg          valid
);

// ---------------------------------------------------------------------------
// Round keys for AesGenerator1R (hardcoded per RandomX spec, sec 3.2)
// TODO: Load from initialisation register for full spec compliance
// ---------------------------------------------------------------------------
localparam [127:0] RK0 = 128'h9f3169c04a1a35ba0ed30095da25baba;
localparam [127:0] RK1 = 128'hf5dba23527af0a5fca5c74a5f7d4a3ab;
localparam [127:0] RK2 = 128'h1b5af2d0a3f78cd7f3e28d56e0eae7be;
localparam [127:0] RK3 = 128'h00000000000000000000000000000000; // TODO: spec value

// Wires for AES round outputs per lane
wire [127:0] lane_in  [0:3];
wire [127:0] lane_out [0:3];

assign lane_in[0] = state_in[127:  0];
assign lane_in[1] = state_in[255:128];
assign lane_in[2] = state_in[383:256];
assign lane_in[3] = state_in[511:384];

// Instantiate one aes_round per lane (full round, not last round)
aes_round u_rnd0 (
    .state_in  (lane_in[0]),
    .round_key (RK0),
    .last_round(1'b0),
    .state_out (lane_out[0])
);

aes_round u_rnd1 (
    .state_in  (lane_in[1]),
    .round_key (RK1),
    .last_round(1'b0),
    .state_out (lane_out[1])
);

aes_round u_rnd2 (
    .state_in  (lane_in[2]),
    .round_key (RK2),
    .last_round(1'b0),
    .state_out (lane_out[2])
);

aes_round u_rnd3 (
    .state_in  (lane_in[3]),
    .round_key (RK3),
    .last_round(1'b0),
    .state_out (lane_out[3])
);

// ---------------------------------------------------------------------------
// Register outputs
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state_out <= 512'b0;
        valid     <= 1'b0;
    end else begin
        valid <= 1'b0;
        if (start) begin
            state_out <= {lane_out[3], lane_out[2], lane_out[1], lane_out[0]};
            valid     <= 1'b1;
        end
    end
end

endmodule
