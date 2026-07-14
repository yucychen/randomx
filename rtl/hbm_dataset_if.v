// =============================================================================
// hbm_dataset_if.v — HBM2 Dataset AXI4 Master Interface Skeleton
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// The RandomX Dataset is ~2.08 GiB (RANDOMX_DATASET_ITEM_COUNT × 64 bytes).
// On the XCVU33P the 8 GB HBM2 is accessed via AXI4 (AXI3 compatible) ports.
// Each HBM pseudo-channel provides a 256-bit wide AXI4 slave port.
//
// This module is an AXI4 master skeleton that:
//   1. Accepts read requests from the VM (64-byte aligned addresses)
//   2. Issues AXI4 ARADDR bursts (1 beat of 512-bit or 4 beats of 128-bit)
//   3. Returns 64-byte dataset items to the VM via a response FIFO
//
// TODO: Connect to actual HBM AXI slave ports provided by Vivado HBM IP.
// TODO: Implement request queue, outstanding transaction tracking.
//
// AXI4 signals follow AMBA AXI4 spec (no QoS/user extensions needed).
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module hbm_dataset_if #(
    parameter AXI_ADDR_WIDTH = 34,  // 16 GB address space (XCVU33P HBM)
    parameter AXI_DATA_WIDTH = 256, // HBM pseudo-channel bus width
    parameter AXI_ID_WIDTH   = 6    // AXI ID width
) (
    input  wire                       clk,
    input  wire                       rst_n,

    // ---- VM request interface ----
    // Request: VM wants a 64-byte item at dataset_addr (item-index)
    input  wire                       req_valid,
    input  wire [31:0]                req_item_idx,  // dataset item index
    output reg                        req_ready,

    // Response: 64-byte dataset item returned to VM
    output reg                        resp_valid,
    output reg  [511:0]               resp_data,     // 64 bytes
    input  wire                       resp_ready,

    // ---- AXI4 Master (Read channel only — dataset is read-only) ----
    // AR channel
    output reg  [AXI_ID_WIDTH-1:0]   m_axi_arid,
    output reg  [AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output reg  [7:0]                 m_axi_arlen,   // burst length-1
    output reg  [2:0]                 m_axi_arsize,  // beat size (log2 bytes)
    output reg  [1:0]                 m_axi_arburst, // INCR = 2'b01
    output reg                        m_axi_arvalid,
    input  wire                       m_axi_arready,
    // R channel
    input  wire [AXI_ID_WIDTH-1:0]   m_axi_rid,
    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]                 m_axi_rresp,
    input  wire                       m_axi_rlast,
    input  wire                       m_axi_rvalid,
    output reg                        m_axi_rready
);

// ---------------------------------------------------------------------------
// Dataset item size = 64 bytes → with 256-bit bus → 2 beats per item
// ---------------------------------------------------------------------------
localparam BEATS_PER_ITEM = 2; // 64 bytes / 32 bytes per beat

// FSM states
localparam ST_IDLE     = 2'd0;
localparam ST_AR       = 2'd1;  // Issue AR request
localparam ST_RDATA    = 2'd2;  // Receive R data beats
localparam ST_RESP     = 2'd3;  // Forward response to VM

reg [1:0]  state;
reg [1:0]  beat_cnt;
reg [255:0] item_buf [0:1]; // 2 × 256-bit = 512-bit buffer

// ---------------------------------------------------------------------------
// Address calculation: item_idx × 64 bytes = item_idx << 6
// ---------------------------------------------------------------------------
wire [AXI_ADDR_WIDTH-1:0] item_addr = {{(AXI_ADDR_WIDTH-32){1'b0}}, req_item_idx} << 6;

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= ST_IDLE;
        req_ready      <= 1'b1;
        resp_valid     <= 1'b0;
        resp_data      <= 512'b0;
        m_axi_arid     <= {AXI_ID_WIDTH{1'b0}};
        m_axi_araddr   <= {AXI_ADDR_WIDTH{1'b0}};
        m_axi_arlen    <= 8'd1;           // 2 beats
        m_axi_arsize   <= 3'd5;           // 32 bytes per beat (256-bit)
        m_axi_arburst  <= 2'b01;          // INCR
        m_axi_arvalid  <= 1'b0;
        m_axi_rready   <= 1'b0;
        beat_cnt       <= 2'd0;
        item_buf[0]    <= 256'b0;
        item_buf[1]    <= 256'b0;
    end else begin
        resp_valid <= 1'b0;

        case (state)
            ST_IDLE: begin
                req_ready <= 1'b1;
                if (req_valid) begin
                    req_ready     <= 1'b0;
                    m_axi_araddr  <= item_addr;
                    m_axi_arlen   <= 8'd1;     // 2 beats (len = beats-1)
                    m_axi_arsize  <= 3'd5;     // 32 bytes
                    m_axi_arburst <= 2'b01;    // INCR
                    m_axi_arvalid <= 1'b1;
                    beat_cnt      <= 2'd0;
                    state         <= ST_AR;
                end
            end

            ST_AR: begin
                if (m_axi_arready && m_axi_arvalid) begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                    state         <= ST_RDATA;
                end
            end

            ST_RDATA: begin
                if (m_axi_rvalid) begin
                    item_buf[beat_cnt] <= m_axi_rdata;
                    beat_cnt           <= beat_cnt + 2'd1;
                    if (m_axi_rlast) begin
                        m_axi_rready <= 1'b0;
                        state        <= ST_RESP;
                    end
                end
            end

            ST_RESP: begin
                // Hold resp_valid high until consumer accepts (resp_ready=1)
                // resp_data is stable for the entire resp handshake window
                if (!resp_valid) begin
                    // First cycle in ST_RESP: present the data
                    resp_valid <= 1'b1;
                    resp_data  <= {item_buf[1], item_buf[0]};
                end else if (resp_ready) begin
                    // Handshake complete — de-assert valid and return to idle
                    resp_valid <= 1'b0;
                    state      <= ST_IDLE;
                    req_ready  <= 1'b1;
                end
                // If resp_valid=1 and resp_ready=0: hold valid (backpressure)
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
