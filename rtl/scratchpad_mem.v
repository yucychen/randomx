// =============================================================================
// scratchpad_mem.v — 2 MiB Scratchpad Memory
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// RandomX uses a 2 MiB scratchpad (Ls = 2097152 bytes) partitioned into:
//   L1: 16 KiB  (mask = 0x3FFF, for ISTORE with L=0)
//   L2: 256 KiB (mask = 0x3FFFF, for ISTORE with L=1)
//   L3: 2 MiB   (mask = 0x1FFFFF, for ISTORE with L=2)
//
// Implemented as 64-bit wide × 262144 deep URAM.
// The (* ram_style = "ultra" *) attribute directs Vivado to use URAM
// on the XCVU33P (which has 320 URAM blocks of 72Kbit each).
// 2 MiB / 8 bytes = 262144 words → 262144 × 64-bit → ~16 Mbit → ~222 URAM needed.
//
// Ports: single read-write port (simple dual-port style via enable signals).
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module scratchpad_mem (
    input  wire         clk,
    input  wire         rst_n,   // unused for URAM but kept for interface consistency

    // Write port
    input  wire         wr_en,
    input  wire [20:0]  wr_addr, // byte address [20:0] → word addr = wr_addr[20:3]
    input  wire [63:0]  wr_data,
    input  wire [1:0]   wr_level,// 0=L1,1=L2,2=L3 → controls address masking

    // Read port
    input  wire         rd_en,
    input  wire [20:0]  rd_addr, // byte address
    input  wire [1:0]   rd_level,// address masking level
    output reg  [63:0]  rd_data,
    output reg          rd_valid  // one-cycle delay
);

// ---------------------------------------------------------------------------
// Address mask per L level
// ---------------------------------------------------------------------------
function [17:0] addr_mask;
    input [1:0] level;
    case (level)
        2'd0: addr_mask = 18'h007FF; // L1: 16 KiB / 8 = 2K words → 11 bits
        2'd1: addr_mask = 18'h07FFF; // L2: 256 KiB / 8 = 32K words → 15 bits
        2'd2: addr_mask = 18'h3FFFF; // L3: 2 MiB / 8 = 256K words → 18 bits
        default: addr_mask = 18'h3FFFF;
    endcase
endfunction

// ---------------------------------------------------------------------------
// URAM inference: 262144 × 64-bit (2 MiB)
// In simulation, use a smaller array to keep elaboration fast.
// `iverilog -DSIMULATION` reduces depth to 4096 × 64-bit (32 KiB).
// The (* ram_style = "ultra" *) attribute is ignored by iverilog.
// ---------------------------------------------------------------------------
`ifdef SIMULATION
localparam SP_DEPTH = 12'hFFF; // 4095 words (12-bit index)
`else
localparam SP_DEPTH = 18'h3FFFF; // 262143 words (18-bit index)
`endif

(* ram_style = "ultra" *)
`ifdef SIMULATION
reg [63:0] scratchpad [0:4095];
`else
reg [63:0] scratchpad [0:262143];
`endif

// Masked word addresses — clip to array size
wire [17:0] wr_waddr_full = wr_addr[20:3] & addr_mask(wr_level);
wire [17:0] rd_waddr_full = rd_addr[20:3] & addr_mask(rd_level);
`ifdef SIMULATION
wire [11:0] wr_waddr = wr_waddr_full[11:0];
wire [11:0] rd_waddr = rd_waddr_full[11:0];
`else
wire [17:0] wr_waddr = wr_waddr_full;
wire [17:0] rd_waddr = rd_waddr_full;
`endif

// Write
always @(posedge clk) begin
    if (wr_en) begin
        scratchpad[wr_waddr] <= wr_data;
    end
end

// Read (synchronous read, 1-cycle latency)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_data  <= 64'b0;
        rd_valid <= 1'b0;
    end else begin
        rd_valid <= rd_en;
        if (rd_en) begin
            rd_data <= scratchpad[rd_waddr];
        end
    end
end

endmodule
