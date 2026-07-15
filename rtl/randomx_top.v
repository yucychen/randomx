// =============================================================================
// randomx_top.v — RandomX Top-level Module
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// Target: Xilinx Virtex UltraScale+ XCVU33P (part xcvu33p-fsvh2104-2L-e)
// Tool:   Vivado 2022.x or later
//
// Interface: AXI-Lite style control/status registers (simplified, not full AXI)
//   Write 0x00 ← seed[511:0] (8 × 32-bit writes)
//   Write 0x20 ← control[0] = start
//   Read  0x24 → status[0] = done
//   Read  0x28..0x47 → hash_out[511:0] (8 × 32-bit reads)
//
// Main FSM:
//   IDLE → CACHE_INIT (Argon2d) → DS_GEN (SuperscalarHash) →
//   VM_RUN (8 iterations) → FINAL_HASH (AesHash1R + Blake2b) → DONE
//
// All sub-modules use synchronous active-low reset (rst_n), single clock (clk).
//
// Verilog-2001 compliant, no vendor IP instantiations.
// =============================================================================

`timescale 1ns/1ps

module randomx_top (
    input  wire         clk,      // System clock (300 MHz target)
    input  wire         rst_n,    // Active-low synchronous reset

    // --- AXI-Lite style control/status register interface ---
    // (simplified: no handshake signals, single-cycle access)
    input  wire         reg_wr_en,
    input  wire [7:0]   reg_wr_addr, // byte address (word-aligned, 4-byte words)
    input  wire [31:0]  reg_wr_data,

    input  wire         reg_rd_en,
    input  wire [7:0]   reg_rd_addr,
    output reg  [31:0]  reg_rd_data,

    // --- AXI4 HBM Master interface (passed through to hbm_dataset_if) ---
    output wire [33:0]  m_axi_araddr,
    output wire [7:0]   m_axi_arlen,
    output wire [2:0]   m_axi_arsize,
    output wire [1:0]   m_axi_arburst,
    output wire         m_axi_arvalid,
    input  wire         m_axi_arready,
    input  wire [255:0] m_axi_rdata,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rlast,
    input  wire         m_axi_rvalid,
    output wire         m_axi_rready,
    // Write channels (dataset generation — driver TODO: superscalar_hash)
    output wire [33:0]  m_axi_awaddr,
    output wire [7:0]   m_axi_awlen,
    output wire [2:0]   m_axi_awsize,
    output wire [1:0]   m_axi_awburst,
    output wire         m_axi_awvalid,
    input  wire         m_axi_awready,
    output wire [255:0] m_axi_wdata,
    output wire [31:0]  m_axi_wstrb,
    output wire         m_axi_wlast,
    output wire         m_axi_wvalid,
    input  wire         m_axi_wready,
    input  wire [1:0]   m_axi_bresp,
    input  wire         m_axi_bvalid,
    output wire         m_axi_bready
);

// ===========================================================================
// Internal registers / signals
// ===========================================================================

// Seed register (512 bits = 64 bytes, written via 16 × 32-bit register writes)
reg [511:0] seed_reg;

// Control / status
reg         start_pulse;
reg         busy;
wire        all_done;

// Hash output
reg [511:0] hash_out;

// Top-level FSM states
localparam FSM_IDLE       = 4'd0;
localparam FSM_CACHE_INIT = 4'd1;   // Argon2d cache fill
localparam FSM_DS_GEN     = 4'd2;   // SuperscalarHash dataset generation
localparam FSM_VM_RUN     = 4'd3;   // RandomX VM execution (8 iterations)
localparam FSM_FINAL_HASH = 4'd4;   // AesHash1R + Blake2b finalization
localparam FSM_DONE       = 4'd5;

reg [3:0] fsm_state;

// ===========================================================================
// Sub-module wires
// ===========================================================================

// --- Argon2d ---
wire        argon2_done;
reg         argon2_start;
// Cache write interface (stub — no physical cache here, forwarded to HBM)
wire        argon2_cache_wr_en;
wire [31:0] argon2_cache_wr_addr;
wire [1023:0] argon2_cache_wr_data;

// Blake2b (shared between argon2 and final hash)
wire        b2b_done;
wire        b2b_start;
wire [1023:0] b2b_msg;
wire [127:0]  b2b_byte_cnt;
wire [511:0]  b2b_h_in;
wire          b2b_last;
wire [511:0]  b2b_h_out;

// --- Scratchpad memory ---
wire        sp_rd_en,    sp_wr_en;
wire [20:0] sp_rd_addr,  sp_wr_addr;
wire [1:0]  sp_rd_level, sp_wr_level;
wire [63:0] sp_rd_data,  sp_wr_data;
wire        sp_rd_valid;

// --- HBM Dataset IF ---
wire        ds_req_valid, ds_req_ready;
wire [31:0] ds_req_idx;
wire [511:0] ds_resp_data;
wire        ds_resp_valid, ds_resp_ready;

// --- AES Hash ---
wire        aes_start;
wire [511:0] aes_data_in;
wire [511:0] aes_hash_out;
wire        aes_hash_valid;

// --- VM ---
wire        vm_done;
reg         vm_start;
wire        vm_prog_wr_en;
wire [7:0]  vm_prog_wr_addr;
wire [63:0] vm_prog_wr_data;
wire [511:0] vm_hash_out;
wire        vm_aes_start;
wire [511:0] vm_aes_data_in;

// ===========================================================================
// Sub-module instantiations
// ===========================================================================

// --- Blake2b core (shared) ---
blake2b_core u_blake2b (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (b2b_start),
    .last_block (b2b_last),
    .msg_block  (b2b_msg),
    .byte_count (b2b_byte_cnt),
    .h_in       (b2b_h_in),
    .h_out      (b2b_h_out),
    .done       (b2b_done)
);

// --- Argon2d cache fill ---
argon2_fill u_argon2 (
    .clk            (clk),
    .rst_n          (rst_n),
    .start          (argon2_start),
    .key            (seed_reg),
    .key_len        (6'd63),    // 64 bytes = all bits (6-bit max 63, TODO: fix key_len encoding)
    .cache_wr_en    (argon2_cache_wr_en),
    .cache_wr_addr  (argon2_cache_wr_addr),
    .cache_wr_data  (argon2_cache_wr_data),
    .cache_wr_rdy   (1'b1),    // TODO: Connect to actual cache (HBM)
    .done           (argon2_done),
    .b2b_start      (b2b_start),
    .b2b_msg        (b2b_msg),
    .b2b_byte_cnt   (b2b_byte_cnt),
    .b2b_h_in       (b2b_h_in),
    .b2b_last       (b2b_last),
    .b2b_h_out      (b2b_h_out),
    .b2b_done       (b2b_done)
);

// --- Scratchpad (2 MiB, URAM) ---
scratchpad_mem u_scratchpad (
    .clk      (clk),
    .rst_n    (rst_n),
    .wr_en    (sp_wr_en),
    .wr_addr  (sp_wr_addr),
    .wr_data  (sp_wr_data),
    .wr_level (sp_wr_level),
    .rd_en    (sp_rd_en),
    .rd_addr  (sp_rd_addr),
    .rd_level (sp_rd_level),
    .rd_data  (sp_rd_data),
    .rd_valid (sp_rd_valid)
);

// --- HBM AXI4 master interface ---
hbm_dataset_if #(
    .AXI_ADDR_WIDTH (34),
    .AXI_DATA_WIDTH (256),
    .AXI_ID_WIDTH   (6)
) u_hbm (
    .clk            (clk),
    .rst_n          (rst_n),
    .req_valid      (ds_req_valid),
    .req_item_idx   (ds_req_idx),
    .req_ready      (ds_req_ready),
    .resp_valid     (ds_resp_valid),
    .resp_data      (ds_resp_data),
    .resp_ready     (ds_resp_ready),
    .wr_req_valid   (1'b0),            // TODO: drive from superscalar_hash
    .wr_req_item_idx(32'b0),
    .wr_req_data    (512'b0),
    .wr_req_ready   (),
    .wr_done        (),
    .m_axi_arid     (),                // ID not connected to top
    .m_axi_araddr   (m_axi_araddr),
    .m_axi_arlen    (m_axi_arlen),
    .m_axi_arsize   (m_axi_arsize),
    .m_axi_arburst  (m_axi_arburst),
    .m_axi_arvalid  (m_axi_arvalid),
    .m_axi_arready  (m_axi_arready),
    .m_axi_rid      (6'b0),
    .m_axi_rdata    (m_axi_rdata),
    .m_axi_rresp    (m_axi_rresp),
    .m_axi_rlast    (m_axi_rlast),
    .m_axi_rvalid   (m_axi_rvalid),
    .m_axi_rready   (m_axi_rready),
    .m_axi_awid     (),
    .m_axi_awaddr   (m_axi_awaddr),
    .m_axi_awlen    (m_axi_awlen),
    .m_axi_awsize   (m_axi_awsize),
    .m_axi_awburst  (m_axi_awburst),
    .m_axi_awvalid  (m_axi_awvalid),
    .m_axi_awready  (m_axi_awready),
    .m_axi_wdata    (m_axi_wdata),
    .m_axi_wstrb    (m_axi_wstrb),
    .m_axi_wlast    (m_axi_wlast),
    .m_axi_wvalid   (m_axi_wvalid),
    .m_axi_wready   (m_axi_wready),
    .m_axi_bid      (6'b0),
    .m_axi_bresp    (m_axi_bresp),
    .m_axi_bvalid   (m_axi_bvalid),
    .m_axi_bready   (m_axi_bready)
);

// --- AesHash1R ---
aes_hash1r u_aes_hash (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (vm_aes_start),
    .data_in  (vm_aes_data_in),
    .hash_out (aes_hash_out),
    .valid    (aes_hash_valid)
);

// --- RandomX VM ---
randomx_vm u_vm (
    .clk           (clk),
    .rst_n         (rst_n),
    .start         (vm_start),
    .prog_wr_en    (1'b0),     // TODO: Program loaded via AXI register writes
    .prog_wr_addr  (8'b0),
    .prog_wr_data  (64'b0),
    .sp_rd_en      (sp_rd_en),
    .sp_rd_addr    (sp_rd_addr),
    .sp_rd_level   (sp_rd_level),
    .sp_rd_data    (sp_rd_data),
    .sp_rd_valid   (sp_rd_valid),
    .sp_wr_en      (sp_wr_en),
    .sp_wr_addr    (sp_wr_addr),
    .sp_wr_level   (sp_wr_level),
    .sp_wr_data    (sp_wr_data),
    .ds_req_valid  (ds_req_valid),
    .ds_req_idx    (ds_req_idx),
    .ds_req_ready  (ds_req_ready),
    .ds_resp_data  (ds_resp_data),
    .ds_resp_valid (ds_resp_valid),
    .ds_resp_ready (ds_resp_ready),
    .aes_start     (vm_aes_start),
    .aes_data_in   (vm_aes_data_in),
    .aes_hash_out  (aes_hash_out),
    .aes_hash_valid(aes_hash_valid),
    .hash_out      (vm_hash_out),
    .done          (vm_done)
);

// ===========================================================================
// Register interface
// ===========================================================================

// Write
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        seed_reg    <= 512'b0;
        start_pulse <= 1'b0;
    end else begin
        start_pulse <= 1'b0;
        if (reg_wr_en) begin
            casez (reg_wr_addr)
                // Seed: 0x00..0x1C (8 words × 4 bytes)
                8'h00: seed_reg[ 31:  0] <= reg_wr_data;
                8'h04: seed_reg[ 63: 32] <= reg_wr_data;
                8'h08: seed_reg[ 95: 64] <= reg_wr_data;
                8'h0C: seed_reg[127: 96] <= reg_wr_data;
                8'h10: seed_reg[159:128] <= reg_wr_data;
                8'h14: seed_reg[191:160] <= reg_wr_data;
                8'h18: seed_reg[223:192] <= reg_wr_data;
                8'h1C: seed_reg[255:224] <= reg_wr_data;
                8'h20: seed_reg[287:256] <= reg_wr_data;
                8'h24: seed_reg[319:288] <= reg_wr_data;
                8'h28: seed_reg[351:320] <= reg_wr_data;
                8'h2C: seed_reg[383:352] <= reg_wr_data;
                8'h30: seed_reg[415:384] <= reg_wr_data;
                8'h34: seed_reg[447:416] <= reg_wr_data;
                8'h38: seed_reg[479:448] <= reg_wr_data;
                8'h3C: seed_reg[511:480] <= reg_wr_data;
                // Control register: 0x40
                8'h40: start_pulse <= reg_wr_data[0];
                default: ; // ignore unknown addresses
            endcase
        end
    end
end

// Read
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        reg_rd_data <= 32'b0;
    end else if (reg_rd_en) begin
        casez (reg_rd_addr)
            // Status: 0x44
            8'h44: reg_rd_data <= {31'b0, ~busy};
            // Hash output: 0x48..0x64 (8 × 32-bit)
            8'h48: reg_rd_data <= hash_out[ 31:  0];
            8'h4C: reg_rd_data <= hash_out[ 63: 32];
            8'h50: reg_rd_data <= hash_out[ 95: 64];
            8'h54: reg_rd_data <= hash_out[127: 96];
            8'h58: reg_rd_data <= hash_out[159:128];
            8'h5C: reg_rd_data <= hash_out[191:160];
            8'h60: reg_rd_data <= hash_out[223:192];
            8'h64: reg_rd_data <= hash_out[255:224];
            8'h68: reg_rd_data <= hash_out[287:256];
            8'h6C: reg_rd_data <= hash_out[319:288];
            8'h70: reg_rd_data <= hash_out[351:320];
            8'h74: reg_rd_data <= hash_out[383:352];
            8'h78: reg_rd_data <= hash_out[415:384];
            8'h7C: reg_rd_data <= hash_out[447:416];
            8'h80: reg_rd_data <= hash_out[479:448];
            8'h84: reg_rd_data <= hash_out[511:480];
            default: reg_rd_data <= 32'hDEADBEEF;
        endcase
    end
end

// ===========================================================================
// Top-level FSM
// ===========================================================================
assign all_done = vm_done;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm_state   <= FSM_IDLE;
        busy        <= 1'b0;
        argon2_start<= 1'b0;
        vm_start    <= 1'b0;
        hash_out    <= 512'b0;
    end else begin
        argon2_start <= 1'b0;
        vm_start     <= 1'b0;

        case (fsm_state)
            FSM_IDLE: begin
                if (start_pulse) begin
                    busy         <= 1'b1;
                    argon2_start <= 1'b1;
                    fsm_state    <= FSM_CACHE_INIT;
                end
            end

            FSM_CACHE_INIT: begin
                if (argon2_done) begin
                    // TODO: Trigger dataset generation (SuperscalarHash passes)
                    // For now: skip directly to VM run
                    vm_start  <= 1'b1;
                    fsm_state <= FSM_VM_RUN;
                end
            end

            FSM_DS_GEN: begin
                // TODO: Wait for dataset generation complete
                vm_start  <= 1'b1;
                fsm_state <= FSM_VM_RUN;
            end

            FSM_VM_RUN: begin
                if (vm_done) begin
                    fsm_state <= FSM_FINAL_HASH;
                end
            end

            FSM_FINAL_HASH: begin
                // Hash already computed by VM's final AES step
                // Run Blake2b to finalize
                // TODO: Implement Blake2b finalization here
                hash_out  <= vm_hash_out;
                fsm_state <= FSM_DONE;
            end

            FSM_DONE: begin
                busy      <= 1'b0;
                fsm_state <= FSM_IDLE;
            end

            default: fsm_state <= FSM_IDLE;
        endcase
    end
end

endmodule
