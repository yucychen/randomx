// =============================================================================
// randomx_vm.v — RandomX Virtual Machine
// Part of RandomX FPGA framework targeting Xilinx XCVU33P
//
// The RandomX VM executes 256-instruction programs over 8 passes.
// Each program execution reads from the scratchpad and writes results back.
//
// Register file:
//   r[0..7]  — 8 × 64-bit integer registers
//   f[0..3]  — 4 × 128-bit FP pair registers (lo/hi)
//   e[0..3]  — 4 × 128-bit FP pair registers (always positive)
//   a[0..3]  — 4 × 128-bit FP pair registers (read-only constants from seed)
//
// Instruction pipeline:
//   IF → ID → EX → WB (4-stage, 1 instruction per cycle in skeleton)
//
// TODO: Implement full instruction decode for all 29 RandomX ISA instructions.
// TODO: Implement FP register forwarding.
// TODO: Implement CBRANCH logic with proper condition evaluation.
// TODO: Implement CFROUND (FP rounding mode update).
// TODO: Implement memory address generation (L1/L2/L3 scratchpad masking).
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns/1ps

module randomx_vm (
    input  wire          clk,
    input  wire          rst_n,

    // Start pulse: begin VM execution with loaded program and seed state
    input  wire          start,

    // Program memory write port (filled externally before start)
    input  wire          prog_wr_en,
    input  wire [7:0]    prog_wr_addr,  // 256-instruction program
    input  wire [63:0]   prog_wr_data,

    // Scratchpad interface (to scratchpad_mem)
    output reg           sp_rd_en,
    output reg  [20:0]   sp_rd_addr,
    output reg  [1:0]    sp_rd_level,
    input  wire [63:0]   sp_rd_data,
    input  wire          sp_rd_valid,
    output reg           sp_wr_en,
    output reg  [20:0]   sp_wr_addr,
    output reg  [1:0]    sp_wr_level,
    output reg  [63:0]   sp_wr_data,

    // Dataset interface (to hbm_dataset_if)
    output reg           ds_req_valid,
    output reg  [31:0]   ds_req_idx,
    input  wire          ds_req_ready,
    input  wire [511:0]  ds_resp_data,
    input  wire          ds_resp_valid,
    output reg           ds_resp_ready,

    // AES hash output for final hash step
    output reg           aes_start,
    output reg  [511:0]  aes_data_in,
    input  wire [511:0]  aes_hash_out,
    input  wire          aes_hash_valid,

    // Final 512-bit hash output
    output reg  [511:0]  hash_out,
    output reg           done
);

// ---------------------------------------------------------------------------
// Program buffer — 256 × 64-bit instruction words
// ---------------------------------------------------------------------------
reg [63:0] prog_mem [0:255];

always @(posedge clk) begin
    if (prog_wr_en)
        prog_mem[prog_wr_addr] <= prog_wr_data;
end

// ---------------------------------------------------------------------------
// Register file
// ---------------------------------------------------------------------------
reg [63:0]  r [0:7];           // integer registers r0..r7
reg [63:0]  f_lo [0:3];        // FP f registers low half
reg [63:0]  f_hi [0:3];        // FP f registers high half
reg [63:0]  e_lo [0:3];        // FP e registers low half
reg [63:0]  e_hi [0:3];        // FP e registers high half
reg [63:0]  a_lo [0:3];        // FP a registers low half (const)
reg [63:0]  a_hi [0:3];        // FP a registers high half (const)
reg [63:0]  ma;                 // Memory address register (scratchpad ptr)
reg [63:0]  mx;                 // Memory mix register
reg [1:0]   fprc;               // FP rounding mode control
reg [7:0]   ic;                 // Instruction counter
reg [2:0]   iter_cnt;           // Iteration count (0..7, 8 per program)

// ---------------------------------------------------------------------------
// Instruction decode (64-bit instruction word layout per RandomX spec §4.5)
//   [63:56] = opcode (8 bits)
//   [55:52] = dst (4 bits → r0..r7, f0..f3, etc.)
//   [51:48] = src (4 bits)
//   [47:32] = mod (16 bits: mem level, condition, etc.)
//   [31: 0] = imm32
// ---------------------------------------------------------------------------
wire [63:0] cur_instr  = prog_mem[ic];
wire [7:0]  op         = cur_instr[63:56];
wire [3:0]  dst_idx    = cur_instr[55:52];
wire [3:0]  src_idx    = cur_instr[51:48];
wire [15:0] mod        = cur_instr[47:32];
wire [31:0] imm32      = cur_instr[31: 0];
wire [63:0] imm64_sext = {{32{imm32[31]}}, imm32};

// Source register value (integer)
wire [63:0] r_src = r[src_idx[2:0]];
wire [63:0] r_dst = r[dst_idx[2:0]];

// Scratchpad address for memory instructions
// L1 mask = 0x3FFF, L2 = 0x3FFFF, L3 = 0x1FFFFF
wire [1:0]  mem_level = mod[1:0];
wire [63:0] mem_addr  = r_src + imm64_sext;

// ---------------------------------------------------------------------------
// ALU instance
// ---------------------------------------------------------------------------
reg  [5:0]  alu_op;
reg  [63:0] alu_a, alu_b, alu_imm;
reg  [1:0]  alu_shift;
reg         alu_en;
wire [63:0] alu_result;
wire        alu_valid;
wire        branch_taken;
wire        alu_mem_wr;
wire [63:0] alu_mem_addr, alu_mem_data;
wire [1:0]  alu_mem_level;

alu_int u_alu (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (alu_en),
    .opcode       (alu_op),
    .src_a        (alu_a),
    .src_b        (alu_b),
    .shift_amt    (alu_shift),
    .imm32_sext   (alu_imm),
    .result       (alu_result),
    .result_valid (alu_valid),
    .branch_taken (branch_taken),
    .mem_wr_en    (alu_mem_wr),
    .mem_wr_addr  (alu_mem_addr),
    .mem_wr_data  (alu_mem_data),
    .mem_wr_level (alu_mem_level)
);

// ---------------------------------------------------------------------------
// FPU instance
// ---------------------------------------------------------------------------
reg  [4:0]  fpu_op;
reg  [63:0] fpu_a, fpu_b;
reg         fpu_en;
wire [63:0] fpu_result;
wire        fpu_valid;

fpu_double u_fpu (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (fpu_en),
    .opcode       (fpu_op),
    .src_a        (fpu_a),
    .src_b        (fpu_b),
    .round_mode   (fprc),
    .result       (fpu_result),
    .result_valid (fpu_valid)
);

// ---------------------------------------------------------------------------
// Loop variable for reset initialization
// ---------------------------------------------------------------------------
integer i;

// ---------------------------------------------------------------------------
// FSM states
// ---------------------------------------------------------------------------
localparam ST_IDLE      = 4'd0;
localparam ST_FETCH     = 4'd1;
localparam ST_DECODE    = 4'd2;
localparam ST_MEM_RD    = 4'd3;
localparam ST_EXEC      = 4'd4;
localparam ST_WB        = 4'd5;
localparam ST_DS_FETCH  = 4'd6;  // dataset fetch for CFROUND/MX update
localparam ST_FINAL     = 4'd7;  // final hash
localparam ST_DONE      = 4'd8;

reg [3:0] state;
reg [3:0] wb_dst;
reg       wb_int_en, wb_fp_en;
reg       fp_hi_sel;  // 0=write lo half, 1=write hi half

// Opcode categories (simplified — full decode TODO)
localparam OPC_IADD_RS  = 8'd0;
localparam OPC_IADD_M   = 8'd1;
localparam OPC_ISUB_R   = 8'd2;
localparam OPC_IMUL_R   = 8'd4;
localparam OPC_IMULH_R  = 8'd6;
localparam OPC_ISMULH_R = 8'd8;
localparam OPC_IMUL_RCP = 8'd10;
localparam OPC_INEG_R   = 8'd11;
localparam OPC_IXOR_R   = 8'd12;
localparam OPC_IROR_R   = 8'd14;
localparam OPC_CBRANCH  = 8'd17;
localparam OPC_ISTORE   = 8'd18;
localparam OPC_FADD_R   = 8'd19;
localparam OPC_FSUB_R   = 8'd20;
localparam OPC_FSCAL_R  = 8'd21;
localparam OPC_FMUL_E   = 8'd22;
localparam OPC_FDIV_M   = 8'd23;
localparam OPC_FSQRT_R  = 8'd24;
localparam OPC_FSWAP_R  = 8'd25;
localparam OPC_CFROUND  = 8'd26;
localparam OPC_ISWAP_R  = 8'd27;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 8; i = i + 1) r[i]    <= 64'b0;
        for (i = 0; i < 4; i = i + 1) f_lo[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) f_hi[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) e_lo[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) e_hi[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) a_lo[i] <= 64'b0;
        for (i = 0; i < 4; i = i + 1) a_hi[i] <= 64'b0;
        ma           <= 64'b0;
        mx           <= 64'b0;
        fprc         <= 2'b00;
        ic           <= 8'd0;
        iter_cnt     <= 3'd0;
        state        <= ST_IDLE;
        alu_en       <= 1'b0;
        fpu_en       <= 1'b0;
        wb_int_en    <= 1'b0;
        wb_fp_en     <= 1'b0;
        wb_dst       <= 4'd0;
        sp_rd_en     <= 1'b0;
        sp_wr_en     <= 1'b0;
        ds_req_valid <= 1'b0;
        ds_resp_ready<= 1'b0;
        aes_start    <= 1'b0;
        hash_out     <= 512'b0;
        done         <= 1'b0;
        alu_op       <= 6'd0;
        alu_a        <= 64'b0;
        alu_b        <= 64'b0;
        alu_imm      <= 64'b0;
        alu_shift    <= 2'd0;
        fpu_op       <= 5'd0;
        fpu_a        <= 64'b0;
        fpu_b        <= 64'b0;
        fp_hi_sel    <= 1'b0;
    end else begin
        alu_en    <= 1'b0;
        fpu_en    <= 1'b0;
        done      <= 1'b0;
        aes_start <= 1'b0;
        sp_rd_en  <= 1'b0;
        sp_wr_en  <= 1'b0;

        // Scratchpad writeback from ALU
        if (alu_valid && alu_mem_wr) begin
            sp_wr_en    <= 1'b1;
            sp_wr_addr  <= alu_mem_addr[20:0];
            sp_wr_level <= alu_mem_level;
            sp_wr_data  <= alu_mem_data;
        end

        // Integer writeback
        if (alu_valid && wb_int_en) begin
            r[wb_dst[2:0]] <= alu_result;
            wb_int_en      <= 1'b0;
        end

        // FP writeback (lo half)
        if (fpu_valid && wb_fp_en) begin
            if (!fp_hi_sel)
                f_lo[wb_dst[1:0]] <= fpu_result;
            else
                f_hi[wb_dst[1:0]] <= fpu_result;
            wb_fp_en <= 1'b0;
        end

        case (state)
            ST_IDLE: begin
                if (start) begin
                    ic       <= 8'd0;
                    iter_cnt <= 3'd0;
                    state    <= ST_FETCH;
                end
            end

            ST_FETCH: begin
                // Instruction available combinationally from prog_mem
                state <= ST_DECODE;
            end

            ST_DECODE: begin
                // Dispatch based on opcode
                case (op)
                    // Integer ALU instructions (no memory)
                    OPC_IADD_RS,
                    OPC_ISUB_R,
                    OPC_IMUL_R,
                    OPC_IMULH_R,
                    OPC_ISMULH_R,
                    OPC_IMUL_RCP,
                    OPC_INEG_R,
                    OPC_IXOR_R,
                    OPC_IROR_R,
                    OPC_ISWAP_R: begin
                        alu_op    <= op[5:0];
                        alu_a     <= r_dst;
                        alu_b     <= r_src;
                        alu_shift <= mod[5:4];
                        alu_imm   <= imm64_sext;
                        alu_en    <= 1'b1;
                        wb_dst    <= dst_idx;
                        wb_int_en <= 1'b1;
                        state     <= ST_WB;
                    end

                    // Memory read instructions
                    OPC_IADD_M,
                    OPC_ISUB_R: begin // TODO: distinguish _M variant properly
                        sp_rd_en    <= 1'b1;
                        sp_rd_addr  <= mem_addr[20:0];
                        sp_rd_level <= mem_level;
                        state       <= ST_MEM_RD;
                    end

                    // CBRANCH
                    OPC_CBRANCH: begin
                        alu_op    <= 6'd17;
                        alu_a     <= r_dst;
                        alu_b     <= 64'b0;
                        alu_imm   <= imm64_sext;
                        alu_shift <= 2'd0;
                        alu_en    <= 1'b1;
                        wb_dst    <= dst_idx;
                        wb_int_en <= 1'b1;
                        state     <= ST_WB;
                    end

                    // ISTORE
                    OPC_ISTORE: begin
                        alu_op    <= 6'd18;
                        alu_a     <= r_dst;
                        alu_b     <= r_src;
                        alu_imm   <= imm64_sext;
                        alu_shift <= 2'd0;
                        alu_en    <= 1'b1;
                        state     <= ST_WB;
                    end

                    // FPU instructions
                    OPC_FADD_R,
                    OPC_FSUB_R: begin
                        fpu_op   <= op[4:0];
                        fpu_a    <= f_lo[dst_idx[1:0]];
                        fpu_b    <= a_lo[src_idx[1:0]];
                        fpu_en   <= 1'b1;
                        wb_dst   <= dst_idx;
                        wb_fp_en <= 1'b1;
                        fp_hi_sel<= 1'b0;
                        state    <= ST_WB;
                    end

                    OPC_FMUL_E: begin
                        fpu_op   <= 5'd4;
                        fpu_a    <= e_lo[dst_idx[1:0]];
                        fpu_b    <= a_lo[src_idx[1:0]];
                        fpu_en   <= 1'b1;
                        wb_dst   <= dst_idx;
                        wb_fp_en <= 1'b1;
                        fp_hi_sel<= 1'b0;
                        state    <= ST_WB;
                    end

                    OPC_FSCAL_R: begin
                        fpu_op   <= 5'd7;
                        fpu_a    <= f_lo[dst_idx[1:0]];
                        fpu_b    <= 64'b0;
                        fpu_en   <= 1'b1;
                        wb_dst   <= dst_idx;
                        wb_fp_en <= 1'b1;
                        fp_hi_sel<= 1'b0;
                        state    <= ST_WB;
                    end

                    OPC_FSWAP_R: begin
                        // Swap low/high halves of f or e register
                        // TODO: Properly determine whether to swap f or e
                        fpu_op   <= 5'd8;
                        fpu_a    <= f_lo[dst_idx[1:0]];
                        fpu_b    <= f_hi[dst_idx[1:0]];
                        fpu_en   <= 1'b1;
                        wb_dst   <= dst_idx;
                        wb_fp_en <= 1'b1;
                        fp_hi_sel<= 1'b0;
                        state    <= ST_WB;
                    end

                    OPC_CFROUND: begin
                        // Update FP rounding mode from r_src
                        fprc  <= r_src[1:0]; // TODO: apply ror(r_src,32)[1:0]
                        state <= ST_WB;
                    end

                    default: begin
                        // Unknown opcode — NOP
                        state <= ST_WB;
                    end
                endcase
            end

            ST_MEM_RD: begin
                // Wait for scratchpad read
                if (sp_rd_valid) begin
                    // Execute with loaded memory operand
                    alu_op    <= op[5:0];
                    alu_a     <= r_dst;
                    alu_b     <= sp_rd_data;
                    alu_imm   <= imm64_sext;
                    alu_shift <= 2'd0;
                    alu_en    <= 1'b1;
                    wb_dst    <= dst_idx;
                    wb_int_en <= 1'b1;
                    state     <= ST_WB;
                end
            end

            ST_EXEC: begin
                // Generic execute wait state (1 cycle for registered ALU)
                state <= ST_WB;
            end

            ST_WB: begin
                // Advance PC; check end of program
                if (ic == 8'd255) begin
                    // End of one program pass
                    ic <= 8'd0;
                    if (iter_cnt == 3'd7) begin
                        // All 8 iterations done — run final hash
                        state <= ST_FINAL;
                    end else begin
                        iter_cnt <= iter_cnt + 3'd1;
                        // TODO: Fetch new dataset item for mix
                        state <= ST_FETCH;
                    end
                end else begin
                    ic    <= ic + 8'd1;
                    state <= ST_FETCH;
                end
            end

            ST_DS_FETCH: begin
                // TODO: Implement dataset item fetch and MX/MA update
                if (ds_resp_valid) begin
                    ds_resp_ready <= 1'b0;
                    state         <= ST_FETCH;
                end else begin
                    ds_resp_ready <= 1'b1;
                end
            end

            ST_FINAL: begin
                // Run AesHash1R on scratchpad to produce final hash
                // TODO: XOR all scratchpad segments, then run AES hash
                aes_start  <= 1'b1;
                aes_data_in <= {r[7],r[6],r[5],r[4],r[3],r[2],r[1],r[0]};
                state      <= ST_DONE;
            end

            ST_DONE: begin
                if (aes_hash_valid) begin
                    hash_out <= aes_hash_out;
                    done     <= 1'b1;
                    state    <= ST_IDLE;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
