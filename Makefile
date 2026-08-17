# RandomX FPGA — build / simulation / lint automation
#
#   make            # 编译并运行全部 testbench（自校验，遇到 FAIL 退出码非 0）
#   make lint       # Verilator lint（--lint-only）
#   make syntax     # 逐模块 iverilog 语法检查
#   make tb_blake2b # 只跑某一个 testbench
#   make clean

IVERILOG ?= iverilog
VVP      ?= vvp
VERILATOR?= verilator

IVFLAGS  := -g2001 -Wall
SIMFLAGS := -DSIMULATION

RTL_DIR  := rtl
SIM_DIR  := sim
BUILD    := $(SIM_DIR)/build

# 完整 RTL 文件列表（顺序无关，iverilog 自行解析）
RTL_SRCS := \
	$(RTL_DIR)/aes_round.v \
	$(RTL_DIR)/aes_gen1r.v \
	$(RTL_DIR)/aes_gen4r.v \
	$(RTL_DIR)/aes_hash1r.v \
	$(RTL_DIR)/blake2b_core.v \
	$(RTL_DIR)/scratchpad_mem.v \
	$(RTL_DIR)/hbm_dataset_if.v \
	$(RTL_DIR)/cache_hbm_if.v \
	$(RTL_DIR)/axi_arbiter.v \
	$(RTL_DIR)/alu_int.v \
	$(RTL_DIR)/fpu_double.v \
	$(RTL_DIR)/superscalar_hash.v \
	$(RTL_DIR)/argon2_fill.v \
	$(RTL_DIR)/randomx_vm.v \
	$(RTL_DIR)/randomx_top.v

# 板级顶层（HBM IP 封装层，不属于 randomx_top 的 lint 范围）
BOARD_SRCS := $(RTL_DIR)/randomx_hbm_top.v

# 每个 testbench 需要的 RTL 子集
SRCS_tb_blake2b         := $(RTL_DIR)/blake2b_core.v
SRCS_tb_hbm_dataset_if  := $(RTL_DIR)/hbm_dataset_if.v
SRCS_tb_cache_hbm_if    := $(RTL_DIR)/cache_hbm_if.v
SRCS_tb_superscalar_hash:= $(RTL_DIR)/alu_int.v $(RTL_DIR)/superscalar_hash.v
SRCS_tb_fpu_double      := $(RTL_DIR)/fpu_double.v
SRCS_tb_argon2_fill     := $(RTL_DIR)/blake2b_core.v $(RTL_DIR)/argon2_fill.v
SRCS_tb_randomx_top     := $(RTL_SRCS)
SRCS_tb_randomx_hbm_top := $(RTL_SRCS) $(BOARD_SRCS)

TESTS := tb_blake2b tb_hbm_dataset_if tb_cache_hbm_if tb_superscalar_hash \
         tb_fpu_double tb_argon2_fill tb_randomx_top tb_randomx_hbm_top

.PHONY: all test lint syntax clean $(TESTS)

all: test

## 运行全部 testbench；任一输出出现 FAIL/ERROR 即整体失败
test: $(TESTS)
	@echo "=== ALL TESTBENCHES PASSED ==="

define TB_rule
$(1): $$(BUILD)/$(1).vvp
	@echo "=== Running $(1) ==="
	@$$(VVP) $$< | tee $$(BUILD)/$(1).log
	@if grep -qE '(FAIL|ERROR|\$$$$fatal)' $$(BUILD)/$(1).log; then \
		echo "=== $(1): FAILED ==="; exit 1; \
	else echo "=== $(1): OK ==="; fi

$$(BUILD)/$(1).vvp: $$(SRCS_$(1)) $$(SIM_DIR)/$(1).v | $$(BUILD)
	$$(IVERILOG) $$(IVFLAGS) $$(SIMFLAGS) -o $$@ $$(SRCS_$(1)) $$(SIM_DIR)/$(1).v
endef

$(foreach t,$(TESTS),$(eval $(call TB_rule,$(t))))

$(BUILD):
	@mkdir -p $(BUILD)

## Verilator lint（不生成模型，仅静态检查）
# 已豁免的告警（均为当前骨架实现的已知状态，见 README 的 TODO 表）：
#   DECLFILENAME   — 单文件内含多个子模块（如 blake2b_g）
#   UNUSEDSIGNAL   — 骨架模块中尚未使用的信号
#   UNUSEDPARAM    — 骨架模块中为将来实现预留的参数
#   PINCONNECTEMPTY— 顶层有意悬空的可选输出端口
#   UNDRIVEN       — randomx_vm.ds_req_idx，等待 Dataset 取数逻辑实现
LINT_WAIVERS := -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
                -Wno-PINCONNECTEMPTY -Wno-UNDRIVEN

lint:
	$(VERILATOR) --lint-only -Wall $(LINT_WAIVERS) \
		+define+SIMULATION --top-module randomx_top $(RTL_SRCS)
	$(VERILATOR) --lint-only -Wall $(LINT_WAIVERS) \
		+define+SIMULATION --top-module randomx_hbm_top $(RTL_SRCS) $(BOARD_SRCS)

## 逐模块语法检查（-y rtl 自动查找子模块）
syntax:
	@rc=0; for f in $(RTL_DIR)/*.v; do \
		echo "Checking $$f..."; \
		$(IVERILOG) $(IVFLAGS) $(SIMFLAGS) -y $(RTL_DIR) -o /dev/null $$f || rc=1; \
	done; exit $$rc

clean:
	rm -rf $(BUILD) obj_dir *.vcd *.vvp
