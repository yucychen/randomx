# RandomX FPGA 纯 Verilog 框架

面向 Xilinx Virtex UltraScale+ **XCVU33P**（型号：`xcvu33p-fsvh2104-2L-e`）的 RandomX 工作量证明算法纯 Verilog-2001 硬件实现框架。

> **状态：骨架/框架**  
> 所有模块均可使用 `iverilog -g2001` 编译并通过仿真，功能逻辑已标注 `TODO` 注释，待完整实现。

---

## 目录结构

```
randomx/
├── rtl/                    # RTL 源码（纯 Verilog-2001）
│   ├── randomx_top.v       # 顶层模块：时钟/复位、寄存器接口、主 FSM
│   ├── blake2b_core.v      # Blake2b-512 哈希核（G函数数据通路骨架）
│   ├── aes_round.v         # AES 单轮函数（SubBytes/ShiftRows/MixColumns/ARK）
│   ├── aes_gen1r.v         # AesGenerator1R（1轮AES × 4 lane）
│   ├── aes_gen4r.v         # AesGenerator4R（4轮AES × 4 lane）
│   ├── aes_hash1r.v        # AesHash1R（4轮AES哈希 × 4 lane）
│   ├── superscalar_hash.v  # SuperscalarHash（数据集生成程序执行骨架）
│   ├── randomx_vm.v        # RandomX 虚拟机（取指/译码/执行/回写）
│   ├── alu_int.v           # 整数执行单元（19条整数指令）
│   ├── fpu_double.v        # 双精度浮点单元骨架（FSCAL/FSWAP 已实现）
│   ├── scratchpad_mem.v    # 2 MiB Scratchpad（URAM 推断，L1/L2/L3 掩码）
│   ├── hbm_dataset_if.v    # HBM2 AXI4 主设备接口骨架（数据集访问）
│   └── argon2_fill.v       # Argon2d Cache 填充骨架（基于 Blake2b）
├── sim/
│   └── tb_randomx_top.v    # 基础功能仿真 testbench
├── vivado/
│   ├── build.tcl           # Vivado TCL 构建脚本（非项目模式）
│   └── constraints.xdc     # 时序约束（300 MHz 时钟 + HBM 占位符）
└── README.md               # 本文档
```

---

## 架构框图（ASCII）

```
                        ┌─────────────────────────────────────────────────┐
                        │              randomx_top.v (顶层)                │
                        │                                                   │
  clk ─────────────────►│  ┌──────────┐   ┌──────────────┐                │
  rst_n ────────────────►│  │  主 FSM  │──►│  argon2_fill │◄──►blake2b    │
                        │  │          │   │  (Cache初始化) │    _core      │
  AXI-Lite ────────────►│  │CACHE_INIT│   └──────────────┘               │
  控制寄存器接口         │  │DS_GEN    │                                    │
  (start/done/seed/hash)│  │VM_RUN    │   ┌──────────────┐                │
                        │  │FINAL_HASH│──►│ randomx_vm   │                │
                        │  └──────────┘   │  ┌─────────┐ │                │
                        │                 │  │ alu_int │ │                │
                        │                 │  ├─────────┤ │                │
                        │                 │  │fpu_doubl│ │                │
                        │                 │  └─────────┘ │                │
                        │                 └──┬───────────┘                │
                        │                    │  ▲                          │
                        │  ┌─────────────────▼──┴──────────────────────┐  │
                        │  │          scratchpad_mem (URAM 2MiB)        │  │
                        │  │          L1(16K) / L2(256K) / L3(2M)      │  │
                        │  └────────────────────────────────────────────┘  │
                        │                                                   │
                        │  ┌────────────────────────────────────────────┐  │
                        │  │   hbm_dataset_if (AXI4 主设备)              │  │──► HBM2 AXI
                        │  │   Dataset (~2GB) 存于 XCVU33P 8GB HBM2     │  │    接口
                        │  └────────────────────────────────────────────┘  │
                        │                                                   │
                        │  AES流水线：aes_round ─► aes_gen1r/4r/hash1r    │
                        └─────────────────────────────────────────────────┘
```

---

## RandomX 算法流程到硬件映射

### RandomX 标准流程

```
种子 (Key/Seed)
    │
    ▼
[1] Argon2d Cache 填充 (argon2_fill.v)
    256 MB Cache = 262144 × 1KB 块
    使用 Blake2b 压缩函数填充
    │
    ▼
[2] SuperscalarHash 数据集生成 (superscalar_hash.v)
    Dataset ≈ 2.08 GB, 存于 HBM2
    每个 Dataset 条目 = 8 轮 SuperscalarHash
    │
    ▼
[3] RandomX VM 执行 (randomx_vm.v)
    8 次迭代，每次：
      a) 获取 Dataset 条目 (hbm_dataset_if.v)
      b) 执行 256 条指令程序
         - 整数指令 → alu_int.v
         - 浮点指令 → fpu_double.v
         - 内存访问 → scratchpad_mem.v
      c) AES 混合 Scratchpad (aes_gen4r.v)
    │
    ▼
[4] 最终哈希 (aes_hash1r.v + blake2b_core.v)
    AesHash1R 压缩 Scratchpad → 512 bit
    Blake2b 最终化 → 256 bit 输出哈希
```

---

## 模块说明

### `randomx_top.v` — 顶层模块
- **接口**：AXI-Lite 风格控制/状态寄存器（简化版，无握手）
- **寄存器映射**：
  | 地址    | 方向 | 描述                        |
  |---------|------|-----------------------------|
  | 0x00~0x3C | 写  | 种子输入（512位，16×32位）   |
  | 0x40    | 写   | 控制寄存器（bit0=start）      |
  | 0x44    | 读   | 状态寄存器（bit0=done/~busy） |
  | 0x48~0x84 | 读 | 哈希输出（512位，16×32位）   |
- **主 FSM**：IDLE → CACHE_INIT → VM_RUN → FINAL_HASH → DONE
- **TODO**：DS_GEN 阶段、完整 AXI-Lite 握手

### `blake2b_core.v` — Blake2b-512 哈希核
- G 函数数据通路（rotr32/rotr24/rotr16/rotr63 已实现）
- 12轮×8步计数 FSM 骨架（约96周期/压缩）
- **TODO**：完整 sigma 置换表（已实现第0、1轮）、完整 G 函数调度

### `aes_round.v` — AES 单轮函数
- SubBytes：256 项 LUT S-box（纯组合逻辑）
- ShiftRows：行移位（组合逻辑）
- MixColumns：GF(2⁸) MDS 矩阵乘法（组合逻辑）
- AddRoundKey：与轮密钥异或
- `last_round` 控制是否跳过 MixColumns（最终轮）

### `aes_gen1r.v` / `aes_gen4r.v` / `aes_hash1r.v`
- 基于 `aes_round.v` 构建的 AES 生成器和哈希器
- 4 × 128-bit lane 并行处理（64字节状态）
- **TODO**：从种子派生正确的轮密钥（当前使用占位符常量）

### `scratchpad_mem.v` — Scratchpad 内存
- 2 MiB（262144 × 64-bit），使用 `(* ram_style = "ultra" *)` 推断 URAM
- L1（16 KiB）/ L2（256 KiB）/ L3（2 MiB）地址掩码
- XCVU33P 配置：需约 222 块 URAM（总共 320 块）
- 仿真模式（`-DSIMULATION`）：缩减为 4096 × 64-bit

### `hbm_dataset_if.v` — HBM2 数据集 AXI4 主设备
- AXI4 读通道主设备骨架（AR + R 通道）
- 256-bit 总线宽度（HBM 伪通道带宽）
- 64字节对齐的 Dataset 条目请求/响应 FIFO 接口
- **TODO**：连接到 Vivado HBM IP；写通道（数据集生成时）

### `alu_int.v` — 整数执行单元
- 完整 RandomX 整数 ISA：IADD_RS, ISUB, IMUL, IMULH, ISMULH, INEG, IXOR, IROR/IROL, ISWAP, CBRANCH, ISTORE
- 有符号/无符号 128-bit 乘法（高64位提取）
- **TODO**：IMUL_RCP（模乘倒数）、CBRANCH 条件掩码

### `fpu_double.v` — 双精度浮点单元
- FSCAL_R：符号位异或 + 指数异或（**已实现**）
- FSWAP_R：寄存器高/低半部交换（**已实现**）
- FADD/FSUB/FMUL：**TODO** — 需要 IEEE 754 加法器/乘法器
- FDIV/FSQRT：**TODO** — 需要迭代除法/开方单元

### `superscalar_hash.v` — SuperscalarHash 骨架
- 程序缓冲区（4096 × 64-bit）
- 简单顺序执行 FSM（取指→译码→执行→回写）
- **TODO**：超标量调度（并行执行端口）

### `argon2_fill.v` — Argon2d Cache 填充骨架
- 状态机：IDLE → H0 → INIT_BLK → FILL → COMPRESS → WRITE → DONE
- 连接 Blake2b 核用于块压缩
- **TODO**：完整 Argon2d G 函数、数据相关的参考块选择

---

## 内存规划（XCVU33P 资源）

| 资源         | 用途               | 容量              | XCVU33P 可用   |
|-------------|-------------------|------------------|----------------|
| URAM        | Scratchpad (L1-L3)| 2 MiB（222块）    | 320 块 URAM    |
| HBM2        | RandomX Dataset   | ~2.08 GiB        | 8 GB HBM2      |
| HBM2        | Argon2 Cache      | 256 MB           | 8 GB HBM2      |
| BRAM        | 程序缓冲区/FIFO    | 16 KB            | 2160 块 BRAM   |
| DSP         | 整数乘法器         | 若干              | 12288 DSP58E2  |
| LUT         | AES S-box, 组合逻辑| 估计 100K LUT    | 1,541,952 LUT  |

---

## 构建说明（Vivado）

### 前提条件
- Xilinx Vivado 2022.1 或更高版本（含 XCVU33P 支持）
- HBM IP 许可（用于实现阶段；综合无需）

### 步骤

**1. 启动综合**
```bash
# 批处理模式
vivado -mode batch -source vivado/build.tcl

# 或在 Vivado GUI 中打开 Tcl 控制台执行：
source vivado/build.tcl
```

**2. 查看综合结果**
```tcl
# 在 Vivado Tcl 控制台中：
open_run synth_1 -name synth_1
report_utilization -file utilization.rpt
report_timing_summary -file timing.rpt
```

**3. 完整实现（需 HBM IP 配置）**
```tcl
launch_runs impl_1 -jobs 8
wait_on_run impl_1
```

---

## 仿真说明（iverilog）

### 安装 iverilog
```bash
# Ubuntu/Debian
sudo apt-get install iverilog

# macOS
brew install icarus-verilog
```

### 编译与运行
```bash
# 仿真模式（缩减内存，快速仿真）
iverilog -g2001 -DSIMULATION \
    -o sim/tb_randomx_top.vvp \
    rtl/aes_round.v rtl/aes_gen1r.v rtl/aes_gen4r.v rtl/aes_hash1r.v \
    rtl/blake2b_core.v rtl/scratchpad_mem.v rtl/hbm_dataset_if.v \
    rtl/alu_int.v rtl/fpu_double.v rtl/superscalar_hash.v \
    rtl/argon2_fill.v rtl/randomx_vm.v rtl/randomx_top.v \
    sim/tb_randomx_top.v

vvp sim/tb_randomx_top.vvp

# 查看波形（需安装 GTKWave）
gtkwave tb_randomx_top.vcd
```

### 仅编译检查（无仿真）
```bash
# 单独检查每个模块语法
for f in rtl/*.v; do
    echo "Checking $f..."
    iverilog -g2001 -DSIMULATION -o /dev/null $f 2>&1 || echo "FAILED: $f"
done
```

### 仿真说明
- 使用 `-DSIMULATION` 宏时：
  - Scratchpad 从 2 MiB 缩减为 32 KiB
  - Argon2d 从 262144 块缩减为 8 块（1 轮）
- HBM 接口在仿真中为 stub（`arready=0`），VM 的 Dataset 访问会等待

---

## TODO / 实现状态

| 模块              | 状态       | 主要 TODO                                  |
|------------------|------------|-------------------------------------------|
| randomx_top.v    | 骨架       | DS_GEN 阶段、完整 AXI-Lite 握手            |
| blake2b_core.v   | 骨架       | sigma 表第2-11轮、完整 G 函数调度           |
| aes_round.v      | **已实现** | 无（SubBytes/ShiftRows/MixColumns/ARK）    |
| aes_gen1r/4r.v   | 骨架       | 从种子派生正确轮密钥                         |
| aes_hash1r.v     | 骨架       | 从种子派生正确轮密钥                         |
| scratchpad_mem.v | **已实现** | 无（URAM 推断、L1/L2/L3 掩码）             |
| hbm_dataset_if.v | 骨架       | 连接 HBM IP、写通道、多事务流水             |
| alu_int.v        | 骨架       | IMUL_RCP（倒数计算）、CBRANCH 条件掩码     |
| fpu_double.v     | 骨架       | FADD/FSUB/FMUL（IEEE 754）、FDIV/FSQRT   |
| superscalar_hash.v| 骨架      | 超标量调度、完整指令集编码                   |
| randomx_vm.v     | 骨架       | 完整指令译码、内存地址计算、CFROUND         |
| argon2_fill.v    | 骨架       | G 函数、数据相关参考块选择、多轮支持         |

---

## 许可

本项目为开源硬件框架骨架，用于 RandomX 算法的 FPGA 研究目的。

RandomX 算法版权归原始作者所有（见 [tevador/RandomX](https://github.com/tevador/RandomX)）。
