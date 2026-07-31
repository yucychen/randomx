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
│   ├── blake2b_core.v      # Blake2b-512 压缩核（12 轮，4 路并行 G，24 周期/块）
│   ├── aes_round.v         # AES 单轮函数（SubBytes/ShiftRows/MixColumns/ARK）
│   ├── aes_gen1r.v         # AesGenerator1R（1轮AES × 4 lane）
│   ├── aes_gen4r.v         # AesGenerator4R（4轮AES × 4 lane）
│   ├── aes_hash1r.v        # AesHash1R（4轮AES哈希 × 4 lane）
│   ├── superscalar_hash.v  # SuperscalarHash（数据集生成程序执行单元）
│   ├── randomx_vm.v        # RandomX 虚拟机（取指/译码/执行/回写）
│   ├── alu_int.v           # 整数执行单元（19条整数指令）
│   ├── fpu_double.v        # 双精度浮点单元骨架（FSCAL/FSWAP 已实现）
│   ├── scratchpad_mem.v    # 2 MiB Scratchpad（URAM 推断，L1/L2/L3 掩码）
│   ├── hbm_dataset_if.v    # HBM2 AXI4 主设备接口（数据集读写，流水化）
│   └── argon2_fill.v       # Argon2d Cache 填充骨架（基于 Blake2b）
├── sim/
│   ├── tb_randomx_top.v    # 基础功能仿真 testbench
│   ├── tb_blake2b.v        # Blake2b-512 参考测试向量 testbench（含多块/busy）
│   ├── tb_hbm_dataset_if.v # HBM AXI4 接口 testbench（含 AXI 从设备模型）
│   └── tb_superscalar_hash.v # SuperscalarHash 全指令集 testbench
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
  | 0x44    | 读   | 状态寄存器（bit0=done/~busy，bit1=HBM AXI 错误粘滞位） |
  | 0x48~0x84 | 读 | 哈希输出（512位，16×32位）   |
- **主 FSM**：IDLE → CACHE_INIT → VM_RUN → FINAL_HASH → DONE
- **TODO**：DS_GEN 阶段、完整 AXI-Lite 握手

### `blake2b_core.v` — Blake2b-512 压缩核（**已完成**）
- `blake2b_g` 子模块：G 函数组合数据通路（rotr32/rotr24/rotr16/rotr63）
- 完整 sigma 置换表（12 轮）；每周期并行执行 4 个 G 函数（1 个半轮），
  共 **24 周期/压缩**（列半轮 + 对角半轮 × 12 轮）
- `init` 输入：由核内部生成 Blake2b 参数块初始链值
  （无密钥，摘要长度由参数 `DIGEST_BYTES` 指定，默认 64 字节），此时忽略 `h_in`
- `busy` 输出：压缩进行中为高；`busy` 期间的 `start` 被忽略，防止状态被破坏
- 已通过参考测试向量验证（`sim/tb_blake2b.v`）：
  `"abc"`（外部 IV / `init` 两种方式）、空消息、200 字节两块链式哈希、`busy` 握手

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

### `hbm_dataset_if.v` — HBM2 数据集 AXI4 主设备（**已实现**）
- AXI4 读通道主设备（AR + R 通道），256-bit 总线宽度（HBM 伪通道带宽）
- 请求 FIFO + 多事务流水（最多 `RD_FIFO_DEPTH` 个未完成读事务，单 ID 保序）；
  AR 发起前预留响应 FIFO 空间，故 `rready` 可常高且不会溢出
- AXI4 写通道（AW + W + B）流水化：写请求 FIFO + AW/W/B 三通道解耦，
  最多 `WR_FIFO_DEPTH` 笔未完成写事务，W 突发可背靠背发送（无气泡），
  数据集生成不再被 B 响应往返阻塞
- 参数化：`DATASET_BASE_ADDR`（数据集在 HBM 中的基址）、`RD/WR_FIFO_DEPTH`、
  总线位宽（每条目拍数与 `arsize/arlen` 由参数自动推导）
- 错误处理：`RRESP` 按突发聚合后随 `resp_err` 返回，`BRESP` 经 `wr_err`
  （与 `wr_done` 同拍）上报，并汇总为粘滞位 `axi_err`（顶层状态寄存器 bit1）
- R 拍重组以 `RLAST` 定界，从设备返回异常长度的突发也不会导致错位
- 单元测试：`sim/tb_hbm_dataset_if.v`（行为级 AXI 从设备模型，含多事务流水、
  背压、错误注入、AXI 属性与基址检查）
- **TODO**：连接到 Vivado HBM IP；将写接口接到 superscalar_hash

### `alu_int.v` — 整数执行单元
- 完整 RandomX 整数 ISA：IADD_RS, ISUB, IMUL, IMULH, ISMULH, INEG, IXOR, IROR/IROL, ISWAP, CBRANCH, ISTORE
- 有符号/无符号 128-bit 乘法（高64位提取）
- CBRANCH 条件掩码/分支判定（规范 5.5.10）；ISTORE L1/L2/L3 级别按 mod 字段解码
- VM 侧：ST_COMPILE 预编译遍历按寄存器使用情况计算分支目标，跳转回 target+1
- **TODO**：IMUL_RCP（模乘倒数）

### `fpu_double.v` — 双精度浮点单元
- FSCAL_R：符号位异或 + 指数异或（**已实现**）
- FSWAP_R：寄存器高/低半部交换（**已实现**）
- FADD/FSUB/FMUL：**TODO** — 需要 IEEE 754 加法器/乘法器
- FDIV/FSQRT：**TODO** — 需要迭代除法/开方单元

### `superscalar_hash.v` — SuperscalarHash 执行单元（**已实现**）
- 程序缓冲区（4096 × 64-bit），指令编码：
  `[63:56]=opcode [55:53]=dst [52:50]=src [49:48]=mod_shift [31:0]=imm32`
- 完整指令集（规范 §6.2）：ISUB_R、IXOR_R、IADD_RS、IMUL_R、IROR_C、
  IADD_C7/C8/C9、IXOR_C7/C8/C9、IMULH_R、ISMULH_R、IMUL_RCP
- 顺序执行 FSM（取指→译码→发射→回写），每条指令写回后再取下一条，
  消除寄存器 RAW/WAW 冒险
- IMUL_RCP 倒数单元：逐位恢复余数除法，计算 `floor(2^(63+bsr)/imm32)`
  （与 `reciprocal.c` 位级一致），耗时 64+bsr(imm32) 周期
- 单元测试：`sim/tb_superscalar_hash.v`（覆盖全部 14 种指令，比对软件模型结果）
- **TODO**：超标量调度（并行执行端口，仅影响吞吐量，结果不变）

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

# Blake2b 核单元测试（RFC 7693 "abc"、空消息、多块、busy 握手）
iverilog -g2001 -o sim/tb_blake2b.vvp rtl/blake2b_core.v sim/tb_blake2b.v
vvp sim/tb_blake2b.vvp   # 输出 ALL TESTS PASSED

# HBM 数据集接口单元测试（AXI4 从设备模型 + 错误注入）
iverilog -g2001 -o sim/tb_hbm_dataset_if.vvp \
    rtl/hbm_dataset_if.v sim/tb_hbm_dataset_if.v
vvp sim/tb_hbm_dataset_if.vvp   # 输出 PASS

# SuperscalarHash 单元测试（全指令集，比对软件模型）
iverilog -g2001 -o sim/tb_superscalar_hash.vvp \
    rtl/alu_int.v rtl/superscalar_hash.v sim/tb_superscalar_hash.v
vvp sim/tb_superscalar_hash.vvp   # 输出 PASS

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
| blake2b_core.v   | **已实现** | 无（12 轮压缩、init/busy 接口，24 周期/块） |
| aes_round.v      | **已实现** | 无（SubBytes/ShiftRows/MixColumns/ARK）    |
| aes_gen1r/4r.v   | 骨架       | 从种子派生正确轮密钥                         |
| aes_hash1r.v     | 骨架       | 从种子派生正确轮密钥                         |
| scratchpad_mem.v | **已实现** | 无（URAM 推断、L1/L2/L3 掩码）             |
| hbm_dataset_if.v | **已实现** | 连接 Vivado HBM IP、写接口接到 superscalar_hash |
| alu_int.v        | 骨架       | IMUL_RCP（倒数计算）、CBRANCH 条件掩码     |
| fpu_double.v     | 骨架       | FADD/FSUB/FMUL（IEEE 754）、FDIV/FSQRT   |
| superscalar_hash.v| **已实现** | 超标量调度（并行执行端口，性能优化）        |
| randomx_vm.v     | 骨架       | 完整指令译码、内存地址计算、CFROUND         |
| argon2_fill.v    | 骨架       | G 函数、数据相关参考块选择、多轮支持         |

---

## 许可

本项目为开源硬件框架骨架，用于 RandomX 算法的 FPGA 研究目的。

RandomX 算法版权归原始作者所有（见 [tevador/RandomX](https://github.com/tevador/RandomX)）。
