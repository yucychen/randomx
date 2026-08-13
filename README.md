# RandomX FPGA 纯 Verilog 框架

面向 Xilinx Virtex UltraScale+ **XCVU33P**（型号：`xcvu33p-fsvh2104-2L-e`）的 RandomX 工作量证明算法纯 Verilog-2001 硬件实现框架。

> **状态：部分实现**  
> 所有模块均可使用 `iverilog -g2001` 编译并通过仿真。
> `blake2b_core` / `aes_round` / `scratchpad_mem` / `hbm_dataset_if` / `cache_hbm_if` /
> `axi_arbiter` / `superscalar_hash` / `fpu_double` / `argon2_fill`
> 已完整实现并有单元测试覆盖（见 [单元测试状态](#单元测试状态)）；
> 其余模块为骨架，功能逻辑以 `TODO` 注释标注，待完整实现。

---

## 目录结构

```
randomx/
├── rtl/                    # RTL 源码（纯 Verilog-2001）
│   ├── randomx_top.v       # 顶层模块：时钟/复位、寄存器接口、主 FSM
│   ├── blake2b_core.v      # Blake2b-512 压缩核（12 轮，4 路并行 G，24 周期/块）
│   │                       #   含 blake2b_g 子模块（G 混合函数，纯组合）
│   ├── aes_round.v         # AES 单轮函数（SubBytes/ShiftRows/MixColumns/ARK）
│   ├── aes_gen1r.v         # AesGenerator1R（1轮AES × 4 lane）
│   ├── aes_gen4r.v         # AesGenerator4R（4轮AES × 4 lane）
│   ├── aes_hash1r.v        # AesHash1R（4轮AES哈希 × 4 lane）
│   ├── superscalar_hash.v  # SuperscalarHash（数据集生成程序执行单元）
│   ├── randomx_vm.v        # RandomX 虚拟机（取指/译码/执行/回写）
│   ├── alu_int.v           # 整数执行单元（19条整数指令）
│   ├── fpu_double.v        # 双精度浮点单元（IEEE 754 加/减/乘/除/开方 + FSCAL/FSWAP）
│   ├── scratchpad_mem.v    # 2 MiB Scratchpad（URAM 推断，L1/L2/L3 掩码）
│   ├── hbm_dataset_if.v    # HBM2 AXI4 主设备接口（数据集读写，流水化）
│   ├── cache_hbm_if.v      # Argon2d Cache 存储接口（1 KiB 块 ↔ AXI4 突发）
│   ├── axi_arbiter.v       # 2 主设备 AXI4 仲裁器（cache + dataset 共享 HBM 端口）
│   └── argon2_fill.v       # Argon2d Cache 填充（基于 Blake2b，已实现）
├── sim/
│   ├── tb_randomx_top.v    # 基础功能仿真 testbench
│   ├── tb_blake2b.v        # Blake2b-512 参考测试向量 testbench（含多块/busy）
│   ├── tb_hbm_dataset_if.v # HBM AXI4 接口 testbench（含 AXI 从设备模型）
│   ├── tb_cache_hbm_if.v   # Cache 存储接口 testbench（含 AXI 从设备模型）
│   ├── tb_argon2_fill.v    # Argon2d 填充 testbench（黄金向量比对）
│   ├── tb_fpu_double.v     # 双精度浮点单元 testbench
│   └── tb_superscalar_hash.v # SuperscalarHash 全指令集 testbench
├── vivado/
│   ├── build.tcl           # Vivado TCL 构建脚本（非项目模式）
│   └── constraints.xdc     # 时序约束（300 MHz 时钟 + HBM 占位符）
├── .github/workflows/ci.yml# CI：语法检查 + 全部 testbench + Verilator lint
├── Makefile                # 编译 / 仿真 / lint 自动化
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
                        │  │   cache_hbm_if (AXI4 主设备)                │  │
                        │  │   Cache (256MB, 1KiB 块) 存于 HBM2         │  │
                        │  ├────────────────────────────────────────────┤  │
                        │  │   hbm_dataset_if (AXI4 主设备)              │  │
                        │  │   Dataset (~2GB) 存于 XCVU33P 8GB HBM2     │  │
                        │  ├────────────────────────────────────────────┤  │
                        │  │   axi_arbiter (2 主设备 → 1 HBM 端口)       │  │──► HBM2 AXI
                        │  └────────────────────────────────────────────┘  │    接口
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
    256 MB Cache = 262144 × 1KB 块，存于 HBM2 (cache_hbm_if.v)
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
  | 0x88    | 写   | Argon2 key 字节数（1~64，复位默认 64） |
- **主 FSM**：IDLE → CACHE_INIT → VM_RUN → FINAL_HASH → DONE
- **HBM 地址映射**（34 位字节地址，8 GB HBM 堆栈）：

  | 区域    | 基址            | 大小      | 访问模块          |
  |---------|-----------------|-----------|-------------------|
  | Dataset | `0x0_0000_0000` | ~2.08 GiB | `hbm_dataset_if.v`|
  | Cache   | `0x0_C000_0000` | 256 MiB   | `cache_hbm_if.v`  |

  两个主设备通过 `axi_arbiter.v` 共享同一个 AXI4 端口
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

### `cache_hbm_if.v` — Argon2d Cache 存储接口（**已实现**）
- Cache 为 262144 × 1 KiB = 256 MiB，远超 XCVU33P 片上 URAM（约 1.4 MiB），
  因此与 Dataset 一样存放在 HBM2 中；本模块把 `argon2_fill.v` 的 1 KiB 块端口
  转换成 256-bit HBM 伪通道上的 AXI4 突发
- 每块 = 1024 字节 / 32 字节每拍 = **32 拍突发**（`awlen/arlen = 31`）；
  字节地址 = `CACHE_BASE_ADDR + 块号 × 1024`，基址 1 KiB 对齐 ⇒ 突发不跨 4 KiB 边界
- 块端口握手与 `argon2_fill.v` 一致：写侧保持 `wr_en` 直到 `wr_rdy` 单周期脉冲
  （此时整块已写入 HBM），读侧保持 `rd_en` 直到 `rd_valid` 与整块数据同时给出；
  一次只服务一笔事务，且必须等 `*_en` 撤销后才采样下一次请求，
  因此持续拉高的 enable 不会被误判为第二次请求
- 字节序与 `argon2_fill` / `hbm_dataset_if` 一致：块位 `[8*k +: 8]` 即块内第 k 字节，
  首拍携带第 0..31 字节
- 错误处理：`RRESP`/`BRESP` 异常置粘滞位 `axi_err`，汇总到顶层状态寄存器 bit1
- 单元测试：`sim/tb_cache_hbm_if.v`（行为级 AXI 从设备模型，含随机背压、
  字节序检查、握手复用检查与 SLVERR 注入）

### `axi_arbiter.v` — 2 主设备 AXI4 仲裁器（**已实现**）
- HBM 伪通道只有一个 AXI4 从端口，而 cache（M0）与 dataset（M1）都要访问它
- 读/写通路独立仲裁；授权是排他的：某主设备仍有未完成事务时另一方不能发地址，
  因此可按记录的 owner 路由 R/B 响应（两个主设备均使用 AXI ID 0，
  排他授权正是其“响应保序”假设成立的前提）
- 通道空闲时按 round-robin 选择请求方（上次被授权者优先级更低），避免饿死
- W 通道跟随写地址 owner，保证 W 突发与被接受的 AW 属于同一主设备

### `alu_int.v` — 整数执行单元
- 完整 RandomX 整数 ISA：IADD_RS, ISUB, IMUL, IMULH, ISMULH, INEG, IXOR, IROR/IROL, ISWAP, CBRANCH, ISTORE
- 有符号/无符号 128-bit 乘法（高64位提取）
- CBRANCH 条件掩码/分支判定（规范 5.5.10）；ISTORE L1/L2/L3 级别按 mod 字段解码
- VM 侧：ST_COMPILE 预编译遍历按寄存器使用情况计算分支目标，跳转回 target+1
- **TODO**：IMUL_RCP（模乘倒数）

### `fpu_double.v` — 双精度浮点单元（**已实现**）
- FADD_R/M、FSUB_R/M：IEEE 754 加减法（对阶 → 加减 → 规格化 → 舍入），单周期
- FMUL_E：IEEE 754 乘法（53×53 → 106 位乘积，规格化 + 舍入），单周期，可推断 DSP48
- FDIV_M：恢复余数迭代除法，56 周期 + 握手周期
- FSQRT_R：恢复余数迭代开方，56 周期 + 握手周期
- FSCAL_R：`dst.u ^= 0x80F0000000000000`（符号位 + 指数低 4 位异或）
- FSWAP_R：128 位寄存器对高/低半部交换
- 四种舍入模式（FPRC）：就近偶数 / 向 -inf / 向 +inf / 向零，
  对上溢时按舍入模式产生 Inf 或最大规格数
- 完整特殊值处理：NaN（安静 NaN `0x7FF8000000000000`）、±Inf、±0、
  非规格化输入与渐进下溢（非规格化输出）
- 握手：`en` 拉高一周期发起运算，`result_valid` 单周期脉冲输出结果；
  多周期运算期间 `busy` 为高，此时不得再次拉高 `en`

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

### `argon2_fill.v` — Argon2d Cache 填充
- 完整实现 RandomX 所用的 Argon2d（version 0x13，type=Argon2d，lanes=1，
  t=3，m=262144 块，salt = `"RandomX\x03"`，不产生最终 tag）：
  1. `H0 = Blake2b-512(LE32(lanes) || LE32(tagLen) || LE32(m) || LE32(t) ||
     LE32(version) || LE32(type) || LE32(|K|) || K || LE32(|S|) || S ||
     LE32(0) || LE32(0))`
  2. `B[0] = H'(1024, H0 || LE32(0) || LE32(lane))`、
     `B[1] = H'(1024, H0 || LE32(1) || LE32(lane))`，
     其中 H' 为 Argon2 变长哈希（31 次 Blake2b-512 链式调用）
  3. 数据相关（Argon2d）参考块选择：由前一块的第 0 个 LE32 字 J1 计算
     `ref_area / relative_position / start_position`，与参考实现
     `index_alpha()` 位级一致
  4. 压缩函数 G：`R = B[i-1] ^ B[ref]`，对 R 的 8 个 128 字节行做 P 置换，
     再对 8 个列做 P 置换，`B[i] = Z ^ R`；t>1 时为 XOR 模式 `B[i] ^= Z ^ R`
  5. P 为 Argon2 BlaMka 置换（带 64 位乘加的 Blake2b 轮函数），
     每半轮 4 个 G_B 并行，一个块共 16 轮 × 2 = 32 周期
- 状态机：IDLE → H0（长 key 时经 H0_NEXT 多块链式）→ HP_FIRST → HP_NEXT → HP_WRITE →
  RD_PREV → RD_REF →（t>1 时 RD_CUR）→ ROUNDS → WRITE → DONE
- 接口：共享 Blake2b 核（`b2b_*`）+ 1 KiB 位宽的 cache 读写端口
  （按块索引寻址，面向外部 URAM/HBM 存储）
- 成本参数与源码一致且可被 parameter 覆盖：`ARGON_M` 默认 262144（仿真缩减
  由 `randomx_top.v` 实例化时覆盖，模块默认值不再随 `-DSIMULATION` 改变），
  `ARGON_T` 支持任意轮数（不再限制为 ≤4），`ARGON_M` 按参考实现规整
  （小于 `2×SYNC_POINTS×lanes` 时抬高，并向下取整到段长的整数倍；H0 仍哈希
  用户请求的 m_cost）
- key 长度可变：`KEY_BYTES` 决定端口宽度（默认 64 字节），H0 按 128 字节分块
  链式哈希，因此超过 80 字节的 key 也与参考实现一致；顶层由寄存器 0x88 指定
  实际字节数
- 单元测试：`sim/tb_argon2_fill.v`，与 Argon2 参考实现生成的黄金向量逐块比对
- **TODO**：接到真实 cache 存储（`randomx_top.v` 中的 cache 端口目前仍为占位）

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

### 快速开始（推荐：Makefile）
```bash
make            # 编译并运行全部 testbench（任一 FAIL 则退出码非 0）
make lint       # Verilator 静态检查（--lint-only -Wall）
make syntax     # 逐模块 iverilog 语法检查
make tb_blake2b # 只运行单个 testbench
make clean      # 清理仿真产物
```
仿真产物统一输出到 `sim/build/`（已被 `.gitignore` 忽略），
每个 testbench 的输出同时保存为 `sim/build/<name>.log`。

### 持续集成（CI）
`.github/workflows/ci.yml` 在每次 push / PR 时自动执行：

| Job        | 内容                                              |
|------------|---------------------------------------------------|
| `simulate` | `make syntax` + `make test`（全部 testbench 自校验）|
| `lint`     | `make lint`（Verilator `-Wall`）                   |

`make test` 会检查每个 testbench 的输出中是否含 `FAIL` / `ERROR`，
因此新增 testbench 时请沿用 `PASS` / `FAIL` 的打印约定。

### 手动编译与运行（等价命令）
```bash
# 仿真模式（缩减内存，快速仿真）
iverilog -g2001 -DSIMULATION \
    -o sim/tb_randomx_top.vvp \
    rtl/aes_round.v rtl/aes_gen1r.v rtl/aes_gen4r.v rtl/aes_hash1r.v \
    rtl/blake2b_core.v rtl/scratchpad_mem.v rtl/hbm_dataset_if.v \
    rtl/cache_hbm_if.v rtl/axi_arbiter.v \
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

# Cache 存储接口单元测试（AXI4 从设备模型 + 随机背压 + SLVERR 注入）
iverilog -g2001 -DSIMULATION -o sim/tb_cache_hbm_if.vvp \
    rtl/cache_hbm_if.v sim/tb_cache_hbm_if.v
vvp sim/tb_cache_hbm_if.vvp   # 输出 ALL TESTS PASSED

# Argon2d Cache 填充单元测试（与 Argon2 参考实现黄金向量比对）
iverilog -g2001 -DSIMULATION -o sim/tb_argon2_fill.vvp \
    rtl/blake2b_core.v rtl/argon2_fill.v sim/tb_argon2_fill.v
vvp sim/tb_argon2_fill.vvp   # 输出 ALL TESTS PASSED（需在仓库根目录运行）

# SuperscalarHash 单元测试（全指令集，比对软件模型）
iverilog -g2001 -o sim/tb_superscalar_hash.vvp \
    rtl/alu_int.v rtl/superscalar_hash.v sim/tb_superscalar_hash.v
vvp sim/tb_superscalar_hash.vvp   # 输出 PASS

# 查看波形（需安装 GTKWave）
gtkwave tb_randomx_top.vcd
```

### 仅编译检查（无仿真）
```bash
make syntax
# 等价于：
for f in rtl/*.v; do
    iverilog -g2001 -DSIMULATION -y rtl -o /dev/null $f || echo "FAILED: $f"
done
```

### Lint 说明
`make lint` 目前豁免了以下 Verilator 告警类别（均为骨架实现的已知状态，
随着 TODO 模块补全应逐步取消豁免）：
`DECLFILENAME`、`UNUSEDSIGNAL`、`UNUSEDPARAM`、`PINCONNECTEMPTY`、`UNDRIVEN`。

### 仿真说明
- 使用 `-DSIMULATION` 宏时：
  - Scratchpad 从 2 MiB 缩减为 32 KiB
  - Argon2d 从 262144 块缩减为 8 块（`randomx_top.v` 中的 parameter 覆盖）
- `sim/tb_randomx_top.v` 内含行为级 HBM AXI4 从设备模型：Cache 窗口
  （`0x0_C000_0000` 起 8 KiB）由内存支持，因此 CACHE_INIT 阶段会真正把
  Argon2d 块写进 HBM 模型；窗口外（尚未生成的 Dataset）读回全 0

---

## 单元测试状态

以下 testbench 均可用 `iverilog -g2001` 编译运行（命令见[仿真说明](#仿真说明iverilog)）：

| Testbench                  | 覆盖范围                                                | 结果 |
|----------------------------|---------------------------------------------------------|------|
| `sim/tb_blake2b.v`         | `"abc"`（外部 IV / `init` 两种）、空消息、200 字节两块链式哈希、`busy` 握手 | PASS（`ALL TESTS PASSED`）|
| `sim/tb_hbm_dataset_if.v`  | 行为级 AXI4 从设备模型：多事务流水、背压、错误注入、AXI 属性与基址检查 | PASS |
| `sim/tb_cache_hbm_if.v`    | 1 KiB 块 ↔ AXI4 突发：写/读回环、HBM 内字节序、随机背压、握手复用、SLVERR 粘滞位 | PASS（`ALL TESTS PASSED`）|
| `sim/tb_superscalar_hash.v`| 全部 14 种 SuperscalarHash 指令，与软件模型逐寄存器比对   | PASS（269 周期）|
| `sim/tb_fpu_double.v`      | FADD/FSUB/FMUL/FDIV/FSQRT/FSCAL/FSWAP，含 4 种舍入模式、NaN/Inf/±0、非规格化、上溢/下溢与 `busy` 握手 | PASS（47 项检查）|
| `sim/tb_argon2_fill.v`     | Argon2d Cache 填充：与 Argon2 参考实现黄金向量逐块比对（m=8/t=3/43 字节 key，m=32/t=1/64 字节 key，m=6→8/t=5/100 字节 key） | PASS |
| `sim/tb_randomx_top.v`     | 顶层集成冒烟测试：寄存器写种子 → start → 轮询 done → 读回哈希；含行为级 HBM 模型，检查 Cache 块确实写入 HBM | 运行完成（哈希值尚未自校验）|

> `make test` 会逐个运行上述 testbench，并在输出中出现 `FAIL` / `ERROR` 时返回非 0 退出码。

### 验证方面的已知缺口
- `tb_randomx_top.v` 尚非自校验：缺少与参考实现的期望哈希比对。
- 尚无 testbench 覆盖：`aes_round` / `aes_gen1r` / `aes_gen4r` / `aes_hash1r`、
  `alu_int`、`scratchpad_mem`、`randomx_vm`。
- 缺少与官方 [tevador/RandomX](https://github.com/tevador/RandomX) 参考实现的
  端到端测试向量对拍（黄金模型对比）。

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
| cache_hbm_if.v   | **已实现** | 连接 Vivado HBM IP（与 dataset 共用端口）    |
| axi_arbiter.v    | **已实现** | 无（读/写通路独立 round-robin 仲裁）         |
| alu_int.v        | 基本实现   | IMUL_RCP（可复用 superscalar_hash 的倒数单元）|
| fpu_double.v     | **已实现** | 流水化以提升 Fmax（当前加/乘为单周期组合路径）|
| superscalar_hash.v| **已实现** | 超标量调度（并行执行端口，性能优化）        |
| randomx_vm.v     | 骨架       | 完整指令译码、内存地址计算、CFROUND         |
| argon2_fill.v    | **已实现** | 无（cache 块经 cache_hbm_if 存入 HBM）      |

---

## 完善路线图（优先级从高到低）

1. ~~**Cache 存储** — 将 `argon2_fill.v` 的 1 KiB 块读写端口接到真实的
   cache 存储（HBM 或大容量 URAM）。~~ **已完成**：新增 `cache_hbm_if.v`
   （1 KiB 块 ↔ 32 拍 AXI4 突发）与 `axi_arbiter.v`（cache/dataset 共享
   HBM 端口），`randomx_top.v` 中的占位连接已去除。
2. **`randomx_vm.v`** — 29 条 ISA 完整译码、CFROUND、L1/L2/L3 地址掩码、
   浮点寄存器前递、Dataset 取数与 MX/MA 更新、程序结束的 Scratchpad XOR + AesHash1R。
3. **`randomx_top.v`** — 打通 DS_GEN（SuperscalarHash 8 pass）与 FINAL_HASH，
   驱动 HBM 写通道。
4. **AES 轮密钥** — `aes_gen1r` / `aes_gen4r` / `aes_hash1r` 中的常量目前为占位值，
   需按 spec 3.3/3.4 从种子派生。
5. **`alu_int.v` IMUL_RCP** — 2^128 / b 倒数计算。
6. **验证** — 补齐单元 testbench，并与参考实现做端到端向量对拍。
7. **后端** — `constraints.xdc` 中的 `PIN_NAME` 占位符、HBM 参考时钟约束、
   跨时钟域约束；`build.tcl` 中实例化 Vivado HBM IP；验证 300 MHz 时序收敛与资源占用。

---

## 许可

本项目为开源硬件框架骨架，用于 RandomX 算法的 FPGA 研究目的。

RandomX 算法版权归原始作者所有（见 [tevador/RandomX](https://github.com/tevador/RandomX)，BSD-3-Clause）。

> **待办**：仓库尚未包含 `LICENSE` 文件。请仓库所有者选定并添加正式许可证
> （建议与上游 RandomX 兼容，例如 BSD-3-Clause）。
