# RandomX FPGA 纯 Verilog 框架

面向 Xilinx Virtex UltraScale+ **XCVU33P**（型号：`xcvu33p-fsvh2104-2L-e`）的 RandomX 工作量证明算法纯 Verilog-2001 硬件实现框架。

> **状态：算法路径已打通**  
> 所有模块均可使用 `iverilog -g2001` 编译并通过仿真，`make test` 的 14 个
> testbench 与 `make lint` 全部通过（见 [单元测试状态](#单元测试状态)）。
> Cache 初始化 → Dataset 生成 → Scratchpad 填充 → 程序生成 → 8 程序链执行 →
> 最终哈希的完整流程已在 RTL 中实现。
> 仍未完成的算法环节只剩片上 `generateSuperscalar`（spec §6.1）：Dataset 生成
> 用的 8 个超标量程序目前仍由主机经寄存器接口写入。
> 完整 `calculate_hash` 的端到端黄金向量对拍亦待补（见 [完善路线图](#完善路线图)）。

---

## 目录结构

```
randomx/
├── rtl/                    # RTL 源码（纯 Verilog-2001）
│   ├── randomx_top.v       # 顶层模块：时钟/复位、寄存器接口、主 FSM
│   ├── blake2b_core.v      # Blake2b-512 压缩核（12 轮，4 路并行 G，24 周期/块）
│   │                       #   含 blake2b_g 子模块（G 混合函数，纯组合）
│   ├── aes_round.v         # AES 单轮函数（AESENC / AESDEC）
│   ├── aes_gen1r.v         # AesGenerator1R（1轮AES × 4 lane）
│   ├── aes_gen4r.v         # AesGenerator4R（4轮AES × 4 lane）
│   ├── aes_hash1r.v        # AesHash1R（数据作轮密钥的流式吸收）
│   ├── superscalar_hash.v  # SuperscalarHash（数据集生成程序执行单元）
│   ├── dataset_gen.v       # Dataset item 生成器（8 次 cache 访问 + SuperscalarHash）
│   ├── prog_gen.v          # VM 程序 / entropy 生成器（AesGenerator4R）
│   ├── randomx_vm.v        # RandomX 虚拟机（取指/译码/执行/回写）
│   ├── alu_int.v           # 整数执行单元（19条整数指令，含 IMUL_RCP）
│   ├── recip.v             # 共享倒数单元（IMUL_RCP，逐位恢复余数除法）
│   ├── fpu_double.v        # 双精度浮点单元（IEEE 754 加/减/乘/除/开方 + FSCAL/FSWAP）
│   ├── scratchpad_mem.v    # 2 MiB Scratchpad（URAM 推断，L1/L2/L3 掩码）
│   ├── hbm_dataset_if.v    # HBM2 AXI4 主设备接口（数据集读写，流水化）
│   ├── cache_hbm_if.v      # Argon2d Cache 存储接口（1 KiB 块 ↔ AXI4 突发）
│   ├── axi_arbiter.v       # 2 主设备 AXI4 仲裁器（cache + dataset 共享 HBM 端口）
│   ├── argon2_fill.v       # Argon2d Cache 填充（基于 Blake2b，已实现）
│   └── randomx_hbm_top.v   # 板级顶层（复位门控 + 地址位宽适配 + HBM IP 例化）
├── sim/
│   ├── tb_randomx_top.v    # 基础功能仿真 testbench
│   ├── tb_randomx_hbm_top.v # 板级顶层 testbench（复位门控/地址适配自校验）
│   ├── tb_blake2b.v        # Blake2b-512 参考测试向量 testbench（含多块/busy）
│   ├── tb_aes.v            # AES 原语 testbench（黄金向量比对）
│   ├── tb_hbm_dataset_if.v # HBM AXI4 接口 testbench（含 AXI 从设备模型）
│   ├── tb_cache_hbm_if.v   # Cache 存储接口 testbench（含 AXI 从设备模型）
│   ├── tb_argon2_fill.v    # Argon2d 填充 testbench（黄金向量比对）
│   ├── tb_randomx_vm.v     # RandomX VM testbench（指令集/主循环自校验）
│   ├── tb_fpu_double.v     # 双精度浮点单元 testbench
│   ├── tb_superscalar_hash.v # SuperscalarHash 全指令集 testbench
│   ├── tb_alu_int.v        # 整数执行单元 testbench（全 19 条 opcode）
│   ├── tb_scratchpad_mem.v # Scratchpad testbench（读写回环 + L1/L2/L3 掩码）
│   ├── tb_prog_gen.v       # 程序/entropy 生成器 testbench（黄金模型比对）
│   └── tb_dataset_gen.v    # Dataset item 生成器 testbench（黄金模型比对）
├── vivado/
│   ├── build.tcl           # Vivado TCL 构建脚本（含 HBM IP / 协议转换器创建）
│   ├── constraints.xdc     # 静态约束（引脚 / IOSTANDARD 占位，纯 XDC 命令）
│   └── constraints.tcl     # 脚本化约束（300 MHz + HBM 时钟 + SLR pblock）
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
                        │  │          │   └──────────────┘               │
  AXI-Lite ────────────►│  │CACHE_INIT│   ┌──────────────┐               │
  控制寄存器接口         │  │DS_GEN    │──►│ dataset_gen  │◄─►superscalar │
  (start/done/seed/hash)│  │SP_FILL   │   │ (Dataset生成) │    _hash      │
  SuperscalarHash 程序   │  │PROG_GEN  │   └──────────────┘               │
                        │  │VM_RUN    │   ┌──────────────┐               │
                        │  │CHAIN_HASH│──►│  prog_gen    │◄─►aes_gen4r   │
                        │  │FINAL_HASH│   │ (程序/entropy)│               │
                        │  │          │   └───────┬──────┘               │
                        │  │          │   ┌───────▼──────┐                │
                        │  │          │──►│ randomx_vm   │                │
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
[2] SuperscalarHash 数据集生成 (dataset_gen.v + superscalar_hash.v)
    Dataset ≈ 2.08 GB, 存于 HBM2
    每个 Dataset 条目 = 8 次 cache 访问 + 8 个 SuperscalarHash 程序
    │
    ▼
[2b] Scratchpad 填充 (aes_gen1r.v) + 程序/entropy 生成 (prog_gen.v)
    2 MiB Scratchpad ← AesGenerator1R
    2176 字节程序（16 entropy 字 + 256 指令）← AesGenerator4R
    │
    ▼
[3] RandomX VM 执行 (randomx_vm.v)
    8 个链式程序，每个程序 2048 次迭代，每次迭代：
      a) 获取 Dataset 条目 (hbm_dataset_if.v)
      b) 执行 256 条指令程序
         - 整数指令 → alu_int.v（IMUL_RCP → recip.v）
         - 浮点指令 → fpu_double.v
         - 内存访问 → scratchpad_mem.v
      c) Scratchpad 载入 / 回写
    程序之间：Blake2b-512(RegisterFile) → 下一个程序的种子 (prog_gen.v)
    │
    ▼
[4] 最终哈希 (aes_hash1r.v + blake2b_core.v)
    AesHash1R 逐 64 字节块压缩整个 Scratchpad → 512 bit，写回 a 寄存器
    Blake2b-256(RegisterFile, 256 字节) → 256 bit 输出哈希
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
  | 0x8C    | 写   | SuperscalarHash 程序字低 32 位          |
  | 0x90    | 写   | SuperscalarHash 程序字高 32 位          |
  | 0x94    | 写   | 程序缓冲索引（写入即提交上面的 64 位指令字）|
  | 0x98    | 写   | 程序配置：[2:0] 程序号、[15:4] 指令数、[18:16] 地址寄存器 |
- **主 FSM**：IDLE → CACHE_INIT（Argon2d）→ DS_GEN（Dataset 生成）→
  SP_FILL（AesGenerator1R 填充 Scratchpad）→ PROG_GEN（AesGenerator4R
  生成程序与 entropy）→ VM_RUN →〔CHAIN_HASH → PROG_GEN → VM_RUN〕×7 →
  FINAL_HASH（Blake2b-256）→ DONE
- **8 程序链**（RANDOMX_PROGRAM_COUNT = 8，规范 §4.6）：
  `fillAes1Rx4` 会写回生成器状态，因此 Scratchpad 填充后的末态即为
  程序 0 的种子；此后每个程序执行完毕，用 Blake2b-512(RegisterFile)
  产生下一个程序的 64 字节种子（CHAIN_HASH 阶段）。
  只有链上最后一个程序（`chain_cnt == 7`）会置起 VM 的 `do_final`，
  触发 `getFinalResult` 的 Scratchpad AesHash1R 压缩
- **最终哈希**：256 字节寄存器堆（`regfile_out`，其 a 寄存器已被
  AesHash1R 摘要覆盖）经共享的 `blake2b_core` 以 Blake2b-256 参数块压缩，
  结果放在 `hash_out[255:0]`。寄存器堆恰为两个 128 字节 Blake2b 块，
  链哈希与最终哈希共用同一个两块时序器，仅参数块 IV 不同（512 位 / 256 位）
- **HBM 地址映射**（34 位字节地址，8 GB HBM 堆栈）：

  | 区域    | 基址            | 大小      | 访问模块          |
  |---------|-----------------|-----------|-------------------|
  | Dataset | `0x0_0000_0000` | ~2.08 GiB | `hbm_dataset_if.v`|
  | Cache   | `0x0_C000_0000` | 256 MiB   | `cache_hbm_if.v`  |

  两个主设备通过 `axi_arbiter.v` 共享同一个 AXI4 端口。
  Cache 读端口由 `argon2_fill`（CACHE_INIT 阶段）与 `dataset_gen`
  （DS_GEN 阶段）分时独占，两个阶段不重叠，因此只需一个归属多路选择器
- **TODO**：完整 AXI-Lite 握手（当前为简化的无握手寄存器接口）

### `blake2b_core.v` — Blake2b-512 压缩核（**已完成**）
- `blake2b_g` 子模块：G 函数组合数据通路（rotr32/rotr24/rotr16/rotr63）
- 完整 sigma 置换表（12 轮）；每周期并行执行 4 个 G 函数（1 个半轮），
  共 **24 周期/压缩**（列半轮 + 对角半轮 × 12 轮）
- `init` 输入：由核内部生成 Blake2b 参数块初始链值
  （无密钥，摘要长度由参数 `DIGEST_BYTES` 指定，默认 64 字节），此时忽略 `h_in`
- `busy` 输出：压缩进行中为高；`busy` 期间的 `start` 被忽略，防止状态被破坏
- 已通过参考测试向量验证（`sim/tb_blake2b.v`）：
  `"abc"`（外部 IV / `init` 两种方式）、空消息、200 字节两块链式哈希、`busy` 握手

### `aes_round.v` — AES 单轮函数（**已实现**）
- 实现 x86 AES-NI 的两个单轮原语（RandomX 即以此定义）：
  - `dec = 0`（AESENC）：ShiftRows → SubBytes → MixColumns → AddRoundKey
  - `dec = 1`（AESDEC）：InvShiftRows → InvSubBytes → InvMixColumns → AddRoundKey
- SubBytes / InvSubBytes：两张 256 项 LUT S-box（纯组合逻辑）；
  字节替换与行移位可交换，故两个方向都按 (Sub, Shift) 顺序实现
- MixColumns / InvMixColumns：GF(2⁸) MDS 矩阵 `[2 3 1 1]` 与逆矩阵 `[14 11 13 9]`
- `last_round` 控制是否跳过 (Inv)MixColumns
- 已用 FIPS-197 AES-128 向量校验过的软件模型比对（`sim/tb_aes.v`）

### `aes_gen1r.v` / `aes_gen4r.v` / `aes_hash1r.v`（**已实现**）
- 基于 `aes_round.v` 构建的 AES 生成器和哈希器，4 × 128-bit lane 并行处理
  （64 字节状态），轮密钥常量取自 RandomX 参考实现（`aes_hash.cpp`），
  按 `{word3, word2, word1, word0}` 拼接以匹配 `_mm_set_epi32()` 的小端布局
- `aes_gen1r`（spec §3.3，`fillAes1Rx4`）：每 64 字节输出一轮，
  lane0/2 用 `aesdec`、lane1/3 用 `aesenc`，密钥为 `AES_GEN_1R_KEY0..3`；
  输出即新状态，由顶层 Scratchpad 填充阶段回灌
- `aes_gen4r`（spec §3.4，`fillAes4Rx4`）：每 64 字节四轮，
  lane0/1 依次用 `key0..key3`、lane2/3 依次用 `key4..key7`，
  同样是 lane0/2 解密、lane1/3 加密；由 `prog_gen` 驱动
- `aes_hash1r`（spec §3.5，`hashAes1Rx4`）：状态初值为固定的
  `AES_HASH_1R_STATE0..3`，**消息块本身作为轮密钥**被吸收
  （lane0/2 加密、lane1/3 解密），最后再用 `xkey0` / `xkey1` 各跑一轮扩散；
  握手为 `start`（重载初值）/ `blk_valid` / `blk_last` / `busy` / `valid`，
  单块哈希即三者同拍拉高

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
- 写接口已由 `dataset_gen.v` 驱动（Dataset item 生成）

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

### `dataset_gen.v` — Dataset item 生成器（**已实现**）
- 实现 spec §7.3 `initDatasetItem`：
  `r0 = (itemNumber + 1) × 6364136223846793005`，`r1..r7 = r0 ^ superscalarAdd1..7`，
  随后 8 次「取 cache 行 → 执行 SuperscalarHash 程序 → 与 cache 行 XOR →
  由程序的地址寄存器决定下一次 cache 行」
- cache 行经 `cache_hbm_if` 的 1 KiB 块端口读取：块号 = 行号 >> 4，
  64 字节混合块 = 块内第 `行号[3:0]` 个 512 位切片
- 生成的 64 字节 item 通过 `hbm_dataset_if` 的写请求端口写入 HBM
- 8 个 SuperscalarHash 程序**不在片上生成**（spec §6.1 的 `generateSuperscalar`
  依赖调度模型，不适合 RTL 实现），而是由主机经寄存器接口写入共享程序缓冲
  （程序 i 占用第 i × 512 个字的窗口），长度与地址寄存器另行配置。
  复位后所有程序长度为 0，此时 SuperscalarHash 退化为 NOP，
  数据通路仍可端到端跑通

### `prog_gen.v` — VM 程序 / entropy 生成器（**已实现**）
- 用 `aes_gen4r` 生成 2176 字节 = 34 个 64 字节块：
  前 2 块（128 字节）→ 16 个程序配置 entropy 字，后 32 块（2048 字节）→
  256 条指令字
- 生成器状态回环喂给下一块，`state_out` 输出末态供后续程序链使用
- 输出直接驱动 `randomx_vm` 的 `prog_wr_*` / `cfg_wr_*` 端口

### `randomx_vm.v` — RandomX 虚拟机（**已实现**）
- **程序缓冲区**：256 × 64-bit（`prog_wr_*` 端口预加载）；
  指令编码 `[63:56]=opcode [55:52]=dst [51:48]=src [47:32]=mod [31:0]=imm32`，
  `mod[1:0]=mod.mem`、`mod[3:2]=mod.shift`、`mod[7:4]=mod.cond`；
  整数 opcode 编号与 `alu_int.v` 一致（0..18），浮点/控制为 19..29
- **完整 29 条 ISA 译码**：整数 R/M 变体（IADD_RS/IADD_M/ISUB/IMUL/IMULH/
  ISMULH/IMUL_RCP/INEG/IXOR/IROR/IROL/ISWAP）、CBRANCH、ISTORE、CFROUND、NOP、
  浮点 FADD_R/M、FSUB_R/M、FSCAL_R、FMUL_R、FDIV_M、FSQRT_R、FSWAP_R
- **内存地址生成**（规范 §5.5）：`src == dst` 时使用立即数寻址 + L3 掩码，
  否则由 `mod.mem` 选择 L1/L2 掩码；ISTORE 由 `mod.cond ≥ 14` 选择 L3
- **浮点执行**：每条浮点指令对寄存器对的低/高半部各发射一次（共享 `fpu_double`），
  FSWAP_R 直接交换半部；CFROUND 按 `fprc = ror(r[src], imm32 % 64) & 3` 更新舍入模式
- **程序配置**（规范 §4.6.4，由 `cfg_wr_*` 写入的 16 个 entropy 字派生）：
  a 寄存器（`getSmallPositiveFloatBits`）、`ma`/`mx`、readReg0..3、
  `datasetOffset = (entropy[13] % (DatasetExtraItems+1)) × 64`（64 周期取模）、
  eMask0/1（`getFloatMask`）
- **主循环**（规范 §4.6.2，`ITERATIONS` 次）：spAddr 混合 → Scratchpad 载入
  （`r[i] ^= load64`，`f[i]`/`e[i]` 由两个 int32 转 double 并做指数/尾数掩码）→
  执行 256 条指令 → `mx` 更新 → Dataset 取数并异或到 r → 交换 `ma`/`mx` →
  回写 `r[i]` 与 `f[i] ^ e[i]` → spAddr 清零
- **CBRANCH**：ST_COMPILE 预编译遍历按寄存器使用情况计算分支目标，跳转回 target+1
- **最终哈希**（`getFinalResult`，仅当 `do_final` 为高，即链上最后一个程序）：
  整个 Scratchpad 按 64 字节块依次流入 AesHash1R（`aes_blk_valid` /
  `aes_blk_last` 握手），得到的 64 字节摘要写回 a 寄存器；
  顶层再对 256 字节寄存器堆（`regfile_out`）做 Blake2b-256 收尾
- **寄存器堆导出**：`regfile_out`（2048 位）按参考实现的 RegisterFile 字节布局
  排列（r[8] / f[4][2] / e[4][2] / a[4][2]），供顶层做程序链哈希与最终哈希
- **参数**：`ITERATIONS`（默认 2048）、`SP_WORDS`（默认 262144）；
  `randomx_top.v` 在 `-DSIMULATION` 下覆盖为 4 / 4096
- 单元测试：`sim/tb_randomx_vm.v`（定向程序覆盖各指令类 + 主循环、最终
  AesHash1R 摘要写回 a 寄存器、`regfile_out` 字节布局，逐寄存器自校验）

### `alu_int.v` — 整数执行单元（**已实现**）
- 完整 RandomX 整数 ISA：IADD_RS, ISUB, IMUL, IMULH, ISMULH, IMUL_RCP, INEG,
  IXOR, IROR/IROL, ISWAP, CBRANCH, ISTORE
- 有符号/无符号 128-bit 乘法（高64位提取）
- CBRANCH 条件掩码/分支判定（规范 5.5.10）；ISTORE L1/L2/L3 级别按 mod 字段解码
- VM 侧：ST_COMPILE 预编译遍历按寄存器使用情况计算分支目标，跳转回 target+1
- IMUL_RCP 复用共享的 `recip.v`：先算出 `randomx_reciprocal(imm32)` 再做 64 位
  乘法，因此该指令为多周期，执行期间 `busy` 为高，结果仍由 `result_valid` 指示
  （调用方 `randomx_vm` / `superscalar_hash` 本就等待该脉冲，无需改动 FSM）
- 单元测试：`sim/tb_alu_int.v`（全部 19 条 opcode + 7 组 IMUL_RCP 黄金向量）

### `recip.v` — 共享倒数单元（**已实现**）
- 实现规范 §5.5.11 / 参考实现 `reciprocal.c` 的 `randomx_reciprocal(divisor)`：
  以逐位恢复余数除法计算 `2^(63+bsr) / divisor` 的低 64 位，
  其中 `bsr` 为最高置位比特的位号 + 1，共 `64 + bsr` 个周期
- `start` / `divisor` 发起，`valid` 脉冲输出 `quotient`，运算期间 `busy` 为高；
  除数为 0 时返回 0
- 由 `alu_int`（VM 的 IMUL_RCP）与 `superscalar_hash`（Dataset 生成的
  IMUL_RCP）共享，两处不再各自实现除法

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
- IMUL_RCP 复用共享的 `recip.v` 倒数单元（与 `reciprocal.c` 位级一致），
  耗时 64+bsr(imm32) 周期
- `prog_base` 输入指定程序在缓冲区中的起始地址，因此 Dataset 生成所需的
  8 个程序可共存于同一个 4096 字的程序缓冲（程序 i 占用第 i × 512 个字起的窗口）
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
- Cache 读写端口已在 `randomx_top.v` 中接到 `cache_hbm_if`（1 KiB 块 ↔ AXI4
  突发），并经 `axi_arbiter` 与 `hbm_dataset_if` 共享 HBM 端口

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

### 两种构建方式

`vivado/build.tcl` 有一个 `hbm_enable` 开关，决定顶层与是否创建 HBM IP：

| 构建 | 命令 | 顶层 | HBM IP | 用途 |
|------|------|------|--------|------|
| 厂商无关 | `vivado -mode batch -source vivado/build.tcl` | `randomx_top` | 否 | 快速综合/资源评估，无需许可 |
| 板级 | `vivado -mode batch -source vivado/build.tcl -tclargs hbm` | `randomx_hbm_top` | 是 | 上板，实现阶段需 HBM 许可 |

**1. 启动综合**
```bash
vivado -mode batch -source vivado/build.tcl            # 厂商无关
vivado -mode batch -source vivado/build.tcl -tclargs hbm  # 板级（含 HBM IP）
```

> 上面两条是**操作系统命令行**（cmd / PowerShell / bash）中的命令，不能直接粘贴到
> Vivado GUI 的 Tcl Console。若在 Tcl Console 中输入，会得到
> `Unknown Tcl command 'vivado -mode batch -source ...' sending command to the OS
> shell for execution`（此时 Vivado 已在运行，无需再启动一次）。
> 在 Vivado GUI 的 Tcl Console 中请改用 `source`：
>
> ```tcl
> source E:/project/vivado/randomx/vivado/build.tcl                      ;# 厂商无关
> set hbm_enable 1; source E:/project/vivado/randomx/vivado/build.tcl    ;# 板级
> ```

**2. 查看综合结果**
```tcl
open_run synth_1 -name synth_1
report_utilization -file utilization.rpt
report_timing_summary -file timing.rpt
# 定位组合逻辑过深的路径：300 MHz 下逻辑级数 > ~8 基本无法收敛
report_design_analysis -logic_level_distribution
```

**3. 完整实现（需 HBM IP 许可）**
```tcl
launch_runs impl_1 -jobs 8
wait_on_run impl_1
```

### HBM IP 配置要点

`build.tcl` 在 `-tclargs hbm` 时创建两个 IP：

| IP | 关键配置 | 理由 |
|----|---------|------|
| `hbm_0` | `USER_HBM_DENSITY 4GB`、`USER_HBM_STACK 1` | XCVU33P 单个 4 GB HBM2 堆栈 |
| | `USER_SAXI_00 true` | 第一版只启用 1 个 AXI 端口，对应唯一的 `m_axi_*` |
| | **`USER_SWITCH_ENABLE_00 true`** | **必须**：见下方地址映射说明 |
| | `USER_AXI_CLK_FREQ 300` | 与核心同频，省掉 CDC |
| | `USER_HBM_REF_CLK_0 100` / `USER_APB_PCLK_0 100` | 参考时钟与 APB 配置时钟 |
| `axi_protocol_converter_0` | `SI_PROTOCOL AXI4` → `MI_PROTOCOL AXI3` | `cache_hbm_if` 发 32 拍突发，HBM 从端口是 AXI3 风格（`awlen` 4 位，最多 16 拍），需拆分 |

> **版本兼容性**：HBM IP 暴露的 `USER_*` 参数随 Vivado 版本变化（例如
> `USER_HBM_REF_CLK_XTAL_0` 只存在于部分版本）。`build.tcl` 通过
> `apply_ip_config` 先用 `list_property` 过滤，只设置当前 IP 真正支持的参数，
> 不支持的参数仅打印 WARNING 并跳过，避免
> `[Vivado 12-4371] Cannot find parameter ... on IP 'hbm_0'` 导致整个
> `set_property -dict` 事务回滚、脚本中断。

> **为什么必须开 Global Addressing**：单个 HBM 伪通道只映射 256 MB，而本设计
> 的 Dataset 在 `0x0_0000_0000`（~2.08 GiB）、Cache 在 `0x0_C000_0000`（256 MiB）。
> 不开内部 AXI Switch 时，`0xC000_0000` 的 Cache 访问会返回 DECERR，
> 表现为状态寄存器 `0x44` 的 bit1（AXI 错误粘滞位）置起。

### `randomx_hbm_top.v` — 板级顶层

`randomx_top.v` 保持厂商无关（不例化任何 IP），HBM 相关的适配全部放在这一层：

1. **复位门控**：HBM 控制器通过 `apb_complete` 报告就绪，
   `sys_rst_n & hbm_init_done` 经同步器后作为核心复位。
   HBM 未就绪就发起 AXI 事务是"综合通过但上板读回全 0"的最常见原因。
2. **地址位宽适配**：核心输出 34 位，4 GB HBM 的 AXI 从端口是 33 位。
   被裁掉的高位在本设计中恒为 0，该假设由硬件检查——一旦某个地址的高位非 0，
   粘滞标志 `hbm_addr_err` 置起并可被主机读回。
3. **可选 IP 例化**：由 `HBM_IP` 宏控制。未定义时（仿真 / lint / `make syntax`）
   AXI 端口暴露在边界上，可继续复用现有的行为级 HBM 模型；
   定义时（`build.tcl` 自动设置）转而驱动 `hbm_0` 与 `axi_protocol_converter_0`。
4. **黑盒引脚全连接**：IP 的输入引脚若悬空，综合会报
   `CRITICAL WARNING: [Synth 8-4442] BlackBox module ... has unconnected pin ...`。
   因此显式接好两处：协议转换器的 `s_axi_awregion` / `s_axi_arregion`（接 0）；
   以及 HBM IP 的 **`AXI_01` 伪通道端口**——HBM IP 的 AXI 端口成对出现，
   打开控制器 0（`USER_MC_ENABLE_00`）会同时引出 `AXI_00` 与 `AXI_01`，
   本设计只用 `AXI_00`（靠 Global Addressing 覆盖整个堆栈），
   `AXI_01` 的时钟/复位照常驱动、握手信号全部拉低、数据总线清零。

### 约束说明（`vivado/constraints.tcl` + `vivado/constraints.xdc`）

- **文件分工**：Vivado 以「约束模式」解析 `.xdc`，其中不允许 `if` / `foreach`
  等 Tcl 控制流，否则报
  `CRITICAL WARNING: [Designutils 20-1307] Command 'if' is not supported in the
  xdc constraint file`，并且被包住的约束会被**静默丢弃**（等于没有约束）。
  因此所有带条件的约束放在 `constraints.tcl`，由 `build.tcl` 以
  `file_type TCL` 加入 `constrs_1`（整份文件按 Tcl 脚本 source）；
  `constraints.xdc` 只保留纯静态 XDC 命令，并继续作为 target constraints file
  （GUI 写回的引脚约束落在这里）
- 同一份脚本同时适配两种顶层：通过 `get_ports -quiet` 判断端口是否存在，
  自动匹配 `clk`/`sys_clk`、`rst_n`/`sys_rst_n`
- **已删除**原先加在 `m_axi_*` 上的一批 `set_false_path`——接上 HBM 后
  它们会掩盖设计中风险最高路径的真实违例；厂商无关构建改用
  `set_input_delay`/`set_output_delay` 给出 I/O 预算
- HBM 参考时钟 / APB 时钟已填实（100 MHz），并与 `sys_clk_300mhz`
  做 `set_clock_groups -asynchronous`
- 提供 SLR pblock 模板（`hbm_pblock_enable`，默认关闭）：把
  `argon2_fill` / `cache_hbm_if` / `hbm_dataset_if` / `axi_arbiter` /
  协议转换器约束到 HBM 所在的底部 SLR。
  **关键原因**：`argon2_fill` → `cache_hbm_if` 是一条 8192 位块总线，
  跨 SLR 会消耗大量 SLL 资源。CLOCKREGION 范围与器件相关，
  需在 Device 视图确认后再启用

### 300 MHz 时序收敛已知风险

| 风险 | 位置 | 对策 |
|------|------|------|
| **最高** | `fpu_double.v` 的 FADD/FMUL 单周期组合路径 | 拆成 2~3 级流水（`tb_fpu_double.v` 的 47 项检查必须仍通过） |
| 高 | `argon2_fill.v` 的 BlaMka 乘加链（32×32 乘 → 64 位加 → 旋转 → 再乘加） | 确认乘法推断成 DSP58；必要时在 `gb` 内插流水（Cache 初始化只做一次，不影响稳态性能） |
| 中 | `axi_arbiter.v` 的纯组合路由 | 若 timing report 指向仲裁器，在其输出侧插一级 AXI Register Slice |
| 待评估 | `randomx_vm.v` ↔ `scratchpad_mem` 地址路径 | 注意 URAM 输出寄存器级；VM 已补全，可在综合后评估 |
| 待评估 | `recip.v` 的逐位除法比较/减法链 | 单比特步进，逻辑级数低；若成为瓶颈可拆成两拍 |

收敛顺序：先 `make test` / `make lint` 回归 → 综合看
`report_design_analysis -logic_level_distribution` → 逻辑级数超标的先改 RTL →
再进实现。WNS 略负（> -0.3 ns）时才值得试
`Performance_ExplorePostRoutePhysOpt` 策略；差得多（< -1 ns）应回到 RTL。

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
make lint       # Verilator 静态检查（--lint-only -Wall，randomx_top + randomx_hbm_top 两个顶层）
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
    rtl/recip.v rtl/alu_int.v rtl/fpu_double.v rtl/superscalar_hash.v \
    rtl/dataset_gen.v rtl/prog_gen.v \
    rtl/argon2_fill.v rtl/randomx_vm.v rtl/randomx_top.v \
    sim/tb_randomx_top.v

vvp sim/tb_randomx_top.vvp

# Blake2b 核单元测试（RFC 7693 "abc"、空消息、多块、busy 握手）
iverilog -g2001 -o sim/tb_blake2b.vvp rtl/blake2b_core.v sim/tb_blake2b.v
vvp sim/tb_blake2b.vvp   # 输出 ALL TESTS PASSED

# AES 原语单元测试（与软件参考模型黄金向量比对）
iverilog -g2001 -o sim/tb_aes.vvp \
    rtl/aes_round.v rtl/aes_gen1r.v rtl/aes_gen4r.v rtl/aes_hash1r.v \
    sim/tb_aes.v
vvp sim/tb_aes.vvp   # 输出 ALL TESTS PASSED

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

# 板级顶层单元测试（复位门控 + 34→33 位地址适配自校验）
iverilog -g2001 -DSIMULATION -o sim/tb_randomx_hbm_top.vvp \
    rtl/aes_round.v rtl/aes_gen1r.v rtl/aes_gen4r.v rtl/aes_hash1r.v \
    rtl/blake2b_core.v rtl/scratchpad_mem.v rtl/hbm_dataset_if.v \
    rtl/cache_hbm_if.v rtl/axi_arbiter.v \
    rtl/recip.v rtl/alu_int.v rtl/fpu_double.v rtl/superscalar_hash.v \
    rtl/dataset_gen.v rtl/prog_gen.v \
    rtl/argon2_fill.v rtl/randomx_vm.v rtl/randomx_top.v rtl/randomx_hbm_top.v \
    sim/tb_randomx_hbm_top.v
vvp sim/tb_randomx_hbm_top.vvp   # 输出 ALL TESTS PASSED

# Dataset item 生成单元测试（与黄金模型逐 item 比对）
iverilog -g2001 -DSIMULATION -o sim/tb_dataset_gen.vvp \
    rtl/recip.v rtl/alu_int.v rtl/superscalar_hash.v rtl/dataset_gen.v \
    sim/tb_dataset_gen.v
vvp sim/tb_dataset_gen.vvp   # 输出 ALL TESTS PASSED

# RandomX VM 单元测试（定向程序覆盖各指令类 + 主循环）
iverilog -g2001 -DSIMULATION -o sim/tb_randomx_vm.vvp \
    rtl/aes_round.v rtl/aes_hash1r.v rtl/scratchpad_mem.v \
    rtl/recip.v rtl/alu_int.v rtl/fpu_double.v rtl/randomx_vm.v \
    sim/tb_randomx_vm.v
vvp sim/tb_randomx_vm.vvp   # 输出 ALL CHECKS PASSED

# SuperscalarHash 单元测试（全指令集，比对软件模型）
iverilog -g2001 -o sim/tb_superscalar_hash.vvp \
    rtl/recip.v rtl/alu_int.v rtl/superscalar_hash.v sim/tb_superscalar_hash.v
vvp sim/tb_superscalar_hash.vvp   # 输出 PASS

# 整数执行单元单元测试（全部 19 条 opcode + IMUL_RCP 黄金向量）
iverilog -g2001 -o sim/tb_alu_int.vvp \
    rtl/recip.v rtl/alu_int.v sim/tb_alu_int.v
vvp sim/tb_alu_int.vvp   # 输出 ALL TESTS PASSED

# Scratchpad 单元测试（读写回环 + L1/L2/L3 掩码别名）
iverilog -g2001 -DSIMULATION -o sim/tb_scratchpad_mem.vvp \
    rtl/scratchpad_mem.v sim/tb_scratchpad_mem.v
vvp sim/tb_scratchpad_mem.vvp   # 输出 ALL TESTS PASSED

# 程序/entropy 生成单元测试（与独立的 aes_gen4r 黄金模型比对）
iverilog -g2001 -DSIMULATION -o sim/tb_prog_gen.vvp \
    rtl/aes_round.v rtl/aes_gen4r.v rtl/prog_gen.v sim/tb_prog_gen.v
vvp sim/tb_prog_gen.vvp   # 输出 ALL TESTS PASSED

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
  - `randomx_vm` 的循环次数从 2048 降为 4、最终折叠的 Scratchpad 字数降为 4096
    （同样由 `randomx_top.v` 的 parameter 覆盖）
- `sim/tb_randomx_top.v` 内含行为级 HBM AXI4 从设备模型：Cache 窗口
  （`0x0_C000_0000` 起 8 KiB）由内存支持，因此 CACHE_INIT 阶段会真正把
  Argon2d 块写进 HBM 模型；窗口外（尚未生成的 Dataset）读回全 0

---

## 单元测试状态

以下 testbench 均可用 `iverilog -g2001` 编译运行（命令见[仿真说明](#仿真说明iverilog)）：

| Testbench                  | 覆盖范围                                                | 结果 |
|----------------------------|---------------------------------------------------------|------|
| `sim/tb_aes.v`             | AES 原语：AESENC/AESDEC（含 last round）、AesGenerator1R 连续 3 块、AesGenerator4R 连续 2 块、AesHash1R 单块与 3 块流式吸收，全部与软件参考模型黄金向量比对 | PASS（11 项检查）|
| `sim/tb_blake2b.v`         | `"abc"`（外部 IV / `init` 两种）、空消息、200 字节两块链式哈希、`busy` 握手 | PASS（`ALL TESTS PASSED`）|
| `sim/tb_hbm_dataset_if.v`  | 行为级 AXI4 从设备模型：多事务流水、背压、错误注入、AXI 属性与基址检查 | PASS |
| `sim/tb_cache_hbm_if.v`    | 1 KiB 块 ↔ AXI4 突发：写/读回环、HBM 内字节序、随机背压、握手复用、SLVERR 粘滞位 | PASS（`ALL TESTS PASSED`）|
| `sim/tb_alu_int.v`         | 全部 19 条整数 opcode（含进位/借位、128 位有符号/无符号高位乘、移位与循环移位边界、ISTORE 的 L1/L2/L3 级别），以及 7 组 IMUL_RCP 黄金向量（含 2 的幂返回 0 的边界） | PASS（`ALL TESTS PASSED`）|
| `sim/tb_scratchpad_mem.v`  | 读写回环与 `rd_valid` 时序、字节地址→字索引截断、覆写、L1/L2/L3 掩码别名（0x4000 在 L1 下别名到 0x0000，在 L3 下互不干扰）、16 字顺序填充回读 | PASS（`ALL TESTS PASSED`）|
| `sim/tb_prog_gen.v`        | 与独立的 aes_gen4r 黄金模型比对 34 个块：16 个 cfg entropy 字与 256 条指令字的地址顺序与内容、`state_out` 链式末态、`busy`/`done` 握手 | PASS（`ALL TESTS PASSED`）|
| `sim/tb_superscalar_hash.v`| 全部 14 种 SuperscalarHash 指令（与软件模型逐寄存器比对），以及最后一个程序窗口（`prog_base`=3584、512 条指令）的取指终止边界 | PASS（269 周期）|
| `sim/tb_fpu_double.v`      | FADD/FSUB/FMUL/FDIV/FSQRT/FSCAL/FSWAP，含 4 种舍入模式、NaN/Inf/±0、非规格化、上溢/下溢与 `busy` 握手 | PASS（47 项检查）|
| `sim/tb_argon2_fill.v`     | Argon2d Cache 填充：与 Argon2 参考实现黄金向量逐块比对（m=8/t=3/43 字节 key，m=32/t=1/64 字节 key，m=6→8/t=5/100 字节 key） | PASS |
| `sim/tb_dataset_gen.v`     | Dataset item 生成：寄存器种子、8 次 cache 访问与地址寄存器链、SuperscalarHash 程序执行、写端口背压，与黄金模型逐 item 比对 | PASS（`ALL TESTS PASSED`）|
| `sim/tb_randomx_vm.v`      | VM 定向程序：IADD_RS/ISUB/IXOR/IROR/ISWAP/IMUL/INEG/ISTORE/IADD_M/CBRANCH（含跳转）/FADD_R/FSCAL_R/FMUL_R/FSQRT_R/FSWAP_R/CFROUND，主循环的 Scratchpad 载入/回写，以及 `getFinalResult` 把 AesHash1R 摘要写回 a 寄存器与 `regfile_out` 的字节布局 | PASS（`ALL CHECKS PASSED`）|
| `sim/tb_randomx_top.v`     | 顶层集成冒烟测试：寄存器写种子 → start → 轮询 done → 读回哈希；含行为级 HBM 模型，检查 Cache 块确实写入 HBM | 运行完成（哈希值尚未自校验）|
| `sim/tb_randomx_hbm_top.v` | 板级顶层：`hbm_init_done` 拉低期间无任何 AXI 地址握手、释放后正常完成、Argon2d 块经 33 位地址总线写入 Cache 窗口、`hbm_addr_err` 保持为 0 | PASS（`ALL TESTS PASSED`）|

> `make test` 会逐个运行上述 testbench，并在输出中出现 `FAIL` / `ERROR` 时返回非 0 退出码。

### 验证方面的已知缺口
- `tb_randomx_top.v` 尚非自校验：缺少与参考实现的期望哈希比对。
- 缺少与官方 [tevador/RandomX](https://github.com/tevador/RandomX) 参考实现的
  端到端测试向量对拍（黄金模型对比）。这是目前唯一还没有黄金向量背书的环节：
  各子模块均已单独对拍，但完整 `calculate_hash` 结果尚未逐位验证。

---

## TODO / 实现状态

| 模块              | 状态       | 主要 TODO                                  |
|------------------|------------|-------------------------------------------|
| randomx_top.v    | **已实现** | 完整 AXI-Lite 握手（8 程序链已实现）        |
| randomx_hbm_top.v| **已实现** | 板级引脚约束（依赖具体板卡）                 |
| blake2b_core.v   | **已实现** | 无（12 轮压缩、init/busy 接口，24 周期/块） |
| aes_round.v      | **已实现** | 无（AESENC / AESDEC 两个方向）              |
| aes_gen1r/4r.v   | **已实现** | 无（规范轮密钥 + enc/dec lane 映射）         |
| aes_hash1r.v     | **已实现** | 无（流式吸收 + xkey0/xkey1 扩散轮）          |
| scratchpad_mem.v | **已实现** | 无（URAM 推断、L1/L2/L3 掩码）             |
| hbm_dataset_if.v | **已实现** | 无（写接口由 `dataset_gen` 驱动，HBM IP 已由 `randomx_hbm_top` 接通）|
| cache_hbm_if.v   | **已实现** | 无（HBM IP 经 `randomx_hbm_top` + 协议转换器接通）|
| axi_arbiter.v    | **已实现** | 无（读/写通路独立 round-robin 仲裁）         |
| alu_int.v        | **已实现** | 无（19 条整数指令，IMUL_RCP 复用 `recip.v`）|
| recip.v          | **已实现** | 无（共享倒数单元，与 `reciprocal.c` 位级一致）|
| fpu_double.v     | **已实现** | 流水化以提升 Fmax（当前加/乘为单周期组合路径）|
| superscalar_hash.v| **已实现** | 超标量调度（并行执行端口，性能优化）        |
| randomx_vm.v     | **已实现** | 无（29 条 ISA、主循环、逐 64 字节块 AesHash1R 最终哈希）|
| argon2_fill.v    | **已实现** | 无（cache 块经 cache_hbm_if 存入 HBM）      |
| dataset_gen.v    | **已实现** | SuperscalarHash 程序仍由主机加载（spec §6.1 生成器未实现）|
| prog_gen.v       | **已实现** | 无                                          |

---

## 完善路线图

### 当前进度概览

| 阶段            | 状态 | 说明 |
|-----------------|------|------|
| Cache 初始化    | ✅   | `argon2_fill` + `cache_hbm_if` + `axi_arbiter`，与参考实现黄金向量对拍通过 |
| Dataset 生成    | 🟨   | `dataset_gen` 完成 8 次 cache 访问 + SuperscalarHash 并写入 HBM；8 个超标量程序仍需主机加载 |
| Scratchpad 填充 | ✅   | 顶层 `SP_FILL` 阶段用 `aes_gen1r` 填满 Scratchpad，轮密钥与 enc/dec 映射已与参考实现对拍 |
| 程序/entropy 生成| ✅   | `prog_gen` 用 `aes_gen4r` 生成 16 个 entropy 字 + 256 条指令，轮密钥已对拍 |
| VM 执行         | ✅   | 29 条 ISA（含 IMUL_RCP）、主循环、Scratchpad 读写，以及 8 程序链（Blake2b-512(RegisterFile) 播种下一程序）|
| 最终哈希        | ✅   | 整个 Scratchpad 逐 64 字节块经 AesHash1R 压缩写回 a 寄存器，再对 256 字节寄存器堆做 Blake2b-256 收尾 |
| 板级/后端       | 🟨   | HBM IP、协议转换器、时钟与复位门控已就位；引脚约束与 300 MHz 收敛待办 |
| 端到端对拍      | 🟨   | 各子模块均已与参考实现黄金向量对拍；完整 `calculate_hash` 结果尚未逐位验证 |

图例：✅ 已完成 ／ 🟨 部分完成 ／ ⬜ 未开始

### 待办事项（优先级从高到低）

1. **SuperscalarHash 程序生成（spec §6.1）**
   - `dataset_gen` 的 8 个程序目前经寄存器接口由主机写入
     （0x8C–0x98），片上 `generateSuperscalar` 未实现。
   - 若要完全脱离主机，需实现 Blake2bGenerator + 调度模型。
   - 这是目前唯一仍依赖主机参与的算法环节。
2. **端到端对拍**
   - `sim/tb_randomx_top.v` 改为自校验：与 [tevador/RandomX](https://github.com/tevador/RandomX)
     参考实现做端到端测试向量对拍。
   - 依赖上一项：只要超标量程序仍由主机加载，端到端向量就必须连同这 8 个
     程序一起从参考实现导出。
3. **时序与性能**
   - `fpu_double` 流水化（加/乘目前是单周期组合路径），这是 300 MHz
     收敛的主要障碍。
   - `superscalar_hash` 的超标量并行执行端口（性能优化，非正确性阻塞项）。
   - `recip.v` 为逐位除法（64+bsr 周期），IMUL_RCP 密集的程序可考虑
     用查表/牛顿迭代缩短延迟。
4. **板级收尾**
   - `vivado/constraints.xdc` 中的 `PACKAGE_PIN` 占位符按具体板卡填实。
   - pblock 的 `CLOCKREGION` 范围按实际 SLR 划分确认。

### 已完成（归档）

- **IMUL_RCP 与共享倒数单元** — 新增 `rtl/recip.v`（规范 §5.5.11 /
  `reciprocal.c` 的逐位恢复余数除法），`superscalar_hash` 改为复用该单元，
  `alu_int` 的 `OP_IMUL_RCP` 由占位改为真正的多周期实现（新增 `busy` 输出）；
  配套 `sim/tb_alu_int.v` 覆盖全部 19 条 opcode 与 7 组倒数黄金向量。
- **规范化的最终哈希与 8 程序链** — `randomx_vm` 新增 `do_final` /
  `regfile_out` 端口，最终哈希由 Scratchpad XOR 折叠改为逐 64 字节块的
  AesHash1R 流式压缩并把摘要写回 a 寄存器；`randomx_top` 增加 `CHAIN_HASH`
  阶段，用 Blake2b-512(RegisterFile) 为下一个程序播种，实现
  RANDOMX_PROGRAM_COUNT = 8 的程序链，最终对 256 字节寄存器堆做 Blake2b-256。
- **验证补齐** — 新增 `sim/tb_alu_int.v`、`sim/tb_scratchpad_mem.v`、
  `sim/tb_prog_gen.v`，`make test` 覆盖 14 个 testbench，`make lint` 无告警。
- **许可** — 添加与上游 RandomX 兼容的 BSD-3-Clause `LICENSE`。

- **AES 原语** — `aes_round.v` 补齐 AESDEC（逆 S-box / InvShiftRows /
  InvMixColumns），`aes_gen1r` / `aes_gen4r` / `aes_hash1r` 换用 RandomX
  参考实现的轮密钥常量与 enc/dec lane 映射，`aes_hash1r` 按 `hashAes1Rx4`
  重写为「固定初值 + 消息作轮密钥吸收 + xkey0/xkey1 扩散」的流式结构；
  配套黄金向量 testbench `sim/tb_aes.v`。
- **顶层数据通路** — `randomx_top.v` 的主 FSM 打通
  CACHE_INIT → DS_GEN → SP_FILL → PROG_GEN → VM_RUN → FINAL_HASH
  （后续又扩展为含 CHAIN_HASH 的 8 程序链，见下文）：
  新增 `dataset_gen.v`（spec §7.3 的 Dataset item 生成，驱动
  `hbm_dataset_if` 写通道）与 `prog_gen.v`（AesGenerator4R 生成程序与
  entropy 并写入 VM），Scratchpad 由 `aes_gen1r` 填充，最终哈希接
  Blake2b-256 收尾；`superscalar_hash` 新增 `prog_base` 以容纳 8 个程序。
- **Cache 存储通路** — `cache_hbm_if.v`（1 KiB 块 ↔ 32 拍 AXI4 突发）与
  `axi_arbiter.v`（cache/dataset 共享 HBM 端口），`randomx_top.v` 中的
  占位连接已去除。
- **Argon2d Cache 填充** — `argon2_fill.v` 与 Argon2 参考实现黄金向量
  逐块比对通过（`sim/tb_argon2_fill.v`）。
- **`randomx_vm.v` 主体** — 全部指令译码、程序配置（entropy）初始化、
  规范 §4.6.2 主循环、Scratchpad 载入/回写，配套自校验 testbench
  `sim/tb_randomx_vm.v`。
- **Blake2b / FPU / SuperscalarHash** — 均已实现并有自校验 testbench。
- **后端与板级顶层** — `randomx_hbm_top.v`（复位门控 + 34→33 位地址适配 +
  可选 IP 例化）、`build.tcl` 的 `-tclargs hbm` 构建（HBM IP + AXI4→AXI3
  协议转换器）、`constraints.tcl` 中填实的 HBM 时钟/时钟分组
  （并移除了掩盖违例的 `m_axi_*` false path）以及 SLR pblock 模板。

---

## 许可

本项目为开源硬件框架骨架，用于 RandomX 算法的 FPGA 研究目的。

本仓库以 **BSD 3-Clause License** 发布，详见 [`LICENSE`](LICENSE)。

RandomX 算法版权归原始作者所有（见 [tevador/RandomX](https://github.com/tevador/RandomX)，
同为 BSD-3-Clause）；本项目是该算法的独立硬件（Verilog）实现。
