# =============================================================================
# constraints.xdc — Xilinx Design Constraints
# RandomX FPGA Framework — Xilinx Virtex UltraScale+ XCVU33P
#
# Target part:  xcvu33p-fsvh2104-2L-e
# Target board: TODO — the PACKAGE_PIN values below still depend on the carrier
#               schematic. Everything that does *not* depend on the board
#               (clock periods, clock groups, SLR floorplanning, timing
#               exceptions) is filled in.
#
# This file supports both build flavours produced by vivado/build.tcl:
#   * top = randomx_top      (hbm_enable = 0) — ports clk / rst_n / m_axi_*
#   * top = randomx_hbm_top  (hbm_enable = 1) — ports sys_clk / sys_rst_n; with
#                                               HBM_IP there is no external AXI
# Constraints that apply to only one flavour are guarded by port existence
# checks, so the same file works unmodified for both.
#
# Contents:
#   1. Primary system clock (300 MHz)
#   2. Reset
#   3. Control register interface
#   4. HBM2 interface (clocks, clock groups, address map)
#   5. SLR floorplanning (pblocks) — the main lever for 300 MHz closure
#   6. URAM configuration
#   7. Timing exceptions for skeleton stubs
# =============================================================================

# ---------------------------------------------------------------------------
# 0. Resolve the system clock / reset port for either top module
# ---------------------------------------------------------------------------
set sys_clk_port [get_ports -quiet sys_clk]
if {[llength $sys_clk_port] == 0} {
    set sys_clk_port [get_ports -quiet clk]
}
set sys_rst_port [get_ports -quiet sys_rst_n]
if {[llength $sys_rst_port] == 0} {
    set sys_rst_port [get_ports -quiet rst_n]
}

# ---------------------------------------------------------------------------
# 1. Primary System Clock — 300 MHz (3.333 ns)
# ---------------------------------------------------------------------------
# TODO(board): set PACKAGE_PIN / IOSTANDARD from the board schematic. On an
# Alveo-class card the 300 MHz clock arrives as a differential pair and is
# buffered by IBUFDS + MMCM rather than driven straight into the fabric.
# set_property PACKAGE_PIN <PIN_NAME> $sys_clk_port
# set_property IOSTANDARD  LVCMOS18   $sys_clk_port

if {[llength $sys_clk_port] > 0} {
    create_clock -name sys_clk_300mhz -period 3.333 $sys_clk_port
}

# ---------------------------------------------------------------------------
# 2. Reset
# ---------------------------------------------------------------------------
# TODO(board): set PACKAGE_PIN / IOSTANDARD.
# set_property PACKAGE_PIN <PIN_NAME> $sys_rst_port
# set_property IOSTANDARD  LVCMOS18   $sys_rst_port

# Asynchronous assertion, synchronous release. randomx_hbm_top contains the
# reset synchroniser; randomx_top expects an already-synchronised reset.
if {[llength $sys_rst_port] > 0} {
    set_false_path -from $sys_rst_port
}

# ---------------------------------------------------------------------------
# 3. Control / status register interface
# ---------------------------------------------------------------------------
# The reg_* interface is meant to be driven by a host bridge (PCIe/XDMA or a
# soft CPU), in which case it needs no pin constraints. Assign pins here only
# if the interface is brought out of the device directly.
# set_property PACKAGE_PIN <PIN_NAME> [get_ports {reg_wr_en}]

# ---------------------------------------------------------------------------
# 4. HBM2 Interface
# ---------------------------------------------------------------------------
# XCVU33P has one 4 GB HBM2 stack wired internally to the hard HBM controller
# (no user I/O pins). The design address map is:
#
#   Dataset  0x0_0000_0000   ~2.08 GiB   hbm_dataset_if.v   (2-beat bursts)
#   Cache    0x0_C000_0000    256  MiB   cache_hbm_if.v     (32-beat bursts)
#
# Both regions reach far outside the 256 MB window a single HBM pseudo-channel
# covers on its own, so the HBM IP MUST be generated with the internal AXI
# switch (Global Addressing) enabled — see vivado/build.tcl. Without it the
# cache accesses at 0xC000_0000 come back as DECERR and the design latches the
# sticky error bit in status register 0x44 bit 1.
#
# NOTE: earlier revisions of this file declared set_false_path on every m_axi_*
# port to silence the unconnected AXI boundary. Those exceptions have been
# removed on purpose — with HBM attached they would mask real violations on the
# highest-risk paths of the design. The vendor-neutral build instead gets an
# explicit I/O budget below so violations stay visible.

# HBM reference clock (100 MHz) — present only in the HBM build.
set hbm_ref_port [get_ports -quiet hbm_ref_clk]
if {[llength $hbm_ref_port] > 0} {
    create_clock -name hbm_ref_clk -period 10.000 $hbm_ref_port
    # TODO(board): set_property PACKAGE_PIN <PIN_NAME> $hbm_ref_port
}

# HBM APB configuration clock (100 MHz) — present only in the HBM build.
set hbm_apb_port [get_ports -quiet hbm_apb_clk]
if {[llength $hbm_apb_port] > 0} {
    create_clock -name hbm_apb_clk -period 10.000 $hbm_apb_port
    # TODO(board): set_property PACKAGE_PIN <PIN_NAME> $hbm_apb_port
}

# The reference and APB clocks are asynchronous to the core clock. The HBM AXI
# port is deliberately clocked by sys_clk (build.tcl: hbm_axi_clk_mhz = 300) so
# that no AXI Clock Converter and no extra CDC constraints are needed. If the
# AXI port is later moved to its own clock (e.g. 450 MHz), add that clock to
# the asynchronous group below as well.
set async_groups {}
foreach ck {hbm_ref_clk hbm_apb_clk} {
    if {[llength [get_clocks -quiet $ck]] > 0} {
        lappend async_groups -group [get_clocks $ck]
    }
}
if {[llength $async_groups] > 0 && [llength [get_clocks -quiet sys_clk_300mhz]] > 0} {
    set_clock_groups -asynchronous -group [get_clocks sys_clk_300mhz] {*}$async_groups
}

# Vendor-neutral build only: the AXI4 master is still a device boundary, so
# give it an I/O budget rather than a false path.
set m_axi_ports [get_ports -quiet m_axi_*]
if {[llength $m_axi_ports] > 0 && [llength [get_clocks -quiet sys_clk_300mhz]] > 0} {
    set m_axi_out [get_ports -quiet -filter {DIRECTION == OUT} $m_axi_ports]
    set m_axi_in  [get_ports -quiet -filter {DIRECTION == IN}  $m_axi_ports]
    if {[llength $m_axi_out] > 0} {
        set_output_delay -clock sys_clk_300mhz 1.000 $m_axi_out
    }
    if {[llength $m_axi_in] > 0} {
        set_input_delay -clock sys_clk_300mhz 1.000 $m_axi_in
    }
}

# ---------------------------------------------------------------------------
# 5. SLR floorplanning — the main lever for 300 MHz closure
# ---------------------------------------------------------------------------
# The HBM controller sits in the bottom SLR. Every SLR crossing costs a Laguna
# (SLL) hop, which is the dominant cause of timing failures on UltraScale+.
#
# Critical detail: argon2_fill drives cache_hbm_if through an 8192-bit block
# bus (wr_data / rd_data). Letting that bus cross an SLR burns a very large
# number of SLLs, so the Argon2 + cache + arbiter cluster must stay in the same
# SLR as the HBM controller.
#
# The CLOCKREGION ranges are device specific, so the pblock is disabled by
# default. Confirm the bottom-SLR clock regions for xcvu33p in the Device view,
# adjust the resize_pblock line, then set hbm_pblock_enable to 1.
set hbm_pblock_enable 0

if {$hbm_pblock_enable} {
    create_pblock pblock_hbm_cluster
    # TODO(device): confirm the bottom-SLR clock regions for xcvu33p.
    resize_pblock pblock_hbm_cluster -add {CLOCKREGION_X0Y0:CLOCKREGION_X7Y3}

    set hbm_cluster_cells [get_cells -quiet -hierarchical -filter \
        {NAME =~ *u_argon2* || NAME =~ *u_cache_hbm* || \
         NAME =~ *u_hbm_dataset* || NAME =~ *u_axi_arb* || \
         NAME =~ *u_axi4_to_axi3*}]
    if {[llength $hbm_cluster_cells] > 0} {
        add_cells_to_pblock pblock_hbm_cluster $hbm_cluster_cells
    }
    # Keep the floorplan a guideline, not a hard fence, until timing closes.
    set_property CONTAIN_ROUTING false [get_pblocks pblock_hbm_cluster]
}

# ---------------------------------------------------------------------------
# 6. URAM Configuration
# ---------------------------------------------------------------------------
# The scratchpad URAM is inferred via (* ram_style = "ultra" *); no explicit
# LOC constraints are needed. The scratchpad does not talk to HBM, so it may
# live in a different SLR from the pblock above — but any signal that then
# crosses the SLR boundary must be registered on both sides.

# ---------------------------------------------------------------------------
# 7. Timing exceptions for skeleton stubs
# ---------------------------------------------------------------------------
# fpu_double currently implements FADD/FMUL as single-cycle combinational paths
# (README TODO: "pipeline to raise Fmax"). This is the single largest obstacle
# to 300 MHz and is deliberately NOT waived here: a false path would hide the
# problem instead of fixing it. Expect these paths to dominate the first
# report_timing_summary.
# set_false_path -through [get_cells u_vm/u_fpu*]
