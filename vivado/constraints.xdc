# =============================================================================
# constraints.xdc — Xilinx Design Constraints (static / XDC-only)
# RandomX FPGA Framework — Xilinx Virtex UltraScale+ XCVU33P
#
# Target part:  xcvu33p-fsvh2104-2L-e
# Target board: TODO — the PACKAGE_PIN values still depend on the carrier
#               schematic.
#
# IMPORTANT — DO NOT PUT Tcl CONTROL FLOW IN THIS FILE
# ----------------------------------------------------
# Vivado reads .xdc files in constraint mode, which accepts only the XDC
# command subset. Commands such as `if`, `foreach`, `while` or `proc` are
# rejected with
#   CRITICAL WARNING: [Designutils 20-1307] Command 'if' is not supported in
#   the xdc constraint file.
# and the constraints they guard are silently dropped, leaving the design
# unconstrained.
#
# All constraints that need to adapt to the build flavour (randomx_top vs
# randomx_hbm_top) therefore live in vivado/constraints.tcl, which build.tcl
# adds to the same constraints fileset as a Tcl script:
#   * primary 300 MHz system clock (clk / sys_clk)
#   * reset false path (rst_n / sys_rst_n)
#   * HBM reference + APB clocks and the asynchronous clock groups
#   * AXI boundary I/O delays for the vendor-neutral build
#   * the SLR pblock template
#
# This file stays the project's target constraints file: constraints written
# from the GUI (pin assignments, I/O standards, hand-added exceptions) land
# here. Only plain, unconditional XDC commands belong here.
#
# Board pin-out placeholders (fill in from the schematic, then uncomment):
#   set_property PACKAGE_PIN <PIN_NAME> [get_ports sys_clk]
#   set_property IOSTANDARD  LVCMOS18   [get_ports sys_clk]
#   set_property PACKAGE_PIN <PIN_NAME> [get_ports sys_rst_n]
#   set_property IOSTANDARD  LVCMOS18   [get_ports sys_rst_n]
#
# The control/status register interface (reg_*) is meant to be driven by a host
# bridge (PCIe/XDMA or a soft CPU) and needs no pin constraints unless it is
# brought out of the device directly.
#
# URAM: the scratchpad is inferred via (* ram_style = "ultra" *); no explicit
# LOC constraints are needed. The scratchpad does not talk to HBM, so it may
# live in a different SLR from the HBM pblock — but any signal that then
# crosses the SLR boundary must be registered on both sides.
#
# Timing exceptions for skeleton stubs: fpu_double currently implements
# FADD/FMUL as single-cycle combinational paths (README TODO: "pipeline to
# raise Fmax"). This is the single largest obstacle to 300 MHz and is
# deliberately NOT waived here: a false path would hide the problem instead of
# fixing it. Expect these paths to dominate the first report_timing_summary.
#   set_false_path -through [get_cells u_vm/u_fpu*]
# =============================================================================
