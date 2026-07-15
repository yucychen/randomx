# =============================================================================
# constraints.xdc — Xilinx Design Constraints
# RandomX FPGA Framework — Xilinx Virtex UltraScale+ XCVU33P
#
# Target board: xcvu33p-fsvh2104-2L-e
# (Typically: Alveo U280 or custom board with XCVU33P)
#
# Contents:
#   1. Primary clock constraint (300 MHz system clock)
#   2. HBM reference clock placeholder
#   3. I/O standard placeholders
#   4. False path declarations for reset
#
# NOTE: This file is a skeleton. Actual pin assignments depend on the
# physical board schematic. Update PACKAGE_PIN values for your board.
# =============================================================================

# ---------------------------------------------------------------------------
# 1. Primary System Clock — 300 MHz
# ---------------------------------------------------------------------------
# TODO: Replace PIN_NAME with the actual FPGA pin from your board schematic.
# For Alveo U280: the 300 MHz system clock is typically on a GTH reference
# or on-board oscillator connected to a global clock buffer.

# create_clock -name sys_clk -period 3.333 [get_ports clk]
# set_property PACKAGE_PIN <PIN_NAME>    [get_ports clk]
# set_property IOSTANDARD  LVCMOS18      [get_ports clk]

# Placeholder: define clock with required period (3.333 ns = 300 MHz)
# Uncomment and set correct port/pin when physical board is known.
create_clock -name sys_clk_300mhz -period 3.333 [get_ports clk]

# ---------------------------------------------------------------------------
# 2. Reset Signal
# ---------------------------------------------------------------------------
# set_property PACKAGE_PIN <PIN_NAME>    [get_ports rst_n]
# set_property IOSTANDARD  LVCMOS18      [get_ports rst_n]

# False path on reset (asynchronous assertion, synchronous deassertion in RTL)
set_false_path -from [get_ports rst_n]

# ---------------------------------------------------------------------------
# 3. AXI-Lite Control Interface Pins (optional, board-dependent)
# ---------------------------------------------------------------------------
# These signals would connect to a host CPU (e.g., via PCIe + AXI bridge).
# For simulation-only use, leave unassigned.
# set_property PACKAGE_PIN <PIN_NAME> [get_ports {reg_wr_en}]
# ... etc.

# ---------------------------------------------------------------------------
# 4. HBM2 Interface
# ---------------------------------------------------------------------------
# The XCVU33P HBM2 is connected internally (not via I/O pins) through the
# hard HBM controller. AXI master ports connect to HBM AXI slave ports
# within the FPGA fabric using the Vivado HBM IP.
#
# HBM Reference Clock (typically 100 MHz or 200 MHz from oscillator):
# TODO: Add create_clock for HBM reference clock when HBM IP is connected.
# create_clock -name hbm_ref_clk -period 10.000 [get_ports hbm_ref_clk_p]

# HBM APB Clock (100 MHz for configuration):
# create_clock -name hbm_apb_clk -period 10.000 [get_ports hbm_apb_clk]

# Placeholder constraints to suppress timing errors on unconnected AXI ports:
set_false_path -to   [get_ports m_axi_araddr*]
set_false_path -to   [get_ports m_axi_arlen*]
set_false_path -to   [get_ports m_axi_arsize*]
set_false_path -to   [get_ports m_axi_arburst*]
set_false_path -to   [get_ports m_axi_arvalid]
set_false_path -from [get_ports m_axi_arready]
set_false_path -from [get_ports m_axi_rdata*]
set_false_path -from [get_ports m_axi_rresp*]
set_false_path -from [get_ports m_axi_rlast]
set_false_path -from [get_ports m_axi_rvalid]
set_false_path -to   [get_ports m_axi_rready]

# ---------------------------------------------------------------------------
# 5. URAM Configuration
# ---------------------------------------------------------------------------
# The scratchpad URAM is inferred via (* ram_style = "ultra" *) attribute.
# No explicit LOC constraints needed; Vivado will place URAMs automatically.
# If you want to constrain URAM placement to a specific SLR (Super Logic Region):
# set_property LOC URAM288_X0Y0 [get_cells u_scratchpad/scratchpad_reg*]

# ---------------------------------------------------------------------------
# 6. Timing exceptions for skeleton stubs
# ---------------------------------------------------------------------------
# The fpu_double module has pass-through (non-functional) paths for FDIV/FSQRT.
# Mark these as false paths to prevent optimization pressure on stubs.
# (These will need to be removed when real FP units are implemented.)
# set_false_path -through [get_cells u_vm/u_fpu*]
