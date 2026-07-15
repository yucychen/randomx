# =============================================================================
# build.tcl — Vivado Project Build Script
# RandomX FPGA Framework — Xilinx Virtex UltraScale+ XCVU33P
#
# Usage (Vivado Tcl console or batch mode):
#   vivado -mode batch -source build.tcl
#   OR open Vivado GUI → Tcl Console → source vivado/build.tcl
#
# What this script does:
#   1. Creates a new Vivado project for part xcvu33p-fsvh2104-2L-e
#   2. Adds all RTL sources (Verilog-2001)
#   3. Adds simulation sources
#   4. Sets the top module to randomx_top
#   5. Adds constraints (clock, HBM placeholder)
#   6. Launches synthesis (out-of-context is acceptable for timing closure)
#
# Note: Full implementation (place & route) is NOT run automatically because
# HBM IP connections require manual configuration of the Xilinx HBM IP.
# See README.md for next steps after synthesis.
# =============================================================================

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
set project_name "randomx_xcvu33p"
set project_dir  [file normalize "[file dirname [info script]]/../vivado_work"]
set part_name    "xcvu33p-fsvh2104-2L-e"
set top_module   "randomx_top"

# RTL source files (relative to project build.tcl location)
set rtl_dir [file normalize "[file dirname [info script]]/../rtl"]
set sim_dir [file normalize "[file dirname [info script]]/../sim"]
set xdc_dir [file normalize "[file dirname [info script]]"]

set rtl_files [list \
    "${rtl_dir}/aes_round.v"       \
    "${rtl_dir}/aes_gen1r.v"       \
    "${rtl_dir}/aes_gen4r.v"       \
    "${rtl_dir}/aes_hash1r.v"      \
    "${rtl_dir}/blake2b_core.v"    \
    "${rtl_dir}/scratchpad_mem.v"  \
    "${rtl_dir}/hbm_dataset_if.v"  \
    "${rtl_dir}/alu_int.v"         \
    "${rtl_dir}/fpu_double.v"      \
    "${rtl_dir}/superscalar_hash.v"\
    "${rtl_dir}/argon2_fill.v"     \
    "${rtl_dir}/randomx_vm.v"      \
    "${rtl_dir}/randomx_top.v"     \
]

set sim_files [list \
    "${sim_dir}/tb_randomx_top.v"  \
]

set xdc_files [list \
    "${xdc_dir}/constraints.xdc"   \
]

# ---------------------------------------------------------------------------
# Create project
# ---------------------------------------------------------------------------
puts "INFO: Creating project '${project_name}' for part '${part_name}'"

file mkdir ${project_dir}
create_project ${project_name} ${project_dir} -part ${part_name} -force

# Set project properties
set_property target_language   Verilog    [current_project]
set_property simulator_language Verilog   [current_project]
set_property default_lib       work       [current_project]

# ---------------------------------------------------------------------------
# Add RTL sources (synthesis)
# ---------------------------------------------------------------------------
puts "INFO: Adding RTL source files..."
add_files -fileset sources_1 ${rtl_files}

# Set all sources to Verilog-2001
foreach src ${rtl_files} {
    set_property file_type {Verilog} [get_files $src]
    # Vivado uses SystemVerilog by default for .v files in some versions;
    # explicitly force Verilog 2001 compatibility
}

# Set top module
set_property top ${top_module} [current_fileset]

# ---------------------------------------------------------------------------
# Add simulation sources
# ---------------------------------------------------------------------------
puts "INFO: Adding simulation files..."
add_files -fileset sim_1 ${sim_files}
set_property top tb_randomx_top [get_filesets sim_1]

# Set SIMULATION define for simulation fileset
set_property verilog_define {SIMULATION=1} [get_filesets sim_1]

# ---------------------------------------------------------------------------
# Add constraints
# ---------------------------------------------------------------------------
puts "INFO: Adding constraints..."
add_files -fileset constrs_1 ${xdc_files}
set_property target_constrs_file [lindex ${xdc_files} 0] [current_fileset -constrset]

# ---------------------------------------------------------------------------
# Synthesis settings
# ---------------------------------------------------------------------------
set_property strategy "Vivado Synthesis Defaults" [get_runs synth_1]

# Flattening — moderate (preserves hierarchy for debug)
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]

# Retiming — disabled (skeleton, not timing-closed)
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING       0       [get_runs synth_1]

# ---------------------------------------------------------------------------
# Run synthesis (elaboration checks the design for errors)
# Comment out the launch_runs line to skip actual synthesis and only elaborate.
# ---------------------------------------------------------------------------
puts "INFO: Running synthesis elaboration check..."

# Elaboration check (fast, no full synthesis)
synth_design -rtl -rtl_skip_mlo -name rtl_1

puts ""
puts "============================================================"
puts " Elaboration complete."
puts " To run full synthesis, execute in Tcl console:"
puts "   launch_runs synth_1 -jobs 8"
puts "   wait_on_run synth_1"
puts " To open the elaborated schematic:"
puts "   show_schematic [get_cells -hierarchical *]"
puts "============================================================"

# Uncomment to launch full synthesis automatically:
# launch_runs synth_1 -jobs 8
# wait_on_run synth_1
# if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
#     error "Synthesis failed. See [get_property DIRECTORY [get_runs synth_1]]/runme.log"
# }
# open_run synth_1 -name synth_1
# report_utilization -file ${project_dir}/utilization_synth.rpt
# report_timing_summary -file ${project_dir}/timing_synth.rpt
