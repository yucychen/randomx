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
#   4. Optionally creates the HBM IP + AXI Protocol Converter (hbm_enable)
#   5. Sets the top module (randomx_hbm_top with HBM, randomx_top without)
#   6. Adds constraints (clocks, HBM, SLR pblocks)
#   7. Launches synthesis (out-of-context is acceptable for timing closure)
#
# HBM build (set hbm_enable to 1 below, or pass -tclargs hbm):
#   * Creates hbm_0 with a single AXI port and the internal AXI switch
#     (Global Addressing) enabled. Global Addressing is mandatory: the design
#     places the Dataset at 0x0_0000_0000 (~2.08 GiB) and the Cache at
#     0x0_C000_0000 (256 MiB), both far outside the 256 MB window a single
#     pseudo-channel covers on its own.
#   * Creates axi_protocol_converter_0 (AXI4 -> AXI3) because cache_hbm_if
#     issues 32-beat bursts while the HBM AXI slave port is AXI3-style with a
#     4-bit AWLEN/ARLEN (16 beats maximum).
#   * Defines the HBM_IP Verilog macro so rtl/randomx_hbm_top.v instantiates
#     both IPs instead of exposing the AXI port at the boundary.
#
# Implementation (place & route) requires an HBM IP licence; synthesis does not.
# =============================================================================

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
set project_name "randomx_xcvu33p"
set project_dir  [file normalize "[file dirname [info script]]/../vivado_work"]
set part_name    "xcvu33p-fsvh2104-2L-e"

# ---------------------------------------------------------------------------
# HBM build switch
#   0 — vendor-neutral build: top = randomx_top, the AXI4 master stays at the
#       boundary (fast elaboration / synthesis checks, no IP, no licence).
#   1 — board build: top = randomx_hbm_top, HBM IP + protocol converter are
#       created and connected. Enable with: vivado -mode batch -source \
#       vivado/build.tcl -tclargs hbm
# ---------------------------------------------------------------------------
set hbm_enable 0
if {[llength $argv] > 0 && [lsearch -exact $argv "hbm"] >= 0} {
    set hbm_enable 1
}

# AXI slave address width of the HBM stack (33 bits = 4 GB).
set hbm_axi_addr_width 33
# HBM AXI port clock. Keep it equal to the 300 MHz core clock for the first
# bring-up so that no AXI Clock Converter and no CDC constraints are needed.
set hbm_axi_clk_mhz 300

if {$hbm_enable} {
    set top_module "randomx_hbm_top"
} else {
    set top_module "randomx_top"
}

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
    "${rtl_dir}/cache_hbm_if.v"    \
    "${rtl_dir}/axi_arbiter.v"     \
    "${rtl_dir}/alu_int.v"         \
    "${rtl_dir}/fpu_double.v"      \
    "${rtl_dir}/superscalar_hash.v"\
    "${rtl_dir}/argon2_fill.v"     \
    "${rtl_dir}/randomx_vm.v"      \
    "${rtl_dir}/randomx_top.v"     \
]

# Board-level top (wraps randomx_top for the HBM IP). Always added as a source
# so that it is elaborated/checked; it only becomes the top when hbm_enable=1.
lappend rtl_files "${rtl_dir}/randomx_hbm_top.v"

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
# HBM IP + AXI Protocol Converter
# ---------------------------------------------------------------------------
if {$hbm_enable} {
    puts "INFO: Creating HBM IP and AXI protocol converter..."

    # The HBM_IP macro switches rtl/randomx_hbm_top.v from "expose the AXI
    # port" to "instantiate hbm_0 + axi_protocol_converter_0".
    set_property verilog_define {HBM_IP=1} [get_filesets sources_1]

    # Sets only the CONFIG.* parameters that the installed IP version actually
    # exposes. Parameters that do not exist are skipped with a warning instead
    # of aborting the build (Vivado applies -dict atomically, so a single
    # unknown parameter would otherwise discard the whole configuration).
    proc apply_ip_config {ip cfg_dict} {
        set supported [list_property $ip]
        set applied   [list]
        foreach {param value} $cfg_dict {
            if {[lsearch -exact $supported $param] >= 0} {
                lappend applied $param $value
            } else {
                puts "WARNING: parameter '$param' is not supported by this Vivado/IP version — skipped."
            }
        }
        if {[llength $applied]} {
            set_property -dict $applied $ip
        }
    }

    # -- AXI4 (32-beat bursts from cache_hbm_if) -> AXI3 (16-beat maximum) --
    create_ip -name axi_protocol_converter -vendor xilinx.com -library ip \
              -module_name axi_protocol_converter_0
    apply_ip_config [get_ips axi_protocol_converter_0] [list \
        CONFIG.ADDR_WIDTH        ${hbm_axi_addr_width} \
        CONFIG.DATA_WIDTH        256                   \
        CONFIG.ID_WIDTH          1                     \
        CONFIG.SI_PROTOCOL       AXI4                  \
        CONFIG.MI_PROTOCOL       AXI3                  \
        CONFIG.TRANSLATION_MODE  2                     \
    ]

    # -- HBM controller --------------------------------------------------
    # Stack 0 only (XCVU33P has a single 4 GB HBM2 stack), one AXI port
    # enabled, internal switch ON so that the single port can reach the whole
    # stack (Global Addressing — see the header comment).
    #
    # NOTE: the exact set of USER_* parameters exposed by the HBM IP differs
    # between Vivado releases (for example USER_HBM_REF_CLK_XTAL_0 only exists
    # in some versions). Setting a parameter the installed IP does not know
    # about aborts the whole -dict transaction with
    #   [Vivado 12-4371] Cannot find parameter '...' on IP 'hbm_0'
    # so every parameter is filtered against the IP's actual property list
    # first and unknown ones are only reported as a warning.
    create_ip -name hbm -vendor xilinx.com -library ip -module_name hbm_0
    apply_ip_config [get_ips hbm_0] [list \
        CONFIG.USER_HBM_DENSITY              4GB            \
        CONFIG.USER_HBM_STACK                1              \
        CONFIG.USER_SAXI_00                  true           \
        CONFIG.USER_SWITCH_ENABLE_00         true           \
        CONFIG.USER_HBM_REF_CLK_0            100            \
        CONFIG.USER_HBM_REF_CLK_XTAL_0       100            \
        CONFIG.USER_APB_PCLK_0               100            \
        CONFIG.USER_HBM_FBDIV_0              12             \
        CONFIG.USER_MC_ENABLE_00             true           \
        CONFIG.USER_AXI_CLK_FREQ             ${hbm_axi_clk_mhz} \
    ]

    # Generate the output products; without this the modules stay black boxes.
    foreach ip {axi_protocol_converter_0 hbm_0} {
        generate_target all [get_ips $ip]
        create_ip_run [get_ips $ip]
    }
    launch_runs axi_protocol_converter_0_synth_1 hbm_0_synth_1 -jobs 4
    wait_on_run axi_protocol_converter_0_synth_1
    wait_on_run hbm_0_synth_1

    puts "INFO: HBM IP generated. NOTE: implementation requires an HBM licence."
} else {
    puts "INFO: hbm_enable = 0 — building the vendor-neutral top (${top_module})."
    puts "INFO: re-run with '-tclargs hbm' to build against the HBM IP."
}

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
puts " Elaboration complete. Top module: ${top_module}"
puts " HBM IP: [expr {$hbm_enable ? {enabled} : {disabled}}]"
puts ""
puts " Timing closure flow (300 MHz target, see README):"
puts "   1. launch_runs synth_1 -jobs 8; wait_on_run synth_1"
puts "   2. open_run synth_1 -name synth_1"
puts "      report_timing_summary"
puts "      report_utilization"
puts "      report_design_analysis -logic_level_distribution"
puts "      -> paths with more than ~8 logic levels will not close at"
puts "         300 MHz; fix them in RTL before running implementation."
puts "   3. launch_runs impl_1 -jobs 8; wait_on_run impl_1  (needs HBM licence)"
puts "   4. If WNS is only slightly negative, retry with the strategy"
puts "      Performance_ExplorePostRoutePhysOpt."
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
# report_design_analysis -logic_level_distribution \
#     -file ${project_dir}/logic_levels_synth.rpt
