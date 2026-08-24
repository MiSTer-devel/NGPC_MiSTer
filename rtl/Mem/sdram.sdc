# SDRAM controller pin timing. Paired with rtl/mem/sdram.sv.
#
# These constraints describe the physical pin relationship the controller owns:
# the inverted DDIO-forwarded SDRAM_CLK, the command/address/write-data launch,
# and the DDIO falling-edge DQ capture. That relationship is part of the
# controller's contract, so it is constrained here beside the controller
# rather than in the core SDC.
#
# Integration requirements:
# - Read this file AFTER the core SDC that runs `derive_pll_clocks`, so the
#   controller clock already exists when the generated pin clock is derived.
#   `files.qip` lists it after NGPC.sdc for that reason.
# - Set `sdram_ctrl_clk_pin` below to the PLL output pin driving the controller.
#   That single line is the only core-specific coupling in this file.
# - Crossings between the controller wrapper and the core's slower clock stay in
#   the core SDC. Those are integration timing, not pin timing.

set sdram_ctrl_clk_pin {emu|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}

set sdram_ctrl_clk [get_clocks $sdram_ctrl_clk_pin]
if {[get_collection_size $sdram_ctrl_clk] == 0} {
	post_message -type error "sdram.sdc: no controller clock matched the configured pin"
}

# SDRAM_CLK is an inverted copy of the controller clock forwarded through the
# output DDIO cell (altddio_out, datain_h=0 / datain_l=1). Modeling it as a
# generated clock ON the pin accounts for the forward path's insertion delay.
# Without this the SDRAM output paths are analyzed against the raw PLL
# reference, which lacks the ~10 ns clock-network insertion that the data path
# actually sees, and every output pin reports a large fictitious clock skew
# (measured ~6 ns at 120 MHz).
create_generated_clock -name sdram_clk_pin -invert \
	-source [get_pins $sdram_ctrl_clk_pin] \
	[get_ports {SDRAM_CLK}]

if {[get_collection_size [get_clocks sdram_clk_pin]] == 0} {
	post_message -type error "sdram.sdc: no SDRAM_CLK generated pin clock matched"
}

set sdram_out_ports [get_ports {SDRAM_CKE SDRAM_A[*] SDRAM_BA[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nCAS SDRAM_nRAS SDRAM_nWE SDRAM_DQ[*]}]

# Outputs launch on the controller clock rising edge and are sampled by the
# SDRAM on the SDRAM_CLK edge a half period later. The inversion lives in the
# generated clock, so no -clock_fall qualifier is used here.
set_output_delay -clock sdram_clk_pin -max 1.500 $sdram_out_ports
set_output_delay -clock sdram_clk_pin -min -0.800 $sdram_out_ports

# DQ is launched by the SDRAM tAC after an SDRAM_CLK edge, so the forwarded pin
# clock is the correct launch reference.
set_input_delay -clock sdram_clk_pin -max 6.000 [get_ports {SDRAM_DQ[*]}]
set_input_delay -clock sdram_clk_pin -min 2.500 [get_ports {SDRAM_DQ[*]}]

# The forwarded clock leaves the FPGA a full clock-network insertion after the
# internal capture edge, so the beat launched by pin-clock edge N is captured by
# the internal falling edge one period later. Move the setup check out by that
# period and keep the hold check on the adjacent edge.
set_multicycle_path -setup 2 -from [get_ports {SDRAM_DQ[*]}]
set_multicycle_path -hold 1 -from [get_ports {SDRAM_DQ[*]}]

# sdram.sv connects only `dataout_l` of the input DDIO cell; the rising-edge
# half is left unconnected. Its register still exists inside the hard cell, so
# exclude the dangling capture instead of reporting it as a real violation.
set dq_unused_rise_regs [get_registers {*altddio_in:u_sdram_dq_capture*dataout_h*}]
if {[get_collection_size $dq_unused_rise_regs] > 0} {
	set_false_path -to $dq_unused_rise_regs
}
