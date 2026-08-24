derive_pll_clocks
derive_clock_uncertainty

# The measured TLCS sequencer state/status/pending-store paths and their
# destination storage advance on ce_t900_g. At the fastest gear ce_t900 is one
# clk_sys pulse in 16;
# run, halt, and pause gating can only increase that spacing. TimeQuest does not
# infer the common clock-enable relationship and otherwise checks these paths
# against one 20.341 ns clk_sys period. Use a conservative two-cycle requirement
# with the matching one-cycle hold adjustment only for the exact path families
# seen in the fitted report. Decode inputs, read/write data, other sequencer
# registers, other BIU registers, and savestate data remain single-cycle timed.
set t900_state_mc_from [get_registers {*|t900_seq:u_seq|st.*}]
set t900_sr_mc_from [get_registers {*|t900_seq:u_seq|sr*}]
set t900_wr_pend_mc_from [get_registers {*|t900_seq:u_seq|wr_pend}]
set t900_int_req_mc_from [get_registers {*|g_rf_read_split.int_req_hold}]
set t900_int_level_mc_from [get_registers {*|g_rf_read_split.int_level_hold*}]
set t900_dma_req_mc_from [get_registers {*|g_rf_read_split.dma_req_hold*}]
set t900_pause_req_mc_from [get_registers {*|g_rf_read_split.pause_req_hold}]
set t900_regfile_mc_to [get_registers {*|t900_regfile:u_regfile|regs*}]
set t900_biu_addr_mc_to [get_registers {*|t900_biu:u_biu|bus_addr_r*}]

if {[get_collection_size $t900_state_mc_from] == 0} {
	post_message -type error "NGPC.sdc: no TLCS sequencer state registers matched"
}
if {[get_collection_size $t900_sr_mc_from] == 0} {
	post_message -type error "NGPC.sdc: no TLCS status registers matched"
}
if {[get_collection_size $t900_wr_pend_mc_from] == 0} {
	post_message -type error "NGPC.sdc: no TLCS pending-store register matched"
}
if {[get_collection_size $t900_regfile_mc_to] == 0} {
	post_message -type error "NGPC.sdc: no TLCS register-file storage matched"
}
if {[get_collection_size $t900_int_level_mc_from] == 0} {
	post_message -type error "NGPC.sdc: no TLCS interrupt-level snapshot registers matched"
}
if {[get_collection_size $t900_int_req_mc_from] == 0} {
	post_message -type error "NGPC.sdc: no TLCS interrupt-request snapshot register matched"
}
if {[get_collection_size $t900_dma_req_mc_from] == 0} {
	post_message -type error "NGPC.sdc: no TLCS DMA-request snapshot registers matched"
}
if {[get_collection_size $t900_pause_req_mc_from] == 0} {
	post_message -type error "NGPC.sdc: no TLCS pause-request snapshot register matched"
}
if {[get_collection_size $t900_biu_addr_mc_to] == 0} {
	post_message -type error "NGPC.sdc: no TLCS BIU address registers matched"
}

set_multicycle_path -setup -from $t900_state_mc_from -to $t900_regfile_mc_to 2
set_multicycle_path -hold  -from $t900_state_mc_from -to $t900_regfile_mc_to 1
set_multicycle_path -setup -from $t900_sr_mc_from -to $t900_regfile_mc_to 2
set_multicycle_path -hold  -from $t900_sr_mc_from -to $t900_regfile_mc_to 1

# wr_pend is set or cleared only in the sequencer's ce-qualified state update
# (apart from reset/restore, when ce_t900_g and normal register-file writes are
# suppressed). The register file can consume its decode cone only when the same
# ce_t900_g pulse asserts rf_wr_en, so no next-clk_sys-edge capture exists.
set_multicycle_path -setup -from $t900_wr_pend_mc_from -to $t900_regfile_mc_to 2
set_multicycle_path -hold  -from $t900_wr_pend_mc_from -to $t900_regfile_mc_to 1

# In the production split path, the control snapshots below are sampled by
# ce_d exactly one clk_sys cycle after a ce_t900 tick and are then frozen until
# the next tick. Even at the fastest gear that gives each source fifteen
# clk_sys cycles before the next possible register-file write. Reset and pause
# may sample them more often, but neither state permits a register-file commit.
# Keep the exceptions limited to these proven snapshots; the decode/read holds
# remain single-cycle.
set_multicycle_path -setup -from $t900_int_level_mc_from -to $t900_regfile_mc_to 2
set_multicycle_path -hold  -from $t900_int_level_mc_from -to $t900_regfile_mc_to 1
set_multicycle_path -setup -from $t900_int_req_mc_from -to $t900_regfile_mc_to 2
set_multicycle_path -hold  -from $t900_int_req_mc_from -to $t900_regfile_mc_to 1
set_multicycle_path -setup -from $t900_dma_req_mc_from -to $t900_regfile_mc_to 2
set_multicycle_path -hold  -from $t900_dma_req_mc_from -to $t900_regfile_mc_to 1
set_multicycle_path -setup -from $t900_pause_req_mc_from -to $t900_regfile_mc_to 2
set_multicycle_path -hold  -from $t900_pause_req_mc_from -to $t900_regfile_mc_to 1

# bus_addr_r advances only in t900_biu's ce-qualified sequential block, so the
# measured state, status, and frozen interrupt launches cannot be consumed
# there until another ce_t900 tick.
set_multicycle_path -setup -from $t900_state_mc_from -to $t900_biu_addr_mc_to 2
set_multicycle_path -hold  -from $t900_state_mc_from -to $t900_biu_addr_mc_to 1
set_multicycle_path -setup -from $t900_sr_mc_from -to $t900_biu_addr_mc_to 2
set_multicycle_path -hold  -from $t900_sr_mc_from -to $t900_biu_addr_mc_to 1
set_multicycle_path -setup -from $t900_int_level_mc_from -to $t900_biu_addr_mc_to 2
set_multicycle_path -hold  -from $t900_int_level_mc_from -to $t900_biu_addr_mc_to 1

# clk_sys (PLL c0, 49.152 MHz) and clk_ram (PLL c1, 98.304 MHz) come from the
# same PLL and are crossed with plain synchronous handshakes in the cart
# SDRAM service. NEVER add set_false_path or set_clock_groups between them:
# the crossing correctness depends on them being timed as related clocks.
# SDRAM pin timing lives in rtl/Mem/sdram.sdc (expects the controller clock
# on PLL output counter general[1], i.e. c1).
