// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_cart_overlay -- transaction owner for cartridge-flash persistence.
//
// This module keeps the logical ledgers, serializes the V1 S0 loader and
// transactional saver onto one 512-byte HPS bridge, and keeps one
// owner on the live-cart p2 and DDR3 channel-2 bundles.  It deliberately does
// not interpret NOR commands: ngp_cart reports exact completed physical block
// mutations after the flash FSM has accepted them.
module ngp_cart_overlay
(
	input  wire        clk,
	input  wire        reset,
	input  wire        cart_replace_i,
	input  wire        cart_ready_i,

	input  wire [31:0] identity_raw_crc32_i,
	input  wire [31:0] identity_raw_bytes_i,
	input  wire [31:0] identity_pristine_crc32_i,
	input  wire [31:0] identity_physical_bytes_i,
	input  wire [1:0]  identity_die0_code_i,
	input  wire [1:0]  identity_die1_code_i,
	input  wire [15:0] identity_catalog_i,
	input  wire [7:0]  identity_subcatalog_i,
	input  wire [95:0] identity_title_i,

	input  wire        event0_i,
	input  wire [5:0]  block0_i,
	input  wire        event1_i,
	input  wire [5:0]  block1_i,
	input  wire [1:0]  die_busy_i,

	// MiSTer S0 lifecycle and menu requests. save_i/load_i are pulses.
	input  wire        mount_i,
	input  wire        mount_readonly_i,
	input  wire [63:0] mount_size_i,
	input  wire        save_i,
	input  wire        load_i,
	input  wire        operation_enable_i,
	input  wire        autosave_disable_i,
	input  wire        osd_open_i,

	// A sparse manual-state load replaces the live overlay but does not replace
	// the committed S0 directory.  It therefore enters the ledger as pending.
	input  wire        state_adopt_i,
	input  wire [34:0] state_map0_i,
	input  wire [34:0] state_map1_i,
	input  wire        state_force_flash_read_i,

	input  wire        pause_ready_i,
	output wire        pause_req_o,
	output wire        force_flash_read_o,
	output wire        busy_o,
	output wire        boot_hold_o,
	output wire        pending_o,
	output wire        mounted_writable_o,
	output reg         save_done_o,
	output reg         save_rejected_o,
	output reg         load_done_o,
	output reg         load_rejected_o,
	output wire [34:0] live0_o,
	output wire [34:0] live1_o,
	output wire [34:0] pending0_o,
	output wire [34:0] pending1_o,
	output wire [34:0] file0_o,
	output wire [34:0] file1_o,

	// MiSTer S0 block interface, one 512-byte sector per request.
	output wire [31:0] sd_lba_o,
	output wire        sd_rd_o,
	output wire        sd_wr_o,
	input  wire        sd_ack_i,
	input  wire [12:0] sd_buff_addr_i,
	input  wire [15:0] sd_buff_dout_i,
	input  wire        sd_buff_wr_i,
	output wire [15:0] sd_buff_din_o,

	// Canonical live-cart external-SDRAM background client.
	output wire        p2_req_o,
	output wire        p2_we_o,
	output wire [24:0] p2_addr_o,
	output wire [15:0] p2_wdata_o,
	output wire [1:0]  p2_be_o,
	input  wire        p2_ready_i,
	input  wire        p2_done_i,
	input  wire [15:0] p2_rdata_i,

	// Shared DDR3 channel-2 client.
	output wire [27:1] ddr_addr_o,
	output wire [63:0] ddr_din_o,
	output wire        ddr_req_o,
	output wire        ddr_rnw_o,
	output wire [7:0]  ddr_be_o,
	input  wire [63:0] ddr_dout_i,
	input  wire        ddr_ready_i
);

	localparam [4:0] ST_IDLE              = 5'd0;
	localparam [4:0] ST_LOAD_CLASSIFY     = 5'd1;
	localparam [4:0] ST_NORMAL_START      = 5'd2;
	localparam [4:0] ST_NORMAL_WAIT       = 5'd3;
	localparam [4:0] ST_IMPORT_START      = 5'd4;
	localparam [4:0] ST_IMPORT_WAIT       = 5'd5;
	localparam [4:0] ST_IMPORT_ENTRY_SET  = 5'd6;
	localparam [4:0] ST_IMPORT_ENTRY_WAIT = 5'd7;
	localparam [4:0] ST_IMPORT_ENTRY_SEND = 5'd8;
	localparam [4:0] ST_IMPORT_COMMIT     = 5'd9;
	localparam [4:0] ST_IMPORT_DONE       = 5'd10;
	localparam [4:0] ST_SAVE_START        = 5'd13;
	localparam [4:0] ST_SAVE_WAIT         = 5'd14;

	localparam [15:0] MAX_FILE_SECTORS = 16'd16386;

	reg [4:0]  state_q;
	reg        boot_wait_q;
	reg        mount_pending_q;
	reg        mount_valid_q;
	reg        mount_readonly_q;
	reg [15:0] mount_sectors_q;
	reg        mount_sectors_valid_q;
	reg        directory_dirty_q;
	reg        metadata_fault_q;
	reg        osd_open_q;
	reg        manual_save_pending_q;
	reg        manual_load_pending_q;
	reg        mount_empty_q;
	reg [6:0]  import_index_q;

	wire child_reset_w = reset || cart_replace_i;

	wire [34:0] ledger_live0_w, ledger_live1_w;
	wire [34:0] ledger_pending0_w, ledger_pending1_w;
	wire [34:0] ledger_file0_w, ledger_file1_w;
	wire [34:0] ledger_snapshot0_w, ledger_snapshot1_w;
	wire ledger_snapshot_active_w;

	wire load_busy_w, load_done_w, load_rejected_w;
	wire load_recoverable_incomplete_w;
	wire load_pause_req_w, load_force_read_w, load_ledger_adopt_w;
	wire [34:0] load_target0_w, load_target1_w;
	wire [31:0] load_generation_w;
	wire load_directory_lba_w;
	wire [34:0] load_active0_w, load_active1_w;
	wire [15:0] load_next_free_w;
	wire [15:0] load_entry_base_w;
	wire [31:0] load_entry_crc_w;

	wire save_busy_w, save_done_w, save_rejected_w;
	wire save_pause_req_w;
	wire save_snapshot_req_w, save_snapshot_abort_w, save_commit_w;
	wire [34:0] save_commit_file0_w, save_commit_file1_w;
	wire [34:0] save_pristine0_w, save_pristine1_w;
	wire save_import_ready_w, save_import_done_w, save_import_rejected_w;

	wire normal_owner_w = (state_q == ST_NORMAL_START) ||
		(state_q == ST_NORMAL_WAIT) || load_busy_w;
	wire save_owner_w = (state_q == ST_SAVE_START) ||
		(state_q == ST_SAVE_WAIT) || save_busy_w;

	wire osd_open_rise_w = osd_open_i && !osd_open_q;
	wire ledger_pending_w = (ledger_pending0_w != 35'd0) ||
		(ledger_pending1_w != 35'd0) || directory_dirty_q;
	wire mounted_writable_w = mount_valid_q && !mount_readonly_q &&
		!metadata_fault_q;
	wire autosave_request_w = osd_open_rise_w && !autosave_disable_i &&
		ledger_pending_w && mounted_writable_w;

	assign busy_o = (state_q != ST_IDLE) || load_busy_w || save_busy_w;
	assign boot_hold_o = boot_wait_q;
	assign pending_o = ledger_pending_w;
	assign mounted_writable_o = mounted_writable_w;
	assign live0_o = ledger_live0_w;
	assign live1_o = ledger_live1_w;
	assign pending0_o = ledger_pending0_w;
	assign pending1_o = ledger_pending1_w;
	assign file0_o = ledger_file0_w;
	assign file1_o = ledger_file1_w;
	assign pause_req_o = load_pause_req_w | save_pause_req_w;
	assign force_flash_read_o = load_force_read_w | state_force_flash_read_i;

	ngp_cart_overlay_ledger u_ledger
	(
		.clk(clk), .reset(reset), .cart_replace_i(cart_replace_i),
		.event0_i(event0_i), .block0_i(block0_i),
		.event1_i(event1_i), .block1_i(block1_i),
		.snapshot_i(save_snapshot_req_w), .abort_i(save_snapshot_abort_w),
		.commit_i(save_commit_w),
		.commit_file0_i(save_commit_file0_w),
		.commit_file1_i(save_commit_file1_w),
		.load_adopt_i(load_ledger_adopt_w),
		.load_map0_i(load_target0_w), .load_map1_i(load_target1_w),
		.state_adopt_i(state_adopt_i),
		.state_map0_i(state_map0_i), .state_map1_i(state_map1_i),
		// Compare-to-pristine only takes effect at commit, so a retained bit
		// cannot clear a later mutation.
		.pristine0_i(save_commit_w ? save_pristine0_w : 35'd0),
		.pristine1_i(save_commit_w ? save_pristine1_w : 35'd0),
		.live0_o(ledger_live0_w), .live1_o(ledger_live1_w),
		.pending0_o(ledger_pending0_w), .pending1_o(ledger_pending1_w),
		.file0_o(ledger_file0_w), .file1_o(ledger_file1_w),
		.snapshot0_o(ledger_snapshot0_w), .snapshot1_o(ledger_snapshot1_w),
		.snapshot_active_o(ledger_snapshot_active_w)
	);

	// -------------------------- sector bridge -----------------------------
	wire bridge_fill_start_w, bridge_buf_wr_w;
	wire [7:0] bridge_buf_addr_w;
	wire [15:0] bridge_buf_wdata_w, bridge_buf_rdata_w;
	wire bridge_fill_ready_w, bridge_buf_wready_w, bridge_buf_full_w;
	wire bridge_fill_fault_w;
	wire [31:0] bridge_lba_w;
	wire bridge_save_req_w, bridge_save_ready_w;
	wire bridge_load_req_w, bridge_load_ready_w;
	wire bridge_rx_release_w, bridge_rx_valid_w, bridge_rx_error_w;
	wire bridge_busy_w;

	wire [31:0] load_sector_lba_w, save_sector_lba_w;
	wire load_sector_req_w, save_sector_req_w;
	wire [7:0] load_sector_addr_w, save_sector_addr_w;
	wire load_sector_release_w;
	wire save_fill_start_w, save_buf_wr_w;
	wire [15:0] save_buf_wdata_w;

	assign bridge_fill_start_w = save_owner_w && save_fill_start_w;
	assign bridge_buf_addr_w = save_owner_w ? save_sector_addr_w :
		load_sector_addr_w;
	assign bridge_buf_wr_w = save_owner_w && save_buf_wr_w;
	assign bridge_buf_wdata_w = save_buf_wdata_w;
	assign bridge_lba_w = save_owner_w ? save_sector_lba_w :
		load_sector_lba_w;
	assign bridge_save_req_w = save_owner_w && save_sector_req_w;
	assign bridge_load_req_w = normal_owner_w ? load_sector_req_w : 1'b0;
	assign bridge_rx_release_w = normal_owner_w ? load_sector_release_w : 1'b0;

	ngp_cart_overlay_sector_bridge u_sector_bridge
	(
		.clk(clk), .reset(child_reset_w),
		.ctrl_fill_start_i(bridge_fill_start_w),
		.ctrl_buf_addr_i(bridge_buf_addr_w), .ctrl_buf_wr_i(bridge_buf_wr_w),
		.ctrl_buf_wdata_i(bridge_buf_wdata_w), .ctrl_buf_rdata_o(bridge_buf_rdata_w),
		.ctrl_fill_ready_o(bridge_fill_ready_w),
		.ctrl_buf_wready_o(bridge_buf_wready_w),
		.ctrl_buf_full_o(bridge_buf_full_w),
		.ctrl_fill_fault_o(bridge_fill_fault_w),
		.ctrl_lba_i(bridge_lba_w), .ctrl_save_req_i(bridge_save_req_w),
		.ctrl_save_ready_o(bridge_save_ready_w),
		.ctrl_load_req_i(bridge_load_req_w),
		.ctrl_load_ready_o(bridge_load_ready_w),
		.ctrl_rx_release_i(bridge_rx_release_w),
		.ctrl_rx_valid_o(bridge_rx_valid_w),
		.ctrl_rx_error_o(bridge_rx_error_w), .ctrl_busy_o(bridge_busy_w),
		.sd_lba_o(sd_lba_o), .sd_rd_o(sd_rd_o), .sd_wr_o(sd_wr_o),
		.sd_ack_i(sd_ack_i), .sd_buff_addr_i(sd_buff_addr_i),
		.sd_buff_dout_i(sd_buff_dout_i), .sd_buff_wr_i(sd_buff_wr_i),
		.sd_buff_din_o(sd_buff_din_o)
	);

	// ------------------------ normal V1 load ------------------------------
	wire load_p2_req_w, load_p2_we_w;
	wire [24:0] load_p2_addr_w;
	wire [15:0] load_p2_wdata_w;
	wire [1:0] load_p2_be_w;
	wire [27:1] load_ddr_addr_w;
	wire [63:0] load_ddr_din_w;
	wire load_ddr_req_w, load_ddr_rnw_w;
	wire [7:0] load_ddr_be_w;

	ngp_cart_overlay_s0_load u_normal_load
	(
		.clk(clk), .reset(child_reset_w), .start_i(state_q == ST_NORMAL_START),
		.busy_o(load_busy_w), .done_o(load_done_w), .rejected_o(load_rejected_w),
		.recoverable_incomplete_o(load_recoverable_incomplete_w),
		.identity_raw_crc32_i(identity_raw_crc32_i),
		.identity_raw_bytes_i(identity_raw_bytes_i),
		.identity_pristine_crc32_i(identity_pristine_crc32_i),
		.identity_physical_bytes_i(identity_physical_bytes_i),
		.identity_die0_code_i(identity_die0_code_i),
		.identity_die1_code_i(identity_die1_code_i),
		.identity_catalog_i(identity_catalog_i),
		.identity_subcatalog_i(identity_subcatalog_i),
		.identity_title_i(identity_title_i),
		.file_sectors_valid_i(mount_sectors_valid_q),
		.file_sectors_i(mount_sectors_q),
		.old_live0_i(ledger_live0_w), .old_live1_i(ledger_live1_w),
		.die_busy_i(die_busy_i), .event0_i(event0_i), .block0_i(block0_i),
		.event1_i(event1_i), .block1_i(block1_i),
		.pause_ready_i(pause_ready_i), .pause_req_o(load_pause_req_w),
		.force_flash_read_o(load_force_read_w),
		.ledger_adopt_o(load_ledger_adopt_w),
		.ledger_target0_o(load_target0_w), .ledger_target1_o(load_target1_w),
		.file_generation_o(load_generation_w),
		.file_directory_lba_o(load_directory_lba_w),
		.file_active0_o(load_active0_w), .file_active1_o(load_active1_w),
		.file_next_free_lba_o(load_next_free_w),
		.file_entry_index_i(import_index_q),
		.file_entry_base_lba_o(load_entry_base_w),
		.file_entry_crc32_o(load_entry_crc_w),
		.sector_lba_o(load_sector_lba_w), .sector_load_req_o(load_sector_req_w),
		.sector_load_ready_i(normal_owner_w && bridge_load_ready_w),
		.sector_rx_valid_i(normal_owner_w && bridge_rx_valid_w),
		.sector_rx_error_i(normal_owner_w && bridge_rx_error_w),
		.sector_buf_addr_o(load_sector_addr_w),
		.sector_buf_rdata_i(bridge_buf_rdata_w),
		.sector_rx_release_o(load_sector_release_w),
		.p2_req_o(load_p2_req_w), .p2_we_o(load_p2_we_w),
		.p2_addr_o(load_p2_addr_w), .p2_wdata_o(load_p2_wdata_w),
		.p2_be_o(load_p2_be_w), .p2_ready_i(normal_owner_w && p2_ready_i),
		.p2_done_i(normal_owner_w && p2_done_i), .p2_rdata_i(p2_rdata_i),
		.ddr_addr_o(load_ddr_addr_w), .ddr_din_o(load_ddr_din_w),
		.ddr_req_o(load_ddr_req_w), .ddr_rnw_o(load_ddr_rnw_w),
		.ddr_be_o(load_ddr_be_w), .ddr_dout_i(ddr_dout_i),
		.ddr_ready_i(normal_owner_w && ddr_ready_i)
	);

	// -------------------------- transactional save ------------------------
	wire save_p2_req_w, save_p2_we_w;
	wire [24:0] save_p2_addr_w;
	wire [15:0] save_p2_wdata_w;
	wire [1:0] save_p2_be_w;
	wire [27:1] save_ddr_addr_w;
	wire [63:0] save_ddr_din_w;
	wire save_ddr_req_w, save_ddr_rnw_w;
	wire [7:0] save_ddr_be_w;

	// A newly mounted slot has no relationship to the prior directory table.
	// Reset the save-side committed metadata while the controller is idle; a
	// busy remount remains fail-closed below rather than changing ownership.
	wire save_metadata_reset_w = child_reset_w ||
		(mount_i && (state_q == ST_IDLE) && !load_busy_w && !save_busy_w);

	ngp_cart_overlay_s0_save u_save
	(
		.clk(clk), .reset(save_metadata_reset_w), .start_i(state_q == ST_SAVE_START),
		.directory_dirty_i(directory_dirty_q),
		.busy_o(save_busy_w), .done_o(save_done_w), .rejected_o(save_rejected_w),
		.identity_raw_crc32_i(identity_raw_crc32_i),
		.identity_raw_bytes_i(identity_raw_bytes_i),
		.identity_pristine_crc32_i(identity_pristine_crc32_i),
		.identity_physical_bytes_i(identity_physical_bytes_i),
		.identity_die0_code_i(identity_die0_code_i),
		.identity_die1_code_i(identity_die1_code_i),
		.identity_catalog_i(identity_catalog_i),
		.identity_subcatalog_i(identity_subcatalog_i),
		.identity_title_i(identity_title_i),
		.live0_i(ledger_live0_w), .live1_i(ledger_live1_w),
		.pending0_i(ledger_pending0_w), .pending1_i(ledger_pending1_w),
		.snapshot0_i(ledger_snapshot0_w), .snapshot1_i(ledger_snapshot1_w),
		.snapshot_active_i(ledger_snapshot_active_w),
		.snapshot_req_o(save_snapshot_req_w),
		.snapshot_abort_o(save_snapshot_abort_w), .commit_o(save_commit_w),
		.commit_file0_o(save_commit_file0_w),
		.commit_file1_o(save_commit_file1_w),
		.pristine0_o(save_pristine0_w), .pristine1_o(save_pristine1_w),
		.die_busy_i(die_busy_i), .event0_i(event0_i), .block0_i(block0_i),
		.event1_i(event1_i), .block1_i(block1_i),
		.pause_ready_i(pause_ready_i), .pause_req_o(save_pause_req_w),
		.import_start_i(state_q == ST_IMPORT_START),
		.import_directory_lba_i(load_directory_lba_w),
		.import_generation_i(load_generation_w),
		.import_file0_i(load_target0_w), .import_file1_i(load_target1_w),
		.import_active0_i(load_active0_w), .import_active1_i(load_active1_w),
		.import_next_free_lba_i(load_next_free_w),
		.import_entry_valid_i(state_q == ST_IMPORT_ENTRY_SEND),
		.import_entry_index_i(import_index_q),
		.import_entry_base_lba_i(load_entry_base_w),
		.import_entry_crc32_i(load_entry_crc_w),
		.import_commit_i(state_q == ST_IMPORT_COMMIT),
		.import_ready_o(save_import_ready_w), .import_done_o(save_import_done_w),
		.import_rejected_o(save_import_rejected_w),
		.sector_fill_start_o(save_fill_start_w),
		.sector_buf_addr_o(save_sector_addr_w),
		.sector_buf_wr_o(save_buf_wr_w), .sector_buf_wdata_o(save_buf_wdata_w),
		.sector_fill_ready_i(save_owner_w && bridge_fill_ready_w),
		.sector_buf_wready_i(save_owner_w && bridge_buf_wready_w),
		.sector_buf_full_i(save_owner_w && bridge_buf_full_w),
		.sector_fill_fault_i(save_owner_w && bridge_fill_fault_w),
		.sector_lba_o(save_sector_lba_w), .sector_save_req_o(save_sector_req_w),
		.sector_save_ready_i(save_owner_w && bridge_save_ready_w),
		.sector_busy_i(save_owner_w && bridge_busy_w),
		.p2_req_o(save_p2_req_w), .p2_we_o(save_p2_we_w),
		.p2_addr_o(save_p2_addr_w), .p2_wdata_o(save_p2_wdata_w),
		.p2_be_o(save_p2_be_w), .p2_ready_i(save_owner_w && p2_ready_i),
		.p2_done_i(save_owner_w && p2_done_i), .p2_rdata_i(p2_rdata_i),
		.ddr_addr_o(save_ddr_addr_w), .ddr_din_o(save_ddr_din_w),
		.ddr_req_o(save_ddr_req_w), .ddr_rnw_o(save_ddr_rnw_w),
		.ddr_be_o(save_ddr_be_w), .ddr_dout_i(ddr_dout_i),
		.ddr_ready_i(save_owner_w && ddr_ready_i)
	);

	// One structural owner for the two memory clients.
	assign p2_req_o = normal_owner_w ? load_p2_req_w :
		save_owner_w ? save_p2_req_w : 1'b0;
	assign p2_we_o = normal_owner_w ? load_p2_we_w :
		save_owner_w ? save_p2_we_w : 1'b0;
	assign p2_addr_o = normal_owner_w ? load_p2_addr_w : save_p2_addr_w;
	assign p2_wdata_o = normal_owner_w ? load_p2_wdata_w : save_p2_wdata_w;
	assign p2_be_o = normal_owner_w ? load_p2_be_w : save_p2_be_w;

	assign ddr_addr_o = normal_owner_w ? load_ddr_addr_w : save_ddr_addr_w;
	assign ddr_din_o = normal_owner_w ? load_ddr_din_w : save_ddr_din_w;
	assign ddr_req_o = normal_owner_w ? load_ddr_req_w :
		save_owner_w ? save_ddr_req_w : 1'b0;
	assign ddr_rnw_o = normal_owner_w ? load_ddr_rnw_w : save_ddr_rnw_w;
	assign ddr_be_o = normal_owner_w ? load_ddr_be_w : save_ddr_be_w;

	// -------------------------- operation sequencer -----------------------
	always @(posedge clk) begin
		save_done_o <= 1'b0;
		save_rejected_o <= 1'b0;
		load_done_o <= 1'b0;
		load_rejected_o <= 1'b0;
		osd_open_q <= osd_open_i;

		if (reset || cart_replace_i) begin
			state_q <= ST_IDLE;
			boot_wait_q <= cart_replace_i;
			mount_pending_q <= 1'b0;
			mount_valid_q <= 1'b0;
			mount_readonly_q <= 1'b0;
			mount_sectors_q <= 16'd0;
			mount_sectors_valid_q <= 1'b0;
			directory_dirty_q <= 1'b0;
			metadata_fault_q <= 1'b0;
			osd_open_q <= 1'b0;
			manual_save_pending_q <= 1'b0;
			manual_load_pending_q <= 1'b0;
			mount_empty_q <= 1'b0;
			import_index_q <= 7'd0;
		end else begin
			if (save_i) manual_save_pending_q <= 1'b1;
			if (load_i) manual_load_pending_q <= 1'b1;
			// Loading a manual state can replace battery-backed flash. Queue a
			// self-contained S0 generation immediately; waiting for a future OSD
			// rising edge loses the change if the user reloads the core first.
			if (state_adopt_i && !autosave_disable_i && mounted_writable_w)
				manual_save_pending_q <= 1'b1;
			if (mount_i) begin
				if ((state_q != ST_IDLE) || load_busy_w || save_busy_w)
					metadata_fault_q <= 1'b1;
				mount_pending_q <= 1'b1;
				mount_valid_q <= 1'b1;
				mount_readonly_q <= mount_readonly_i;
				mount_empty_q <= mount_size_i == 64'd0;
				mount_sectors_q <= mount_size_i[24:9];
				mount_sectors_valid_q <=
					(mount_size_i[63:25] == 39'd0) &&
					(mount_size_i[8:0] == 9'd0) &&
					(mount_size_i[24:9] <= MAX_FILE_SECTORS);
			end

			if (load_ledger_adopt_w) begin
				directory_dirty_q <= 1'b0;
			end else if (state_adopt_i) begin
				directory_dirty_q <= 1'b1;
			end else if (save_commit_w) begin
				directory_dirty_q <= 1'b0;
			end

			case (state_q)
				ST_IDLE: begin
					if (operation_enable_i && mount_pending_q && cart_ready_i) begin
						mount_pending_q <= 1'b0;
						state_q <= ST_LOAD_CLASSIFY;
					end else if (operation_enable_i && manual_load_pending_q &&
					             mount_valid_q && cart_ready_i) begin
						manual_load_pending_q <= 1'b0;
						// A runtime backup load changes the same live cartridge bytes
						// as boot-time mount application. Hold the machine in reset for
						// the complete validate/apply/import transaction as well.
						boot_wait_q <= 1'b1;
						state_q <= ST_LOAD_CLASSIFY;
					end else if (operation_enable_i &&
					             (manual_save_pending_q || autosave_request_w) &&
					             mounted_writable_w && cart_ready_i) begin
						manual_save_pending_q <= 1'b0;
						state_q <= ST_SAVE_START;
					end
				end

				ST_LOAD_CLASSIFY: begin
					if (mount_empty_q) begin
						metadata_fault_q <= 1'b0;
						load_done_o <= 1'b1;
						boot_wait_q <= 1'b0;
						state_q <= ST_IDLE;
					end else if (!mount_sectors_valid_q ||
					             (mount_sectors_q == 16'd0)) begin
						metadata_fault_q <= 1'b1;
						load_rejected_o <= 1'b1;
						boot_wait_q <= 1'b0;
						state_q <= ST_IDLE;
					end else begin
						state_q <= ST_NORMAL_START;
					end
				end

				ST_NORMAL_START: state_q <= ST_NORMAL_WAIT;

				ST_NORMAL_WAIT: begin
					if (load_done_w) begin
						import_index_q <= 7'd0;
						state_q <= ST_IMPORT_START;
					end else if (load_rejected_w) begin
						metadata_fault_q <= !load_recoverable_incomplete_w;
						if (load_recoverable_incomplete_w)
							directory_dirty_q <= 1'b1;
						load_rejected_o <= 1'b1;
						boot_wait_q <= 1'b0;
						state_q <= ST_IDLE;
					end
				end

				ST_IMPORT_START: state_q <= ST_IMPORT_WAIT;

				ST_IMPORT_WAIT: begin
					if (save_import_rejected_w) begin
						metadata_fault_q <= 1'b1;
						load_rejected_o <= 1'b1;
						boot_wait_q <= 1'b0;
						state_q <= ST_IDLE;
					end else if (save_import_ready_w) begin
						state_q <= ST_IMPORT_ENTRY_SET;
					end
				end

				ST_IMPORT_ENTRY_SET: state_q <= ST_IMPORT_ENTRY_WAIT;
				ST_IMPORT_ENTRY_WAIT: state_q <= ST_IMPORT_ENTRY_SEND;

				ST_IMPORT_ENTRY_SEND: begin
					if (import_index_q == 7'd69) state_q <= ST_IMPORT_COMMIT;
					else begin
						import_index_q <= import_index_q + 7'd1;
						state_q <= ST_IMPORT_ENTRY_SET;
					end
				end

				ST_IMPORT_COMMIT: state_q <= ST_IMPORT_DONE;

				ST_IMPORT_DONE: begin
					if (save_import_done_w) begin
						metadata_fault_q <= 1'b0;
						load_done_o <= 1'b1;
						boot_wait_q <= 1'b0;
						state_q <= ST_IDLE;
					end else if (save_import_rejected_w) begin
						metadata_fault_q <= 1'b1;
						load_rejected_o <= 1'b1;
						boot_wait_q <= 1'b0;
						state_q <= ST_IDLE;
					end
				end

				ST_SAVE_START: state_q <= ST_SAVE_WAIT;

				ST_SAVE_WAIT: begin
					if (save_done_w) begin
						save_done_o <= 1'b1;
						state_q <= ST_IDLE;
					end else if (save_rejected_w) begin
						save_rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end
				end

				default: state_q <= ST_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
