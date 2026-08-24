// Copyright (c) 2026 Jamie Blanks

`default_nettype none
`timescale 1ns/1ps

// Transactional sparse-S0 save controller.
//
// The live cartridge is never exposed to HPS. Complete pending physical blocks
// are captured into DDR3 staging through ngp_cart_overlay_mover, compared with
// the immutable DDR3 shadow, and then transported one 512-byte sector at a
// time. The prior committed allocation table remains in its own M10K until the
// inactive directory sector has completed; the directory codec's M10K is the
// proposed table for the in-flight transaction.
module ngp_cart_overlay_s0_save
(
	input  wire        clk,
	input  wire        reset,
	input  wire        start_i,
	input  wire        directory_dirty_i,
	output wire        busy_o,
	output reg         done_o,
	output reg         rejected_o,

	input  wire [31:0] identity_raw_crc32_i,
	input  wire [31:0] identity_raw_bytes_i,
	input  wire [31:0] identity_pristine_crc32_i,
	input  wire [31:0] identity_physical_bytes_i,
	input  wire [1:0]  identity_die0_code_i,
	input  wire [1:0]  identity_die1_code_i,
	input  wire [15:0] identity_catalog_i,
	input  wire [7:0]  identity_subcatalog_i,
	input  wire [95:0] identity_title_i,

	// Exact ledger snapshot/commit seam.
	input  wire [34:0] live0_i,
	input  wire [34:0] live1_i,
	input  wire [34:0] pending0_i,
	input  wire [34:0] pending1_i,
	input  wire [34:0] snapshot0_i,
	input  wire [34:0] snapshot1_i,
	input  wire        snapshot_active_i,
	output reg         snapshot_req_o,
	output reg         snapshot_abort_o,
	output reg         commit_o,
	output wire [34:0] commit_file0_o,
	output wire [34:0] commit_file1_o,
	output wire [34:0] pristine0_o,
	output wire [34:0] pristine1_o,

	// Exact mutation and exceptional pause seam.
	input  wire [1:0]  die_busy_i,
	input  wire        event0_i,
	input  wire [5:0]  block0_i,
	input  wire        event1_i,
	input  wire [5:0]  block1_i,
	input  wire        pause_ready_i,
	output wire        pause_req_o,

	// Atomically import the selected, already validated normal directory from
	// the S0 loader. Entries must be supplied once each in index order 0..69.
	input  wire        import_start_i,
	input  wire        import_directory_lba_i,
	input  wire [31:0] import_generation_i,
	input  wire [34:0] import_file0_i,
	input  wire [34:0] import_file1_i,
	input  wire [34:0] import_active0_i,
	input  wire [34:0] import_active1_i,
	input  wire [15:0] import_next_free_lba_i,
	input  wire        import_entry_valid_i,
	input  wire [6:0]  import_entry_index_i,
	input  wire [15:0] import_entry_base_lba_i,
	input  wire [31:0] import_entry_crc32_i,
	input  wire        import_commit_i,
	output wire        import_ready_o,
	output reg         import_done_o,
	output reg         import_rejected_o,

	// Controller side of ngp_cart_overlay_sector_bridge.
	output wire        sector_fill_start_o,
	output wire [7:0]  sector_buf_addr_o,
	output wire        sector_buf_wr_o,
	output wire [15:0] sector_buf_wdata_o,
	input  wire        sector_fill_ready_i,
	input  wire        sector_buf_wready_i,
	input  wire        sector_buf_full_i,
	input  wire        sector_fill_fault_i,
	output wire [31:0] sector_lba_o,
	output wire        sector_save_req_o,
	input  wire        sector_save_ready_i,
	input  wire        sector_busy_i,

	// Canonical live cartridge p2 client.
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

	localparam [5:0] ST_IDLE              = 6'd0;
	localparam [5:0] ST_WAIT_SNAPSHOT     = 6'd1;
	localparam [5:0] ST_PROP_COPY_REQ     = 6'd2;
	localparam [5:0] ST_PROP_COPY_LATCH   = 6'd3;
	localparam [5:0] ST_PROP_COPY_WRITE   = 6'd4;
	localparam [5:0] ST_SCRUB_REQ         = 6'd5;
	localparam [5:0] ST_SCRUB_CHECK       = 6'd6;
	localparam [5:0] ST_CAPTURE_SCAN      = 6'd7;
	localparam [5:0] ST_CAPTURE_ENTRY_REQ = 6'd8;
	localparam [5:0] ST_CAPTURE_ENTRY_GET = 6'd9;
	localparam [5:0] ST_CAPTURE_WAIT_IDLE = 6'd10;
	localparam [5:0] ST_CAPTURE_START     = 6'd11;
	localparam [5:0] ST_CAPTURE_WAIT      = 6'd12;
	localparam [5:0] ST_COMPARE_STAGE     = 6'd13;
	localparam [5:0] ST_COMPARE_SHADOW    = 6'd14;
	localparam [5:0] ST_MARK_FILL_START   = 6'd15;
	localparam [5:0] ST_MARK_FILL         = 6'd16;
	localparam [5:0] ST_MARK_SAVE_REQ     = 6'd17;
	localparam [5:0] ST_MARK_WAIT_BUSY    = 6'd18;
	localparam [5:0] ST_MARK_WAIT_DONE    = 6'd19;
	localparam [5:0] ST_PAYLOAD_SCAN      = 6'd20;
	localparam [5:0] ST_PAYLOAD_ENTRY_REQ = 6'd21;
	localparam [5:0] ST_PAYLOAD_ENTRY_GET = 6'd22;
	localparam [5:0] ST_PAYLOAD_FILL_START = 6'd23;
	localparam [5:0] ST_PAYLOAD_DDR_READ  = 6'd24;
	localparam [5:0] ST_PAYLOAD_BUF_WRITE = 6'd25;
	localparam [5:0] ST_PAYLOAD_SAVE_REQ  = 6'd26;
	localparam [5:0] ST_PAYLOAD_WAIT_BUSY = 6'd27;
	localparam [5:0] ST_PAYLOAD_WAIT_DONE = 6'd28;
	localparam [5:0] ST_DIR_FILL_START    = 6'd29;
	localparam [5:0] ST_DIR_BUILD_START   = 6'd30;
	localparam [5:0] ST_DIR_FILL          = 6'd31;
	localparam [5:0] ST_DIR_SAVE_REQ      = 6'd32;
	localparam [5:0] ST_DIR_WAIT_BUSY     = 6'd33;
	localparam [5:0] ST_DIR_WAIT_DONE     = 6'd34;
	localparam [5:0] ST_ADOPT_REQ         = 6'd35;
	localparam [5:0] ST_ADOPT_WRITE       = 6'd36;
	localparam [5:0] ST_IMPORT_ENTRIES    = 6'd37;
	localparam [5:0] ST_IMPORT_READY      = 6'd38;
	localparam [5:0] ST_ABORT_WAIT        = 6'd39;
	localparam [5:0] ST_CAPTURE_DONE      = 6'd40;
	localparam [5:0] ST_PAYLOAD_DONE      = 6'd41;

	localparam [15:0] MAX_FILE_SECTORS = 16'd16386;

	reg [5:0] state_q;
	reg       operation_import_q;
	reg       snapshot_acquired_q;
	reg       pause_hold_q;

	reg       committed_valid_q;
	reg       committed_directory_lba_q;
	reg [31:0] committed_generation_q;
	reg [34:0] committed_file0_q, committed_file1_q;
	reg [34:0] committed_active0_q, committed_active1_q;
	reg [15:0] committed_next_free_q;

	reg       import_directory_lba_q;
	reg [31:0] import_generation_q;
	reg [34:0] import_file0_q, import_file1_q;
	reg [34:0] import_active0_q, import_active1_q;
	reg [15:0] import_next_free_q;
	reg [6:0] import_expect_q;

	reg [34:0] proposed_live0_q, proposed_live1_q;
	reg [34:0] proposed_active0_q, proposed_active1_q;
	reg [15:0] proposed_next_free_q;
	reg [31:0] proposed_generation_q;
	reg       proposed_directory_lba_q;
	reg [34:0] capture0_q, capture1_q;
	reg [34:0] payload0_q, payload1_q;
	reg [34:0] first_alloc0_q, first_alloc1_q;
	reg [34:0] pristine0_q, pristine1_q;

	reg       scan_die_q;
	reg [5:0] scan_block_q;
	reg [6:0] table_index_q;
	reg [47:0] entry_latch_q;
	reg [15:0] selected_base_q;
	reg       stale_retry_q;

	reg [15:0] compare_word_q;
	reg [15:0] compare_stage_word_q;
	reg [31:0] compare_crc_q;
	reg        compare_equal_q;

	reg [7:0] sector_word_q;
	reg [31:0] marker_crc_q;
	reg       marker_lba_q;
	reg [31:0] sector_lba_q;
	reg [7:0] payload_sector_q;
	reg [7:0] payload_sector_count_q;
	reg       payload_copy_q;
	reg [15:0] payload_word_q;
	reg [15:0] payload_data_q;
	reg [6:0] adopt_index_q;
	reg       adopt_import_q;

	function automatic [31:0] crc32_byte;
		input [31:0] crc_i;
		input [7:0] data_i;
		reg [31:0] crc_v;
		integer bit_i;
		begin
			crc_v = crc_i ^ {24'd0, data_i};
			for (bit_i = 0; bit_i < 8; bit_i = bit_i + 1) begin
				if (crc_v[0]) crc_v = (crc_v >> 1) ^ 32'hEDB88320;
				else          crc_v = crc_v >> 1;
			end
			crc32_byte = crc_v;
		end
	endfunction

	function automatic [31:0] crc32_word;
		input [31:0] crc_i;
		input [15:0] data_i;
		begin
			crc32_word = crc32_byte(crc32_byte(crc_i, data_i[7:0]), data_i[15:8]);
		end
	endfunction

	function automatic [34:0] die_mask;
		input [1:0] code_i;
		begin
			case (code_i)
				2'd1: die_mask = 35'h0000007FF;
				2'd2: die_mask = 35'h00007FFFF;
				2'd3: die_mask = 35'h7FFFFFFFF;
				default: die_mask = 35'd0;
			endcase
		end
	endfunction

	function automatic [21:0] die_bytes;
		input [1:0] code_i;
		begin
			case (code_i)
				2'd1: die_bytes = 22'h080000;
				2'd2: die_bytes = 22'h100000;
				2'd3: die_bytes = 22'h200000;
				default: die_bytes = 22'd0;
			endcase
		end
	endfunction

	function automatic [15:0] marker_word;
		input [7:0] index_i;
		begin
			case (index_i)
				8'd0: marker_word = 16'h474E; // "NG"
				8'd1: marker_word = 16'h4950; // "PI"
				8'd2: marker_word = 16'h434E; // "NC"
				8'd3: marker_word = 16'h0031; // "1\0"
				8'd4: marker_word = 16'h0001;
				default: marker_word = 16'd0;
			endcase
		end
	endfunction

	function automatic [15:0] select_ddr_word;
		input [63:0] value_i;
		input [1:0] lane_i;
		begin
			case (lane_i)
				2'd0: select_ddr_word = value_i[15:0];
				2'd1: select_ddr_word = value_i[31:16];
				2'd2: select_ddr_word = value_i[47:32];
				default: select_ddr_word = value_i[63:48];
			endcase
		end
	endfunction

	wire [34:0] valid_mask0_w = die_mask(identity_die0_code_i);
	wire [34:0] valid_mask1_w = die_mask(identity_die1_code_i);
	wire [21:0] identity_die0_bytes_w = die_bytes(identity_die0_code_i);
	wire [21:0] identity_die1_bytes_w = die_bytes(identity_die1_code_i);
	wire [22:0] identity_total_bytes_w = {1'b0, identity_die0_bytes_w} +
	                                     {1'b0, identity_die1_bytes_w};
	wire identity_valid_w = (identity_raw_bytes_i != 32'd0) &&
	                        (identity_raw_bytes_i <= identity_physical_bytes_i) &&
	                        (identity_physical_bytes_i == {9'd0, identity_total_bytes_w});
	wire live_maps_valid_w = ((live0_i & ~valid_mask0_w) == 35'd0) &&
	                         ((live1_i & ~valid_mask1_w) == 35'd0) &&
	                         ((pending0_i & ~valid_mask0_w) == 35'd0) &&
	                         ((pending1_i & ~valid_mask1_w) == 35'd0);
	wire import_maps_valid_w = ((import_file0_i & ~valid_mask0_w) == 35'd0) &&
	                           ((import_file1_i & ~valid_mask1_w) == 35'd0) &&
	                           ((import_active0_i & ~valid_mask0_w) == 35'd0) &&
	                           ((import_active1_i & ~valid_mask1_w) == 35'd0) &&
	                           (import_next_free_lba_i >= 16'd2) &&
	                           (import_next_free_lba_i <= MAX_FILE_SECTORS);
	wire save_work_w = directory_dirty_i || (pending0_i != 35'd0) ||
	                   (pending1_i != 35'd0) ||
	                   (live0_i != committed_file0_q) ||
	                   (live1_i != committed_file1_q);

	assign busy_o = (state_q != ST_IDLE);
	assign pause_req_o = pause_hold_q;
	assign commit_file0_o = proposed_live0_q;
	assign commit_file1_o = proposed_live1_q;
	assign pristine0_o = pristine0_q;
	assign pristine1_o = pristine1_q;
	assign import_ready_o = (state_q == ST_IMPORT_ENTRIES);

	wire [6:0] scan_entry_index_w = scan_die_q ?
		(7'd35 + {1'b0, scan_block_q}) : {1'b0, scan_block_q};
	wire selected_capture_w = scan_die_q ? capture1_q[scan_block_q] :
	                                             capture0_q[scan_block_q];
	wire selected_live_w = scan_die_q ? proposed_live1_q[scan_block_q] :
	                                          proposed_live0_q[scan_block_q];
	wire selected_payload_w = scan_die_q ? payload1_q[scan_block_q] :
	                                             payload0_q[scan_block_q];
	wire selected_first_alloc_w = scan_die_q ? first_alloc1_q[scan_block_q] :
	                                                 first_alloc0_q[scan_block_q];
	wire selected_committed_file_w = scan_die_q ? committed_file1_q[scan_block_q] :
	                                                   committed_file0_q[scan_block_q];
	wire selected_committed_active_w = scan_die_q ? committed_active1_q[scan_block_q] :
	                                                     committed_active0_q[scan_block_q];
	wire selected_proposed_active_w = scan_die_q ? proposed_active1_q[scan_block_q] :
	                                                   proposed_active0_q[scan_block_q];

	wire geometry_valid_w;
	wire [20:0] geometry_base_w;
	wire [16:0] geometry_bytes_w;
	wire [15:0] geometry_words_w;
	wire [1:0] selected_size_code_w = scan_die_q ? identity_die1_code_i :
	                                                   identity_die0_code_i;

	ngp_cart_overlay_geometry u_geometry
	(
		.size_code_i(selected_size_code_w), .block_i(scan_block_q),
		.valid_o(geometry_valid_w), .base_o(geometry_base_w),
		.bytes_o(geometry_bytes_w), .words_o(geometry_words_w)
	);

	wire [23:0] cart_word_base_w = (scan_die_q ? 24'h100000 : 24'd0) +
	                                  {4'd0, geometry_base_w[20:1]};
	wire [23:0] staging_word_base_w = 24'h200000 + cart_word_base_w;
	wire [15:0] geometry_sectors_w = {8'd0, geometry_bytes_w[16:9]};
	wire [16:0] new_allocation_end_w = {1'b0, proposed_next_free_q} +
	                                       {1'b0, geometry_sectors_w} +
	                                       {1'b0, geometry_sectors_w};

	// Committed allocation table. It is written only after an import commit or
	// after the inactive on-disk directory has completed.
	wire committed_ram_we_w = (state_q == ST_ADOPT_WRITE);
	wire [6:0] committed_ram_addr_a_w = adopt_index_q;
	wire [47:0] committed_ram_data_a_w;
	wire [6:0] committed_ram_addr_b_w = table_index_q;
	wire [47:0] committed_ram_q_b_w;
	/* verilator lint_off UNUSED */
	wire [47:0] committed_ram_q_a_unused_w;
	/* verilator lint_on UNUSED */

	cache_ram_dp_be #(.ADDR_WIDTH(7), .DATA_WIDTH(48)) u_committed_entries
	(
		.clk_i(clk), .addr_a_i(committed_ram_addr_a_w),
		.wren_a_i(committed_ram_we_w), .be_a_i(6'b111111),
		.wdata_a_i(committed_ram_data_a_w), .q_a_o(committed_ram_q_a_unused_w),
		.addr_b_i(committed_ram_addr_b_w), .wren_b_i(1'b0),
		.be_b_i(6'd0), .wdata_b_i(48'd0), .q_b_o(committed_ram_q_b_w)
	);

	// Proposed directory store and byte-exact V1 builder.
	reg        codec_entry_we_w;
	reg [6:0]  codec_entry_index_w;
	reg [15:0] codec_entry_base_w;
	reg [31:0] codec_entry_crc_w;
	reg [6:0]  codec_meta_index_w;
	wire [15:0] codec_meta_base_w;
	wire [31:0] codec_meta_crc_w;
	wire codec_build_rejected_w;
	wire codec_emit_valid_w;
	wire [7:0] codec_emit_index_w;
	wire [15:0] codec_emit_word_w;
	wire codec_emit_done_w;
	wire codec_build_start_w = (state_q == ST_DIR_BUILD_START);
	wire codec_emit_ready_w = (state_q == ST_DIR_FILL) && sector_buf_wready_i;
	/* verilator lint_off UNUSED */
	wire codec_sector_ready_unused_w, codec_accepted_unused_w;
	wire codec_rejected_unused_w, codec_valid_unused_w;
	wire [31:0] codec_generation_unused_w;
	wire [7:0] codec_flags_unused_w;
	wire [34:0] codec_ledger0_unused_w, codec_ledger1_unused_w;
	wire [34:0] codec_active0_unused_w, codec_active1_unused_w;
	wire [15:0] codec_next_free_unused_w;
	/* verilator lint_on UNUSED */

	ngp_cart_overlay_directory u_proposed_directory
	(
		.clk(clk), .reset(reset),
		.identity_raw_crc32_i(identity_raw_crc32_i),
		.identity_raw_bytes_i(identity_raw_bytes_i),
		.identity_pristine_crc32_i(identity_pristine_crc32_i),
		.identity_physical_bytes_i(identity_physical_bytes_i),
		.identity_die0_code_i(identity_die0_code_i),
		.identity_die1_code_i(identity_die1_code_i),
		.identity_catalog_i(identity_catalog_i),
		.identity_subcatalog_i(identity_subcatalog_i),
		.identity_title_i(identity_title_i),
		.file_sectors_valid_i(1'b0), .file_sectors_i(16'd0),
		.sector_start_i(1'b0), .sector_word_valid_i(1'b0),
		.sector_word_i(16'd0), .sector_word_ready_o(codec_sector_ready_unused_w),
		.accepted_o(codec_accepted_unused_w), .rejected_o(codec_rejected_unused_w),
		.directory_valid_o(codec_valid_unused_w),
		.generation_o(codec_generation_unused_w), .flags_o(codec_flags_unused_w),
		.ledger0_o(codec_ledger0_unused_w), .ledger1_o(codec_ledger1_unused_w),
		.active0_o(codec_active0_unused_w), .active1_o(codec_active1_unused_w),
		.next_free_lba_o(codec_next_free_unused_w),
		.entry_config_we_i(codec_entry_we_w),
		.entry_config_index_i(codec_entry_index_w),
		.entry_config_base_lba_i(codec_entry_base_w),
		.entry_config_crc32_i(codec_entry_crc_w),
		.entry_meta_index_i(codec_meta_index_w),
		.entry_meta_base_lba_o(codec_meta_base_w),
		.entry_meta_crc32_o(codec_meta_crc_w),
		.build_start_i(codec_build_start_w),
		.build_generation_i(proposed_generation_q), .build_flags_i(8'd0),
		.build_ledger0_i(proposed_live0_q), .build_ledger1_i(proposed_live1_q),
		.build_active0_i(proposed_active0_q), .build_active1_i(proposed_active1_q),
		.build_next_free_lba_i(proposed_next_free_q),
		.build_rejected_o(codec_build_rejected_w),
		.emit_valid_o(codec_emit_valid_w), .emit_word_index_o(codec_emit_index_w),
		.emit_word_o(codec_emit_word_w), .emit_ready_i(codec_emit_ready_w),
		.emit_done_o(codec_emit_done_w)
	);

	assign committed_ram_data_a_w = {codec_meta_crc_w, codec_meta_base_w};

	// One bounded live-to-staging mover. Stale captures never reach media.
	wire mover_start_w = (state_q == ST_CAPTURE_START);
	wire mover_busy_w, mover_done_w, mover_stale_w, mover_rejected_w;
	wire mover_p2_req_w, mover_p2_we_w;
	wire [24:0] mover_p2_addr_w;
	wire [15:0] mover_p2_wdata_w;
	wire [1:0] mover_p2_be_w;
	wire [27:1] mover_ddr_addr_w;
	wire [63:0] mover_ddr_din_w;
	wire mover_ddr_req_w, mover_ddr_rnw_w;
	wire [7:0] mover_ddr_be_w;

	ngp_cart_overlay_mover u_mover
	(
		.clk(clk), .reset(reset), .start_i(mover_start_w), .direction_i(1'b0),
		.die_i(scan_die_q), .block_i(scan_block_q),
		.size_code_i(selected_size_code_w), .ddr_word_base_i(staging_word_base_w),
		.die_busy_i(die_busy_i), .event0_i(event0_i), .block0_i(block0_i),
		.event1_i(event1_i), .block1_i(block1_i),
		.busy_o(mover_busy_w), .done_o(mover_done_w), .stale_o(mover_stale_w),
		.rejected_o(mover_rejected_w),
		.p2_req_o(mover_p2_req_w), .p2_we_o(mover_p2_we_w),
		.p2_addr_o(mover_p2_addr_w), .p2_wdata_o(mover_p2_wdata_w),
		.p2_be_o(mover_p2_be_w), .p2_ready_i(p2_ready_i),
		.p2_done_i(p2_done_i), .p2_rdata_i(p2_rdata_i),
		.ddr_addr_o(mover_ddr_addr_w), .ddr_din_o(mover_ddr_din_w),
		.ddr_req_o(mover_ddr_req_w), .ddr_rnw_o(mover_ddr_rnw_w),
		.ddr_be_o(mover_ddr_be_w), .ddr_dout_i(ddr_dout_i),
		.ddr_ready_i(ddr_ready_i)
	);

	assign p2_req_o = mover_p2_req_w;
	assign p2_we_o = mover_p2_we_w;
	assign p2_addr_o = mover_p2_addr_w;
	assign p2_wdata_o = mover_p2_wdata_w;
	assign p2_be_o = mover_p2_be_w;

	wire mover_owner_w = mover_busy_w || (state_q == ST_CAPTURE_START) ||
	                                      (state_q == ST_CAPTURE_WAIT);
	wire [23:0] compare_stage_address_w = staging_word_base_w +
	                                           {8'd0, compare_word_q};
	wire [23:0] compare_shadow_address_w = cart_word_base_w +
	                                            {8'd0, compare_word_q};
	wire [23:0] payload_ddr_address_w = staging_word_base_w +
		{8'd0, payload_sector_q[6:0], 8'd0} + {8'd0, payload_word_q};
	wire [23:0] controller_ddr_word_w = (state_q == ST_COMPARE_STAGE) ?
		compare_stage_address_w : (state_q == ST_COMPARE_SHADOW) ?
		compare_shadow_address_w : payload_ddr_address_w;
	wire controller_ddr_req_w = (state_q == ST_COMPARE_STAGE) ||
	                            (state_q == ST_COMPARE_SHADOW) ||
	                            (state_q == ST_PAYLOAD_DDR_READ);

	assign ddr_addr_o = mover_owner_w ? mover_ddr_addr_w :
		{3'd0, controller_ddr_word_w[23:2], 2'b00};
	assign ddr_din_o = mover_owner_w ? mover_ddr_din_w : 64'd0;
	assign ddr_req_o = mover_owner_w ? mover_ddr_req_w : controller_ddr_req_w;
	assign ddr_rnw_o = mover_owner_w ? mover_ddr_rnw_w : 1'b1;
	assign ddr_be_o = mover_owner_w ? mover_ddr_be_w : 8'd0;

	wire [15:0] compare_shadow_word_w = select_ddr_word(
		ddr_dout_i, compare_shadow_address_w[1:0]);
	wire [31:0] compare_crc_next_w = crc32_word(compare_crc_q, compare_stage_word_q);
	wire compare_equal_next_w = compare_equal_q &&
	                            (compare_stage_word_q == compare_shadow_word_w);
	wire compare_last_w = compare_word_q == (geometry_words_w - 16'd1);
	wire compare_allocation_ok_w = compare_equal_next_w ||
	                               (selected_base_q != 16'd0) ||
	                               (new_allocation_end_w <= {1'b0, MAX_FILE_SECTORS});
	wire [15:0] compare_new_base_w = (selected_base_q == 16'd0) ?
	                                     proposed_next_free_q : selected_base_q;
	wire compare_new_active_w = (selected_base_q == 16'd0) ? 1'b0 :
	                            (selected_committed_file_w ?
	                             ~selected_committed_active_w : 1'b0);

	// Proposed-entry writes are restricted to idle codec periods.
	always @* begin
		codec_entry_we_w = 1'b0;
		codec_entry_index_w = 7'd0;
		codec_entry_base_w = 16'd0;
		codec_entry_crc_w = 32'd0;

		if ((state_q == ST_IMPORT_ENTRIES) && import_entry_valid_i &&
		    (import_entry_index_i == import_expect_q)) begin
			codec_entry_we_w = 1'b1;
			codec_entry_index_w = import_entry_index_i;
			codec_entry_base_w = import_entry_base_lba_i;
			codec_entry_crc_w = import_entry_crc32_i;
		end else if (state_q == ST_PROP_COPY_WRITE) begin
			codec_entry_we_w = 1'b1;
			codec_entry_index_w = table_index_q;
			codec_entry_base_w = entry_latch_q[15:0];
			codec_entry_crc_w = entry_latch_q[47:16];
		end else if ((state_q == ST_SCRUB_CHECK) && !selected_live_w &&
		             (codec_meta_base_w != 16'd0)) begin
			codec_entry_we_w = 1'b1;
			codec_entry_index_w = scan_entry_index_w;
			codec_entry_base_w = codec_meta_base_w;
			codec_entry_crc_w = 32'd0;
		end else if ((state_q == ST_COMPARE_SHADOW) && ddr_ready_i && compare_last_w &&
		             compare_allocation_ok_w) begin
			codec_entry_we_w = 1'b1;
			codec_entry_index_w = scan_entry_index_w;
			codec_entry_base_w = compare_equal_next_w ? selected_base_q :
			                         compare_new_base_w;
			codec_entry_crc_w = compare_equal_next_w ? 32'd0 : ~compare_crc_next_w;
		end
	end

	always @* begin
		case (state_q)
			ST_SCRUB_REQ, ST_SCRUB_CHECK,
			ST_CAPTURE_ENTRY_REQ, ST_CAPTURE_ENTRY_GET,
			ST_PAYLOAD_ENTRY_REQ, ST_PAYLOAD_ENTRY_GET:
				codec_meta_index_w = scan_entry_index_w;
			ST_ADOPT_REQ, ST_ADOPT_WRITE:
				codec_meta_index_w = adopt_index_q;
			default: codec_meta_index_w = 7'd0;
		endcase
	end

	// Sector-buffer sources.
	wire marker_fill_w = (state_q == ST_MARK_FILL);
	wire payload_buf_write_w = (state_q == ST_PAYLOAD_BUF_WRITE);
	wire directory_fill_w = (state_q == ST_DIR_FILL);
	wire [15:0] marker_data_w = (sector_word_q < 8'd254) ? marker_word(sector_word_q) :
		(sector_word_q == 8'd254) ? ~marker_crc_q[15:0] : ~marker_crc_q[31:16];

	assign sector_fill_start_o = ((state_q == ST_MARK_FILL_START) ||
		(state_q == ST_PAYLOAD_FILL_START) || (state_q == ST_DIR_FILL_START)) &&
		sector_fill_ready_i;
	assign sector_buf_addr_o = marker_fill_w ? sector_word_q :
		payload_buf_write_w ? payload_word_q[7:0] : codec_emit_index_w;
	assign sector_buf_wr_o = (marker_fill_w && sector_buf_wready_i) ||
		(payload_buf_write_w && sector_buf_wready_i) ||
		(directory_fill_w && codec_emit_valid_w && sector_buf_wready_i);
	assign sector_buf_wdata_o = marker_fill_w ? marker_data_w :
		payload_buf_write_w ? payload_data_q : codec_emit_word_w;
	assign sector_lba_o = sector_lba_q;
	assign sector_save_req_o = (state_q == ST_MARK_SAVE_REQ) ||
		(state_q == ST_PAYLOAD_SAVE_REQ) || (state_q == ST_DIR_SAVE_REQ);

	// Advance one deterministic die/block scan position.
	task automatic advance_scan;
		begin
			if (!scan_die_q && (scan_block_q == 6'd34)) begin
				scan_die_q <= 1'b1;
				scan_block_q <= 6'd0;
			end else begin
				scan_block_q <= scan_block_q + 6'd1;
			end
		end
	endtask

	always @(posedge clk) begin
		done_o <= 1'b0;
		rejected_o <= 1'b0;
		import_done_o <= 1'b0;
		import_rejected_o <= 1'b0;
		snapshot_req_o <= 1'b0;
		snapshot_abort_o <= 1'b0;
		commit_o <= 1'b0;

		if (reset) begin
			state_q <= ST_IDLE;
			operation_import_q <= 1'b0;
			snapshot_acquired_q <= 1'b0;
			pause_hold_q <= 1'b0;
			committed_valid_q <= 1'b0;
			committed_directory_lba_q <= 1'b0;
			committed_generation_q <= 32'd0;
			committed_file0_q <= 35'd0;
			committed_file1_q <= 35'd0;
			committed_active0_q <= 35'd0;
			committed_active1_q <= 35'd0;
			committed_next_free_q <= 16'd2;
			import_directory_lba_q <= 1'b0;
			import_generation_q <= 32'd0;
			import_file0_q <= 35'd0;
			import_file1_q <= 35'd0;
			import_active0_q <= 35'd0;
			import_active1_q <= 35'd0;
			import_next_free_q <= 16'd2;
			import_expect_q <= 7'd0;
			proposed_live0_q <= 35'd0;
			proposed_live1_q <= 35'd0;
			proposed_active0_q <= 35'd0;
			proposed_active1_q <= 35'd0;
			proposed_next_free_q <= 16'd2;
			proposed_generation_q <= 32'd0;
			proposed_directory_lba_q <= 1'b0;
			capture0_q <= 35'd0;
			capture1_q <= 35'd0;
			payload0_q <= 35'd0;
			payload1_q <= 35'd0;
			first_alloc0_q <= 35'd0;
			first_alloc1_q <= 35'd0;
			pristine0_q <= 35'd0;
			pristine1_q <= 35'd0;
			scan_die_q <= 1'b0;
			scan_block_q <= 6'd0;
			table_index_q <= 7'd0;
			entry_latch_q <= 48'd0;
			selected_base_q <= 16'd0;
			stale_retry_q <= 1'b0;
			compare_word_q <= 16'd0;
			compare_stage_word_q <= 16'd0;
			compare_crc_q <= 32'hFFFFFFFF;
			compare_equal_q <= 1'b1;
			sector_word_q <= 8'd0;
			marker_crc_q <= 32'hFFFFFFFF;
			marker_lba_q <= 1'b0;
			sector_lba_q <= 32'd0;
			payload_sector_q <= 8'd0;
			payload_sector_count_q <= 8'd0;
			payload_copy_q <= 1'b0;
			payload_word_q <= 16'd0;
			payload_data_q <= 16'd0;
			adopt_index_q <= 7'd0;
			adopt_import_q <= 1'b0;
		end else begin
			if ((state_q == ST_WAIT_SNAPSHOT) && snapshot_active_i)
				snapshot_acquired_q <= 1'b1;

			case (state_q)
				ST_IDLE: begin
					pause_hold_q <= 1'b0;
					snapshot_acquired_q <= 1'b0;
					if (import_start_i) begin
						operation_import_q <= 1'b1;
						if (!identity_valid_w || !import_maps_valid_w) begin
							import_rejected_o <= 1'b1;
						end else begin
							import_directory_lba_q <= import_directory_lba_i;
							import_generation_q <= import_generation_i;
							import_file0_q <= import_file0_i;
							import_file1_q <= import_file1_i;
							import_active0_q <= import_active0_i;
							import_active1_q <= import_active1_i;
							import_next_free_q <= import_next_free_lba_i;
							import_expect_q <= 7'd0;
							state_q <= ST_IMPORT_ENTRIES;
						end
					end else if (start_i) begin
						operation_import_q <= 1'b0;
						if (!identity_valid_w || !live_maps_valid_w ||
						    snapshot_active_i ||
						    (committed_valid_q && (committed_generation_q == 32'hFFFFFFFF))) begin
							rejected_o <= 1'b1;
						end else if (!save_work_w) begin
							done_o <= 1'b1;
						end else begin
							snapshot_req_o <= 1'b1;
							state_q <= ST_WAIT_SNAPSHOT;
						end
					end
				end

				ST_IMPORT_ENTRIES: begin
					if (import_entry_valid_i) begin
						if (import_entry_index_i != import_expect_q) begin
							state_q <= ST_ABORT_WAIT;
						end else if (import_expect_q == 7'd69) begin
							state_q <= ST_IMPORT_READY;
						end else begin
							import_expect_q <= import_expect_q + 7'd1;
						end
					end
				end

				ST_IMPORT_READY: begin
					if (import_commit_i) begin
						adopt_import_q <= 1'b1;
						adopt_index_q <= 7'd0;
						state_q <= ST_ADOPT_REQ;
					end
				end

				ST_WAIT_SNAPSHOT: begin
					if (snapshot_active_i) begin
						proposed_live0_q <= live0_i;
						proposed_live1_q <= live1_i;
						proposed_active0_q <= committed_valid_q ? committed_active0_q : 35'd0;
						proposed_active1_q <= committed_valid_q ? committed_active1_q : 35'd0;
						proposed_next_free_q <= committed_valid_q ? committed_next_free_q : 16'd2;
						proposed_generation_q <= committed_valid_q ?
							(committed_generation_q + 32'd1) : 32'd1;
						proposed_directory_lba_q <= committed_valid_q ?
							~committed_directory_lba_q : 1'b0;
						capture0_q <= snapshot0_i |
							(live0_i & ~(committed_valid_q ? committed_file0_q : 35'd0));
						capture1_q <= snapshot1_i |
							(live1_i & ~(committed_valid_q ? committed_file1_q : 35'd0));
						payload0_q <= 35'd0;
						payload1_q <= 35'd0;
						first_alloc0_q <= 35'd0;
						first_alloc1_q <= 35'd0;
						pristine0_q <= 35'd0;
						pristine1_q <= 35'd0;
						table_index_q <= 7'd0;
						state_q <= ST_PROP_COPY_REQ;
					end
				end

				ST_PROP_COPY_REQ: state_q <= ST_PROP_COPY_LATCH;

				ST_PROP_COPY_LATCH: begin
					entry_latch_q <= committed_valid_q ? committed_ram_q_b_w : 48'd0;
					state_q <= ST_PROP_COPY_WRITE;
				end

				ST_PROP_COPY_WRITE: begin
					if (table_index_q == 7'd69) begin
						scan_die_q <= 1'b0;
						scan_block_q <= 6'd0;
						state_q <= ST_SCRUB_REQ;
					end else begin
						table_index_q <= table_index_q + 7'd1;
						state_q <= ST_PROP_COPY_REQ;
					end
				end

				ST_SCRUB_REQ: state_q <= ST_SCRUB_CHECK;

				ST_SCRUB_CHECK: begin
					if (!selected_live_w) begin
						if (scan_die_q) proposed_active1_q[scan_block_q] <= 1'b0;
						else            proposed_active0_q[scan_block_q] <= 1'b0;
					end
					if (scan_die_q && (scan_block_q == 6'd34)) begin
						scan_die_q <= 1'b0;
						scan_block_q <= 6'd0;
						state_q <= ST_CAPTURE_SCAN;
					end else begin
						advance_scan;
						state_q <= ST_SCRUB_REQ;
					end
				end

				ST_CAPTURE_SCAN: begin
					if (selected_capture_w) begin
						if (!selected_live_w) begin
							if (scan_die_q) begin
								capture1_q[scan_block_q] <= 1'b0;
								pristine1_q[scan_block_q] <= 1'b1;
							end else begin
								capture0_q[scan_block_q] <= 1'b0;
								pristine0_q[scan_block_q] <= 1'b1;
							end
							if (scan_die_q && (scan_block_q == 6'd34))
								state_q <= ST_CAPTURE_DONE;
							else
								advance_scan;
						end else if (!geometry_valid_w) begin
							state_q <= ST_ABORT_WAIT;
						end else begin
							state_q <= ST_CAPTURE_ENTRY_REQ;
						end
					end else if (scan_die_q && (scan_block_q == 6'd34)) begin
						state_q <= ST_CAPTURE_DONE;
					end else begin
						advance_scan;
					end
				end

				ST_CAPTURE_DONE: begin
					pause_hold_q <= 1'b0;
					// A fresh mount can receive real BIOS flash traffic that
					// ultimately restores the complete block to its pristine
					// bytes. Retire that captured generation in the ledger, but
					// do not create an empty two-sector save file. An existing
					// directory still needs a metadata commit when its last
					// payload is removed.
					if (!committed_valid_q &&
					    (proposed_live0_q == 35'd0) &&
					    (proposed_live1_q == 35'd0)) begin
						commit_o <= 1'b1;
						done_o <= 1'b1;
						snapshot_acquired_q <= 1'b0;
						state_q <= ST_IDLE;
					end else if (!committed_valid_q) begin
						marker_lba_q <= 1'b0;
						sector_lba_q <= 32'd0;
						state_q <= ST_MARK_FILL_START;
					end else begin
						scan_die_q <= 1'b0;
						scan_block_q <= 6'd0;
						state_q <= ST_PAYLOAD_SCAN;
					end
				end

				ST_CAPTURE_ENTRY_REQ: state_q <= ST_CAPTURE_ENTRY_GET;

				ST_CAPTURE_ENTRY_GET: begin
					selected_base_q <= codec_meta_base_w;
					stale_retry_q <= 1'b0;
					state_q <= ST_CAPTURE_WAIT_IDLE;
				end

				ST_CAPTURE_WAIT_IDLE: begin
					if ((!scan_die_q && !die_busy_i[0]) ||
					    (scan_die_q && !die_busy_i[1])) begin
						if (!pause_hold_q || pause_ready_i)
							state_q <= ST_CAPTURE_START;
					end
				end

				ST_CAPTURE_START: state_q <= ST_CAPTURE_WAIT;

				ST_CAPTURE_WAIT: begin
					if (mover_rejected_w) begin
						state_q <= ST_ABORT_WAIT;
					end else if (mover_done_w) begin
						if (mover_stale_w) begin
							if (stale_retry_q) pause_hold_q <= 1'b1;
							else               stale_retry_q <= 1'b1;
							state_q <= ST_CAPTURE_WAIT_IDLE;
						end else begin
							compare_word_q <= 16'd0;
							compare_crc_q <= 32'hFFFFFFFF;
							compare_equal_q <= 1'b1;
							state_q <= ST_COMPARE_STAGE;
						end
					end
				end

				ST_COMPARE_STAGE: begin
					if (ddr_ready_i) begin
						compare_stage_word_q <= select_ddr_word(
							ddr_dout_i, compare_stage_address_w[1:0]);
						state_q <= ST_COMPARE_SHADOW;
					end
				end

				ST_COMPARE_SHADOW: begin
					if (ddr_ready_i) begin
						compare_crc_q <= compare_crc_next_w;
						compare_equal_q <= compare_equal_next_w;
						if (compare_last_w) begin
							if (!compare_allocation_ok_w) begin
								state_q <= ST_ABORT_WAIT;
							end else begin
								if (scan_die_q) begin
									capture1_q[scan_block_q] <= 1'b0;
									proposed_live1_q[scan_block_q] <= !compare_equal_next_w;
									pristine1_q[scan_block_q] <= compare_equal_next_w;
									payload1_q[scan_block_q] <= !compare_equal_next_w;
									proposed_active1_q[scan_block_q] <= compare_equal_next_w ?
										1'b0 : compare_new_active_w;
									if (!compare_equal_next_w && (selected_base_q == 16'd0))
										first_alloc1_q[scan_block_q] <= 1'b1;
								end else begin
									capture0_q[scan_block_q] <= 1'b0;
									proposed_live0_q[scan_block_q] <= !compare_equal_next_w;
									pristine0_q[scan_block_q] <= compare_equal_next_w;
									payload0_q[scan_block_q] <= !compare_equal_next_w;
									proposed_active0_q[scan_block_q] <= compare_equal_next_w ?
										1'b0 : compare_new_active_w;
									if (!compare_equal_next_w && (selected_base_q == 16'd0))
										first_alloc0_q[scan_block_q] <= 1'b1;
								end
								if (!compare_equal_next_w && (selected_base_q == 16'd0))
									proposed_next_free_q <= new_allocation_end_w[15:0];
								pause_hold_q <= 1'b0;
								if (scan_die_q && (scan_block_q == 6'd34)) begin
									state_q <= ST_CAPTURE_DONE;
								end else begin
									advance_scan;
									state_q <= ST_CAPTURE_SCAN;
								end
							end
						end else begin
							compare_word_q <= compare_word_q + 16'd1;
							state_q <= ST_COMPARE_STAGE;
						end
					end
				end

				ST_MARK_FILL_START: begin
					if (sector_fill_ready_i) begin
						sector_word_q <= 8'd0;
						marker_crc_q <= 32'hFFFFFFFF;
						state_q <= ST_MARK_FILL;
					end
				end

				ST_MARK_FILL: begin
					if (sector_fill_fault_i) begin
						state_q <= ST_ABORT_WAIT;
					end else if (sector_buf_wready_i) begin
						if (sector_word_q < 8'd254)
							marker_crc_q <= crc32_word(marker_crc_q, marker_data_w);
						if (sector_word_q == 8'hFF) state_q <= ST_MARK_SAVE_REQ;
						else sector_word_q <= sector_word_q + 8'd1;
					end
				end

				ST_MARK_SAVE_REQ: begin
					if (sector_fill_fault_i) state_q <= ST_ABORT_WAIT;
					else if (sector_save_ready_i && sector_buf_full_i)
						state_q <= ST_MARK_WAIT_BUSY;
				end

				ST_MARK_WAIT_BUSY: if (sector_busy_i) state_q <= ST_MARK_WAIT_DONE;

				ST_MARK_WAIT_DONE: begin
					if (!sector_busy_i) begin
						if (!marker_lba_q) begin
							marker_lba_q <= 1'b1;
							sector_lba_q <= 32'd1;
							state_q <= ST_MARK_FILL_START;
						end else begin
							scan_die_q <= 1'b0;
							scan_block_q <= 6'd0;
							state_q <= ST_PAYLOAD_SCAN;
						end
					end
				end

				ST_PAYLOAD_SCAN: begin
					if (selected_payload_w) begin
						if (!geometry_valid_w) state_q <= ST_ABORT_WAIT;
						else state_q <= ST_PAYLOAD_ENTRY_REQ;
					end else if (scan_die_q && (scan_block_q == 6'd34)) begin
						state_q <= ST_PAYLOAD_DONE;
					end else begin
						advance_scan;
					end
				end

				ST_PAYLOAD_DONE: state_q <= ST_DIR_FILL_START;

				ST_PAYLOAD_ENTRY_REQ: state_q <= ST_PAYLOAD_ENTRY_GET;

				ST_PAYLOAD_ENTRY_GET: begin
					selected_base_q <= codec_meta_base_w;
					payload_sector_count_q <= geometry_bytes_w[16:9];
					payload_sector_q <= 8'd0;
					payload_copy_q <= selected_first_alloc_w ? 1'b0 :
					                  selected_proposed_active_w;
					sector_lba_q <= {16'd0, codec_meta_base_w} +
						((selected_first_alloc_w || !selected_proposed_active_w) ?
						 32'd0 : {24'd0, geometry_bytes_w[16:9]});
					state_q <= ST_PAYLOAD_FILL_START;
				end

				ST_PAYLOAD_FILL_START: begin
					if (sector_fill_ready_i) begin
						payload_word_q <= 16'd0;
						state_q <= ST_PAYLOAD_DDR_READ;
					end
				end

				ST_PAYLOAD_DDR_READ: begin
					if (ddr_ready_i) begin
						payload_data_q <= select_ddr_word(
							ddr_dout_i, payload_ddr_address_w[1:0]);
						state_q <= ST_PAYLOAD_BUF_WRITE;
					end
				end

				ST_PAYLOAD_BUF_WRITE: begin
					if (sector_fill_fault_i) begin
						state_q <= ST_ABORT_WAIT;
					end else if (sector_buf_wready_i) begin
						if (payload_word_q == 16'd255) state_q <= ST_PAYLOAD_SAVE_REQ;
						else begin
							payload_word_q <= payload_word_q + 16'd1;
							state_q <= ST_PAYLOAD_DDR_READ;
						end
					end
				end

				ST_PAYLOAD_SAVE_REQ: begin
					if (sector_fill_fault_i) state_q <= ST_ABORT_WAIT;
					else if (sector_save_ready_i && sector_buf_full_i)
						state_q <= ST_PAYLOAD_WAIT_BUSY;
				end

				ST_PAYLOAD_WAIT_BUSY: if (sector_busy_i) state_q <= ST_PAYLOAD_WAIT_DONE;

				ST_PAYLOAD_WAIT_DONE: begin
					if (!sector_busy_i) begin
						if (payload_sector_q != (payload_sector_count_q - 8'd1)) begin
							payload_sector_q <= payload_sector_q + 8'd1;
							sector_lba_q <= sector_lba_q + 32'd1;
							state_q <= ST_PAYLOAD_FILL_START;
						end else if (selected_first_alloc_w && !payload_copy_q) begin
							payload_copy_q <= 1'b1;
							payload_sector_q <= 8'd0;
							sector_lba_q <= {16'd0, selected_base_q} +
								{24'd0, payload_sector_count_q};
							state_q <= ST_PAYLOAD_FILL_START;
						end else begin
							if (scan_die_q && (scan_block_q == 6'd34)) begin
								state_q <= ST_PAYLOAD_DONE;
							end else begin
								advance_scan;
								state_q <= ST_PAYLOAD_SCAN;
							end
						end
					end
				end

				ST_DIR_FILL_START: begin
					if (sector_fill_ready_i) state_q <= ST_DIR_BUILD_START;
				end

				ST_DIR_BUILD_START: state_q <= ST_DIR_FILL;

				ST_DIR_FILL: begin
					if (sector_fill_fault_i || codec_build_rejected_w) begin
						state_q <= ST_ABORT_WAIT;
					end else if (codec_emit_done_w) begin
						sector_lba_q <= {31'd0, proposed_directory_lba_q};
						state_q <= ST_DIR_SAVE_REQ;
					end
				end

				ST_DIR_SAVE_REQ: begin
					if (sector_fill_fault_i) state_q <= ST_ABORT_WAIT;
					else if (sector_save_ready_i && sector_buf_full_i)
						state_q <= ST_DIR_WAIT_BUSY;
				end

				ST_DIR_WAIT_BUSY: if (sector_busy_i) state_q <= ST_DIR_WAIT_DONE;

				ST_DIR_WAIT_DONE: begin
					if (!sector_busy_i) begin
						// The disk commit is now irreversible; the bounded
						// M10K adoption below always finishes.
						adopt_import_q <= 1'b0;
						adopt_index_q <= 7'd0;
						state_q <= ST_ADOPT_REQ;
					end
				end

				ST_ADOPT_REQ: state_q <= ST_ADOPT_WRITE;

				ST_ADOPT_WRITE: begin
					if (adopt_index_q == 7'd69) begin
						if (adopt_import_q) begin
							committed_valid_q <= 1'b1;
							committed_directory_lba_q <= import_directory_lba_q;
							committed_generation_q <= import_generation_q;
							committed_file0_q <= import_file0_q;
							committed_file1_q <= import_file1_q;
							committed_active0_q <= import_active0_q;
							committed_active1_q <= import_active1_q;
							committed_next_free_q <= import_next_free_q;
							import_done_o <= 1'b1;
						end else begin
							committed_valid_q <= 1'b1;
							committed_directory_lba_q <= proposed_directory_lba_q;
							committed_generation_q <= proposed_generation_q;
							committed_file0_q <= proposed_live0_q;
							committed_file1_q <= proposed_live1_q;
							committed_active0_q <= proposed_active0_q;
							committed_active1_q <= proposed_active1_q;
							committed_next_free_q <= proposed_next_free_q;
							commit_o <= 1'b1;
							done_o <= 1'b1;
							snapshot_acquired_q <= 1'b0;
						end
						state_q <= ST_IDLE;
					end else begin
						adopt_index_q <= adopt_index_q + 7'd1;
						state_q <= ST_ADOPT_REQ;
					end
				end

				ST_ABORT_WAIT: begin
					pause_hold_q <= 1'b0;
					if (!sector_busy_i && !mover_busy_w) begin
						if (operation_import_q) import_rejected_o <= 1'b1;
						else begin
							rejected_o <= 1'b1;
							if (snapshot_acquired_q || snapshot_active_i)
								snapshot_abort_o <= 1'b1;
						end
						snapshot_acquired_q <= 1'b0;
						state_q <= ST_IDLE;
					end
				end

				default: state_q <= ST_ABORT_WAIT;
			endcase
		end
	end

	// Geometry is erase-byte aligned and DDR3 requests address complete beats;
	// retain those structurally fixed low bits in the lint cone.
	/* verilator lint_off UNUSED */
	wire unused_ok = &{1'b0, geometry_base_w[0], geometry_bytes_w[8:0],
	                   controller_ddr_word_w[1:0], 1'b0};
	/* verilator lint_on UNUSED */

endmodule

`default_nettype wire
