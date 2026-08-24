// Copyright (c) 2026 Jamie Blanks

`default_nettype none
`timescale 1ns/1ps

// Transactional sparse S0 overlay loader.
//
// The surrounding overlay controller owns the one-sector HPS bridge. This
// module requests sectors through that bridge, parses both V1 directory
// generations, and writes selected payloads only into the DDR3 staging window.
// No live cartridge request is possible until one complete generation has
// passed identity, allocation, geometry, and payload CRC validation.
//
// After validation, ngp_cart_overlay_apply holds the normal frame-safe pause,
// restores the prior live-difference blocks from the immutable DDR3 shadow,
// applies the staged target blocks, and atomically publishes the target map.
module ngp_cart_overlay_s0_load
(
	input  wire        clk,
	input  wire        reset,
	input  wire        start_i,
	output wire        busy_o,
	output reg         done_o,
	output reg         rejected_o,
	// A CRC-valid NGPINC1 marker is a new-format first-create transaction
	// interrupted before any normal directory commit. The parent may safely
	// reinitialize this one rejection class; every other nonempty rejection is
	// fail-closed.
	output wire        recoverable_incomplete_o,

	input  wire [31:0] identity_raw_crc32_i,
	input  wire [31:0] identity_raw_bytes_i,
	input  wire [31:0] identity_pristine_crc32_i,
	input  wire [31:0] identity_physical_bytes_i,
	input  wire [1:0]  identity_die0_code_i,
	input  wire [1:0]  identity_die1_code_i,
	input  wire [15:0] identity_catalog_i,
	input  wire [7:0]  identity_subcatalog_i,
	input  wire [95:0] identity_title_i,
	input  wire        file_sectors_valid_i,
	input  wire [15:0] file_sectors_i,

	input  wire [34:0] old_live0_i,
	input  wire [34:0] old_live1_i,
	input  wire [1:0]  die_busy_i,
	input  wire        event0_i,
	input  wire [5:0]  block0_i,
	input  wire        event1_i,
	input  wire [5:0]  block1_i,
	input  wire        pause_ready_i,
	output wire        pause_req_o,
	output wire        force_flash_read_o,
	output wire        ledger_adopt_o,
	output wire [34:0] ledger_target0_o,
	output wire [34:0] ledger_target1_o,
	// Selected committed-directory metadata. A parent controller scans the
	// 70 entries after done_o and imports them into the save-side committed
	// table; keeping that table separate preserves rollback on a failed save.
	output wire [31:0] file_generation_o,
	output wire        file_directory_lba_o,
	output wire [34:0] file_active0_o,
	output wire [34:0] file_active1_o,
	output wire [15:0] file_next_free_lba_o,
	input  wire [6:0]  file_entry_index_i,
	output wire [15:0] file_entry_base_lba_o,
	output wire [31:0] file_entry_crc32_o,

	// One-sector bridge controller side. Buffer reads are synchronous with
	// one clock of latency; a received sector is held until release_o.
	output wire [31:0] sector_lba_o,
	output wire        sector_load_req_o,
	input  wire        sector_load_ready_i,
	input  wire        sector_rx_valid_i,
	input  wire        sector_rx_error_i,
	output wire [7:0]  sector_buf_addr_o,
	input  wire [15:0] sector_buf_rdata_i,
	output wire        sector_rx_release_o,

	// Canonical live-cart p2 port. It is driven only by the atomic apply phase.
	output wire        p2_req_o,
	output wire        p2_we_o,
	output wire [24:0] p2_addr_o,
	output wire [15:0] p2_wdata_o,
	output wire [1:0]  p2_be_o,
	input  wire        p2_ready_i,
	input  wire        p2_done_i,
	input  wire [15:0] p2_rdata_i,

	// Shared DDR3 channel-2 client. Payload receive writes the staging window;
	// the final apply phase reads pristine and staging through the same pins.
	output wire [27:1] ddr_addr_o,
	output wire [63:0] ddr_din_o,
	output wire        ddr_req_o,
	output wire        ddr_rnw_o,
	output wire [7:0]  ddr_be_o,
	input  wire [63:0] ddr_dout_i,
	input  wire        ddr_ready_i
);

	localparam [4:0] ST_IDLE             = 5'd0;
	localparam [4:0] ST_DIR_REQ          = 5'd1;
	localparam [4:0] ST_DIR_WAIT_RX      = 5'd2;
	localparam [4:0] ST_DIR_START        = 5'd3;
	localparam [4:0] ST_DIR_BUF_REQ      = 5'd4;
	localparam [4:0] ST_DIR_BUF_SEND     = 5'd5;
	localparam [4:0] ST_DIR_RESULT       = 5'd6;
	localparam [4:0] ST_DIR_RELEASE      = 5'd7;
	localparam [4:0] ST_DIR_RELEASE_WAIT = 5'd8;
	localparam [4:0] ST_PICK             = 5'd9;
	localparam [4:0] ST_SCAN             = 5'd10;
	localparam [4:0] ST_ENTRY_REQ        = 5'd11;
	localparam [4:0] ST_ENTRY_CHECK      = 5'd12;
	localparam [4:0] ST_PAY_REQ          = 5'd13;
	localparam [4:0] ST_PAY_WAIT_RX      = 5'd14;
	localparam [4:0] ST_PAY_BUF_REQ      = 5'd15;
	localparam [4:0] ST_PAY_BUF_LATCH    = 5'd16;
	localparam [4:0] ST_PAY_DDR_WRITE    = 5'd17;
	localparam [4:0] ST_PAY_RELEASE      = 5'd18;
	localparam [4:0] ST_PAY_RELEASE_WAIT = 5'd19;
	localparam [4:0] ST_BLOCK_CHECK      = 5'd20;
	localparam [4:0] ST_FALLBACK         = 5'd21;
	localparam [4:0] ST_APPLY_START      = 5'd22;
	localparam [4:0] ST_APPLY_WAIT       = 5'd23;
	localparam [4:0] ST_REJECT           = 5'd24;

	reg [4:0]  state_q;
	reg        dir_select_q;
	reg [7:0]  buffer_index_q;
	reg        candidate_q;
	reg        fallback_q;
	reg        scan_die_q;
	reg [5:0]  scan_block_q;
	reg [34:0] target0_q, target1_q;
	reg [34:0] active0_q, active1_q;
	reg [31:0] sector_lba_q;
	reg [7:0]  block_sectors_q;
	reg [7:0]  sector_index_q;
	reg [23:0] staging_word_base_q;
	reg [31:0] target_crc_q;
	reg [31:0] block_crc_q;
	reg [15:0] payload_word_q;
	reg        payload_rx_bad_q;
	reg        marker_match_q;
	reg [31:0] marker_crc_q;
	reg [15:0] marker_crc_lo_q;
	reg        dir0_incomplete_q;
	reg        dir1_incomplete_q;

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

	function automatic [15:0] incomplete_word;
		input [7:0] index_i;
		begin
			case (index_i)
				8'd0: incomplete_word = 16'h474E; // "NG"
				8'd1: incomplete_word = 16'h4950; // "PI"
				8'd2: incomplete_word = 16'h434E; // "NC"
				8'd3: incomplete_word = 16'h0031; // "1\0"
				8'd4: incomplete_word = 16'h0001;
				default: incomplete_word = 16'd0;
			endcase
		end
	endfunction

	assign recoverable_incomplete_o = dir0_incomplete_q || dir1_incomplete_q;

	wire codec_reset_w = reset || ((state_q == ST_IDLE) && start_i);
	wire dir_sector_start_w = (state_q == ST_DIR_START);
	wire dir_sector_word_valid_w = (state_q == ST_DIR_BUF_SEND);

	wire dir0_word_ready_w, dir1_word_ready_w;
	wire dir0_accepted_w, dir0_rejected_w, dir0_valid_w;
	wire dir1_accepted_w, dir1_rejected_w, dir1_valid_w;
	wire [31:0] dir0_generation_w, dir1_generation_w;
	wire [7:0]  dir0_flags_w, dir1_flags_w;
	wire [34:0] dir0_ledger0_w, dir0_ledger1_w;
	wire [34:0] dir1_ledger0_w, dir1_ledger1_w;
	wire [34:0] dir0_active0_w, dir0_active1_w;
	wire [34:0] dir1_active0_w, dir1_active1_w;
	wire [15:0] dir0_next_free_w, dir1_next_free_w;

	wire [6:0] scan_entry_index_w = scan_die_q ?
		(7'd35 + {1'b0, scan_block_q}) : {1'b0, scan_block_q};
	wire [6:0] directory_entry_index_w = (state_q == ST_IDLE) ?
		file_entry_index_i : scan_entry_index_w;
	wire [15:0] dir0_entry_base_w, dir1_entry_base_w;
	wire [31:0] dir0_entry_crc_w, dir1_entry_crc_w;

	// Builder pins are unused by the read-only loader instance.
	wire dir0_build_rejected_w, dir1_build_rejected_w;
	wire dir0_emit_valid_w, dir1_emit_valid_w;
	wire [7:0] dir0_emit_index_w, dir1_emit_index_w;
	wire [15:0] dir0_emit_word_w, dir1_emit_word_w;
	wire dir0_emit_done_w, dir1_emit_done_w;

	ngp_cart_overlay_directory u_directory0
	(
		.clk(clk), .reset(codec_reset_w),
		.identity_raw_crc32_i(identity_raw_crc32_i),
		.identity_raw_bytes_i(identity_raw_bytes_i),
		.identity_pristine_crc32_i(identity_pristine_crc32_i),
		.identity_physical_bytes_i(identity_physical_bytes_i),
		.identity_die0_code_i(identity_die0_code_i),
		.identity_die1_code_i(identity_die1_code_i),
		.identity_catalog_i(identity_catalog_i),
		.identity_subcatalog_i(identity_subcatalog_i),
		.identity_title_i(identity_title_i),
		.file_sectors_valid_i(file_sectors_valid_i),
		.file_sectors_i(file_sectors_i),
		.sector_start_i(dir_sector_start_w && !dir_select_q),
		.sector_word_valid_i(dir_sector_word_valid_w && !dir_select_q),
		.sector_word_i(sector_buf_rdata_i),
		.sector_word_ready_o(dir0_word_ready_w),
		.accepted_o(dir0_accepted_w), .rejected_o(dir0_rejected_w),
		.directory_valid_o(dir0_valid_w), .generation_o(dir0_generation_w),
		.flags_o(dir0_flags_w), .ledger0_o(dir0_ledger0_w),
		.ledger1_o(dir0_ledger1_w), .active0_o(dir0_active0_w),
		.active1_o(dir0_active1_w), .next_free_lba_o(dir0_next_free_w),
		.entry_config_we_i(1'b0), .entry_config_index_i(7'd0),
		.entry_config_base_lba_i(16'd0), .entry_config_crc32_i(32'd0),
		.entry_meta_index_i(directory_entry_index_w),
		.entry_meta_base_lba_o(dir0_entry_base_w),
		.entry_meta_crc32_o(dir0_entry_crc_w),
		.build_start_i(1'b0), .build_generation_i(32'd0),
		.build_flags_i(8'd0), .build_ledger0_i(35'd0),
		.build_ledger1_i(35'd0), .build_active0_i(35'd0),
		.build_active1_i(35'd0), .build_next_free_lba_i(16'd2),
		.build_rejected_o(dir0_build_rejected_w),
		.emit_valid_o(dir0_emit_valid_w), .emit_word_index_o(dir0_emit_index_w),
		.emit_word_o(dir0_emit_word_w), .emit_ready_i(1'b0),
		.emit_done_o(dir0_emit_done_w)
	);

	ngp_cart_overlay_directory u_directory1
	(
		.clk(clk), .reset(codec_reset_w),
		.identity_raw_crc32_i(identity_raw_crc32_i),
		.identity_raw_bytes_i(identity_raw_bytes_i),
		.identity_pristine_crc32_i(identity_pristine_crc32_i),
		.identity_physical_bytes_i(identity_physical_bytes_i),
		.identity_die0_code_i(identity_die0_code_i),
		.identity_die1_code_i(identity_die1_code_i),
		.identity_catalog_i(identity_catalog_i),
		.identity_subcatalog_i(identity_subcatalog_i),
		.identity_title_i(identity_title_i),
		.file_sectors_valid_i(file_sectors_valid_i),
		.file_sectors_i(file_sectors_i),
		.sector_start_i(dir_sector_start_w && dir_select_q),
		.sector_word_valid_i(dir_sector_word_valid_w && dir_select_q),
		.sector_word_i(sector_buf_rdata_i),
		.sector_word_ready_o(dir1_word_ready_w),
		.accepted_o(dir1_accepted_w), .rejected_o(dir1_rejected_w),
		.directory_valid_o(dir1_valid_w), .generation_o(dir1_generation_w),
		.flags_o(dir1_flags_w), .ledger0_o(dir1_ledger0_w),
		.ledger1_o(dir1_ledger1_w), .active0_o(dir1_active0_w),
		.active1_o(dir1_active1_w), .next_free_lba_o(dir1_next_free_w),
		.entry_config_we_i(1'b0), .entry_config_index_i(7'd0),
		.entry_config_base_lba_i(16'd0), .entry_config_crc32_i(32'd0),
		.entry_meta_index_i(directory_entry_index_w),
		.entry_meta_base_lba_o(dir1_entry_base_w),
		.entry_meta_crc32_o(dir1_entry_crc_w),
		.build_start_i(1'b0), .build_generation_i(32'd0),
		.build_flags_i(8'd0), .build_ledger0_i(35'd0),
		.build_ledger1_i(35'd0), .build_active0_i(35'd0),
		.build_active1_i(35'd0), .build_next_free_lba_i(16'd2),
		.build_rejected_o(dir1_build_rejected_w),
		.emit_valid_o(dir1_emit_valid_w), .emit_word_index_o(dir1_emit_index_w),
		.emit_word_o(dir1_emit_word_w), .emit_ready_i(1'b0),
		.emit_done_o(dir1_emit_done_w)
	);

	wire dir0_usable_w = dir0_valid_w && (dir0_flags_w == 8'd0);
	wire dir1_usable_w = dir1_valid_w && (dir1_flags_w == 8'd0);
	wire selected_live_w = scan_die_q ? target1_q[scan_block_q] :
	                                            target0_q[scan_block_q];
	wire selected_active_w = scan_die_q ? active1_q[scan_block_q] :
	                                              active0_q[scan_block_q];
	wire [15:0] selected_entry_base_w = candidate_q ? dir1_entry_base_w :
	                                                        dir0_entry_base_w;
	wire [31:0] selected_entry_crc_w = candidate_q ? dir1_entry_crc_w :
	                                                       dir0_entry_crc_w;

	wire geometry_valid_w;
	wire [20:0] geometry_base_w;
	wire [16:0] geometry_bytes_w;
	wire [15:0] geometry_words_unused_w;
	wire [1:0] selected_size_code_w = scan_die_q ? identity_die1_code_i :
	                                                     identity_die0_code_i;

	ngp_cart_overlay_geometry u_geometry
	(
		.size_code_i(selected_size_code_w), .block_i(scan_block_q),
		.valid_o(geometry_valid_w), .base_o(geometry_base_w),
		.bytes_o(geometry_bytes_w), .words_o(geometry_words_unused_w)
	);

	assign sector_lba_o = sector_lba_q;
	// Hold the request independently of ready. The shared bridge returns ready
	// from its own arbitration, so feeding ready back into request would form a
	// combinational loop once load and save clients share that bridge.
	assign sector_load_req_o = (state_q == ST_DIR_REQ) ||
		(state_q == ST_PAY_REQ);
	assign sector_buf_addr_o = buffer_index_q;
	assign sector_rx_release_o = (state_q == ST_DIR_RELEASE) ||
		(state_q == ST_PAY_RELEASE);

	// Staging uses the cart-linear physical word offset at DDR byte +0x400000.
	wire [23:0] payload_ddr_word_w = staging_word_base_q +
		{9'd0, sector_index_q[6:0], 8'd0} + {16'd0, buffer_index_q};
	reg [63:0] payload_ddr_din_w;
	reg [7:0]  payload_ddr_be_w;

	always @* begin
		payload_ddr_din_w = 64'd0;
		payload_ddr_be_w = 8'd0;
		case (payload_ddr_word_w[1:0])
			2'd0: begin payload_ddr_din_w[15:0]  = payload_word_q; payload_ddr_be_w = 8'h03; end
			2'd1: begin payload_ddr_din_w[31:16] = payload_word_q; payload_ddr_be_w = 8'h0C; end
			2'd2: begin payload_ddr_din_w[47:32] = payload_word_q; payload_ddr_be_w = 8'h30; end
			default: begin payload_ddr_din_w[63:48] = payload_word_q; payload_ddr_be_w = 8'hC0; end
		endcase
	end

	wire apply_busy_w, apply_done_w, apply_rejected_w;
	wire apply_pause_req_w;
	wire apply_p2_req_w, apply_p2_we_w;
	wire [24:0] apply_p2_addr_w;
	wire [15:0] apply_p2_wdata_w;
	wire [1:0] apply_p2_be_w;
	wire [27:1] apply_ddr_addr_w;
	wire [63:0] apply_ddr_din_w;
	wire apply_ddr_req_w, apply_ddr_rnw_w;
	wire [7:0] apply_ddr_be_w;

	ngp_cart_overlay_apply u_apply
	(
		.clk(clk), .reset(reset), .start_i(state_q == ST_APPLY_START),
		.pause_ready_i(pause_ready_i),
		.old_live0_i(old_live0_i), .old_live1_i(old_live1_i),
		.target0_i(target0_q), .target1_i(target1_q),
		.die0_code_i(identity_die0_code_i), .die1_code_i(identity_die1_code_i),
		.die_busy_i(die_busy_i), .event0_i(event0_i), .block0_i(block0_i),
		.event1_i(event1_i), .block1_i(block1_i),
		.pause_req_o(apply_pause_req_w), .busy_o(apply_busy_w),
		.done_o(apply_done_w), .rejected_o(apply_rejected_w),
		.p2_req_o(apply_p2_req_w), .p2_we_o(apply_p2_we_w),
		.p2_addr_o(apply_p2_addr_w), .p2_wdata_o(apply_p2_wdata_w),
		.p2_be_o(apply_p2_be_w), .p2_ready_i(p2_ready_i),
		.p2_done_i(p2_done_i), .p2_rdata_i(p2_rdata_i),
		.ddr_addr_o(apply_ddr_addr_w), .ddr_din_o(apply_ddr_din_w),
		.ddr_req_o(apply_ddr_req_w), .ddr_rnw_o(apply_ddr_rnw_w),
		.ddr_be_o(apply_ddr_be_w), .ddr_dout_i(ddr_dout_i),
		.ddr_ready_i(ddr_ready_i)
	);

	wire apply_owner_w = (state_q == ST_APPLY_START) ||
		(state_q == ST_APPLY_WAIT) || apply_busy_w;
	assign busy_o = (state_q != ST_IDLE) || apply_busy_w;
	assign pause_req_o = apply_pause_req_w;
	// The child apply pulse occurs one clock before this wrapper observes its
	// done pulse. Publish the atomic ledger/read-array commit together with the
	// wrapper's terminal done_o so downstream state cannot sample mixed epochs.
	assign force_flash_read_o = done_o;
	assign ledger_adopt_o = done_o;
	assign ledger_target0_o = target0_q;
	assign ledger_target1_o = target1_q;
	assign file_generation_o = candidate_q ? dir1_generation_w : dir0_generation_w;
	assign file_directory_lba_o = candidate_q;
	assign file_active0_o = active0_q;
	assign file_active1_o = active1_q;
	assign file_next_free_lba_o = candidate_q ? dir1_next_free_w : dir0_next_free_w;
	assign file_entry_base_lba_o = candidate_q ? dir1_entry_base_w : dir0_entry_base_w;
	assign file_entry_crc32_o = candidate_q ? dir1_entry_crc_w : dir0_entry_crc_w;

	assign p2_req_o = apply_p2_req_w;
	assign p2_we_o = apply_p2_we_w;
	assign p2_addr_o = apply_p2_addr_w;
	assign p2_wdata_o = apply_p2_wdata_w;
	assign p2_be_o = apply_p2_be_w;

	assign ddr_addr_o = apply_owner_w ? apply_ddr_addr_w :
		{3'd0, payload_ddr_word_w[23:2], 2'b00};
	assign ddr_din_o = apply_owner_w ? apply_ddr_din_w : payload_ddr_din_w;
	assign ddr_req_o = apply_owner_w ? apply_ddr_req_w :
		(state_q == ST_PAY_DDR_WRITE);
	assign ddr_rnw_o = apply_owner_w ? apply_ddr_rnw_w : 1'b0;
	assign ddr_be_o = apply_owner_w ? apply_ddr_be_w : payload_ddr_be_w;

	// The selected directory's complete physical block payload has already
	// been bounded by the directory codec. These explicit checks remain at the
	// consumption boundary so a corrupt metadata RAM value cannot cause I/O.
	wire [8:0] selected_payload_end_w = {1'b0, geometry_bytes_w[16:9]};
	wire [16:0] selected_lba_end_w = {1'b0, selected_entry_base_w} +
		{8'd0, selected_payload_end_w} + {8'd0, selected_payload_end_w};

	always @(posedge clk) begin
		done_o <= 1'b0;
		rejected_o <= 1'b0;

		if (reset) begin
			state_q <= ST_IDLE;
			dir_select_q <= 1'b0;
			buffer_index_q <= 8'd0;
			candidate_q <= 1'b0;
			fallback_q <= 1'b0;
			scan_die_q <= 1'b0;
			scan_block_q <= 6'd0;
			target0_q <= 35'd0;
			target1_q <= 35'd0;
			active0_q <= 35'd0;
			active1_q <= 35'd0;
			sector_lba_q <= 32'd0;
			block_sectors_q <= 8'd0;
			sector_index_q <= 8'd0;
			staging_word_base_q <= 24'd0;
			target_crc_q <= 32'd0;
			block_crc_q <= 32'hFFFFFFFF;
			payload_word_q <= 16'd0;
			payload_rx_bad_q <= 1'b0;
			marker_match_q <= 1'b0;
			marker_crc_q <= 32'hFFFFFFFF;
			marker_crc_lo_q <= 16'd0;
			dir0_incomplete_q <= 1'b0;
			dir1_incomplete_q <= 1'b0;
		end else begin
			case (state_q)
				ST_IDLE: if (start_i) begin
					dir_select_q <= 1'b0;
					sector_lba_q <= 32'd0;
					dir0_incomplete_q <= 1'b0;
					dir1_incomplete_q <= 1'b0;
					state_q <= ST_DIR_REQ;
				end

				ST_DIR_REQ: if (sector_load_ready_i) state_q <= ST_DIR_WAIT_RX;

				ST_DIR_WAIT_RX: begin
					if (sector_rx_error_i) state_q <= ST_DIR_RELEASE;
					else if (sector_rx_valid_i) state_q <= ST_DIR_START;
				end

				ST_DIR_START: begin
					buffer_index_q <= 8'd0;
					marker_match_q <= 1'b1;
					marker_crc_q <= 32'hFFFFFFFF;
					marker_crc_lo_q <= 16'd0;
					state_q <= ST_DIR_BUF_REQ;
				end

				ST_DIR_BUF_REQ: state_q <= ST_DIR_BUF_SEND;

				ST_DIR_BUF_SEND: begin
					if (dir_select_q ? dir1_word_ready_w : dir0_word_ready_w) begin
						if (buffer_index_q < 8'd254) begin
							if (sector_buf_rdata_i != incomplete_word(buffer_index_q))
								marker_match_q <= 1'b0;
							marker_crc_q <= crc32_word(marker_crc_q, sector_buf_rdata_i);
						end else if (buffer_index_q == 8'd254) begin
							marker_crc_lo_q <= sector_buf_rdata_i;
						end else begin
							if (!dir_select_q)
								dir0_incomplete_q <= marker_match_q &&
									({sector_buf_rdata_i, marker_crc_lo_q} == ~marker_crc_q);
							else
								dir1_incomplete_q <= marker_match_q &&
									({sector_buf_rdata_i, marker_crc_lo_q} == ~marker_crc_q);
						end
						if (buffer_index_q == 8'hFF) state_q <= ST_DIR_RESULT;
						else begin
							buffer_index_q <= buffer_index_q + 8'd1;
							state_q <= ST_DIR_BUF_REQ;
						end
					end
				end

				ST_DIR_RESULT: begin
					if (dir_select_q ? (dir1_accepted_w || dir1_rejected_w) :
					                   (dir0_accepted_w || dir0_rejected_w))
						state_q <= ST_DIR_RELEASE;
				end

				ST_DIR_RELEASE: state_q <= ST_DIR_RELEASE_WAIT;

				ST_DIR_RELEASE_WAIT: begin
					if (!sector_rx_valid_i && !sector_rx_error_i) begin
						if (!dir_select_q) begin
							dir_select_q <= 1'b1;
							sector_lba_q <= 32'd1;
							state_q <= ST_DIR_REQ;
						end else begin
							state_q <= ST_PICK;
						end
					end
				end

				ST_PICK: begin
					if (dir0_usable_w && dir1_usable_w) begin
						fallback_q <= 1'b1;
						if (dir1_generation_w > dir0_generation_w) begin
							candidate_q <= 1'b1;
							target0_q <= dir1_ledger0_w;
							target1_q <= dir1_ledger1_w;
							active0_q <= dir1_active0_w;
							active1_q <= dir1_active1_w;
						end else begin
							candidate_q <= 1'b0;
							target0_q <= dir0_ledger0_w;
							target1_q <= dir0_ledger1_w;
							active0_q <= dir0_active0_w;
							active1_q <= dir0_active1_w;
						end
						scan_die_q <= 1'b0;
						scan_block_q <= 6'd0;
						state_q <= ST_SCAN;
					end else if (dir1_usable_w) begin
						candidate_q <= 1'b1;
						fallback_q <= 1'b0;
						target0_q <= dir1_ledger0_w;
						target1_q <= dir1_ledger1_w;
						active0_q <= dir1_active0_w;
						active1_q <= dir1_active1_w;
						scan_die_q <= 1'b0;
						scan_block_q <= 6'd0;
						state_q <= ST_SCAN;
					end else if (dir0_usable_w) begin
						candidate_q <= 1'b0;
						fallback_q <= 1'b0;
						target0_q <= dir0_ledger0_w;
						target1_q <= dir0_ledger1_w;
						active0_q <= dir0_active0_w;
						active1_q <= dir0_active1_w;
						scan_die_q <= 1'b0;
						scan_block_q <= 6'd0;
						state_q <= ST_SCAN;
					end else begin
						state_q <= ST_REJECT;
					end
				end

				ST_SCAN: begin
					if (selected_live_w) begin
						state_q <= ST_ENTRY_REQ;
					end else if (!scan_die_q && (scan_block_q == 6'd34)) begin
						scan_die_q <= 1'b1;
						scan_block_q <= 6'd0;
					end else if (scan_die_q && (scan_block_q == 6'd34)) begin
						state_q <= ST_APPLY_START;
					end else begin
						scan_block_q <= scan_block_q + 6'd1;
					end
				end

				ST_ENTRY_REQ: state_q <= ST_ENTRY_CHECK;

				ST_ENTRY_CHECK: begin
					if (!geometry_valid_w || (selected_entry_base_w < 16'd2) ||
					    (selected_lba_end_w > {1'b0, candidate_q ?
					     dir1_next_free_w : dir0_next_free_w})) begin
						state_q <= ST_FALLBACK;
					end else begin
						block_sectors_q <= geometry_bytes_w[16:9];
						sector_index_q <= 8'd0;
						sector_lba_q <= {16'd0, selected_entry_base_w} +
							(selected_active_w ? {24'd0, geometry_bytes_w[16:9]} : 32'd0);
						staging_word_base_q <= 24'h200000 +
							(scan_die_q ? 24'h100000 : 24'd0) +
							{4'd0, geometry_base_w[20:1]};
						target_crc_q <= selected_entry_crc_w;
						block_crc_q <= 32'hFFFFFFFF;
						payload_rx_bad_q <= 1'b0;
						state_q <= ST_PAY_REQ;
					end
				end

				ST_PAY_REQ: if (sector_load_ready_i) state_q <= ST_PAY_WAIT_RX;

				ST_PAY_WAIT_RX: begin
					if (sector_rx_error_i) begin
						payload_rx_bad_q <= 1'b1;
						state_q <= ST_PAY_RELEASE;
					end else if (sector_rx_valid_i) begin
						buffer_index_q <= 8'd0;
						state_q <= ST_PAY_BUF_REQ;
					end
				end

				ST_PAY_BUF_REQ: state_q <= ST_PAY_BUF_LATCH;

				ST_PAY_BUF_LATCH: begin
					payload_word_q <= sector_buf_rdata_i;
					block_crc_q <= crc32_word(block_crc_q, sector_buf_rdata_i);
					state_q <= ST_PAY_DDR_WRITE;
				end

				ST_PAY_DDR_WRITE: if (ddr_ready_i) begin
					if (buffer_index_q == 8'hFF) state_q <= ST_PAY_RELEASE;
					else begin
						buffer_index_q <= buffer_index_q + 8'd1;
						state_q <= ST_PAY_BUF_REQ;
					end
				end

				ST_PAY_RELEASE: state_q <= ST_PAY_RELEASE_WAIT;

				ST_PAY_RELEASE_WAIT: begin
					if (!sector_rx_valid_i && !sector_rx_error_i) begin
						if (payload_rx_bad_q) begin
							state_q <= ST_FALLBACK;
						end else if (sector_index_q == (block_sectors_q - 8'd1)) begin
							state_q <= ST_BLOCK_CHECK;
						end else begin
							sector_index_q <= sector_index_q + 8'd1;
							sector_lba_q <= sector_lba_q + 32'd1;
							state_q <= ST_PAY_REQ;
						end
					end
				end

				ST_BLOCK_CHECK: begin
					if ((~block_crc_q) != target_crc_q) begin
						state_q <= ST_FALLBACK;
					end else if (!scan_die_q && (scan_block_q == 6'd34)) begin
						scan_die_q <= 1'b1;
						scan_block_q <= 6'd0;
						state_q <= ST_SCAN;
					end else if (scan_die_q && (scan_block_q == 6'd34)) begin
						state_q <= ST_APPLY_START;
					end else begin
						scan_block_q <= scan_block_q + 6'd1;
						state_q <= ST_SCAN;
					end
				end

				ST_FALLBACK: begin
					if (fallback_q) begin
						fallback_q <= 1'b0;
						candidate_q <= ~candidate_q;
						if (candidate_q) begin
							target0_q <= dir0_ledger0_w;
							target1_q <= dir0_ledger1_w;
							active0_q <= dir0_active0_w;
							active1_q <= dir0_active1_w;
						end else begin
							target0_q <= dir1_ledger0_w;
							target1_q <= dir1_ledger1_w;
							active0_q <= dir1_active0_w;
							active1_q <= dir1_active1_w;
						end
						scan_die_q <= 1'b0;
						scan_block_q <= 6'd0;
						state_q <= ST_SCAN;
					end else begin
						state_q <= ST_REJECT;
					end
				end

				ST_APPLY_START: state_q <= ST_APPLY_WAIT;

				ST_APPLY_WAIT: begin
					if (apply_rejected_w) state_q <= ST_REJECT;
					else if (apply_done_w) begin
						done_o <= 1'b1;
						state_q <= ST_IDLE;
					end
				end

				ST_REJECT: begin
					rejected_o <= 1'b1;
					state_q <= ST_IDLE;
				end

				default: state_q <= ST_REJECT;
			endcase
		end
	end

	// Retain codec builder outputs in the lint cone without creating control.
	/* verilator lint_off UNUSED */
	wire unused_ok = &{1'b0, dir0_build_rejected_w, dir1_build_rejected_w,
		dir0_emit_valid_w, dir1_emit_valid_w, dir0_emit_index_w,
		dir1_emit_index_w, dir0_emit_word_w, dir1_emit_word_w,
		dir0_emit_done_w, dir1_emit_done_w, geometry_base_w[0],
		geometry_bytes_w[8:0], geometry_words_unused_w, 1'b0};
	/* verilator lint_on UNUSED */

endmodule

`default_nettype wire
