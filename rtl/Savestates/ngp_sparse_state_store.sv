// Copyright (c) 2026 Jamie Blanks

`default_nettype none
`timescale 1ns/1ps

// ngp_sparse_state_store -- sparse manual-Type3 transaction.
//
// SAVE pre-captures only blocks in the evolving live ledger into the low DDR3
// physical staging window. Each stable complete block is then packed into the
// selected state slot while its CRC32 is calculated. Exact flash events clear
// the corresponding valid bit, causing that block to be recopied. The final
// ledger is sealed only after the frame-safe pause is acknowledged and both
// flash dies have drained; the CRC-protected manifest is written last. Pause
// remains held through engine_save_done_i so generic internals/types 0..2 and
// the sparse Type3 describe one machine instant.
//
// LOAD first reads the stored outer size and manifest, then CRC-checks every
// packed payload while copying it into the physical staging window. This
// preflight has no live-cart p2 write path. Only engine_restore_begin_i
// authorizes ngp_cart_overlay_apply. The child apply may overlap the generic
// engine's ch1/memory-tap restore, but its force/adopt pulses are suppressed:
// public completion, read-array forcing, and ledger adoption wait for BOTH the
// final apply p2 acknowledgement and engine_load_done_i.
//
// Address units:
//   slot_base_dword_i: same 32-bit-dword unit used by savestates.sv
//   Type3 manifest:    slot-relative dword 8418
//   Type3 payload:     manifest + 512 bytes
//   physical staging: DDR byte 0x400000 + cart-linear physical byte offset
//
// The module owns exactly one DDR channel-2 bundle. No full cartridge image is
// serialized: only selected complete physical erase blocks are transferred.
module ngp_sparse_state_store
(
	input  wire        clk,
	input  wire        reset,

	// Transaction request. save_start_i and load_start_i are mutually
	// exclusive one-cycle pulses while request_ready_o is high.
	input  wire        save_start_i,
	input  wire        load_start_i,
	input  wire        request_is_rewind_i,
	input  wire        cancel_i,
	input  wire [25:0] slot_base_dword_i,
	output wire        request_ready_o,
	output reg         request_accepted_o,
	output wire        busy_o,
	output reg         done_o,
	output reg         rejected_o,
	output reg         rewind_bypass_o,

	// Exact currently loaded cartridge identity.
	input  wire [31:0] identity_raw_crc32_i,
	input  wire [31:0] identity_raw_bytes_i,
	input  wire [31:0] identity_pristine_crc32_i,
	input  wire [31:0] identity_physical_bytes_i,
	input  wire [1:0]  identity_die0_code_i,
	input  wire [1:0]  identity_die1_code_i,
	input  wire        geometry_ready_i,
	input  wire [15:0] identity_catalog_i,
	input  wire [7:0]  identity_subcatalog_i,
	input  wire [95:0] identity_title_i,

	// Current canonical live-difference ledger and exact flash activity.
	input  wire [34:0] live_ledger0_i,
	input  wire [34:0] live_ledger1_i,
	input  wire [1:0]  die_busy_i,
	input  wire        event0_i,
	input  wire [5:0]  block0_i,
	input  wire        event1_i,
	input  wire [5:0]  block1_i,

	// Generic-engine transaction barrier.
	output wire        pause_req_o,
	input  wire        pause_ready_i,
	output reg         save_prepared_o,
	input  wire        engine_save_done_i,
	output reg         load_preflight_ready_o,
	input  wire        engine_restore_begin_i,
	input  wire        engine_load_done_i,
	output reg         load_apply_ready_o,
	output reg         load_apply_failed_o,
	output reg  [31:0] frozen_type3_bytes_o,
	output reg  [31:0] frozen_state_size_dwords_o,
	output reg  [34:0] frozen_ledger0_o,
	output reg  [34:0] frozen_ledger1_o,

	// Atomic load result. These pulses are withheld until both halves of the
	// load transaction are complete.
	output reg         ledger_adopt_o,
	output reg         force_flash_read_o,
	output wire [34:0] ledger_target0_o,
	output wire [34:0] ledger_target1_o,

	// Canonical live-cart p2 mailbox.
	output wire        p2_req_o,
	output wire        p2_we_o,
	output wire [24:0] p2_addr_o,
	output wire [15:0] p2_wdata_o,
	output wire [1:0]  p2_be_o,
	input  wire        p2_ready_i,
	input  wire        p2_done_i,
	input  wire [15:0] p2_rdata_i,

	// One shared DDR3 channel-2 client.
	output wire [27:1] ddr_addr_o,
	output wire [63:0] ddr_din_o,
	output wire        ddr_req_o,
	output wire        ddr_rnw_o,
	output wire [7:0]  ddr_be_o,
	input  wire [63:0] ddr_dout_i,
	input  wire        ddr_ready_i
);

	localparam [31:0] FIXED_STATE_DWORDS = 32'd8416;
	localparam [31:0] EMPTY_STATE_DWORDS = 32'd8544;
	localparam [31:0] MAX_STATE_DWORDS   = 32'd1057120;
	localparam [26:0] TYPE3_WORD_OFFSET  = 27'd16836; // 8418 dwords * 2
	localparam [26:0] PAYLOAD_WORD_OFFSET = 27'd17092; // + 256 manifest words
	localparam [23:0] STAGING_WORD_BASE  = 24'h200000; // DDR byte 0x400000

	localparam [5:0] ST_IDLE                 = 6'd0;
	localparam [5:0] ST_SAVE_CLEAR_CRC       = 6'd1;
	localparam [5:0] ST_SAVE_SCAN            = 6'd2;
	localparam [5:0] ST_SAVE_CAPTURE_START   = 6'd3;
	localparam [5:0] ST_SAVE_CAPTURE_WAIT    = 6'd4;
	localparam [5:0] ST_SAVE_COPY_READ       = 6'd5;
	localparam [5:0] ST_SAVE_COPY_WRITE      = 6'd6;
	localparam [5:0] ST_SAVE_CRC_COMMIT      = 6'd7;
	localparam [5:0] ST_SAVE_WAIT_PAUSE      = 6'd8;
	localparam [5:0] ST_SAVE_BUILD_START     = 6'd9;
	localparam [5:0] ST_SAVE_BUILD_WAIT      = 6'd10;
	localparam [5:0] ST_SAVE_WAIT_ENGINE     = 6'd11;
	localparam [5:0] ST_LOAD_HEADER_READ     = 6'd12;
	localparam [5:0] ST_LOAD_MANIFEST_START  = 6'd13;
	localparam [5:0] ST_LOAD_MANIFEST_READ   = 6'd14;
	localparam [5:0] ST_LOAD_MANIFEST_FEED   = 6'd15;
	localparam [5:0] ST_LOAD_MANIFEST_WAIT   = 6'd16;
	localparam [5:0] ST_LOAD_SCAN            = 6'd17;
	localparam [5:0] ST_LOAD_CRC_REQ         = 6'd18;
	localparam [5:0] ST_LOAD_CRC_LATCH       = 6'd19;
	localparam [5:0] ST_LOAD_PAYLOAD_READ    = 6'd20;
	localparam [5:0] ST_LOAD_PAYLOAD_WRITE   = 6'd21;
	localparam [5:0] ST_LOAD_WAIT_BEGIN      = 6'd22;
	localparam [5:0] ST_LOAD_WAIT_GEOMETRY   = 6'd23;
	localparam [5:0] ST_LOAD_APPLY_START     = 6'd24;
	localparam [5:0] ST_LOAD_APPLY_WAIT      = 6'd25;

	reg [5:0]  state_q;
	reg        pause_hold_q;
	reg [25:0] slot_base_dword_q;
	reg [26:0] manifest_word_base_q;
	reg [26:0] payload_word_base_q;

	reg [34:0] save_ledger0_q, save_ledger1_q;
	reg [34:0] save_valid0_q, save_valid1_q;
	reg [34:0] save_packed0_q, save_packed1_q;
	reg [34:0] target0_q, target1_q;

	reg [6:0]  clear_index_q;
	reg [6:0]  scan_index_q;
	reg [25:0] packed_word_offset_q;
	reg        op_die_q;
	reg [5:0]  op_block_q;
	reg [6:0]  op_index_q;
	reg [1:0]  op_size_code_q;
	reg [15:0] op_word_count_q;
	reg [15:0] op_word_index_q;
	reg [23:0] op_stage_word_base_q;
	reg [26:0] op_slot_word_base_q;
	reg        op_changed_q;
	reg [15:0] transfer_word_q;
	reg [31:0] block_crc_q;
	reg [31:0] final_crc_q;
	reg [31:0] expected_crc_q;

	reg [7:0]  manifest_word_index_q;
	reg [15:0] manifest_feed_word_q;
	reg [31:0] stored_state_size_q;
	reg        apply_done_seen_q;
	reg        apply_failed_seen_q;
	reg        engine_load_done_seen_q;

	function automatic [31:0] crc32_byte;
		input [31:0] crc_i;
		input [7:0] data_i;
		reg [31:0] crc_v;
		integer bit_i;
		begin
			crc_v = crc_i ^ {24'd0, data_i};
			for (bit_i = 0; bit_i < 8; bit_i = bit_i + 1) begin
				if (crc_v[0]) crc_v = (crc_v >> 1) ^ 32'hEDB88320;
				else          crc_v =  crc_v >> 1;
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

	function automatic [34:0] block_mask;
		input [1:0] code_i;
		begin
			case (code_i)
				2'd1: block_mask = 35'h0000007FF;
				2'd2: block_mask = 35'h00007FFFF;
				2'd3: block_mask = 35'h7FFFFFFFF;
				default: block_mask = 35'd0;
			endcase
		end
	endfunction

	function automatic [15:0] select_word64;
		input [63:0] value_i;
		input [1:0] lane_i;
		begin
			case (lane_i)
				2'd0: select_word64 = value_i[15:0];
				2'd1: select_word64 = value_i[31:16];
				2'd2: select_word64 = value_i[47:32];
				default: select_word64 = value_i[63:48];
			endcase
		end
	endfunction

	function automatic [27:1] ddr_address_for_word;
		input [24:0] word_upper_i;
		begin
			ddr_address_for_word = {word_upper_i, 2'b00};
		end
	endfunction

	wire [34:0] event0_mask_w = (event0_i && (block0_i < 6'd35)) ?
	                              (35'd1 << block0_i) : 35'd0;
	wire [34:0] event1_mask_w = (event1_i && (block1_i < 6'd35)) ?
	                              (35'd1 << block1_i) : 35'd0;
	wire any_event_w = (event0_mask_w != 35'd0) || (event1_mask_w != 35'd0);

	wire save_gather_w = (state_q >= ST_SAVE_CLEAR_CRC) &&
	                     (state_q <= ST_SAVE_WAIT_PAUSE);
	wire [34:0] save_effective0_w = save_gather_w ?
		(save_ledger0_q | live_ledger0_i | event0_mask_w) : save_ledger0_q;
	wire [34:0] save_effective1_w = save_gather_w ?
		(save_ledger1_q | live_ledger1_i | event1_mask_w) : save_ledger1_q;
	wire [34:0] save_added0_w = save_effective0_w & ~save_ledger0_q;
	wire [34:0] save_added1_w = save_effective1_w & ~save_ledger1_q;
	wire save_map_growth_w = (save_added0_w != 35'd0) ||
	                         (save_added1_w != 35'd0);
	wire save_maps_valid_w = ((save_effective0_w & ~block_mask(identity_die0_code_i)) == 35'd0) &&
	                         ((save_effective1_w & ~block_mask(identity_die1_code_i)) == 35'd0);
	wire save_all_valid_w = ((save_valid0_q & save_effective0_w) == save_effective0_w) &&
	                        ((save_valid1_q & save_effective1_w) == save_effective1_w);
	wire save_all_packed_w = ((save_packed0_q & save_effective0_w) == save_effective0_w) &&
	                         ((save_packed1_q & save_effective1_w) == save_effective1_w);

	// Fixed 35-entry die boundary without division or modulus.
	wire scan_die_w = (scan_index_q >= 7'd35);
	wire [5:0] scan_block_w = scan_die_w ?
	                              (scan_index_q[5:0] - 6'd35) : scan_index_q[5:0];
	wire scan_is_save_w = (state_q >= ST_SAVE_CLEAR_CRC) &&
	                      (state_q <= ST_SAVE_WAIT_ENGINE);
	wire [34:0] scan_map0_w = scan_is_save_w ? save_effective0_w : target0_q;
	wire [34:0] scan_map1_w = scan_is_save_w ? save_effective1_w : target1_q;
	wire scan_selected_w = scan_die_w ? scan_map1_w[scan_block_w] :
	                                      scan_map0_w[scan_block_w];
	wire scan_valid_w = scan_die_w ? save_valid1_q[scan_block_w] :
	                                   save_valid0_q[scan_block_w];
	wire scan_packed_w = scan_die_w ? save_packed1_q[scan_block_w] :
	                                    save_packed0_q[scan_block_w];
	wire [1:0] scan_size_code_w = scan_die_w ? identity_die1_code_i :
	                                             identity_die0_code_i;
	wire geometry_valid_w;
	wire [20:0] geometry_base_w;
	wire [16:0] geometry_bytes_w;
	wire [15:0] geometry_words_w;

	ngp_cart_overlay_geometry u_geometry
	(
		.size_code_i(scan_size_code_w), .block_i(scan_block_w),
		.valid_o(geometry_valid_w), .base_o(geometry_base_w),
		.bytes_o(geometry_bytes_w), .words_o(geometry_words_w)
	);

	wire [23:0] scan_physical_word_w =
		(scan_die_w ? 24'h100000 : 24'd0) + {4'd0, geometry_base_w[20:1]};
	wire [25:0] packed_offset_next_w = packed_word_offset_q +
		{10'd0, geometry_words_w};
	wire op_selected_event_w = op_die_q ?
		(event1_i && (block1_i == op_block_q)) :
		(event0_i && (block0_i == op_block_q));
	wire op_die_busy_w = op_die_q ? die_busy_i[1] : die_busy_i[0];
	wire op_invalid_now_w = op_changed_q || op_selected_event_w || op_die_busy_w;
	wire save_stream_invalid_w = op_invalid_now_w || save_map_growth_w;
	wire [31:0] crc_after_word_w = crc32_word(block_crc_q, transfer_word_q);

	// Manifest codec. Its candidate and committed CRC tables live in one M10K.
	wire codec_reset_w = reset || ((state_q == ST_IDLE) && request_ready_o &&
		((save_start_i || load_start_i) && !request_is_rewind_i));
	wire manifest_start_w = (state_q == ST_LOAD_MANIFEST_START);
	wire manifest_word_valid_w = (state_q == ST_LOAD_MANIFEST_FEED);
	wire manifest_word_ready_w;
	wire manifest_accepted_w, manifest_rejected_w, manifest_valid_w;
	wire [34:0] manifest_ledger0_w, manifest_ledger1_w;
	wire [31:0] manifest_payload_bytes_w, manifest_type3_bytes_w;
	wire [6:0] manifest_included_blocks_w;

	wire manifest_crc_config_we_w = (state_q == ST_SAVE_CLEAR_CRC) ||
		((state_q == ST_SAVE_CRC_COMMIT) && !save_stream_invalid_w);
	wire [6:0] manifest_crc_config_index_w =
		(state_q == ST_SAVE_CLEAR_CRC) ? clear_index_q : op_index_q;
	wire [31:0] manifest_crc_config_data_w =
		(state_q == ST_SAVE_CLEAR_CRC) ? 32'd0 : final_crc_q;
	wire [6:0] manifest_crc_meta_index_w = op_index_q;
	wire [31:0] manifest_crc_meta_data_w;

	wire manifest_build_start_w = (state_q == ST_SAVE_BUILD_START);
	wire manifest_build_rejected_w;
	wire [31:0] manifest_build_payload_bytes_w;
	wire [31:0] manifest_build_type3_bytes_w;
	wire [6:0] manifest_build_included_blocks_w;
	wire manifest_emit_valid_w;
	wire [7:0] manifest_emit_index_w;
	wire [15:0] manifest_emit_word_w;
	wire manifest_emit_done_w;
	wire manifest_busy_w;
	wire manifest_emit_ready_w;

	ngp_sparse_state_manifest u_manifest
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
		.manifest_start_i(manifest_start_w),
		.manifest_word_valid_i(manifest_word_valid_w),
		.manifest_word_i(manifest_feed_word_q),
		.manifest_word_ready_o(manifest_word_ready_w),
		.accepted_o(manifest_accepted_w), .rejected_o(manifest_rejected_w),
		.manifest_valid_o(manifest_valid_w),
		.ledger0_o(manifest_ledger0_w), .ledger1_o(manifest_ledger1_w),
		.payload_bytes_o(manifest_payload_bytes_w),
		.type3_bytes_o(manifest_type3_bytes_w),
		.included_blocks_o(manifest_included_blocks_w),
		.crc_config_we_i(manifest_crc_config_we_w),
		.crc_config_index_i(manifest_crc_config_index_w),
		.crc_config_data_i(manifest_crc_config_data_w),
		.crc_meta_index_i(manifest_crc_meta_index_w),
		.crc_meta_data_o(manifest_crc_meta_data_w),
		.build_start_i(manifest_build_start_w),
		.build_ledger0_i(save_ledger0_q), .build_ledger1_i(save_ledger1_q),
		.build_rejected_o(manifest_build_rejected_w),
		.build_payload_bytes_o(manifest_build_payload_bytes_w),
		.build_type3_bytes_o(manifest_build_type3_bytes_w),
		.build_included_blocks_o(manifest_build_included_blocks_w),
		.emit_valid_o(manifest_emit_valid_w),
		.emit_word_index_o(manifest_emit_index_w),
		.emit_word_o(manifest_emit_word_w),
		.emit_ready_i(manifest_emit_ready_w),
		.emit_done_o(manifest_emit_done_w), .busy_o(manifest_busy_w)
	);

	// Live-to-staging mover used only by SAVE pre-capture.
	wire mover_busy_w, mover_done_w, mover_stale_w, mover_rejected_w;
	wire mover_p2_req_w, mover_p2_we_w;
	wire [24:0] mover_p2_addr_w;
	wire [15:0] mover_p2_wdata_w;
	wire [1:0] mover_p2_be_w;
	wire [27:1] mover_ddr_addr_w;
	wire [63:0] mover_ddr_din_w;
	wire mover_ddr_req_w, mover_ddr_rnw_w;
	wire [7:0] mover_ddr_be_w;
	wire mover_owner_w;

	ngp_cart_overlay_mover u_capture_mover
	(
		.clk(clk), .reset(reset),
		.start_i(state_q == ST_SAVE_CAPTURE_START), .direction_i(1'b0),
		.die_i(op_die_q), .block_i(op_block_q), .size_code_i(op_size_code_q),
		.ddr_word_base_i(op_stage_word_base_q),
		.die_busy_i(die_busy_i), .event0_i(event0_i), .block0_i(block0_i),
		.event1_i(event1_i), .block1_i(block1_i),
		.busy_o(mover_busy_w), .done_o(mover_done_w),
		.stale_o(mover_stale_w), .rejected_o(mover_rejected_w),
		.p2_req_o(mover_p2_req_w), .p2_we_o(mover_p2_we_w),
		.p2_addr_o(mover_p2_addr_w), .p2_wdata_o(mover_p2_wdata_w),
		.p2_be_o(mover_p2_be_w), .p2_ready_i(p2_ready_i),
		.p2_done_i(p2_done_i), .p2_rdata_i(p2_rdata_i),
		.ddr_addr_o(mover_ddr_addr_w), .ddr_din_o(mover_ddr_din_w),
		.ddr_req_o(mover_ddr_req_w), .ddr_rnw_o(mover_ddr_rnw_w),
		.ddr_be_o(mover_ddr_be_w), .ddr_dout_i(ddr_dout_i),
		.ddr_ready_i(mover_owner_w && ddr_ready_i)
	);

	// Common atomic pristine-restore/apply primitive used only after load
	// preflight and generic-engine restore authorization.
	wire apply_pause_req_w, apply_busy_w, apply_done_w, apply_rejected_w;
	wire apply_p2_req_w, apply_p2_we_w;
	wire [24:0] apply_p2_addr_w;
	wire [15:0] apply_p2_wdata_w;
	wire [1:0] apply_p2_be_w;
	wire [27:1] apply_ddr_addr_w;
	wire [63:0] apply_ddr_din_w;
	wire apply_ddr_req_w, apply_ddr_rnw_w;
	wire [7:0] apply_ddr_be_w;
	wire apply_owner_w;

	ngp_cart_overlay_apply u_apply
	(
		.clk(clk), .reset(reset), .start_i(state_q == ST_LOAD_APPLY_START),
		.pause_ready_i(pause_ready_i),
		.old_live0_i(live_ledger0_i), .old_live1_i(live_ledger1_i),
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
		.ddr_ready_i(apply_owner_w && ddr_ready_i)
	);

	assign mover_owner_w = (state_q == ST_SAVE_CAPTURE_START) ||
	                       (state_q == ST_SAVE_CAPTURE_WAIT) || mover_busy_w;
	assign apply_owner_w = (state_q == ST_LOAD_APPLY_START) ||
	                       (state_q == ST_LOAD_APPLY_WAIT) || apply_busy_w;

	// Local DDR word walker for packed save/load and manifest transport.
	reg [26:0] local_word_addr_w;
	reg [15:0] local_word_data_w;
	reg        local_req_w;
	reg        local_rnw_w;

	always @* begin
		local_word_addr_w = 27'd0;
		local_word_data_w = 16'd0;
		local_req_w = 1'b0;
		local_rnw_w = 1'b0;
		case (state_q)
			ST_SAVE_COPY_READ: begin
				local_word_addr_w = {3'd0, op_stage_word_base_q} +
				                    {11'd0, op_word_index_q};
				local_req_w = 1'b1;
				local_rnw_w = 1'b1;
			end
			ST_SAVE_COPY_WRITE: begin
				local_word_addr_w = op_slot_word_base_q +
				                    {11'd0, op_word_index_q};
				local_word_data_w = transfer_word_q;
				local_req_w = 1'b1;
			end
			ST_SAVE_BUILD_WAIT: begin
				local_word_addr_w = manifest_word_base_q +
				                    {19'd0, manifest_emit_index_w};
				local_word_data_w = manifest_emit_word_w;
				local_req_w = manifest_emit_valid_w;
			end
			ST_LOAD_HEADER_READ: begin
				local_word_addr_w = {slot_base_dword_q, 1'b0};
				local_req_w = 1'b1;
				local_rnw_w = 1'b1;
			end
			ST_LOAD_MANIFEST_READ: begin
				local_word_addr_w = manifest_word_base_q +
				                    {19'd0, manifest_word_index_q};
				local_req_w = 1'b1;
				local_rnw_w = 1'b1;
			end
			ST_LOAD_PAYLOAD_READ: begin
				local_word_addr_w = op_slot_word_base_q +
				                    {11'd0, op_word_index_q};
				local_req_w = 1'b1;
				local_rnw_w = 1'b1;
			end
			ST_LOAD_PAYLOAD_WRITE: begin
				local_word_addr_w = {3'd0, op_stage_word_base_q} +
				                    {11'd0, op_word_index_q};
				local_word_data_w = transfer_word_q;
				local_req_w = 1'b1;
			end
			default: ;
		endcase
	end

	reg [63:0] local_ddr_din_w;
	reg [7:0] local_ddr_be_w;
	always @* begin
		local_ddr_din_w = 64'd0;
		local_ddr_be_w = local_rnw_w ? 8'hFF : 8'd0;
		case (local_word_addr_w[1:0])
			2'd0: begin local_ddr_din_w[15:0] = local_word_data_w; local_ddr_be_w = local_rnw_w ? 8'hFF : 8'h03; end
			2'd1: begin local_ddr_din_w[31:16] = local_word_data_w; local_ddr_be_w = local_rnw_w ? 8'hFF : 8'h0C; end
			2'd2: begin local_ddr_din_w[47:32] = local_word_data_w; local_ddr_be_w = local_rnw_w ? 8'hFF : 8'h30; end
			default: begin local_ddr_din_w[63:48] = local_word_data_w; local_ddr_be_w = local_rnw_w ? 8'hFF : 8'hC0; end
		endcase
	end

	wire local_ready_w = !apply_owner_w && !mover_owner_w && ddr_ready_i;
	assign manifest_emit_ready_w = (state_q == ST_SAVE_BUILD_WAIT) &&
	                               manifest_emit_valid_w && local_ready_w;

	assign ddr_addr_o = apply_owner_w ? apply_ddr_addr_w :
	                    mover_owner_w ? mover_ddr_addr_w :
	                    ddr_address_for_word(local_word_addr_w[26:2]);
	assign ddr_din_o = apply_owner_w ? apply_ddr_din_w :
	                   mover_owner_w ? mover_ddr_din_w : local_ddr_din_w;
	assign ddr_req_o = apply_owner_w ? apply_ddr_req_w :
	                   mover_owner_w ? mover_ddr_req_w : local_req_w;
	assign ddr_rnw_o = apply_owner_w ? apply_ddr_rnw_w :
	                   mover_owner_w ? mover_ddr_rnw_w : local_rnw_w;
	assign ddr_be_o = apply_owner_w ? apply_ddr_be_w :
	                  mover_owner_w ? mover_ddr_be_w : local_ddr_be_w;

	assign p2_req_o = apply_owner_w ? apply_p2_req_w :
	                  mover_owner_w ? mover_p2_req_w : 1'b0;
	assign p2_we_o = apply_owner_w ? apply_p2_we_w :
	                 mover_owner_w ? mover_p2_we_w : 1'b0;
	assign p2_addr_o = apply_owner_w ? apply_p2_addr_w :
	                   mover_owner_w ? mover_p2_addr_w : 25'd0;
	assign p2_wdata_o = apply_owner_w ? apply_p2_wdata_w :
	                    mover_owner_w ? mover_p2_wdata_w : 16'd0;
	assign p2_be_o = apply_owner_w ? apply_p2_be_w :
	                 mover_owner_w ? mover_p2_be_w : 2'd0;

	assign request_ready_o = (state_q == ST_IDLE) && !mover_busy_w &&
	                         !apply_busy_w && !manifest_busy_w;
	assign busy_o = !request_ready_o;
	assign pause_req_o = pause_hold_q || apply_pause_req_w;
	assign ledger_target0_o = target0_q;
	assign ledger_target1_o = target1_q;

	wire [31:0] manifest_state_size_w = FIXED_STATE_DWORDS +
		{2'd0, manifest_type3_bytes_w[31:2]};
	wire manifest_size_valid_w = (manifest_type3_bytes_w[1:0] == 2'd0) &&
		(stored_state_size_q >= EMPTY_STATE_DWORDS) &&
		(stored_state_size_q <= MAX_STATE_DWORDS) &&
		(stored_state_size_q == manifest_state_size_w);
	wire [26:0] load_payload_end_words_w =
		{1'b0, packed_word_offset_q} + {11'd0, geometry_words_w};
	wire [31:0] load_payload_end_bytes_w =
		{4'd0, load_payload_end_words_w, 1'b0};

	always @(posedge clk) begin
		request_accepted_o     <= 1'b0;
		done_o                 <= 1'b0;
		rejected_o             <= 1'b0;
		rewind_bypass_o        <= 1'b0;
		ledger_adopt_o         <= 1'b0;
		force_flash_read_o     <= 1'b0;
		load_apply_failed_o    <= 1'b0;

		if (reset) begin
			state_q                     <= ST_IDLE;
			pause_hold_q                <= 1'b0;
			slot_base_dword_q           <= 26'd0;
			manifest_word_base_q        <= 27'd0;
			payload_word_base_q         <= 27'd0;
			save_ledger0_q              <= 35'd0;
			save_ledger1_q              <= 35'd0;
			save_valid0_q               <= 35'd0;
			save_valid1_q               <= 35'd0;
			save_packed0_q              <= 35'd0;
			save_packed1_q              <= 35'd0;
			target0_q                   <= 35'd0;
			target1_q                   <= 35'd0;
			clear_index_q               <= 7'd0;
			scan_index_q                <= 7'd0;
			packed_word_offset_q        <= 26'd0;
			op_die_q                    <= 1'b0;
			op_block_q                  <= 6'd0;
			op_index_q                  <= 7'd0;
			op_size_code_q              <= 2'd0;
			op_word_count_q             <= 16'd0;
			op_word_index_q             <= 16'd0;
			op_stage_word_base_q        <= 24'd0;
			op_slot_word_base_q         <= 27'd0;
			op_changed_q                <= 1'b0;
			transfer_word_q             <= 16'd0;
			block_crc_q                 <= 32'hFFFFFFFF;
			final_crc_q                 <= 32'd0;
			expected_crc_q              <= 32'd0;
			manifest_word_index_q       <= 8'd0;
			manifest_feed_word_q        <= 16'd0;
			stored_state_size_q         <= 32'd0;
			apply_done_seen_q           <= 1'b0;
			apply_failed_seen_q         <= 1'b0;
			engine_load_done_seen_q     <= 1'b0;
			save_prepared_o             <= 1'b0;
			load_preflight_ready_o      <= 1'b0;
			load_apply_ready_o          <= 1'b0;
			load_apply_failed_o         <= 1'b0;
			frozen_type3_bytes_o        <= 32'd0;
			frozen_state_size_dwords_o  <= 32'd0;
			frozen_ledger0_o            <= 35'd0;
			frozen_ledger1_o            <= 35'd0;
		end else begin
			// Before the save manifest is built, include newly live blocks and
			// invalidate any pre-capture touched by an exact flash completion.
			if (save_gather_w) begin
				save_ledger0_q <= save_effective0_w;
				save_ledger1_q <= save_effective1_w;
				// A newly inserted block changes every later deterministic
				// payload offset, so discard the provisional packed map. Exact
				// events leave ordering intact and invalidate only their block.
				if (save_map_growth_w) begin
					save_packed0_q <= 35'd0;
					save_packed1_q <= 35'd0;
				end else begin
					if (event0_mask_w != 35'd0)
						save_packed0_q <= save_packed0_q & ~event0_mask_w;
					if (event1_mask_w != 35'd0)
						save_packed1_q <= save_packed1_q & ~event1_mask_w;
				end
				if (event0_mask_w != 35'd0)
					save_valid0_q <= save_valid0_q & ~event0_mask_w;
				if (event1_mask_w != 35'd0)
					save_valid1_q <= save_valid1_q & ~event1_mask_w;
			end
			if (((state_q == ST_SAVE_CAPTURE_START) ||
			     (state_q == ST_SAVE_CAPTURE_WAIT) ||
			     (state_q == ST_SAVE_COPY_READ) ||
			     (state_q == ST_SAVE_COPY_WRITE) ||
			     (state_q == ST_SAVE_CRC_COMMIT)) &&
			    (op_selected_event_w || op_die_busy_w)) op_changed_q <= 1'b1;
			if (((state_q == ST_SAVE_COPY_READ) ||
			     (state_q == ST_SAVE_COPY_WRITE) ||
			     (state_q == ST_SAVE_CRC_COMMIT)) && save_map_growth_w)
				op_changed_q <= 1'b1;

			case (state_q)
				ST_IDLE: begin
					pause_hold_q <= 1'b0;
					save_prepared_o <= 1'b0;
					load_preflight_ready_o <= 1'b0;
					if (request_ready_o && (save_start_i || load_start_i)) begin
						request_accepted_o <= 1'b1;
						if (request_is_rewind_i) begin
							rewind_bypass_o <= 1'b1;
							done_o <= 1'b1;
						end else begin
							slot_base_dword_q <= slot_base_dword_i;
							manifest_word_base_q <= {slot_base_dword_i, 1'b0} +
							                        TYPE3_WORD_OFFSET;
							payload_word_base_q <= {slot_base_dword_i, 1'b0} +
							                       PAYLOAD_WORD_OFFSET;
							if (save_start_i) begin
								save_ledger0_q <= live_ledger0_i | event0_mask_w;
								save_ledger1_q <= live_ledger1_i | event1_mask_w;
								save_valid0_q <= 35'd0;
								save_valid1_q <= 35'd0;
								save_packed0_q <= 35'd0;
								save_packed1_q <= 35'd0;
								clear_index_q <= 7'd0;
								state_q <= ST_SAVE_CLEAR_CRC;
							end else begin
								state_q <= ST_LOAD_HEADER_READ;
							end
						end
					end
				end

				ST_SAVE_CLEAR_CRC: begin
					if (clear_index_q == 7'd69) begin
						scan_index_q <= 7'd0;
						packed_word_offset_q <= 26'd0;
						state_q <= ST_SAVE_SCAN;
					end else begin
						clear_index_q <= clear_index_q + 7'd1;
					end
				end

				ST_SAVE_SCAN: begin
					if (!save_maps_valid_w) begin
						rejected_o <= 1'b1;
						pause_hold_q <= 1'b0;
						state_q <= ST_IDLE;
					end else if (save_map_growth_w) begin
						// Restart immediately because the provisional stream offsets
						// were derived from an older, smaller ledger.
						scan_index_q <= 7'd0;
						packed_word_offset_q <= 26'd0;
					end else if (scan_selected_w && !geometry_valid_w) begin
						rejected_o <= 1'b1;
						pause_hold_q <= 1'b0;
						state_q <= ST_IDLE;
					end else if (scan_selected_w && !scan_valid_w &&
					             (scan_die_w ? die_busy_i[1] : die_busy_i[0])) begin
						// Do not spend a complete block transfer on data already
						// known to be changing. Exact completion will be gathered,
						// then this same scan position captures the stable block.
					end else if (scan_selected_w && !scan_valid_w) begin
						op_die_q <= scan_die_w;
						op_block_q <= scan_block_w;
						op_index_q <= scan_index_q;
						op_size_code_q <= scan_size_code_w;
						op_word_count_q <= geometry_words_w;
						op_stage_word_base_q <= STAGING_WORD_BASE + scan_physical_word_w;
						op_slot_word_base_q <= payload_word_base_q +
						                       {1'b0, packed_word_offset_q};
						op_changed_q <= scan_die_w ? die_busy_i[1] : die_busy_i[0];
						if (scan_die_w) save_packed1_q[scan_block_w] <= 1'b0;
						else            save_packed0_q[scan_block_w] <= 1'b0;
						state_q <= ST_SAVE_CAPTURE_START;
					end else if (scan_selected_w && !scan_packed_w) begin
						// Pack coherent staging speculatively. The paused final scan
						// will reuse it only if no later map growth or event invalidates it.
						op_die_q <= scan_die_w;
						op_block_q <= scan_block_w;
						op_index_q <= scan_index_q;
						op_size_code_q <= scan_size_code_w;
						op_word_count_q <= geometry_words_w;
						op_word_index_q <= 16'd0;
						op_stage_word_base_q <= STAGING_WORD_BASE + scan_physical_word_w;
						op_slot_word_base_q <= payload_word_base_q +
						                       {1'b0, packed_word_offset_q};
						op_changed_q <= 1'b0;
						block_crc_q <= 32'hFFFFFFFF;
						state_q <= ST_SAVE_COPY_READ;
					end else begin
						if (scan_selected_w)
							packed_word_offset_q <= packed_offset_next_w;
						if (scan_index_q == 7'd69) begin
							if (!pause_hold_q) begin
								pause_hold_q <= 1'b1;
								state_q <= ST_SAVE_WAIT_PAUSE;
							end else if (!save_all_valid_w || !save_all_packed_w ||
							             any_event_w || save_map_growth_w ||
							             (die_busy_i != 2'd0)) begin
								scan_index_q <= 7'd0;
								packed_word_offset_q <= 26'd0;
							end else begin
								save_ledger0_q <= save_effective0_w;
								save_ledger1_q <= save_effective1_w;
								frozen_ledger0_o <= save_effective0_w;
								frozen_ledger1_o <= save_effective1_w;
								state_q <= ST_SAVE_BUILD_START;
							end
						end else begin
							scan_index_q <= scan_index_q + 7'd1;
						end
					end
				end

				ST_SAVE_CAPTURE_START: state_q <= ST_SAVE_CAPTURE_WAIT;

				ST_SAVE_CAPTURE_WAIT: begin
					if (mover_rejected_w) begin
						rejected_o <= 1'b1;
						pause_hold_q <= 1'b0;
						state_q <= ST_IDLE;
					end else if (mover_done_w) begin
						if (mover_stale_w || op_invalid_now_w) begin
							scan_index_q <= 7'd0;
							packed_word_offset_q <= 26'd0;
							state_q <= ST_SAVE_SCAN;
						end else begin
							if (op_die_q) save_valid1_q[op_block_q] <= 1'b1;
							else          save_valid0_q[op_block_q] <= 1'b1;
							scan_index_q <= 7'd0;
							packed_word_offset_q <= 26'd0;
							state_q <= ST_SAVE_SCAN;
						end
					end
				end

				ST_SAVE_COPY_READ: begin
					if (local_ready_w) begin
						transfer_word_q <= select_word64(ddr_dout_i,
						                                  local_word_addr_w[1:0]);
						state_q <= ST_SAVE_COPY_WRITE;
					end
				end

				ST_SAVE_COPY_WRITE: begin
					if (local_ready_w) begin
						block_crc_q <= crc_after_word_w;
						if (op_word_index_q == (op_word_count_q - 16'd1)) begin
							if (save_stream_invalid_w) begin
								scan_index_q <= 7'd0;
								packed_word_offset_q <= 26'd0;
								state_q <= ST_SAVE_SCAN;
							end else begin
								final_crc_q <= ~crc_after_word_w;
								state_q <= ST_SAVE_CRC_COMMIT;
							end
						end else begin
							op_word_index_q <= op_word_index_q + 16'd1;
							state_q <= ST_SAVE_COPY_READ;
						end
					end
				end

				ST_SAVE_CRC_COMMIT: begin
					if (save_stream_invalid_w) begin
						scan_index_q <= 7'd0;
						packed_word_offset_q <= 26'd0;
						state_q <= ST_SAVE_SCAN;
					end else begin
						if (op_die_q) save_packed1_q[op_block_q] <= 1'b1;
						else          save_packed0_q[op_block_q] <= 1'b1;
						packed_word_offset_q <= packed_offset_next_w;
						if (op_index_q == 7'd69) begin
							scan_index_q <= 7'd0;
							packed_word_offset_q <= 26'd0;
							state_q <= ST_SAVE_SCAN;
						end else begin
							scan_index_q <= op_index_q + 7'd1;
							state_q <= ST_SAVE_SCAN;
						end
					end
				end

				ST_SAVE_WAIT_PAUSE: begin
					if (pause_ready_i && (die_busy_i == 2'd0)) begin
						scan_index_q <= 7'd0;
						packed_word_offset_q <= 26'd0;
						state_q <= ST_SAVE_SCAN;
					end
				end

				ST_SAVE_BUILD_START: state_q <= ST_SAVE_BUILD_WAIT;

				ST_SAVE_BUILD_WAIT: begin
					if (manifest_build_rejected_w) begin
						rejected_o <= 1'b1;
						pause_hold_q <= 1'b0;
						state_q <= ST_IDLE;
					end else if (manifest_emit_done_w) begin
						frozen_type3_bytes_o <= manifest_build_type3_bytes_w;
						frozen_state_size_dwords_o <= FIXED_STATE_DWORDS +
							{2'd0, manifest_build_type3_bytes_w[31:2]};
						save_prepared_o <= 1'b1;
						state_q <= ST_SAVE_WAIT_ENGINE;
					end
				end

				ST_SAVE_WAIT_ENGINE: begin
					if (engine_save_done_i) begin
						save_prepared_o <= 1'b0;
						pause_hold_q <= 1'b0;
						done_o <= 1'b1;
						state_q <= ST_IDLE;
					end else if (cancel_i) begin
						save_prepared_o <= 1'b0;
						pause_hold_q <= 1'b0;
						rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end
				end

				ST_LOAD_HEADER_READ: begin
					if (local_ready_w) begin
						stored_state_size_q <= ddr_dout_i[63:32];
						if ((ddr_dout_i[63:32] < EMPTY_STATE_DWORDS) ||
						    (ddr_dout_i[63:32] > MAX_STATE_DWORDS)) begin
							rejected_o <= 1'b1;
							state_q <= ST_IDLE;
						end else begin
							state_q <= ST_LOAD_MANIFEST_START;
						end
					end
				end

				ST_LOAD_MANIFEST_START: begin
					manifest_word_index_q <= 8'd0;
					state_q <= ST_LOAD_MANIFEST_READ;
				end

				ST_LOAD_MANIFEST_READ: begin
					if (local_ready_w) begin
						manifest_feed_word_q <= select_word64(ddr_dout_i,
						                                      local_word_addr_w[1:0]);
						state_q <= ST_LOAD_MANIFEST_FEED;
					end
				end

				ST_LOAD_MANIFEST_FEED: begin
					if (manifest_word_ready_w) begin
						if (manifest_word_index_q == 8'hFF)
							state_q <= ST_LOAD_MANIFEST_WAIT;
						else begin
							manifest_word_index_q <= manifest_word_index_q + 8'd1;
							state_q <= ST_LOAD_MANIFEST_READ;
						end
					end
				end

				ST_LOAD_MANIFEST_WAIT: begin
					if (manifest_rejected_w) begin
						rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end else if (manifest_accepted_w) begin
						if (!manifest_size_valid_w) begin
							rejected_o <= 1'b1;
							state_q <= ST_IDLE;
						end else begin
							target0_q <= manifest_ledger0_w;
							target1_q <= manifest_ledger1_w;
							frozen_ledger0_o <= manifest_ledger0_w;
							frozen_ledger1_o <= manifest_ledger1_w;
							frozen_type3_bytes_o <= manifest_type3_bytes_w;
							frozen_state_size_dwords_o <= stored_state_size_q;
							scan_index_q <= 7'd0;
							packed_word_offset_q <= 26'd0;
							state_q <= ST_LOAD_SCAN;
						end
					end
				end

				ST_LOAD_SCAN: begin
					if (scan_selected_w) begin
						if (!geometry_valid_w) begin
							rejected_o <= 1'b1;
							state_q <= ST_IDLE;
						end else begin
							op_die_q <= scan_die_w;
							op_block_q <= scan_block_w;
							op_index_q <= scan_index_q;
							op_size_code_q <= scan_size_code_w;
							op_word_count_q <= geometry_words_w;
							op_stage_word_base_q <= STAGING_WORD_BASE +
							                        scan_physical_word_w;
							op_slot_word_base_q <= payload_word_base_q +
							                       {1'b0, packed_word_offset_q};
							state_q <= ST_LOAD_CRC_REQ;
						end
					end else if (scan_index_q == 7'd69) begin
						if ({5'd0, packed_word_offset_q, 1'b0} !=
						    manifest_payload_bytes_w) begin
							rejected_o <= 1'b1;
							state_q <= ST_IDLE;
						end else begin
							load_preflight_ready_o <= 1'b1;
							state_q <= ST_LOAD_WAIT_BEGIN;
						end
					end else begin
						scan_index_q <= scan_index_q + 7'd1;
					end
				end

				ST_LOAD_CRC_REQ: state_q <= ST_LOAD_CRC_LATCH;

				ST_LOAD_CRC_LATCH: begin
					expected_crc_q <= manifest_crc_meta_data_w;
					op_word_index_q <= 16'd0;
					block_crc_q <= 32'hFFFFFFFF;
					state_q <= ST_LOAD_PAYLOAD_READ;
				end

				ST_LOAD_PAYLOAD_READ: begin
					if (local_ready_w) begin
						transfer_word_q <= select_word64(ddr_dout_i,
						                                  local_word_addr_w[1:0]);
						state_q <= ST_LOAD_PAYLOAD_WRITE;
					end
				end

				ST_LOAD_PAYLOAD_WRITE: begin
					if (local_ready_w) begin
						block_crc_q <= crc_after_word_w;
						if (op_word_index_q == (op_word_count_q - 16'd1)) begin
							if ((~crc_after_word_w) != expected_crc_q) begin
								rejected_o <= 1'b1;
								state_q <= ST_IDLE;
							end else if (scan_index_q == 7'd69) begin
								if (load_payload_end_bytes_w !=
								    manifest_payload_bytes_w) begin
									rejected_o <= 1'b1;
									state_q <= ST_IDLE;
								end else begin
									packed_word_offset_q <= load_payload_end_words_w[25:0];
									load_preflight_ready_o <= 1'b1;
									state_q <= ST_LOAD_WAIT_BEGIN;
								end
							end else begin
								packed_word_offset_q <= packed_offset_next_w;
								scan_index_q <= scan_index_q + 7'd1;
								state_q <= ST_LOAD_SCAN;
							end
						end else begin
							op_word_index_q <= op_word_index_q + 16'd1;
							state_q <= ST_LOAD_PAYLOAD_READ;
						end
					end
				end

				ST_LOAD_WAIT_BEGIN: begin
					if (engine_restore_begin_i) begin
						load_preflight_ready_o <= 1'b0;
						load_apply_ready_o <= 1'b0;
						pause_hold_q <= 1'b1;
						apply_done_seen_q <= 1'b0;
						apply_failed_seen_q <= 1'b0;
						engine_load_done_seen_q <= engine_load_done_i;
						state_q <= ST_LOAD_WAIT_GEOMETRY;
					end else if (cancel_i) begin
						load_preflight_ready_o <= 1'b0;
						rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end
				end

				ST_LOAD_WAIT_GEOMETRY: begin
					// The generic engine has issued its synchronous power-on reset.
					// ngp_cart intentionally reloads die population from the retained
					// image afterward; starting earlier would validate the nonempty
					// target map against transient zero size codes.
					if (engine_load_done_i) engine_load_done_seen_q <= 1'b1;
					if (geometry_ready_i) state_q <= ST_LOAD_APPLY_START;
				end

				ST_LOAD_APPLY_START: state_q <= ST_LOAD_APPLY_WAIT;

				ST_LOAD_APPLY_WAIT: begin
					if (apply_done_w) apply_done_seen_q <= 1'b1;
					if (apply_rejected_w) apply_failed_seen_q <= 1'b1;
					if (engine_load_done_i) engine_load_done_seen_q <= 1'b1;
					if (apply_rejected_w || apply_failed_seen_q) begin
						// The generic engine is still held before its first
						// internals/RAM write. Release that reset-held transaction as
						// a clean rejection instead of waiting for engine_load_done.
						pause_hold_q <= 1'b0;
						load_apply_ready_o <= 1'b0;
						load_apply_failed_o <= 1'b1;
						rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end else if (apply_done_w || apply_done_seen_q) begin
						// Hold this level until the generic engine has observed it,
						// released reset, re-parked, and restored CPU/RAM state.
						load_apply_ready_o <= 1'b1;
					end
					if ((apply_done_seen_q || apply_done_w) &&
					    (engine_load_done_seen_q || engine_load_done_i)) begin
						pause_hold_q <= 1'b0;
						load_apply_ready_o <= 1'b0;
						force_flash_read_o <= 1'b1;
						ledger_adopt_o <= 1'b1;
						done_o <= 1'b1;
						state_q <= ST_IDLE;
					end
				end

				default: begin
					rejected_o <= 1'b1;
					pause_hold_q <= 1'b0;
					load_apply_ready_o <= 1'b0;
					state_q <= ST_IDLE;
				end
			endcase
		end
	end

	// Parser/build-only status fields and child commit pulses are intentionally
	// not control inputs at this level. The store supplies the stronger public
	// barriers documented above.
	/* verilator lint_off UNUSED */
	wire unused_ok = &{1'b0, manifest_valid_w, manifest_included_blocks_w,
		manifest_build_payload_bytes_w, manifest_build_included_blocks_w,
		geometry_base_w[0], geometry_bytes_w, 1'b0};
	/* verilator lint_on UNUSED */

endmodule

`default_nettype wire
