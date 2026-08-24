// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_cart_overlay_directory -- sector codec for the frozen NGPC sparse S0
// V1 directory. It has no HPS, DDR3, or live-cart side effects: the overlay
// controller owns those transactions and presents complete sectors here.
//
// A read starts with sector_start_i and supplies exactly 256 little-endian
// 16-bit words through sector_word_*_. A successful input updates the exposed
// directory metadata atomically; an unsuccessful input leaves the prior
// accepted directory intact. The 70 entry records are held in one M10K.
//
// entry_config_* programs that same entry store while idle. build_start_i
// snapshots the supplied directory fields, checks the complete allocation
// table, then emits one V1 sector through emit_*_. Gaps between emitted entry
// triplets are legal; emit_word_index_o always identifies the frozen sector
// word being transferred.
//
// Validation uses serialized M10K scans rather than a large register array.
// The first scan proves every entry's local geometry/bounds rules and totals
// the allocated two-copy spans. A bounded pairwise scan then proves that no
// two spans overlap. For a normal directory, non-overlap plus
// sum(spans) == next_free_lba - 2 proves gap-free coverage from LBA 2 without
// imposing entry-index order: a low-numbered block may legitimately be
// allocated after a high-numbered block in a later save generation.
`timescale 1ns/1ps
module ngp_cart_overlay_directory
(
	input  wire        clk,
	input  wire        reset,

	// Exact identity of the currently loaded raw image. title_i[7:0] is raw
	// title byte 0, title_i[15:8] is byte 1, and so on.
	input  wire [31:0] identity_raw_crc32_i,
	input  wire [31:0] identity_raw_bytes_i,
	input  wire [31:0] identity_pristine_crc32_i,
	input  wire [31:0] identity_physical_bytes_i,
	input  wire [1:0]  identity_die0_code_i,
	input  wire [1:0]  identity_die1_code_i,
	input  wire [15:0] identity_catalog_i,
	input  wire [7:0]  identity_subcatalog_i,
	input  wire [95:0] identity_title_i,

	// If asserted, next_free is also checked against this actual mounted-file
	// size. A normal directory may have a smaller declared EOF because Main
	// cannot truncate a stale prior tail.
	input  wire        file_sectors_valid_i,
	input  wire [15:0] file_sectors_i,

	// Sequential sector input.
	input  wire        sector_start_i,
	input  wire        sector_word_valid_i,
	input  wire [15:0] sector_word_i,
	output wire        sector_word_ready_o,
	output reg         accepted_o,
	output reg         rejected_o,

	// Last atomically accepted directory.
	output reg         directory_valid_o,
	output reg  [31:0] generation_o,
	output reg  [7:0]  flags_o,
	output reg  [34:0] ledger0_o,
	output reg  [34:0] ledger1_o,
	output reg  [34:0] active0_o,
	output reg  [34:0] active1_o,
	output reg  [15:0] next_free_lba_o,

	// Entry RAM write/read interface. The metadata read is one clock late,
	// matching cache_ram_dp_be's synchronous B port.
	input  wire        entry_config_we_i,
	input  wire [6:0]  entry_config_index_i,
	input  wire [15:0] entry_config_base_lba_i,
	input  wire [31:0] entry_config_crc32_i,
	input  wire [6:0]  entry_meta_index_i,
	output wire [15:0] entry_meta_base_lba_o,
	output wire [31:0] entry_meta_crc32_o,

	// Sequential sector builder. Entry configuration must be stable before
	// build_start_i; writes are deliberately disabled during a build.
	input  wire        build_start_i,
	input  wire [31:0] build_generation_i,
	input  wire [7:0]  build_flags_i,
	input  wire [34:0] build_ledger0_i,
	input  wire [34:0] build_ledger1_i,
	input  wire [34:0] build_active0_i,
	input  wire [34:0] build_active1_i,
	input  wire [15:0] build_next_free_lba_i,
	output reg         build_rejected_o,
	output reg         emit_valid_o,
	output reg  [7:0]  emit_word_index_o,
	output reg  [15:0] emit_word_o,
	input  wire        emit_ready_i,
	output reg         emit_done_o
);

	localparam [7:0]  FORMAT_VERSION       = 8'd1;
	localparam [15:0] PROTECTED_WORDS      = 16'd508;
	localparam [15:0] MAX_FILE_SECTORS     = 16'd16386;
	localparam [7:0]  WORD_ENTRIES_FIRST   = 8'd40;
	localparam [7:0]  WORD_ENTRIES_LAST    = 8'd249;

	localparam [3:0] ST_IDLE             = 4'd0;
	localparam [3:0] ST_READ             = 4'd1;
	localparam [3:0] ST_SCAN_ENTRY_REQ   = 4'd2;
	localparam [3:0] ST_SCAN_ENTRY_CHECK = 4'd3;
	localparam [3:0] ST_SCAN_OUTER_REQ   = 4'd4;
	localparam [3:0] ST_SCAN_OUTER_CHECK = 4'd5;
	localparam [3:0] ST_SCAN_INNER_REQ   = 4'd6;
	localparam [3:0] ST_SCAN_INNER_CHECK = 4'd7;
	localparam [3:0] ST_EMIT_HEADER      = 4'd8;
	localparam [3:0] ST_EMIT_ENTRY_FETCH = 4'd9;
	localparam [3:0] ST_EMIT_ENTRY       = 4'd10;
	localparam [3:0] ST_EMIT_TAIL        = 4'd11;
	localparam [3:0] ST_EMIT_CRC_LO      = 4'd12;
	localparam [3:0] ST_EMIT_CRC_HI      = 4'd13;

	reg [3:0]  state_q;
	reg [7:0]  read_word_index_q;
	reg [31:0] read_crc_q;
	reg [15:0] read_crc_lo_q;
	reg         read_bad_q;
	reg [31:0] read_generation_q;
	reg [7:0]  read_flags_q;
	reg [63:0] read_ledger0_q;
	reg [63:0] read_ledger1_q;
	reg [71:0] read_active_q;
	reg [15:0] read_next_free_q;
	reg [6:0]  read_entry_index_q;
	reg [1:0]  read_entry_word_q;
	reg [15:0] read_entry_base_q;
	reg [15:0] read_entry_crc_lo_q;

	reg [31:0] build_generation_q;
	reg [7:0]  build_flags_q;
	reg [34:0] build_ledger0_q;
	reg [34:0] build_ledger1_q;
	reg [34:0] build_active0_q;
	reg [34:0] build_active1_q;
	reg [15:0] build_next_free_q;
	reg         scan_bad_q;
	reg [6:0]  scan_entry_index_q;
	reg         scan_read_q;
	reg [6:0]  scan_outer_index_q;
	reg [6:0]  scan_inner_index_q;
	reg [15:0] scan_outer_base_q;
	reg [16:0] scan_outer_end_q;
	reg [16:0] scan_allocated_sectors_q;
	reg [7:0]  emit_header_index_q;
	reg [6:0]  emit_entry_index_q;
	reg [1:0]  emit_entry_word_q;
	reg [7:0]  emit_entry_sector_index_q;
	reg [1:0]  emit_tail_index_q;
	reg [31:0] emit_crc_q;

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
		reg [31:0] crc_v;
		begin
			crc_v = crc32_byte(crc_i, data_i[7:0]);
			crc32_word = crc32_byte(crc_v, data_i[15:8]);
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

	function automatic [5:0] die_block_count;
		input [1:0] code_i;
		begin
			case (code_i)
				2'd1: die_block_count = 6'd11;
				2'd2: die_block_count = 6'd19;
				2'd3: die_block_count = 6'd35;
				default: die_block_count = 6'd0;
			endcase
		end
	endfunction

	function automatic [34:0] die_block_mask;
		input [1:0] code_i;
		begin
			case (code_i)
				2'd1: die_block_mask = 35'h0000007FF;
				2'd2: die_block_mask = 35'h00007FFFF;
				2'd3: die_block_mask = 35'h7FFFFFFFF;
				default: die_block_mask = 35'd0;
			endcase
		end
	endfunction

	function automatic [15:0] two_copy_sector_count;
		input [1:0] code_i;
		input [5:0] block_i;
		begin
			two_copy_sector_count = 16'd0;
			case (code_i)
				2'd1: begin
					if (block_i < 6'd7) two_copy_sector_count = 16'd256;
					else case (block_i)
						6'd7: two_copy_sector_count = 16'd128;
						6'd8, 6'd9: two_copy_sector_count = 16'd32;
						6'd10: two_copy_sector_count = 16'd64;
						default: ;
					endcase
				end
				2'd2: begin
					if (block_i < 6'd15) two_copy_sector_count = 16'd256;
					else case (block_i)
						6'd15: two_copy_sector_count = 16'd128;
						6'd16, 6'd17: two_copy_sector_count = 16'd32;
						6'd18: two_copy_sector_count = 16'd64;
						default: ;
					endcase
				end
				2'd3: begin
					if (block_i < 6'd31) two_copy_sector_count = 16'd256;
					else case (block_i)
						6'd31: two_copy_sector_count = 16'd128;
						6'd32, 6'd33: two_copy_sector_count = 16'd32;
						6'd34: two_copy_sector_count = 16'd64;
						default: ;
					endcase
				end
				default: ;
			endcase
		end
	endfunction

	function automatic [15:0] entry_two_copy_sector_count;
		input [6:0] index_i;
		input [1:0] die0_code_i;
		input [1:0] die1_code_i;
		reg [5:0] block_v;
		reg [1:0] code_v;
		begin
			if (index_i < 7'd35) begin
				block_v = index_i[5:0];
				code_v = die0_code_i;
			end else begin
				// Restore the high index bit around the fixed die-1 origin.
				if (index_i[6]) block_v = index_i[5:0] + 6'd29;
				else            block_v = index_i[5:0] - 6'd35;
				code_v = die1_code_i;
			end
			entry_two_copy_sector_count = two_copy_sector_count(code_v, block_v);
		end
	endfunction

	function automatic entry_bad;
		input [6:0] index_i;
		input [15:0] base_i;
		input [31:0] crc_i;
		input [34:0] ledger0_i;
		input [34:0] ledger1_i;
		input [34:0] active0_i;
		input [34:0] active1_i;
		input [1:0] die0_code_i;
		input [1:0] die1_code_i;
		input [15:0] next_free_i;
		reg [5:0] block_v;
		reg [1:0] code_v;
		reg       valid_v;
		reg       live_v;
		reg       active_v;
		reg [16:0] end_v;
		begin
			if (index_i < 7'd35) begin
				block_v  = index_i[5:0];
				code_v   = die0_code_i;
				live_v   = ledger0_i[block_v];
				active_v = active0_i[block_v];
			end else begin
				// index 64..69 needs the high index bit restored after
				// subtracting the fixed die-1 entry origin (35).
				if (index_i[6]) block_v = index_i[5:0] + 6'd29;
				else            block_v = index_i[5:0] - 6'd35;
				code_v   = die1_code_i;
				live_v   = ledger1_i[block_v];
				active_v = active1_i[block_v];
			end
			valid_v = block_v < die_block_count(code_v);
			entry_bad = 1'b0;
			if (!valid_v) begin
				if ((base_i != 16'd0) || (crc_i != 32'd0)) entry_bad = 1'b1;
			end else if (base_i == 16'd0) begin
				if (live_v || active_v || (crc_i != 32'd0)) entry_bad = 1'b1;
			end else begin
				end_v = {1'b0, base_i} + {1'b0, two_copy_sector_count(code_v, block_v)};
				if (base_i < 16'd2) entry_bad = 1'b1;
				if (end_v > {1'b0, next_free_i}) entry_bad = 1'b1;
				if (!live_v && (active_v || (crc_i != 32'd0))) entry_bad = 1'b1;
			end
		end
	endfunction

	function automatic [15:0] header_word;
		input [7:0] index_i;
		input [31:0] generation_i;
		input [7:0] flags_i;
		input [34:0] ledger0_i;
		input [34:0] ledger1_i;
		input [34:0] active0_i;
		input [34:0] active1_i;
		input [15:0] next_free_i;
		reg [71:0] active_v;
		begin
			active_v = {2'b00, active1_i, active0_i};
			header_word = 16'd0;
			case (index_i)
				8'd0:  header_word = 16'h474E; // "NG"
				8'd1:  header_word = 16'h5350; // "PS"
				8'd2:  header_word = 16'h564F; // "OV"
				8'd3:  header_word = 16'h0031; // "1\0"
				8'd4:  header_word = {flags_i, FORMAT_VERSION};
				8'd5:  header_word = PROTECTED_WORDS;
				8'd6:  header_word = generation_i[15:0];
				8'd7:  header_word = generation_i[31:16];
				8'd8:  header_word = identity_raw_crc32_i[15:0];
				8'd9:  header_word = identity_raw_crc32_i[31:16];
				8'd10: header_word = identity_raw_bytes_i[15:0];
				8'd11: header_word = identity_raw_bytes_i[31:16];
				8'd12: header_word = identity_pristine_crc32_i[15:0];
				8'd13: header_word = identity_pristine_crc32_i[31:16];
				8'd14: header_word = identity_physical_bytes_i[15:0];
				8'd15: header_word = identity_physical_bytes_i[31:16];
				8'd16: header_word = {6'd0, identity_die1_code_i, 6'd0, identity_die0_code_i};
				8'd17: header_word = identity_catalog_i;
				8'd18: header_word = {8'd0, identity_subcatalog_i};
				8'd19: header_word = identity_title_i[15:0];
				8'd20: header_word = identity_title_i[31:16];
				8'd21: header_word = identity_title_i[47:32];
				8'd22: header_word = identity_title_i[63:48];
				8'd23: header_word = identity_title_i[79:64];
				8'd24: header_word = identity_title_i[95:80];
				8'd25: header_word = ledger0_i[15:0];
				8'd26: header_word = ledger0_i[31:16];
				8'd27: header_word = {13'd0, ledger0_i[34:32]};
				8'd28: header_word = 16'd0;
				8'd29: header_word = ledger1_i[15:0];
				8'd30: header_word = ledger1_i[31:16];
				8'd31: header_word = {13'd0, ledger1_i[34:32]};
				8'd32: header_word = 16'd0;
				8'd33: header_word = active_v[15:0];
				8'd34: header_word = active_v[31:16];
				8'd35: header_word = active_v[47:32];
				8'd36: header_word = active_v[63:48];
				8'd37: header_word = {8'd0, active_v[71:64]};
				8'd38: header_word = next_free_i;
				default: ;
			endcase
		end
	endfunction

	wire [21:0] identity_die_bytes0_w = die_bytes(identity_die0_code_i);
	wire [21:0] identity_die_bytes1_w = die_bytes(identity_die1_code_i);
	wire [22:0] identity_total_bytes_w = {1'b0, identity_die_bytes0_w} +
	                                      {1'b0, identity_die_bytes1_w};
	wire identity_valid_w = (identity_raw_bytes_i != 32'd0) &&
	                        (identity_physical_bytes_i != 32'd0) &&
	                        (identity_raw_bytes_i <= identity_physical_bytes_i) &&
	                        (identity_physical_bytes_i == {9'd0, identity_total_bytes_w});

	wire [34:0] read_valid_mask0_w = die_block_mask(identity_die0_code_i);
	wire [34:0] read_valid_mask1_w = die_block_mask(identity_die1_code_i);
	wire read_maps_valid_w = ((read_ledger0_q[63:35] == 29'd0) &&
	                         (read_ledger1_q[63:35] == 29'd0) &&
	                         ((read_ledger0_q[34:0] & ~read_valid_mask0_w) == 35'd0) &&
	                         ((read_ledger1_q[34:0] & ~read_valid_mask1_w) == 35'd0) &&
	                         ((read_active_q[34:0] & ~read_valid_mask0_w) == 35'd0) &&
	                         ((read_active_q[69:35] & ~read_valid_mask1_w) == 35'd0) &&
	                         (read_active_q[71:70] == 2'd0));
	wire read_next_free_valid_w = (read_next_free_q >= 16'd2) &&
	                              (read_next_free_q <= MAX_FILE_SECTORS) &&
	                              (!file_sectors_valid_i || (read_next_free_q <= file_sectors_i));

	wire [34:0] build_valid_mask0_w = die_block_mask(identity_die0_code_i);
	wire [34:0] build_valid_mask1_w = die_block_mask(identity_die1_code_i);
	wire build_maps_valid_w = ((build_ledger0_i & ~build_valid_mask0_w) == 35'd0) &&
	                          ((build_ledger1_i & ~build_valid_mask1_w) == 35'd0) &&
	                          ((build_active0_i & ~build_valid_mask0_w) == 35'd0) &&
	                          ((build_active1_i & ~build_valid_mask1_w) == 35'd0) &&
	                          (build_flags_i == 8'd0) &&
	                          (build_next_free_lba_i >= 16'd2) &&
	                          (build_next_free_lba_i <= MAX_FILE_SECTORS);

	wire sector_word_fire_w = sector_word_valid_i && sector_word_ready_o;
	assign sector_word_ready_o = (state_q == ST_READ);

	wire [34:0] scan_ledger0_w = scan_read_q ? read_ledger0_q[34:0] : build_ledger0_q;
	wire [34:0] scan_ledger1_w = scan_read_q ? read_ledger1_q[34:0] : build_ledger1_q;
	wire [34:0] scan_active0_w = scan_read_q ? read_active_q[34:0] : build_active0_q;
	wire [34:0] scan_active1_w = scan_read_q ? read_active_q[69:35] : build_active1_q;
	wire [15:0] scan_next_free_w = scan_read_q ? read_next_free_q : build_next_free_q;

	// The single entry RAM has one write source at a time. Its B port is reused
	// by the builder scanner/emitter; outside a build it is the public metadata
	// read port.
	wire read_entry_write_w = sector_word_fire_w &&
	                          (read_word_index_q >= WORD_ENTRIES_FIRST) &&
	                          (read_word_index_q <= WORD_ENTRIES_LAST) &&
	                          (read_entry_word_q == 2'd2);
	wire entry_ram_we_a_w = read_entry_write_w ||
	                         ((state_q == ST_IDLE) && entry_config_we_i);
	wire [6:0] entry_ram_addr_a_w = read_entry_write_w ? read_entry_index_q :
	                               entry_config_index_i;
	wire [47:0] entry_ram_data_a_w = read_entry_write_w ?
	                                {sector_word_i, read_entry_crc_lo_q, read_entry_base_q} :
	                                {entry_config_crc32_i, entry_config_base_lba_i};
	wire [6:0] entry_ram_addr_b_w = ((state_q == ST_SCAN_ENTRY_REQ) ||
	                                (state_q == ST_SCAN_ENTRY_CHECK)) ? scan_entry_index_q :
	                               ((state_q == ST_SCAN_OUTER_REQ) ||
	                                (state_q == ST_SCAN_OUTER_CHECK)) ? scan_outer_index_q :
	                               ((state_q == ST_SCAN_INNER_REQ) ||
	                                (state_q == ST_SCAN_INNER_CHECK)) ? scan_inner_index_q :
	                               ((state_q == ST_EMIT_ENTRY_FETCH) ||
	                                (state_q == ST_EMIT_ENTRY)) ? emit_entry_index_q :
	                               entry_meta_index_i;
	wire [47:0] entry_ram_q_b_w;
	/* verilator lint_off UNUSED */
	wire [47:0] entry_ram_q_a_unused_w;
	/* verilator lint_on UNUSED */

	cache_ram_dp_be #(.ADDR_WIDTH(7), .DATA_WIDTH(48)) u_entries
	(
		.clk_i    (clk),
		.addr_a_i (entry_ram_addr_a_w),
		.wren_a_i (entry_ram_we_a_w),
		.be_a_i   (6'b111111),
		.wdata_a_i(entry_ram_data_a_w),
		.q_a_o    (entry_ram_q_a_unused_w),
		.addr_b_i (entry_ram_addr_b_w),
		.wren_b_i (1'b0),
		.be_b_i   (6'd0),
		.wdata_b_i(48'd0),
		.q_b_o    (entry_ram_q_b_w)
	);

	assign entry_meta_base_lba_o = entry_ram_q_b_w[15:0];
	assign entry_meta_crc32_o = entry_ram_q_b_w[47:16];

	wire scan_entry_bad_w = entry_bad(scan_entry_index_q, entry_ram_q_b_w[15:0],
	                                 entry_ram_q_b_w[47:16], scan_ledger0_w,
	                                 scan_ledger1_w, scan_active0_w,
	                                 scan_active1_w, identity_die0_code_i,
	                                 identity_die1_code_i, scan_next_free_w);
	wire read_entry_bad_w = entry_bad(read_entry_index_q, read_entry_base_q,
	                                 {sector_word_i, read_entry_crc_lo_q},
	                                 read_ledger0_q[34:0], read_ledger1_q[34:0],
	                                 read_active_q[34:0], read_active_q[69:35],
	                                 identity_die0_code_i, identity_die1_code_i,
	                                 read_next_free_q);

	wire [15:0] scan_entry_sectors_w = entry_two_copy_sector_count(
	                                  scan_entry_index_q, identity_die0_code_i,
	                                  identity_die1_code_i);
	wire [15:0] scan_outer_sectors_w = entry_two_copy_sector_count(
	                                  scan_outer_index_q, identity_die0_code_i,
	                                  identity_die1_code_i);
	wire [15:0] scan_inner_sectors_w = entry_two_copy_sector_count(
	                                  scan_inner_index_q, identity_die0_code_i,
	                                  identity_die1_code_i);
	wire [16:0] scan_inner_end_w = {1'b0, entry_ram_q_b_w[15:0]} +
	                                     {1'b0, scan_inner_sectors_w};
	wire scan_inner_overlap_w = (entry_ram_q_b_w[15:0] != 16'd0) &&
	                            ({1'b0, scan_outer_base_q} < scan_inner_end_w) &&
	                            ({1'b0, entry_ram_q_b_w[15:0]} < scan_outer_end_q);
	wire scan_normal_packing_bad_w = (scan_allocated_sectors_q + 17'd2) !=
	                                 {1'b0, scan_next_free_w};

	always @* begin
		emit_valid_o = 1'b0;
		emit_word_index_o = 8'd0;
		emit_word_o = 16'd0;
		case (state_q)
			ST_EMIT_HEADER: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = emit_header_index_q;
				emit_word_o = header_word(emit_header_index_q, build_generation_q,
				                            build_flags_q, build_ledger0_q,
				                            build_ledger1_q, build_active0_q,
				                            build_active1_q, build_next_free_q);
			end
			ST_EMIT_ENTRY: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = emit_entry_sector_index_q;
				case (emit_entry_word_q)
					2'd0: emit_word_o = entry_ram_q_b_w[15:0];
					2'd1: emit_word_o = entry_ram_q_b_w[31:16];
					default: emit_word_o = entry_ram_q_b_w[47:32];
				endcase
			end
			ST_EMIT_TAIL: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = 8'd250 + {6'd0, emit_tail_index_q};
				emit_word_o = 16'd0;
			end
			ST_EMIT_CRC_LO: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = 8'd254;
				emit_word_o = ~emit_crc_q[15:0];
			end
			ST_EMIT_CRC_HI: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = 8'd255;
				emit_word_o = ~emit_crc_q[31:16];
			end
			default: ;
		endcase
	end

	always @(posedge clk) begin
		accepted_o       <= 1'b0;
		rejected_o       <= 1'b0;
		build_rejected_o <= 1'b0;
		emit_done_o      <= 1'b0;

		if (reset) begin
			state_q                  <= ST_IDLE;
			read_word_index_q        <= 8'd0;
			read_crc_q               <= 32'hFFFFFFFF;
			read_crc_lo_q            <= 16'd0;
			read_bad_q               <= 1'b0;
			read_generation_q        <= 32'd0;
			read_flags_q             <= 8'd0;
			read_ledger0_q           <= 64'd0;
			read_ledger1_q           <= 64'd0;
			read_active_q            <= 72'd0;
			read_next_free_q          <= 16'd0;
			read_entry_index_q       <= 7'd0;
			read_entry_word_q        <= 2'd0;
			read_entry_base_q        <= 16'd0;
			read_entry_crc_lo_q      <= 16'd0;
			build_generation_q       <= 32'd0;
			build_flags_q            <= 8'd0;
			build_ledger0_q          <= 35'd0;
			build_ledger1_q          <= 35'd0;
			build_active0_q          <= 35'd0;
			build_active1_q          <= 35'd0;
			build_next_free_q        <= 16'd0;
			scan_bad_q               <= 1'b0;
			scan_entry_index_q       <= 7'd0;
			scan_read_q              <= 1'b0;
			scan_outer_index_q       <= 7'd0;
			scan_inner_index_q       <= 7'd0;
			scan_outer_base_q        <= 16'd0;
			scan_outer_end_q         <= 17'd0;
			scan_allocated_sectors_q <= 17'd0;
			emit_header_index_q      <= 8'd0;
			emit_entry_index_q       <= 7'd0;
			emit_entry_word_q        <= 2'd0;
			emit_entry_sector_index_q <= 8'd40;
			emit_tail_index_q        <= 2'd0;
			emit_crc_q               <= 32'hFFFFFFFF;
			directory_valid_o        <= 1'b0;
			generation_o             <= 32'd0;
			flags_o                  <= 8'd0;
			ledger0_o                <= 35'd0;
			ledger1_o                <= 35'd0;
			active0_o                <= 35'd0;
			active1_o                <= 35'd0;
			next_free_lba_o          <= 16'd0;
		end else begin
			case (state_q)
				ST_IDLE: begin
					if (sector_start_i) begin
						state_q             <= ST_READ;
						read_word_index_q   <= 8'd0;
						read_crc_q          <= 32'hFFFFFFFF;
						read_crc_lo_q       <= 16'd0;
						read_bad_q          <= 1'b0;
						read_generation_q   <= 32'd0;
						read_flags_q        <= 8'd0;
						read_ledger0_q      <= 64'd0;
						read_ledger1_q      <= 64'd0;
						read_active_q       <= 72'd0;
						read_next_free_q    <= 16'd0;
						read_entry_index_q  <= 7'd0;
						read_entry_word_q   <= 2'd0;
					end else if (build_start_i) begin
						build_generation_q <= build_generation_i;
						build_flags_q      <= build_flags_i;
						build_ledger0_q    <= build_ledger0_i;
						build_ledger1_q    <= build_ledger1_i;
						build_active0_q    <= build_active0_i;
						build_active1_q    <= build_active1_i;
						build_next_free_q  <= build_next_free_lba_i;
						scan_bad_q         <= !identity_valid_w || !build_maps_valid_w;
						scan_entry_index_q <= 7'd0;
						scan_read_q        <= 1'b0;
						scan_allocated_sectors_q <= 17'd0;
						state_q            <= ST_SCAN_ENTRY_REQ;
					end
				end

				ST_READ: begin
					if (sector_word_fire_w) begin
						if (read_word_index_q < 8'd254)
							read_crc_q <= crc32_word(read_crc_q, sector_word_i);

						case (read_word_index_q)
							8'd0: if (sector_word_i != 16'h474E) read_bad_q <= 1'b1;
							8'd1: if (sector_word_i != 16'h5350) read_bad_q <= 1'b1;
							8'd2: if (sector_word_i != 16'h564F) read_bad_q <= 1'b1;
							8'd3: if (sector_word_i != 16'h0031) read_bad_q <= 1'b1;
							8'd4: begin
								read_flags_q <= sector_word_i[15:8];
								if ((sector_word_i[7:0] != FORMAT_VERSION) ||
								    (sector_word_i[15:8] != 8'd0)) read_bad_q <= 1'b1;
							end
							8'd5: if (sector_word_i != PROTECTED_WORDS) read_bad_q <= 1'b1;
							8'd6: read_generation_q[15:0] <= sector_word_i;
							8'd7: read_generation_q[31:16] <= sector_word_i;
							8'd8: if (sector_word_i != identity_raw_crc32_i[15:0]) read_bad_q <= 1'b1;
							8'd9: if (sector_word_i != identity_raw_crc32_i[31:16]) read_bad_q <= 1'b1;
							8'd10: if (sector_word_i != identity_raw_bytes_i[15:0]) read_bad_q <= 1'b1;
							8'd11: if (sector_word_i != identity_raw_bytes_i[31:16]) read_bad_q <= 1'b1;
							8'd12: if (sector_word_i != identity_pristine_crc32_i[15:0]) read_bad_q <= 1'b1;
							8'd13: if (sector_word_i != identity_pristine_crc32_i[31:16]) read_bad_q <= 1'b1;
							8'd14: if (sector_word_i != identity_physical_bytes_i[15:0]) read_bad_q <= 1'b1;
							8'd15: if (sector_word_i != identity_physical_bytes_i[31:16]) read_bad_q <= 1'b1;
							8'd16: if (sector_word_i != {6'd0, identity_die1_code_i, 6'd0, identity_die0_code_i}) read_bad_q <= 1'b1;
							8'd17: if (sector_word_i != identity_catalog_i) read_bad_q <= 1'b1;
							8'd18: if (sector_word_i != {8'd0, identity_subcatalog_i}) read_bad_q <= 1'b1;
							8'd19: if (sector_word_i != identity_title_i[15:0]) read_bad_q <= 1'b1;
							8'd20: if (sector_word_i != identity_title_i[31:16]) read_bad_q <= 1'b1;
							8'd21: if (sector_word_i != identity_title_i[47:32]) read_bad_q <= 1'b1;
							8'd22: if (sector_word_i != identity_title_i[63:48]) read_bad_q <= 1'b1;
							8'd23: if (sector_word_i != identity_title_i[79:64]) read_bad_q <= 1'b1;
							8'd24: if (sector_word_i != identity_title_i[95:80]) read_bad_q <= 1'b1;
							8'd25: read_ledger0_q[15:0] <= sector_word_i;
							8'd26: read_ledger0_q[31:16] <= sector_word_i;
							8'd27: begin
								read_ledger0_q[34:32] <= sector_word_i[2:0];
								if (sector_word_i[15:3] != 13'd0) read_bad_q <= 1'b1;
							end
							8'd28: if (sector_word_i != 16'd0) read_bad_q <= 1'b1;
							8'd29: read_ledger1_q[15:0] <= sector_word_i;
							8'd30: read_ledger1_q[31:16] <= sector_word_i;
							8'd31: begin
								read_ledger1_q[34:32] <= sector_word_i[2:0];
								if (sector_word_i[15:3] != 13'd0) read_bad_q <= 1'b1;
							end
							8'd32: if (sector_word_i != 16'd0) read_bad_q <= 1'b1;
							8'd33: read_active_q[15:0] <= sector_word_i;
							8'd34: read_active_q[31:16] <= sector_word_i;
							8'd35: read_active_q[47:32] <= sector_word_i;
							8'd36: read_active_q[63:48] <= sector_word_i;
							8'd37: begin
								read_active_q[71:64] <= sector_word_i[7:0];
								if (sector_word_i[15:8] != 8'd0) read_bad_q <= 1'b1;
							end
							8'd38: read_next_free_q <= sector_word_i;
							8'd39, 8'd250, 8'd251, 8'd252, 8'd253:
								if (sector_word_i != 16'd0) read_bad_q <= 1'b1;
							8'd254: read_crc_lo_q <= sector_word_i;
							default: ;
						endcase

						if ((read_word_index_q >= WORD_ENTRIES_FIRST) &&
						    (read_word_index_q <= WORD_ENTRIES_LAST)) begin
							case (read_entry_word_q)
								2'd0: read_entry_base_q <= sector_word_i;
								2'd1: read_entry_crc_lo_q <= sector_word_i;
								default: if (read_entry_bad_w) read_bad_q <= 1'b1;
							endcase
							if (read_entry_word_q == 2'd2) begin
								read_entry_word_q <= 2'd0;
								read_entry_index_q <= read_entry_index_q + 7'd1;
							end else begin
								read_entry_word_q <= read_entry_word_q + 2'd1;
							end
						end

						if (read_word_index_q == 8'd255) begin
							if (!read_bad_q && identity_valid_w && read_maps_valid_w &&
							    read_next_free_valid_w &&
							    ((~read_crc_q) == {sector_word_i, read_crc_lo_q})) begin
								scan_bad_q               <= 1'b0;
								scan_entry_index_q       <= 7'd0;
								scan_read_q              <= 1'b1;
								scan_allocated_sectors_q <= 17'd0;
								state_q                  <= ST_SCAN_ENTRY_REQ;
							end else begin
								rejected_o <= 1'b1;
								state_q <= ST_IDLE;
							end
						end else begin
							read_word_index_q <= read_word_index_q + 8'd1;
						end
					end
				end

				ST_SCAN_ENTRY_REQ: state_q <= ST_SCAN_ENTRY_CHECK;

				ST_SCAN_ENTRY_CHECK: begin
					if (scan_entry_bad_w) scan_bad_q <= 1'b1;
					if (entry_ram_q_b_w[15:0] != 16'd0) begin
						scan_allocated_sectors_q <= scan_allocated_sectors_q +
						                            {1'b0, scan_entry_sectors_w};
					end
					if (scan_entry_index_q == 7'd69) begin
						if (scan_bad_q || scan_entry_bad_w) begin
							if (scan_read_q) rejected_o <= 1'b1;
							else             build_rejected_o <= 1'b1;
							state_q <= ST_IDLE;
						end else begin
							scan_outer_index_q <= 7'd0;
							state_q <= ST_SCAN_OUTER_REQ;
						end
					end else begin
						scan_entry_index_q <= scan_entry_index_q + 7'd1;
						state_q <= ST_SCAN_ENTRY_REQ;
					end
				end

				ST_SCAN_OUTER_REQ: state_q <= ST_SCAN_OUTER_CHECK;

				ST_SCAN_OUTER_CHECK: begin
					if ((entry_ram_q_b_w[15:0] == 16'd0) ||
					    (scan_outer_index_q == 7'd69)) begin
						if (scan_outer_index_q == 7'd69) begin
							if (scan_normal_packing_bad_w) begin
								if (scan_read_q) rejected_o <= 1'b1;
								else             build_rejected_o <= 1'b1;
								state_q <= ST_IDLE;
							end else if (scan_read_q) begin
								accepted_o        <= 1'b1;
								directory_valid_o <= 1'b1;
								generation_o      <= read_generation_q;
								flags_o           <= read_flags_q;
								ledger0_o         <= read_ledger0_q[34:0];
								ledger1_o         <= read_ledger1_q[34:0];
								active0_o         <= read_active_q[34:0];
								active1_o         <= read_active_q[69:35];
								next_free_lba_o   <= read_next_free_q;
								state_q <= ST_IDLE;
							end else begin
								emit_header_index_q       <= 8'd0;
								emit_entry_index_q        <= 7'd0;
								emit_entry_word_q         <= 2'd0;
								emit_entry_sector_index_q <= WORD_ENTRIES_FIRST;
								emit_tail_index_q         <= 2'd0;
								emit_crc_q                <= 32'hFFFFFFFF;
								state_q                   <= ST_EMIT_HEADER;
							end
						end else begin
							scan_outer_index_q <= scan_outer_index_q + 7'd1;
							state_q <= ST_SCAN_OUTER_REQ;
						end
					end else begin
						scan_outer_base_q <= entry_ram_q_b_w[15:0];
						scan_outer_end_q <= {1'b0, entry_ram_q_b_w[15:0]} +
						                    {1'b0, scan_outer_sectors_w};
						scan_inner_index_q <= scan_outer_index_q + 7'd1;
						state_q <= ST_SCAN_INNER_REQ;
					end
				end

				ST_SCAN_INNER_REQ: state_q <= ST_SCAN_INNER_CHECK;

				ST_SCAN_INNER_CHECK: begin
					if (scan_inner_overlap_w) begin
						if (scan_read_q) rejected_o <= 1'b1;
						else             build_rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end else if (scan_inner_index_q == 7'd69) begin
						scan_outer_index_q <= scan_outer_index_q + 7'd1;
						state_q <= ST_SCAN_OUTER_REQ;
					end else begin
						scan_inner_index_q <= scan_inner_index_q + 7'd1;
						state_q <= ST_SCAN_INNER_REQ;
					end
				end

				ST_EMIT_HEADER: begin
					if (emit_ready_i) begin
						emit_crc_q <= crc32_word(emit_crc_q, emit_word_o);
						if (emit_header_index_q == 8'd39) begin
							emit_entry_index_q <= 7'd0;
							emit_entry_word_q <= 2'd0;
							emit_entry_sector_index_q <= WORD_ENTRIES_FIRST;
							state_q <= ST_EMIT_ENTRY_FETCH;
						end else begin
							emit_header_index_q <= emit_header_index_q + 8'd1;
						end
					end
				end

				ST_EMIT_ENTRY_FETCH: state_q <= ST_EMIT_ENTRY;

				ST_EMIT_ENTRY: begin
					if (emit_ready_i) begin
						emit_crc_q <= crc32_word(emit_crc_q, emit_word_o);
						if (emit_entry_word_q == 2'd2) begin
							if (emit_entry_index_q == 7'd69) begin
								emit_tail_index_q <= 2'd0;
								state_q <= ST_EMIT_TAIL;
							end else begin
								emit_entry_index_q <= emit_entry_index_q + 7'd1;
								emit_entry_word_q <= 2'd0;
								emit_entry_sector_index_q <= emit_entry_sector_index_q + 8'd1;
								state_q <= ST_EMIT_ENTRY_FETCH;
							end
						end else begin
							emit_entry_word_q <= emit_entry_word_q + 2'd1;
							emit_entry_sector_index_q <= emit_entry_sector_index_q + 8'd1;
						end
					end
				end

				ST_EMIT_TAIL: begin
					if (emit_ready_i) begin
						emit_crc_q <= crc32_word(emit_crc_q, 16'd0);
						if (emit_tail_index_q == 2'd3) state_q <= ST_EMIT_CRC_LO;
						else emit_tail_index_q <= emit_tail_index_q + 2'd1;
					end
				end

				ST_EMIT_CRC_LO: if (emit_ready_i) state_q <= ST_EMIT_CRC_HI;

				ST_EMIT_CRC_HI: begin
					if (emit_ready_i) begin
						emit_done_o <= 1'b1;
						state_q <= ST_IDLE;
					end
				end

				default: state_q <= ST_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
