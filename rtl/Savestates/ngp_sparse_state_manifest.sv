// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_sparse_state_manifest -- codec for the fixed 512-byte sparse Type-3
// manifest used by manual NGPC savestates.
//
// Complete payload blocks follow this manifest in deterministic order: die 0
// blocks 0..34, then die 1 blocks 0..34. The two ledgers and immutable flash
// geometry therefore determine every payload offset and length; the indexed
// CRC32 table authenticates each complete block without storing an offset
// table. Entries outside the ledger must be zero.
//
// Candidate CRC metadata is written into one half of an M10K and copied into
// the committed half only after the whole manifest, identity, geometry, maps,
// derived byte count, and manifest CRC validate. A rejected candidate cannot
// corrupt the last accepted metadata. This module has no DDR3 or live-cart
// side effects; ngp_sparse_state_store owns payload CRC preflight and may act
// only after accepted_o.
`timescale 1ns/1ps
module ngp_sparse_state_manifest
(
	input  wire        clk,
	input  wire        reset,

	// Exact identity of the cartridge currently loaded into the core.
	input  wire [31:0] identity_raw_crc32_i,
	input  wire [31:0] identity_raw_bytes_i,
	input  wire [31:0] identity_pristine_crc32_i,
	input  wire [31:0] identity_physical_bytes_i,
	input  wire [1:0]  identity_die0_code_i,
	input  wire [1:0]  identity_die1_code_i,
	input  wire [15:0] identity_catalog_i,
	input  wire [7:0]  identity_subcatalog_i,
	input  wire [95:0] identity_title_i,

	// Sequential 256-word manifest parser.
	input  wire        manifest_start_i,
	input  wire        manifest_word_valid_i,
	input  wire [15:0] manifest_word_i,
	output wire        manifest_word_ready_o,
	output reg         accepted_o,
	output reg         rejected_o,
	output reg         manifest_valid_o,
	output reg  [34:0] ledger0_o,
	output reg  [34:0] ledger1_o,
	output reg  [31:0] payload_bytes_o,
	output reg  [31:0] type3_bytes_o,
	output reg  [6:0]  included_blocks_o,

	// Last accepted/build-configured per-block CRC metadata. The read is one
	// clock late, matching cache_ram_dp_be's synchronous B port.
	input  wire        crc_config_we_i,
	input  wire [6:0]  crc_config_index_i,
	input  wire [31:0] crc_config_data_i,
	input  wire [6:0]  crc_meta_index_i,
	output wire [31:0] crc_meta_data_o,

	// Sequential manifest builder. The builder derives payload/type3 sizes and
	// included count from the frozen maps and physical geometry; callers cannot
	// supply a contradictory size.
	input  wire        build_start_i,
	input  wire [34:0] build_ledger0_i,
	input  wire [34:0] build_ledger1_i,
	output reg         build_rejected_o,
	output reg  [31:0] build_payload_bytes_o,
	output reg  [31:0] build_type3_bytes_o,
	output reg  [6:0]  build_included_blocks_o,
	output reg         emit_valid_o,
	output reg  [7:0]  emit_word_index_o,
	output reg  [15:0] emit_word_o,
	input  wire        emit_ready_i,
	output reg         emit_done_o,
	output wire        busy_o
);

	localparam [7:0]  FORMAT_VERSION  = 8'd1;
	localparam [15:0] PROTECTED_BYTES = 16'd508;
	localparam [31:0] MANIFEST_BYTES  = 32'd512;
	localparam [7:0]  WORD_CRC_FIRST  = 8'd40;
	localparam [7:0]  WORD_CRC_LAST   = 8'd179;
	localparam [7:0]  WORD_ZERO_FIRST = 8'd180;
	localparam [7:0]  WORD_ZERO_LAST  = 8'd253;

	localparam [3:0] ST_IDLE             = 4'd0;
	localparam [3:0] ST_READ             = 4'd1;
	localparam [3:0] ST_READ_SCAN_REQ    = 4'd2;
	localparam [3:0] ST_READ_SCAN_CHECK  = 4'd3;
	localparam [3:0] ST_COPY_REQ         = 4'd4;
	localparam [3:0] ST_COPY_WRITE       = 4'd5;
	localparam [3:0] ST_BUILD_SCAN_REQ   = 4'd6;
	localparam [3:0] ST_BUILD_SCAN_CHECK = 4'd7;
	localparam [3:0] ST_EMIT_HEADER      = 4'd8;
	localparam [3:0] ST_EMIT_CRC_FETCH   = 4'd9;
	localparam [3:0] ST_EMIT_CRC         = 4'd10;
	localparam [3:0] ST_EMIT_ZERO        = 4'd11;
	localparam [3:0] ST_EMIT_SUM_LO      = 4'd12;
	localparam [3:0] ST_EMIT_SUM_HI      = 4'd13;

	reg [3:0]  state_q;

	reg [7:0]  read_word_index_q;
	reg [31:0] read_crc_q;
	reg [15:0] read_crc_lo_q;
	reg        read_bad_q;
	reg [31:0] read_payload_bytes_q;
	reg [31:0] read_type3_bytes_q;
	reg [63:0] read_ledger0_q;
	reg [63:0] read_ledger1_q;
	reg [15:0] read_included_blocks_q;
	reg [6:0]  read_crc_index_q;
	reg        read_crc_word_q;
	reg [15:0] read_crc_low_q;

	reg [34:0] build_ledger0_q;
	reg [34:0] build_ledger1_q;
	reg        build_bad_q;

	reg [6:0]  scan_index_q;
	reg [31:0] scan_payload_q;
	reg [6:0]  scan_count_q;
	reg [6:0]  copy_index_q;

	reg [7:0]  emit_header_index_q;
	reg [6:0]  emit_crc_index_q;
	reg        emit_crc_word_q;
	reg [7:0]  emit_stream_index_q;
	reg [31:0] emit_sum_q;

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

	function automatic [15:0] header_word;
		input [7:0]  index_i;
		input [31:0] payload_bytes_i;
		input [31:0] type3_bytes_i;
		input [31:0] raw_crc32_i;
		input [31:0] raw_bytes_i;
		input [31:0] pristine_crc32_i;
		input [31:0] physical_bytes_i;
		input [1:0]  die0_code_i;
		input [1:0]  die1_code_i;
		input [15:0] catalog_i;
		input [7:0]  subcatalog_i;
		input [95:0] title_i;
		input [34:0] ledger0_i;
		input [34:0] ledger1_i;
		input [6:0]  included_blocks_i;
		begin
			header_word = 16'd0;
			case (index_i)
				8'd0:  header_word = 16'h474E; // "NG"
				8'd1:  header_word = 16'h5350; // "PS"
				8'd2:  header_word = 16'h5653; // "SV"
				8'd3:  header_word = 16'h0031; // "1\0"
				8'd4:  header_word = {8'd0, FORMAT_VERSION};
				8'd5:  header_word = PROTECTED_BYTES;
				8'd6:  header_word = payload_bytes_i[15:0];
				8'd7:  header_word = payload_bytes_i[31:16];
				8'd8:  header_word = type3_bytes_i[15:0];
				8'd9:  header_word = type3_bytes_i[31:16];
				8'd10: header_word = raw_crc32_i[15:0];
				8'd11: header_word = raw_crc32_i[31:16];
				8'd12: header_word = raw_bytes_i[15:0];
				8'd13: header_word = raw_bytes_i[31:16];
				8'd14: header_word = pristine_crc32_i[15:0];
				8'd15: header_word = pristine_crc32_i[31:16];
				8'd16: header_word = physical_bytes_i[15:0];
				8'd17: header_word = physical_bytes_i[31:16];
				8'd18: header_word = {6'd0, die1_code_i, 6'd0, die0_code_i};
				8'd19: header_word = catalog_i;
				8'd20: header_word = {8'd0, subcatalog_i};
				8'd21: header_word = title_i[15:0];
				8'd22: header_word = title_i[31:16];
				8'd23: header_word = title_i[47:32];
				8'd24: header_word = title_i[63:48];
				8'd25: header_word = title_i[79:64];
				8'd26: header_word = title_i[95:80];
				8'd28: header_word = ledger0_i[15:0];
				8'd29: header_word = ledger0_i[31:16];
				8'd30: header_word = {13'd0, ledger0_i[34:32]};
				8'd32: header_word = ledger1_i[15:0];
				8'd33: header_word = ledger1_i[31:16];
				8'd34: header_word = {13'd0, ledger1_i[34:32]};
				8'd36: header_word = {9'd0, included_blocks_i};
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

	wire [34:0] valid_mask0_w = die_block_mask(identity_die0_code_i);
	wire [34:0] valid_mask1_w = die_block_mask(identity_die1_code_i);
	wire read_maps_valid_w = (read_ledger0_q[63:35] == 29'd0) &&
	                         (read_ledger1_q[63:35] == 29'd0) &&
	                         ((read_ledger0_q[34:0] & ~valid_mask0_w) == 35'd0) &&
	                         ((read_ledger1_q[34:0] & ~valid_mask1_w) == 35'd0);
	wire build_maps_valid_w = ((build_ledger0_i & ~valid_mask0_w) == 35'd0) &&
	                          ((build_ledger1_i & ~valid_mask1_w) == 35'd0);

	assign manifest_word_ready_o = (state_q == ST_READ);
	assign busy_o = (state_q != ST_IDLE);
	wire manifest_word_fire_w = manifest_word_valid_i && manifest_word_ready_o;

	// One 256x32 M10K contains candidate entries at 0..69 and committed entries
	// at 128..197. This prevents a rejected parse from damaging prior metadata.
	wire candidate_crc_write_w = manifest_word_fire_w &&
	                             (read_word_index_q >= WORD_CRC_FIRST) &&
	                             (read_word_index_q <= WORD_CRC_LAST) &&
	                             read_crc_word_q;
	wire [31:0] crc_ram_q_b_w;
	wire copy_crc_write_w = (state_q == ST_COPY_WRITE);
	wire config_crc_write_w = (state_q == ST_IDLE) && crc_config_we_i;
	wire crc_ram_we_a_w = copy_crc_write_w || candidate_crc_write_w || config_crc_write_w;
	wire [7:0] crc_ram_addr_a_w = copy_crc_write_w ? {1'b1, copy_index_q} :
	                              candidate_crc_write_w ? {1'b0, read_crc_index_q} :
	                              {1'b1, crc_config_index_i};
	wire [31:0] crc_ram_data_a_w = copy_crc_write_w ? crc_ram_q_b_w :
	                               candidate_crc_write_w ? {manifest_word_i, read_crc_low_q} :
	                               crc_config_data_i;

	wire scan_is_read_w = (state_q == ST_READ_SCAN_REQ) ||
	                      (state_q == ST_READ_SCAN_CHECK);
	wire scan_is_build_w = (state_q == ST_BUILD_SCAN_REQ) ||
	                       (state_q == ST_BUILD_SCAN_CHECK);
	wire copy_read_w = (state_q == ST_COPY_REQ) || (state_q == ST_COPY_WRITE);
	wire emit_crc_read_w = (state_q == ST_EMIT_CRC_FETCH) ||
	                       (state_q == ST_EMIT_CRC);
	wire [7:0] crc_ram_addr_b_w = scan_is_read_w ? {1'b0, scan_index_q} :
	                              scan_is_build_w ? {1'b1, scan_index_q} :
	                              copy_read_w ? {1'b0, copy_index_q} :
	                              emit_crc_read_w ? {1'b1, emit_crc_index_q} :
	                              {1'b1, crc_meta_index_i};
	/* verilator lint_off UNUSED */
	wire [31:0] crc_ram_q_a_unused_w;
	/* verilator lint_on UNUSED */

	cache_ram_dp_be #(.ADDR_WIDTH(8), .DATA_WIDTH(32)) u_crc_metadata
	(
		.clk_i    (clk),
		.addr_a_i (crc_ram_addr_a_w),
		.wren_a_i (crc_ram_we_a_w),
		.be_a_i   (4'b1111),
		.wdata_a_i(crc_ram_data_a_w),
		.q_a_o    (crc_ram_q_a_unused_w),
		.addr_b_i (crc_ram_addr_b_w),
		.wren_b_i (1'b0),
		.be_b_i   (4'd0),
		.wdata_b_i(32'd0),
		.q_b_o    (crc_ram_q_b_w)
	);

	assign crc_meta_data_o = crc_ram_q_b_w;

	// The scanner shares the canonical flash-geometry module used by S0.
	wire scan_die_w = (scan_index_q >= 7'd35);
	// For indices 64..69 the six-bit subtraction wraps by 64, yielding the
	// intended die-1 block indices 29..34 without an implicit truncation.
	wire [5:0] scan_block_w = scan_die_w ?
	                               (scan_index_q[5:0] - 6'd35) : scan_index_q[5:0];
	wire [1:0] scan_size_code_w = scan_die_w ? identity_die1_code_i : identity_die0_code_i;
	wire [34:0] scan_ledger0_w = scan_is_read_w ? read_ledger0_q[34:0] : build_ledger0_q;
	wire [34:0] scan_ledger1_w = scan_is_read_w ? read_ledger1_q[34:0] : build_ledger1_q;
	wire scan_map_bit_w = scan_die_w ? scan_ledger1_w[scan_block_w] :
	                                    scan_ledger0_w[scan_block_w];
	wire scan_geometry_valid_w;
	wire [16:0] scan_block_bytes_w;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [20:0] scan_block_base_unused_w;
	wire [15:0] scan_block_words_unused_w;
	/* verilator lint_on UNUSEDSIGNAL */

	ngp_cart_overlay_geometry u_geometry
	(
		.size_code_i(scan_size_code_w),
		.block_i    (scan_block_w),
		.valid_o    (scan_geometry_valid_w),
		.base_o     (scan_block_base_unused_w),
		.bytes_o    (scan_block_bytes_w),
		.words_o    (scan_block_words_unused_w)
	);

	wire scan_selected_w = scan_geometry_valid_w && scan_map_bit_w;
	wire scan_crc_bad_w = (!scan_selected_w) && (crc_ram_q_b_w != 32'd0);
	wire [31:0] scan_selected_bytes_w = scan_selected_w ?
	                                           {15'd0, scan_block_bytes_w} : 32'd0;
	wire [31:0] scan_payload_next_w = scan_payload_q + scan_selected_bytes_w;
	wire [6:0] scan_count_next_w = scan_count_q + {6'd0, scan_selected_w};

	always @* begin
		emit_valid_o = 1'b0;
		emit_word_index_o = 8'd0;
		emit_word_o = 16'd0;
		case (state_q)
			ST_EMIT_HEADER: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = emit_header_index_q;
				emit_word_o = header_word(emit_header_index_q,
				                          build_payload_bytes_o,
				                          build_type3_bytes_o,
				                          identity_raw_crc32_i,
				                          identity_raw_bytes_i,
				                          identity_pristine_crc32_i,
				                          identity_physical_bytes_i,
				                          identity_die0_code_i,
				                          identity_die1_code_i,
				                          identity_catalog_i,
				                          identity_subcatalog_i,
				                          identity_title_i,
				                          build_ledger0_q,
				                          build_ledger1_q,
				                          build_included_blocks_o);
			end
			ST_EMIT_CRC: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = emit_stream_index_q;
				emit_word_o = emit_crc_word_q ? crc_ram_q_b_w[31:16] :
				                                    crc_ram_q_b_w[15:0];
			end
			ST_EMIT_ZERO: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = emit_stream_index_q;
				emit_word_o = 16'd0;
			end
			ST_EMIT_SUM_LO: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = 8'd254;
				emit_word_o = ~emit_sum_q[15:0];
			end
			ST_EMIT_SUM_HI: begin
				emit_valid_o = 1'b1;
				emit_word_index_o = 8'd255;
				emit_word_o = ~emit_sum_q[31:16];
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
			state_q                    <= ST_IDLE;
			read_word_index_q          <= 8'd0;
			read_crc_q                 <= 32'hFFFFFFFF;
			read_crc_lo_q              <= 16'd0;
			read_bad_q                 <= 1'b0;
			read_payload_bytes_q       <= 32'd0;
			read_type3_bytes_q         <= 32'd0;
			read_ledger0_q             <= 64'd0;
			read_ledger1_q             <= 64'd0;
			read_included_blocks_q     <= 16'd0;
			read_crc_index_q           <= 7'd0;
			read_crc_word_q            <= 1'b0;
			read_crc_low_q             <= 16'd0;
			build_ledger0_q            <= 35'd0;
			build_ledger1_q            <= 35'd0;
			build_bad_q                <= 1'b0;
			scan_index_q               <= 7'd0;
			scan_payload_q             <= 32'd0;
			scan_count_q               <= 7'd0;
			copy_index_q               <= 7'd0;
			emit_header_index_q        <= 8'd0;
			emit_crc_index_q           <= 7'd0;
			emit_crc_word_q            <= 1'b0;
			emit_stream_index_q        <= 8'd0;
			emit_sum_q                 <= 32'hFFFFFFFF;
			manifest_valid_o           <= 1'b0;
			ledger0_o                  <= 35'd0;
			ledger1_o                  <= 35'd0;
			payload_bytes_o            <= 32'd0;
			type3_bytes_o              <= 32'd0;
			included_blocks_o          <= 7'd0;
			build_payload_bytes_o      <= 32'd0;
			build_type3_bytes_o        <= 32'd0;
			build_included_blocks_o    <= 7'd0;
		end else begin
			case (state_q)
				ST_IDLE: begin
					if (manifest_start_i) begin
						state_q                <= ST_READ;
						read_word_index_q      <= 8'd0;
						read_crc_q             <= 32'hFFFFFFFF;
						read_crc_lo_q          <= 16'd0;
						read_bad_q             <= 1'b0;
						read_payload_bytes_q   <= 32'd0;
						read_type3_bytes_q     <= 32'd0;
						read_ledger0_q         <= 64'd0;
						read_ledger1_q         <= 64'd0;
						read_included_blocks_q <= 16'd0;
						read_crc_index_q       <= 7'd0;
						read_crc_word_q        <= 1'b0;
					end else if (build_start_i) begin
						build_ledger0_q <= build_ledger0_i;
						build_ledger1_q <= build_ledger1_i;
						build_bad_q     <= !identity_valid_w || !build_maps_valid_w;
						scan_index_q    <= 7'd0;
						scan_payload_q  <= 32'd0;
						scan_count_q    <= 7'd0;
						state_q         <= ST_BUILD_SCAN_REQ;
					end
				end

				ST_READ: begin
					if (manifest_word_fire_w) begin
						if (read_word_index_q < 8'd254)
							read_crc_q <= crc32_word(read_crc_q, manifest_word_i);

						case (read_word_index_q)
							8'd0: if (manifest_word_i != 16'h474E) read_bad_q <= 1'b1;
							8'd1: if (manifest_word_i != 16'h5350) read_bad_q <= 1'b1;
							8'd2: if (manifest_word_i != 16'h5653) read_bad_q <= 1'b1;
							8'd3: if (manifest_word_i != 16'h0031) read_bad_q <= 1'b1;
							8'd4: if (manifest_word_i != {8'd0, FORMAT_VERSION}) read_bad_q <= 1'b1;
							8'd5: if (manifest_word_i != PROTECTED_BYTES) read_bad_q <= 1'b1;
							8'd6: read_payload_bytes_q[15:0] <= manifest_word_i;
							8'd7: read_payload_bytes_q[31:16] <= manifest_word_i;
							8'd8: read_type3_bytes_q[15:0] <= manifest_word_i;
							8'd9: read_type3_bytes_q[31:16] <= manifest_word_i;
							8'd10: if (manifest_word_i != identity_raw_crc32_i[15:0]) read_bad_q <= 1'b1;
							8'd11: if (manifest_word_i != identity_raw_crc32_i[31:16]) read_bad_q <= 1'b1;
							8'd12: if (manifest_word_i != identity_raw_bytes_i[15:0]) read_bad_q <= 1'b1;
							8'd13: if (manifest_word_i != identity_raw_bytes_i[31:16]) read_bad_q <= 1'b1;
							8'd14: if (manifest_word_i != identity_pristine_crc32_i[15:0]) read_bad_q <= 1'b1;
							8'd15: if (manifest_word_i != identity_pristine_crc32_i[31:16]) read_bad_q <= 1'b1;
							8'd16: if (manifest_word_i != identity_physical_bytes_i[15:0]) read_bad_q <= 1'b1;
							8'd17: if (manifest_word_i != identity_physical_bytes_i[31:16]) read_bad_q <= 1'b1;
							8'd18: if (manifest_word_i != {6'd0, identity_die1_code_i, 6'd0, identity_die0_code_i}) read_bad_q <= 1'b1;
							8'd19: if (manifest_word_i != identity_catalog_i) read_bad_q <= 1'b1;
							8'd20: if (manifest_word_i != {8'd0, identity_subcatalog_i}) read_bad_q <= 1'b1;
							8'd21: if (manifest_word_i != identity_title_i[15:0]) read_bad_q <= 1'b1;
							8'd22: if (manifest_word_i != identity_title_i[31:16]) read_bad_q <= 1'b1;
							8'd23: if (manifest_word_i != identity_title_i[47:32]) read_bad_q <= 1'b1;
							8'd24: if (manifest_word_i != identity_title_i[63:48]) read_bad_q <= 1'b1;
							8'd25: if (manifest_word_i != identity_title_i[79:64]) read_bad_q <= 1'b1;
							8'd26: if (manifest_word_i != identity_title_i[95:80]) read_bad_q <= 1'b1;
							8'd27, 8'd31, 8'd35, 8'd37, 8'd38, 8'd39:
								if (manifest_word_i != 16'd0) read_bad_q <= 1'b1;
							8'd28: read_ledger0_q[15:0] <= manifest_word_i;
							8'd29: read_ledger0_q[31:16] <= manifest_word_i;
							8'd30: begin
								read_ledger0_q[47:32] <= manifest_word_i;
								if (manifest_word_i[15:3] != 13'd0) read_bad_q <= 1'b1;
							end
							8'd32: read_ledger1_q[15:0] <= manifest_word_i;
							8'd33: read_ledger1_q[31:16] <= manifest_word_i;
							8'd34: begin
								read_ledger1_q[47:32] <= manifest_word_i;
								if (manifest_word_i[15:3] != 13'd0) read_bad_q <= 1'b1;
							end
							8'd36: begin
								read_included_blocks_q <= manifest_word_i;
								if (manifest_word_i[15:7] != 9'd0) read_bad_q <= 1'b1;
							end
							8'd254: read_crc_lo_q <= manifest_word_i;
							default: ;
						endcase

						if ((read_word_index_q >= WORD_ZERO_FIRST) &&
						    (read_word_index_q <= WORD_ZERO_LAST) &&
						    (manifest_word_i != 16'd0)) read_bad_q <= 1'b1;

						if ((read_word_index_q >= WORD_CRC_FIRST) &&
						    (read_word_index_q <= WORD_CRC_LAST)) begin
							if (!read_crc_word_q) begin
								read_crc_low_q <= manifest_word_i;
								read_crc_word_q <= 1'b1;
							end else begin
								read_crc_word_q <= 1'b0;
								read_crc_index_q <= read_crc_index_q + 7'd1;
							end
						end

						if (read_word_index_q == 8'd255) begin
							if (!read_bad_q && identity_valid_w && read_maps_valid_w &&
							    ((~read_crc_q) == {manifest_word_i, read_crc_lo_q})) begin
								scan_index_q   <= 7'd0;
								scan_payload_q <= 32'd0;
								scan_count_q   <= 7'd0;
								state_q        <= ST_READ_SCAN_REQ;
							end else begin
								rejected_o <= 1'b1;
								state_q <= ST_IDLE;
							end
						end else begin
							read_word_index_q <= read_word_index_q + 8'd1;
						end
					end
				end

				ST_READ_SCAN_REQ: state_q <= ST_READ_SCAN_CHECK;

				ST_READ_SCAN_CHECK: begin
					if (scan_crc_bad_w) begin
						rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end else if (scan_index_q == 7'd69) begin
						if ((scan_payload_next_w != read_payload_bytes_q) ||
						    ((scan_payload_next_w + MANIFEST_BYTES) != read_type3_bytes_q) ||
						    ({9'd0, scan_count_next_w} != read_included_blocks_q)) begin
							rejected_o <= 1'b1;
							state_q <= ST_IDLE;
						end else begin
							copy_index_q <= 7'd0;
							state_q <= ST_COPY_REQ;
						end
					end else begin
						scan_payload_q <= scan_payload_next_w;
						scan_count_q <= scan_count_next_w;
						scan_index_q <= scan_index_q + 7'd1;
						state_q <= ST_READ_SCAN_REQ;
					end
				end

				ST_COPY_REQ: state_q <= ST_COPY_WRITE;

				ST_COPY_WRITE: begin
					if (copy_index_q == 7'd69) begin
						manifest_valid_o  <= 1'b1;
						ledger0_o         <= read_ledger0_q[34:0];
						ledger1_o         <= read_ledger1_q[34:0];
						payload_bytes_o   <= read_payload_bytes_q;
						type3_bytes_o     <= read_type3_bytes_q;
						included_blocks_o <= read_included_blocks_q[6:0];
						accepted_o         <= 1'b1;
						state_q            <= ST_IDLE;
					end else begin
						copy_index_q <= copy_index_q + 7'd1;
						state_q <= ST_COPY_REQ;
					end
				end

				ST_BUILD_SCAN_REQ: state_q <= ST_BUILD_SCAN_CHECK;

				ST_BUILD_SCAN_CHECK: begin
					if (build_bad_q || scan_crc_bad_w) begin
						build_rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end else if (scan_index_q == 7'd69) begin
						build_payload_bytes_o   <= scan_payload_next_w;
						build_type3_bytes_o     <= scan_payload_next_w + MANIFEST_BYTES;
						build_included_blocks_o <= scan_count_next_w;
						emit_header_index_q     <= 8'd0;
						emit_sum_q              <= 32'hFFFFFFFF;
						state_q                 <= ST_EMIT_HEADER;
					end else begin
						scan_payload_q <= scan_payload_next_w;
						scan_count_q <= scan_count_next_w;
						scan_index_q <= scan_index_q + 7'd1;
						state_q <= ST_BUILD_SCAN_REQ;
					end
				end

				ST_EMIT_HEADER: begin
					if (emit_ready_i) begin
						emit_sum_q <= crc32_word(emit_sum_q, emit_word_o);
						if (emit_header_index_q == 8'd39) begin
							emit_crc_index_q <= 7'd0;
							emit_crc_word_q <= 1'b0;
							emit_stream_index_q <= WORD_CRC_FIRST;
							state_q <= ST_EMIT_CRC_FETCH;
						end else begin
							emit_header_index_q <= emit_header_index_q + 8'd1;
						end
					end
				end

				ST_EMIT_CRC_FETCH: state_q <= ST_EMIT_CRC;

				ST_EMIT_CRC: begin
					if (emit_ready_i) begin
						emit_sum_q <= crc32_word(emit_sum_q, emit_word_o);
						emit_stream_index_q <= emit_stream_index_q + 8'd1;
						if (emit_crc_word_q) begin
							if (emit_crc_index_q == 7'd69) begin
								state_q <= ST_EMIT_ZERO;
							end else begin
								emit_crc_index_q <= emit_crc_index_q + 7'd1;
								emit_crc_word_q <= 1'b0;
								state_q <= ST_EMIT_CRC_FETCH;
							end
						end else begin
							emit_crc_word_q <= 1'b1;
						end
					end
				end

				ST_EMIT_ZERO: begin
					if (emit_ready_i) begin
						emit_sum_q <= crc32_word(emit_sum_q, 16'd0);
						if (emit_stream_index_q == WORD_ZERO_LAST)
							state_q <= ST_EMIT_SUM_LO;
						else
							emit_stream_index_q <= emit_stream_index_q + 8'd1;
					end
				end

				ST_EMIT_SUM_LO: if (emit_ready_i) state_q <= ST_EMIT_SUM_HI;

				ST_EMIT_SUM_HI: begin
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
