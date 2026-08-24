// Copyright (c) 2026 Jamie Blanks

// NGPC physical-read cheat engine using the standard MiSTer record:
// 128       record-valid strobe
// 105:104   replacement method (replace, OR, AND)
// 102:100   width (byte, 16-bit, 32-bit)
// 96        compare enable
// 95:64     physical byte address
// 63:32     compare value
// 31:0      replacement value
//
// User-visible behavior follows the standard MiSTer cheat semantics: records
// expand little-endian into 32-byte slots, newer matches win, a failed newer
// compare falls back to an older match, and replace/OR/AND apply to the raw CPU
// read after the complete memory mux.
//
// The storage is adapted to this core's hardware rules and clock ratio. Four
// mirrored 64x43 M10K tables provide eight read ports over one active 32-entry
// bank while a complete replacement generation is written into the other.
// The bank and active count change together only after the loader has ended
// the transfer, every pending byte has been written, and the machine is
// paused. This prevents a multi-record instruction patch becoming visible one
// record at a time.
//
// The fabric raises bus_rdy when the raw response is valid, one geared CPU
// state before T2. Four groups of eight entries plus the synchronous read
// latency finish in five clk_sys clocks, inside that eight-clock interval,
// without adding a CPU wait state or a CAM on the read-return timing path.
//
// NGPC also has dynamic 8/16-bit regions. An odd address on an 8-bit region
// returns on data[7:0], while an odd byte on a 16-bit region uses data[15:8].
// be_i therefore carries both lane validity and the physical-address map.
module ngp_cheat_engine
(
	input  wire         clk_sys_i,
	input  wire         load_begin_i,
	input  wire         invalidate_i,
	input  wire         commit_req_i,
	input  wire         paused_i,
	input  wire         enable_i,
	input  wire         response_ready_i,
	input  wire [128:0] code_i,
	input  wire [23:0]  addr_i,
	input  wire [1:0]   be_i,
	input  wire [15:0]  data_i,
	output wire [15:0]  data_o,
	output wire         available_o,
	output reg          commit_done_o
);

	localparam [5:0] MAX_CODES_COUNT = 6'd32;
	localparam [1:0] METHOD_REPLACE = 2'd0;
	localparam [1:0] METHOD_OR      = 2'd1;
	localparam [1:0] METHOD_AND     = 2'd2;

	wire        code_valid_w          = code_i[128];
	wire [1:0]  code_method_w         = code_i[105:104];
	wire [2:0]  code_width_w          = code_i[102:100];
	wire        code_compare_enable_w = code_i[96];
	wire [7:0]  code_addr_high_w      = code_i[95:88];
	wire [23:0] code_addr_w           = code_i[87:64];
	wire [31:0] code_compare_w        = code_i[63:32];
	wire [31:0] code_value_w          = code_i[31:0];

	// A word cycle is aligned and names both byte lanes. Single-byte cycles name
	// the physical byte directly, regardless of which data lane carries it.
	wire [23:0] read_addr_low_w = (be_i == 2'b11) ?
		{addr_i[23:1], 1'b0} : addr_i;
	wire [23:0] read_addr_high_w = (be_i == 2'b11) ?
		{addr_i[23:1], 1'b1} : addr_i;

	reg        active_bank_q;
	reg        key_bank_q;
	reg        load_active_q;
	reg [5:0]  active_count_q;
	reg [5:0]  staging_count_q;
	reg [2:0]  pending_count_q;
	reg [23:0] pending_addr_q;
	reg [31:0] pending_value_q;
	reg [31:0] pending_compare_q;
	reg        pending_compare_enable_q;
	reg [1:0]  pending_method_q;
	reg [2:0]  record_byte_count;
	reg        record_aligned;

	// Table entry: address, compare, value, compare-enable, method.
	wire        table_write_w = load_active_q && (pending_count_q != 3'd0);
	wire [5:0]  table_write_addr_w = {
		~active_bank_q, staging_count_q[4:0]
	};
	wire [42:0] table_write_data_w = {
		pending_addr_q,
		pending_compare_q[7:0],
		pending_value_q[7:0],
		pending_compare_enable_q,
		pending_method_q
	};

	// A request key change starts an early scan. The rising edge of bus_rdy
	// restarts it from the point where raw read data is guaranteed valid; bus_rdy
	// then remains high until the CPU consumes T2, so a level must not restart
	// the scan repeatedly.
	reg [23:0] key_addr_q;
	reg [1:0]  key_be_q;
	reg [5:0]  key_count_q;
	reg        response_ready_q;
	reg        scan_active_q;
	reg        scan_data_valid_q;
	reg [1:0]  scan_group_q;

	wire key_changed_w = enable_i &&
		((addr_i != key_addr_q) || (be_i != key_be_q) ||
		 (active_count_q != key_count_q) ||
		 (active_bank_q != key_bank_q));
	wire response_ready_w = response_ready_i && !response_ready_q;
	wire scan_restart_w = enable_i && (key_changed_w || response_ready_w);
	wire [1:0] next_group_w = scan_group_q + 2'd1;
	wire [4:0] scan_base_w = scan_restart_w ? 5'd0 :
		{next_group_w, 3'b000};

	wire [5:0] table0_addr_a_w = table_write_w ? table_write_addr_w :
		{active_bank_q, scan_base_w};
	wire [5:0] table0_addr_b_w = {
		active_bank_q, scan_base_w + 5'd1
	};
	wire [5:0] table1_addr_a_w = table_write_w ? table_write_addr_w :
		{active_bank_q, scan_base_w + 5'd2};
	wire [5:0] table1_addr_b_w = {
		active_bank_q, scan_base_w + 5'd3
	};
	wire [5:0] table2_addr_a_w = table_write_w ? table_write_addr_w :
		{active_bank_q, scan_base_w + 5'd4};
	wire [5:0] table2_addr_b_w = {
		active_bank_q, scan_base_w + 5'd5
	};
	wire [5:0] table3_addr_a_w = table_write_w ? table_write_addr_w :
		{active_bank_q, scan_base_w + 5'd6};
	wire [5:0] table3_addr_b_w = {
		active_bank_q, scan_base_w + 5'd7
	};

	wire [42:0] entry0_w;
	wire [42:0] entry1_w;
	wire [42:0] entry2_w;
	wire [42:0] entry3_w;
	wire [42:0] entry4_w;
	wire [42:0] entry5_w;
	wire [42:0] entry6_w;
	wire [42:0] entry7_w;

	cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(43)) table0
	(
		.clk_i     (clk_sys_i),
		.addr_a_i  (table0_addr_a_w),
		.wren_a_i  (table_write_w),
		.wdata_a_i (table_write_data_w),
		.q_a_o     (entry0_w),
		.addr_b_i  (table0_addr_b_w),
		.wren_b_i  (1'b0),
		.wdata_b_i (43'd0),
		.q_b_o     (entry1_w)
	);

	cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(43)) table1
	(
		.clk_i     (clk_sys_i),
		.addr_a_i  (table1_addr_a_w),
		.wren_a_i  (table_write_w),
		.wdata_a_i (table_write_data_w),
		.q_a_o     (entry2_w),
		.addr_b_i  (table1_addr_b_w),
		.wren_b_i  (1'b0),
		.wdata_b_i (43'd0),
		.q_b_o     (entry3_w)
	);

	cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(43)) table2
	(
		.clk_i     (clk_sys_i),
		.addr_a_i  (table2_addr_a_w),
		.wren_a_i  (table_write_w),
		.wdata_a_i (table_write_data_w),
		.q_a_o     (entry4_w),
		.addr_b_i  (table2_addr_b_w),
		.wren_b_i  (1'b0),
		.wdata_b_i (43'd0),
		.q_b_o     (entry5_w)
	);

	cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(43)) table3
	(
		.clk_i     (clk_sys_i),
		.addr_a_i  (table3_addr_a_w),
		.wren_a_i  (table_write_w),
		.wdata_a_i (table_write_data_w),
		.q_a_o     (entry6_w),
		.addr_b_i  (table3_addr_b_w),
		.wren_b_i  (1'b0),
		.wdata_b_i (43'd0),
		.q_b_o     (entry7_w)
	);

	function entry_match_fn;
		input [23:0] entry_addr_i;
		input [7:0] entry_compare_i;
		input entry_compare_enable_i;
		input [4:0] entry_index_i;
		input [5:0] active_count_i;
		input [23:0] read_addr_i;
		input [7:0] read_data_i;
		begin
			entry_match_fn = ({1'b0, entry_index_i} < active_count_i) &&
				(entry_addr_i == read_addr_i) &&
				(!entry_compare_enable_i || (entry_compare_i == read_data_i));
		end
	endfunction

	wire [4:0] entry_index0_w = {scan_group_q, 3'b000};
	wire [4:0] entry_index1_w = entry_index0_w + 5'd1;
	wire [4:0] entry_index2_w = entry_index0_w + 5'd2;
	wire [4:0] entry_index3_w = entry_index0_w + 5'd3;
	wire [4:0] entry_index4_w = entry_index0_w + 5'd4;
	wire [4:0] entry_index5_w = entry_index0_w + 5'd5;
	wire [4:0] entry_index6_w = entry_index0_w + 5'd6;
	wire [4:0] entry_index7_w = entry_index0_w + 5'd7;

	wire low_match0_w = be_i[0] && entry_match_fn(entry0_w[42:19], entry0_w[18:11],
		entry0_w[2], entry_index0_w, active_count_q, read_addr_low_w, data_i[7:0]);
	wire low_match1_w = be_i[0] && entry_match_fn(entry1_w[42:19], entry1_w[18:11],
		entry1_w[2], entry_index1_w, active_count_q, read_addr_low_w, data_i[7:0]);
	wire low_match2_w = be_i[0] && entry_match_fn(entry2_w[42:19], entry2_w[18:11],
		entry2_w[2], entry_index2_w, active_count_q, read_addr_low_w, data_i[7:0]);
	wire low_match3_w = be_i[0] && entry_match_fn(entry3_w[42:19], entry3_w[18:11],
		entry3_w[2], entry_index3_w, active_count_q, read_addr_low_w, data_i[7:0]);
	wire low_match4_w = be_i[0] && entry_match_fn(entry4_w[42:19], entry4_w[18:11],
		entry4_w[2], entry_index4_w, active_count_q, read_addr_low_w, data_i[7:0]);
	wire low_match5_w = be_i[0] && entry_match_fn(entry5_w[42:19], entry5_w[18:11],
		entry5_w[2], entry_index5_w, active_count_q, read_addr_low_w, data_i[7:0]);
	wire low_match6_w = be_i[0] && entry_match_fn(entry6_w[42:19], entry6_w[18:11],
		entry6_w[2], entry_index6_w, active_count_q, read_addr_low_w, data_i[7:0]);
	wire low_match7_w = be_i[0] && entry_match_fn(entry7_w[42:19], entry7_w[18:11],
		entry7_w[2], entry_index7_w, active_count_q, read_addr_low_w, data_i[7:0]);
	wire high_match0_w = be_i[1] && entry_match_fn(entry0_w[42:19], entry0_w[18:11],
		entry0_w[2], entry_index0_w, active_count_q, read_addr_high_w, data_i[15:8]);
	wire high_match1_w = be_i[1] && entry_match_fn(entry1_w[42:19], entry1_w[18:11],
		entry1_w[2], entry_index1_w, active_count_q, read_addr_high_w, data_i[15:8]);
	wire high_match2_w = be_i[1] && entry_match_fn(entry2_w[42:19], entry2_w[18:11],
		entry2_w[2], entry_index2_w, active_count_q, read_addr_high_w, data_i[15:8]);
	wire high_match3_w = be_i[1] && entry_match_fn(entry3_w[42:19], entry3_w[18:11],
		entry3_w[2], entry_index3_w, active_count_q, read_addr_high_w, data_i[15:8]);
	wire high_match4_w = be_i[1] && entry_match_fn(entry4_w[42:19], entry4_w[18:11],
		entry4_w[2], entry_index4_w, active_count_q, read_addr_high_w, data_i[15:8]);
	wire high_match5_w = be_i[1] && entry_match_fn(entry5_w[42:19], entry5_w[18:11],
		entry5_w[2], entry_index5_w, active_count_q, read_addr_high_w, data_i[15:8]);
	wire high_match6_w = be_i[1] && entry_match_fn(entry6_w[42:19], entry6_w[18:11],
		entry6_w[2], entry_index6_w, active_count_q, read_addr_high_w, data_i[15:8]);
	wire high_match7_w = be_i[1] && entry_match_fn(entry7_w[42:19], entry7_w[18:11],
		entry7_w[2], entry_index7_w, active_count_q, read_addr_high_w, data_i[15:8]);

	reg       low_hit_q;
	reg [1:0] low_method_q;
	reg [7:0] low_value_q;
	reg       high_hit_q;
	reg [1:0] high_method_q;
	reg [7:0] high_value_q;
	reg       result_low_hit_q;
	reg [1:0] result_low_method_q;
	reg [7:0] result_low_value_q;
	reg       result_high_hit_q;
	reg [1:0] result_high_method_q;
	reg [7:0] result_high_value_q;
	reg       group_low_hit;
	reg [1:0] group_low_method;
	reg [7:0] group_low_value;
	reg       group_high_hit;
	reg [1:0] group_high_method;
	reg [7:0] group_high_value;

	// Quartus maps these explicit initial values into FPGA power-up state. The
	// ordinary console reset is deliberately not an engine input.
	initial begin
		active_bank_q = 1'b0;
		key_bank_q = 1'b0;
		load_active_q = 1'b0;
		active_count_q = 6'd0;
		staging_count_q = 6'd0;
		pending_count_q = 3'd0;
		pending_addr_q = 24'd0;
		pending_value_q = 32'd0;
		pending_compare_q = 32'd0;
		pending_compare_enable_q = 1'b0;
		pending_method_q = METHOD_REPLACE;
		key_addr_q = 24'd0;
		key_be_q = 2'd0;
		key_count_q = 6'd0;
		response_ready_q = 1'b0;
		scan_active_q = 1'b0;
		scan_data_valid_q = 1'b0;
		scan_group_q = 2'd0;
		low_hit_q = 1'b0;
		low_method_q = METHOD_REPLACE;
		low_value_q = 8'd0;
		high_hit_q = 1'b0;
		high_method_q = METHOD_REPLACE;
		high_value_q = 8'd0;
		result_low_hit_q = 1'b0;
		result_low_method_q = METHOD_REPLACE;
		result_low_value_q = 8'd0;
		result_high_hit_q = 1'b0;
		result_high_method_q = METHOD_REPLACE;
		result_high_value_q = 8'd0;
		commit_done_o = 1'b0;
	end

	// Standard-record reserved flag bits are intentionally ignored.
	/* verilator lint_off UNUSED */
	wire unused_ok = &{1'b0, code_i[127:106], code_i[103],
		code_i[99:97], 1'b0};
	/* verilator lint_on UNUSED */

	// Select the newest match inside the current eight-entry group. Groups are
	// scanned in ascending order, so a later group overwrites an earlier one.
	always @* begin
		group_low_hit = 1'b0;
		group_low_method = METHOD_REPLACE;
		group_low_value = 8'd0;
		if (low_match0_w) begin
			group_low_hit = 1'b1;
			group_low_method = entry0_w[1:0];
			group_low_value = entry0_w[10:3];
		end
		if (low_match1_w) begin
			group_low_hit = 1'b1;
			group_low_method = entry1_w[1:0];
			group_low_value = entry1_w[10:3];
		end
		if (low_match2_w) begin
			group_low_hit = 1'b1;
			group_low_method = entry2_w[1:0];
			group_low_value = entry2_w[10:3];
		end
		if (low_match3_w) begin
			group_low_hit = 1'b1;
			group_low_method = entry3_w[1:0];
			group_low_value = entry3_w[10:3];
		end
		if (low_match4_w) begin
			group_low_hit = 1'b1;
			group_low_method = entry4_w[1:0];
			group_low_value = entry4_w[10:3];
		end
		if (low_match5_w) begin
			group_low_hit = 1'b1;
			group_low_method = entry5_w[1:0];
			group_low_value = entry5_w[10:3];
		end
		if (low_match6_w) begin
			group_low_hit = 1'b1;
			group_low_method = entry6_w[1:0];
			group_low_value = entry6_w[10:3];
		end
		if (low_match7_w) begin
			group_low_hit = 1'b1;
			group_low_method = entry7_w[1:0];
			group_low_value = entry7_w[10:3];
		end

		group_high_hit = 1'b0;
		group_high_method = METHOD_REPLACE;
		group_high_value = 8'd0;
		if (high_match0_w) begin
			group_high_hit = 1'b1;
			group_high_method = entry0_w[1:0];
			group_high_value = entry0_w[10:3];
		end
		if (high_match1_w) begin
			group_high_hit = 1'b1;
			group_high_method = entry1_w[1:0];
			group_high_value = entry1_w[10:3];
		end
		if (high_match2_w) begin
			group_high_hit = 1'b1;
			group_high_method = entry2_w[1:0];
			group_high_value = entry2_w[10:3];
		end
		if (high_match3_w) begin
			group_high_hit = 1'b1;
			group_high_method = entry3_w[1:0];
			group_high_value = entry3_w[10:3];
		end
		if (high_match4_w) begin
			group_high_hit = 1'b1;
			group_high_method = entry4_w[1:0];
			group_high_value = entry4_w[10:3];
		end
		if (high_match5_w) begin
			group_high_hit = 1'b1;
			group_high_method = entry5_w[1:0];
			group_high_value = entry5_w[10:3];
		end
		if (high_match6_w) begin
			group_high_hit = 1'b1;
			group_high_method = entry6_w[1:0];
			group_high_value = entry6_w[10:3];
		end
		if (high_match7_w) begin
			group_high_hit = 1'b1;
			group_high_method = entry7_w[1:0];
			group_high_value = entry7_w[10:3];
		end
	end

	function [7:0] apply_method_fn;
		input [1:0] method_i;
		input [7:0] read_byte_i;
		input [7:0] value_i;
		begin
			case (method_i)
				METHOD_OR:  apply_method_fn = read_byte_i | value_i;
				METHOD_AND: apply_method_fn = read_byte_i & value_i;
				default:    apply_method_fn = value_i;
			endcase
		end
	endfunction

	assign available_o = (active_count_q != 6'd0);
	assign data_o[7:0] = (enable_i && be_i[0] && result_low_hit_q) ?
		apply_method_fn(result_low_method_q, data_i[7:0],
			result_low_value_q) : data_i[7:0];
	assign data_o[15:8] = (enable_i && be_i[1] && result_high_hit_q) ?
		apply_method_fn(result_high_method_q, data_i[15:8],
			result_high_value_q) : data_i[15:8];

	// Only byte/16/32-bit widths at natural alignment are accepted, and only
	// while all expanded bytes still fit in the table.
	always @* begin
		record_byte_count = 3'd0;
		record_aligned = 1'b0;
		case (code_width_w)
			3'b000, 3'b001: begin
				record_byte_count = 3'd1;
				record_aligned = 1'b1;
			end
			3'b010: begin
				record_byte_count = 3'd2;
				record_aligned = !code_addr_w[0];
			end
			3'b100: begin
				record_byte_count = 3'd4;
				record_aligned = !(|code_addr_w[1:0]);
			end
			default: begin
				record_byte_count = 3'd0;
				record_aligned = 1'b0;
			end
		endcase
	end

	// commit_req_i is a level held by the loader. Acknowledgement waits for the
	// machine pause and for the final wide record to finish expanding, so the
	// active bank and count are one atomic generation change.
	wire commit_w = commit_req_i && paused_i && load_active_q &&
		!code_valid_w && (pending_count_q == 3'd0);

	always @(posedge clk_sys_i) begin
		response_ready_q <= response_ready_i;
		commit_done_o <= 1'b0;

		if (invalidate_i) begin
			active_count_q <= 6'd0;
			staging_count_q <= 6'd0;
			pending_count_q <= 3'd0;
			load_active_q <= 1'b0;
			scan_active_q <= 1'b0;
			scan_data_valid_q <= 1'b0;
			result_low_hit_q <= 1'b0;
			result_high_hit_q <= 1'b0;
		end else if (load_begin_i) begin
			// The active generation remains visible until commit. Only the inactive
			// bank's logical length and any abandoned expansion are reset here.
			staging_count_q <= 6'd0;
			pending_count_q <= 3'd0;
			load_active_q <= 1'b1;
		end else begin
			if (commit_w) begin
				active_bank_q <= ~active_bank_q;
				active_count_q <= staging_count_q;
				load_active_q <= 1'b0;
				commit_done_o <= 1'b1;
				scan_active_q <= 1'b0;
				scan_data_valid_q <= 1'b0;
				result_low_hit_q <= 1'b0;
				result_high_hit_q <= 1'b0;
			end else if (table_write_w) begin
				staging_count_q <= staging_count_q + 6'd1;
				pending_addr_q <= pending_addr_q + 24'd1;
				pending_value_q <= {8'd0, pending_value_q[31:8]};
				pending_compare_q <= {8'd0, pending_compare_q[31:8]};
				pending_count_q <= pending_count_q - 3'd1;
				scan_active_q <= 1'b0;
				scan_data_valid_q <= 1'b0;
				result_low_hit_q <= 1'b0;
				result_high_hit_q <= 1'b0;
			end else if (!enable_i) begin
				scan_active_q <= 1'b0;
				scan_data_valid_q <= 1'b0;
				result_low_hit_q <= 1'b0;
				result_high_hit_q <= 1'b0;
			end else if (scan_restart_w) begin
				key_addr_q <= addr_i;
				key_be_q <= be_i;
				key_count_q <= active_count_q;
				key_bank_q <= active_bank_q;
				scan_active_q <= 1'b1;
				scan_data_valid_q <= 1'b1;
				scan_group_q <= 2'd0;
				low_hit_q <= 1'b0;
				high_hit_q <= 1'b0;
				result_low_hit_q <= 1'b0;
				result_high_hit_q <= 1'b0;
			end else if (scan_active_q && scan_data_valid_q) begin
				if (scan_group_q == 2'd3) begin
					// Start with the best prior group, then let the selected
					// match in this final, newer group overwrite it.
					result_low_hit_q <= low_hit_q;
					result_low_method_q <= low_method_q;
					result_low_value_q <= low_value_q;
					result_high_hit_q <= high_hit_q;
					result_high_method_q <= high_method_q;
					result_high_value_q <= high_value_q;
					if (group_low_hit) begin
						result_low_hit_q <= 1'b1;
						result_low_method_q <= group_low_method;
						result_low_value_q <= group_low_value;
					end
					if (group_high_hit) begin
						result_high_hit_q <= 1'b1;
						result_high_method_q <= group_high_method;
						result_high_value_q <= group_high_value;
					end
					scan_active_q <= 1'b0;
					scan_data_valid_q <= 1'b0;
				end else begin
					if (group_low_hit) begin
						low_hit_q <= 1'b1;
						low_method_q <= group_low_method;
						low_value_q <= group_low_value;
					end
					if (group_high_hit) begin
						high_hit_q <= 1'b1;
						high_method_q <= group_high_method;
						high_value_q <= group_high_value;
					end
					scan_group_q <= scan_group_q + 2'd1;
				end
			end

			if (load_active_q && !commit_w && code_valid_w && !table_write_w &&
				!(|code_addr_high_w) && record_aligned &&
				(record_byte_count != 3'd0) &&
				(({1'b0, staging_count_q} + {4'd0, record_byte_count}) <=
					{1'b0, MAX_CODES_COUNT})) begin
				pending_addr_q <= code_addr_w;
				pending_value_q <= code_value_w;
				pending_compare_q <= code_compare_w;
				pending_compare_enable_q <= code_compare_enable_w;
				pending_method_q <= code_method_w;
				pending_count_q <= record_byte_count;
			end
		end
	end

endmodule
