// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_cart_overlay_mover -- copy one complete physical flash block between
// canonical cartridge SDRAM and DDR3.  The caller assigns a non-overlapping
// DDR3 byte range (pristine or staging); this mover only derives physical
// block bounds and preserves each 16-bit word's byte lanes.
//
// direction_i = 0: live cartridge -> DDR3 (coherent snapshot/staging)
// direction_i = 1: DDR3 -> live cartridge (pristine restore/overlay apply)
//
// A flash completion in the selected block, or a busy selected die, makes a
// capture stale. The data movement is allowed to finish so callers can reuse
// the bounded path, but they must retry instead of committing that capture.
module ngp_cart_overlay_mover
(
	input  wire        clk,
	input  wire        reset,
	input  wire        start_i,
	input  wire        direction_i,
	input  wire        die_i,
	input  wire [5:0]  block_i,
	input  wire [1:0]  size_code_i,
	input  wire [24:1] ddr_word_base_i, // DDR3 byte base divided by two
	input  wire [1:0]  die_busy_i,
	input  wire        event0_i,
	input  wire [5:0]  block0_i,
	input  wire        event1_i,
	input  wire [5:0]  block1_i,

	output wire        busy_o,
	output wire        done_o,
	output wire        stale_o,
	output wire        rejected_o,

	output wire        p2_req_o,
	output wire        p2_we_o,
	output wire [24:0] p2_addr_o,
	output wire [15:0] p2_wdata_o,
	output wire [1:0]  p2_be_o,
	input  wire        p2_ready_i,
	input  wire        p2_done_i,
	input  wire [15:0] p2_rdata_i,

	output wire [27:1] ddr_addr_o,
	output wire [63:0] ddr_din_o,
	output wire        ddr_req_o,
	output wire        ddr_rnw_o,
	output wire [7:0]  ddr_be_o,
	input  wire [63:0] ddr_dout_i,
	input  wire        ddr_ready_i
);

	localparam [2:0] ST_IDLE       = 3'd0;
	localparam [2:0] ST_P2_RD_REQ  = 3'd1;
	localparam [2:0] ST_P2_RD_WAIT = 3'd2;
	localparam [2:0] ST_DDR_WR_REQ = 3'd3;
	localparam [2:0] ST_DDR_RD_REQ = 3'd4;
	localparam [2:0] ST_P2_WR_REQ  = 3'd5;
	localparam [2:0] ST_P2_WR_WAIT = 3'd6;

	reg [2:0]  state_q;
	reg        die_q;
	reg [5:0]  block_q;
	reg [20:0] block_base_q;
	reg [23:0] ddr_word_base_q;
	reg [15:0] word_count_q;
	reg [15:0] word_index_q;
	reg [15:0] data_q;
	reg        stale_q;
	reg        done_q;
	reg        rejected_q;

	wire        geometry_valid_w;
	wire [20:0] geometry_base_w;
	wire [15:0] geometry_words_w;

	ngp_cart_overlay_geometry geometry
	(
		.size_code_i (size_code_i),
		.block_i     (block_i),
		.valid_o     (geometry_valid_w),
		.base_o      (geometry_base_w),
		.bytes_o     (),
		.words_o     (geometry_words_w)
	);

	wire [16:0] word_byte_offset_w = {word_index_q, 1'b0};
	wire [24:0] p2_address_w = {die_q ? 4'd1 : 4'd0, block_base_q} +
		{8'd0, word_byte_offset_w};
	// Both endpoints are 16-bit word streams. The caller supplies the DDR3
	// base in word units, so no byte-address bit is discarded in this datapath.
	wire [23:0] ddr_even_address_w = ddr_word_base_q + {8'd0, word_index_q};

	assign busy_o     = (state_q != ST_IDLE);
	assign done_o     = done_q;
	assign stale_o    = stale_q;
	assign rejected_o = rejected_q;

	// p2 is a one-entry request mailbox. It admits a pulse only while ready,
	// then its own done signal authorizes advancement to the next word.
	assign p2_req_o   = ((state_q == ST_P2_RD_REQ) ||
			(state_q == ST_P2_WR_REQ)) && p2_ready_i;
	assign p2_we_o    = (state_q == ST_P2_WR_REQ);
	assign p2_addr_o  = p2_address_w;
	assign p2_wdata_o = data_q;
	assign p2_be_o    = 2'b11;

	// ddram.v works in eight-byte beats. A 16-bit mover word selects one of
	// four lanes; full blocks are even-byte sized by FlashMem geometry.
	assign ddr_addr_o = {3'd0, ddr_even_address_w[23:2], 2'b00};
	assign ddr_req_o  = (state_q == ST_DDR_WR_REQ) ||
		(state_q == ST_DDR_RD_REQ);
	assign ddr_rnw_o  = (state_q == ST_DDR_RD_REQ);

	reg [63:0] ddr_din_q;
	reg [7:0]  ddr_be_q;

	always @* begin
		ddr_din_q = 64'd0;
		ddr_be_q  = 8'd0;
		case (ddr_even_address_w[1:0])
			2'd0: begin ddr_din_q[15:0]  = data_q; ddr_be_q = 8'h03; end
			2'd1: begin ddr_din_q[31:16] = data_q; ddr_be_q = 8'h0C; end
			2'd2: begin ddr_din_q[47:32] = data_q; ddr_be_q = 8'h30; end
			default: begin ddr_din_q[63:48] = data_q; ddr_be_q = 8'hC0; end
		endcase
	end

	assign ddr_din_o = ddr_din_q;
	assign ddr_be_o  = ddr_be_q;

	function [15:0] selected_ddr_word;
		input [63:0] value_i;
		input [1:0] lane_i;
		begin
			case (lane_i)
				2'd0: selected_ddr_word = value_i[15:0];
				2'd1: selected_ddr_word = value_i[31:16];
				2'd2: selected_ddr_word = value_i[47:32];
				default: selected_ddr_word = value_i[63:48];
			endcase
		end
	endfunction

	wire selected_event_w = (die_q == 1'b0) ?
		(event0_i && (block0_i == block_q)) :
		(event1_i && (block1_i == block_q));
	wire selected_die_busy_w = die_q ? die_busy_i[1] : die_busy_i[0];

	always @(posedge clk) begin
		done_q     <= 1'b0;
		rejected_q <= 1'b0;

		if (reset) begin
			state_q      <= ST_IDLE;
			die_q        <= 1'b0;
			block_q      <= 6'd0;
			block_base_q <= 21'd0;
			ddr_word_base_q <= 24'd0;
			word_count_q <= 16'd0;
			word_index_q <= 16'd0;
			data_q       <= 16'd0;
			stale_q      <= 1'b0;
		end else begin
			if (busy_o && (selected_event_w || selected_die_busy_w)) begin
				stale_q <= 1'b1;
			end

			case (state_q)
				ST_IDLE: begin
					if (start_i) begin
						if (!geometry_valid_w) begin
							rejected_q <= 1'b1;
						end else begin
							die_q        <= die_i;
							block_q      <= block_i;
							block_base_q <= geometry_base_w;
							ddr_word_base_q <= ddr_word_base_i;
							word_count_q <= geometry_words_w;
							word_index_q <= 16'd0;
							stale_q      <= die_i ? die_busy_i[1] : die_busy_i[0];
							state_q      <= direction_i ? ST_DDR_RD_REQ : ST_P2_RD_REQ;
						end
					end
				end

				ST_P2_RD_REQ: begin
					if (p2_ready_i) state_q <= ST_P2_RD_WAIT;
				end

				ST_P2_RD_WAIT: begin
					if (p2_done_i) begin
						data_q  <= p2_rdata_i;
						state_q <= ST_DDR_WR_REQ;
					end
				end

				ST_DDR_WR_REQ: begin
					if (ddr_ready_i) begin
						if (word_index_q == (word_count_q - 16'd1)) begin
							state_q <= ST_IDLE;
							done_q  <= 1'b1;
						end else begin
							word_index_q <= word_index_q + 16'd1;
							state_q <= ST_P2_RD_REQ;
						end
					end
				end

				ST_DDR_RD_REQ: begin
					if (ddr_ready_i) begin
						data_q  <= selected_ddr_word(ddr_dout_i, ddr_even_address_w[1:0]);
						state_q <= ST_P2_WR_REQ;
					end
				end

				ST_P2_WR_REQ: begin
					if (p2_ready_i) state_q <= ST_P2_WR_WAIT;
				end

				default: begin
					if (p2_done_i) begin
						if (word_index_q == (word_count_q - 16'd1)) begin
							state_q <= ST_IDLE;
							done_q  <= 1'b1;
						end else begin
							word_index_q <= word_index_q + 16'd1;
							state_q <= ST_DDR_RD_REQ;
						end
					end
				end
			endcase
		end
	end

endmodule

`default_nettype wire
