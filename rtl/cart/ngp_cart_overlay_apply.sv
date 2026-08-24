// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_cart_overlay_apply -- atomic sparse-overlay replacement.
//
// The S0/state front-end must have already staged and CRC-validated every
// target block before it raises start_i. This block then holds the machine at
// the existing frame-safe pause boundary, restores old live-difference blocks
// not replaced by the target from immutable DDR3 shadow, and only then applies
// the staged complete target blocks. It owns no HPS path and deliberately has
// no speculative write path:
// an invalid map causes zero p2 writes.
//
// DDR word bases are relative to the ddram aperture:
//   pristine shadow: 0x000000 byte / 0x000000 word
//   mutable staging: 0x400000 byte / 0x200000 word
// A two-die cartridge uses its normal cart-linear +0x200000 byte die-1 base.
module ngp_cart_overlay_apply
(
	input  wire        clk,
	input  wire        reset,
	input  wire        start_i,
	input  wire        pause_ready_i,

	input  wire [34:0] old_live0_i,
	input  wire [34:0] old_live1_i,
	input  wire [34:0] target0_i,
	input  wire [34:0] target1_i,
	input  wire [1:0]  die0_code_i,
	input  wire [1:0]  die1_code_i,
	input  wire [1:0]  die_busy_i,
	input  wire        event0_i,
	input  wire [5:0]  block0_i,
	input  wire        event1_i,
	input  wire [5:0]  block1_i,

	output wire        pause_req_o,
	output wire        busy_o,
	output reg         done_o,
	output reg         rejected_o,

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
	localparam [2:0] ST_PAUSE      = 3'd1;
	localparam [2:0] ST_FIND       = 3'd2;
	localparam [2:0] ST_MOVE_START = 3'd3;
	localparam [2:0] ST_MOVE_WAIT  = 3'd4;
	localparam [2:0] ST_ADOPT      = 3'd5;

	reg [2:0]  state_q;
	reg        apply_q;
	reg        scan_die_q;
	reg [5:0]  scan_block_q;
	reg        op_die_q;
	reg [5:0]  op_block_q;
	reg [34:0] old0_q, old1_q;
	reg [34:0] target0_q, target1_q;

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

	wire maps_valid_w = ((old_live0_i & ~block_mask(die0_code_i)) == 35'd0) &&
	                    ((old_live1_i & ~block_mask(die1_code_i)) == 35'd0) &&
	                    ((target0_i & ~block_mask(die0_code_i)) == 35'd0) &&
	                    ((target1_i & ~block_mask(die1_code_i)) == 35'd0);

	wire scan_selected_w = apply_q ?
		(scan_die_q ? target1_q[scan_block_q] : target0_q[scan_block_q]) :
		(scan_die_q ? old1_q[scan_block_q] : old0_q[scan_block_q]);
	wire [1:0] op_size_code_w = op_die_q ? die1_code_i : die0_code_i;

	wire geometry_valid_w;
	wire [20:0] geometry_base_w;
	wire [16:0] geometry_bytes_unused_w;
	wire [15:0] geometry_words_unused_w;

	ngp_cart_overlay_geometry u_geometry
	(
		.size_code_i (op_size_code_w),
		.block_i     (op_block_q),
		.valid_o     (geometry_valid_w),
		.base_o      (geometry_base_w),
		.bytes_o     (geometry_bytes_unused_w),
		.words_o     (geometry_words_unused_w)
	);

	// The mover receives a 16-bit-word base. `apply_q==0` reads immutable
	// shadow; applying reads the non-overlapping staging window.
	wire [23:0] physical_word_base_w =
		(op_die_q ? 24'h100000 : 24'd0) + {4'd0, geometry_base_w[20:1]};
	wire [23:0] mover_ddr_word_base_w = apply_q ?
		(24'h200000 + physical_word_base_w) : physical_word_base_w;

	wire mover_busy_w;
	wire mover_done_w;
	wire mover_stale_w;
	wire mover_rejected_w;

	ngp_cart_overlay_mover u_mover
	(
		.clk               (clk),
		.reset             (reset),
		.start_i           (state_q == ST_MOVE_START),
		.direction_i       (1'b1),
		.die_i             (op_die_q),
		.block_i           (op_block_q),
		.size_code_i       (op_size_code_w),
		.ddr_word_base_i   (mover_ddr_word_base_w),
		.die_busy_i        (die_busy_i),
		.event0_i          (event0_i),
		.block0_i          (block0_i),
		.event1_i          (event1_i),
		.block1_i          (block1_i),
		.busy_o            (mover_busy_w),
		.done_o            (mover_done_w),
		.stale_o           (mover_stale_w),
		.rejected_o        (mover_rejected_w),
		.p2_req_o          (p2_req_o),
		.p2_we_o           (p2_we_o),
		.p2_addr_o         (p2_addr_o),
		.p2_wdata_o        (p2_wdata_o),
		.p2_be_o           (p2_be_o),
		.p2_ready_i        (p2_ready_i),
		.p2_done_i         (p2_done_i),
		.p2_rdata_i        (p2_rdata_i),
		.ddr_addr_o        (ddr_addr_o),
		.ddr_din_o         (ddr_din_o),
		.ddr_req_o         (ddr_req_o),
		.ddr_rnw_o         (ddr_rnw_o),
		.ddr_be_o          (ddr_be_o),
		.ddr_dout_i        (ddr_dout_i),
		.ddr_ready_i       (ddr_ready_i)
	);

	assign pause_req_o = (state_q != ST_IDLE);
	assign busy_o = (state_q != ST_IDLE) || mover_busy_w;

	always @(posedge clk) begin
		done_o     <= 1'b0;
		rejected_o <= 1'b0;

		if (reset) begin
			state_q   <= ST_IDLE;
			apply_q   <= 1'b0;
			scan_die_q <= 1'b0;
			scan_block_q <= 6'd0;
			op_die_q  <= 1'b0;
			op_block_q <= 6'd0;
			old0_q    <= 35'd0;
			old1_q    <= 35'd0;
			target0_q <= 35'd0;
			target1_q <= 35'd0;
		end else begin
			case (state_q)
				ST_IDLE: if (start_i) begin
					if (!maps_valid_w) begin
						rejected_o <= 1'b1;
					end else begin
						// A target block is replaced in full, so restoring its old
						// bytes first only lengthens the held pause. Restore A & ~B;
						// the apply pass supplies every complete B block afterward.
						old0_q    <= old_live0_i & ~target0_i;
						old1_q    <= old_live1_i & ~target1_i;
						if (event0_i && (block0_i < 6'd35) && !target0_i[block0_i])
							old0_q[block0_i] <= 1'b1;
						if (event1_i && (block1_i < 6'd35) && !target1_i[block1_i])
							old1_q[block1_i] <= 1'b1;
						target0_q <= target0_i;
						target1_q <= target1_i;
						apply_q   <= 1'b0;
						scan_die_q <= 1'b0;
						scan_block_q <= 6'd0;
						state_q   <= ST_PAUSE;
					end
				end

				ST_PAUSE: begin
					// The frame-safe pause can take several cycles to acknowledge.
					// Keep folding completed flash mutations into the old overlay
					// until both dies have drained, so the later atomic adoption
					// cannot silently discard a just-completed write.
					if (event0_i && (block0_i < 6'd35) && !target0_q[block0_i])
						old0_q[block0_i] <= 1'b1;
					if (event1_i && (block1_i < 6'd35) && !target1_q[block1_i])
						old1_q[block1_i] <= 1'b1;
					if (pause_ready_i && (die_busy_i == 2'd0)) state_q <= ST_FIND;
				end

				ST_FIND: begin
					if (scan_selected_w) begin
						op_die_q   <= scan_die_q;
						op_block_q <= scan_block_q;
						state_q    <= ST_MOVE_START;
					end else if (!scan_die_q && (scan_block_q == 6'd34)) begin
						scan_die_q <= 1'b1;
						scan_block_q <= 6'd0;
					end else if (scan_die_q && (scan_block_q == 6'd34)) begin
						if (!apply_q) begin
							apply_q <= 1'b1;
							scan_die_q <= 1'b0;
							scan_block_q <= 6'd0;
						end else begin
							state_q <= ST_ADOPT;
						end
					end else begin
						scan_block_q <= scan_block_q + 6'd1;
					end
				end

				ST_MOVE_START: begin
					if (!geometry_valid_w) begin
						rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end else begin
						state_q <= ST_MOVE_WAIT;
					end
				end

				ST_MOVE_WAIT: if (mover_done_w) begin
					// Under the held pause a real mutation is an integrity failure;
					// never publish a partly restored overlay.
					if (mover_rejected_w || mover_stale_w) begin
						rejected_o <= 1'b1;
						state_q <= ST_IDLE;
					end else begin
						if (!scan_die_q && (scan_block_q == 6'd34)) begin
							scan_die_q <= 1'b1;
							scan_block_q <= 6'd0;
						end else if (scan_die_q && (scan_block_q == 6'd34)) begin
							if (!apply_q) begin
								apply_q <= 1'b1;
								scan_die_q <= 1'b0;
								scan_block_q <= 6'd0;
							end else begin
								state_q <= ST_ADOPT;
							end
						end else begin
							scan_block_q <= scan_block_q + 6'd1;
						end
						if (!(scan_die_q && (scan_block_q == 6'd34) && apply_q))
							state_q <= ST_FIND;
					end
				end

				ST_ADOPT: begin
					done_o <= 1'b1;
					state_q <= ST_IDLE;
				end

				default: begin
					rejected_o <= 1'b1;
					state_q <= ST_IDLE;
				end
			endcase
		end
	end

	// The geometry outputs are intentionally retained in the design cone; they
	// document that an apply request is grounded in a valid physical block.
	/* verilator lint_off UNUSED */
	wire unused_ok = &{1'b0, geometry_base_w[0], geometry_bytes_unused_w,
		geometry_words_unused_w, 1'b0};
	/* verilator lint_on UNUSED */

endmodule

`default_nettype wire
