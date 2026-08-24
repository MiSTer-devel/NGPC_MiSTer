// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_cart_overlay_ledger -- ownership of sparse-flash overlay membership.
//
// The three maps deliberately have different lifetimes:
//   live_diff  blocks whose live SDRAM may differ from the pristine shadow;
//   pending    mutations not represented by the committed S0 generation;
//   file       blocks selected by that committed S0 directory.
//
// The `snapshot`/`commit` pair handles the critical same-block race. A flash
// completion after snapshot belongs in post_snapshot even if it is the same
// bit as the captured map; commit clears only the captured generation.
module ngp_cart_overlay_ledger
(
	input  wire        clk,
	input  wire        reset,
	input  wire        cart_replace_i,

	input  wire        event0_i,
	input  wire [5:0]  block0_i,
	input  wire        event1_i,
	input  wire [5:0]  block1_i,

	input  wire        snapshot_i,
	input  wire        abort_i,
	input  wire        commit_i,
	input  wire [34:0] commit_file0_i,
	input  wire [34:0] commit_file1_i,

	// Overlay load replaces the complete live/file maps. A manual state uses
	// state_adopt_i instead so it also marks its blocks pending for the next S0.
	input  wire        load_adopt_i,
	input  wire [34:0] load_map0_i,
	input  wire [34:0] load_map1_i,
	input  wire        state_adopt_i,
	input  wire [34:0] state_map0_i,
	input  wire [34:0] state_map1_i,

	// A staged complete physical block exactly equal to pristine can be removed
	// without storing a redundant payload. Allocation retention is owned by the
	// directory controller, not by these logical maps.
	input  wire [34:0] pristine0_i,
	input  wire [34:0] pristine1_i,

	output reg  [34:0] live0_o,
	output reg  [34:0] live1_o,
	output reg  [34:0] pending0_o,
	output reg  [34:0] pending1_o,
	output reg  [34:0] file0_o,
	output reg  [34:0] file1_o,
	output reg  [34:0] snapshot0_o,
	output reg  [34:0] snapshot1_o,
	output wire        snapshot_active_o
);

	reg        snapshot_active_q;
	reg [34:0] post_snapshot0_q;
	reg [34:0] post_snapshot1_q;

	assign snapshot_active_o = snapshot_active_q;

	wire event0_valid_w = event0_i && (block0_i < 6'd35);
	wire event1_valid_w = event1_i && (block1_i < 6'd35);
	wire [34:0] event0_mask_w = event0_valid_w ? (35'd1 << block0_i) : 35'd0;
	wire [34:0] event1_mask_w = event1_valid_w ? (35'd1 << block1_i) : 35'd0;

	always @(posedge clk) begin
		if (reset || cart_replace_i) begin
			live0_o           <= 35'd0;
			live1_o           <= 35'd0;
			pending0_o        <= 35'd0;
			pending1_o        <= 35'd0;
			file0_o           <= 35'd0;
			file1_o           <= 35'd0;
			snapshot0_o       <= 35'd0;
			snapshot1_o       <= 35'd0;
			post_snapshot0_q  <= 35'd0;
			post_snapshot1_q  <= 35'd0;
			snapshot_active_q <= 1'b0;
		end else if (load_adopt_i) begin
			live0_o           <= load_map0_i;
			live1_o           <= load_map1_i;
			pending0_o        <= 35'd0;
			pending1_o        <= 35'd0;
			file0_o           <= load_map0_i;
			file1_o           <= load_map1_i;
			snapshot0_o       <= 35'd0;
			snapshot1_o       <= 35'd0;
			post_snapshot0_q  <= 35'd0;
			post_snapshot1_q  <= 35'd0;
			snapshot_active_q <= 1'b0;
		end else if (state_adopt_i) begin
			live0_o           <= state_map0_i;
			live1_o           <= state_map1_i;
			pending0_o        <= state_map0_i;
			pending1_o        <= state_map1_i;
			snapshot0_o       <= 35'd0;
			snapshot1_o       <= 35'd0;
			post_snapshot0_q  <= 35'd0;
			post_snapshot1_q  <= 35'd0;
			snapshot_active_q <= 1'b0;
		end else begin
			// A fresh physical completion always wins over a simultaneous
			// compare-to-pristine indication. The two are normally serialized by
			// the mover, but this priority makes the ledger safe if that contract
			// is ever relaxed.
			live0_o <= (live0_o & ~pristine0_i) | event0_mask_w;
			live1_o <= (live1_o & ~pristine1_i) | event1_mask_w;

			if (abort_i && snapshot_active_q) begin
				// The snapshot never removed its bits from pending. Abort only
				// releases transaction ownership and folds in the separately
				// tracked post-snapshot epoch, so a retry cannot lose work.
				pending0_o        <= (pending0_o & ~pristine0_i) |
				                     post_snapshot0_q | event0_mask_w;
				pending1_o        <= (pending1_o & ~pristine1_i) |
				                     post_snapshot1_q | event1_mask_w;
				snapshot0_o       <= 35'd0;
				snapshot1_o       <= 35'd0;
				post_snapshot0_q  <= 35'd0;
				post_snapshot1_q  <= 35'd0;
				snapshot_active_q <= 1'b0;
			end else if (commit_i && snapshot_active_q) begin
				file0_o           <= commit_file0_i;
				file1_o           <= commit_file1_i;
				pending0_o        <= (((pending0_o & ~pristine0_i) |
				                       event0_mask_w) & ~snapshot0_o) |
				                      post_snapshot0_q | event0_mask_w;
				pending1_o        <= (((pending1_o & ~pristine1_i) |
				                       event1_mask_w) & ~snapshot1_o) |
				                      post_snapshot1_q | event1_mask_w;
				snapshot0_o       <= 35'd0;
				snapshot1_o       <= 35'd0;
				post_snapshot0_q  <= 35'd0;
				post_snapshot1_q  <= 35'd0;
				snapshot_active_q <= 1'b0;
			end else begin
				pending0_o <= (pending0_o & ~pristine0_i) | event0_mask_w;
				pending1_o <= (pending1_o & ~pristine1_i) | event1_mask_w;

				if (snapshot_i && !snapshot_active_q) begin
					snapshot0_o       <= (pending0_o & ~pristine0_i) | event0_mask_w;
					snapshot1_o       <= (pending1_o & ~pristine1_i) | event1_mask_w;
					post_snapshot0_q  <= 35'd0;
					post_snapshot1_q  <= 35'd0;
					snapshot_active_q <= 1'b1;
				end else if (snapshot_active_q) begin
					post_snapshot0_q <= post_snapshot0_q | event0_mask_w;
					post_snapshot1_q <= post_snapshot1_q | event1_mask_w;
				end
			end
		end
	end

endmodule

`default_nettype wire
