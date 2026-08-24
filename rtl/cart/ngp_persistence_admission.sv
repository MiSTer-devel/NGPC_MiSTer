// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// Shared admission boundary for cartridge replacement, sparse S0, and sparse
// manual states. statemanager pipelines a UI request for several clocks before
// savestate_busy rises, so the raw request reserves p2/DDR until then.
module ngp_persistence_admission
(
	input  wire clk,
	input  wire ce,
	input  wire reset,
	input  wire cart_download_i,
	input  wire overlay_busy_i,
	input  wire savestate_busy_i,
	input  wire seed_busy_i,
	input  wire ss_save_i,
	input  wire ss_load_i,
	output wire state_reserved_o,
	output wire persistent_busy_o,
	output wire overlay_enable_o
);

	reg state_pending_q;

	always @(posedge clk) begin
		if (reset) begin
			state_pending_q <= 1'b0;
		end else if (ce) begin
			if (ss_save_i || ss_load_i)
				state_pending_q <= 1'b1;
			else if (savestate_busy_i)
				state_pending_q <= 1'b0;
		end
	end

	assign state_reserved_o = savestate_busy_i || state_pending_q ||
	                          ss_save_i || ss_load_i;
	assign persistent_busy_o = overlay_busy_i || state_reserved_o || seed_busy_i;
	assign overlay_enable_o = !state_reserved_o && !cart_download_i;

endmodule

`default_nettype wire
