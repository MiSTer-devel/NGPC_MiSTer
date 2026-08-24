// Copyright (c) 2026 Jamie Blanks

module statemanager
#(
	parameter integer Softmap_SaveState_ADDR = 0,
	parameter integer Softmap_Rewind_ADDR = 0,
	parameter integer SAVESTATE_SHIFT = 17
)
(
	input  wire clk,
	input  wire reset,

	input  wire rewind_on,
	input  wire rewind_active,

	input  integer savestate_number,
	input  wire save,
	input  wire load,

	output reg  sleep_rewind = 1'b0,
	input  wire vsync,

	output reg  request_savestate = 1'b0,
	output reg  request_loadstate = 1'b0,
	// High in the same cycle as a request pulse when that request belongs to
	// the rewind ring rather than a user slot.  savestates.sv latches
	// state_size_i / savetype3_size_i / is_rewind_i at request acceptance, so
	// the parent must mux those off this flag combinationally, not a cycle
	// later.
	output reg  request_is_rewind = 1'b0,
	output integer request_address,
	input  wire request_busy
);

`ifdef SYNTHESIS
	localparam SS_EQUIV_SCALE = 1'b0;
`elsif SS_EQUIV_FAST
	localparam SS_EQUIV_SCALE = 1'b1;
`else
	localparam SS_EQUIV_SCALE = 1'b0;
`endif

	localparam integer REWIND_COUNT = 4;

	// TIME_CAPTURE is the gap between rewind snapshots in clk_sys cycles
	// (49.152 MHz).  The machine is frozen for the whole of a capture, so the
	// number is a freeze-duty budget rather than a taste call:
	//
	//   freeze = 32,768 bytes of BRAM regions (work RAM 12288 + Z80 shared
	//            4096 + video 16384) at ~10 clk_sys per byte through the
	//            byte-serial tap, plus ~10 us of internals
	//          = 329,318 clk_sys ~= 6.7 ms
	//   budget = freeze <= 1% of wall time  ->  gap >= 100 * freeze
	//          = 32,931,800 clk_sys
	//   frame  = 515 dots x 199 lines x 8 clk_sys per dot = 819,880 clk_sys
	//   gap    = ceil(32,931,800 / 819,880) = 41 frames
	//          = 41 * 819,880 = 33,615,080 clk_sys = 683.9 ms, duty 0.98%
	//
	// A whole number of frames keeps every hitch at the same raster phase
	// instead of walking it across the picture.  The cost is rewind depth:
	// REWIND_COUNT * 683.9 ms = 2.7 s of history.
	localparam integer TIME_CAPTURE = SS_EQUIV_SCALE ? 40 : 33615080;
	localparam integer TIME_REWIND = SS_EQUIV_SCALE ? 24 : 5000000;

	reg save_q = 1'b0;
	reg load_q = 1'b0;
	reg save_pending_q = 1'b0;
	reg load_pending_q = 1'b0;
	reg rewind_enabled_q = 1'b0;
	reg rewind_load_pending_q = 1'b0;
	integer capture_timer_q = 0;
	integer rewind_timer_q = 0;
	integer rewind_count_q = 0;
	integer rewind_position_q = 0;
	integer vsync_count_q = 0;
	reg vsync_q;

	function integer savestate_slot_addr;
		input integer base_addr_i;
		input integer slot_i;
		begin
			savestate_slot_addr = base_addr_i + (slot_i << SAVESTATE_SHIFT);
		end
	endfunction

	always @(posedge clk) begin
		request_savestate <= 1'b0;
		request_loadstate <= 1'b0;
		request_is_rewind <= 1'b0;
		vsync_q <= vsync;

		save_q <= save;
		if (save && !save_q) begin
			save_pending_q <= 1'b1;
		end
		load_q <= load;
		if (load && !load_q) begin
			load_pending_q <= 1'b1;
		end

		// rewind_load_pending_q is sticky (see the request block below), so it
		// has to be dropped wherever the ring itself is invalidated: turning
		// rewind off or a reset re-primes the ring from position 1, and a
		// pending load carrying the old position would restore a stale slot.
		if (!rewind_on || reset) begin
			rewind_enabled_q <= 1'b0;
			rewind_load_pending_q <= 1'b0;
		end

		if (!rewind_active) begin
			rewind_timer_q <= 0;
		end else if (rewind_timer_q < TIME_REWIND) begin
			rewind_timer_q <= rewind_timer_q + 1;
		end

		if (rewind_active) begin
			capture_timer_q <= 0;
		end else if (capture_timer_q < TIME_CAPTURE) begin
			capture_timer_q <= capture_timer_q + 1;
		end

		if ((vsync_count_q < 2) && vsync && !vsync_q) begin
			vsync_count_q <= vsync_count_q + 1;
		end

		sleep_rewind <= 1'b0;
		if ((vsync_count_q == 2) && rewind_active) begin
			sleep_rewind <= 1'b1;
		end

		if (!reset && !request_busy) begin
			if (save_pending_q) begin
				request_address <= savestate_slot_addr(Softmap_SaveState_ADDR, savestate_number);
				request_savestate <= 1'b1;
				save_pending_q <= 1'b0;
			end else if (load_pending_q) begin
				request_address <= savestate_slot_addr(Softmap_SaveState_ADDR, savestate_number);
				request_loadstate <= 1'b1;
				load_pending_q <= 1'b0;
			end else if (!rewind_enabled_q && rewind_on) begin
				request_address <= Softmap_Rewind_ADDR;
				request_savestate <= 1'b1;
				request_is_rewind <= 1'b1;
				rewind_enabled_q <= 1'b1;
				capture_timer_q <= 0;
				rewind_count_q <= 1;
				rewind_position_q <= 1;
			end else if (rewind_enabled_q && (capture_timer_q == TIME_CAPTURE)) begin
				request_address <= savestate_slot_addr(Softmap_Rewind_ADDR, rewind_position_q);
				request_savestate <= 1'b1;
				request_is_rewind <= 1'b1;
				capture_timer_q <= 0;
				if (rewind_count_q < REWIND_COUNT) begin
					rewind_count_q <= rewind_count_q + 1;
				end
				if (rewind_position_q < (REWIND_COUNT - 1)) begin
					rewind_position_q <= rewind_position_q + 1;
				end else begin
					rewind_position_q <= 0;
				end
			end else if (rewind_enabled_q && (rewind_timer_q == TIME_REWIND)) begin
				if (rewind_count_q > 1) begin
					rewind_count_q <= rewind_count_q - 1;
					if (rewind_position_q > 0) begin
						rewind_position_q <= rewind_position_q - 1;
					end else begin
						rewind_position_q <= REWIND_COUNT - 1;
					end
					rewind_load_pending_q <= 1'b1;
				end
				rewind_timer_q <= 0;
			end else if (rewind_load_pending_q) begin
				// Consumed here and only here, so a busy engine cannot lose
				// a rewind step whose ring position has already moved.
				request_address <= savestate_slot_addr(Softmap_Rewind_ADDR, rewind_position_q);
				request_loadstate <= 1'b1;
				request_is_rewind <= 1'b1;
				rewind_load_pending_q <= 1'b0;
				vsync_count_q <= 0;
			end
		end
	end

endmodule
