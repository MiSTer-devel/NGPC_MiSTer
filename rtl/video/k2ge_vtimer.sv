// Copyright (c) 2026 Jamie Blanks

// K2GE raster timer.  The cold BIOS reset path spins on 0x8010 bit 6 (BLNK),
// so a machine without this counter hangs there.
//
//   6.144 MHz dot clock                    (NPK2 board Y1)
//   515 dots per line                      (K1GE p.17 / K2GE p.22)
//   199 lines, 0..198                      (K2GE p.21, REF reset 0xC6 = 198)
//   lines 0..151 drawn, 152..198 V blank   (K1GE p.4 / K2GE p.5, 160x152)
//   frame = 515 x 199 = 102485 dots = 16.6805 ms = 59.9502 Hz
//
// The chip renders one line while it displays another (K1GE p.4 feature 1).
// Which line each pass composes is fixed by the timing figure: "The signal
// generation begins 1 H before the Hardware Drawing Period starts.  (Please be
// aware H_INT signal is not generated at line 151 and signal generation for the
// 0th line occurs at the beginning of line 198.)" (K1GE p.25 / K2GE p.35).  So
// the Hardware Drawing Period of raster line L composes display row L and the
// Hint one line earlier announces it, giving 152 Hints and 152 composition
// passes (K1GE 3-4-12 / K2GE 4-5-2).
//
// Row L therefore shifts out during raster line L+1, which is why k2ge_scanout
// drives DE over raster lines 1..152 rather than 0..151.  BLNK and Vint still
// name the line being COMPOSED, because every CPU-visible quantity in the
// register reference is defined against the drawing operation rather than the
// panel: RAS.V is the "Horizontal drawing operation line number" (K1GE 3-4-13),
// Vint "occurs when the line WBA.V + WSI.V is drawn" (K1GE 3-4-11), and a
// register write "affects the display from the line drawn after setting"
// (K1GE 3-4-9).  RAS.V advances uniformly one interval ahead of the composed
// line, including through vertical blank; see the counter note below.
//
// Strobe convention: every strobe exported here is asserted on the clk_sys
// cycle whose rising edge ENTERS the named dot.  `line_start` is high while
// hdot still reads 514 and the edge it qualifies is the one that makes hdot 0,
// so a copy registered on that edge is stable for the whole of dot 0 -- which
// is what the latch groups and the drawing period need.  The same rule fixes
// the interrupt pins: the Hint pin is already low AT dot 480, so the clear must
// be taken by the edge entering dot P_HINT_FALL and not by a tick sampled
// during it.  PA1/TI0 follows the documented HINT waveform, so no edges are
// generated while HINT is inactive.
//
// RAS.H is a 10-bit DOWN counter over the 515-clock line, a "Negation Counter"
// (K1GE 3-4-13), reloaded from `P_RASH_RELOAD`.  The Hint schedule's line 198
// is written as the constant the documents draw rather than as `ref_last`: REF
// is "locked, priority user only" (K2GE 4-7) and the BIOS never writes it.

module k2ge_vtimer
#(
	// Last dot of the Hardware Drawing Period.  515 - 31 = 484 from the
	// documented "H Time Remaining (approx. 5 uS)" (K1GE section 4).
	parameter [9:0] P_HDRAW_END   = 10'd484,
	// Dot at which the Hint pin falls.
	parameter [9:0] P_HINT_FALL   = 10'd480,
	// RAS.H down-counter reload, loaded as the line wraps to dot 0.  514
	// makes RAS.H run 128 -> 0 across the line.
	parameter [9:0] P_RASH_RELOAD = 10'd514
)
(
	input  wire        clk_sys,        // 49.152 MHz
	input  wire        ce_6m144,       // dot-clock enable, 1 of 8
	input  wire        rst,            // synchronous power-on reset

	// --- from the register file -----------------------------------------
	input  wire        vi_e,           // 0x8000 D7, gates the Vint pin
	input  wire        hi_e,           // 0x8000 D6, gates the Hint pin
	input  wire [7:0]  ref_last,       // 0x8006 REF, per-frame latched copy
	input  wire        soft_reset,     // 0x87E0 = 0x52 wrote this cycle
	input  wire        covr_set,       // contention model; tie 0 in this build

	// --- raster position and status --------------------------------------
	output wire [9:0]  hdot,           // 0..514
	output wire [7:0]  vline,          // 0..REF
	output wire [7:0]  ras_h,          // 0x8008
	output wire [7:0]  ras_v,          // 0x8009
	output wire        blnk,           // 0x8010 D6
	output wire        covr,           // 0x8010 D7

	// --- shared strobes (see the convention note above) -------------------
	output wire        line_start,     // entering dot 0 of any line
	output wire        frame_start,    // entering dot 0 of line 0
	output wire        hdraw_end,      // entering dot P_HDRAW_END
	output wire        pass_start,     // entering dot 0 of a composition line
	output wire [7:0]  pass_line,      // display row that pass composes (= vline)

	// --- interrupt pins ---------------------------------------------------
	output wire        vint,           // -> TLCS INT4
	output wire        hint,           // documented 152-pulse external HINT
	output wire        ti0,            // MCU Timer0 alias of HINT

	// --- savestate internals slice ----------------------------------------
	// Local word 0 bits 2:0 and the whole of local word 6.  Word 0 is
	// co-owned with k2ge_mmr on a disjoint bit split: this module drives bits
	// 2:0 and reads 0 everywhere else, so the chip parent can merge the two
	// images with a plain OR.
	input  wire [4:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata,

	// --- pause / drain handshake ------------------------------------------
	input  wire        pause_req,
	output wire        pause_ready
);

	// Fixed geometry

	localparam [9:0] LAST_DOT     = 10'd514;   // 515 dots, 0..514
	localparam [7:0] V_ACTIVE     = 8'd152;    // rows 0..151 are drawn
	localparam [7:0] LAST_HINT_LN = 8'd150;    // last in-frame Hint line
	localparam [7:0] WRAP_HINT_LN = 8'd198;    // the Hint that announces row 0
	localparam [7:0] LAST_PASS_LN = 8'd151;    // last composition line (row 151)

	reg [9:0] hdot_q;
	reg [7:0] vline_q;
	reg [9:0] rash_q;

	reg       hint_q;                          // raw pin, before HI.E
	reg       vint_q;                          // raw pin, before VI.E
	reg       covr_q;                          // 0x8010 D7

	reg       pause_ready_q;

	// Next-position arithmetic: two 10-bit and one 8-bit increment.  Everything
	// else in the module compares against these, which is what keeps every
	// event on the edge that enters its dot.

	wire       line_wrap = (hdot_q == LAST_DOT);
	wire [9:0] hdot_nxt  = line_wrap ? 10'd0 : (hdot_q + 10'd1);
	wire [7:0] vline_nxt = !line_wrap ? vline_q
	                     : ((vline_q == ref_last) ? 8'd0 : (vline_q + 8'd1));

	// 152 Hint pulses, one per displayed row, each 1 H before the drawing period
	// it announces (K1GE p.25 / K2GE p.35): lines 0..150 plus line 198.
	wire hint_line_nxt = (vline_nxt <= LAST_HINT_LN) || (vline_nxt == WRAP_HINT_LN);

	// The composition pass runs on lines 0..151 and composes display row
	// `vline`: one drawing period per displayed row, one line after the Hint
	// that announced it.  RAS.V is a CPU-visible counter, not the
	// composition-pass coordinate.  Moving these passes to 1..152 makes a write
	// during line L affect row L instead of the documented next row -- Last
	// Blade's line-31 S1SO.V write of 0x78 then maps row 31 onto tilemap row
	// 151, copying the bottom of PAUSE across the top of the playfield.
	wire pass_line_nxt = (vline_nxt <= LAST_PASS_LN);

	wire ce_wrap = ce_6m144 && line_wrap;      // the edge that enters dot 0

	// Savestate tap

	wire ss_wr = ss_wren && pause_req;         // honoured only while paused

	reg [63:0] ss_rdata_r;

	always @* begin
		case (ss_addr)
			// Word 0, this module's three bits.  k2ge_mmr owns 63:47 of the
			// same word and reads 0 here.
			5'd0:    ss_rdata_r = {61'd0, covr_q, vint_q, hint_q};
			// Word 6: the raster counters.  The VRAM arbiter's slot phase
			// is not this module's state and is left as zeroes.
			5'd6:    ss_rdata_r = {hdot_q, vline_q, rash_q, 36'd0};
			default: ss_rdata_r = 64'd0;
		endcase
	end

	assign ss_rdata = ss_rdata_r;

	// RAS.H is the upper 8 bits of a 10-bit subtraction counter over the
	// 515-clock line; the value DECREASES as the line progresses (K1GE 3-4-13;
	// K2GE p.22, "Negation Counter").  Reloading 514 at the wrap makes it read
	// 128 at dot 0 and 0 at dot 514.  Both RAS registers are documented as
	// "accessible during V blank", but the live value is returned at any time
	// because raster-effect software reads them mid-line.
	//
	// A software reset (0x87E0 = 0x52) deliberately does not touch these:
	// resetting the raster mid-frame would tear the display.
	//
	// RAS.V is the next horizontal-drawing line (vline + 1, wrapping at REF).
	// It advances once per line boundary across the whole 0..REF range, so
	// interval 197 reads 198 and 198 wraps to 0, and a poll of 0x8009 therefore
	// reports at most 198.  Mr. Do's Hint ISR reads RAS.V and writes S2SO.V at
	// 0/16/136 to form exact 16/120/16 bands, which only the +1 form places on
	// the rows hardware shows.
	wire [7:0] rasv_draw_line = (vline_q == ref_last)
	                            ? 8'd0 : (vline_q + 8'd1);

	always @(posedge clk_sys) begin
		if (rst) begin
			hdot_q  <= 10'd0;
			vline_q <= 8'd0;
			rash_q  <= P_RASH_RELOAD;
		end else if (ss_wr && (ss_addr == 5'd6)) begin
			hdot_q  <= ss_wdata[63:54];
			vline_q <= ss_wdata[53:46];
			rash_q  <= ss_wdata[45:36];
		end else if (ce_6m144) begin
			hdot_q  <= hdot_nxt;
			vline_q <= vline_nxt;
			rash_q  <= line_wrap ? P_RASH_RELOAD : (rash_q - 10'd1);
		end
	end

	// Hint  : set entering dot 0 of an announcing interval, cleared entering
	//         dot P_HINT_FALL of every line (K1GE p.25).
	// Vint  : set entering the first line past the drawing period, cleared
	//         entering line 0 (K1GE 3-4-11, "Vint occurs when the line
	//         WBA.V + WSI.V is drawn").
	// C.OVR : set by the contention model, "cleared with the end of V blanking"
	//         (K1GE 3-5) = entering line 0, and by the 2D software reset
	//         (K1GE 3-10).
	//
	// P_HINT_FALL = 0 would make the Hint set and clear collide on the same
	// edge; the SET wins, so the pin asserts rather than silently vanishing.
	//
	// Both Vint arms are equality compares against `vline_nxt`, which only ever
	// takes 0..ref_last, so both targets must be lines the counter actually
	// reaches and they must differ.  A set line of 0 collides with the clear and
	// leaves `vint_q` latched high with no further rising edge; a set line above
	// ref_last never fires while the clear fires every frame, leaving the pin
	// low.  Either way ngp_intc sees no INT4 rising edge, the BIOS VBlank ISR
	// stops running, its watchdog kick stops with it, and INT_WATCHDOG powers
	// the machine off with a blank panel.  min(ref_last, 153) with a clear line
	// of 1 when the set landed on 0 satisfies the rule for every ref_last >= 1;
	// ref_last == 0 has no second line to clear on and cannot be satisfied.
	//
	// Vint marks the end of the drawing period and does not follow the window.
	// It must fire one line PAST the highest line a frame-sync poll can resolve
	// to, or a title polling RAS.V loses a frame per iteration, and it must not
	// fire at the START of a polled line or the poll never observes that value.
	// The `ref_last` arm keeps the only dependence worth having: a REF
	// programmed below the active area must still leave the target line
	// reachable.
	wire [7:0] vint_set_line = (ref_last < (V_ACTIVE + 8'd1))
	                           ? ref_last : (V_ACTIVE + 8'd1);
	wire [7:0] vint_clr_line = (vint_set_line == 8'd0) ? 8'd1 : 8'd0;

	always @(posedge clk_sys) begin
		if (rst) begin
			hint_q <= 1'b0;
			vint_q <= 1'b0;
			covr_q <= 1'b0;
		end else if (ss_wr && (ss_addr == 5'd0)) begin
			covr_q <= ss_wdata[2];
			vint_q <= ss_wdata[1];
			hint_q <= ss_wdata[0];
		end else begin
			if (ce_6m144) begin
				if (line_wrap && hint_line_nxt) begin
					hint_q <= 1'b1;
				end else if (hdot_nxt == P_HINT_FALL) begin
					hint_q <= 1'b0;
				end

				if (line_wrap && (vline_nxt == vint_set_line)) begin
					vint_q <= 1'b1;
				end else if (line_wrap && (vline_nxt == vint_clr_line)) begin
					vint_q <= 1'b0;
				end

				if (covr_set) begin
					covr_q <= 1'b1;
				end else if (line_wrap && (vline_nxt == 8'd0)) begin
					covr_q <= 1'b0;
				end
			end
			// The software reset wins over the same-cycle raster clear; it
			// is placed last so its assignment is the surviving one.
			if (soft_reset) begin
				covr_q <= 1'b0;
			end
		end
	end

	// Pause: let the current native frame finish and report ready on entry to
	// line 0.  The CRT coordinator releases this parked boundary only at its own
	// frame wrap, which is what makes strict two-bank ownership safe without
	// per-frame pacing.  Once the parent sees `pause_ready` it gates ce_6m144,
	// so this flop holds with no further CE ticks.

	always @(posedge clk_sys) begin
		if (rst) begin
			pause_ready_q <= 1'b0;
		end else if (!pause_req) begin
			pause_ready_q <= 1'b0;
		end else if (ce_wrap && (vline_nxt == 8'd0)) begin
			pause_ready_q <= 1'b1;
		end
	end

	assign pause_ready = pause_ready_q;

	// Outputs

	assign hdot        = hdot_q;
	assign vline       = vline_q;
	assign ras_h       = rash_q[9:2];
	assign ras_v       = rasv_draw_line;
	assign blnk        = (vline_q >= V_ACTIVE);
	assign covr        = covr_q;

	assign line_start  = ce_wrap;
	assign frame_start = ce_wrap && (vline_nxt == 8'd0);
	assign pass_start  = ce_wrap && pass_line_nxt;
	assign hdraw_end   = ce_6m144 && (hdot_nxt == P_HDRAW_END);

	// Formed from vline_nxt so that it already names the new line on the
	// pass_start edge and still names the line under composition for the rest
	// of the line (vline_nxt == vline everywhere except that edge).  Kept
	// separate from the raw RAS.V readback: a write during line L is captured
	// at the next line start and governs display row L+1, which is the phase
	// the register documentation requires.
	assign pass_line   = vline_nxt;

	assign hint        = hint_q & hi_e;
	assign ti0         = hint;

	// Active high here on purpose: this is the chip-internal event, high for
	// the whole vertical blanking period.  The pad is active low and the
	// inversion is done once where the pad is bonded, because vint_q is a
	// savestate bit and inverting it here would change the state format.
	assign vint        = vint_q & vi_e;

	// Deliberately unread inputs: only three bits of savestate word 0 and 28
	// bits of word 6 are ours.
	wire unused_ok = &{1'b0, ss_wdata, 1'b0};

endmodule
