// Copyright (c) 2026 Jamie Blanks

// K2GE scanout: window clip, OOWC fill, NEG inversion, the panel dot-clock
// enable and the LCD output framing.  The last block inside the chip.  It walks
// the displayed line, reads one entry per dot out of k2ge_linebuf, decides what
// that dot is -- window fill, background, or a composed pixel -- hands the
// decision to k2ge_palette, and registers the result with the framing that
// describes it.  Out comes the chip's native raster on its LCD pads: 160 x 152
// inside 515 x 199 at 6.144 MHz, 59.9503 Hz.  ngpc_crt_framebuffer downstream
// turns that into the MiSTer framework's video bus; nothing here knows about
// the framework.
//
// CE_PIXEL is a divide-by-3 enable RESTARTED at every line start.  It must keep
// pulsing through blanking, because the framework samples syncs and blanks only
// on CE_PIXEL ticks (ascal, the gamma stage and the scandoubler all do) and an
// enable that stops during blanking makes the scaler lose the edges.  And 515
// is not divisible by 3, so a free-running divider would slip a third of a pixel
// per line and the phase would walk.  Restarting gives:
//
//     ticks at hdot = 0, 3, 6, ... 513              172 ticks per line
//     active     hdot = 0, 3, ... 477               160 ticks, lcd_de = 1
//     blanking   hdot = 480, 483, ... 513            12 ticks, lcd_de = 0
//     the gap from the tick at 513 to the next line's tick at 0 is 2 dots,
//     because 515 = 3*171 + 2
//
// Every line has the same phase and exactly one 2-dot gap, at the end of the
// line.  The framework sees a 172 x 199 raster at an average 2.0517 MHz pixel
// rate and a true 59.9503 Hz frame rate.  Everything the panel sees -- RGB, DE,
// HS, VS, LP, SP -- changes only on one of those ticks.
//
// The four pipeline stages all fit inside one CE period (a tick every 3 dots =
// 24 clk_sys cycles), so the pixel data comes out one CE tick behind the raster
// counters and the framing is pushed through the same one stage.  Cycle by
// cycle, after the tick for dot d (pixel p):
//
//   tick    latch the line-buffer address and the framing for pixel p, and
//           emit the finished pixel p-1 with ITS framing
//   +1      the line buffer has the address                       (P0)
//   +2      the entry is back; capture it
//   +3      form and register the palette address                 (P1)
//   +4      the palette RAM has the address
//   +5      the entry is back; NEG, blanking, register the colour (P2/P3)
//   ...     18 idle cycles of slack before the next tick
//
// Framing and colour are registered together in P3, so DE marks exactly the 160
// dots whose colour is on the bus and no edge can land off a tick.
//
// The window is an origin (WBA) and a SIZE (WSI), not a pair of corners
// (K1GE 3-4-10; K2GE 4-5), and the area outside it is filled with the OOWC
// colour rather than left undrawn: "Blank area is set in ... 2D control register
// OOWC".  WSI = 0 makes the whole screen window fill, a WBA outside the display
// area does the same, and a WBA + WSI past the edge simply clips.
//
// The window bounds are NINE bits: an 8-bit truncating add lets WBA.H = 1 /
// WSI.H = 0xFF wrap below its own origin and blank the entire frame, and WSI.V's
// reset value is 0xFF (K2GEres), which several titles write literally.  For
// every sum below 256 the nine-bit compare is bit-identical to an eight-bit one,
// and a window past the panel edge is clipped by the 160-dot / 152-row active
// area.
//
// The bounds are deliberately NOT precomputed at the per-line latch.  The window
// registers are latched in k2ge_mmr and both latch strobes land on the very edge
// that also starts dot 0, so a precomputed copy would be one dot stale for
// exactly the first pixel of the line -- the pixel the clip matters most for.
// The compare is done live and registered into `inwin_q` once per tick: the path
// is (k2ge_mmr flop) -> 8-bit add -> compare -> AND -> flop, about 4 ns of the
// 20.3 ns period on a Cyclone V -7.
//
// NEG "affects the RGB signal outputted by the K2GE" (K2GE 4-11): a bitwise NOT
// of the 12 bits after the palette, applied to every dot the chip drives inside
// the active area, background and window fill included.  Blanking is forced to
// black and is NOT inverted, because a real panel is not driven during blanking
// and inverting it would paint the framework's blanking white.
//
// Outside raster lines 1..152 (which carry display rows 0..151; see the
// `disp_row` note in the body) or the 160 active dots, `lcd_de` is low and RGB
// is forced to zero, so blanks cover all of the non-drawn area.
//
// Not owned here
//   the palette RAM and the mono LUT   k2ge_palette, a sibling; this module
//                                      drives `pal_form` / `pal_entry` /
//                                      `pal_in_win` and consumes pal_r/g/b
//   the line buffer                    k2ge_linebuf; this module drives a read
//                                      per active dot and expects the entry one
//                                      clk_sys cycle later, exactly like a block
//                                      RAM.  k2ge_linebuf clears as it is read
//                                      and that clear is driven by these reads,
//                                      so a read is issued for all 160 active
//                                      dots whether or not the window shows them
//   the raster counters                k2ge_vtimer; `hdot`, `vline` and
//                                      `line_start` come in
//   the framework video bus            ngpc_crt_framebuffer
//
// `ce` is the chip's 49.152 MHz enable, tied high in normal operation.  Dropping
// it freezes the scanout and stops `lcd_dclk_ce`, which is why
// ngpc_crt_framebuffer free-runs its own raster: the framework must keep seeing
// syncs while the machine is paused for a savestate or sitting in standby.
//
// This module claims bits 47:0 of internals word 13 and reads 0 elsewhere,
// k2ge_palette claims 63:48, and the parent ORs the two images.  The registers
// that are not in the image -- the five-stage strobe shift, the pipelined
// framing copies and the line-buffer address -- are all quiescent at a line
// boundary, the only place the pause rule allows a save or restore, and they
// re-derive within one tick.

module k2ge_scanout
#(
	// Horizontal sync window, in dots, on the raster counter.  Both ends are
	// multiples of 3 so both edges land on a CE tick, and both sit inside the
	// horizontal blank.
	parameter [9:0] P_HS_START = 10'd489,
	parameter [9:0] P_HS_END   = 10'd507,   // exclusive
	// Vertical sync window, in lines, inside the V blank (lines 152..198).
	parameter [7:0] P_VS_START = 8'd168,
	parameter [7:0] P_VS_END   = 8'd171,    // exclusive
	// EXPERIMENT (agents/CHANGES.md 2026-08-24, burst-window model).  1 = the
	// 160 active pixels are resolved as a ONE-DOT-PER-PIXEL burst over dots
	// P_BURST_START .. +159 instead of every third dot over 0..477, so pixel x
	// sees register/palette state as of dot P_BURST_START + x.  The twelve
	// blanking ticks follow at the normal /3 pitch.  Framing that is anchored
	// to `hdot` (HS, LP, SP) is re-anchored to the tick index in this mode,
	// because the sync window at dots 489..506 contains no burst tick.
	// NOT a hardware claim.  0 = shipped behaviour, bit-identical.
`ifdef NGPC_BURST_PIXEL
	parameter       P_BURST_PIXEL = 1'b1,
`else
	parameter       P_BURST_PIXEL = 1'b0,
`endif
`ifdef NGPC_BURST_START
	parameter [9:0] P_BURST_START = `NGPC_BURST_START,
`else
	parameter [9:0] P_BURST_START = 10'd60,
`endif
	// P_DOT_BURST -- the SPLIT form of the same model, and the one meant for
	// hardware.  The resolve engine walks pixels 0..159 through the line
	// buffer and palette during dots P_BURST_START .. +159, one pixel per dot,
	// and writes finished 12-bit colours into `u_dotbuf`.  The emit engine
	// keeps this module's original /3 cadence and `hdot`-anchored framing and
	// shifts those colours out of the buffer, so CE_PIXEL, DE, HS, LP, SP and
	// everything downstream of ngpc_crt_framebuffer are unchanged.  Emission
	// therefore runs one raster line behind the resolve, which moves the
	// active window from lines 1..152 to lines 2..153; the picture is
	// identical, it starts one line later inside the 199-line frame.
	// Supersedes P_BURST_PIXEL, which resolves and emits on the same tick and
	// so drags the panel framing with it.  P_DOT_BURST wins if both are set.
`ifdef NGPC_DOT_BURST
	parameter       P_DOT_BURST = 1'b1
`else
	parameter       P_DOT_BURST = 1'b0
`endif
)
(
	input  wire        clk_sys,       // 49.152 MHz
	input  wire        ce,            // chip enable, tie 1 (see header)
	input  wire        ce_6m144,      // dot-clock enable, 1 of 8
	input  wire        rst,           // synchronous power-on reset

	// --- raster position (k2ge_vtimer) --------------------------------------
	input  wire [9:0]  hdot,          // 0..514
	input  wire [7:0]  vline,         // 0..REF
	input  wire        line_start,    // entering dot 0 of any line

	// --- per-frame latched window registers (k2ge_mmr) ----------------------
	input  wire [7:0]  win_wba_h,
	input  wire [7:0]  win_wba_v,
	input  wire [7:0]  win_wsi_h,
	input  wire [7:0]  win_wsi_v,

	// --- per-line latched registers (k2ge_mmr) ------------------------------
	// Only NEG lands here.  OOWC, BGC and BGON are colour codes, so they go
	// straight from k2ge_mmr to k2ge_palette: this module decides WHICH source
	// a dot comes from, and the palette turns that decision into an address.
	input  wire        ln_neg,        // 0x8012 D7

	// --- line buffer read port (k2ge_linebuf) -------------------------------
	output wire        lb_rd_en,      // one active dot, read-and-clear
	output wire [7:0]  lb_rd_x,       // 0..159
	input  wire [8:0]  lb_q,          // entry, one clk_sys cycle after the read

	// --- colour lookup (k2ge_palette, sibling) ------------------------------
	output wire        pal_form,      // P1 strobe
	output wire [8:0]  pal_entry,     // the line-buffer entry for this dot
	output wire        pal_in_win,    // this dot is inside the window
	input  wire [3:0]  pal_r,
	input  wire [3:0]  pal_g,
	input  wire [3:0]  pal_b,

	// --- LCD panel port (pad names from the NPK2 board) ---------------------
	output wire [3:0]  lcd_r,
	output wire [3:0]  lcd_g,
	output wire [3:0]  lcd_b,
	output wire        lcd_dclk_ce,   // /3, restarted every line
	output wire        lcd_de,        // the 160 x 152 active area
	output wire        lcd_hs,        // active high
	output wire        lcd_vs,        // active high
	output wire        lcd_lp,        // line pulse, one per line
	output wire        lcd_sp,        // frame start pulse, one per frame

	// --- savestate internals, local word 13 bits 47:0 (see header) ---------
	output wire [47:0] ss_state,
	input  wire [47:0] ss_wdata,
	input  wire        ss_wren
);

	// Fixed geometry (K1GE p.4 / K2GE p.5; K1GE p.17 / K2GE p.22)

	localparam [7:0] V_ACTIVE = 8'd152;    // 152 rows; shown on lines 1..152
	localparam [7:0] H_ACTIVE = 8'd160;    // 160 dots per line

	// The divide-by-3 dot enable, restarted at every line start
	// `line_start` is asserted on the clk_sys cycle whose rising edge ENTERS
	// dot 0 (k2ge_vtimer's strobe convention), so clearing the phase on it
	// makes the phase read 0 during dot 0 - which is where the first tick of
	// the line belongs.  The phase is then simply hdot mod 3 for the whole
	// line, and it cannot drift, because it is re-armed every line rather
	// than divided out of a free-running counter.

	reg [1:0] ph_q;

	always @(posedge clk_sys) begin
		if (rst) begin
			ph_q <= 2'd0;
		end else if (ce) begin
			if (line_start)    ph_q <= 2'd0;
			else if (ce_6m144) ph_q <= (ph_q == 2'd2) ? 2'd0 : (ph_q + 2'd1);
		end
	end

	wire dclk_norm = ce && ce_6m144 && (ph_q == 2'd0);

	// Burst mode (P_BURST_PIXEL): 160 ticks at one per dot, then the twelve
	// blanking ticks at the /3 pitch.  `bph_q` only advances inside the
	// post-burst span, so its first dot always carries a tick.

	wire in_burst   = (hdot >= P_BURST_START)
	               && (hdot <  P_BURST_START + 10'd160);
	wire post_burst = (hdot >= P_BURST_START + 10'd160)
	               && (hdot <  P_BURST_START + 10'd194);

	reg [1:0] bph_q;

	always @(posedge clk_sys) begin
		if (rst) begin
			bph_q <= 2'd0;
		end else if (ce) begin
			if (line_start)                    bph_q <= 2'd0;
			else if (ce_6m144 && post_burst)   bph_q <= (bph_q == 2'd2) ? 2'd0
			                                                           : (bph_q + 2'd1);
		end
	end

	wire dclk_burst = ce && ce_6m144
	               && (in_burst || (post_burst && (bph_q == 2'd0)));

	// Resolve tick: 160 ticks, one per dot, only inside the burst.
	wire rdclk = ce && ce_6m144 && in_burst;

	// The tick that drives the render pipeline, and the tick that drives the
	// pads.  In split mode they are different signals; in the other two modes
	// they are the same one, which is what the original design assumed.
	wire dclk  = (P_DOT_BURST   != 0) ? rdclk
	           : (P_BURST_PIXEL != 0) ? dclk_burst : dclk_norm;
	wire edclk = (P_DOT_BURST   != 0) ? dclk_norm : dclk;

	// Pixel index inside the line
	// 0..171: 0..159 are the active dots, 160..171 the twelve blanking ticks.
	// Dot 514 has phase 1, so a tick can never coincide with `line_start` and
	// the two branches below cannot fight.

	reg [7:0] px_q;

	always @(posedge clk_sys) begin
		if (rst)             px_q <= 8'd0;
		else if (ss_wren)    px_q <= ss_wdata[19:12];
		else if (ce) begin
			if (line_start)  px_q <= 8'd0;
			else if (dclk)   px_q <= px_q + 8'd1;
		end
	end

	// Emit pixel index (split mode only)
	// `px_q` above counts the RESOLVE walk, which in split mode runs only
	// inside the burst.  The pads still need the original 0..171 walk, so it
	// gets its own counter on `edclk`.  In the other two modes nothing reads
	// this and Quartus removes it.

	reg [7:0] epx_q;

	always @(posedge clk_sys) begin
		if (rst)            epx_q <= 8'd0;
		else if (ss_wren)   epx_q <= 8'd0;
		else if (ce) begin
			if (line_start) epx_q <= 8'd0;
			else if (edclk) epx_q <= epx_q + 8'd1;
		end
	end

	// Window clip
	// NINE-bit adders, so a window whose origin plus size reaches 256 runs off
	// the panel edge and is clipped by the active area instead of wrapping
	// below its own origin and blanking the frame.  See the header for the
	// full argument and for why the compare is live rather than precomputed.

	wire [8:0] win_x1 = {1'b0, win_wba_h} + {1'b0, win_wsi_h};
	wire [8:0] win_y1 = {1'b0, win_wba_v} + {1'b0, win_wsi_v};

	// Which display row is on the panel during raster line `vline`
	// The composition pass for display row R runs during the Hardware Drawing
	// Period of raster line R (K1GE p.25 / K2GE p.35 - "the signal
	// generation begins 1 H before the Hardware Drawing Period starts ...
	// signal generation for the 0th line occurs at the beginning of line
	// 198"), so row R is finished at the end of line R and is shifted out to
	// the panel during raster line R + 1.  That is what a double-buffered line
	// memory with a line-latch pulse physically does, and it is why the active
	// area is raster lines 1..152 rather than 0..151.  Still 152 lines; the
	// framework sees no difference, and every K2GE-visible counter (RAS.V,
	// Vint, BLNK) keeps naming the line being COMPOSED, as the documents do.
	//
	// The window registers clip DISPLAY ROWS, not raster lines, so the
	// vertical compare uses `disp_row`.  See k2ge_vtimer's header for the
	// full argument.

	// The pass on raster line L composes row L; the double-buffered row is
	// scanned out one line later.  Keep this panel phase independent of the
	// CPU-visible RAS.V value.
	wire [7:0] disp_row = vline - 8'd1;

	wire in_win_x = (px_q     >= win_wba_h) && ({1'b0, px_q}     < win_x1);
	wire in_win_y = (disp_row >= win_wba_v) && ({1'b0, disp_row} < win_y1);
	wire in_win   = in_win_x && in_win_y;

	// Raw framing for the dot at the current tick

	wire de_now = (px_q < H_ACTIVE) && (vline >= 8'd1) && (vline <= V_ACTIVE);
	// In burst mode the dot-anchored framing is re-anchored to the tick index:
	// HS covers the same six ticks it covers normally (489/3 = 163 through
	// 506/3), and LP/SP move to the line's first tick.
	wire cheap_only = (P_BURST_PIXEL != 0) && (P_DOT_BURST == 0);
	wire hs_now = cheap_only
	            ? ((px_q >= 8'd163) && (px_q < 8'd169))
	            : ((hdot >= P_HS_START) && (hdot < P_HS_END));
	wire vs_now = (vline >= P_VS_START) && (vline < P_VS_END);
	wire lp_now = cheap_only ? (px_q == 8'd0) : (hdot == 10'd0);
	wire sp_now = lp_now && (vline == 8'd0);

	// Emission DE (split mode)
	// The resolve for display row R runs during raster line R + 1 and the
	// emission one line after that, so the active window moves from raster
	// lines 1..152 to 2..153.  Same 152 rows, one line later in the frame.
	wire ede_now = (epx_q < H_ACTIVE)
	            && (vline >= 8'd2) && (vline <= (V_ACTIVE + 8'd1));

	// The DE that the output stage uses, and the tick it uses it on.
	wire de_emit = (P_DOT_BURST != 0) ? ede_now : de_now;

	// Micro-pipeline strobes
	// One shift register, one cycle per stage, all gated by `ce` so a freeze
	// stops the pipeline where it stands instead of finishing with stale data.

	reg [4:0] st_q;

	always @(posedge clk_sys) begin
		if (rst)      st_q <= 5'd0;
		else if (ce)  st_q <= {st_q[3:0], dclk};
	end

	wire st_addr = st_q[0];    // +1: the line-buffer address is presented
	wire st_ent  = st_q[1];    // +2: the entry is on lb_q
	wire st_form = st_q[2];    // +3: form and register the palette address
	wire st_out  = st_q[4];    // +5: the palette entry is on pal_r/g/b

	// Per-dot capture

	reg [7:0] lbx_q;
	reg       lbrd_q;
	reg       inwin_q;
	reg [8:0] ent_q;

	always @(posedge clk_sys) begin
		if (rst) begin
			lbx_q   <= 8'd0;
			lbrd_q  <= 1'b0;
			inwin_q <= 1'b0;
			ent_q   <= 9'd0;
		end else if (ss_wren) begin
			lbx_q   <= 8'd0;
			lbrd_q  <= 1'b0;
			inwin_q <= ss_wdata[20];
			ent_q   <= ss_wdata[29:21];
		end else if (ce) begin
			if (dclk) begin
				lbx_q   <= px_q;
				lbrd_q  <= de_now;
				inwin_q <= in_win;
			end
			if (st_ent) ent_q <= lb_q;
		end
	end

	assign lb_rd_en   = st_addr && lbrd_q;
	assign lb_rd_x    = lbx_q;

	assign pal_form   = st_form;
	assign pal_entry  = ent_q;
	assign pal_in_win = inwin_q;

	// Framing pipeline and the output stage
	// `*_pipe_q` describes the dot whose colour is being computed right now;
	// `*_q` describes the dot on the output pads.  They are one CE tick apart,
	// which is exactly the pipeline delay, so colour and framing always agree.

	reg        de_pipe_q, hs_pipe_q, vs_pipe_q, lp_pipe_q, sp_pipe_q;
	reg [11:0] rgb_pipe_q;
	reg        de_q, hs_q, vs_q, lp_q, sp_q;
	reg [11:0] rgb_q;

	// NEG is a bitwise NOT of the 12 bits the palette produced; blanking is
	// black and is not inverted (see the header).
	wire [11:0] pal_rgb  = {pal_b, pal_g, pal_r};
	wire [11:0] dot_rgb  = ln_neg ? ~pal_rgb : pal_rgb;
	wire [11:0] next_rgb = de_pipe_q ? dot_rgb : 12'h000;

	// The dot buffer (split mode)
	// One 512 x 16 block, two 160-entry halves selected by `vline[0]`, holding
	// finished 12-bit colours.  Double-buffered because the resolve and the
	// emission overlap in time: inside line V the emission is reading slots
	// 0..159 at one per three dots while the resolve is writing slots 0..159
	// at one per dot, and from dot P_BURST_START + 90 onwards the write
	// pointer is ahead of the read pointer.  Not in the savestate image: the
	// buffer is rebuilt every line, so a restore costs one emitted line of
	// stale colour and then self-corrects, which is the same bargain
	// k2ge_palette's `bg_line_q` takes.
	//
	// `rpx_lat_q` / `rde_lat_q` hold the resolve pixel's index and validity
	// from its tick until its colour arrives five cycles later.  Safe because
	// resolve ticks are eight clk_sys cycles apart and the pipeline is five
	// deep, and because the burst sits well inside the line so `vline` cannot
	// move between the two.

	reg [7:0] rpx_lat_q;
	reg       rde_lat_q;

	always @(posedge clk_sys) begin
		if (rst) begin
			rpx_lat_q <= 8'd0;
			rde_lat_q <= 1'b0;
		end else if (ce && rdclk) begin
			rpx_lat_q <= px_q;
			rde_lat_q <= de_now;
		end
	end

	wire        db_wren  = (P_DOT_BURST != 0) && st_out;
	wire [8:0]  db_waddr = {vline[0], rpx_lat_q};
	wire [15:0] db_wdata = {4'd0, (rde_lat_q ? dot_rgb : 12'h000)};

	// Read side: address registered on the emit tick, so the block answers two
	// cycles later.  `eb_str_q[1]` is that cycle.

	reg [8:0] db_raddr_q;
	reg [1:0] eb_str_q;

	always @(posedge clk_sys) begin
		if (rst) begin
			db_raddr_q <= 9'd0;
			eb_str_q   <= 2'd0;
		end else if (ce) begin
			eb_str_q <= {eb_str_q[0], edclk};
			if (edclk) db_raddr_q <= {~vline[0], epx_q};
		end
	end

	wire [15:0] db_q;
	wire [15:0] db_q_a;    // port A is write-only here; see `unused_ok`

	cache_ram_dp_be #(
		.ADDR_WIDTH (9),
		.DATA_WIDTH (16)
	) u_dotbuf (
		.clk_i     (clk_sys),
		.addr_a_i  (db_waddr),
		.wren_a_i  (db_wren),
		.be_a_i    (2'b11),
		.wdata_a_i (db_wdata),
		.q_a_o     (db_q_a),
		.addr_b_i  (db_raddr_q),
		.wren_b_i  (1'b0),
		.be_b_i    (2'b11),
		.wdata_b_i (16'd0),
		.q_b_o     (db_q)
	);

	always @(posedge clk_sys) begin
		if (rst) begin
			de_pipe_q  <= 1'b0;
			hs_pipe_q  <= 1'b0;
			vs_pipe_q  <= 1'b0;
			lp_pipe_q  <= 1'b0;
			sp_pipe_q  <= 1'b0;
			rgb_pipe_q <= 12'h000;
			de_q       <= 1'b0;
			hs_q       <= 1'b0;
			vs_q       <= 1'b0;
			lp_q       <= 1'b0;
			sp_q       <= 1'b0;
			rgb_q      <= 12'h000;
		end else if (ss_wren) begin
			de_pipe_q  <= 1'b0;
			hs_pipe_q  <= 1'b0;
			vs_pipe_q  <= 1'b0;
			lp_pipe_q  <= 1'b0;
			sp_pipe_q  <= 1'b0;
			rgb_pipe_q <= ss_wdata[41:30];
			de_q       <= ss_wdata[42];
			hs_q       <= ss_wdata[43];
			vs_q       <= ss_wdata[44];
			lp_q       <= 1'b0;
			sp_q       <= 1'b0;
			rgb_q      <= ss_wdata[11:0];
		end else if (ce) begin
			if (edclk) begin
				// emit the dot the pipeline just finished ...
				rgb_q     <= rgb_pipe_q;
				de_q      <= de_pipe_q;
				hs_q      <= hs_pipe_q;
				vs_q      <= vs_pipe_q;
				lp_q      <= lp_pipe_q;
				sp_q      <= sp_pipe_q;
				// ... and start the dot at this tick
				de_pipe_q <= de_emit;
				hs_pipe_q <= hs_now;
				vs_pipe_q <= vs_now;
				lp_pipe_q <= lp_now;
				sp_pipe_q <= sp_now;
			end
			// Split mode takes the colour out of the dot buffer; the other two
			// take it straight off the palette, as the original design did.
			if (P_DOT_BURST != 0) begin
				if (eb_str_q[1])
					rgb_pipe_q <= de_pipe_q ? db_q[11:0] : 12'h000;
			end else begin
				if (st_out) rgb_pipe_q <= next_rgb;
			end
		end
	end

	assign lcd_r       = rgb_q[3:0];
	assign lcd_g       = rgb_q[7:4];
	assign lcd_b       = rgb_q[11:8];
	assign lcd_de      = de_q;
	assign lcd_hs      = hs_q;
	assign lcd_vs      = vs_q;
	assign lcd_lp      = lp_q;
	assign lcd_sp      = sp_q;
	assign lcd_dclk_ce = edclk;

	// Savestate image

	// Layout, which the chip parent must mirror when it merges this image with
	// k2ge_palette's bits 63:48:
	//
	//   [11:0]  rgb_q        the colour on the pads
	//   [19:12] px_q         pixel index inside the line
	//   [20]    inwin_q
	//   [29:21] ent_q        the line-buffer entry in flight
	//   [41:30] rgb_pipe_q   the colour the pipeline just finished
	//   [42]    de_q
	//   [43]    hs_q
	//   [44]    vs_q
	//   [47:45] spare, read 0

	assign ss_state = {3'd0, vs_q, hs_q, de_q, rgb_pipe_q, ent_q, inwin_q,
	                   px_q, rgb_q};

	// Deliberately unread inputs
	// st_q[3] is the palette RAM's own address cycle, which nothing in this
	// module has to act on, and the top three savestate bits are spare.
	// `db_q_a` is the write-only port's readback and `db_q[15:12]` are the four
	// bits above a 12-bit colour in a 16-bit word; neither has a reader.
	wire unused_ok = &{1'b0, st_q[3], ss_wdata[47:45], db_q_a, db_q[15:12],
	                   1'b0};

endmodule
