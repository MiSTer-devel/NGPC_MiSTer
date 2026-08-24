// Copyright (c) 2026 Jamie Blanks

// k2ge -- the SNK K2GE graphics engine at the pins it shows the rest of the
// K2-CHIP.  Assembly only: every behaviour lives in a submodule.  This level
// owns the wiring, the CPU read mux, the savestate word map and the pause AND.
//
//   k2ge_vtimer   515 x 199 raster, RAS.H/RAS.V, BLNK, Vint, 152 Hints
//   k2ge_mmr      0x8000-0x81FF and 0x8400-0x87FF, the three latch groups
//   k2ge_vram     character / scroll / sprite / palette RAM, CP.C, the
//                 four-slot character arbiter, the flat savestate tap
//   k2ge_scroll   x2, one per plane
//   k2ge_obj      the 64-sprite chain evaluator and the 8-dot drawer
//   k2ge_linebuf  the double 160 x 9 composition buffer and its rank compare
//   k2ge_palette  the mono LUT, the palette address formation, the mono bypass
//   k2ge_scanout  window clip, NEG, the /3 panel enable and the LCD framing
//
// mono_strap forces MODE to 1 on the display path (`mode_compat_eff`).  That is
// more than a palette bypass: the K1GE has no CP.C, so the scroll planes and
// the sprite engine take their four-bit code from P.C.  0x87E2 was added to the
// K1GE (K2GE 5-1 item 4) and sits inside the K1GE "open area" (K1GE 3-2), so on
// a mono build k2ge_mmr drops the write and reads 0x00, and k2ge_vram stops
// claiming colour palette RAM (0x8200-0x83FF) and the sprite CP.C nibbles
// (0x8C00-0x8C3F).  Nothing drawn passes through that register.
//
// CPU read mux: k2ge_mmr and k2ge_vram each report whether they own the
// address, and the mux is a priority select on k2ge_mmr's hit rather than a
// wired OR.  An address nobody owns reads 0x0000 (K1GE 3-2; K2GE 4-2).  There
// is no wait or ready output and there must never be one: SNK specifies
// zero-wait access to every K2GE region (SysPro p.6-7; K2GE 4-15 caution 5).
//
// Pause holds the internal ce and ce_6m144 low once pause_ready is reported.
// The submodules' savestate write paths are deliberately not ce-gated, so the
// walker still runs while the chip is frozen; lcd_dclk_ce drops, which is why
// the board-level CRT presenter free-runs its own raster.  pause_ready is the
// AND of k2ge_vtimer (ready only at the native frame boundary, which the
// presentation store needs) and k2ge_mmr (line boundary), so the worst-case
// pause latency is one native frame.
//
// Savestate word map -- local words 0..23 of the internals slice (global
// 64..87).  Words 0..8, 15 and 16 come from k2ge_mmr and k2ge_vtimer, which
// drive disjoint bits of word 0 and read 0 elsewhere, so those two images merge
// with a plain OR.  The render and scanout words are muxed:
//
//   word  0  mmr 63:53 | this level: [5] displayed line-buffer half,
//            [4] mono_strap, [3] lb_init | vtimer 2:0
//   word  1-5, 7, 8    k2ge_mmr (live registers, LED, latched copies, window)
//   word  6            k2ge_vtimer (hdot, vline, rash)
//   word  9, 17        k2ge_scroll plane 1, ss_state[127:64] then [63:0]
//   word 10, 18        k2ge_scroll plane 2
//   word 11, 12, 19    k2ge_obj: evaluator, fetch/skid, drawer
//   word 13            [63:48] k2ge_palette, [47:0] k2ge_scanout
//   word 14            contention hooks, tied off: reads 0, restore is a no-op
//   word 15, 16        k2ge_mmr (the undocumented register files)
//   word 20            display-palette sync controller
//   word 21-23         spare, read 0
//
// k2ge_scroll takes its whole 128-bit image on one strobe and k2ge_obj its
// 192-bit image, but the walker delivers one 64-bit word per cycle, so the
// earlier words are held here and the write applied on the last word of each
// group (17, 18, 19).  That relies on savestates.sv walking BUS_Adr upwards
// from 0.  The flat memory tap (TYPE2, 16384 bytes) goes into k2ge_vram, which
// owns the layout and forwards the line-buffer window to k2ge_linebuf.

module k2ge
#(
	// --- raster and timing parameters --------------------------------------
	// Last dot of the Hardware Drawing Period.  484 = 515 - 31, from the
	// documented ~5 us H time remaining (K1GE section 4).
	parameter [9:0] P_HDRAW_END      = 10'd484,
	// How far into the line the composition pass keeps the CPU off
	// char/scroll/sprite VRAM. Separate from P_HDRAW_END on purpose: that
	// one also times the MMR latch, and the VRAM hold is wider than the
	// drawing period (measured ~1.96 states/access vs the 1.88 implied by
	// 484/515).
	parameter [9:0] P_VRAM_HOLD_END  = 10'd515,
	// Dot at which the Hint pin falls.
	parameter [9:0] P_HINT_FALL      = 10'd480,
	// RAS.H down-counter reload.
	parameter [9:0] P_RASH_RELOAD    = 10'd514,
	// Window latch timing.  Metal Slug 1/2 update WBA.H and
	// WSI.H after raster line 128 to draw the lower status bar, so horizontal
	// fields latch per line.  Vertical fields stay per frame because they derive
	// Vint and no game evidence requires moving that timing mid-frame.
	parameter       P_WINDOW_H_LATCH = 1'b1,
	parameter       P_WINDOW_V_LATCH = 1'b0,
	// Composition-side per-line latch phase.  0 = at the start of
	// the pass, 1 = at the end of the previous drawing period.
	parameter       P_RENDER_LATCH   = 1'b0,
	// Palette byte writes: 1 = write the addressed lane, 0 = ignore.
	parameter       P_PAL_BYTE_WRITE = 1'b0,
	// BG colour reset value.  K2GE documents 0x00 (K2GE 4-6); the NGP register
	// table prints 0x07 ("Background Color BG, 0x8118 ... Initial Value 07"),
	// which is BGON invalid with BGC = 7.  The two documents disagree, so the
	// split stays a parameter pair rather than one constant.
	parameter [7:0] P_BGC_RESET      = 8'h00,
	parameter [7:0] P_MONO_BGC_RESET = 8'h07,
	// Mono bypass: 1 = background and window codes take the same 3-bit
	// gradation ramp as the character LUT, 0 = they are black.
	parameter       P_MONO_BG_RAMP   = 1'b1
)
(
	input  wire        clk_sys,       // 49.152 MHz
	input  wire        ce_6m144,      // dot-clock enable, 1 of 8
	input  wire        rst,           // synchronous power-on reset
	input  wire        main_clk_run,  // SoC standby gate for fast local helpers
	input  wire        palette_frame_boundary, // raw OSD request, frame-latched

	// ---- CPU bus face.  Palette RAM is zero-wait; character, scroll and
	// sprite VRAM cost the CPU two wait states during the drawing period --
	// see the CPU access rules in k2ge_vram.sv, which this pin implements. ----
	input  wire [13:0] cpu_a,         // byte address inside 0x8000-0xBFFF
	input  wire [15:0] cpu_din,
	output wire [15:0] cpu_dout,
	input  wire        cpu_cs,        // window select from the SoC decoder
	input  wire        cpu_rd,        // read strobe
	input  wire [1:0]  cpu_we,        // [0] = even byte, [1] = odd byte
	output wire [2:0]  cpu_wait_states, // VRAM contention wait, 0 or 2

	// ---- interrupt pins ----------------------------------------------------
	output wire        vint,          // -> TLCS INT4  (level, gated by VI.E)
	output wire        hint,          // documented 152-pulse external HINT
	output wire        ti0,           // MCU Timer0 alias of HINT

	// ---- LCD panel port (pad names from the NPK2 board) -------------------
	output wire [3:0]  lcd_r,
	output wire [3:0]  lcd_g,
	output wire [3:0]  lcd_b,
	output wire        lcd_dclk_ce,   // panel dot-clock enable, /3 per line
	output wire        lcd_de,        // the 160 x 152 active area
	output wire        lcd_hs,        // active high
	output wire        lcd_vs,        // active high
	output wire        lcd_lp,        // line pulse, one per line
	output wire        lcd_sp,        // frame start pulse, one per frame

	// ---- misc chip pads ----------------------------------------------------
	output wire        led,           // LED drive pad (0x8400/0x8402)
	input  wire        inp0,          // reserved input pad, read at 0x87FE D6
	input  wire        mono_strap,    // 1 = NGP (mono) system build

	// ---- savestate internals slice, local words 0..23 ---------------------
	input  wire [4:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata,

	// ---- savestate flat memory tap (TYPE2, 16384 bytes) -------------------
	input  wire        ss_mem_active,
	input  wire [13:0] ss_mem_addr,
	input  wire [7:0]  ss_mem_wdata,
	input  wire        ss_mem_wren,
	input  wire        ss_mem_rden,
	output wire [7:0]  ss_mem_rdata,
	input  wire        restore_hold,

	// ---- pause / drain handshake ------------------------------------------
	input  wire        pause_req,
	output wire        pause_ready
);

	// Freeze and the internal clock enables

	wire pr_timer;
	wire pr_mmr;

	wire pause_rdy_w = pr_timer & pr_mmr;
	wire frozen      = restore_hold || (pause_req & pause_rdy_w);

	wire ce_int = ~frozen;                     // the submodules' `ce`
	wire ce_dot = ce_6m144 & ~frozen;          // the dot enable they see

	assign pause_ready = pause_rdy_w;

	// Raster timer <-> register file

	wire [9:0] hdot;
	wire [7:0] vline;
	wire [7:0] ras_h;
	wire [7:0] ras_v;
	wire       blnk;
	wire       covr;
	wire       line_start;
	wire       frame_start;
	wire       hdraw_end;
	wire       pass_start;
	wire [7:0] pass_line;

	wire       vi_e;
	wire       hi_e;
	wire [7:0] ref_last;
	wire       soft_reset;

	// Latched and immediate register exports

	wire [7:0] win_wba_h, win_wba_v, win_wsi_h, win_wsi_v;
	wire [7:0] ln_po_h, ln_po_v;
	wire [7:0] ln_s1so_h, ln_s1so_v, ln_s2so_h, ln_s2so_v;
	wire       ln_p_f;
	wire       sc_p_f;
	wire [53:0] sc_mono_lut;
	wire       ln_neg;
	wire [2:0] ln_oowc;
	wire [1:0] ln_bgon;
	wire [2:0] ln_bgc;
	wire       mode_compat;

	// Which line-buffer half is displayed
	// 199 lines is odd, so vline[0] does not alternate across the frame wrap;
	// the displayed half is a toggle flipped at every line start.
	reg buf_half_q;

	always @(posedge clk_sys) begin
		if (rst)             buf_half_q <= 1'b0;
		else if (w0_wren)    buf_half_q <= ss_wdata[5];
		else if (line_start) buf_half_q <= ~buf_half_q;
	end

	// Mono LUT endianness
	// k2ge_mmr packs the LUT index-0-first from the top of the vector;
	// k2ge_palette indexes from the bottom.  The reversal is done here at
	// the joint.
	wire [53:0] mono_lut_lsb;

	genvar gi;
	generate
		for (gi = 0; gi < 18; gi = gi + 1) begin : g_lut_endian
			assign mono_lut_lsb[gi*3 +: 3] = sc_mono_lut[53 - gi*3 -: 3];
		end
	endgenerate

	// A mono system has no CP.C and no K2GE palette region, so the display
	// path is forced into K1GE upper-palette compatible mode.
	// k2ge_palette makes the same substitution for itself from `mono_strap`;
	// the planes and the sprite engine need it made for them.
	wire mode_compat_eff = mode_compat | mono_strap;

	// P.F = 0 puts plane 1 in front (K1GE 3-4-8); the front plane ranks 4
	// and the back plane 2.  These decide ranks during the composition pass,
	// so they take the composition-side copy; k2ge_palette turns the same rank
	// back into a layer a line later and takes `sc_p_f` instead.
	wire [2:0] s1_rank = ln_p_f ? 3'd2 : 3'd4;
	wire [2:0] s2_rank = ln_p_f ? 3'd4 : 3'd2;

	// CPU read mux

	wire [15:0] mmr_dout;
	wire        mmr_hit;
	wire [15:0] vram_dout;

	assign cpu_dout = mmr_hit ? mmr_dout : vram_dout;

	// Savestate plumbing

	wire [63:0]  ss_rd_timer;
	wire [63:0]  ss_rd_mmr;
	wire [127:0] s1_ss_state;
	wire [127:0] s2_ss_state;
	wire [191:0] ob_ss_state;
	wire [15:0]  pal_ss_state;
	wire [47:0]  sc_ss_state;
	wire         lb_init;

	// Honoured only while paused, matching what k2ge_mmr and k2ge_vtimer
	// already do inside their own slices.
	wire ss_wr = ss_wren && pause_req;

	// The earlier words of a multi-word group, held until the group's last
	// word arrives.
	reg [63:0] s1_hi_q;
	reg [63:0] s2_hi_q;
	reg [63:0] ob_w11_q;
	reg [63:0] ob_w12_q;

	always @(posedge clk_sys) begin
		if (rst) begin
			s1_hi_q  <= 64'd0;
			s2_hi_q  <= 64'd0;
			ob_w11_q <= 64'd0;
			ob_w12_q <= 64'd0;
		end else if (ss_wr) begin
			case (ss_addr)
				5'd9:    s1_hi_q  <= ss_wdata;
				5'd10:   s2_hi_q  <= ss_wdata;
				5'd11:   ob_w11_q <= ss_wdata;
				5'd12:   ob_w12_q <= ss_wdata;
				default: ;
			endcase
		end
	end

	wire s1_ss_wren = ss_wr && (ss_addr == 5'd17);
	wire s2_ss_wren = ss_wr && (ss_addr == 5'd18);
	wire ob_ss_wren = ss_wr && (ss_addr == 5'd19);
	wire p13_wren   = ss_wr && (ss_addr == 5'd13);
	wire w0_wren    = ss_wr && (ss_addr == 5'd0);
	wire w20_wren   = ss_wr && (ss_addr == 5'd20);

	wire [127:0] s1_ss_wdata = {s1_hi_q, ss_wdata};
	wire [127:0] s2_ss_wdata = {s2_hi_q, ss_wdata};
	wire [191:0] ob_ss_wdata = {ob_w11_q, ob_w12_q, ss_wdata};
	wire [11:0]  pal_ctrl_ss_state;

	// Words 0..8, 15 and 16 are the OR of two disjoint images plus the two
	// bits this level owns; everything else is a mux. Word 14 and words 21..23
	// fall through to the default and read 0, which is what the contention
	// hooks and the remaining spares are supposed to do.
	reg [63:0] ss_rd_r;

	always @* begin
		case (ss_addr)
			5'd0:    ss_rd_r = ss_rd_mmr | ss_rd_timer
			                   | {58'd0, buf_half_q, mono_strap, lb_init, 3'd0};
			5'd9:    ss_rd_r = s1_ss_state[127:64];
			5'd17:   ss_rd_r = s1_ss_state[63:0];
			5'd10:   ss_rd_r = s2_ss_state[127:64];
			5'd18:   ss_rd_r = s2_ss_state[63:0];
			5'd11:   ss_rd_r = ob_ss_state[191:128];
			5'd12:   ss_rd_r = ob_ss_state[127:64];
			5'd19:   ss_rd_r = ob_ss_state[63:0];
			5'd13:   ss_rd_r = {pal_ss_state, sc_ss_state};
			5'd20:   ss_rd_r = {52'd0, pal_ctrl_ss_state};
			default: ss_rd_r = ss_rd_mmr | ss_rd_timer;
		endcase
	end

	assign ss_rdata = ss_rd_r;

	// vtimer -- the raster
	// covr_set is tied 0: this build has no contention model, so C.OVR never
	// sets.

	k2ge_vtimer #(
		.P_HDRAW_END   (P_HDRAW_END),
		.P_HINT_FALL   (P_HINT_FALL),
		.P_RASH_RELOAD (P_RASH_RELOAD)
	) u_vtimer (
		.clk_sys     (clk_sys),
		.ce_6m144    (ce_dot),
		.rst         (rst),

		.vi_e        (vi_e),
		.hi_e        (hi_e),
		.ref_last    (ref_last),
		.soft_reset  (soft_reset),
		.covr_set    (1'b0),

		.hdot        (hdot),
		.vline       (vline),
		.ras_h       (ras_h),
		.ras_v       (ras_v),
		.blnk        (blnk),
		.covr        (covr),

		.line_start  (line_start),
		.frame_start (frame_start),
		.hdraw_end   (hdraw_end),
		.pass_start  (pass_start),
		.pass_line   (pass_line),

		.vint        (vint),
		.hint        (hint),
		.ti0         (ti0),

		.ss_addr     (ss_addr),
		.ss_wdata    (ss_wdata),
		.ss_wren     (ss_wren),
		.ss_rdata    (ss_rd_timer),

		.pause_req   (pause_req),
		.pause_ready (pr_timer)
	);

	// mmr -- the register file and the three latch groups

	k2ge_mmr #(
		.P_WINDOW_H_LATCH (P_WINDOW_H_LATCH),
		.P_WINDOW_V_LATCH (P_WINDOW_V_LATCH),
		.P_RENDER_LATCH   (P_RENDER_LATCH),
		.P_BGC_RESET      (P_BGC_RESET),
		.P_MONO_BGC_RESET (P_MONO_BGC_RESET)
	) u_mmr (
		.clk_sys     (clk_sys),
		.ce_6m144    (ce_dot),
		.rst         (rst),
		.mono_strap  (mono_strap),

		.cpu_a       (cpu_a),
		.cpu_din     (cpu_din),
		.cpu_dout    (mmr_dout),
		.cpu_hit     (mmr_hit),
		.cpu_cs      (cpu_cs),
		.cpu_rd      (cpu_rd),
		.cpu_we      (cpu_we),

		.ras_h       (ras_h),
		.ras_v       (ras_v),
		.blnk        (blnk),
		.covr        (covr),
		.line_start  (line_start),
		.frame_start (frame_start),
		.hdraw_end   (hdraw_end),

		.vi_e        (vi_e),
		.hi_e        (hi_e),
		.ref_last    (ref_last),
		.vint_line   (),
		.soft_reset  (soft_reset),

		.win_wba_h   (win_wba_h),
		.win_wba_v   (win_wba_v),
		.win_wsi_h   (win_wsi_h),
		.win_wsi_v   (win_wsi_v),
		.ln_po_h     (ln_po_h),
		.ln_po_v     (ln_po_v),
		.ln_s1so_h   (ln_s1so_h),
		.ln_s1so_v   (ln_s1so_v),
		.ln_s2so_h   (ln_s2so_h),
		.ln_s2so_v   (ln_s2so_v),
		.ln_p_f      (ln_p_f),
		.sc_p_f      (sc_p_f),
		.mono_lut    (),               // live view; the palette takes sc_mono_lut
		.sc_mono_lut (sc_mono_lut),
		.ln_neg      (ln_neg),
		.ln_oowc     (ln_oowc),
		.ln_bgon     (ln_bgon),
		.ln_bgc      (ln_bgc),

		.mode_compat (mode_compat),

		.inp0        (inp0),
		.led         (led),

		.ss_addr     (ss_addr),
		.ss_wdata    (ss_wdata),
		.ss_wren     (ss_wren),
		.ss_rdata    (ss_rd_mmr),

		.pause_req   (pause_req),
		.pause_ready (pr_mmr)
	);

	// vram -- every memory the renderer reads, and the character arbiter

	wire [9:0]  s1_map_addr;
	wire [15:0] s1_map_q;
	wire        s1_ch_req;
	wire [11:0] s1_ch_addr;
	wire        s1_ch_ack;
	wire        s1_ch_valid;
	wire [15:0] s1_ch_data;

	wire [9:0]  s2_map_addr;
	wire [15:0] s2_map_q;
	wire        s2_ch_req;
	wire [11:0] s2_ch_addr;
	wire        s2_ch_ack;
	wire        s2_ch_valid;
	wire [15:0] s2_ch_data;

	wire [6:0]  oam_addr;
	wire [15:0] oam_q;
	wire [5:0]  cpc_idx;
	wire [3:0]  cpc_q;
	wire        ob_ch_req;
	wire [11:0] ob_ch_addr;
	wire        ob_ch_ack;
	wire        ob_ch_valid;
	wire [15:0] ob_ch_data;

	wire [7:0]  pal_rd_addr;
	wire [15:0] pal_rd_data;
	wire        pal_sync_start = line_start && (pass_line == ref_last);

	wire        ss_lb_sel;
	wire [9:0]  ss_lb_addr;
	wire [7:0]  ss_lb_wdata;
	wire        ss_lb_wren;
	wire [7:0]  ss_lb_rdata;

	// The composition pass owns character/scroll/sprite VRAM for the whole
	// line (dots 0..P_VRAM_HOLD_END) on every line that has a pass.  The
	// hold is raster-mechanical: it does not depend on fetch demand or on
	// window size.
	wire        render_owns = (vline <= 8'd151) && (hdot < P_VRAM_HOLD_END);

	k2ge_vram #(
		.P_PAL_BYTE_WRITE (P_PAL_BYTE_WRITE)
	) u_vram (
		.clk_sys       (clk_sys),
		.ce            (ce_int),
		.ce_6m144      (ce_dot),
		.rst           (rst),
		.mono_strap    (mono_strap),
		.line_start    (line_start),
		.run           (main_clk_run),
		.frame_start   (frame_start),
		.palette_sync_start (pal_sync_start),
		.palette_frame_boundary (palette_frame_boundary),
		.palette_ss_wdata (ss_wdata[11:0]),
		.palette_ss_wren  (w20_wren),
		.palette_ss_state (pal_ctrl_ss_state),

		.cpu_a         (cpu_a),
		.cpu_din       (cpu_din),
		.cpu_dout      (vram_dout),
		.cpu_cs        (cpu_cs),
		.cpu_rd        (cpu_rd),
		.cpu_we        (cpu_we),

		.render_owns   (render_owns),
		.cpu_wait_states (cpu_wait_states),

		.s1_map_addr   (s1_map_addr),
		.s1_map_q      (s1_map_q),
		.s1_ch_req     (s1_ch_req),
		.s1_ch_addr    (s1_ch_addr),
		.s1_ch_ack     (s1_ch_ack),
		.s1_ch_valid   (s1_ch_valid),
		.s1_ch_data    (s1_ch_data),

		.s2_map_addr   (s2_map_addr),
		.s2_map_q      (s2_map_q),
		.s2_ch_req     (s2_ch_req),
		.s2_ch_addr    (s2_ch_addr),
		.s2_ch_ack     (s2_ch_ack),
		.s2_ch_valid   (s2_ch_valid),
		.s2_ch_data    (s2_ch_data),

		.oam_addr      (oam_addr),
		.oam_q         (oam_q),
		.cpc_idx       (cpc_idx),
		.cpc_q         (cpc_q),
		.ob_ch_req     (ob_ch_req),
		.ob_ch_addr    (ob_ch_addr),
		.ob_ch_ack     (ob_ch_ack),
		.ob_ch_valid   (ob_ch_valid),
		.ob_ch_data    (ob_ch_data),

		.pal_rd_addr   (pal_rd_addr),
		.pal_rd_data   (pal_rd_data),

		.ss_mem_active (ss_mem_active),
		.ss_mem_addr   (ss_mem_addr),
		.ss_mem_wdata  (ss_mem_wdata),
		.ss_mem_wren   (ss_mem_wren),
		.ss_mem_rden   (ss_mem_rden),
		.ss_mem_rdata  (ss_mem_rdata),

		.ss_lb_sel     (ss_lb_sel),
		.ss_lb_addr    (ss_lb_addr),
		.ss_lb_wdata   (ss_lb_wdata),
		.ss_lb_wren    (ss_lb_wren),
		.ss_lb_rdata   (ss_lb_rdata)
	);

	// The two scroll planes
	// Identical instances.  Which plane is in front is a property of P.F and
	// arrives as `rank`; nothing inside either instance knows which it is.

	wire       s1_lb_ready;
	wire       s1_lb_wr;
	wire [7:0] s1_lb_x;
	wire [8:0] s1_lb_data;
	wire       s1_busy;

	k2ge_scroll u_scr1 (
		.clk_sys     (clk_sys),
		.ce          (ce_int),
		.rst         (rst),

		.pass_start  (pass_start),
		.pass_line   (pass_line),

		.so_h        (ln_s1so_h),
		.so_v        (ln_s1so_v),
		.rank        (s1_rank),
		.mode_compat (mode_compat_eff),

		.map_addr    (s1_map_addr),
		.map_q       (s1_map_q),

		.ch_req      (s1_ch_req),
		.ch_addr     (s1_ch_addr),
		.ch_ack      (s1_ch_ack),
		.ch_valid    (s1_ch_valid),
		.ch_data     (s1_ch_data),

		.lb_ready    (s1_lb_ready),
		.lb_wr       (s1_lb_wr),
		.lb_x        (s1_lb_x),
		.lb_data     (s1_lb_data),

		.busy        (s1_busy),

		.ss_state    (s1_ss_state),
		.ss_wdata    (s1_ss_wdata),
		.ss_wren     (s1_ss_wren)
	);

	wire       s2_lb_ready;
	wire       s2_lb_wr;
	wire [7:0] s2_lb_x;
	wire [8:0] s2_lb_data;
	wire       s2_busy;

	k2ge_scroll u_scr2 (
		.clk_sys     (clk_sys),
		.ce          (ce_int),
		.rst         (rst),

		.pass_start  (pass_start),
		.pass_line   (pass_line),

		.so_h        (ln_s2so_h),
		.so_v        (ln_s2so_v),
		.rank        (s2_rank),
		.mode_compat (mode_compat_eff),

		.map_addr    (s2_map_addr),
		.map_q       (s2_map_q),

		.ch_req      (s2_ch_req),
		.ch_addr     (s2_ch_addr),
		.ch_ack      (s2_ch_ack),
		.ch_valid    (s2_ch_valid),
		.ch_data     (s2_ch_data),

		.lb_ready    (s2_lb_ready),
		.lb_wr       (s2_lb_wr),
		.lb_x        (s2_lb_x),
		.lb_data     (s2_lb_data),

		.busy        (s2_busy),

		.ss_state    (s2_ss_state),
		.ss_wdata    (s2_ss_wdata),
		.ss_wren     (s2_ss_wren)
	);

	// The sprite engine
	// PO comes from the same per-line latch group as the scroll offsets and is
	// read live several cycles into the pass, which is the post-latch value.
	// k2ge_scroll now samples its offsets one cycle after `pass_start` for the
	// same reason, so the two agree about which display line a write lands on.

	wire       ob_lb_req;
	wire       ob_lb_ready;
	wire       ob_lb_wr;
	wire [7:0] ob_lb_x;
	wire [8:0] ob_lb_data;
	wire       ob_busy;

	k2ge_obj u_obj (
		.clk_sys     (clk_sys),
		.ce          (ce_int),
		.rst         (rst),

		.pass_start  (pass_start),
		.pass_line   (pass_line),

		.po_h        (ln_po_h),
		.po_v        (ln_po_v),
		.mode_compat (mode_compat_eff),

		.oam_addr    (oam_addr),
		.oam_q       (oam_q),
		.cpc_idx     (cpc_idx),
		.cpc_q       (cpc_q),

		.ch_req      (ob_ch_req),
		.ch_addr     (ob_ch_addr),
		.ch_ack      (ob_ch_ack),
		.ch_valid    (ob_ch_valid),
		.ch_data     (ob_ch_data),

		.lb_req      (ob_lb_req),
		.lb_ready    (ob_lb_ready),
		.lb_wr       (ob_lb_wr),
		.lb_x        (ob_lb_x),
		.lb_data     (ob_lb_data),

		.busy        (ob_busy),

		.ss_state    (ob_ss_state),
		.ss_wdata    (ob_ss_wdata),
		.ss_wren     (ob_ss_wren)
	);

	// The line buffer
	// `*_req` is a producer's "I am in a pass" level, which is k2ge_scroll's
	// `busy` and k2ge_obj's `lb_req` (k2ge_linebuf's header names both).

	wire       so_rd;
	wire [7:0] so_x;
	wire [8:0] so_data;

	k2ge_linebuf u_lbuf (
		.clk_sys      (clk_sys),
		.ce           (ce_int),
		.rst          (rst),

		.buf_sel      (buf_half_q),
		.blnk         (blnk),

		.s1_req       (s1_busy),
		.s1_ready     (s1_lb_ready),
		.s1_wr        (s1_lb_wr),
		.s1_x         (s1_lb_x),
		.s1_data      (s1_lb_data),

		.s2_req       (s2_busy),
		.s2_ready     (s2_lb_ready),
		.s2_wr        (s2_lb_wr),
		.s2_x         (s2_lb_x),
		.s2_data      (s2_lb_data),

		.ob_req       (ob_lb_req),
		.ob_ready     (ob_lb_ready),
		.ob_wr        (ob_lb_wr),
		.ob_x         (ob_lb_x),
		.ob_data      (ob_lb_data),

		.so_rd        (so_rd),
		.so_x         (so_x),
		.so_data      (so_data),

		.ss_lb_sel    (ss_lb_sel),
		.ss_lb_addr   (ss_lb_addr),
		.ss_lb_wdata  (ss_lb_wdata),
		.ss_lb_wren   (ss_lb_wren),
		.ss_lb_rdata  (ss_lb_rdata),

		.lb_init      (lb_init),
		.ss_init_val  (ss_wdata[3]),
		.ss_init_wren (w0_wren)
	);

	// Scanout and the colour lookup.  Siblings, not parent and child:
	// k2ge_scanout decides what each dot is and k2ge_palette turns that
	// decision into twelve bits.

	wire       pal_form;
	wire [8:0] pal_entry;
	wire       pal_in_win;
	wire [3:0] pal_r, pal_g, pal_b;

	k2ge_scanout u_scanout (
		.clk_sys     (clk_sys),
		.ce          (ce_int),
		.ce_6m144    (ce_dot),
		.rst         (rst),

		.hdot        (hdot),
		.vline       (vline),
		.line_start  (line_start),

		.win_wba_h   (win_wba_h),
		.win_wba_v   (win_wba_v),
		.win_wsi_h   (win_wsi_h),
		.win_wsi_v   (win_wsi_v),

		.ln_neg      (ln_neg),

		.lb_rd_en    (so_rd),
		.lb_rd_x     (so_x),
		.lb_q        (so_data),

		.pal_form    (pal_form),
		.pal_entry   (pal_entry),
		.pal_in_win  (pal_in_win),
		.pal_r       (pal_r),
		.pal_g       (pal_g),
		.pal_b       (pal_b),

		.lcd_r       (lcd_r),
		.lcd_g       (lcd_g),
		.lcd_b       (lcd_b),
		.lcd_dclk_ce (lcd_dclk_ce),
		.lcd_de      (lcd_de),
		.lcd_hs      (lcd_hs),
		.lcd_vs      (lcd_vs),
		.lcd_lp      (lcd_lp),
		.lcd_sp      (lcd_sp),

		.ss_state    (sc_ss_state),
		.ss_wdata    (ss_wdata[47:0]),
		.ss_wren     (p13_wren)
	);

	k2ge_palette #(
		.P_MONO_BG_RAMP (P_MONO_BG_RAMP)
	) u_pal_lut (
		.clk_sys     (clk_sys),
		.ce          (ce_int),
		.rst         (rst),

		.form        (pal_form),
		.lb_entry    (pal_entry),
		.in_win      (pal_in_win),

		.line_start  (line_start),

		.p_f         (sc_p_f),
		.mode_compat (mode_compat),
		.mono_lut    (mono_lut_lsb),
		.bgon        (ln_bgon),
		.bgc         (ln_bgc),
		.oowc        (ln_oowc),

		.mono_strap  (mono_strap),

		.pal_rd_addr (pal_rd_addr),
		.pal_rd_data (pal_rd_data),

		.pal_r       (pal_r),
		.pal_g       (pal_g),
		.pal_b       (pal_b),

		.ss_state    (pal_ss_state),
		.ss_wdata    (ss_wdata[63:48]),
		.ss_wren     (p13_wren)
	);

	// Simulation-only guard: a composition pass must drain before the next
	// pass_start and before hdraw_end.  Not a hardware model of budget overrun.
`ifndef SYNTHESIS
	always @(posedge clk_sys) begin
		if (!rst) begin
			if (pass_start && (s1_busy || s2_busy || ob_busy))
				$display("k2ge: WARNING composition pass overran its line (s1=%b s2=%b ob=%b) at line %0d",
					s1_busy, s2_busy, ob_busy, vline);
			if (hdraw_end && (s1_busy || s2_busy || ob_busy))
				$display("k2ge: WARNING composition pass still running at hdraw_end (s1=%b s2=%b ob=%b) at line %0d",
					s1_busy, s2_busy, ob_busy, vline);
		end
	end
`endif

	// Deliberately unread
	// ob_busy is read only by the overrun watch above, which is inside
	// `ifndef SYNTHESIS, so the synthesised netlist has no reader for it.
	wire unused_ok = &{1'b0, ob_busy, 1'b0};

endmodule
