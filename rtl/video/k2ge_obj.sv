// Copyright (c) 2026 Jamie Blanks

// K2GE sprite engine -- the 64-entry chain evaluator and the 8-dot drawer.  One
// composition pass visits all 64 sprite table entries in index order, exactly
// once, and hands the visible ones to k2ge_linebuf eight dots at a time.  There
// is no per-priority pass and no sorting: the line buffer's strict rank compare
// makes pass order irrelevant, so one scan replaces the three the documented
// layer order suggests.
//
// The sprite table is four bytes at 0x8800 + 4n plus one nibble at 0x8C00 + n
// (K1GE 3-3-3; K2GE 4-3-3).  On the little-endian 16-bit bus:
//
//   word 2n     = {byte1, byte0}
//                 [15] H.F  [14] V.F  [13] P.C  [12:11] PR.C
//                 [10] H.ch [ 9] V.ch [ 8] C.C[8]   [7:0] C.C[7:0]
//   word 2n+1   = {byte3, byte2}
//                 [15:8] V.P    [7:0] H.P
//   0x8C00 + n  = CP.C[3:0], K2GE mode only (K2GE 4-3-3-3)
//
// Per sprite:
//
//   x_raw = H.ch ? (x_last + H.P) & 0xFF : H.P      chain from the previous
//   y_raw = V.ch ? (y_last + V.P) & 0xFF : V.P      sprite's computed position
//   x_last, y_last <= x_raw, y_raw                  ALWAYS - see below
//   x_eff = (x_raw + PO.H) & 0xFF                   (K1GE 3-3-4: H' = H.P +
//   y_eff = (y_raw + PO.V) & 0xFF                    PO.H, "over flow ignored")
//   dy      = (pass_line - y_eff) & 0xFF
//   visible = (PR.C != 00) && (dy < 8)
//   row     = dy[2:0] ^ {3{V.F}}
//   char    = character RAM[{C.C[8:0], row}]
//   emit 8 dots from x_eff, dot k pixel = char[15 - 2*(k ^ {3{H.F}}) -: 2]
//   rank    = PR.C 01 -> 1, 10 -> 3, 11 -> 5      code = CP.C or {3'b000, P.C}
//
// Three points that are easy to get wrong:
//
//  1. `x_last`/`y_last` update for EVERY sprite index, including ones that are
//     hidden (PR.C = 00), off this line, or entirely off screen.  A hidden
//     sprite is still a link in the chain and still contributes its offset.  No
//     SNK document states this; every known emulator does it this way.  They
//     reset to 0,0 at the start of every pass.
//  2. PO is added ONCE, after the chain, never per link.  That follows the
//     documented formula H' = H.P + PO.H, in which H.P is the sprite's own
//     (possibly chained) position (K1GE 3-3-4).  PO.H/PO.V at 0x8020/0x8021 are
//     the SPRITE global offset; the scroll planes have S1SO/S2SO instead and the
//     two must not be confused.
//  3. Sprite 0 is on top.  This module does nothing to arrange that beyond
//     scanning 0 to 63 and writing as it goes: the line buffer's strict rank
//     compare keeps the first write in any rank class (K1GE 3-3-3-1;
//     K2GE 4-3-3-1).  Never reverse this scan order, and never let anything
//     reorder the emitted dots between here and the buffer.
//
// Both axes wrap in eight bits, so a sprite whose origin is at y = 252 shows its
// lower rows on lines 0..3, and one at x = 254 puts its first two dots off the
// right edge and the remaining six at x = 0..5.  Dots outside 0..159 are
// discarded after the wrap; (K1GE 3-3-4) gives the position arithmetic as
// "overflow ignored".  Clipping is done at write time by the drawer rather than
// by a pre-scan, so a sprite straddling either edge costs exactly the same slots
// as one in the middle and pass length stays data-independent.
//
// The pipeline
//   E_IDLE  present the even OAM word address
//   E_WORD0 take word 2n, present the odd word address
//   E_WORD1 take word 2n+1, do the CHAIN adds, update x_last/y_last
//   E_CALC  do the PO adds, the row subtract and the visibility compare, read
//           CP.C, form the character address
//   E_CREQ  hold the character request until the arbiter's sprite slot
//   E_CWAIT take the character word, hand the sprite to the drawer
//
// The 8-bit chain add, PO add, row subtract and compare would be a long
// combinational chain in one enable period, and this module is not dot-paced, so
// the work is split structurally instead: chain adds in E_WORD1, PO adds,
// subtract and compare in E_CALC.
//
// E_IDLE..E_CWAIT and the drawer hand over through `full_q`, a one-sprite skid
// buffer, so evaluation of sprite n+1 overlaps the drawing of sprite n.  The
// drawer needs eight grants and the line buffer rotates between three producers,
// so a sprite takes far longer to draw than to evaluate and the evaluator is
// never the limit.  Once the planes have finished their pass the drawer gets
// every other cycle and 64 sprites cost about 1700 cycles = 212 dots of the
// 484-dot drawing period.
//
// `ce` is the chip's 49.152 MHz enable, tied high in normal operation.  There is
// no dot enable: the pipeline is clocked at the internal rate, one dot per
// granted write.
//
// PO.H/PO.V are read LIVE from k2ge_mmr's per-line latched copies rather than
// sampled at `pass_start`.  k2ge_mmr latches the composition-side registers on
// `latch_render`, which with P_RENDER_LATCH = 0 is the same edge as
// `pass_start`, so a copy taken here on that edge would capture the PREVIOUS
// line's value and land the write one display line late.  The latched copy
// cannot move again inside the pass, so reading it live is both correct and free.

module k2ge_obj
#(
	// Dots after `pass_start` before the sprite chain begins its OAM scan.
	// DERIVED: the two scroll planes issue 80 of the pass's 272 VRAM
	// accesses, ~144 dots of the 484-dot period.  The constant stands for
	// one claim: the sprite chain is fetched after both scroll planes.
	// Must stay below 512 for the 12-bit hold counter.
	parameter [9:0] P_OBJ_START_DOT = 10'd144
)
(
	input  wire         clk_sys,      // 49.152 MHz
	input  wire         ce,           // chip enable, tie 1 (see header)
	input  wire         rst,          // synchronous power-on reset

	// --- pass control (k2ge_vtimer) ----------------------------------------
	input  wire         pass_start,   // a composition pass begins this cycle
	input  wire [7:0]   pass_line,    // display line the pass composes

	// --- per-line latched registers (k2ge_mmr) -----------------------------
	input  wire [7:0]   po_h,         // 0x8020, sprite global X offset
	input  wire [7:0]   po_v,         // 0x8021, sprite global Y offset
	input  wire         mode_compat,  // 0x87E2 D7

	// --- sprite table port (k2ge_vram, free-running, one clock) -------------
	output wire [6:0]   oam_addr,
	input  wire [15:0]  oam_q,

	// --- sprite palette code (k2ge_vram, flops, same cycle) -----------------
	output wire [5:0]   cpc_idx,
	input  wire [3:0]   cpc_q,

	// --- character fetch through the k2ge_vram slot arbiter -----------------
	output wire         ch_req,
	output wire [11:0]  ch_addr,
	input  wire         ch_ack,
	input  wire         ch_valid,
	input  wire [15:0]  ch_data,

	// --- line-buffer write port (k2ge_linebuf) ------------------------------
	output wire         lb_req,       // -> k2ge_linebuf `ob_req`
	input  wire         lb_ready,     // write grant
	output wire         lb_wr,
	output wire [7:0]   lb_x,         // 0..255, the buffer drops x >= 160
	output wire [8:0]   lb_data,      // {rank[2:0], code[3:0], pix[1:0]}

	output wire         busy,         // a pass is in progress

	// --- savestate internals ------------------------------------------------
	// The live state is 134 bits, so it needs three words, exactly as
	// k2ge_scroll needed two: 191:128 = word 11 (evaluator), 127:64 = word 12
	// (fetch and skid buffer), 63:0 = one spare word (19) for the drawer.  The
	// alternative - truncating the drawer - would restore mid-pass states that
	// draw a sprite with the wrong x or the wrong character, which is exactly
	// the defect class that later shows up as one corrupted scanline nobody can
	// explain.
	output wire [191:0] ss_state,
	input  wire [191:0] ss_wdata,
	input  wire         ss_wren
);

	localparam [6:0] N_SPRITES = 7'd64;

	localparam [2:0] E_IDLE  = 3'd0;
	localparam [2:0] E_WORD0 = 3'd1;
	localparam [2:0] E_WORD1 = 3'd2;
	localparam [2:0] E_CALC  = 3'd3;
	localparam [2:0] E_CREQ  = 3'd4;
	localparam [2:0] E_CWAIT = 3'd5;

	// State

	// Pass level.
	reg        run_q;
	reg [11:0] hold_q;           // clk_sys cycles until the chain starts;
	                             // loaded with P_OBJ_START_DOT * 8 at
	                             // pass_start, freezes the whole engine
	reg [7:0]  line_q;           // the display line being composed

	// Evaluator.
	reg [6:0]  idx_q;            // 0..64; 64 means the scan is finished
	reg [2:0]  est_q;
	reg [15:0] w0_q;             // sprite word 2n, held for E_WORD1 and E_CALC
	reg [7:0]  x_last_q;         // chain accumulators - updated for every index
	reg [7:0]  y_last_q;

	// Fetch and the one-sprite skid buffer.
	reg        full_q;
	reg [11:0] fch_addr_q;       // {C.C[8:0], row[2:0]}
	reg [15:0] fchar_q;
	reg [3:0]  fcode_q;
	reg [1:0]  fprc_q;
	reg        fhf_q;
	reg [7:0]  fx_q;

	// Drawer.
	reg        em_run_q;
	reg [2:0]  em_dot_q;
	reg [7:0]  em_x_q;           // 8-bit, wraps: see header note (b)
	reg [15:0] em_word_q;
	reg [3:0]  em_code_q;
	reg [1:0]  em_prc_q;
	reg        em_hf_q;

	// Sprite table fields

	wire       a_hf  = w0_q[15];
	wire       a_vf  = w0_q[14];
	wire       a_pc  = w0_q[13];
	wire [1:0] a_prc = w0_q[12:11];
	wire       a_hch = w0_q[10];
	wire       a_vch = w0_q[9];
	wire [8:0] a_cc  = {w0_q[8], w0_q[7:0]};

	// Word 2n+1 is on the port during E_WORD1, so its fields are taken live.
	wire [7:0] a_hp = oam_q[7:0];
	wire [7:0] a_vp = oam_q[15:8];

	// Chain: an unchained sprite takes its position absolutely, a chained one
	// takes it relative to the previous sprite's computed position.
	wire [7:0] x_raw = a_hch ? (x_last_q + a_hp) : a_hp;
	wire [7:0] y_raw = a_vch ? (y_last_q + a_vp) : a_vp;

	// Global sprite offset, added once after the chain, overflow ignored.
	wire [7:0] x_eff = x_last_q + po_h;
	wire [7:0] y_eff = y_last_q + po_v;

	// Row within the 8x8 character, wrapped in eight bits.
	wire [7:0] dy  = line_q - y_eff;
	wire [2:0] row = dy[2:0] ^ {3{a_vf}};

	wire visible = (a_prc != 2'b00) && (dy[7:3] == 5'd0);

	// K2GE mode takes the four-bit palette code from CP.C; the K1GE upper
	// palette compatible mode has only the one-bit P.C (K2GE 4-3-3-3).
	wire [3:0] a_code = mode_compat ? {3'b000, a_pc} : cpc_q;

	// The even word address is presented in E_IDLE and the odd one in E_WORD0.
	assign oam_addr = {idx_q[5:0], (est_q == E_WORD0)};
	assign cpc_idx  = idx_q[5:0];

	assign ch_req  = (est_q == E_CREQ);
	assign ch_addr = fch_addr_q;

	// The drawer's dot
	// PR.C 01 "Furthest" -> rank 1, 10 "Middle" -> 3, 11 "Front" -> 5
	// (K1GE Table 1).  PR.C 00 never reaches here.

	reg [2:0] em_rank;

	always @* begin
		case (em_prc_q)
			2'b01:   em_rank = 3'd1;
			2'b10:   em_rank = 3'd3;
			default: em_rank = 3'd5;
		endcase
	end

	// Horizontal flip mirrors the dot index inside the character row; inside
	// the fetched word bits 15:14 are dot 0 and bits 1:0 are dot 7
	// (K1GE 3-3-5-1; K2GE 4-3-5-1).
	wire [2:0] em_i   = em_dot_q ^ {3{em_hf_q}};
	wire [1:0] em_pix = em_word_q[{~em_i, 1'b1} -: 2];

	// Dots that land outside the 160-dot screen after the 8-bit wrap are simply
	// not written.
	wire em_vis = (em_x_q < 8'd160);

	assign lb_req  = em_run_q;
	assign lb_wr   = em_run_q && lb_ready && em_vis && (em_pix != 2'b00);
	assign lb_x    = em_x_q;
	assign lb_data = {em_rank, em_code_q, em_pix};

	// The start hold is part of the pass: without it, k2ge.sv's overrun
	// watch (and any other consumer) would see a pass "drained" while the
	// engine was merely waiting out the scroll planes' share of the line.
	assign busy = run_q || (hold_q != 12'd0);

	// Savestate image

	// Word 11: 52 bits of evaluator plus the 12-bit start hold (was padding).
	// Word 12: 43 bits of fetch and skid buffer, 21 of padding.
	// Word 19: 35 bits of drawer, 29 of padding.
	assign ss_state = {run_q, line_q, idx_q, est_q, x_last_q, y_last_q,
	                   full_q, w0_q, hold_q,
	                   fch_addr_q, fchar_q, fcode_q, fprc_q, fhf_q, fx_q,
	                   21'd0,
	                   em_run_q, em_dot_q, em_x_q, em_word_q, em_code_q,
	                   em_prc_q, em_hf_q, 29'd0};

	// The pipeline
	// One clocked block, because the evaluator and the drawer share `full_q`:
	// two blocks would be two drivers for it, which synthesis rejects.  The
	// drawer's handover is written last so that a
	// sprite completing its fetch on this edge is not also taken on this edge.

	always @(posedge clk_sys) begin
		if (rst) begin
			run_q      <= 1'b0;
			hold_q     <= 12'd0;
			line_q     <= 8'd0;
			idx_q      <= 7'd0;
			est_q      <= E_IDLE;
			w0_q       <= 16'd0;
			x_last_q   <= 8'd0;
			y_last_q   <= 8'd0;
			full_q     <= 1'b0;
			fch_addr_q <= 12'd0;
			fchar_q    <= 16'd0;
			fcode_q    <= 4'd0;
			fprc_q     <= 2'd0;
			fhf_q      <= 1'b0;
			fx_q       <= 8'd0;
			em_run_q   <= 1'b0;
			em_dot_q   <= 3'd0;
			em_x_q     <= 8'd0;
			em_word_q  <= 16'd0;
			em_code_q  <= 4'd0;
			em_prc_q   <= 2'd0;
			em_hf_q    <= 1'b0;
		end else if (ss_wren) begin
			run_q      <= ss_wdata[191];
			hold_q     <= ss_wdata[139:128];
			line_q     <= ss_wdata[190:183];
			idx_q      <= ss_wdata[182:176];
			est_q      <= ss_wdata[175:173];
			x_last_q   <= ss_wdata[172:165];
			y_last_q   <= ss_wdata[164:157];
			full_q     <= ss_wdata[156];
			w0_q       <= ss_wdata[155:140];
			fch_addr_q <= ss_wdata[127:116];
			fchar_q    <= ss_wdata[115:100];
			fcode_q    <= ss_wdata[99:96];
			fprc_q     <= ss_wdata[95:94];
			fhf_q      <= ss_wdata[93];
			fx_q       <= ss_wdata[92:85];
			em_run_q   <= ss_wdata[63];
			em_dot_q   <= ss_wdata[62:60];
			em_x_q     <= ss_wdata[59:52];
			em_word_q  <= ss_wdata[51:36];
			em_code_q  <= ss_wdata[35:32];
			em_prc_q   <= ss_wdata[31:30];
			em_hf_q    <= ss_wdata[29];
		end else if (ce) begin

			// --- pass start ------------------------------------------------
			// The chain accumulators default to 0,0 at the head of the list.
			// A pass always restarts
			// the engine: one pass per line against a 484-dot drawing period
			// means a pass can never begin before the previous one drained, so
			// this is a defensive reset rather than a mode.
			if (pass_start) begin
				// The engine holds for P_OBJ_START_DOT dots (8 clk_sys
				// cycles each) before the chain starts -- the scroll
				// planes' share of the drawing period; see the parameter.
				hold_q   <= {P_OBJ_START_DOT[8:0], 3'b000};
				run_q    <= (P_OBJ_START_DOT == 10'd0);
				line_q   <= pass_line;
				idx_q    <= 7'd0;
				est_q    <= E_IDLE;
				w0_q     <= 16'd0;
				x_last_q <= 8'd0;
				y_last_q <= 8'd0;
				full_q   <= 1'b0;
				em_run_q <= 1'b0;
				em_dot_q <= 3'd0;
			end else if (hold_q != 12'd0) begin
				hold_q <= hold_q - 12'd1;
				if (hold_q == 12'd1) run_q <= 1'b1;
			end else begin

				// --- evaluator and fetch --------------------------------
				case (est_q)
					E_IDLE: begin
						// The even word address is on the port this cycle; the
						// word arrives on the next one.  Held here while the
						// skid buffer is full, which is the only back pressure
						// in the engine.
						if (run_q && (idx_q != N_SPRITES) && !full_q)
							est_q <= E_WORD0;
					end

					E_WORD0: begin
						w0_q  <= oam_q;
						est_q <= E_WORD1;
					end

					E_WORD1: begin
						// The chain, and the accumulator update that happens
						// for every index whether or not the sprite is drawn.
						x_last_q <= x_raw;
						y_last_q <= y_raw;
						est_q    <= E_CALC;
					end

					E_CALC: begin
						// x_last_q/y_last_q now hold THIS sprite's chained raw
						// position, so the offset adds, the row subtract and
						// the compare all read them.
						fch_addr_q <= {a_cc, row};
						fcode_q    <= a_code;
						fprc_q     <= a_prc;
						fhf_q      <= a_hf;
						fx_q       <= x_eff;

						if (visible) begin
							est_q <= E_CREQ;
						end else begin
							idx_q <= idx_q + 7'd1;
							est_q <= E_IDLE;
						end
					end

					E_CREQ: begin
						// Hold the request until the arbiter's sprite slot
						// comes round.  Dropping it on the ack keeps a
						// long-held request from being taken twice.
						if (ch_ack) est_q <= E_CWAIT;
					end

					E_CWAIT: begin
						if (ch_valid) begin
							fchar_q <= ch_data;
							full_q  <= 1'b1;
							idx_q   <= idx_q + 7'd1;
							est_q   <= E_IDLE;
						end
					end

					default: est_q <= E_IDLE;
				endcase

				// --- drawer ---------------------------------------------
				if (em_run_q) begin
					if (lb_ready) begin
						em_x_q   <= em_x_q + 8'd1;   // 8-bit wrap, note (b)
						em_dot_q <= em_dot_q + 3'd1;
						if (em_dot_q == 3'd7) em_run_q <= 1'b0;
					end
				end else if (full_q) begin
					em_word_q <= fchar_q;
					em_code_q <= fcode_q;
					em_prc_q  <= fprc_q;
					em_hf_q   <= fhf_q;
					em_x_q    <= fx_q;
					em_dot_q  <= 3'd0;
					em_run_q  <= 1'b1;
					full_q    <= 1'b0;
				end else if ((idx_q == N_SPRITES) && (est_q == E_IDLE)) begin
					// All 64 visited, nothing in the skid buffer, nothing left
					// to draw: the pass is over.
					run_q <= 1'b0;
				end
			end
		end
	end

	// Deliberately unread inputs
	// The savestate image is 146 bits inside three words; the rest is padding.
	wire unused_ok = &{1'b0, ss_wdata[84:64],
	                   ss_wdata[28:0], 1'b0};

endmodule
