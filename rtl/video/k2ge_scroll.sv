// Copyright (c) 2026 Jamie Blanks

// K2GE tilemap fetch pipeline -- one scroll plane.  Instantiated twice, once
// per plane, wired to that plane's map RAM port and character-fetch slot in
// k2ge_vram.  The module is identical for both planes: which one is in front is
// a property of P.F, and the parent passes the result in as `rank`.
//
// A pass for display line `pass_line` starts on k2ge_vtimer's `pass_start`
// strobe (K1GE p.4 feature 1; K1GE p.25 / K2GE p.35) and does
// (K1GE 3-4-5/3-4-6/3-4-9, 3-3-5-1; K2GE 4-4-4/4-4-5):
//
//   veff  = (pass_line + SnSO.V) & 0xFF        the plane is a 256x256 cyclical
//   for tile t = 0 .. 20:                      virtual field, 32x32 cells
//     heff      = (t*8 + SnSO.H) & 0xFF
//     map word  = scroll RAM[{veff[7:3], heff[7:3]}]   +1 right, +32 down
//     C.C[8:0]  = map[8:0]      CP.C[3:0] = map[12:9]  (K2GE mode)
//     P.C       = map[13]       V.F = map[14]   H.F = map[15]
//     row       = veff[2:0] ^ {3{V.F}}
//     char word = character RAM[{C.C[8:0], row}]
//     emit 8 dots, dot k pixel = char[15 - 2*(k ^ {3{H.F}}) -: 2]
//
// Twenty-one tiles, not twenty: 160 dots is twenty cells, and any horizontal
// offset that is not a multiple of eight clips the first cell and needs a
// twenty-first to fill the right edge.
//
// heff has the same low three bits for every t, so the map column is just
// (t + SnSO.H[7:3]) mod 32 -- a counter, no multiplier.  Screen x for dot k of
// tile t is t*8 + k - SnSO.H[2:0], which can be negative at the left edge and
// past 159 at the right, so the emitter carries a 9-bit signed x and
// out-of-range dots are simply not written.  Clipping at write time costs the
// same slots as any other tile, which keeps the pass length constant.
//
// Pixel bit order: "even byte holds dots 4-7, odd byte dots 0-3" on a
// little-endian 16-bit bus (K1GE 3-3-5-1; K2GE 4-3-5-1), so inside the fetched
// word bits 15:14 are dot 0 and bits 1:0 are dot 7.
//
// The 9-bit entry out is lb_data = { rank[2:0], code[3:0], pix[1:0] }:
//
//   rank  2 = back plane, 4 = front plane; the parent derives it from the
//         per-line latched P.F (P.F = 0 means plane 1 is in front, K1GE 3-4-8)
//   code  K2GE mode -> CP.C from map[12:9]; compatibility mode -> {3'b000, P.C}
//         from map[13]
//   pix   the 2-bit character pixel
//
// The composite rule is "write iff pix != 0 && rank_new > rank_stored"
// (K1GE 3-3-3-1 write-first; K1GE 3-7 caution 1, "00 is always clear").  This
// module enforces the pix half -- a clear dot never asserts lb_wr, on any layer
// -- and k2ge_linebuf enforces the rank half in its read-modify-write.  Do not
// collapse that rank compare into an unconditional write.
//
// The pipeline is
//
//   S0 map address -> S1 map data / char address -> S2 char data -> S3 emit 8
//
// where S0..S2 are a fetch engine running one tile ahead and S3 an emitter
// walking eight dots.  They hand a tile over through `full_q`, so the fetch of
// tile t+1 overlaps the emission of tile t and the plane produces eight dots per
// eight line-buffer write slots without stalling the writer.  A pipeline with a
// one-tile skid buffer, deliberately not a loop.
//
// `lb_ready` is the line buffer's write grant.  The emitter advances exactly one
// dot per granted cycle whether or not that dot is written, so pass length is
// data-independent and matches the 160-op-per-plane budget.  At the two-clk_sys
// read-modify-write rate eight dots take sixteen cycles while the worst-case
// fetch of the next tile takes about thirteen (one to issue the map read, one
// for the word, up to eight waiting for this plane's character slot, two for the
// slot, one to hand over), so the fetch engine stays ahead.  If a parent grants
// writes faster the emitter waits for `full_q`.
//
// The offsets are sampled one cycle AFTER `pass_start`.  k2ge_mmr latches the
// composition-side registers on `latch_render`, which with P_RENDER_LATCH = 0 is
// the same edge as `pass_start`; sampling `so_h`/`so_v` on that edge with a
// non-blocking assignment would capture the PREVIOUS line's latched copy and run
// the plane one display line behind the sprite engine, which reads PO from the
// same latch group live several cycles into the pass.  So pass initialisation is
// taken on `start_q`, the registered copy of `pass_start`: one clk_sys cycle
// later out of the 3872 in a line, with the latch settled.
//
// `ce` is the chip's 49.152 MHz enable, tied high in normal operation and
// dropped to freeze the block for a pause or a savestate walk.  There is no dot
// enable here -- the pipeline is clocked at the internal rate, one dot per
// granted write.
//
// The pass emits its dots in tile order 0..20 and dot order 0..7 within a tile,
// which is the order the documents draw but is only observable through C.OVR
// truncation.

module k2ge_scroll
(
	input  wire        clk_sys,       // 49.152 MHz
	input  wire        ce,            // chip enable, tie 1 (see header)
	input  wire        rst,           // synchronous power-on reset

	// --- pass control (k2ge_vtimer) ----------------------------------------
	input  wire        pass_start,    // a composition pass begins this cycle
	input  wire [7:0]  pass_line,     // display line the pass composes

	// --- per-line latched registers (k2ge_mmr) -----------------------------
	input  wire [7:0]  so_h,          // S1SO.H or S2SO.H
	input  wire [7:0]  so_v,          // S1SO.V or S2SO.V
	input  wire [2:0]  rank,          // 2 = back plane, 4 = front plane
	input  wire        mode_compat,   // 0x87E2 D7

	// --- scroll map port (k2ge_vram, free-running, one clock) ---------------
	output wire [9:0]  map_addr,
	input  wire [15:0] map_q,

	// --- character fetch through the k2ge_vram slot arbiter -----------------
	output wire        ch_req,
	output wire [11:0] ch_addr,
	input  wire        ch_ack,
	input  wire        ch_valid,
	input  wire [15:0] ch_data,

	// --- line-buffer write port (k2ge_linebuf) ------------------------------
	input  wire        lb_ready,      // write grant
	output wire        lb_wr,
	output wire [7:0]  lb_x,          // 0..159
	output wire [8:0]  lb_data,       // {rank[2:0], code[3:0], pix[1:0]}

	output wire        busy,          // a pass is in progress

	// --- savestate internals ------------------------------------------------
	// The plane's live state is 90 bits and is exported as two 64-bit words.
	output wire [127:0] ss_state,
	input  wire [127:0] ss_wdata,
	input  wire         ss_wren
);

	localparam [4:0] N_TILES = 5'd21;

	// State

	// Pass level.
	reg        start_q;         // pass_start, delayed one cycle (see the header)
	reg        run_q;
	reg [7:0]  veff_q;          // (pass_line + so_v) & 0xFF

	// Fetch engine (S0..S2).
	reg [4:0]  ftile_q;         // tiles fetched so far, 0..21
	reg [4:0]  fcol_q;          // map column, wraps mod 32
	reg [1:0]  fst_q;
	reg        full_q;          // a fetched tile is waiting for the emitter
	reg [11:0] fch_addr_q;      // {C.C[8:0], row[2:0]}
	reg [3:0]  fcode_q;
	reg        fhf_q;
	reg [15:0] fchar_q;

	// Emitter (S3).
	reg        sh_run_q;
	reg [2:0]  sh_dot_q;
	reg [8:0]  sh_x_q;          // signed screen x, -8 .. 167
	reg [15:0] sh_word_q;
	reg [3:0]  sh_code_q;
	reg        sh_hf_q;

	localparam [1:0] F_IDLE  = 2'd0;   // S0: issue the map address
	localparam [1:0] F_MAPW  = 2'd1;   // S1: map word arrives, form char address
	localparam [1:0] F_CREQ  = 2'd2;   // S2a: hold the request until the slot
	localparam [1:0] F_CWAIT = 2'd3;   // S2b: wait for the slot's data

	// Combinational decode of the fetched map word and the emitted dot

	wire       f_start = run_q && (ftile_q != N_TILES) && !full_q
	                     && (fst_q == F_IDLE);

	assign map_addr = {veff_q[7:3], fcol_q};

	// Map word fields (K1GE 3-4-5; K2GE 4-4-4).
	wire [8:0] m_cc = map_q[8:0];
	wire [3:0] m_cp = map_q[12:9];
	wire       m_pc = map_q[13];
	wire       m_vf = map_q[14];
	wire       m_hf = map_q[15];

	// Vertical flip picks the mirrored row of the same character.
	wire [2:0] m_row = veff_q[2:0] ^ {3{m_vf}};

	// K2GE mode takes the four-bit palette code from the map word; the K1GE
	// upper-palette compatible mode has only the one-bit P.C (K2GE 4-4-4).
	wire [3:0] m_code = mode_compat ? {3'b000, m_pc} : m_cp;

	assign ch_req  = (fst_q == F_CREQ);
	assign ch_addr = fch_addr_q;

	// Horizontal flip mirrors the dot index inside the character row.
	wire [2:0] sh_i   = sh_dot_q ^ {3{sh_hf_q}};
	wire [1:0] sh_pix = sh_word_q[{~sh_i, 1'b1} -: 2];

	// Dots that fall outside 0..159 after the offset are discarded here rather
	// than by a pre-scan, so a plane clipped at either edge costs the same
	// slots as one that is not.
	wire sh_vis = !sh_x_q[8] && (sh_x_q[7:0] < 8'd160);

	assign lb_wr   = sh_run_q && lb_ready && sh_vis && (sh_pix != 2'b00);
	assign lb_x    = sh_x_q[7:0];
	assign lb_data = {rank, sh_code_q, sh_pix};

	assign busy = run_q;

	// Savestate image

	assign ss_state = {run_q, veff_q, ftile_q, fcol_q, fst_q, full_q,       // 22
	                   fch_addr_q, fcode_q, fhf_q, fchar_q,                 // 33
	                   sh_run_q, sh_dot_q, sh_x_q, sh_word_q, sh_code_q,    // 33
	                   sh_hf_q,                                             //  1
	                   start_q,                                             //  1
	                   38'd0};

	// The pipeline
	// One clocked block, because the fetch engine and the emitter share
	// `full_q`: two blocks would mean two drivers for it, which synthesis
	// rejects.  The emitter's handover is written last so
	// that a tile completing and a tile being taken on the same edge resolve in
	// favour of the take, which is the case that keeps the pipeline moving.

	always @(posedge clk_sys) begin
		if (rst) begin
			start_q    <= 1'b0;
			run_q      <= 1'b0;
			veff_q     <= 8'd0;
			ftile_q    <= 5'd0;
			fcol_q     <= 5'd0;
			fst_q      <= F_IDLE;
			full_q     <= 1'b0;
			fch_addr_q <= 12'd0;
			fcode_q    <= 4'd0;
			fhf_q      <= 1'b0;
			fchar_q    <= 16'd0;
			sh_run_q   <= 1'b0;
			sh_dot_q   <= 3'd0;
			sh_x_q     <= 9'd0;
			sh_word_q  <= 16'd0;
			sh_code_q  <= 4'd0;
			sh_hf_q    <= 1'b0;
		end else if (ss_wren) begin
			start_q    <= ss_wdata[38];
			run_q      <= ss_wdata[127];
			veff_q     <= ss_wdata[126:119];
			ftile_q    <= ss_wdata[118:114];
			fcol_q     <= ss_wdata[113:109];
			fst_q      <= ss_wdata[108:107];
			full_q     <= ss_wdata[106];
			fch_addr_q <= ss_wdata[105:94];
			fcode_q    <= ss_wdata[93:90];
			fhf_q      <= ss_wdata[89];
			fchar_q    <= ss_wdata[88:73];
			sh_run_q   <= ss_wdata[72];
			sh_dot_q   <= ss_wdata[71:69];
			sh_x_q     <= ss_wdata[68:60];
			sh_word_q  <= ss_wdata[59:44];
			sh_code_q  <= ss_wdata[43:40];
			sh_hf_q    <= ss_wdata[39];
		end else if (ce) begin

			start_q <= pass_start;

			// --- pass start, one cycle after the strobe --------------------
			// The delay is what makes `so_h`/`so_v` the POST-latch values, so
			// this plane and the sprite engine agree about which display line a
			// write lands on (see the header).  A pass always restarts the
			// plane.  There is no case in the documented schedule where a pass
			// begins before the previous one has drained (one pass per line, 21
			// tiles against 484 dots), so this is a defensive reset rather than
			// a mode.
			if (start_q) begin
				run_q    <= 1'b1;
				veff_q   <= pass_line + so_v;            // 8-bit, wraps: the
				                                         // field is 256 high
				ftile_q  <= 5'd0;
				fcol_q   <= so_h[7:3];                   // first cell column
				fst_q    <= F_IDLE;
				full_q   <= 1'b0;
				sh_run_q <= 1'b0;
				sh_dot_q <= 3'd0;
				// Tile 0's leftmost dot sits SnSO.H[2:0] pixels off the left
				// edge, so screen x starts negative whenever the offset is not
				// a multiple of eight.
				sh_x_q   <= 9'd0 - {6'd0, so_h[2:0]};
			end else begin

				// --- fetch engine, S0..S2 ----------------------------------
				case (fst_q)
					F_IDLE: begin
						// S0: the map address is on the port this cycle; the
						// word arrives on the next one.
						if (f_start) fst_q <= F_MAPW;
					end

					F_MAPW: begin
						// S1: take the map word and form the character
						// address.  Everything else the emitter needs is
						// extracted here so the raw word need not be kept.
						fch_addr_q <= {m_cc, m_row};
						fcode_q    <= m_code;
						fhf_q      <= m_hf;
						fst_q      <= F_CREQ;
					end

					F_CREQ: begin
						// S2a: hold the request until this plane's slot comes
						// round.  Dropping it on the ack keeps a long-held
						// request from being taken twice.
						if (ch_ack) fst_q <= F_CWAIT;
					end

					F_CWAIT: begin
						// S2b: the arbiter returns the word two cycles after
						// the ack.
						if (ch_valid) begin
							fchar_q <= ch_data;
							full_q  <= 1'b1;
							ftile_q <= ftile_q + 5'd1;
							fcol_q  <= fcol_q + 5'd1;    // wraps mod 32: the
							                             // map is cyclical
							fst_q   <= F_IDLE;
						end
					end

					default: fst_q <= F_IDLE;
				endcase

				// --- emitter, S3 --------------------------------------------
				if (sh_run_q) begin
					if (lb_ready) begin
						sh_x_q   <= sh_x_q   + 9'd1;
						sh_dot_q <= sh_dot_q + 3'd1;
						if (sh_dot_q == 3'd7) sh_run_q <= 1'b0;
					end
				end else if (full_q) begin
					// Handover.  Written after the fetch engine so that a tile
					// arriving on this edge is not also taken on this edge.
					sh_word_q <= fchar_q;
					sh_code_q <= fcode_q;
					sh_hf_q   <= fhf_q;
					sh_dot_q  <= 3'd0;
					sh_run_q  <= 1'b1;
					full_q    <= 1'b0;
				end else if ((ftile_q == N_TILES) && (fst_q == F_IDLE)) begin
					// Twenty-one tiles fetched, nothing in flight, nothing
					// left to emit: the pass is over.
					run_q <= 1'b0;
				end
			end
		end
	end

	// Deliberately unread inputs
	// The savestate image is 90 bits inside two words; the rest is padding.
	wire unused_ok = &{1'b0, ss_wdata[37:0], 1'b0};

endmodule
