// Copyright (c) 2026 Jamie Blanks

// K2GE line buffer -- the double 160 x 9 composition buffer and its rank
// compare.  Everything the render pipeline produces lands here and everything
// the scanout displays comes out of here.  The module owns one block RAM, the
// read-modify-write that composes into it, the write grants the three producers
// share, the read-and-clear the scanout uses, the one-shot power-on clear sweep
// and the savestate window k2ge_vram forwards.
//
// (K2GE 4-1) gives the buffer's width and only the width: "{20 x (9 x 8)} x 2 =
// 360 Bytes ... number of bits necessary to describe 1 dot".  The field
// assignment inside those nine bits is this design's:
//
//   entry[8:0] = { rank[2:0], code[3:0], pix[1:0] }
//
//   rank  0 background (never written), 1 sprite PR.C=01, 2 back plane,
//         3 sprite PR.C=10, 4 front plane, 5 sprite PR.C=11
//   code  CP.C in K2GE mode, {3'b000, P.C} in compatibility mode
//   pix   the 2-bit character pixel, never 00
//
// Every write is a rank-compare read-modify-write:
//
//   write iff (pix != 2'b00) && (rank_new > rank_stored)
//
// which reproduces both documented behaviours at once.  Different ranks layer
// back to front (K1GE 3-3-3-2 Fig 4; K2GE 4-3-3-2), and equal ranks keep the
// earlier write -- which, because the sprite evaluator scans 0 to 63, puts
// sprite 0 on top: "The hardware reads the values from the VRAM 0 address and
// writes to the line buffer.  During the write to the line buffer, the hardware
// checks the priority ... to avoid writing over previously written data"
// (K1GE 3-3-3-1; K2GE 4-3-3-1).
//
// Because the comparison is STRICT the six documented passes may be run in any
// order and interleaved arbitrarily, which is what lets the chip do one sprite
// scan instead of three with no per-pass "already written" bitmap.  Do not
// collapse the compare into an unconditional write, and do not make it
// non-strict: `>=` silently inverts sprite-versus-sprite order.
//
// The pix half of the rule is not enforced here; it belongs to the producers,
// which know the pixel before they ask for a slot.  A producer that asserts its
// write strobe for a clear pixel would store rank != 0 with pix == 00, which
// has no palette entry at all (K1GE 3-7 caution 1; K2GE 5-1, "00 b is always
// treated as clear color"), so it is a hard contract on the producers.
//
// Rank 0 is never written by anybody: the buffer is cleared to zero and the
// scanout substitutes the background colour wherever it reads rank 0, which is
// what "clear pixels never modify the line memory" means and saves a fill pass
// per line.
//
// The buffer is one 512 x 16 byte-enabled block (`cache_ram_dp_be`) rather than
// 512 x 9, with the entry in bits 8:0 and bits 15:9 held at zero: the same
// single M10K, and byte enables make the savestate walker's byte write exact.
// Address = {half, x[7:0]}, and since x is 0..159 the 96 unused words per half
// are cleared by the power-on sweep anyway.  Producers clip their own dots to
// x < 160 at write time (both k2ge_scroll and k2ge_obj do, which keeps pass
// length data-independent); this module deliberately does not re-clip, since a
// second guard would mask a producer that lost its own.
//
// The chip composes display row L during raster line L and shifts it out during
// raster line L+1 (K1GE p.4 feature 1; K2GE p.6), so with `buf_sel` toggling
// once per line:
//
//   render read-modify-write  ->  half ~buf_sel   (the line being composed)
//   scanout read-and-clear    ->  half  buf_sel   (the line being displayed)
//
// One flip per line, no explicit swap logic.  The scanout clears as it reads,
// so the half is empty when the pass two lines later reuses it, and the two
// ports can never touch the same half in the same line -- which is why they
// need no collision arbitration.
//
// Port A carries the write grants for three producers: scroll 1, scroll 2 and
// the sprite drawer.  A read-modify-write is two clk_sys cycles:
//
//   cycle 0  grant: the winning producer's x drives the RAM, its {wr, x, data}
//            are captured, and the entry stored there comes out
//   cycle 1  the captured address drives the RAM again with the write enable
//            gated by the rank compare
//
// One grant every two cycles, rotating round-robin between the producers that
// are asking, so none can be starved and a lone requester gets every slot.  The
// grant is issued whether or not the producer writes the dot -- a clipped or
// clear dot costs the same slot as any other, which keeps a pass constant-time.
// Worst case per line is 320 plane dots plus 512 sprite dots = 832 ops = 1664
// cycles = 208 dots against the 484-dot drawing period.  `*_req` is a
// producer's "I am in a pass" level: k2ge_scroll's `busy`, k2ge_obj's `lb_req`.
//
// Port B is the scanout read-and-clear and the power-on sweep.  Assert so_rd
// with so_x; the next cycle so_data holds the stored entry and the entry is
// overwritten with zero.  That takes two port cycles because the wrapper
// returns new data on a read-during-write, so reading and clearing in one cycle
// would return the zero just written.  `so_ready` falls for the clear cycle,
// capping the rate at one pixel every two cycles; the scanout asks once every
// 24 cycles (8 clk_sys per dot, CE_PIXEL one dot in three) and never stalls.
//
// After power-on both halves hold whatever the block came up with, so a
// one-shot sweep clears all 512 words during the first V blank, gated by
// `lb_init`.  It must not run every V blank: the halves are self-clearing, so a
// repeat sweep buys nothing and can only race the scanout.  It starts at `blnk`
// = line 152, which still carries display row 151, so on the first frame it can
// zero part of a row that was garbage anyway; the port-B mux gives the scanout
// priority.  `lb_init` is exported for savestate word 0 and can be restored, so
// that a restored state does not sweep away the buffer the tap just wrote.
//
// k2ge_vram forwards flat tap bytes 0x0900-0x0B7F here as a 0..639 byte offset:
// 320 entries, one 16-bit word each, high seven bits zero:
//
//   word w = ss_lb_addr[9:1]      w = 0..159   -> half 0, x = w
//                                 w = 160..319 -> half 1, x = w - 160
//   even byte  entry[7:0]         odd byte  {7'b0, entry[8]}
//
// ss_lb_rdata is the byte one cycle after ss_lb_addr, exactly like a block RAM,
// because it is the block RAM's output with the lane bit registered alongside.
// The tap steals port A, is not gated by `ce` and takes priority over the render
// side, which is legal because the pause protocol quiesces the chip at a line
// boundary before the walker runs, so no read-modify-write can be in flight.
// One that somehow were is cancelled rather than completed with a corrupted
// compare.
//
// `ce` is the chip's 49.152 MHz enable, tied high in normal operation and
// dropped to freeze the block for a pause or a savestate walk.  It gates the
// grants, the read-modify-write, the scanout handshake and the sweep, but never
// the savestate window.

module k2ge_linebuf
(
	input  wire        clk_sys,      // 49.152 MHz
	input  wire        ce,           // chip enable, tie 1 (see header)
	input  wire        rst,          // synchronous power-on reset

	// --- raster context (k2ge_vtimer) --------------------------------------
	input  wire        buf_sel,      // vline[0]: the half being displayed
	input  wire        blnk,         // in V blank: gates the one-shot sweep

	// --- producer: scroll plane 1 ------------------------------------------
	input  wire        s1_req,       // wire to k2ge_scroll `busy`
	output wire        s1_ready,     // -> k2ge_scroll `lb_ready`
	input  wire        s1_wr,
	input  wire [7:0]  s1_x,
	input  wire [8:0]  s1_data,

	// --- producer: scroll plane 2 ------------------------------------------
	input  wire        s2_req,
	output wire        s2_ready,
	input  wire        s2_wr,
	input  wire [7:0]  s2_x,
	input  wire [8:0]  s2_data,

	// --- producer: sprite drawer -------------------------------------------
	input  wire        ob_req,       // wire to k2ge_obj `lb_req`
	output wire        ob_ready,
	input  wire        ob_wr,
	input  wire [7:0]  ob_x,
	input  wire [8:0]  ob_data,

	// --- scanout read-and-clear (k2ge_scanout) -----------------------------
	input  wire        so_rd,
	input  wire [7:0]  so_x,
	output wire [8:0]  so_data,      // one cycle after an accepted so_rd

	// --- savestate line-buffer window (from k2ge_vram) ---------------------
	input  wire        ss_lb_sel,
	input  wire [9:0]  ss_lb_addr,   // byte offset 0..639
	input  wire [7:0]  ss_lb_wdata,
	input  wire        ss_lb_wren,
	output wire [7:0]  ss_lb_rdata,  // one cycle after the address

	// --- savestate: the sweep flag lives in internals word 0 ---------------
	output wire        lb_init,
	input  wire        ss_init_val,
	input  wire        ss_init_wren
);

	// Savestate window decode
	// 320 entries as 640 bytes.  The subtraction folds the second half back to
	// x = 0..159: for w in 160..319 the true difference fits eight bits, so the
	// 8-bit wrap is exact and no 9-bit adder is needed.

	wire [8:0] ss_word = ss_lb_addr[9:1];
	wire       ss_half = (ss_word >= 9'd160);
	wire [7:0] ss_x    = ss_half ? (ss_word[7:0] - 8'd160) : ss_word[7:0];
	wire [8:0] ss_ra   = {ss_half, ss_x};

	// The tap owns port A whenever it is addressing this window, gated by
	// neither `ce` nor anything else - it has to work while the chip is frozen.
	wire tap_use = ss_lb_sel;

	// Producer rotation
	// Round-robin over the requesters, one grant every two cycles.  `rr_q`
	// names the producer served last, and the search starts at the one after
	// it, so a producer that keeps asking cannot lock the others out.

	localparam [1:0] P_S1   = 2'd0;
	localparam [1:0] P_S2   = 2'd1;
	localparam [1:0] P_OB   = 2'd2;
	localparam [1:0] P_NONE = 2'd3;

	reg  [1:0] rr_q;

	reg        op_q;        // 1 = this cycle is the write half of an op
	reg  [8:0] op_addr_q;
	reg  [8:0] op_data_q;
	reg        op_wr_q;

	wire [2:0] req = {ob_req, s2_req, s1_req};

	reg  [1:0] pick;

	always @* begin
		case (rr_q)
			P_S1: begin
				if      (req[1]) pick = P_S2;
				else if (req[2]) pick = P_OB;
				else if (req[0]) pick = P_S1;
				else             pick = P_NONE;
			end
			P_S2: begin
				if      (req[2]) pick = P_OB;
				else if (req[0]) pick = P_S1;
				else if (req[1]) pick = P_S2;
				else             pick = P_NONE;
			end
			default: begin
				if      (req[0]) pick = P_S1;
				else if (req[1]) pick = P_S2;
				else if (req[2]) pick = P_OB;
				else             pick = P_NONE;
			end
		endcase
	end

	wire grant = ce && !op_q && !tap_use && (pick != P_NONE);

	assign s1_ready = grant && (pick == P_S1);
	assign s2_ready = grant && (pick == P_S2);
	assign ob_ready = grant && (pick == P_OB);

	// The granted producer's dot, selected on the grant cycle.  Registered
	// outputs of one module through a 3:1 mux into a RAM address - the
	// shortest path a shared write port can be built from.
	reg  [7:0] g_x;
	reg  [8:0] g_data;
	reg        g_wr;

	always @* begin
		case (pick)
			P_S1: begin
				g_x    = s1_x;
				g_data = s1_data;
				g_wr   = s1_wr;
			end
			P_S2: begin
				g_x    = s2_x;
				g_data = s2_data;
				g_wr   = s2_wr;
			end
			default: begin
				g_x    = ob_x;
				g_data = ob_data;
				g_wr   = ob_wr;
			end
		endcase
	end

	// The composed half is the one the scanout is not displaying.
	wire [8:0] g_addr = {~buf_sel, g_x};

	// Port B users: scanout read-and-clear, and the one-shot sweep

	reg        clr_pend_q;
	reg  [8:0] clr_addr_q;

	reg        lb_init_q;
	reg        sweep_run_q;
	reg  [8:0] sweep_addr_q;

	wire so_ready   = !clr_pend_q;

	wire so_take    = ce && so_rd && so_ready;
	wire clr_go     = clr_pend_q && !so_take;
	wire sweep_go   = sweep_run_q && !so_take && !clr_pend_q;

	// The block RAM

	reg  [8:0]  addr_a;
	reg         wren_a;
	reg  [1:0]  be_a;
	reg  [15:0] wdata_a;
	wire [15:0] q_a;

	reg  [8:0]  addr_b;
	reg         wren_b;
	wire [15:0] q_b;

	// The rank compare.  Strict: an equal rank never overwrites, which is what
	// puts sprite 0 on top of every later sprite in its class.
	wire rank_gt = (op_data_q[8:6] > q_a[8:6]);

	always @* begin
		if (tap_use) begin
			addr_a  = ss_ra;
			wren_a  = ss_lb_wren;
			be_a    = {ss_lb_addr[0], ~ss_lb_addr[0]};
			// The odd byte carries only entry bit 8; writing it zeroes bits
			// 15:9 so the packed word stays canonical and a round trip is
			// idempotent.
			wdata_a = {7'd0, ss_lb_wdata[0], ss_lb_wdata};
		end else if (op_q) begin
			addr_a  = op_addr_q;
			wren_a  = ce && op_wr_q && rank_gt;
			be_a    = 2'b11;
			wdata_a = {7'd0, op_data_q};
		end else begin
			addr_a  = g_addr;
			wren_a  = 1'b0;
			be_a    = 2'b11;
			wdata_a = 16'd0;
		end
	end

	always @* begin
		if (so_take) begin
			addr_b = {buf_sel, so_x};
			wren_b = 1'b0;
		end else if (clr_go) begin
			addr_b = clr_addr_q;
			wren_b = 1'b1;
		end else begin
			addr_b = sweep_addr_q;
			wren_b = sweep_go;
		end
	end

	cache_ram_dp_be #(
		.ADDR_WIDTH (9),
		.DATA_WIDTH (16)
	) u_lbuf (
		.clk_i     (clk_sys),
		.addr_a_i  (addr_a),
		.wren_a_i  (wren_a),
		.be_a_i    (be_a),
		.wdata_a_i (wdata_a),
		.q_a_o     (q_a),
		.addr_b_i  (addr_b),
		.wren_b_i  (wren_b),
		.be_b_i    (2'b11),
		.wdata_b_i (16'd0),
		.q_b_o     (q_b)
	);

	// Port A sequencing
	// The tap cancels a pending write rather than completing it: by the time
	// the walker runs, the chip has been quiesced at a line boundary, so there
	// is nothing to cancel - and if there ever were, the compare it would have
	// used has already been overwritten by the tap's own read.

	always @(posedge clk_sys) begin
		if (rst) begin
			rr_q      <= P_OB;
			op_q      <= 1'b0;
			op_addr_q <= 9'd0;
			op_data_q <= 9'd0;
			op_wr_q   <= 1'b0;
		end else if (tap_use) begin
			op_q      <= 1'b0;
		end else if (ce) begin
			op_q <= 1'b0;
			if (grant) begin
				rr_q      <= pick;
				op_q      <= 1'b1;
				op_addr_q <= g_addr;
				op_data_q <= g_data;
				op_wr_q   <= g_wr;
			end
		end
	end

	// Port B sequencing

	always @(posedge clk_sys) begin
		if (rst) begin
			clr_pend_q   <= 1'b0;
			clr_addr_q   <= 9'd0;
			lb_init_q    <= 1'b1;
			sweep_run_q  <= 1'b0;
			sweep_addr_q <= 9'd0;
		end else begin
			if (ss_init_wren) lb_init_q <= ss_init_val;

			if (ce) begin
				if (so_take) begin
					clr_pend_q <= 1'b1;
					clr_addr_q <= {buf_sel, so_x};
				end else if (clr_go) begin
					clr_pend_q <= 1'b0;
				end

				// One shot, during the first V blank only.  Running it again
				// would erase the line-0 buffer the pass on line 198 wrote.
				if (sweep_run_q) begin
					if (sweep_go) begin
						sweep_addr_q <= sweep_addr_q + 9'd1;
						if (sweep_addr_q == 9'd511) begin
							sweep_run_q <= 1'b0;
							if (!ss_init_wren) lb_init_q <= 1'b0;
						end
					end
				end else if (lb_init_q && blnk) begin
					sweep_run_q  <= 1'b1;
					sweep_addr_q <= 9'd0;
				end
			end
		end
	end

	assign so_data  = q_b[8:0];
	assign lb_init  = lb_init_q;

	// Savestate read path
	// The RAM answers one cycle after the address, so the lane bit is
	// registered alongside it and the byte is a plain mux of the RAM output.
	// That is the k2ge_vram contract met literally, not approximately.

	reg ss_lane_q;

	always @(posedge clk_sys) begin
		if (rst) ss_lane_q <= 1'b0;
		else     ss_lane_q <= ss_lb_addr[0];
	end

	assign ss_lb_rdata = ss_lane_q ? {7'd0, q_a[8]} : q_a[7:0];

	// Deliberately unread inputs
	// Only bit 0 of an odd savestate byte exists, and the packed word's high
	// seven bits are always zero by construction.
	wire unused_ok = &{1'b0, ss_lb_wdata[7:1], q_a[15:9], q_b[15:9], 1'b0};

endmodule
