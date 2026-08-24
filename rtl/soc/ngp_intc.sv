// Copyright (c) 2026 Jamie Blanks

// NGP / TMP95C061-class interrupt controller and micro-DMA request matcher.
// Owns SFRs 0x70-0x7F.
//
// The datasheet block diagram (TMP95C061 datasheet p.18) is one request
// flip-flop, one priority-level field and one micro-DMA start-vector comparison
// per channel, feeding a priority network that hands the CPU a single winner.
// This module is built the same way:
//
//   22 request flip-flops   one per maskable channel (INT0..INTTC3)
//    2 non-maskable latches  NMI pin, watchdog INTWD, both fixed level 7
//   22 three-bit levels      the write-only halves of INTE0AD..INTETC23
//    4 start-vector regs     DMA0V-DMA3V, five bits each
//    1 priority tree         32-leaf balanced tree of magnitude comparators
//
// The datasheet's "total of 24 channels" (p.17) is the 22 maskable ones plus NMI
// and INTWD; there are eleven INTE bytes, so only 22 maskable flip-flops exist.
// Savestate word 0x04 is 24 bits wide and its top two bits read 0.
//
// Slots are numbered in vector order so that "smaller vector wins a tie" (p.17)
// is the same thing as "smaller slot wins a tie":
//
//   slot  0- 4  INT0, INT4, INT5, INT6, INT7        vectors 0x28-0x38
//   slot  5- 8  INTT0-INTT3    (ngp_t8)             vectors 0x40-0x4C
//   slot  9-12  INTTR4-INTTR7  (ngp_t16)            vectors 0x50-0x5C
//   slot 13-16  INTRX0, INTTX0, INTRX1, INTTX1      vectors 0x60-0x6C
//   slot 17     INTAD                               vector  0x70
//   slot 18-21  INTTC0-INTTC3  (micro-DMA end)      vectors 0x74-0x80
//
// The priority tree works on 5-bit ids: id 0 = NMI (vector 0x20), id 1 = INTWD
// (vector 0x24), id = slot + 2 for everything else.  The vector of an id is then
// uniformly (id + 8 or 9) * 4, which is the Toshiba vector table (p.12 Table
// 3.3(1)) with its one hole at index 0x0F.
//
// Bus timing:
//   * sfr_rdata is combinational; nothing in the read path depends on ce.
//   * sfr_wr / sfr_rd are one-clk_sys strobes and are not additionally gated by
//     ce.  The strobe is itself the enable; re-gating it with ce would drop
//     writes.  This is the one place a clocked block here is not ce-gated.
//   * Write-only registers (IIMC, DMA0V-3V) read 0xFF on the CPU bus; their
//     shadows appear only on the savestate tap.  The INTE bytes are readable:
//     the flag bit reports the request flip-flop (p.17, "the status of the
//     interrupt request flip-flop is detected by reading the clear bit"), and
//     the level bits read as all-ones -- they do not echo the stored level.
//   * Everything that models the passage of time -- the pin edge detectors and
//     the acceptance of a source pulse -- is ce-gated, so freezing the machine
//     for a savestate is just gating the enables.
//
// The priority tree is registered on every clk_sys edge and deliberately not
// ce-gated: a 32-leaf comparator tree is a long combinational chain, and there
// are eight clk_sys cycles inside one machine state to hide the register in.
// The visible consequence is one machine state of latency -- a flip-flop that
// sets on the ce edge of state N is presented to the CPU at state N+1, which is
// what silicon does too (p.24, "an interrupt request is sampled with the rising
// edge of the CLK signal").  Gating this register with ce would cost a second
// state, because the CPU samples int_req on the same edge the register updates.
//
// A request flip-flop that is set and cleared on the same clk_sys edge stays
// SET.  A source edge is a physical event that must not be swallowed by an
// acceptance or a software clear landing on the same edge; a lost interrupt is
// worse than an extra one.  No document states how silicon resolves this.
//
// Behaviour that is not fully pinned down, each marked at its use site:
//   1. IIMC<I0LE> polarity: parameter I0LE_SELECTS_LEVEL, see below.
//   2. INT4/INT6 edge selection from CAP12M/CAP34M: rising except in capture
//      mode 10, which is the mode that captures the falling edge of the pin.
//   3. The Port B "pin is an input" gate is applied to all four of INT4-INT7,
//      which are built the same way, rather than to INT4/INT5 only.
//   4. Micro-DMA matching ignores the source's interrupt level (the micro-DMA
//      application note recommends level 0 for DMA-only sources).

module ngp_intc
#(
	// IIMC<I0LE> chooses how INT0 is sensed.  The datasheet register figure
	// reads as 0 = rising edge, 1 = high level, and the BIOS writes IIMC = 0x04
	// (edge) for the RTC alarm.  The figure is an image in the available PDF and
	// the opposite reading cannot be excluded, so set this to 0 to invert.
	parameter I0LE_SELECTS_LEVEL = 1
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	// SFR bus, owns 0x70-0x7F
	input  wire [6:0]  sfr_addr,
	input  wire [7:0]  sfr_wdata,
	input  wire        sfr_wr,
	input  wire        sfr_rd,
	output wire [7:0]  sfr_rdata,

	// non-maskable sources
	input  wire        nmi_n,          // power button, already gated by 0xB3.2
	input  wire        intwd,          // one-ce pulse from ngp_wdt on overflow

	// external / SoC maskable sources
	input  wire        int0,           // RTC alarm (ngp_rtc)
	input  wire        int4,           // K2GE VBlank
	input  wire        int5,           // Z80 wrote 0xC000
	input  wire        int6,           // unbonded on NGP, tie 0
	input  wire        int7,           // panel-switch clock-gear regen
	input  wire [3:0]  pb_is_input,    // PBCR bits 0,1,4,5 = 0 -> pin is input
	input  wire [1:0]  cap12m,         // T4MOD[4:3], selects INT4 edge
	input  wire [1:0]  cap34m,         // T5MOD[4:3], selects INT6 edge

	// on-chip maskable sources (one-ce pulses)
	input  wire [3:0]  intt,           // INTT0-3   from ngp_t8
	input  wire [3:0]  inttr,          // INTTR4-7  from ngp_t16
	input  wire [1:0]  intrx,          // INTRX0/1  from ngp_sio
	input  wire [1:0]  inttx,          // INTTX0/1  from ngp_sio
	// SCnBUF was read this cycle -> clear INTRXn, the same side effect ADREGnH
	// has on IADC (TMP95C061 datasheet p.137, p.152).
	input  wire [1:0]  intrx_clear,
	input  wire        intad,          // from ngp_adc
	input  wire        intad_clear,    // ADREGnH read this cycle -> clear IADC

	// CPU interrupt interface (mates t900_cpu)
	output wire        int_req,
	output wire [2:0]  int_level,
	output wire [7:0]  int_vector,
	// The identity half of the handshake.  `int_id` names the source this
	// controller is presenting; `int_ack_id` is that identity handed back by
	// the CPU on the acknowledge.  The clear and the vector both follow
	// `int_ack_id`, never the live winner -- see the acknowledge note below.
	output wire [4:0]  int_id,
	input  wire        int_ack,
	input  wire [4:0]  int_ack_id,

	// CPU micro-DMA interface (mates t900_cpu)
	output wire [3:0]  dma_req,
	input  wire [3:0]  dma_ack,
	input  wire [3:0]  dma_end,

	// standby
	output wire        halt_release,

	// savestate tap, words 0x00-0x07
	input  wire [7:0]  ss_reg_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	// Slot names

	localparam integer SL_INT0   = 0;
	localparam integer SL_INT4   = 1;
	localparam integer SL_INT5   = 2;
	localparam integer SL_INT6   = 3;
	localparam integer SL_INT7   = 4;
	localparam integer SL_INTT0  = 5;   // .. 8
	localparam integer SL_INTTR4 = 9;   // .. 12
	localparam integer SL_INTRX0 = 13;  // .. 16
	localparam integer SL_INTAD  = 17;
	localparam integer SL_INTTC0 = 18;  // .. 21

	localparam integer N_SLOT = 22;

	// State

	reg  [2:0]  lvl [0:N_SLOT-1];   // INTE level fields, write-only in silicon
	reg  [21:0] req_ff;             // maskable request flip-flops
	reg  [2:0]  iimc;               // {I0IE, I0LE, NMIREE}
	reg  [4:0]  dmav [0:3];         // DMA0V-DMA3V, start vector / 4

	reg         nmi_ff;             // non-maskable latches, fixed level 7
	reg         intwd_ff;

	reg         nmi_prev;           // pin history for the edge conditioners
	reg         int0_prev;
	reg         int4_prev;
	reg         int5_prev;
	reg         int6_prev;
	reg         int7_prev;

	reg         win_valid;          // registered priority-tree result
	reg  [2:0]  win_level;
	reg  [4:0]  win_id;

	integer i;

	// Address decode helpers
	// The eleven INTE bytes hold two channels each: the low nibble is source
	// A, the high nibble source B (TMP95C061 datasheet pp.184-185).

	function automatic [4:0] slot_lo(input [3:0] r);
	begin
		case (r)
			4'h0:    slot_lo = 5'd0;                        // INTE0AD  <- INT0
			4'h1:    slot_lo = 5'd1;                        // INTE45   <- INT4
			4'h2:    slot_lo = 5'd3;                        // INTE67   <- INT6
			4'h3:    slot_lo = 5'd5;                        // INTET10  <- INTT0
			4'h4:    slot_lo = 5'd7;                        // INTET32  <- INTT2
			4'h5:    slot_lo = 5'd9;                        // INTET54  <- INTTR4
			4'h6:    slot_lo = 5'd11;                       // INTET76  <- INTTR6
			4'h7:    slot_lo = 5'd13;                       // INTES0   <- INTRX0
			4'h8:    slot_lo = 5'd15;                       // INTES1   <- INTRX1
			4'h9:    slot_lo = 5'd18;                       // INTETC01 <- INTTC0
			default: slot_lo = 5'd20;                       // INTETC23 <- INTTC2
		endcase
	end
	endfunction

	function automatic [4:0] slot_hi(input [3:0] r);
	begin
		case (r)
			4'h0:    slot_hi = 5'd17;                       // INTE0AD  <- INTAD
			4'h1:    slot_hi = 5'd2;                        // INTE45   <- INT5
			4'h2:    slot_hi = 5'd4;                        // INTE67   <- INT7
			4'h3:    slot_hi = 5'd6;                        // INTET10  <- INTT1
			4'h4:    slot_hi = 5'd8;                        // INTET32  <- INTT3
			4'h5:    slot_hi = 5'd10;                       // INTET54  <- INTTR5
			4'h6:    slot_hi = 5'd12;                       // INTET76  <- INTTR7
			4'h7:    slot_hi = 5'd14;                       // INTES0   <- INTTX0
			4'h8:    slot_hi = 5'd16;                       // INTES1   <- INTTX1
			4'h9:    slot_hi = 5'd19;                       // INTETC01 <- INTTC1
			default: slot_hi = 5'd21;                       // INTETC23 <- INTTC3
		endcase
	end
	endfunction

	// Vector of a priority-tree id.  id 0 = NMI (0x20), id 1 = INTWD (0x24),
	// id 2.. = slot 0.. .  Channel index 0x0F does not exist, which is the
	// step in the addition (TMP95C061 datasheet p.12 Table 3.3(1)).
	function automatic [7:0] vec_of_id(input [4:0] id);
		reg [5:0] idx;
	begin
		idx        = (id <= 5'd6) ? ({1'b0, id} + 6'd8) : ({1'b0, id} + 6'd9);
		vec_of_id  = {idx, 2'b00};
	end
	endfunction

	// Micro-DMA start vector index -> {valid, slot}.  Valid indices are
	// 0x0A-0x1C excluding 0x0F: INT0 through INTAD.  The INTTC channels have
	// no start vector (TMP95C061 datasheet p.12 table; micro-DMA application
	// note p.1).
	function automatic [5:0] slot_of_vecidx(input [4:0] vi);
	begin
		if ((vi >= 5'd10) && (vi <= 5'd14)) begin
			slot_of_vecidx = {1'b1, vi - 5'd10};
		end else if ((vi >= 5'd16) && (vi <= 5'd28)) begin
			slot_of_vecidx = {1'b1, vi - 5'd11};
		end else begin
			slot_of_vecidx = {1'b0, 5'd0};
		end
	end
	endfunction

	wire in_range = (sfr_addr[6:4] == 3'h7);
	wire is_inte  = in_range && (sfr_addr[3:0] <= 4'hA);
	wire is_iimc  = in_range && (sfr_addr[3:0] == 4'hB);
	wire is_dmav  = in_range && (sfr_addr[3:2] == 2'b11);

	wire inte_wr  = sfr_wr && is_inte;

	// CPU-side readback (combinational, never ce-dependent)

	wire [4:0] rd_lo = slot_lo(sfr_addr[3:0]);
	wire [4:0] rd_hi = slot_hi(sfr_addr[3:0]);

	// IIMC and DMA0V-3V are write-only and read 0xFF, as do the addresses
	// outside this module's window (the shell never routes them here, but the
	// mux arm must still answer).
	//
	// An INTE register reads its FLAG bits true and its LEVEL fields as
	// all-ones; the level latches are write-only on silicon and do not echo the
	// stored level.  Emulators generally echo the level back instead, so any
	// game analysis leaning on a read-modify-write INTE idiom is suspect.
	assign sfr_rdata = is_inte ? {req_ff[rd_hi], 3'b111, req_ff[rd_lo], 3'b111}
	                           : 8'hFF;

	// Savestate tap
	// Word 0x00-0x02 carry the INTE level fields and IIMC, word 0x03 the four
	// start vectors, word 0x04 the request flip-flops and word 0x05 the pin
	// history and the two non-maskable latches.  The INTE images report the
	// level fields only, with the flag bits reading 0: the flags live in word
	// 0x04 and exactly one word must own each piece of state, so that a
	// read-all / write-all round trip restores the controller exactly.

	function automatic [7:0] inte_img(input [3:0] r);
	begin
		inte_img = {1'b0, lvl[slot_hi(r)], 1'b0, lvl[slot_lo(r)]};
	end
	endfunction

	reg [31:0] ss_rdata_r;

	always @* begin
		case (ss_reg_addr)
			8'h00:   ss_rdata_r = {inte_img(4'h0), inte_img(4'h1), inte_img(4'h2), inte_img(4'h3)};
			8'h01:   ss_rdata_r = {inte_img(4'h4), inte_img(4'h5), inte_img(4'h6), inte_img(4'h7)};
			8'h02:   ss_rdata_r = {inte_img(4'h8), inte_img(4'h9), inte_img(4'hA), 5'd0, iimc};
			8'h03:   ss_rdata_r = {3'd0, dmav[0], 3'd0, dmav[1], 3'd0, dmav[2], 3'd0, dmav[3]};
			8'h04:   ss_rdata_r = {10'd0, req_ff};
			8'h05:   ss_rdata_r = {24'd0, int7_prev, int6_prev, int5_prev, int4_prev,
			                       int0_prev, nmi_prev, intwd_ff, nmi_ff};
			default: ss_rdata_r = 32'd0;   // 0x06-0x07 reserved, read 0
		endcase
	end

	assign ss_rdata    = ss_rdata_r;
	assign pause_ready = 1'b1;            // no multi-cycle transaction to drain

	wire ss_wr = ss_wren && pause_req;    // honoured only while paused

	// Register file: INTE levels, IIMC, DMA0V-3V

	// Byte lanes of savestate words 0x00-0x02 map to INTE register indices
	// 0-10 in address order, four per word.
	wire [3:0] ss_r0 = {ss_reg_addr[1:0], 2'b00};
	wire [3:0] ss_r1 = ss_r0 + 4'd1;
	wire [3:0] ss_r2 = ss_r0 + 4'd2;
	wire [3:0] ss_r3 = ss_r0 + 4'd3;

	always @(posedge clk) begin
		if (reset) begin
			// Chip reset (TMP95C061 datasheet pp.184-185): every INTE byte 0x00, IIMC 0x00,
			// DMA0V-DMA3V 0x00.
			for (i = 0; i < N_SLOT; i = i + 1) begin
				lvl[i] <= 3'd0;
			end
			iimc <= 3'd0;
			for (i = 0; i < 4; i = i + 1) begin
				dmav[i] <= 5'd0;
			end
		end else if (ss_wr && (ss_reg_addr <= 8'h03)) begin
			if (ss_reg_addr == 8'h03) begin
				dmav[0] <= ss_wdata[28:24];
				dmav[1] <= ss_wdata[20:16];
				dmav[2] <= ss_wdata[12:8];
				dmav[3] <= ss_wdata[4:0];
			end else begin
				if (ss_r0 <= 4'hA) begin
					lvl[slot_lo(ss_r0)] <= ss_wdata[26:24];
					lvl[slot_hi(ss_r0)] <= ss_wdata[30:28];
				end
				if (ss_r1 <= 4'hA) begin
					lvl[slot_lo(ss_r1)] <= ss_wdata[18:16];
					lvl[slot_hi(ss_r1)] <= ss_wdata[22:20];
				end
				if (ss_r2 <= 4'hA) begin
					lvl[slot_lo(ss_r2)] <= ss_wdata[10:8];
					lvl[slot_hi(ss_r2)] <= ss_wdata[14:12];
				end
				if (ss_r3 <= 4'hA) begin
					lvl[slot_lo(ss_r3)] <= ss_wdata[2:0];
					lvl[slot_hi(ss_r3)] <= ss_wdata[6:4];
				end
				if (ss_reg_addr == 8'h02) begin
					iimc <= ss_wdata[2:0];
				end
			end
		end else begin
			if (inte_wr) begin
				lvl[rd_lo] <= sfr_wdata[2:0];
				lvl[rd_hi] <= sfr_wdata[6:4];
			end
			if (sfr_wr && is_iimc) begin
				iimc <= sfr_wdata[2:0];
			end
			if (sfr_wr && is_dmav) begin
				dmav[sfr_addr[1:0]] <= sfr_wdata[4:0];
			end
			// Terminal count disarms the channel, which is what makes
			// micro-DMA one-shot (TMP95C061 datasheet p.21).  Placed last so the
			// hardware clear wins a same-cycle CPU re-arm; the two never
			// collide in practice because software re-arms inside the INTTC
			// handler, several states later.
			for (i = 0; i < 4; i = i + 1) begin
				if (dma_end[i]) begin
					dmav[i] <= 5'd0;
				end
			end
		end
	end

	// External pin conditioning (TMP95C061 datasheet p.20)

	wire i0ie       = iimc[2];
	wire i0_level   = (I0LE_SELECTS_LEVEL != 0) ? iimc[1] : ~iimc[1];
	wire nmiree     = iimc[0];

	// INT4/INT6 double as the TI4/TI6 capture inputs, so their edge follows
	// the capture-mode field.  Mode 10 is the one that captures the falling
	// edge of the pin (CAP12M 00 = disabled, 01 = TI4 up/TI5 up, 10 = TI4 up/TI4
	// down, 11 = TFF1 both edges); every other mode leaves the interrupt on the
	// rising edge.  The datasheet's pin-function table is an image in the
	// available copy, so this reading is not fully certain.
	wire int4_falling = (cap12m == 2'b10);
	wire int6_falling = (cap34m == 2'b10);

	wire nmi_edge   = (nmi_prev && !nmi_n) || (nmiree && !nmi_prev && nmi_n);
	wire int0_edge  = !int0_prev && int0;
	wire int4_edge  = int4_falling ? (int4_prev && !int4) : (!int4_prev && int4);
	wire int5_edge  = !int5_prev && int5;
	wire int6_edge  = int6_falling ? (int6_prev && !int6) : (!int6_prev && int6);
	wire int7_edge  = !int7_prev && int7;

	// INT0 is only accepted when IIMC<I0IE> = 1; otherwise PB7 is a plain port
	// pin.  In level mode the flip-flop follows the level, so it also clears
	// when the level goes away.
	wire int0_set   = ce && i0ie && (i0_level ? int0 : int0_edge);
	wire int0_lvl_c = ce && i0ie && i0_level && !int0;

	wire nmi_set    = ce && nmi_edge;
	// INTWD is a strobe from ngp_wdt, which registers it; see the note on
	// set_req below for why it is not re-gated by ce.
	wire intwd_set  = intwd;

	// Request flip-flops

	reg  [21:0] set_req;
	reg  [21:0] clr_req;

	wire [3:0]  dma_hit;
	wire [19:0] dma_slot;

	// The five external inputs are LEVELS, sampled on ce so that one machine
	// state is one sample; the conditioners above turn them into edges.
	//
	// The on-chip inputs are STROBES, one clk_sys cycle wide, produced by a
	// peripheral on a ce tick.  They are deliberately NOT re-gated by ce
	// here.  Whether a strobe is high on the same clk_sys cycle as its ce
	// tick or on the next one depends on whether the producing module
	// registers its output -- ngp_t8 drives intt combinationally, while
	// ngp_adc, ngp_sio and ngp_wdt register intad / intrx / inttx / intwd and
	// so present them one cycle later, so gating with ce would drop those three
	// sources entirely.  Setting an already-set flip-flop is idempotent, so
	// accepting the strobe on any cycle costs nothing and removes a phase
	// coupling between modules that no document asks for.
	always @* begin
		set_req            = 22'd0;
		set_req[SL_INT0]   = int0_set;
		set_req[SL_INT4]   = ce && pb_is_input[0] && int4_edge;
		set_req[SL_INT5]   = ce && pb_is_input[1] && int5_edge;
		set_req[SL_INT6]   = ce && pb_is_input[2] && int6_edge;
		set_req[SL_INT7]   = ce && pb_is_input[3] && int7_edge;
		for (i = 0; i < 4; i = i + 1) begin
			set_req[SL_INTT0  + i] = intt[i];
			set_req[SL_INTTR4 + i] = inttr[i];
			// INTTCn is raised by the CPU's terminal-count strobe.
			set_req[SL_INTTC0 + i] = dma_end[i];
		end
		set_req[SL_INTRX0 + 0] = intrx[0];
		set_req[SL_INTRX0 + 1] = inttx[0];
		set_req[SL_INTRX0 + 2] = intrx[1];
		set_req[SL_INTRX0 + 3] = inttx[1];
		set_req[SL_INTAD]      = intad;
	end

	// INTAD and INTRX0/INTRX1 request flip-flops cannot be cleared by an
	// instruction (TMP95C061 datasheet p.22 note); only acceptance or micro-DMA
	// consumes them, and the clears below are unaffected because the note is
	// about instruction writes.  The TX slots stay clearable.
	function insn_clear_blocked(input integer slot);
	begin
		insn_clear_blocked = (slot == SL_INTAD) ||
		                     (slot == SL_INTRX0) ||
		                     (slot == SL_INTRX0 + 2);
	end
	endfunction

	always @* begin
		clr_req = 22'd0;
		// A CPU write of 0 to a flag bit clears that request flip-flop;
		// writing 1 leaves it alone (TMP95C061 datasheet p.17, p.19) -- except the slots
		// insn_clear_blocked names, whose FFs only acceptance or micro-DMA
		// can consume.
		if (inte_wr) begin
			clr_req[rd_lo] = ~sfr_wdata[3] && !insn_clear_blocked({27'd0, rd_lo});
			clr_req[rd_hi] = ~sfr_wdata[7] && !insn_clear_blocked({27'd0, rd_hi});
		end
		// Acceptance clears the source the CPU says it ACCEPTED, on the same
		// edge the CPU strobes int_ack (TMP95C061 datasheet p.17).  Not the live
		// winner: the priority tree republishes on every clk_sys edge and the CPU
		// decides on a snapshot, so "whoever is winning right now" and "whoever
		// the CPU is entering the handler for" are not the same source.
		if (int_ack && (int_ack_id >= 5'd2)) begin
			clr_req[int_ack_id - 5'd2] = 1'b1;
		end
		// A micro-DMA start consumes the source's request, exactly as the
		// interrupt path would have.
		for (i = 0; i < 4; i = i + 1) begin
			if (dma_ack[i] && dma_hit[i]) begin
				clr_req[dma_slot[i*5 +: 5]] = 1'b1;
			end
		end
		// Reading ADREGnH clears the INTAD request (TMP95C061 datasheet p.152
		// item (8)), and reading SCnBUF clears that channel's INTRXn (p.137).  Slots
		// SL_INTRX0+0 and SL_INTRX0+2 are INTRX0 and INTRX1; the odd slots
		// between them are the transmit channels, which have no such clear.
		if (intad_clear) begin
			clr_req[SL_INTAD] = 1'b1;
		end
		if (intrx_clear[0]) begin
			clr_req[SL_INTRX0 + 0] = 1'b1;
		end
		if (intrx_clear[1]) begin
			clr_req[SL_INTRX0 + 2] = 1'b1;
		end
		if (int0_lvl_c) begin
			clr_req[SL_INT0] = 1'b1;
		end
	end

	always @(posedge clk) begin
		if (reset) begin
			req_ff <= 22'd0;
		end else if (ss_wr && (ss_reg_addr == 8'h04)) begin
			req_ff <= ss_wdata[21:0];
		end else begin
			// Set beats clear: see the header.
			req_ff <= (req_ff & ~clr_req) | set_req;
		end
	end

	always @(posedge clk) begin
		if (reset) begin
			nmi_ff    <= 1'b0;
			intwd_ff  <= 1'b0;
			nmi_prev  <= 1'b1;          // NMI idles high, so no reset edge
			int0_prev <= 1'b0;
			int4_prev <= 1'b0;
			int5_prev <= 1'b0;
			int6_prev <= 1'b0;
			int7_prev <= 1'b0;
		end else if (ss_wr && (ss_reg_addr == 8'h05)) begin
			nmi_ff    <= ss_wdata[0];
			intwd_ff  <= ss_wdata[1];
			nmi_prev  <= ss_wdata[2];
			int0_prev <= ss_wdata[3];
			int4_prev <= ss_wdata[4];
			int5_prev <= ss_wdata[5];
			int6_prev <= ss_wdata[6];
			int7_prev <= ss_wdata[7];
		end else begin
			if (ce) begin
				nmi_prev  <= nmi_n;
				int0_prev <= int0;
				int4_prev <= int4;
				int5_prev <= int5;
				int6_prev <= int6;
				int7_prev <= int7;
			end
			if (nmi_set) begin
				nmi_ff <= 1'b1;
			end else if (int_ack && (int_ack_id == 5'd0)) begin
				nmi_ff <= 1'b0;
			end
			if (intwd_set) begin
				intwd_ff <= 1'b1;
			end else if (int_ack && (int_ack_id == 5'd1)) begin
				intwd_ff <= 1'b0;
			end
		end
	end

	// Priority tree
	// Each leaf is {level[2:0], ~id[4:0]}, so one unsigned magnitude compare
	// picks the higher level and, among equal levels, the smaller id - which
	// is the smaller vector, the documented tie-break (TMP95C061 datasheet p.17).
	// A channel only enters with level 1-6; writing 0 or 7 disables it.

	function automatic [7:0] pick(input [7:0] a, input [7:0] b);
	begin
		pick = (a >= b) ? a : b;
	end
	endfunction

	wire [255:0] cand;
	wire [127:0] t1;
	wire [63:0]  t2;
	wire [31:0]  t3;
	wire [15:0]  t4;
	wire [7:0]   t5;

	assign cand[7:0]  = {nmi_ff   ? 3'd7 : 3'd0, ~5'd0};
	assign cand[15:8] = {intwd_ff ? 3'd7 : 3'd0, ~5'd1};

	genvar g;
	generate
		for (g = 0; g < N_SLOT; g = g + 1) begin : gen_cand
			localparam [4:0] CID = g + 2;
			wire elig = req_ff[g] && (lvl[g] != 3'd0) && (lvl[g] != 3'd7);
			assign cand[(g+2)*8 +: 8] = {elig ? lvl[g] : 3'd0, ~CID};
		end
		for (g = N_SLOT + 2; g < 32; g = g + 1) begin : gen_pad
			localparam integer PIDI = g;
			localparam [4:0]   PID  = PIDI[4:0];
			assign cand[g*8 +: 8] = {3'd0, ~PID};
		end
		for (g = 0; g < 16; g = g + 1) begin : gen_t1
			assign t1[g*8 +: 8] = pick(cand[(2*g)*8 +: 8], cand[(2*g+1)*8 +: 8]);
		end
		for (g = 0; g < 8; g = g + 1) begin : gen_t2
			assign t2[g*8 +: 8] = pick(t1[(2*g)*8 +: 8], t1[(2*g+1)*8 +: 8]);
		end
		for (g = 0; g < 4; g = g + 1) begin : gen_t3
			assign t3[g*8 +: 8] = pick(t2[(2*g)*8 +: 8], t2[(2*g+1)*8 +: 8]);
		end
		for (g = 0; g < 2; g = g + 1) begin : gen_t4
			assign t4[g*8 +: 8] = pick(t3[(2*g)*8 +: 8], t3[(2*g+1)*8 +: 8]);
		end
	endgenerate

	assign t5 = pick(t4[7:0], t4[15:8]);

	wire [2:0] win_level_c = t5[7:5];
	wire [4:0] win_id_c    = ~t5[4:0];
	wire       win_valid_c = (t5[7:5] != 3'd0);

	always @(posedge clk) begin
		if (reset) begin
			win_valid <= 1'b0;
			win_level <= 3'd0;
			win_id    <= 5'd0;
		end else begin
			win_valid <= win_valid_c;
			win_level <= win_level_c;
			win_id    <= win_id_c;
		end
	end

	// The 0x28 default-vector race (TMP95C061 datasheet p.22 Notes)
	// If the instruction that clears the winner's request flag reaches the bus
	// in the very cycle the CPU reads the vector, silicon hands back the
	// default vector 0x0028 and the handler runs from 0xFFFF28.  Only the
	// vector changes: the acceptance still happens and the flip-flop still
	// clears.

	// The vector answers for the source being ACKNOWLEDGED while an acknowledge
	// is in progress, and for the presented winner otherwise.  Toshiba's step 1
	// is one transaction -- "read the interrupt vector ... and clear that
	// source's request flip-flop" (TMP95C061 datasheet p.10-11) -- so the vector
	// read and the clear have to name the same source.  Selecting on int_ack is
	// what makes them do that when the CPU accepted a snapshot rather than the
	// live winner, and it is also what lets a split CPU observe the 0x28 race.
	wire [4:0] vec_id       = int_ack ? int_ack_id : win_id;
	wire       vec_maskable = (vec_id >= 5'd2);
	wire [4:0] vec_slot     = vec_id - 5'd2;

	// A blocked slot's flag write is a complete no-op (the FF does not clear --
	// see insn_clear_blocked), so it cannot race the vector read either: the
	// 0x28 race exists because the clear and the read name the same source, and
	// for these slots there is no clear.
	wire clr_hits_winner = vec_maskable && inte_wr &&
	                       !insn_clear_blocked({27'd0, vec_slot}) &&
	                       (((rd_lo == vec_slot) && !sfr_wdata[3]) ||
	                        ((rd_hi == vec_slot) && !sfr_wdata[7]));

	assign int_req    = win_valid;
	assign int_level  = win_level;
	assign int_id     = win_id;
	assign int_vector = clr_hits_winner ? 8'h28 : vec_of_id(vec_id);

	// HALT release: Table 3.4(2) marks the "request level < IFF" column
	// x (cannot be used for halt release) for every source EXCEPT INT0 --
	// only INT0's circle allows a below-mask request to resume execution
	// with no handler; NMI/INTWD arrive at level 7 and release through the
	// CPU's own acceptance path (TMP95C061 datasheet p.28 Table 3.4(2), and the
	// p.18 block diagram: release = Reset | INT0 | NMI | accepted interrupt).
	// A level-0 (disabled) INT0 does not wake either -- the table's rows are per
	// enabled source.
	assign halt_release = req_ff[SL_INT0] && (lvl[SL_INT0] != 3'd0);

	// Micro-DMA request matching
	// A channel is armed when its start-vector register is non-zero, and it
	// requests while the source whose vector index equals DMAnV has its
	// request flip-flop set (TMP95C061 datasheet p.21).  The source's interrupt
	// LEVEL is deliberately not part of the match: the micro-DMA application note
	// tells programmers to set the level to 0 for DMA-only sources, which only
	// works if the match looks at the flip-flop alone.
	//
	// Chaining needs no logic here.  Two channels holding the same start
	// vector both request; the CPU's fixed channel priority runs the lower one
	// to completion, and its terminal count zeroes only its own DMAnV, so the
	// higher channel simply takes over (TMP95C061 datasheet p.21).

	generate
		for (g = 0; g < 4; g = g + 1) begin : gen_dma
			wire [5:0] dec = slot_of_vecidx(dmav[g]);
			assign dma_hit[g]           = dec[5];
			assign dma_slot[g*5 +: 5]   = dec[4:0];
			assign dma_req[g]           = (dmav[g] != 5'd0) && dec[5] && req_ff[dec[4:0]];
		end
	endgenerate

	// This module has no read side effects, so sfr_rd is observed but unused; the
	// port stays for symmetry with the rest of the SFR bus.  Only some bits of
	// each savestate word are meaningful.
	wire unused_ok = &{1'b0, sfr_rd, ss_wdata};

endmodule
