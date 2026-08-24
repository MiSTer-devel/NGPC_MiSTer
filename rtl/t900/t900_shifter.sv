// Copyright (c) 2026 Jamie Blanks

// TLCS-900/H shift and rotate unit (RLC RRC RL RR SLA SRA SLL SRL).
//
// One Toshiba detail page per instruction: RL (CPU900H p.130), RLC (p.131),
// RR (p.133), RRC (p.134), SLA (p.141), SLL (p.142), SRA (p.143), SRL (p.144).
// The summary table "900/H Instruction Lists (8/10) Rotate and Shift" (p.165)
// carries all eight rows together and agrees with the detail pages; it is the
// tie-breaker cited below where a detail page contradicts itself.
//
// Toshiba charges the register forms (#4,r and A,r) "3 + n/4" states, with
// n the shift count 1..16 (CPU900H p.165). The division is a truncating
// integer division, so the state cost steps at n = 4, 8, 12, 16.
//
// That budget says the hardware moves the operand four bit positions per
// state, so this unit is a 4-position barrel slice iterated once per ce.
//
// The load tick performs the remainder (n mod 4) and each following tick
// moves 4, so ticks from start to done = 1 + floor(n/4), which supplies
// Toshiba's n/4 term.
//
// The caller normalizes the count.  The ISA rule (CPU900H p.165 note 1, and the
// Note on every detail page):
// the shift count comes either from a 4-bit immediate (#4) or from the low
// four bits of register A, is taken modulo 16, and a code of 0 means 16
// shifts -- not 0. The (mem) forms shift exactly once regardless.
//
// This module does NOT implement that rule. `count` is the already
// normalized value 1..16; the sequencer applies "value = code[3:0]; if
// (value == 0) value = 16" before starting us, and passes 1 for the (mem)
// forms. A count of 0 arriving here is a caller bug; the unit degrades
// safely to a zero-position shift (result and carry unchanged) rather than
// silently shifting once.
//
// The count is 1..16 for every operand size. A byte operand really can be
// rotated 16 times -- for RL/RR that is not a no-op, because those rotate a
// 9-bit quantity.
//
// Direction is bit 0 of the opcode, and that is exactly bit 0 of the
// second opcode byte in the ISA (E8..EF / F8..FF / 78..7F), so op[] below
// is the low three bits of the real opcode (CPU900H p.165).
//
//   RLC  rotate left, carry NOT in the ring  (CPU900H p.131)
//        CY <- dst<MSB>, dst <- left rotate of dst. The bit leaving the
//        MSB re-enters at the LSB and is also copied to CY.
//   RRC  rotate right, carry NOT in the ring (CPU900H p.134)
//        CY <- dst<LSB>, dst <- right rotate of dst.
//   RL   rotate left THROUGH carry           (CPU900H p.130)
//        {CY,dst} rotates left as one (size+1)-bit ring: LSB takes the old
//        CY, CY takes the old MSB.
//   RR   rotate right THROUGH carry          (CPU900H p.133)
//        {CY,dst} rotates right as one (size+1)-bit ring.
//   SLA  shift left, LSB filled with 0       (CPU900H p.141)
//   SLL  shift left, LSB filled with 0       (CPU900H p.142)
//   SRA  shift right, MSB held (sign)        (CPU900H p.143)
//   SRL  shift right, MSB filled with 0      (CPU900H p.144)
//
// SLA and SLL are the SAME operation on this CPU. Their Operation lines,
// their description charts and their flag tables are identical, both
// worked examples are "SLA/SLL 4,HL with HL=1234H gives 2340H, CY=1", and
// the summary table gives both the identical function diagram
// "CY <- [MSB<-0] <- 0" (CPU900H p.141, p.142, p.165). Only the opcode
// differs: no result or flag difference.
//
// Z80 conventions do NOT carry over unchanged, in two places:
//   - The TLCS-90 accumulator forms left S, Z and V alone. Toshiba calls
//     this out explicitly: "When the following instructions are used in
//     the TLCS-90, the S, Z and V flags do not change ... In the TLCS-900,
//     these flags change" (CPU900H p.165 notes 2 and 3). Every form here
//     writes S, Z, H, V, N and C.
//   - SLL is not the undocumented Z80 "shift left and set bit 0"; it is a
//     plain logical left shift, identical to SLA.
//
// The flag table is identical for all eight instructions.
// The summary table shows "**0P0*" for every row (CPU900H p.165), i.e.
// S and Z from the result, H forced 0, V = parity, N forced 0, C set.
// The detail pages spell the same thing out:
//   S = MSB of the result, taken at the operand size.
//   Z = 1 when the sized result is zero.
//   H = 0 always (all eight pages say "Reset to 0"), including 32-bit.
//   V = PARITY, not overflow: 1 when the number of 1s in the sized result
//       is EVEN. This is true for the shifts as well as the rotates.
//       "If the operand is 32 bits, an undefined value is set."
//   N = 0 always.
//   C = the LAST bit shifted or rotated out, not the first. Every page
//       says "value of dst BEFORE THE LAST shift/rotate", and the worked
//       examples confirm it: RLC 4,HL with HL=1230H sets CY=1, which is
//       bit 15 of the third intermediate value, not of 1230H.
//       For RL/RR the page says "the value after rotate", which is the
//       same bit: it is whatever landed in the carry cell of the ring.
//
// Two typos in the detail pages, resolved against the Operation line, the
// description chart and the summary table (all three agree):
//   - RRC's flag note says "C = MSB value of dst before the last rotate"
//     (CPU900H p.134). That is copied from the RLC page. RRC's own
//     Operation line says "CY <- dst<LSB>" and its chart draws the carry
//     tap at the right-hand end. C is the last bit out of the LSB.
//     (Equivalently, the MSB of the result -- for a right rotate they are
//     the same bit.)
//   - SLL's description prose says "loads 0 to the MSB of dst"
//     (CPU900H p.142). Its Operation line says "dst<LSB> <- 0" and its
//     chart feeds the 0 into the LSB. It is an LSB zero-fill.
//
// V for 32-bit operands is documented as undefined; this unit computes true
// 32-bit parity, which is cheap in fabric.
//
// F layout, matching the SR low byte (CPU900H p.7):
//   bit 7 S, 6 Z, 5 zero, 4 H, 3 zero, 2 V/P, 1 N, 0 C.
// Bits 5 and 3 always read 0 (the TLCS-90 X flag was deleted), so they are
// driven 0 here rather than passed through.
//
// `size` selects which field rotates: byte = data_in[7:0], word =
// data_in[15:0], long = the whole 32 bits. S, Z and V are taken from that
// field only.
//
// The ISA has no opinion on "the other bits", because there are none: a
// byte operand IS a byte register lane and t900_regfile writes only that
// lane back. This unit passes the bits above the operand through unchanged
// so the sequencer can hand a whole 32-bit register in and hand the whole
// 32-bit result back to the size-aware regfile write port without masking
// anything. Nothing architectural depends on it.
//
// Handshake -- everything advances on ce, one state per ce.
//   start     one-ce pulse; op/size/count/data_in/flags_in must be valid
//             on that ce tick. Asserting start while busy restarts the
//             unit with the new operands.
//   busy      high while iteration steps remain.
//   done      one-ce pulse on the tick that completes the last iteration.
//   result    the working register. Valid from the done tick onward and
//             held until the next start; it moves while busy.
//   flags_out combinational from the working register, same validity.

module t900_shifter
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire        start,       // one-ce pulse, latches the operands
	input  wire [2:0]  op,          // OP_* below = low 3 bits of the opcode
	input  wire [1:0]  size,        // 0=byte 1=word 2=long
	input  wire [4:0]  count,       // normalized 1..16, see header
	input  wire [31:0] data_in,
	// Only flags_in[0] (C) is consumed, by RL and RR. Every flag these
	// instructions touch is written unconditionally, so the rest has no
	// effect; the whole byte stays on the port because the sequencer moves
	// F as a unit.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [7:0]  flags_in,
	/* verilator lint_on UNUSEDSIGNAL */

	output reg         busy,
	output reg         done,        // one-ce pulse after the result is registered
	output wire        finish,      // final busy tick; result/flags already show it
	output wire [31:0] result,
	output wire [7:0]  flags_out
);

	// Opcode low bits (CPU900H p.165): reg forms are E8+op (#4,r) and
	// F8+op (A,r), the memory forms are 78+op. op[0] selects direction.
	localparam [2:0] OP_RLC = 3'd0;   // E8/F8/78  rotate left,  carry outside the ring
	localparam [2:0] OP_RRC = 3'd1;   // E9/F9/79  rotate right, carry outside the ring
	localparam [2:0] OP_RL  = 3'd2;   // EA/FA/7A  rotate left  through carry
	localparam [2:0] OP_RR  = 3'd3;   // EB/FB/7B  rotate right through carry
	localparam [2:0] OP_SLA = 3'd4;   // EC/FC/7C  shift left,  zero fill
	localparam [2:0] OP_SRA = 3'd5;   // ED/FD/7D  shift right, sign fill
	localparam [2:0] OP_SLL = 3'd6;   // EE/FE/7E  shift left,  zero fill (== SLA)
	localparam [2:0] OP_SRL = 3'd7;   // EF/FF/7F  shift right, zero fill

	localparam [1:0] SIZE_B = 2'd0;
	localparam [1:0] SIZE_W = 2'd1;
	localparam [1:0] SIZE_L = 2'd2;

	reg [31:0] work;      // operand, right justified; bits above the size pass through
	reg        cy;        // carry cell
	reg [4:0]  remain;    // bit positions still to move
	reg [2:0]  op_r;
	reg [1:0]  size_r;

	// One bit position. State travels as {value[31:0], carry} so the four
	// slices below chain as plain expressions.
	function automatic [32:0] shift1(input [32:0] state, input [2:0] o, input [1:0] sz);
		reg [31:0] v;
		reg [31:0] nv;
		reg        c;
		reg        msb;
		reg        lsb;
		reg        fill;   // bit entering the vacated end
		reg        nc;     // bit leaving into the carry cell
	begin
		v   = state[32:1];
		c   = state[0];
		msb = (sz == SIZE_B) ? v[7] : (sz == SIZE_W) ? v[15] : v[31];
		lsb = v[0];

		case (o)
			OP_RLC:  fill = msb;    // the bit leaving the top comes back in at the bottom
			OP_RRC:  fill = lsb;    // and vice versa
			OP_RL:   fill = c;      // carry is inside the ring
			OP_RR:   fill = c;
			OP_SRA:  fill = msb;    // sign held
			OP_SLA:  fill = 1'b0;   // zero fill
			OP_SLL:  fill = 1'b0;   // zero fill, identical to SLA
			OP_SRL:  fill = 1'b0;   // zero fill
		endcase

		nc = o[0] ? lsb : msb;

		if (o[0]) begin
			case (sz)
				SIZE_B:  nv = {v[31:8],  fill, v[7:1]};
				SIZE_W:  nv = {v[31:16], fill, v[15:1]};
				SIZE_L:  nv = {fill, v[31:1]};
				default: nv = {fill, v[31:1]};
			endcase
		end else begin
			case (sz)
				SIZE_B:  nv = {v[31:8],  v[6:0],  fill};
				SIZE_W:  nv = {v[31:16], v[14:0], fill};
				SIZE_L:  nv = {v[30:0], fill};
				default: nv = {v[30:0], fill};
			endcase
		end

		shift1 = {nv, nc};
	end
	endfunction

	// Four positions per ce step. A slice is bypassed once the remaining
	// count runs out, so the final step moves 1 to 4 positions and a
	// count of 0 moves none. `remain` is always a multiple of four here,
	// because the start tick takes the remainder away, but the guards stay:
	// they cost nothing and they keep the step correct on its own terms.
	wire [32:0] st0 = {work, cy};
	wire [32:0] st1 = (remain > 5'd0) ? shift1(st0, op_r, size_r) : st0;
	wire [32:0] st2 = (remain > 5'd1) ? shift1(st1, op_r, size_r) : st1;
	wire [32:0] st3 = (remain > 5'd2) ? shift1(st2, op_r, size_r) : st2;
	wire [32:0] st4 = (remain > 5'd3) ? shift1(st3, op_r, size_r) : st3;

	// `finish` is the overlap indication the sequencer consumes; `done` is
	// the registered pulse for standalone clients.
	wire        start_finish = start && (cnt_align == 5'd0);
	wire        finishing    = busy && (remain <= 5'd4);
	wire [31:0] result_now   = start_finish ? ld3[32:1] :
	                           finishing    ? st4[32:1] : work;
	wire        cy_now       = start_finish ? ld3[0] : finishing ? st4[0] : cy;
	wire [1:0]  size_now     = start_finish ? size : size_r;

	// The load tick's own chain: the remainder, n mod 4, applied straight to
	// the incoming operand with the incoming op and size (op_r / size_r do
	// not exist yet on that tick). `count` is 1..16, so count[1:0] IS n mod 4
	// and 16 correctly leaves a remainder of zero.
	wire [1:0]  rem4      = count[1:0];
	wire [4:0]  cnt_align = count - {3'd0, rem4};
	wire [32:0] ld0 = {data_in, flags_in[0]};
	wire [32:0] ld1 = (rem4 > 2'd0) ? shift1(ld0, op, size) : ld0;
	wire [32:0] ld2 = (rem4 > 2'd1) ? shift1(ld1, op, size) : ld1;
	wire [32:0] ld3 = (rem4 > 2'd2) ? shift1(ld2, op, size) : ld2;

	always @(posedge clk) begin
		if (reset) begin
			work   <= 32'd0;
			cy     <= 1'b0;
			remain <= 5'd0;
			op_r   <= OP_RLC;
			size_r <= SIZE_B;
			busy   <= 1'b0;
			done   <= 1'b0;
		end else if (ce) begin
			done <= 1'b0;

			if (start) begin
				work   <= ld3[32:1];
				cy     <= ld3[0];
				op_r   <= op;
				size_r <= size;
				remain <= cnt_align;
				// Counts of 1..3 (and the caller-bug count of 0) are finished
				// by the load tick's own chain and report done immediately.
				busy   <= (cnt_align != 5'd0);
				done   <= (cnt_align == 5'd0);
			end else if (busy) begin
				work <= st4[32:1];
				cy   <= st4[0];
				if (remain <= 5'd4) begin
					remain <= 5'd0;
					busy   <= 1'b0;
					done   <= 1'b1;
				end else begin
					remain <= remain - 5'd4;
				end
			end
		end
	end

	// S, Z and V read the sized field only. V = 1 when the number of ones
	// is even, so it is the XNOR reduction (CPU900H p.165 "P").
	wire s_bit = (size_now == SIZE_B) ? result_now[7] :
	             (size_now == SIZE_W) ? result_now[15] : result_now[31];

	wire z_bit = (size_now == SIZE_B) ? (result_now[7:0]  == 8'd0) :
	             (size_now == SIZE_W) ? (result_now[15:0] == 16'd0) :
	                                                   (result_now == 32'd0);

	wire v_bit = (size_now == SIZE_B) ? ~^result_now[7:0] :
	             (size_now == SIZE_W) ? ~^result_now[15:0] : ~^result_now[31:0];

	assign finish = start_finish | finishing;
	assign result = result_now;

	//                  S      Z     -     H     -     V      N     C
	assign flags_out = {s_bit, z_bit, 1'b0, 1'b0, 1'b0, v_bit, 1'b0, cy_now};

endmodule
