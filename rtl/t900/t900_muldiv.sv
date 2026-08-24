// Copyright (c) 2026 Jamie Blanks

// TLCS-900/H multiply / divide unit.
//
// Iterative, one machine state per ce, like the rest of the core. No `*`,
// `/` or `%` appears anywhere below: the multiply is a shift-add over the
// classic {product:multiplier} pair and the divide is a non-restoring
// shift-subtract over the classic {remainder:quotient} pair. Both share the
// same two registers (acc, lo) and the same 17-bit adder.
//
// Interface
// op[1:0]: 00 MUL, 01 MULS, 10 DIV, 11 DIVS.
//          op[1] = divide, op[0] = signed. This matches the ISA ordering of
//          the reg-map second bytes 08/09/0A/0B and 4x/5x (CPU900H p.169).
// size:    0 = byte class, 1 = word class.
//
// Operand convention (the sequencer must match this):
//   a = the destination operand, right justified.
//       MUL/MULS  a is the multiplicand = the LOW half of the destination
//                 register: byte class a[7:0], word class a[15:0]
//                 ("dst <- dst<lower half> x src" (CPU900H p.108, p.111)).
//       DIV/DIVS  a is the whole dividend, i.e. the full destination
//                 register: byte class a[15:0] (16-bit), word class a[31:0]
//                 (CPU900H p.73, p.75).
//   b = the source operand, right justified: byte class b[7:0], word class
//       b[15:0]. b[31:16] is never used.
//   Bits above the class width are ignored, not checked.
//
// result:
//   MUL/MULS  byte class 8x8 -> result[15:0], word class 16x16 -> result[31:0].
//   DIV/DIVS  quotient in the LOW half, remainder in the HIGH half
//             (CPU900H p.73): byte class {rem[7:0], quo[7:0]} in result[15:0],
//             word class {rem[15:0], quo[15:0]} in result[31:0].
//   Unused high bits of a byte-class result read as zero.
//
// v_flag is only meaningful for DIV/DIVS. MUL and MULS change NO flags at all
// (CPU900H p.108, p.111 "S/Z/H/V/N/C = No change"), so the sequencer must not
// write V for them; the unit drives v_flag = 0 there for determinism.
// DIV/DIVS touch V only, never S/Z/H/N/C (CPU900H p.73, p.75).
//
// Handshake: start is a one-ce pulse sampled while busy is low (a start
// asserted while busy is ignored).  busy rises on the ce that latches the
// operands and falls on the ce that pulses done.  done is high for exactly one
// ce interval; result/v_flag are updated by the same edge and hold until the
// next operation completes.  A new start may be issued in the ce interval in
// which done is seen, so back-to-back operations cost no dead state.
//
// `finish` is the final multiply fix-up state, with `finish_result` already
// showing the signed/unsigned product from the registered accumulator, so the
// sequencer may share that state with architectural writeback.  Division has no
// early finish: its remainder correction and overflow logic stay behind the
// result register.
//
// Every operation costs one load state + iterations + one fix-up state. The
// multiply is radix-4 (two multiplier bits per state, 4 / 8 iterations); the
// divide is radix-2 (one quotient bit per state, 8 / 16 iterations):
//
//   MUL, MULS    6 ce ticks byte class, 10 ce ticks word class
//   DIV, DIVS   10 ce ticks byte class, 18 ce ticks word class
//
// counted from the tick that latches the operands to the tick that pulses
// done. A sequencer that waits to observe done before moving on spends one
// more tick (7 / 11 and 11 / 19).
//
// Appendix B totals for the whole instruction in 16-bit 0-wait memory
// (CPU900H p.158-167), and what is left for fetch, decode and operand access
// once this unit has had its ticks:
//
//   op    form       byte class        word class
//   MUL   R,r        11 - 6  = 5       14 - 10 = 4
//   MUL   rr,#       12 - 6  = 6       15 - 10 = 5
//   MUL   R,(mem)    13 - 6  = 7       16 - 10 = 6
//   MULS  R,r         9 - 6  = 3       12 - 10 = 2   <- tightest rows
//   MULS  rr,#       10 - 6  = 4       13 - 10 = 3
//   MULS  R,(mem)    11 - 6  = 5       14 - 10 = 4
//   DIV   R,r        15 - 10 = 5       23 - 18 = 5
//   DIV   rr,#       15 - 10 = 5       23 - 18 = 5
//   DIV   R,(mem)    16 - 10 = 6       24 - 18 = 6
//   DIVS  R,r        18 - 10 = 8       26 - 18 = 8
//   DIVS  rr,#       18 - 10 = 8       26 - 18 = 8
//   DIVS  R,(mem)    19 - 10 = 9       27 - 18 = 9
//
// Every row has margin, so the sequencer can pad each instruction out to its
// table value exactly.  If it burns a tick observing done instead of counting
// the latency it already knows, subtract one more from every row; the tightest
// is then MULS R,r word class with one state to spare (12 - 11).
//
// The multiply is radix-4 because Toshiba charges only +3 states for the 8
// extra multiplier bits of the word class (MUL 11 -> 14, MULS 9 -> 12), so the
// real multiplier is not one bit per state: a radix-2 unit needs 18 ticks for a
// word multiply against a 12..16 state budget and cannot be padded into the
// table at all.  No regular radix matches +3 exactly, but radix-4's 4 states
// land under every total with room for fetch and decode.
//
// The divide stays radix-2 because Toshiba charges exactly +8 states for the 8
// extra quotient bits (DIV 15 -> 23, DIVS 18 -> 26), one quotient bit per
// state, and that shape is also what reproduces the documented divide-by-zero
// bit pattern structurally.
//
// Divide-by-zero and overflow behaviour.  The recurrence below is bit-exact on
// every in-range division and also reproduces the console-measured register
// vectors for out-of-range and divide-by-zero cases, which is what fixes its
// shape:
//
//   unsigned: 9-bit (17-bit word) partial remainder, an INITIAL
//     compare/subtract (a discarded 9th quotient-bit step: if hi >= divisor,
//     hi -= divisor), then N non-restoring steps deciding on the pre-shift
//     sign, quotient bit = ~sign(new P), final +divisor restore if negative.
//     The init step is a no-op in range (hi < divisor) and is what produces
//     the measured clean zeros at quotient exactly 2^N (0100/01 = 0000,
//     4000/40 = 0000) and the measured deep-overflow wreckage
//     (FFFF/01 = FE01, 8000/02 = FA83, 4000/10 = 30DD).  Zero-divide falls
//     out with no special case: quotient = ~hi, remainder = lo
//     (width-uniform on silicon).
//   signed: the divisor enters as a MAGNITUDE, the dividend's high half
//     enters RAW and sign-extended (no dividend negation -- the console
//     zero-divide rows refute a magnitude-based dividend: the remainder is
//     the RAW dividend low half in all four measured cases).  Same init
//     step, magnitude-compared, stepping toward zero; N steps deciding on
//     the post-shift 9-bit sign, quotient bit = ~bit7 (byte; ~bit15 word) --
//     identical to the 9-bit sign for every in-range value, diverging only
//     in wreckage.  End corrections:
//       remainder: one +/-divisor step toward the dividend's sign when the
//         signs disagree (P != 0), plus the exact-negative-division restore
//         (P == -divisor -> 0);
//       quotient: complement if the divisor is negative, then +1 chosen by
//         exact (P ended 0):      divisor<0
//         sign-fix fired:         divisor<0 XOR last-quotient-bit
//         otherwise:              divisor<0 XNOR last-quotient-bit,
//           suppressed when the raw quotient is zero and divisor >= 0.
//     This reproduces 7F45/0 = 4502, 8000/0 = 00FF, FF80/0 = 8000,
//     8123/0 = 23FD, 8000/01 = 03FD and 8000/FF = 0303 exactly.
//
// V rule (CPU900H p.73, p.75) "1 is set when divided by 0 or the quotient
// exceeds the numerals which can be expressed in bits of dst": computed
// EXACTLY at load time from the operand magnitudes (it no longer reads the
// quotient register, whose overflow bits are wreckage):
//   unsigned: V = (dividend high half >= divisor), i.e. quotient >= 2^N;
//             divisor 0 falls out of the same compare.
//   signed:   V = divisor == 0, or floor(|dvd|/|dvs|) >= 2^(N-1) + (signs
//             same ? 0 : 1) -- the strict reading: only -2^(N-1) itself
//             survives with the top bit set.  Implemented as
//             two subtract-compares against |dvs| << (N-1).
//
// Signed division truncates toward zero and the remainder takes the sign of
// the dividend (CPU900H p.75). Verified against the datasheet's own worked
// examples: DIV XIX,IY with XIX=12345678h, IY=89ABh -> XIX=0FDA21DAh and
// DIVS the same operands -> XIX=16EED89Eh (CPU900H p.74, p.76).

module t900_muldiv
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire        start,        // one-ce pulse, ignored while busy
	input  wire [1:0]  op,           // 00 MUL, 01 MULS, 10 DIV, 11 DIVS
	input  wire        size,         // 0 = byte class, 1 = word class
	input  wire [31:0] a,            // destination operand (see header)
	input  wire [31:0] b,            // source operand

	output reg         busy,
	output reg         done,         // one-ce pulse
	output wire        finish,       // final MUL/MULS fix-up state
	output wire [31:0] finish_result,
	output reg  [31:0] result,
	output reg         v_flag
);

	localparam [1:0] ST_IDLE = 2'd0;
	localparam [1:0] ST_RUN  = 2'd1;
	localparam [1:0] ST_FIN  = 2'd2;

	// acc holds the partial product's high half (multiply) or the partial
	// remainder (divide, two's complement, N+1 bits). lo holds the multiplier
	// then the product's low half (multiply), or the dividend's low half
	// which the quotient bits shift into (divide). opnd is the multiplicand
	// or the divisor, always as a magnitude. mul3 is three times the
	// multiplicand, the one radix-4 partial product that is not a plain shift
	// of opnd; it is formed once by an adder at load time so the iteration
	// keeps a single adder in its path.
	reg  [16:0] acc;
	reg  [15:0] lo;
	reg  [15:0] opnd;
	reg  [17:0] mul3;

	reg  [1:0]  state;
	reg  [4:0]  cnt;
	reg         r_div;
	reg         r_signed;
	reg         r_size;
	reg         neg_res;              // negate the product / quotient at the end
	reg         neg_rem;              // negate the remainder at the end
	// ovf_mag: magnitude quotient needs more than N bits. V is computed
	// exactly at load time from the operand magnitudes (`div_v_in`) -- the
	// quotient register cannot carry it, its overflow bits being
	// console-measured wreckage.
	reg         ovf_mag;

	// Load-time operand conditioning (sampled on the start ce)

	wire in_div    = op[1];
	wire in_signed = op[0];

	wire dvd_sign   = size ? a[31] : a[15];
	wire mcand_sign = size ? a[15] : a[7];
	wire src_sign   = size ? b[15] : b[7];

	// Signed operands are divided/multiplied as magnitudes and the sign is
	// re-applied at the end; -2^(w-1) negates to itself, whose bit pattern is
	// its own magnitude, so no width growth is needed.
	wire [31:0] dvd_val = size ? a : {16'd0, a[15:0]};
	wire [31:0] dvd_mag = (in_signed && dvd_sign) ?
	                      (size ? (32'd0 - a) : {16'd0, (16'd0 - a[15:0])}) : dvd_val;

	wire [15:0] mcand_val = size ? a[15:0] : {8'd0, a[7:0]};
	wire [15:0] mcand_mag = (in_signed && mcand_sign) ?
	                        (size ? (16'd0 - a[15:0]) : {8'd0, (8'd0 - a[7:0])}) : mcand_val;

	wire [15:0] src_val = size ? b[15:0] : {8'd0, b[7:0]};
	wire [15:0] src_mag = (in_signed && src_sign) ?
	                      (size ? (16'd0 - b[15:0]) : {8'd0, (8'd0 - b[7:0])}) : src_val;

	wire [15:0] dvd_hi = size ? dvd_mag[31:16] : {8'd0, dvd_mag[15:8]};

	// The dividend's low half enters RAW in both signednesses.
	wire [15:0] dvd_lo = size ? a[15:0] : {8'd0, a[7:0]};

	// Divide load conditioning: initial compare/subtract step + strict V

	// Unsigned: P0 = raw high half, minus the divisor if it fits (the
	// discarded 9th/17th quotient-bit step; no-op in range).
	wire [16:0] u_p0    = size ? {1'b0, a[31:16]} : {9'd0, a[15:8]};
	wire        u_init  = (u_p0 >= {1'b0, src_val});
	wire [16:0] u_p0i   = u_init ? (u_p0 - {1'b0, src_val}) : u_p0;

	// Signed: P0 = sign-extended raw high half; one step toward zero when
	// its magnitude fits the (magnitude) divisor.  9-bit arithmetic for the
	// byte class, 17-bit for the word class.
	wire [8:0]  s_p0b   = {a[15], a[15:8]};
	wire [8:0]  s_p0b_m = a[15] ? (9'd0 - s_p0b) : s_p0b;
	wire        s_initb = (src_mag[7:0] != 8'd0) && (s_p0b_m >= {1'b0, src_mag[7:0]});
	wire [8:0]  s_p0b_i = !s_initb ? s_p0b :
	                      a[15] ? (s_p0b + {1'b0, src_mag[7:0]})
	                            : (s_p0b - {1'b0, src_mag[7:0]});
	wire [16:0] s_p0w   = {a[31], a[31:16]};
	wire [16:0] s_p0w_m = a[31] ? (17'd0 - s_p0w) : s_p0w;
	wire        s_initw = (src_mag != 16'd0) && (s_p0w_m >= {1'b0, src_mag});
	wire [16:0] s_p0w_i = !s_initw ? s_p0w :
	                      a[31] ? (s_p0w + {1'b0, src_mag})
	                            : (s_p0w - {1'b0, src_mag});
	wire [16:0] div_p0  = in_signed ? (size ? s_p0w_i : {8'd0, s_p0b_i})
	                                : u_p0i;

	// Strict V, computed once from the operand magnitudes (the overflow
	// quotient bits are wreckage and cannot carry it).  Signed: quotient
	// magnitude >= 2^(N-1) (+1 more when only -2^(N-1) itself would fit).
	wire        v_expneg = dvd_sign ^ src_sign;
	wire [33:0] v_t      = size
	                     ? ({2'd0, dvd_mag} - {3'd0, src_mag, 15'd0})
	                     : ({18'd0, dvd_mag[15:0]} - {19'd0, src_mag[7:0], 7'd0});
	wire [33:0] v_t2     = v_t - {18'd0, src_mag};
	wire        v_signed = (src_mag == 16'd0) || (v_expneg ? !v_t2[33] : !v_t[33]);
	wire        v_uns    = (dvd_hi >= src_mag);
	wire        div_v_in = in_signed ? v_signed : v_uns;

	// The source operand is at most 16 bits wide in every form of these
	// instructions; the port is 32 bits so the sequencer can hand over a
	// register read unmodified. b[31:16] is never read.
	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused_src_high = &{1'b0, b[31:16]};
	/* verilator lint_on UNUSEDSIGNAL */

	// One iteration step. Byte class works in the low 9 bits of acc and the
	// low 8 of lo; the bits above are held at zero by the muxes below, and
	// an adder's low bits never depend on its high bits, so the same 17-bit
	// adder serves both classes.

	wire        lo_msb  = r_size ? lo[15] : lo[7];
	wire        acc_neg = r_size ? acc[16] : acc[8];

	// Non-restoring divide: shift the {remainder:dividend} pair up by one,
	// then subtract the divisor if the remainder is non-negative, add it back
	// if it went negative. Dropping acc[16] in the shift is harmless:
	// everything is exact modulo 2^(N+1) and the true value fits in N+1 bits
	// (except on overflow, which is where the measured wreckage comes from).
	//
	// The signed form differs in two console-measured ways:
	// the add/sub decision reads the sign AFTER the shift (the divisor is a
	// magnitude, so the step always drives P toward zero), and the quotient
	// bit is the complement of BIT 7 (byte; bit 15 word) rather than of the
	// (N+1)-bit sign -- identical in range, divergent only in wreckage.
	wire [16:0] shifted  = {acc[15:0], lo_msb};
	wire        sh_neg   = r_size ? shifted[16] : shifted[8];
	wire        dec_neg  = r_signed ? sh_neg : acc_neg;
	wire [16:0] div_sum  = dec_neg ? (shifted + {1'b0, opnd}) : (shifted - {1'b0, opnd});
	wire        div_qbit = r_signed ? (r_size ? ~div_sum[15] : ~div_sum[7])
	                                : (r_size ? ~div_sum[16] : ~div_sum[8]);
	wire [16:0] div_acc  = r_size ? div_sum : {8'd0, div_sum[8:0]};
	wire [15:0] div_lo   = r_size ? {lo[14:0], div_qbit} : {8'd0, lo[6:0], div_qbit};

	// Radix-4 shift-add multiply: two multiplier bits per state pick one of
	// {0, 1x, 2x, 3x} the multiplicand, then the {product:multiplier} pair
	// shifts down by two. The partial product never exceeds 3x(2^N - 1) and
	// the sum never exceeds 4x(2^N - 1), so the shifted-down accumulator
	// always fits back into N bits with no carry to remember.
	function automatic [17:0] pp_sel(input [1:0] digit, input [15:0] m, input [17:0] m3);
	begin
		case (digit)
			2'd0:    pp_sel = 18'd0;
			2'd1:    pp_sel = {2'd0, m};
			2'd2:    pp_sel = {1'b0, m, 1'b0};
			default: pp_sel = m3;
		endcase
	end
	endfunction

	wire [17:0] mul_pp  = pp_sel(lo[1:0], opnd, mul3);
	wire [17:0] mul_sum = {2'd0, acc[15:0]} + mul_pp;
	wire [16:0] mul_acc = r_size ? {1'b0, mul_sum[17:2]} : {9'd0, mul_sum[9:2]};
	wire [15:0] mul_lo  = r_size ? {mul_sum[1:0], lo[15:2]} : {8'd0, mul_sum[1:0], lo[7:2]};

	// Result assembly (fix-up state)

	// Unsigned: non-restoring leaves a negative remainder one divisor short.
	wire [15:0] rem_restore = acc_neg ? (acc[15:0] + opnd) : acc[15:0];
	wire [15:0] u_rem = r_size ? rem_restore : {8'd0, rem_restore[7:0]};
	wire [15:0] u_quo = r_size ? lo : {8'd0, lo[7:0]};

	// Signed end corrections (console-fitted; see the header).  neg_rem holds
	// the dividend's sign, neg_res^neg_rem the divisor's.
	wire        s_dneg  = neg_res ^ neg_rem;
	wire        s_pz    = r_size ? (acc == 17'd0) : (acc[8:0] == 9'd0);
	wire [16:0] acc_pd  = acc + {1'b0, opnd};
	wire [16:0] acc_md  = acc - {1'b0, opnd};
	wire        s_sfix  = !s_pz && (acc_neg != neg_rem);
	wire        s_efix  = neg_rem && !s_pz &&
	                      (r_size ? (acc_pd == 17'd0) : (acc_pd[8:0] == 9'd0));
	wire [16:0] s_rem17 = s_sfix ? (acc_neg ? acc_pd : acc_md)
	                             : (s_efix ? acc_pd : acc);
	wire        s_lastq = lo[0];
	wire        s_qz    = r_size ? (lo == 16'd0) : (lo[7:0] == 8'd0);
	wire        s_inc   = (s_pz || s_efix) ? s_dneg :
	                      s_sfix           ? (s_dneg ^ s_lastq) :
	                                         ((s_dneg == s_lastq) && !(s_qz && !s_dneg));
	wire [15:0] s_qc    = s_dneg ? ~lo : lo;
	wire [15:0] s_quo   = s_qc + {15'd0, s_inc};

	wire [15:0] rem_fix = r_signed ? s_rem17[15:0] : u_rem;
	wire [15:0] quo_fix = r_signed ? s_quo : u_quo;
	wire [31:0] div_res = r_size ? {rem_fix, quo_fix} : {16'd0, rem_fix[7:0], quo_fix[7:0]};

	wire [31:0] prod_mag = r_size ? {acc[15:0], lo} : {16'd0, acc[7:0], lo[7:0]};
	wire [31:0] prod_fix = neg_res ? (32'd0 - prod_mag) : prod_mag;
	wire [31:0] mul_res  = r_size ? prod_fix : {16'd0, prod_fix[15:0]};

	// Word MULS rr,# has only thirteen Toshiba states including fetch and
	// decode. Waiting one more state for the registered `done` pulse makes it
	// fourteen. The product below is already complete in ST_FIN; expose only
	// the multiply fix-up so writeback can occupy this same state without
	// putting the divider's correction chain on the register-file path.
	assign finish        = busy && !r_div && (state == ST_FIN);
	assign finish_result = finish ? mul_res : result;

	always @(posedge clk) begin
		if (reset) begin
			state    <= ST_IDLE;
			cnt      <= 5'd0;
			acc      <= 17'd0;
			lo       <= 16'd0;
			opnd     <= 16'd0;
			mul3     <= 18'd0;
			r_div    <= 1'b0;
			r_signed <= 1'b0;
			r_size   <= 1'b0;
			neg_res  <= 1'b0;
			neg_rem  <= 1'b0;
			ovf_mag  <= 1'b0;
			busy     <= 1'b0;
			done     <= 1'b0;
			result   <= 32'd0;
			v_flag   <= 1'b0;
		end else if (ce) begin
			done <= 1'b0;

			case (state)
				ST_IDLE: begin
					if (start) begin
						r_div    <= in_div;
						r_signed <= in_signed;
						r_size   <= size;
						// radix-2 divide: N iterations.
						// radix-4 multiply: N/2 iterations.
						cnt      <= in_div ? (size ? 5'd15 : 5'd7)
						                   : (size ? 5'd7  : 5'd3);
						busy     <= 1'b1;
						state    <= ST_RUN;

						if (in_div) begin
							acc     <= div_p0;
							lo      <= dvd_lo;
							opnd    <= src_mag;
							neg_res <= in_signed && (dvd_sign ^ src_sign);
							neg_rem <= in_signed && dvd_sign;
							ovf_mag <= div_v_in;
						end else begin
							acc     <= 17'd0;
							lo      <= src_mag;          // multiplier
							opnd    <= mcand_mag;        // multiplicand
							mul3    <= {1'b0, mcand_mag, 1'b0} + {2'd0, mcand_mag};
							neg_res <= in_signed && (mcand_sign ^ src_sign);
							neg_rem <= 1'b0;
							ovf_mag <= 1'b0;
						end
					end
				end

				ST_RUN: begin
					acc <= r_div ? div_acc : mul_acc;
					lo  <= r_div ? div_lo  : mul_lo;
					cnt <= cnt - 5'd1;
					if (cnt == 5'd0) state <= ST_FIN;
				end

				ST_FIN: begin
					result <= r_div ? div_res : mul_res;
					v_flag <= r_div ? ovf_mag : 1'b0;
					busy   <= 1'b0;
					done   <= 1'b1;
					state  <= ST_IDLE;
				end

				default: begin
					state <= ST_IDLE;
					busy  <= 1'b0;
				end
			endcase
		end
	end

endmodule
