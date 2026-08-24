// Copyright (c) 2026 Jamie Blanks

// TLCS-900/H arithmetic and logic unit.
//
// Purely combinational: one operation is selected by `op`, one 32-bit result
// and one 8-bit flag word come out. There is no state and no clock, so the
// sequencer can evaluate an operand pair in the same state it reads the
// register file.
//
// Data convention matches t900_regfile: operands are right-justified in 32
// bits for every size, and `size` is 0=byte, 1=word, 2=long. Bits above the
// selected size are ignored on the inputs and driven to zero on the result,
// so a byte result is always {24'd0, value}.
//
// F register layout (CPU900H p.148 status-register diagram):
//   bit 7 S, bit 6 Z, bit 5 "0", bit 4 H, bit 3 "0", bit 2 V/P, bit 1 N,
//   bit 0 C.
// Bits 5 and 3 have no flip-flops in the real SR and always read 0. This
// module passes them through untouched; forcing them to zero belongs to the
// SR/flag register itself, not to the ALU.
//
// Flag semantics from the Toshiba instruction pages.
//
// ADD / ADC (CPU900H p.50, p.48): S = MSB of the result.  Z = result is 0.
//   H = carry from bit 3 to bit 4; "if the operand is 32-bit, an undefined
//   value is set".  V = overflow.  N = 0.  C = carry out of the MSB.
// SUB (p.147), SBC (p.137), CP (p.62): as ADD, except H is a borrow from bit 3
//   to bit 4 (again undefined at 32 bits), N = 1, and C is a borrow from the
//   MSB.
//
// H does NOT scale with the operand size: the datasheet says "bit 3 to bit 4"
// for both the byte and the word form, so it is always the carry or borrow
// across the bit-3/bit-4 boundary of the low byte.  Only the 32-bit case is
// called out as undefined.
//
// INC (p.83) / DEC (p.70): H = carry (INC) or borrow (DEC) from bit 3 to bit 4,
//   with no 32-bit caveat.  V = overflow.  N = 0 (INC) / 1 (DEC).  C unchanged.
//   Both pages add "With the INC/DEC #3,r instruction, if the operand is a word
//   or a long word, no flags change"; that suppression is a sequencer decision,
//   since it simply does not commit flags_out for those encodings, and this ALU
//   always produces the arithmetic flags with C preserved.  "#3 in operands
//   indicates from 1 to 8", so the sequencer maps the encoded 0 to 8 and
//   presents 1..8 on b.
// NEG (p.113): dst <- 0 - dst.  S, Z, H (borrow bit3->bit4), V, N = 1, C
//   (borrow from the MSB).  The operand arrives on `a`; the ALU supplies the
//   zero augend internally.
// AND (p.52): H = 1.  V = parity of the result, even -> 1 and odd -> 0,
//   undefined at 32 bits.  N = 0.  C = 0.  OR (p.116) and XOR (p.153) are
//   identical except H = 0.  Parity polarity: P (the V bit) is 1 for EVEN
//   parity, and parity is defined for the byte and word forms only.
// CPL (p.67): S, Z, V, C unchanged; H = 1; N = 1.
// EXTS (p.80) / EXTZ (p.81): no flags change.  EXTS copies the sign bit of the
//   lower half over the whole upper half (bit 7 for a word, bit 15 for a long
//   word); EXTZ clears the upper half.
// MIRR (p.107): dst<MSB:LSB> <- dst<LSB:MSB>, word only, no flags.
// LD (p.88): no flags change -- this is the PASS operation.
//
// DAA (CPU900H p.68 table, CPU900H p.69 flags). The fix-value table is
// transcribed verbatim below:
//
//   Opera-  N before  C before  Upper 4    H before  Lower 4   Added  C after
//   tion    DAA       DAA       bits dst   DAA       bits dst  value  DAA
//   ------  --------  --------  --------   --------  --------  -----  -------
//   ADD        0         0       0 to 9       0       0 to 9    00      0
//              0         0       0 to 8       0       A to F    06      0
//   ADC        0         0       0 to 9       1       0 to 3    06      0
//              0         0       A to F       0       0 to 9    60      1
//              0         0       9 to F       0       A to F    66      1
//              0         0       A to F       1       0 to 3    66      1
//              0         1       0 to 2       0       0 to 9    60      1
//              0         1       0 to 2       0       A to F    66      1
//              0         1       0 to 3       1       0 to 3    66      1
//   ------  --------  --------  --------   --------  --------  -----  -------
//   SUB        1         0       0 to 9       0       0 to 9    00      0
//   SBC        1         0       0 to 8       1       6 to F    FA      0
//   NEG        1         1       7 to F       0       0 to 9    A0      1
//              1         1       6 to F       1       6 to F    9A      1
//
//   Note: Decimal adjustment cannot be performed for the INC or DEC
//   instruction. This is because the C flag does not change.
//
// DAA flags (CPU900H p.69), verbatim:
//   S = MSB value of the result is set.
//   Z = 1 is set if the result is 0, otherwise 0.
//   H = 1 is set if a carry from bit 3 to bit 4 occurs as a result of the
//       operation, otherwise 0.
//   V = 1 is set if the parity (number of 1s) of the result is even,
//       otherwise 0.
//   N = No change.
//   C = 1 is set if a carry occurs from the MSB as a result of the operation
//       or a carry was 1 before operation, otherwise 0.
//
// The N=1 "added values" FA/A0/9A are the two's complements of 06/60/66, so
// the hardware here selects one magnitude from {00,06,60,66} out of the
// C/H/digit comparators and lets N pick the adder direction.
//
// The adjust direction changes what "carry" means.  The p.69 prose says "carry"
// for both H and C with no borrow caveat, but when N=1 the adjust runs as a
// subtraction, which is the case SUB (CPU900H p.147) spells out as "borrow".
// So both flags are read off the one byte adder and follow it:
//
//   N = 0:  C_after = C_before | carry  out of dst + fix
//   N = 1:  C_after = C_before | borrow out of dst - fix, i.e. (dst < fix)
//
// with H the same story one nibble down.  This module builds a single 9-bit sum
// and takes bit 8, which is already carry or borrow according to the direction
// it just ran in.  Do not shorten the C rule to "C_before | fix[6]": that form
// is an accident of the N=0 rows and is wrong for N=1 in both directions.
//
// Behavior in cases the datasheet leaves undefined
//  - 32-bit H for ADD/ADC/SUB/SBC/CP is documented as undefined; reported
//    as 0 here.
//  - 32-bit V (parity) for AND/OR/XOR is documented as undefined; reported
//    as 0 here.
//  - DAA input combinations the table does not list (they cannot follow a
//    valid BCD add or subtract) fall back to the boundary form of the same
//    comparators; the thirteen listed rows are unaffected.
//  - Sizes the ISA does not define for an operation (EXTZ/EXTS/MIRR/CPL at
//    other sizes, DAA at word/long) are still given a defined, latch-free
//    answer: the operation generalises to the selected size, and DAA always
//    works on the low byte. Unreachable in practice.

module t900_alu
(
	input  wire [4:0]  op,
	input  wire [1:0]  size,        // 0=byte 1=word 2=long
	input  wire [31:0] a,           // primary / destination operand
	input  wire [31:0] b,           // source operand (INC/DEC: the 1..8 count)
	input  wire [7:0]  flags_in,    // S Z 0 H 0 V N C

	output reg  [31:0] result,
	output reg  [7:0]  flags_out
);

	localparam [1:0] SIZE_B = 2'd0;
	localparam [1:0] SIZE_W = 2'd1;
	localparam [1:0] SIZE_L = 2'd2;

	// Operation codes. op[4:3] is the group, op[2:0] the member:
	//   00 = adder group   (result from the shared add/subtract path)
	//   01 = logic group   (bitwise, parity in V)
	//   10 = field group   (extend / mirror / decimal adjust / move)
	localparam [4:0] ALU_ADD  = 5'b00_000;  // 0x00 a + b
	localparam [4:0] ALU_ADC  = 5'b00_001;  // 0x01 a + b + C
	localparam [4:0] ALU_SUB  = 5'b00_010;  // 0x02 a - b
	localparam [4:0] ALU_SBC  = 5'b00_011;  // 0x03 a - b - C
	localparam [4:0] ALU_CP   = 5'b00_100;  // 0x04 flags of a - b, result = a
	localparam [4:0] ALU_INC  = 5'b00_101;  // 0x05 a + b (b=1..8), C preserved
	localparam [4:0] ALU_DEC  = 5'b00_110;  // 0x06 a - b (b=1..8), C preserved
	localparam [4:0] ALU_NEG  = 5'b00_111;  // 0x07 0 - a
	localparam [4:0] ALU_AND  = 5'b01_000;  // 0x08 a & b
	localparam [4:0] ALU_OR   = 5'b01_001;  // 0x09 a | b
	localparam [4:0] ALU_XOR  = 5'b01_010;  // 0x0A a ^ b
	localparam [4:0] ALU_CPL  = 5'b01_011;  // 0x0B ~a, sets H and N only
	localparam [4:0] ALU_EXTZ = 5'b10_000;  // 0x10 zero-extend the lower half
	localparam [4:0] ALU_EXTS = 5'b10_001;  // 0x11 sign-extend the lower half
	localparam [4:0] ALU_MIRR = 5'b10_010;  // 0x12 bit-reverse the operand
	localparam [4:0] ALU_DAA  = 5'b10_011;  // 0x13 decimal adjust (byte only)
	localparam [4:0] ALU_PASS = 5'b10_100;  // 0x14 result = a, flags untouched

	// Operand conditioning
	reg [31:0] size_mask;

	always @(*) begin
		case (size)
			SIZE_B:  size_mask = 32'h0000_00ff;
			SIZE_W:  size_mask = 32'h0000_ffff;
			default: size_mask = 32'hffff_ffff;
		endcase
	end

	wire [31:0] a_op = a & size_mask;
	wire [31:0] b_op = b & size_mask;

	wire h_in = flags_in[4];
	wire n_in = flags_in[1];
	wire c_in = flags_in[0];

	// Shared add/subtract path
	// NEG is "0 - dst" (CPU900H p.113), and dst arrives on a, so the augend
	// is forced to zero and the operand moves to the subtrahend input.
	wire        is_neg  = (op == ALU_NEG);
	wire [31:0] add_a   = is_neg ? 32'd0 : a_op;
	wire [31:0] add_b   = is_neg ? a_op  : b_op;

	wire op_sub = (op == ALU_SUB) || (op == ALU_SBC) || (op == ALU_CP) ||
	              (op == ALU_DEC) || (op == ALU_NEG);

	// Carry into bit 0. A subtract is a + ~b + 1; a borrowing subtract is
	// a + ~b + ~C.
	reg add_cin;

	always @(*) begin
		case (op)
			ALU_ADC: add_cin = c_in;
			ALU_SBC: add_cin = ~c_in;
			default: add_cin = op_sub;
		endcase
	end

	// The addend is inverted inside the operand size so the carry out lands
	// on the size boundary and not on bit 32.
	wire [31:0] addend  = op_sub ? ((~add_b) & size_mask) : add_b;
	wire [32:0] add_sum = {1'b0, add_a} + {1'b0, addend} + {32'd0, add_cin};
	wire [31:0] add_res = add_sum[31:0] & size_mask;

	// Carry out of the MSB of the selected size.
	reg add_cout;

	always @(*) begin
		case (size)
			SIZE_B:  add_cout = add_sum[8];
			SIZE_W:  add_cout = add_sum[16];
			default: add_cout = add_sum[32];
		endcase
	end

	// H is the carry into bit 4, taken against the un-inverted operand so the
	// subtract case yields the borrow the datasheet describes
	// (CPU900H p.50 "carry from bit 3 to bit 4", CPU900H p.147 "borrow from
	// bit 3 to bit 4"). Same bit position for every size.
	wire add_half = add_a[4] ^ add_b[4] ^ add_sum[4];

	// Signed overflow: the two adder inputs agree in sign and the sum does
	// not. Using the inverted addend makes this the subtract rule for free.
	// Only the three candidate sign positions are built.
	wire ovf_b = (~(add_a[7]  ^ addend[7]))  & (add_a[7]  ^ add_sum[7]);
	wire ovf_w = (~(add_a[15] ^ addend[15])) & (add_a[15] ^ add_sum[15]);
	wire ovf_l = (~(add_a[31] ^ addend[31])) & (add_a[31] ^ add_sum[31]);

	reg add_ovf;

	always @(*) begin
		case (size)
			SIZE_B:  add_ovf = ovf_b;
			SIZE_W:  add_ovf = ovf_w;
			default: add_ovf = ovf_l;
		endcase
	end

	// Logic path
	reg [31:0] logic_res;

	always @(*) begin
		case (op)
			ALU_AND: logic_res = a_op & b_op;
			ALU_OR:  logic_res = a_op | b_op;
			default: logic_res = a_op ^ b_op;
		endcase
	end

	// Shared S / Z / P generation for the adder and logic groups
	reg [31:0] flag_val;

	always @(*) begin
		case (op)
			ALU_AND, ALU_OR, ALU_XOR: flag_val = logic_res;
			default:                  flag_val = add_res;
		endcase
	end

	wire sign_flag = (size == SIZE_B) ? flag_val[7] :
	                 ((size == SIZE_W) ? flag_val[15] : flag_val[31]);
	wire zero_flag = (flag_val == 32'd0);

	// P = 1 for even parity (CPU900H p.52, "1 is set if a parity of the
	// result is even, 0 if odd"). Undefined at 32 bits; reported as 0 here.
	wire par_flag = (size == SIZE_B) ? ~(^flag_val[7:0]) :
	                ((size == SIZE_W) ? ~(^flag_val[15:0]) : 1'b0);

	// H is undefined for the 32-bit forms of ADD/ADC/SUB/SBC/CP; reported as
	// 0 here.
	wire h_undef32 = (size == SIZE_L) &&
	                 ((op == ALU_ADD) || (op == ALU_ADC) || (op == ALU_SUB) ||
	                  (op == ALU_SBC) || (op == ALU_CP));
	wire h_flag = add_half & ~h_undef32;

	// EXTZ / EXTS
	// "Lower half" is 4 bits of a byte, 8 bits of a word, 16 bits of a long
	// word; only the word and long forms exist in the ISA (CPU900H p.80).
	wire ext_half_sign = (size == SIZE_B) ? a_op[3] :
	                     ((size == SIZE_W) ? a_op[7] : a_op[15]);
	wire ext_fill = (op == ALU_EXTS) ? ext_half_sign : 1'b0;

	reg [31:0] ext_res;

	always @(*) begin
		case (size)
			SIZE_B:  ext_res = {24'd0, {4{ext_fill}}, a_op[3:0]};
			SIZE_W:  ext_res = {16'd0, {8{ext_fill}}, a_op[7:0]};
			default: ext_res = {{16{ext_fill}}, a_op[15:0]};
		endcase
	end

	// MIRR
	// One 32-bit reversal serves every size: reversing the low 16 bits is the
	// top half of the reversed long word, and the low 8 bits the top quarter.
	wire [31:0] rev32;

	genvar gi;
	generate
		for (gi = 0; gi < 32; gi = gi + 1) begin : g_mirror
			assign rev32[gi] = a_op[31 - gi];
		end
	endgenerate

	reg [31:0] mirr_res;

	always @(*) begin
		case (size)
			SIZE_B:  mirr_res = {24'd0, rev32[31:24]};
			SIZE_W:  mirr_res = {16'd0, rev32[31:16]};
			default: mirr_res = rev32;
		endcase
	end

	// DAA
	// DAA is byte only (CPU900H p.68), so it always works on the low byte
	// and ignores `size`.
	wire [7:0] daa_in = a[7:0];
	wire [3:0] daa_hi = daa_in[7:4];
	wire [3:0] daa_lo = daa_in[3:0];

	// Fix magnitude from the C, H and digit comparators. The four branches
	// below are the CPU900H-68 rows; N only picks the adder direction.
	reg [7:0] daa_fix;

	always @(*) begin
		if (c_in) begin
			// C=1 rows: 60 for a clean low digit, 66 otherwise.
			daa_fix = (h_in || (daa_lo > 4'h9)) ? 8'h66 : 8'h60;
		end else if (h_in) begin
			// C=0, H=1 rows: upper 0-9 takes 06, upper A-F takes 66.
			daa_fix = (daa_in < 8'h9a) ? 8'h06 : 8'h66;
		end else if (daa_lo > 4'h9) begin
			// C=0, H=0, low digit out of range.
			daa_fix = (daa_hi > 4'h8) ? 8'h66 : 8'h06;
		end else begin
			// C=0, H=0, low digit in range.
			daa_fix = (daa_hi > 4'h9) ? 8'h60 : 8'h00;
		end
	end

	// One 9-bit adder, run forwards for N=0 and backwards for N=1. Bit 8 is
	// its carry out of dst + fix in the first case and its borrow out of
	// dst - fix in the second, which is exactly the direction-dependent
	// "carry" the CPU900H-68 C-after column asks for.
	wire [8:0] daa_sum = n_in ? ({1'b0, daa_in} - {1'b0, daa_fix})
	                          : ({1'b0, daa_in} + {1'b0, daa_fix});
	wire [7:0] daa_res = daa_sum[7:0];

	// C is a decimal carry that was already pending, or one this adjust just
	// produced. Never "c_in | daa_fix[6]" -- see the header note.
	wire daa_carry = c_in | daa_sum[8];
	// H is the same rule one nibble down: carry into bit 4 when N=0, borrow
	// into bit 4 when N=1. The XOR form is both at once.
	wire daa_half  = daa_in[4] ^ daa_fix[4] ^ daa_res[4];
	wire daa_par   = ~(^daa_res);
	wire daa_zero  = (daa_res == 8'd0);

	// Result and flag selection
	always @(*) begin
		// Defaults: move the primary operand through and leave every flag
		// alone (CPU900H p.88, LD changes no flags). Assigning both outputs
		// before the case keeps this block latch free.
		result    = a_op;
		flags_out = flags_in;

		case (op)
			ALU_ADD, ALU_ADC: begin
				result       = add_res;
				flags_out[7] = sign_flag;
				flags_out[6] = zero_flag;
				flags_out[4] = h_flag;
				flags_out[2] = add_ovf;
				flags_out[1] = 1'b0;            // (CPU900H p.50) N cleared
				flags_out[0] = add_cout;
			end

			ALU_SUB, ALU_SBC: begin
				result       = add_res;
				flags_out[7] = sign_flag;
				flags_out[6] = zero_flag;
				flags_out[4] = h_flag;
				flags_out[2] = add_ovf;
				flags_out[1] = 1'b1;            // (CPU900H p.147) N set
				flags_out[0] = ~add_cout;       // borrow
			end

			ALU_CP: begin
				result       = a_op;            // (CPU900H p.62) flags only
				flags_out[7] = sign_flag;
				flags_out[6] = zero_flag;
				flags_out[4] = h_flag;
				flags_out[2] = add_ovf;
				flags_out[1] = 1'b1;
				flags_out[0] = ~add_cout;
			end

			ALU_NEG: begin
				result       = add_res;
				flags_out[7] = sign_flag;
				flags_out[6] = zero_flag;
				flags_out[4] = h_flag;
				flags_out[2] = add_ovf;
				flags_out[1] = 1'b1;            // (CPU900H p.113) N set
				flags_out[0] = ~add_cout;
			end

			ALU_INC: begin
				result       = add_res;
				flags_out[7] = sign_flag;
				flags_out[6] = zero_flag;
				flags_out[4] = h_flag;
				flags_out[2] = add_ovf;
				flags_out[1] = 1'b0;            // (CPU900H p.83) N cleared
				// flags_out[0] keeps flags_in[0]: "C = No change".
			end

			ALU_DEC: begin
				result       = add_res;
				flags_out[7] = sign_flag;
				flags_out[6] = zero_flag;
				flags_out[4] = h_flag;
				flags_out[2] = add_ovf;
				flags_out[1] = 1'b1;            // (CPU900H p.70) N set
				// flags_out[0] keeps flags_in[0]: "C = No change".
			end

			ALU_AND: begin
				result       = logic_res;
				flags_out[7] = sign_flag;
				flags_out[6] = zero_flag;
				flags_out[4] = 1'b1;            // (CPU900H p.52) H = 1
				flags_out[2] = par_flag;
				flags_out[1] = 1'b0;
				flags_out[0] = 1'b0;
			end

			ALU_OR, ALU_XOR: begin
				result       = logic_res;
				flags_out[7] = sign_flag;
				flags_out[6] = zero_flag;
				flags_out[4] = 1'b0;            // (CPU900H p.116, p.153) H = 0
				flags_out[2] = par_flag;
				flags_out[1] = 1'b0;
				flags_out[0] = 1'b0;
			end

			ALU_CPL: begin
				result       = (~a_op) & size_mask;
				flags_out[4] = 1'b1;            // (CPU900H p.67) H = 1
				flags_out[1] = 1'b1;            // (CPU900H p.67) N = 1
			end

			ALU_EXTZ, ALU_EXTS: begin
				result = ext_res;               // (CPU900H p.80, p.81) no flags
			end

			ALU_MIRR: begin
				result = mirr_res;              // (CPU900H p.107) no flags
			end

			ALU_DAA: begin
				result       = {24'd0, daa_res};
				flags_out[7] = daa_res[7];
				flags_out[6] = daa_zero;
				flags_out[4] = daa_half;
				flags_out[2] = daa_par;
				// flags_out[1] keeps flags_in[1]: "N = No change".
				flags_out[0] = daa_carry;
			end

			ALU_PASS: begin
				result = a_op;                  // (CPU900H p.88) no flags
			end

			default: begin
				// Unassigned op codes behave like PASS so the block stays
				// fully specified; the sequencer never issues them.
				result = a_op;
			end
		endcase
	end

endmodule
