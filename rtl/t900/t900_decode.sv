// Copyright (c) 2026 Jamie Blanks

// TLCS-900/H instruction decoder.
//
// Purely combinational: given the first opcode byte (and the second when the
// first is a class prefix) it produces the effective-address plan and the
// resolved operation descriptor. Sequencing -- fetching displacement and
// immediate bytes, reading registers, running the bus -- belongs to
// t900_seq; this block only says what must happen.
//
// First-byte structure (Toshiba code map, CPU900H p.168):
//
//   0x00-0x7F  main map, decoded directly
//   0x80-0xBF  10 zz mmmm : zz 00/01/10 = src class size B/W/L, zz 11 = dst
//              class; mmmm bit3 selects (XRR) vs (XRR+d8), bits 2:0 = XRR
//   0xC0-0xFF  11 zz .... : zz 00/01/10 = size B/W/L, zz 11 = dst/main
//              bit3 = 0 : bits 2:0 select the memory form
//                         0-5 = (#8) (#16) (#24) ext (-XRR) (XRR+)
//                         6   = undefined
//                         7   = register class via a full register-code byte
//                               (zz 11 -> 0xF7 LDX, a main-map instruction)
//              bit3 = 1 : register class, short 3-bit register code
//                         (zz 11 -> 0xF8-0xFF SWI n, main-map instructions)
//
// The class decode above is hardwired rather than table-driven because it is
// pure bit structure; simulation asserts it agrees with the generated
// table's PFX_* markers for all 256 first bytes, so the two derivations
// cannot silently drift apart.

module t900_decode
(
	input  wire [7:0]  byte0,
	input  wire [7:0]  byte1,

	// First-byte class
	output reg  [1:0]  cls,          // T900_CLS_*
	output reg  [4:0]  ea_kind,      // operand kind of the class EA
	output wire [2:0]  ea_reg,       // XRR / short R selector from byte0
	output reg  [1:0]  cls_size,     // B/W/L carried by the class byte
	output wire        needs_byte1,  // byte1 participates in this decode

	// Resolved operation
	output wire [7:0]  op_id,
	output wire [4:0]  op1_kind,
	output wire [4:0]  op2_kind,
	output wire [1:0]  op1_size,
	output wire [1:0]  op2_size,
	output wire [5:0]  states,       // base cost, class size applied
	output wire [3:0]  group,
	output wire        undef
);

	`include "t900_defs.svh"
	`include "t900_decode_tables.svh"

	localparam [1:0] CLS_MAIN = 2'd0;
	localparam [1:0] CLS_SRC  = 2'd1;
	localparam [1:0] CLS_DST  = 2'd2;
	localparam [1:0] CLS_REG  = 2'd3;

	wire [1:0] zz  = byte0[5:4];
	wire       hi3 = byte0[3];
	wire [2:0] low = byte0[2:0];

	assign ea_reg = low;

	// Class and EA plan from the first byte.
	always @(*) begin
		cls      = CLS_MAIN;
		ea_kind  = T900_K_NONE;
		cls_size = T900_SZ_B;

		if (byte0[7] == 1'b0) begin
			// 0x00-0x7F: main map, no class EA.
			cls = CLS_MAIN;
		end else if (byte0[7:6] == 2'b10) begin
			// (XRR) and (XRR+d8) forms.
			ea_kind  = hi3 ? T900_K_XRR_D8 : T900_K_XRR;
			if (zz == 2'b11) begin
				cls      = CLS_DST;
				cls_size = T900_SZ_B; // dst size comes from the second byte
			end else begin
				cls      = CLS_SRC;
				cls_size = zz;
			end
		end else begin
			// 0xC0-0xFF
			if (hi3) begin
				if (zz == 2'b11) begin
					cls = CLS_MAIN;              // 0xF8-0xFF SWI n
				end else begin
					cls      = CLS_REG;
					ea_kind  = T900_K_SHORT_R;         // short register code in byte0
					cls_size = zz;
				end
			end else begin
				case (low)
					3'd0: ea_kind = T900_K_N8;
					3'd1: ea_kind = T900_K_N16;
					3'd2: ea_kind = T900_K_N24;
					3'd3: ea_kind = T900_K_EXT;
					3'd4: ea_kind = T900_K_PRE_DEC;
					3'd5: ea_kind = T900_K_POST_INC;
					default: ea_kind = T900_K_NONE;
				endcase

				if (low <= 3'd5) begin
					cls      = (zz == 2'b11) ? CLS_DST : CLS_SRC;
					cls_size = (zz == 2'b11) ? T900_SZ_B : zz;
				end else if (low == 3'd7 && zz != 2'b11) begin
					cls      = CLS_REG;          // 0xC7/0xD7/0xE7 register-code byte
					ea_kind  = T900_K_RCODE;
					cls_size = zz;
				end else begin
					// 0xC6/0xD6/0xE6 undefined, 0xF6 undefined, 0xF7 LDX:
					// all resolve in the main map.
					cls = CLS_MAIN;
				end
			end
		end
	end

	assign needs_byte1 = (cls != CLS_MAIN);

	// Table lookup. The main map is indexed by byte0; every class map is
	// indexed by the second byte.
	wire [T900_ENTRY_W-1:0] entry_main = t900_tbl_main(byte0);
	wire [T900_ENTRY_W-1:0] entry_reg  = t900_tbl_reg(byte1);
	wire [T900_ENTRY_W-1:0] entry_src  = t900_tbl_src(byte1);
	wire [T900_ENTRY_W-1:0] entry_dst  = t900_tbl_dst(byte1);

	reg [T900_ENTRY_W-1:0] entry;
	always @(*) begin
		case (cls)
			CLS_SRC: entry = entry_src;
			CLS_DST: entry = entry_dst;
			CLS_REG: entry = entry_reg;
			default: entry = entry_main;
		endcase
	end

	assign op_id    = entry[7:0];
	assign op1_kind = entry[12:8];
	assign op2_kind = entry[17:13];
	assign op1_size = entry[19:18];
	assign op2_size = entry[21:20];
	assign group    = entry[43:40];

	// Class-sized entries carry one state count per size; main-map entries
	// store the same value in all three fields.
	reg [5:0] states_sel;
	always @(*) begin
		case (cls_size)
			T900_SZ_B: states_sel = entry[27:22];
			T900_SZ_W: states_sel = entry[33:28];
			default:   states_sel = entry[39:34];
		endcase
	end
	assign states = states_sel;

	assign undef = (op_id == T900_OP_UNDEF);

endmodule
