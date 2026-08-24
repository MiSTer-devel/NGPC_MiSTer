// Generated from rtl/t900/tables/*.tsv -- do not edit by hand.
// Table digest: 4293b1d689e73e79d2817dbd658ce5cd47b36ad37b75fefb0419676a81316853
//
// Deliberately unguarded: these are module-scoped localparams and
// every module that decodes includes its own copy. An include guard
// would leave the second and later modules without the definitions.
//
// This is a shared symbol table, so any single consumer uses only a
// fraction of it. UNUSEDPARAM is waived for the declarations rather
// than at the lint command line, which would blind whole modules to
// the class.

/* verilator lint_off UNUSEDPARAM */

// Operand sizes
localparam [1:0] T900_SZ_B = 2'd0;
localparam [1:0] T900_SZ_W = 2'd1;
localparam [1:0] T900_SZ_L = 2'd2;
// SZ_CLASS: size comes from the first-byte class, not the table entry.
localparam [1:0] T900_SZ_CLASS = 2'd3;

// Operation ids (92 entries)
localparam [7:0] T900_OP_UNDEF = 8'd0;
localparam [7:0] T900_OP_ADC = 8'd1;
localparam [7:0] T900_OP_ADD = 8'd2;
localparam [7:0] T900_OP_AND = 8'd3;
localparam [7:0] T900_OP_ANDCF = 8'd4;
localparam [7:0] T900_OP_BIT = 8'd5;
localparam [7:0] T900_OP_BS1B = 8'd6;
localparam [7:0] T900_OP_BS1F = 8'd7;
localparam [7:0] T900_OP_CALL = 8'd8;
localparam [7:0] T900_OP_CALR = 8'd9;
localparam [7:0] T900_OP_CCF = 8'd10;
localparam [7:0] T900_OP_CHG = 8'd11;
localparam [7:0] T900_OP_CP = 8'd12;
localparam [7:0] T900_OP_CPD = 8'd13;
localparam [7:0] T900_OP_CPDR = 8'd14;
localparam [7:0] T900_OP_CPI = 8'd15;
localparam [7:0] T900_OP_CPIR = 8'd16;
localparam [7:0] T900_OP_CPL = 8'd17;
localparam [7:0] T900_OP_DAA = 8'd18;
localparam [7:0] T900_OP_DEC = 8'd19;
localparam [7:0] T900_OP_DECF = 8'd20;
localparam [7:0] T900_OP_DIV = 8'd21;
localparam [7:0] T900_OP_DIVS = 8'd22;
localparam [7:0] T900_OP_DJNZ = 8'd23;
localparam [7:0] T900_OP_EI = 8'd24;
localparam [7:0] T900_OP_EX = 8'd25;
localparam [7:0] T900_OP_EXTS = 8'd26;
localparam [7:0] T900_OP_EXTZ = 8'd27;
localparam [7:0] T900_OP_HALT = 8'd28;
localparam [7:0] T900_OP_INC = 8'd29;
localparam [7:0] T900_OP_INCF = 8'd30;
localparam [7:0] T900_OP_JP = 8'd31;
localparam [7:0] T900_OP_JR = 8'd32;
localparam [7:0] T900_OP_JRL = 8'd33;
localparam [7:0] T900_OP_LD = 8'd34;
localparam [7:0] T900_OP_LDA = 8'd35;
localparam [7:0] T900_OP_LDC = 8'd36;
localparam [7:0] T900_OP_LDCF = 8'd37;
localparam [7:0] T900_OP_LDD = 8'd38;
localparam [7:0] T900_OP_LDDR = 8'd39;
localparam [7:0] T900_OP_LDF = 8'd40;
localparam [7:0] T900_OP_LDI = 8'd41;
localparam [7:0] T900_OP_LDIR = 8'd42;
localparam [7:0] T900_OP_LDX = 8'd43;
localparam [7:0] T900_OP_LINK = 8'd44;
localparam [7:0] T900_OP_MDEC1 = 8'd45;
localparam [7:0] T900_OP_MDEC2 = 8'd46;
localparam [7:0] T900_OP_MDEC4 = 8'd47;
localparam [7:0] T900_OP_MINC1 = 8'd48;
localparam [7:0] T900_OP_MINC2 = 8'd49;
localparam [7:0] T900_OP_MINC4 = 8'd50;
localparam [7:0] T900_OP_MIRR = 8'd51;
localparam [7:0] T900_OP_MUL = 8'd52;
localparam [7:0] T900_OP_MULA = 8'd53;
localparam [7:0] T900_OP_MULS = 8'd54;
localparam [7:0] T900_OP_NEG = 8'd55;
localparam [7:0] T900_OP_NOP = 8'd56;
localparam [7:0] T900_OP_OR = 8'd57;
localparam [7:0] T900_OP_ORCF = 8'd58;
localparam [7:0] T900_OP_PAA = 8'd59;
localparam [7:0] T900_OP_PFX_DST = 8'd60;
localparam [7:0] T900_OP_PFX_REG = 8'd61;
localparam [7:0] T900_OP_PFX_SRC = 8'd62;
localparam [7:0] T900_OP_POP = 8'd63;
localparam [7:0] T900_OP_PUSH = 8'd64;
localparam [7:0] T900_OP_RCF = 8'd65;
localparam [7:0] T900_OP_RES = 8'd66;
localparam [7:0] T900_OP_RET = 8'd67;
localparam [7:0] T900_OP_RETD = 8'd68;
localparam [7:0] T900_OP_RETI = 8'd69;
localparam [7:0] T900_OP_RL = 8'd70;
localparam [7:0] T900_OP_RLC = 8'd71;
localparam [7:0] T900_OP_RLD = 8'd72;
localparam [7:0] T900_OP_RR = 8'd73;
localparam [7:0] T900_OP_RRC = 8'd74;
localparam [7:0] T900_OP_RRD = 8'd75;
localparam [7:0] T900_OP_SBC = 8'd76;
localparam [7:0] T900_OP_SCC = 8'd77;
localparam [7:0] T900_OP_SCF = 8'd78;
localparam [7:0] T900_OP_SET = 8'd79;
localparam [7:0] T900_OP_SLA = 8'd80;
localparam [7:0] T900_OP_SLL = 8'd81;
localparam [7:0] T900_OP_SRA = 8'd82;
localparam [7:0] T900_OP_SRL = 8'd83;
localparam [7:0] T900_OP_STCF = 8'd84;
localparam [7:0] T900_OP_SUB = 8'd85;
localparam [7:0] T900_OP_SWI = 8'd86;
localparam [7:0] T900_OP_TSET = 8'd87;
localparam [7:0] T900_OP_UNLK = 8'd88;
localparam [7:0] T900_OP_XOR = 8'd89;
localparam [7:0] T900_OP_XORCF = 8'd90;
localparam [7:0] T900_OP_ZCF = 8'd91;

// Operand kinds (29 entries)
localparam [4:0] T900_K_NONE = 5'd0; // TSV '-'
localparam [4:0] T900_K_RCODE_R = 5'd1; // TSV 'r'
localparam [4:0] T900_K_SHORT_R = 5'd2; // TSV 'R'
localparam [4:0] T900_K_RCODE = 5'd3; // TSV 'rcode'
localparam [4:0] T900_K_XRR = 5'd4; // TSV 'xrr'
localparam [4:0] T900_K_XRR_D8 = 5'd5; // TSV 'xrr+d8'
localparam [4:0] T900_K_PRE_DEC = 5'd6; // TSV '-xrr'
localparam [4:0] T900_K_POST_INC = 5'd7; // TSV 'xrr+'
localparam [4:0] T900_K_N8 = 5'd8; // TSV 'n8'
localparam [4:0] T900_K_N16 = 5'd9; // TSV 'n16'
localparam [4:0] T900_K_N24 = 5'd10; // TSV 'n24'
localparam [4:0] T900_K_EXT = 5'd11; // TSV 'ext'
localparam [4:0] T900_K_MEM = 5'd12; // TSV 'mem'
localparam [4:0] T900_K_MEM8 = 5'd13; // TSV 'mem8'
localparam [4:0] T900_K_MEM16 = 5'd14; // TSV 'mem16'
localparam [4:0] T900_K_IMM = 5'd15; // TSV 'imm'
localparam [4:0] T900_K_IMM3 = 5'd16; // TSV 'imm3'
localparam [4:0] T900_K_IMM8 = 5'd17; // TSV 'imm8'
localparam [4:0] T900_K_IMM16 = 5'd18; // TSV 'imm16'
localparam [4:0] T900_K_IMM24 = 5'd19; // TSV 'imm24'
localparam [4:0] T900_K_IMM32 = 5'd20; // TSV 'imm32'
localparam [4:0] T900_K_D8 = 5'd21; // TSV 'd8'
localparam [4:0] T900_K_D16 = 5'd22; // TSV 'd16'
localparam [4:0] T900_K_CC = 5'd23; // TSV 'cc'
localparam [4:0] T900_K_CR = 5'd24; // TSV 'cr'
localparam [4:0] T900_K_REG_A = 5'd25; // TSV 'A'
localparam [4:0] T900_K_FLAGS = 5'd26; // TSV 'F'
localparam [4:0] T900_K_FLAGS_ALT = 5'd27; // TSV "F'"
localparam [4:0] T900_K_SR = 5'd28; // TSV 'SR'

// Instruction groups (9 entries)
localparam [3:0] T900_G_UNDEFINED = 4'd0;
localparam [3:0] T900_G_ALU = 4'd1;
localparam [3:0] T900_G_BIT = 4'd2;
localparam [3:0] T900_G_CONTROL_TRANSFER = 4'd3;
localparam [3:0] T900_G_LOAD = 4'd4;
localparam [3:0] T900_G_PREFIX = 4'd5;
localparam [3:0] T900_G_SHIFT = 4'd6;
localparam [3:0] T900_G_STRING = 4'd7;
localparam [3:0] T900_G_SYSTEM = 4'd8;

// Decode entry layout, LSB first:
//   [7:0]   op id
//   [12:8]  operand 1 kind
//   [17:13] operand 2 kind
//   [19:18] operand 1 size
//   [21:20] operand 2 size
//   [27:22] states, byte class
//   [33:28] states, word class
//   [39:34] states, long class
//   [43:40] group
localparam int T900_ENTRY_W = 44;

/* verilator lint_on UNUSEDPARAM */
