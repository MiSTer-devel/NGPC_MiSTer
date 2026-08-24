// Copyright (c) 2026 Jamie Blanks

// TMP95C061-class I/O ports: output latches, direction registers, function
// registers, and the two pattern generators.
//
// SFRs owned: 0x01-0x1F, 0x2C-0x2F, 0x4C-0x4E (TMP95C061 datasheet
// pp.175-177, p.181).
//
// This module is also the default arm of the shell's SFR read mux, which is
// why the read decode below ends in "everything else reads 0xFF". Two things
// land on that default: the write-only registers this module owns (every P*CR
// and P*FC, whose shadows appear only on the savestate tap - the BIOS keeps
// its own RAM copies of registers it cannot read, so returning the shadow
// would hide a real software dependency), and the unmapped gaps in the whole
// 0x00-0x7F space. Both 0xFF answers are behavioural choices. Address 0x00 is
// the one exception and must read 0x00: Ogre Battle's script interpreter
// performs a null-data read there.
//
// Per port there is an output latch, a direction register P*nCR (0 = input,
// 1 = output) and a function register P*nFC. A pin drives its alternate
// function when the function bit is set, and drives at all only when the
// direction bit says output (datasheet pp.30-50). Reading a port register
// returns the pin state for input pins and the latch for output pins - the
// distinction matters for PB, where the BIOS writes 0xB2 and then relies on
// PB0/PB1/PB4/PB5 staying inputs. P5, P6, PA and P9 do not bond all eight
// bits; the registers here are full bytes and every bit behaves the same way.
//
// NGP bonding:
//   PA2 = TO1 and PA3 = TO3 -> the Z80 interrupt path. The BIOS arms it with
//     PAFC = 0x0C then PACR = 0x0C.
//   PB7 = INT0, fed internally by the RTC alarm rather than by a pin.
//   PB0/PB1/PB4/PB5 = INT4/INT5/INT6/INT7; pb_is_input publishes their
//     direction bits so ngp_intc can gate edge detection.
//   P8 = the serial pins; the BIOS enables TxD0 with P8FC = 0x01.
//   P9 is the ADC input port and is read-only.
//   P6 carries the chip selects; the BIOS enables them with P6FC = 0x3F.
//   P1, P2, P5, P7 have no known NGP consumer beyond bus control.
//
// PACR and PAFC reset to 0x00 (datasheet p.177). The BIOS writes 0x0C before
// anything uses TO1 or TO3, so do not seed these registers to make the Z80
// interrupt work.

module ngp_ports
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire [6:0]  sfr_addr,
	input  wire [7:0]  sfr_wdata,
	input  wire        sfr_wr,
	input  wire        sfr_rd,
	output wire [7:0]  sfr_rdata,     // also the shell's mux default for gaps

	// Pin groups, split DIN/DOUT/OE per AGENTS.md. P2 and P6 are output-only
	// ports and have no direction register, so they have no OE either.
	input  wire [7:0]  p1_in,  output wire [7:0] p1_out,  output wire [7:0] p1_oe,
	output wire [7:0]  p2_out,
	input  wire [7:0]  p5_in,  output wire [7:0] p5_out,  output wire [7:0] p5_oe,
	output wire [7:0]  p6_out,
	input  wire [7:0]  p7_in,  output wire [7:0] p7_out,  output wire [7:0] p7_oe,
	input  wire [7:0]  p8_in,  output wire [7:0] p8_out,  output wire [7:0] p8_oe,
	input  wire [3:0]  p9_in,                                   // AN0-AN3 as digital
	input  wire [3:0]  pa_in,  output wire [3:0] pa_out,  output wire [3:0] pa_oe,
	input  wire [7:0]  pb_in,  output wire [7:0] pb_out,  output wire [7:0] pb_oe,

	// Alternate functions routed in from the other submodules.
	input  wire        to1, to3,                 // ngp_t8
	input  wire        to4, to5, to6,            // ngp_t16
	input  wire        pg0t, pg1t,               // ngp_t16 T45CR<PG0T, PG1T>
	input  wire        txd0, sclk0, txd1, sclk1, // ngp_sio
	input  wire [3:0]  cs_n,                     // ngp_csc
	output wire        ti0_pin, wait_pin,        // PA1, PA0 read back to consumers
	output wire [3:0]  pb_is_input,              // to ngp_intc for INT4/5/6/7 gating

	input  wire [7:0]  ss_reg_addr,              // words 0x28-0x2F
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	// Registers. Reset values are chip reset values (datasheet pp.175-177):
	// latches P1 = 0x00 (input mode), P2 = 0xFF, P5 = 0x3D, P6 = 0x3B,
	// P7 = 0xFF, P8 = 0x3F, PA = 0x0F, PB = 0xFF, and every CR and FC register
	// 0x00, including PACR and PAFC.
	reg [7:0] p1_latch, p2_latch, p5_latch, p6_latch, p7_latch, p8_latch, pb_latch;
	reg [3:0] pa_latch;

	reg [7:0] p1cr, p5cr, p7cr, p8cr, pbcr;
	reg [3:0] pacr;
	reg [7:0] p2fc, p5fc, p6fc, p7fc, p8fc, pbfc;
	reg [3:0] pafc;

	// Pattern generator control, PG01CR at 0x4E. The bit figure in the
	// TMP95C061 PDF is an image; the field order below is read out of the same
	// peripheral's register table in the 1994 TLCS-900 databook (TMP96C141,
	// address 0x4E).
	reg [7:0] pg01cr;

	localparam integer PG_PAT1 = 7;   // PG1 write mode: 0 = 8-bit, 1 = 4-bit (pattern mode)
	localparam integer PG_CCW1 = 6;   // PG1 rotation direction
	localparam integer PG_PG1M = 5;   // PG1 excitation mode
	localparam integer PG_PG1E = 4;   // PG1 trigger input enable
	localparam integer PG_PAT0 = 3;
	localparam integer PG_CCW0 = 2;
	localparam integer PG_PG0M = 1;
	localparam integer PG_PG0E = 0;

	wire [3:0] pg0_pat, pg0_sa;
	wire [3:0] pg1_pat, pg1_sa;

	// Reading a port: latch for output bits, pin for input bits.
	function automatic [7:0] port_read(input [7:0] latch_val, input [7:0] cr, input [7:0] pin);
		port_read = (latch_val & cr) | (pin & ~cr);
	endfunction

	function automatic [3:0] port_read4(input [3:0] latch_val, input [3:0] cr, input [3:0] pin);
		port_read4 = (latch_val & cr) | (pin & ~cr);
	endfunction

	// Pin outputs. Alternate function when the FC bit is set, latch otherwise;
	// the OE is the direction bit, so a function output only reaches the
	// outside world when the port is also set to output.
	//
	// Three function groups are not routed into this module and therefore
	// present the latch even with their FC bit set: P1's D8-D15 (AM8/16 is tied
	// to 8-bit on the NGP so the function does not exist), P2's A23-A16 and
	// P5's R/W/BUSAK/BUSRQ/HWR (the SoC fabric owns the external bus directly),
	// and P6's RAS/REFOUT (the DRAM controller is a stub, see rtl/soc/ngp_csc.sv).
	assign p1_out = p1_latch;
	assign p1_oe  = p1cr;

	assign p2_out = p2_latch;

	assign p5_out = p5_latch;
	assign p5_oe  = p5cr;

	assign p6_out = { p6_latch[7:6],
	                  p6_latch[5:4],                            // REFOUT, RAS: stub
	                  p6fc[3] ? cs_n[3] : p6_latch[3],
	                  p6fc[2] ? cs_n[2] : p6_latch[2],
	                  p6fc[1] ? cs_n[1] : p6_latch[1],
	                  p6fc[0] ? cs_n[0] : p6_latch[0] };

	// P7FC low nibble selects PG0 output, high nibble PG1 output, bit by bit
	// (datasheet p.111: "since port and functions can be switched on a bit
	// basis using P7FC, any port pin can be assigned to pattern generator
	// output").
	assign p7_out = { p7fc[7] ? pg1_pat[3] : p7_latch[7],
	                  p7fc[6] ? pg1_pat[2] : p7_latch[6],
	                  p7fc[5] ? pg1_pat[1] : p7_latch[5],
	                  p7fc[4] ? pg1_pat[0] : p7_latch[4],
	                  p7fc[3] ? pg0_pat[3] : p7_latch[3],
	                  p7fc[2] ? pg0_pat[2] : p7_latch[2],
	                  p7fc[1] ? pg0_pat[1] : p7_latch[1],
	                  p7fc[0] ? pg0_pat[0] : p7_latch[0] };
	assign p7_oe  = p7cr;

	assign p8_out = { p8_latch[7:6],
	                  p8fc[5] ? sclk1 : p8_latch[5],
	                  p8_latch[4],                              // RxD1, input function
	                  p8fc[3] ? txd1  : p8_latch[3],
	                  p8fc[2] ? sclk0 : p8_latch[2],
	                  p8_latch[1],                              // RxD0, input function
	                  p8fc[0] ? txd0  : p8_latch[0] };
	assign p8_oe  = p8cr;

	assign pa_out = { pafc[3] ? to3 : pa_latch[3],
	                  pafc[2] ? to1 : pa_latch[2],
	                  pa_latch[1:0] };                          // TI0, WAIT: input functions
	assign pa_oe  = pacr;

	assign pb_out = { pb_latch[7],
	                  pbfc[6] ? to6 : pb_latch[6],
	                  pb_latch[5:4],
	                  pbfc[3] ? to5 : pb_latch[3],
	                  pbfc[2] ? to4 : pb_latch[2],
	                  pb_latch[1:0] };
	assign pb_oe  = pbcr;

	// PA0 and PA1 read back to their consumers as pin state. When the port
	// drives the pin the pin carries what the port drives, so the input
	// path sees the latch in that case.
	assign wait_pin = pacr[0] ? pa_latch[0] : pa_in[0];
	assign ti0_pin  = pacr[1] ? pa_latch[1] : pa_in[1];

	// INT4/INT5/INT6/INT7 sit on PB0/PB1/PB4/PB5.
	assign pb_is_input = { ~pbcr[5], ~pbcr[4], ~pbcr[1], ~pbcr[0] };

	// SFR reads. Combinational, zero wait states. Address 0x00 is an explicit
	// behavioural exception. Write-only registers and every other unmapped
	// address fall through to 0xFF.
	reg [7:0] rd;

	always_comb begin
		case (sfr_addr)
			7'h00:   rd = 8'h00;                                // Ogre null-data read
			7'h01:   rd = port_read(p1_latch, p1cr, p1_in);
			7'h06:   rd = p2_latch;                             // output-only port
			7'h0d:   rd = port_read(p5_latch, p5cr, p5_in);
			7'h12:   rd = p6_latch;                             // output-only port
			7'h13:   rd = port_read(p7_latch, p7cr, p7_in);
			7'h18:   rd = port_read(p8_latch, p8cr, p8_in);
			7'h19:   rd = {4'hf, p9_in};                        // read-only ADC port
			7'h1e:   rd = {4'hf, port_read4(pa_latch, pacr, pa_in)};
			7'h1f:   rd = port_read(pb_latch, pbcr, pb_in);
			// PG*REG bits 7:4 are the pattern output latch and are
			// write-only; bits 3:0 are the shift alternate register and read
			// back. The 1994 databook's table for the same peripheral has the
			// two halves the other way round and the TMP95C061 figure is an
			// image that cannot arbitrate. Nothing on the NGP can tell the
			// difference: P7 is unbonded and the BIOS writes P7FC = 0x00.
			7'h4c:   rd = {4'hf, pg0_sa};
			7'h4d:   rd = {4'hf, pg1_sa};
			7'h4e:   rd = pg01cr;
			default: rd = 8'hff;
		endcase
	end

	assign sfr_rdata = rd;

	// Savestate tap. The write-only shadows are visible here and nowhere else.
	assign pause_ready = pause_req;

	wire ss_hit = ss_wren && pause_ready;
	wire ss_28  = ss_hit && (ss_reg_addr == 8'h28);
	wire ss_29  = ss_hit && (ss_reg_addr == 8'h29);
	wire ss_2a  = ss_hit && (ss_reg_addr == 8'h2a);
	wire ss_2b  = ss_hit && (ss_reg_addr == 8'h2b);
	wire ss_2c  = ss_hit && (ss_reg_addr == 8'h2c);
	wire ss_2d  = ss_hit && (ss_reg_addr == 8'h2d);

	reg [31:0] ss_rd;

	always_comb begin
		case (ss_reg_addr)
			8'h28:   ss_rd = {p1_latch, p2_latch, p5_latch, p6_latch};
			8'h29:   ss_rd = {p7_latch, p8_latch, {4'h0, pa_latch}, pb_latch};
			8'h2a:   ss_rd = {p1cr, p2fc, p5cr, p5fc};
			8'h2b:   ss_rd = {p6fc, p7cr, p7fc, p8cr};
			8'h2c:   ss_rd = {p8fc, {4'h0, pacr}, {4'h0, pafc}, pbcr};
			8'h2d:   ss_rd = {pbfc, pg0_pat, pg0_sa, pg1_pat, pg1_sa, pg01cr};
			default: ss_rd = 32'h00000000;                      // 0x2E-0x2F reserved
		endcase
	end

	assign ss_rdata = ss_rd;

	// Register file. The write strobe is the enable and is deliberately not
	// re-gated by ce. P9 is read-only and P2/P6 have no direction register, so
	// neither has a CR write target.
	always @(posedge clk) begin
		if (reset) begin
			p1_latch <= 8'h00;
			p2_latch <= 8'hff;
			p5_latch <= 8'h3d;
			p6_latch <= 8'h3b;
			p7_latch <= 8'hff;
			p8_latch <= 8'h3f;
			pa_latch <= 4'hf;
			pb_latch <= 8'hff;

			p1cr     <= 8'h00;
			p5cr     <= 8'h00;
			p7cr     <= 8'h00;
			p8cr     <= 8'h00;
			pacr     <= 4'h0;                   // datasheet reset value; see header
			pbcr     <= 8'h00;

			p2fc     <= 8'h00;
			p5fc     <= 8'h00;
			p6fc     <= 8'h00;
			p7fc     <= 8'h00;
			p8fc     <= 8'h00;
			pafc     <= 4'h0;                   // datasheet reset value; see header
			pbfc     <= 8'h00;

			pg01cr   <= 8'h00;
		end else begin
			if (sfr_wr) begin
				case (sfr_addr)
					7'h01: p1_latch <= sfr_wdata;
					7'h04: p1cr     <= sfr_wdata;
					7'h06: p2_latch <= sfr_wdata;
					7'h09: p2fc     <= sfr_wdata;
					7'h0d: p5_latch <= sfr_wdata;
					7'h10: p5cr     <= sfr_wdata;
					7'h11: p5fc     <= sfr_wdata;
					7'h12: p6_latch <= sfr_wdata;
					7'h13: p7_latch <= sfr_wdata;
					7'h15: p6fc     <= sfr_wdata;
					7'h16: p7cr     <= sfr_wdata;
					7'h17: p7fc     <= sfr_wdata;
					7'h18: p8_latch <= sfr_wdata;
					7'h1a: p8cr     <= sfr_wdata;
					7'h1b: p8fc     <= sfr_wdata;
					7'h1e: pa_latch <= sfr_wdata[3:0];
					7'h1f: pb_latch <= sfr_wdata;
					7'h2c: pacr     <= sfr_wdata[3:0];
					7'h2d: pafc     <= sfr_wdata[3:0];
					7'h2e: pbcr     <= sfr_wdata;
					7'h2f: pbfc     <= sfr_wdata;
					7'h4e: pg01cr   <= sfr_wdata;
					default: ;                  // 0x19 is read-only; gaps ignore writes
				endcase
			end

			if (ss_28) begin
				p1_latch <= ss_wdata[31:24];
				p2_latch <= ss_wdata[23:16];
				p5_latch <= ss_wdata[15:8];
				p6_latch <= ss_wdata[7:0];
			end
			if (ss_29) begin
				p7_latch <= ss_wdata[31:24];
				p8_latch <= ss_wdata[23:16];
				pa_latch <= ss_wdata[11:8];
				pb_latch <= ss_wdata[7:0];
			end
			if (ss_2a) begin
				p1cr <= ss_wdata[31:24];
				p2fc <= ss_wdata[23:16];
				p5cr <= ss_wdata[15:8];
				p5fc <= ss_wdata[7:0];
			end
			if (ss_2b) begin
				p6fc <= ss_wdata[31:24];
				p7cr <= ss_wdata[23:16];
				p7fc <= ss_wdata[15:8];
				p8cr <= ss_wdata[7:0];
			end
			if (ss_2c) begin
				p8fc <= ss_wdata[31:24];
				pacr <= ss_wdata[19:16];
				pafc <= ss_wdata[11:8];
				pbcr <= ss_wdata[7:0];
			end
			if (ss_2d) begin
				pbfc   <= ss_wdata[31:24];
				pg01cr <= ss_wdata[7:0];
			end
		end
	end

	// Pattern generators. They live here because they exist only to drive
	// port 7. PG0's shift trigger is the flip-flop of 8-bit timer 0/1 (TO1) or
	// of 16-bit timer 4 (TO4), and PG1's is 8-bit timer 2/3 (TO3) or 16-bit
	// timer 5 (TO5), with the choice made by T45CR<PG0T>/<PG1T> (datasheet
	// pp.111, 115). T45CR lives in ngp_t16, which publishes the two bits on
	// pg0t / pg1t. Nothing on the NGP is affected: P7 is unbonded, the BIOS
	// writes P7FC = 0x00 and never writes PG01CR, so both generators stay
	// disabled.
	ngp_pg pg0
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),
		.wr(sfr_wr && (sfr_addr == 7'h4c)),
		.wdata(sfr_wdata),
		.pat(pg01cr[PG_PAT0]),
		.ccw(pg01cr[PG_CCW0]),
		.excite_mode(pg01cr[PG_PG0M]),
		.tenable(pg01cr[PG_PG0E]),
		.trigger(pg0t ? to4 : to1),
		.ss_wren(ss_2d),
		.ss_wdata(ss_wdata[23:16]),
		.pat_out(pg0_pat),
		.sa_out(pg0_sa)
	);

	ngp_pg pg1
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),
		.wr(sfr_wr && (sfr_addr == 7'h4d)),
		.wdata(sfr_wdata),
		.pat(pg01cr[PG_PAT1]),
		.ccw(pg01cr[PG_CCW1]),
		.excite_mode(pg01cr[PG_PG1M]),
		.tenable(pg01cr[PG_PG1E]),
		.trigger(pg1t ? to5 : to3),
		.ss_wren(ss_2d),
		.ss_wdata(ss_wdata[15:8]),
		.pat_out(pg1_pat),
		.sa_out(pg1_sa)
	);

	// sfr_rd is in the normative port list; no port register has a read side
	// effect, so it drives nothing here.
	wire unused_ok = &{1'b0, sfr_rd};

endmodule

// One pattern generator channel.
//
// PG*REG layout as implemented: bits 7:4 are the 4-bit pattern output latch
// that reaches P7, bits 3:0 the shift alternate register. In pattern
// generation mode (PG01CR <PATn> = 1) a CPU write reaches only the shift
// alternate register, and the trigger transfers it into the output latch, so
// a pattern is emitted in step with a timer (datasheet p.116). With <PATn> = 0
// the write is a plain 8-bit write of both halves and the trigger rotates
// the output latch, which is 4-phase 1-step/2-step stepping motor drive:
// <CCWn> = 0 rotates PG00 -> PG01 -> PG02 -> PG03, 1 the other way
// (datasheet p.119).
//
// Not modelled: 4-phase 1-2 excitation (<PGnM> = 1), whose eight-state
// sequence is only given in figures that are images in the TMP95C061 PDF. The
// NGP has no consumer for any of this: P7 is not bonded to anything known.
module ngp_pg
(
	input  wire       clk,
	input  wire       ce,
	input  wire       reset,
	input  wire       wr,            // CPU write strobe to this channel's PG*REG
	input  wire [7:0] wdata,
	input  wire       pat,           // PG01CR <PATn>
	input  wire       ccw,           // PG01CR <CCWn>
	input  wire       excite_mode,   // PG01CR <PGnM>
	input  wire       tenable,       // PG01CR <PGnTE>
	input  wire       trigger,       // timer flip-flop output
	input  wire       ss_wren,
	input  wire [7:0] ss_wdata,
	output reg  [3:0] pat_out,       // to P7
	output reg  [3:0] sa_out         // shift alternate register, readable
);

	// The trigger is a timer flip-flop, so it changes on a machine state:
	// sample it on ce and shift on the rising edge (datasheet p.119).
	reg  trigger_q;
	wire trigger_rise = ce && trigger && !trigger_q;

	always @(posedge clk) begin
		if (reset) begin
			trigger_q <= 1'b0;
			// The datasheet marks the pattern bits undefined after reset
			// (p.181); zero is chosen so the model is deterministic.
			pat_out   <= 4'h0;
			sa_out    <= 4'h0;
		end else begin
			if (ce) begin
				trigger_q <= trigger;
			end

			if (wr) begin
				sa_out <= wdata[3:0];
				if (!pat) begin
					pat_out <= wdata[7:4];      // 8-bit write mode
				end
			end

			if (trigger_rise && tenable) begin
				if (pat) begin
					pat_out <= sa_out;
				end else begin
					pat_out <= ccw ? {pat_out[0], pat_out[3:1]}
					               : {pat_out[2:0], pat_out[3]};
				end
			end

			if (ss_wren) begin
				pat_out <= ss_wdata[7:4];
				sa_out  <= ss_wdata[3:0];
			end
		end
	end

	// <PGnM> selects 1-step/2-step versus 1-2 excitation; the latter is not
	// modelled (see the module header), so the bit is stored in PG01CR and
	// read back but changes nothing.
	wire unused_ok = &{1'b0, excite_mode};

endmodule
