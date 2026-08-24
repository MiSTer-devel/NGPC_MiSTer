// Copyright (c) 2026 Jamie Blanks

// TMP95C061-class 8-bit timer block: the prescaler and timers T0-T3.
//
// Owns SFRs 0x20 and 0x22-0x29. Bus convention: sfr_rdata is a pure
// combinational function of sfr_addr and state; sfr_wr / sfr_rd are
// one-clk_sys strobes and are not additionally gated by ce (the strobe is
// itself the enable); everything that models the passage of time is ce-gated.
//
// Write-only registers read 0xFF on the CPU bus and keep shadows that appear
// only on the savestate tap (TMP95C061 datasheet p.178; TLCS-900 8-bit timer
// manual p.9). TFFCR's two control fields always read 11, so TFFCR reads 0xCC
// out of reset (TMP95C061 datasheet p.178).
//
// The block is built the way the silicon is: one prescaler chain feeding two
// identical timer pairs (ngp_t8_pair below). Pair 0 is timers 0/1 with TFF1
// on TO1; pair 1 is timers 2/3 with TFF3 on TO3.
//
// NGP wiring: TI0 is the K2GE horizontal blanking signal used as timer 0's
// external clock, not an interrupt, and TO3 is timer flip-flop 3 on PA3,
// which the board wires to the Z80's interrupt input (TLCS-900 8-bit timer
// manual p.11 footnote 3).

module ngp_t8
#(
	// M16_INTT_LOW: in 16-bit timer mode the low-byte comparator does not
	// raise INTT0 (TMP95C061 datasheet p.84, Figure 3.8(11)). Set to 1 to
	// enable it for A/B testing.
	parameter M16_INTT_LOW = 0
)
(
	input  wire        clk,
	input  wire        ce,          // ce_cpu, geared; serial baud follows it
	input  wire        ce_timer,    // fixed 6.144 MHz machine enable
	input  wire        reset,

	input  wire [6:0]  sfr_addr,    // owns 0x20, 0x22-0x29
	input  wire [7:0]  sfr_wdata,
	input  wire        sfr_wr,
	input  wire        sfr_rd,
	output wire [7:0]  sfr_rdata,

	input  wire        ti0,         // K2GE HBlank, level; rising edge counts
	output wire        to1,         // TFF1 -> PA2
	output wire        to3,         // TFF3 -> PA3 -> Z80 INT
	output wire [3:0]  intt,        // INTT0-3, one-ce pulses
	output wire        to2_trg,     // timer 2 match, serial baud alternative clock

	// prescaler taps published to ngp_sio (one-ce-wide enables)
	output wire        phi_t0,
	output wire        phi_t2,
	output wire        phi_t8,
	output wire        phi_t32,

	// The timer taps of the same chain, published to ngp_t16: T4CLK/T5CLK
	// select phi_T1, phi_T4 or phi_T16 (TMP95C061 datasheet p.94), and the
	// chain that produces them lives here.
	output wire        phi_t1,
	output wire        phi_t4,
	output wire        phi_t16,
	output wire        prrun,

	// TRUN<T4RUN> and <T5RUN> sit in TRUN at 0x20, which this module owns,
	// but they run the 16-bit timers (TMP95C061 datasheet p.79). Published
	// for the same reason as the taps above.
	output wire        t4run,
	output wire        t5run,

	input  wire [7:0]  ss_reg_addr, // words 0x08-0x0F
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	// SFR addresses (TMP95C061 datasheet p.178)
	localparam [6:0] A_TRUN   = 7'h20;
	localparam [6:0] A_TREG0  = 7'h22;
	localparam [6:0] A_TREG1  = 7'h23;
	localparam [6:0] A_T01MOD = 7'h24;
	localparam [6:0] A_TFFCR  = 7'h25;
	localparam [6:0] A_TREG2  = 7'h26;
	localparam [6:0] A_TREG3  = 7'h27;
	localparam [6:0] A_T23MOD = 7'h28;
	localparam [6:0] A_TRDC   = 7'h29;

	// Register file
	reg [7:0] trun;                      // 0x20, bit 6 is not implemented
	reg [7:0] t01mod;                    // 0x24, write-only shadow
	reg [7:0] t23mod;                    // 0x28, write-only shadow
	reg       ff3ie, ff3is;              // 0x25 bits 5, 4
	reg       ff1ie, ff1is;              // 0x25 bits 1, 0
	reg [1:0] trdc;                      // 0x29 bits 1:0
	reg [7:0] treg0, treg1, treg2, treg3;
	reg [7:0] treg0_buf, treg2_buf;      // register buffers (datasheet p.75)
	reg       ti0_q;                     // TI0 edge detector
	// 16-bit mode interlock: writing TREG0 (TREG2) disables the pair's
	// comparator until TREG1 (TREG3) is written (datasheet p.84). Reset leaves
	// the comparator armed; the interlock only matters in 16-bit mode.
	reg [1:0] cmp_armed;

	// CPU read path - combinational, no ce anywhere.
	// TRUN bit 6 is not implemented and reads 0 (datasheet p.79 Fig 3.8(7)).
	wire [7:0] trun_rb  = {trun[7], 1'b0, trun[5:0]};
	// TFFCR's two control fields always read 11 (datasheet p.78).
	wire [7:0] tffcr_rb = {2'b11, ff3ie, ff3is, 2'b11, ff1ie, ff1is};
	wire [7:0] trdc_rb  = {6'b000000, trdc};

	reg [7:0] rdata;
	always_comb begin
		case (sfr_addr)
			A_TRUN:  rdata = trun_rb;
			A_TFFCR: rdata = tffcr_rb;
			A_TRDC:  rdata = trdc_rb;

			// T01MOD and T23MOD are read-write: the detail figures
			// (TMP95C061 datasheet pp.76-77) mark every field of both "R/W".
			// The sound driver depends on it - it starts with
			// `and (0x28),0x33` then `or (0x28),0x04`, a read-modify-write
			// that forces T23M = 00 and T3CLK = 01 while preserving T2CLK.
			// If these read 0xFF instead, TFF3 never edges and the Z80 sound
			// driver never gets its interrupt.
			A_T01MOD: rdata = t01mod;
			A_T23MOD: rdata = t23mod;

			// TREG0-3 really are write-only: the CPU reads 0xFF and only the
			// savestate tap sees the shadows (TMP95C061 datasheet p.75).
			default: rdata = 8'hFF;
		endcase
	end
	assign sfr_rdata = rdata;

	// Prescaler.
	//
	// The stock part is a 9-bit chain taking fc/4 with the timer taps at chain
	// bits 1/3/5/8 (TMP95C061 datasheet p.73 Fig 3.8(2)). The NGP's numbers do
	// not fit that: the TLCS-900 8-bit timer manual p.7 tabulates
	// phi_T1/T4/T16/T256 as 20.83 us / 83.33 us / 333.3 us / 5.333 ms, while
	// the BIOS link setup BR0CR = 0x05 (phi_T0, divisor 5) has to yield
	// 19200 bps, which needs phi_T0 = 1.536 MHz = 6.144 MHz / 4.
	//
	// A 13-bit chain reproduces every documented number, with the timer taps
	// four stages below the datasheet's:
	//
	//   input tick = ce_cpu / 4 = 1.536 MHz at gear 0 = phi_T0
	//   serial taps: phi_T2 = bit 1, phi_T8 = bit 3, phi_T32 = bit 5
	//   timer  taps: phi_T1 = bit 4, phi_T4 = bit 6,
	//                phi_T16 = bit 8, phi_T256 = bit 12
	//
	// In ce_cpu ticks the timer taps are 128 / 512 / 2048 / 32768, which is
	// the 8-bit timer manual's four periods at 6.144 MHz exactly.
	//
	// Two copies of the divider state are kept because the NGP has two clock
	// contracts: baud rate follows the CPU gear, while the timer block input
	// is fixed at 384 kHz. Both share PRRUN stop/clear semantics, but the
	// serial copy advances on geared `ce` and the timer copy on fixed
	// `ce_timer`. Observable at slow CPU gears: BIOS music keeps tempo while
	// link baud slows with the gear.
	localparam integer TAP_T2   = 1;
	localparam integer TAP_T8   = 3;
	localparam integer TAP_T32  = 5;
	localparam integer TAP_T1   = 4;
	localparam integer TAP_T4   = 6;
	localparam integer TAP_T16  = 8;
	localparam integer TAP_T256 = 12;

	reg [1:0]  serial_div;               // geared ce_cpu / 4 predivider
	reg [12:0] serial_presc;
	reg [1:0]  timer_div;                // fixed 6.144 MHz / 4 predivider
	reg [12:0] timer_presc;

	// A tap fires on the input tick that rolls its stage over, so it produces
	// one enable every 2^(bitpos+1) input ticks.
	function automatic tap_fire(input [12:0] cnt, input integer bitpos);
		reg [13:0] mask;
	begin
		mask     = (14'd1 << (bitpos + 1)) - 14'd1;
		tap_fire = (({1'b0, cnt} & mask) == mask);
	end
	endfunction

	// PRRUN = 0 stops and zero-clears both divider paths (TLCS-900 8-bit timer
	// manual pp.7, 14).
	wire presc_run   = trun[7];
	wire serial_tick = presc_run && ce       && (serial_div == 2'd3);
	wire timer_tick  = presc_run && ce_timer && (timer_div  == 2'd3);

	wire phi_t0_i   = serial_tick;
	wire phi_t2_i   = serial_tick && tap_fire(serial_presc, TAP_T2);
	wire phi_t8_i   = serial_tick && tap_fire(serial_presc, TAP_T8);
	wire phi_t32_i  = serial_tick && tap_fire(serial_presc, TAP_T32);
	wire phi_t1_i   = timer_tick && tap_fire(timer_presc, TAP_T1);
	wire phi_t4_i   = timer_tick && tap_fire(timer_presc, TAP_T4);
	wire phi_t16_i  = timer_tick && tap_fire(timer_presc, TAP_T16);
	wire phi_t256_i = timer_tick && tap_fire(timer_presc, TAP_T256);

	assign phi_t0  = phi_t0_i;
	assign phi_t2  = phi_t2_i;
	assign phi_t8  = phi_t8_i;
	assign phi_t32 = phi_t32_i;

	assign phi_t1  = phi_t1_i;
	assign phi_t4  = phi_t4_i;
	assign phi_t16 = phi_t16_i;
	assign prrun   = presc_run;

	assign t4run   = trun[4];
	assign t5run   = trun[5];

	// TI0 arrives as a level from the K2GE; the rising edge is the count.
	wire ti0_rise = ce_timer && ti0 && !ti0_q;

	// The two timer pairs
	wire tffcr_wr = sfr_wr && (sfr_addr == A_TFFCR);

	wire ss_wr_uc  = ss_wren && (ss_reg_addr == 8'h0B);
	wire ss_wr_tff = ss_wren && (ss_reg_addr == 8'h0C);

	wire [7:0] uc0, uc1, uc2, uc3;
	wire       intt0, intt1, intt2, intt3;
	wire       to0trg, to2trg;
	wire       buf_shift0, buf_shift2;
	wire       tff1, tff3;

	ngp_t8_pair #(.M16_INTT_LOW(M16_INTT_LOW)) pair01
	(
		.clk(clk),
		.ce(ce_timer),
		.reset(reset),
		.mod_reg(t01mod),
		.run_a(trun[0]),
		.run_b(trun[1]),
		.treg_a(treg0),
		.treg_b(treg1),
		.cmp_armed(cmp_armed[0]),
		.ext_tick(ti0_rise),
		.phi_t1(phi_t1_i),
		.phi_t4(phi_t4_i),
		.phi_t16(phi_t16_i),
		.phi_t256(phi_t256_i),
		.ff_ie(ff1ie),
		.ff_is(ff1is),
		.ff_cmd_wr(tffcr_wr),
		.ff_cmd(sfr_wdata[3:2]),
		.ss_wr_uc(ss_wr_uc),
		.ss_uc_a(ss_wdata[31:24]),
		.ss_uc_b(ss_wdata[23:16]),
		.ss_wr_tff(ss_wr_tff),
		.ss_tff(ss_wdata[17]),
		.uc_a(uc0),
		.uc_b(uc1),
		.int_a(intt0),
		.int_b(intt1),
		.match_a(to0trg),
		.buf_shift(buf_shift0),
		.tff(tff1)
	);

	// Pair 2/3 has no external clock pin: there is no TI2 on the part, so
	// T2CLK = 00 selects no clock at all (TMP95C061 datasheet p.77 prints "-").
	ngp_t8_pair #(.M16_INTT_LOW(M16_INTT_LOW)) pair23
	(
		.clk(clk),
		.ce(ce_timer),
		.reset(reset),
		.mod_reg(t23mod),
		.run_a(trun[2]),
		.run_b(trun[3]),
		.treg_a(treg2),
		.treg_b(treg3),
		.cmp_armed(cmp_armed[1]),
		.ext_tick(1'b0),
		.phi_t1(phi_t1_i),
		.phi_t4(phi_t4_i),
		.phi_t16(phi_t16_i),
		.phi_t256(phi_t256_i),
		.ff_ie(ff3ie),
		.ff_is(ff3is),
		.ff_cmd_wr(tffcr_wr),
		.ff_cmd(sfr_wdata[7:6]),
		.ss_wr_uc(ss_wr_uc),
		.ss_uc_a(ss_wdata[15:8]),
		.ss_uc_b(ss_wdata[7:0]),
		.ss_wr_tff(ss_wr_tff),
		.ss_tff(ss_wdata[18]),
		.uc_a(uc2),
		.uc_b(uc3),
		.int_a(intt2),
		.int_b(intt3),
		.match_a(to2trg),
		.buf_shift(buf_shift2),
		.tff(tff3)
	);

	assign intt    = {intt3, intt2, intt1, intt0};
	assign to1     = tff1;
	assign to3     = tff3;
	assign to2_trg = to2trg;

	// Register state update.
	// One clocked block owns every register here, so there is never more than
	// one driver. Section order matters: the double-buffer transfer goes
	// first, then CPU writes override it, then a savestate restore overrides
	// both (it only ever runs while the machine is paused).
	always @(posedge clk) begin
		if (reset) begin
			// Chip-reset values (TMP95C061 datasheet p.178).
			trun      <= 8'h00;
			t01mod    <= 8'h00;
			t23mod    <= 8'h00;
			ff3ie     <= 1'b0;
			ff3is     <= 1'b0;
			ff1ie     <= 1'b0;
			ff1is     <= 1'b0;
			trdc      <= 2'b00;
			// TREG0-3 are undefined on hardware (datasheet p.75); modelled as 0.
			treg0     <= 8'h00;
			treg1     <= 8'h00;
			treg2     <= 8'h00;
			treg3     <= 8'h00;
			treg0_buf <= 8'h00;
			treg2_buf <= 8'h00;
			serial_div   <= 2'd0;
			serial_presc <= 13'd0;
			timer_div    <= 2'd0;
			timer_presc  <= 13'd0;
			ti0_q     <= 1'b0;
			cmp_armed <= 2'b11;
		end else begin
			// -- prescaler ------------------------------------------------
			if (!presc_run) begin
				serial_div   <= 2'd0;
				serial_presc <= 13'd0;
				timer_div    <= 2'd0;
				timer_presc  <= 13'd0;
			end else begin
				if (ce) begin
					serial_div <= serial_div + 2'd1;
					if (serial_div == 2'd3) serial_presc <= serial_presc + 13'd1;
				end
				if (ce_timer) begin
					timer_div <= timer_div + 2'd1;
					if (timer_div == 2'd3) timer_presc <= timer_presc + 13'd1;
				end
			end

			if (ce_timer) ti0_q <= ti0;

			// -- double-buffer transfer (datasheet pp.75, 87, 89) ---------
			if (buf_shift0) treg0 <= treg0_buf;
			if (buf_shift2) treg2 <= treg2_buf;

			// -- CPU register writes --------------------------------------
			// The strobe is the enable; this path is deliberately not
			// ce-gated.
			if (sfr_wr) begin
				case (sfr_addr)
					A_TRUN: begin
						// TRUN bits 5:4 are T5RUN/T4RUN and belong to the
						// 16-bit timers. This module owns the address so it
						// stores them and publishes them on t4run / t5run.
						trun <= {sfr_wdata[7], 1'b0, sfr_wdata[5:0]};
					end
					A_TREG0: begin
						// TRDC<TR0DE> = 1 sends the write to the buffer only.
						treg0_buf <= sfr_wdata;
						if (!trdc[0]) treg0 <= sfr_wdata;
						cmp_armed[0] <= 1'b0;
					end
					A_TREG1: begin
						treg1        <= sfr_wdata;
						cmp_armed[0] <= 1'b1;
					end
					A_T01MOD: t01mod <= sfr_wdata;
					A_TFFCR: begin
						ff3ie <= sfr_wdata[5];
						ff3is <= sfr_wdata[4];
						ff1ie <= sfr_wdata[1];
						ff1is <= sfr_wdata[0];
					end
					A_TREG2: begin
						treg2_buf <= sfr_wdata;
						if (!trdc[1]) treg2 <= sfr_wdata;
						cmp_armed[1] <= 1'b0;
					end
					A_TREG3: begin
						treg3        <= sfr_wdata;
						cmp_armed[1] <= 1'b1;
					end
					A_T23MOD: t23mod <= sfr_wdata;
					A_TRDC:   trdc   <= sfr_wdata[1:0];
					default:  ;
				endcase
			end

			// -- savestate restore ----------------------------------------
			// Honoured only while paused (the shell gates ss_wren), and last
			// in the block so a restore always wins the cycle.
			if (ss_wren) begin
				case (ss_reg_addr)
					8'h08: begin
						trun   <= {ss_wdata[31], 1'b0, ss_wdata[29:24]};
						t01mod <= ss_wdata[23:16];
						t23mod <= ss_wdata[15:8];
						ff3ie  <= ss_wdata[5];
						ff3is  <= ss_wdata[4];
						ff1ie  <= ss_wdata[1];
						ff1is  <= ss_wdata[0];
					end
					8'h09: begin
						treg0 <= ss_wdata[31:24];
						treg1 <= ss_wdata[23:16];
						treg2 <= ss_wdata[15:8];
						treg3 <= ss_wdata[7:0];
					end
					8'h0A: begin
						treg0_buf <= ss_wdata[31:24];
						treg2_buf <= ss_wdata[23:16];
						trdc      <= ss_wdata[9:8];
					end
					8'h0C: begin
						timer_div   <= ss_wdata[14:13];
						timer_presc <= ss_wdata[12:0];
					end
					8'h0D: begin
						serial_div   <= ss_wdata[16:15];
						serial_presc <= ss_wdata[14:2];
						ti0_q        <= ss_wdata[1];
					end
					8'h0E:   cmp_armed <= ss_wdata[1:0];
					default: ;
				endcase
			end
		end
	end

	// Savestate tap, words 0x08-0x0F. Word 0x0E carries the 16-bit comparator
	// interlock. Word 0x0D's to0trg_pending field reads 0: the TO0TRG/TO2TRG
	// cascade is evaluated in the same ce tick as the match, as the TMP95C061
	// datasheet p.83 Fig 3.8(10) draws it, so there is no pending bit to keep.
	reg [31:0] ss_rd;
	always_comb begin
		case (ss_reg_addr)
			8'h08:   ss_rd = {trun_rb, t01mod, t23mod, tffcr_rb};
			8'h09:   ss_rd = {treg0, treg1, treg2, treg3};
			8'h0A:   ss_rd = {treg0_buf, treg2_buf, trdc_rb, 8'h00};
			8'h0B:   ss_rd = {uc0, uc1, uc2, uc3};
			8'h0C:   ss_rd = {13'h0, tff3, tff1, 2'b00, timer_div, timer_presc};
			8'h0D:   ss_rd = {15'h0, serial_div, serial_presc, ti0_q, 1'b0};
			8'h0E:   ss_rd = {30'h0, cmp_armed};
			default: ss_rd = 32'h0;
		endcase
	end
	assign ss_rdata = ss_rd;

	// The timer block holds no multi-cycle transaction, so it is always ready
	// to freeze. The freeze itself is k2_soc's: it withholds `ce` from the
	// whole chip once every stage reports ready.
	assign pause_ready = 1'b1;

	// sfr_rd is in the normative port group but this module has no register
	// with a read side effect; pause_req is unused for the same reason.
	// TRUN bit 6 is stored as a constant 0 so the tap word keeps its shape;
	// TO0TRG is consumed inside pair01 and no shell consumer needs it.
	wire unused_ok = &{1'b0, sfr_rd, pause_req, trun[6], to0trg,
	                   ss_wdata[30], ss_wdata[0]};

endmodule


// One pair of 8-bit timers: two up-counters, the comparators, the pair mode
// logic and the pair's timer flip-flop (TMP95C061 datasheet pp.74-90).
// "Timer A" is the even timer (T0 / T2), "timer B" the odd one (T1 / T3).
//
// Every time-advancing input is already a one-ce-wide enable produced by the
// parent; ce is taken as well so the counters and the flip-flop advance only
// on machine states, which is what freezing for a savestate relies on.
module ngp_t8_pair
#(
	parameter M16_INTT_LOW = 0
)
(
	input  wire       clk,
	input  wire       ce,
	input  wire       reset,

	input  wire [7:0] mod_reg,       // T01MOD / T23MOD
	input  wire       run_a,         // TRUN<T0RUN> / <T2RUN>
	input  wire       run_b,         // TRUN<T1RUN> / <T3RUN>
	input  wire [7:0] treg_a,        // TREG0 / TREG2, after the double buffer
	input  wire [7:0] treg_b,        // TREG1 / TREG3
	input  wire       cmp_armed,     // 16-bit mode TREG0-first interlock

	input  wire       ext_tick,      // TI0 rising edge; tied 0 on pair 2/3
	input  wire       phi_t1,
	input  wire       phi_t4,
	input  wire       phi_t16,
	input  wire       phi_t256,

	input  wire       ff_ie,         // FF1IE / FF3IE
	input  wire       ff_is,         // FF1IS / FF3IS
	input  wire       ff_cmd_wr,     // TFFCR write strobe
	input  wire [1:0] ff_cmd,        // that write's control field

	input  wire       ss_wr_uc,
	input  wire [7:0] ss_uc_a,
	input  wire [7:0] ss_uc_b,
	input  wire       ss_wr_tff,
	input  wire       ss_tff,

	output reg  [7:0] uc_a,
	output reg  [7:0] uc_b,
	output wire       int_a,         // INTT0 / INTT2
	output wire       int_b,         // INTT1 / INTT3
	output wire       match_a,       // TO0TRG / TO2TRG
	output wire       buf_shift,     // move the register buffer into TREG0/2
	output reg        tff            // TFF1 / TFF3
);

	localparam [1:0] MODE_8BIT  = 2'b00;
	localparam [1:0] MODE_16BIT = 2'b01;
	localparam [1:0] MODE_PPG   = 2'b10;
	localparam [1:0] MODE_PWM   = 2'b11;

	wire [1:0] md   = mod_reg[7:6];
	wire [1:0] pwmc = mod_reg[5:4];
	wire [1:0] selb = mod_reg[3:2];
	wire [1:0] sela = mod_reg[1:0];

	// -- timer A input clock (datasheet pp.76-77) -----------------------
	reg ta_sel;
	always_comb begin
		case (sela)
			2'b00:   ta_sel = ext_tick;
			2'b01:   ta_sel = phi_t1;
			2'b10:   ta_sel = phi_t4;
			default: ta_sel = phi_t16;
		endcase
	end
	wire ta = ta_sel && run_a;

	// -- timer A comparators --------------------------------------------
	// Compare-after-increment, so TREG = N gives a period of exactly N input
	// clocks and TREG = 0 gives 256 - the match then lands on the counter's
	// overflow (datasheet p.74). The datasheet p.81 example sets
	// TREG1 = 100 for an interrupt every 100 phi_T1 periods.
	wire [7:0] uca_next = uc_a + 8'd1;
	wire [7:0] ucb_next = uc_b + 8'd1;

	wire ma      = ta && (uca_next == treg_a);
	wire ppg_per = ta && (uca_next == treg_b);   // PPG cycle (datasheet p.86)

	// PWM cycle counter: PWM00 = 01/10/11 selects 2^6-1 / 2^7-1 / 2^8-1.
	// PWM00 = 00 is printed "-" and selects no cycle at all.
	reg [7:0] pwm_lim;
	always_comb begin
		case (pwmc)
			2'b01:   pwm_lim = 8'd63;
			2'b10:   pwm_lim = 8'd127;
			2'b11:   pwm_lim = 8'd255;
			default: pwm_lim = 8'd0;
		endcase
	end
	wire pwm_ovf = ta && (pwmc != 2'b00) && (uca_next == pwm_lim);

	// 16-bit mode: the pair is one 16-bit counter compared against
	// TREG1 x 256 + TREG0 (datasheet p.85 Fig 3.8(11)). The p.84 example
	// sets TREG1 = F4H, TREG0 = 24H for a period of 62500 input clocks.
	wire [15:0] uc16_next = {uc_b, uc_a} + 16'd1;
	wire m16     = ta && cmp_armed && (uc16_next == {treg_b, treg_a});
	wire m16_low = ta && cmp_armed && (uc16_next[7:0] == treg_a);

	assign match_a = (md == MODE_16BIT) ? m16_low : ma;

	// -- timer B input clock --------------------------------------------
	reg tb_sel;
	always_comb begin
		if (md == MODE_16BIT) begin
			// The overflow of the low half, regardless of T1CLK (datasheet p.84).
			tb_sel = ta && (uc_a == 8'hFF);
		end else begin
			case (selb)
				2'b00:   tb_sel = match_a;      // TO0TRG / TO2TRG
				2'b01:   tb_sel = phi_t1;
				2'b10:   tb_sel = phi_t16;
				default: tb_sel = phi_t256;
			endcase
		end
	end
	wire tb = tb_sel && run_b;

	wire mb = tb && (ucb_next == treg_b);

	// -- per-mode assembly ----------------------------------------------
	reg clr_a, clr_b, ia, ib, flip, shift;
	always_comb begin
		case (md)
			MODE_16BIT: begin
				// Both counters clear on the 16-bit match, which raises
				// INTT1 (INTT3). See M16_INTT_LOW at the top of this file for
				// the low-byte interrupt.
				clr_a = m16;
				clr_b = m16;
				ia    = (M16_INTT_LOW != 0) ? m16_low : 1'b0;
				ib    = m16;
				flip  = ff_ie && m16;
				shift = 1'b0;
			end

			MODE_PPG: begin
				// One counter and two comparators: TREG0 sets the duty and
				// TREG1 the cycle, and both invert the flip-flop
				// (datasheet p.86 Fig 3.8(12)). Timer B has no comparator here,
				// so it free-runs if TRUN says so - the datasheet simply says
				// "timer 1 and timer 3 cannot be used" (datasheet p.85).
				clr_a = ppg_per;
				clr_b = 1'b0;
				ia    = ma;
				ib    = ppg_per;
				flip  = ff_ie && (ma || ppg_per);
				shift = ppg_per;
			end

			MODE_PWM: begin
				// Timer A is the PWM generator: the output inverts on the
				// TREG0 match and again on the 2^n-1 overflow, and only the
				// overflow raises INTT0 (datasheet p.88). Timer B stays a plain
				// 8-bit timer.
				clr_a = pwm_ovf;
				clr_b = mb;
				ia    = pwm_ovf;
				ib    = mb;
				flip  = ff_ie && (ma || pwm_ovf);
				shift = pwm_ovf;
			end

			MODE_8BIT: begin
				// Two independent interval timers; FFnIS picks which one
				// inverts the flip-flop (8-bit timer manual p.13).
				clr_a = ma;
				clr_b = mb;
				ia    = ma;
				ib    = mb;
				flip  = ff_ie && (ff_is ? mb : ma);
				shift = 1'b0;
			end
		endcase
	end

	assign int_a     = ia;
	assign int_b     = ib;
	assign buf_shift = shift;

	// State update
	always @(posedge clk) begin
		if (reset) begin
			uc_a <= 8'h00;
			uc_b <= 8'h00;
			tff  <= 1'b0;
		end else begin
			if (ce) begin
				// TRUN bit = 0 stops AND clears; it is a level, not an event
				// (8-bit timer manual p.14, "0: Stop & Clear").
				if (!run_a)      uc_a <= 8'h00;
				else if (clr_a)  uc_a <= 8'h00;
				else if (ta)     uc_a <= uca_next;

				if (!run_b)      uc_b <= 8'h00;
				else if (clr_b)  uc_b <= 8'h00;
				else if (tb)     uc_b <= ucb_next;

				if (flip) tff <= ~tff;
			end

			// A software command in TFFCR is a bus strobe, not a machine
			// event, so it is not ce-gated, and it wins over a match landing
			// in the same clk_sys cycle (8-bit timer manual p.13).
			if (ff_cmd_wr) begin
				case (ff_cmd)
					2'b00:   tff <= ~tff;
					2'b01:   tff <= 1'b1;
					2'b10:   tff <= 1'b0;
					default: tff <= tff;    // 11 = don't care
				endcase
			end

			if (ss_wr_uc) begin
				uc_a <= ss_uc_a;
				uc_b <= ss_uc_b;
			end
			if (ss_wr_tff) tff <= ss_tff;
		end
	end

endmodule
