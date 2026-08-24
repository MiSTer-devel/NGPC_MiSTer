// Copyright (c) 2026 Jamie Blanks

// t900_mcu - the peripheral half of the SNK K2-CHIP's TMP95C061-class MCU
// block. It owns exactly the 128 bytes at CPU addresses 0x000000-0x00007F and
// the hardware behind them (TMP95C061 datasheet p.173): the internal SFR bus,
// the parent-side read mux, the savestate tap mux, and the wiring between the
// eight submodules. It holds no register of its own - every register and every
// reset value lives in the submodule that owns it.
//
// It does not own the SNK block 0x80-0xBF (clock gear, RTC, sound, inputs,
// power - see rtl/soc/ngp_sysreg.sv and rtl/soc/ngp_rtc.sv), the micro-DMA
// channel registers DMAS/DMAD/DMAC/DMAM (CPU control registers inside
// t900_seq; the shell owns only the four start vectors DMA0V-DMA3V), or the
// internal memory decode (ngp_csc answers for the external bus only; the
// k2_soc fabric decodes the internal regions ahead of it).
//
// The SFR bus is byte wide with no lanes: the internal I/O area is 8-bit, so
// the BIU never presents a 16-bit cycle there and a word store arrives as two
// byte cycles (datasheet p.52 Table 3.6(2)). Reads are combinational and cost
// zero wait states. sfr_wr and sfr_rd are one-clk_sys strobes produced by the
// fabric at the T2 edge, fanned out to the owning submodule only; a register
// write is not additionally gated by ce, because the strobe is the enable.
//
// The read mux is a case, never a wired-OR: a stuck slave must fail visibly.
// ngp_ports is the default arm because its addresses are the most scattered,
// so the unmapped gaps land there. Address 0x00 reads 0x00 for Ogre Battle's
// null-data read; the other gaps and write-only registers read 0xFF. Both are
// behavioural choices, not documented facts.
//
// Two signals look internal but are pins. TI0, the K2GE horizontal blanking
// line, is the external clock input of 8-bit timer 0 and not an interrupt: it
// is PA1, so the fabric drives pa_in[1], ngp_ports reports it on ti0_pin and
// the shell routes that to ngp_t8. The WAIT pin is PA0 and reaches ngp_csc the
// same way (nothing on the NGP board drives it). TO3 is timer flip-flop 3 on
// PA3, which the board wires to the Z80's interrupt input; it leaves ngp_t8,
// passes through ngp_ports gated by PAFC and PACR bit 3, and leaves on
// to3_z80 as the pin value. The Z80-side latch belongs to ngp_snd.
//
// Savestate: 48 words at ss_reg_addr 0x00-0x2F, muxed with the same case
// idiom. ss_wren is honoured only while paused and is routed to the owning
// submodule alone. pause_ready is the AND of the eight submodule bits, so it
// rises only when the ADC has drained a conversion and both serial channels
// have finished the byte in their shift registers.

module t900_mcu
(
	input  wire        clk,
	input  wire        ce,            // fosc/2 / gear execution-state enable
	input  wire        ce_periph,     // 6.144 MHz / gear peripheral clock
	input  wire        ce_timer,      // fixed 6.144 MHz peripheral enable
	input  wire        reset,

	// SFR bus, area 0x000000-0x00007F.
	// sfr_wr / sfr_rd are the fabric's one-clk_sys io_wr / io_rd strobes.
	input  wire [6:0]  sfr_addr,
	input  wire [7:0]  sfr_wdata,
	input  wire        sfr_wr,
	input  wire        sfr_rd,
	output wire [7:0]  sfr_rdata,

	// External bus controller (ngp_csc)
	input  wire [23:0] plan_addr,     // the address the BIU is about to drive
	output wire        plan_width8,   // combinational, same cycle
	output wire [3:0]  plan_cs_n,
	input  wire [23:0] bus_addr,      // the address on the bus
	input  wire        bus_active,
	output wire [2:0]  bus_waits,
	output wire [3:0]  cs_n,

	// CPU interrupt / micro-DMA / standby
	output wire        int_req,
	output wire [2:0]  int_level,
	output wire [7:0]  int_vector,
	output wire [4:0]  int_id,
	input  wire        int_ack,
	input  wire [4:0]  int_ack_id,
	output wire [3:0]  dma_req,
	input  wire [3:0]  dma_ack,
	input  wire [3:0]  dma_end,
	output wire        halt_release,
	input  wire        halted,        // CPU is in HALT
	output wire [1:0]  haltm,         // WDMOD[3:2], for the standby controller
	output wire        wdt_warm,      // WDMOD[4],   likewise (warm-up length)
	output wire        wdtout_n,      // watchdog pin
	output wire        wdt_reset,     // WDMOD<RESCR> qualified overflow

	// SoC interrupt sources
	input  wire        nmi_n,         // power button, gated by 0xB3.2 in sysreg
	input  wire        int0,          // RTC alarm
	input  wire        int4,          // K2GE VBlank
	input  wire        int5,          // Z80 wrote 0xC000
	input  wire        int6,          // unbonded on the NGP, tie 0
	input  wire        int7,          // panel switch
	output wire        to3_z80,       // PA3 pin: the Z80 interrupt line

	// Analog
	input  wire [9:0]  an0,           // battery model; AN1-AN3 unbonded

	// Serial link pins
	input  wire        rxd0,
	input  wire        cts0_n,
	output wire        txd0,
	output wire        txd0_oe,
	input  wire        sclk0_in,
	output wire        sclk0_out,
	output wire        sclk0_oe,
	input  wire        rxd1,
	output wire        txd1,
	output wire        txd1_oe,
	input  wire        sclk1_in,
	output wire        sclk1_out,
	output wire        sclk1_oe,

	// Port pins, split DIN/DOUT/OE
	// PA1 carries TI0 (K2GE HBlank) and PA0 the WAIT pin; see the header.
	input  wire [7:0]  p1_in,  output wire [7:0] p1_out,  output wire [7:0] p1_oe,
	output wire [7:0]  p2_out,
	input  wire [7:0]  p5_in,  output wire [7:0] p5_out,  output wire [7:0] p5_oe,
	output wire [7:0]  p6_out,
	input  wire [7:0]  p7_in,  output wire [7:0] p7_out,  output wire [7:0] p7_oe,
	input  wire [7:0]  p8_in,  output wire [7:0] p8_out,  output wire [7:0] p8_oe,
	input  wire [3:0]  p9_in,
	input  wire [3:0]  pa_in,  output wire [3:0] pa_out,  output wire [3:0] pa_oe,
	input  wire [7:0]  pb_in,  output wire [7:0] pb_out,  output wire [7:0] pb_oe,

	// Savestate tap, words 0x00-0x2F
	input  wire [7:0]  ss_reg_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        restore_hold,
	input  wire        pause_req,
	output wire        pause_ready
);

	// SFR ownership. One decode drives both the read mux and the strobe
	// fan-out, so the two can never disagree about who owns an address.

	localparam [2:0] OWN_PORTS = 3'd0;   // 0x01-0x1F, 0x2C-0x2F, 0x4C-0x4E + gaps
	localparam [2:0] OWN_T8    = 3'd1;   // 0x20, 0x22-0x29
	localparam [2:0] OWN_T16   = 3'd2;   // 0x30-0x3A, 0x40-0x49
	localparam [2:0] OWN_CSC   = 3'd3;   // 0x3C-0x3F, 0x5A-0x5F, 0x68-0x6C
	localparam [2:0] OWN_SIO   = 3'd4;   // 0x50-0x58
	localparam [2:0] OWN_ADC   = 3'd5;   // 0x60-0x67, 0x6D
	localparam [2:0] OWN_WDT   = 3'd6;   // 0x6E, 0x6F
	localparam [2:0] OWN_INTC  = 3'd7;   // 0x70-0x7F

	// Magnitude compares only -- no arithmetic, one LUT level per arm.
	function automatic [2:0] sfr_owner(input [6:0] a);
	begin
		case (a[6:4])
			3'h2:    sfr_owner = ((a == 7'h20) || ((a >= 7'h22) && (a <= 7'h29)))
			                       ? OWN_T8 : OWN_PORTS;
			3'h3:    sfr_owner = (a <= 7'h3A) ? OWN_T16
			                   : (a >= 7'h3C) ? OWN_CSC : OWN_PORTS;
			3'h4:    sfr_owner = (a <= 7'h49) ? OWN_T16 : OWN_PORTS;
			3'h5:    sfr_owner = (a <= 7'h58) ? OWN_SIO
			                   : (a >= 7'h5A) ? OWN_CSC : OWN_PORTS;
			3'h6:    sfr_owner = (a <= 7'h67) ? OWN_ADC
			                   : (a <= 7'h6C) ? OWN_CSC
			                   : (a == 7'h6D) ? OWN_ADC : OWN_WDT;
			3'h7:    sfr_owner = OWN_INTC;
			default: sfr_owner = OWN_PORTS;              // 0x00-0x1F
		endcase
	end
	endfunction

	wire [2:0] owner = sfr_owner(sfr_addr);

	wire [7:0] rd_ports, rd_t8, rd_t16, rd_csc, rd_sio, rd_adc, rd_wdt, rd_intc;

	// A case, never a wired-OR: a stuck slave must fail visibly.
	reg [7:0] sfr_rdata_r;
	always_comb begin
		case (owner)
			OWN_T8:   sfr_rdata_r = rd_t8;
			OWN_T16:  sfr_rdata_r = rd_t16;
			OWN_CSC:  sfr_rdata_r = rd_csc;
			OWN_SIO:  sfr_rdata_r = rd_sio;
			OWN_ADC:  sfr_rdata_r = rd_adc;
			OWN_WDT:  sfr_rdata_r = rd_wdt;
			OWN_INTC: sfr_rdata_r = rd_intc;
			default:  sfr_rdata_r = rd_ports;   // scattered map + every gap
		endcase
	end

	assign sfr_rdata = sfr_rdata_r;

	// Strobe fan-out. Exactly one of each group can be high in any cycle.
	wire wr_ports = sfr_wr && (owner == OWN_PORTS);
	wire wr_t8    = sfr_wr && (owner == OWN_T8);
	wire wr_t16   = sfr_wr && (owner == OWN_T16);
	wire wr_csc   = sfr_wr && (owner == OWN_CSC);
	wire wr_sio   = sfr_wr && (owner == OWN_SIO);
	wire wr_adc   = sfr_wr && (owner == OWN_ADC);
	wire wr_wdt   = sfr_wr && (owner == OWN_WDT);
	wire wr_intc  = sfr_wr && (owner == OWN_INTC);

	wire rd_s_ports = sfr_rd && (owner == OWN_PORTS);
	wire rd_s_t8    = sfr_rd && (owner == OWN_T8);
	wire rd_s_t16   = sfr_rd && (owner == OWN_T16);
	wire rd_s_csc   = sfr_rd && (owner == OWN_CSC);
	wire rd_s_sio   = sfr_rd && (owner == OWN_SIO);
	wire rd_s_adc   = sfr_rd && (owner == OWN_ADC);
	wire rd_s_wdt   = sfr_rd && (owner == OWN_WDT);
	wire rd_s_intc  = sfr_rd && (owner == OWN_INTC);

	// Savestate slice

	localparam [2:0] SS_INTC  = 3'd0;    // 0x00-0x07
	localparam [2:0] SS_T8    = 3'd1;    // 0x08-0x0F
	localparam [2:0] SS_T16   = 3'd2;    // 0x10-0x17
	localparam [2:0] SS_SIO   = 3'd3;    // 0x18-0x1B
	localparam [2:0] SS_ADC   = 3'd4;    // 0x1C-0x1F
	localparam [2:0] SS_WDT   = 3'd5;    // 0x20-0x21
	localparam [2:0] SS_CSC   = 3'd6;    // 0x22-0x27
	localparam [2:0] SS_PORTS = 3'd7;    // 0x28-0x2F

	// Words outside 0x00-0x2F are not this shell's; they read 0 and no write
	// reaches any submodule, so a walker sweeping the whole 256-word space
	// cannot disturb the shell.
	wire ss_mine = (ss_reg_addr <= 8'h2F);

	function automatic [2:0] ss_owner(input [7:0] a);
	begin
		if      (a <= 8'h07) ss_owner = SS_INTC;
		else if (a <= 8'h0F) ss_owner = SS_T8;
		else if (a <= 8'h17) ss_owner = SS_T16;
		else if (a <= 8'h1B) ss_owner = SS_SIO;
		else if (a <= 8'h1F) ss_owner = SS_ADC;
		else if (a <= 8'h21) ss_owner = SS_WDT;
		else if (a <= 8'h27) ss_owner = SS_CSC;
		else                 ss_owner = SS_PORTS;
	end
	endfunction

	wire [2:0] ss_own = ss_owner(ss_reg_addr);

	wire [31:0] ss_rd_intc, ss_rd_t8, ss_rd_t16, ss_rd_sio;
	wire [31:0] ss_rd_adc, ss_rd_wdt, ss_rd_csc, ss_rd_ports;

	reg [31:0] ss_rdata_r;
	always_comb begin
		if (!ss_mine) begin
			ss_rdata_r = 32'h0000_0000;
		end else begin
			case (ss_own)
				SS_INTC: ss_rdata_r = ss_rd_intc;
				SS_T8:   ss_rdata_r = ss_rd_t8;
				SS_T16:  ss_rdata_r = ss_rd_t16;
				SS_SIO:  ss_rdata_r = ss_rd_sio;
				SS_ADC:  ss_rdata_r = ss_rd_adc;
				SS_WDT:  ss_rdata_r = ss_rd_wdt;
				SS_CSC:  ss_rdata_r = ss_rd_csc;
				default: ss_rdata_r = ss_rd_ports;
			endcase
		end
	end

	assign ss_rdata = ss_rdata_r;

	wire pr_intc, pr_t8, pr_t16, pr_sio, pr_adc, pr_wdt, pr_csc, pr_ports;

	assign pause_ready = pr_intc & pr_t8 & pr_t16 & pr_sio &
	                     pr_adc  & pr_wdt & pr_csc & pr_ports;

	// A restore is honoured only while the whole shell is parked, and only by
	// the module that owns the word.
	wire ss_wr_eff = ss_wren && (pause_ready || restore_hold) && ss_mine;

	wire ss_wr_intc  = ss_wr_eff && (ss_own == SS_INTC);
	wire ss_wr_t8    = ss_wr_eff && (ss_own == SS_T8);
	wire ss_wr_t16   = ss_wr_eff && (ss_own == SS_T16);
	wire ss_wr_sio   = ss_wr_eff && (ss_own == SS_SIO);
	wire ss_wr_adc   = ss_wr_eff && (ss_own == SS_ADC);
	wire ss_wr_wdt   = ss_wr_eff && (ss_own == SS_WDT);
	wire ss_wr_csc   = ss_wr_eff && (ss_own == SS_CSC);
	wire ss_wr_ports = ss_wr_eff && (ss_own == SS_PORTS);

	// Inter-module signals
	wire       to1, to3, to2_trg;
	wire [3:0] intt;
	wire       phi_t0, phi_t2, phi_t8, phi_t32;
	wire       phi_t1, phi_t4, phi_t16, prrun;
	wire       t4run, t5run;

	wire       to4, to5, to6;
	wire [3:0] inttr;
	wire [1:0] cap12m, cap34m;
	wire       pg0t, pg1t;

	wire [1:0] intrx, inttx, intrx_clear;
	wire       txd0_i, txd1_i, sclk0_i, sclk1_i;

	wire       intad, intad_clear;
	wire       intwd;

	wire [3:0] cs_n_i;
	wire [3:0] pb_is_input;
	wire       ti0_pin, wait_pin;

	wire [3:0] pa_out_i, pa_oe_i;

	// Interrupt controller: 0x70-0x7F, tap words 0x00-0x07
	ngp_intc u_intc
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.sfr_addr(sfr_addr),
		.sfr_wdata(sfr_wdata),
		.sfr_wr(wr_intc),
		.sfr_rd(rd_s_intc),
		.sfr_rdata(rd_intc),

		.nmi_n(nmi_n),
		.intwd(intwd),

		.int0(int0),
		.int4(int4),
		.int5(int5),
		.int6(int6),
		.int7(int7),
		.pb_is_input(pb_is_input),
		.cap12m(cap12m),
		.cap34m(cap34m),

		.intt(intt),
		.inttr(inttr),
		.intrx(intrx),
		.inttx(inttx),
		.intrx_clear(intrx_clear),
		.intad(intad),
		.intad_clear(intad_clear),

		.int_req(int_req),
		.int_level(int_level),
		.int_vector(int_vector),
		.int_id(int_id),
		.int_ack(int_ack),
		.int_ack_id(int_ack_id),

		.dma_req(dma_req),
		.dma_ack(dma_ack),
		.dma_end(dma_end),

		.halt_release(halt_release),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wr_intc),
		.ss_rdata(ss_rd_intc),
		.pause_req(pause_req),
		.pause_ready(pr_intc)
	);

	// 8-bit timers and the prescaler: 0x20, 0x22-0x29, tap words 0x08-0x0F.
	// TI0 arrives as the PA1 pin state, which is where the K2GE's HBlank line
	// is bonded.
	ngp_t8 u_t8
	(
		.clk(clk),
		.ce(ce_periph),
		.ce_timer(ce_timer),
		.reset(reset),

		.sfr_addr(sfr_addr),
		.sfr_wdata(sfr_wdata),
		.sfr_wr(wr_t8),
		.sfr_rd(rd_s_t8),
		.sfr_rdata(rd_t8),

		.ti0(ti0_pin),
		.to1(to1),
		.to3(to3),
		.intt(intt),
		.to2_trg(to2_trg),

		.phi_t0(phi_t0),
		.phi_t2(phi_t2),
		.phi_t8(phi_t8),
		.phi_t32(phi_t32),
		.phi_t1(phi_t1),
		.phi_t4(phi_t4),
		.phi_t16(phi_t16),
		.prrun(prrun),
		.t4run(t4run),
		.t5run(t5run),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wr_t8),
		.ss_rdata(ss_rd_t8),
		.pause_req(pause_req),
		.pause_ready(pr_t8)
	);

	// 16-bit timers: 0x30-0x3A, 0x40-0x49, tap words 0x10-0x17.
	// TI4/TI5/TI6/TI7 are PB0/PB1/PB4/PB5. On the NGP those pins carry
	// INT4-INT7, which are fed internally (VBlank, Z80, ...), so the capture
	// inputs see the port pins themselves.
	ngp_t16 u_t16
	(
		.clk(clk),
		.ce(ce_timer),
		.reset(reset),

		.sfr_addr(sfr_addr),
		.sfr_wdata(sfr_wdata),
		.sfr_wr(wr_t16),
		.sfr_rd(rd_s_t16),
		.sfr_rdata(rd_t16),

		.ti4(pb_in[0]),
		.ti5(pb_in[1]),
		.ti6(pb_in[4]),
		.ti7(pb_in[5]),
		.tff1(to1),
		.phi_t1(phi_t1),
		.phi_t4(phi_t4),
		.phi_t16(phi_t16),
		.prrun(prrun),
		.t4run(t4run),
		.t5run(t5run),
		.to4(to4),
		.to5(to5),
		.to6(to6),
		.inttr(inttr),
		.cap12m(cap12m),
		.cap34m(cap34m),
		.pg0t(pg0t),
		.pg1t(pg1t),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wr_t16),
		.ss_rdata(ss_rd_t16),
		.pause_req(pause_req),
		.pause_ready(pr_t16)
	);

	// Chip select / wait controller and the DRAM stub: 0x3C-0x3F, 0x5A-0x5F,
	// 0x68-0x6C, tap words 0x22-0x27
	ngp_csc u_csc
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.sfr_addr(sfr_addr),
		.sfr_wdata(sfr_wdata),
		.sfr_wr(wr_csc),
		.sfr_rd(rd_s_csc),
		.sfr_rdata(rd_csc),

		.plan_addr(plan_addr),
		.plan_width8(plan_width8),
		.plan_cs_n(plan_cs_n),
		.bus_addr(bus_addr),
		.bus_active(bus_active),
		.wait_pin(wait_pin),
		.bus_waits(bus_waits),
		.cs_n(cs_n_i),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wr_csc),
		.ss_rdata(ss_rd_csc),
		.pause_req(pause_req),
		.pause_ready(pr_csc)
	);

	assign cs_n = cs_n_i;

	// Serial channels: 0x50-0x58, tap words 0x18-0x1B
	ngp_sio u_sio
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.sfr_addr(sfr_addr),
		.sfr_wdata(sfr_wdata),
		.sfr_wr(wr_sio),
		.sfr_rd(rd_s_sio),
		.sfr_rdata(rd_sio),

		.phi_t0(phi_t0),
		.phi_t2(phi_t2),
		.phi_t8(phi_t8),
		.phi_t32(phi_t32),
		.to2_trg(to2_trg),

		.rxd0(rxd0),
		.cts0_n(cts0_n),
		.txd0(txd0_i),
		.txd0_oe(txd0_oe),
		.sclk0_in(sclk0_in),
		.sclk0_out(sclk0_i),
		.sclk0_oe(sclk0_oe),
		.rxd1(rxd1),
		.txd1(txd1_i),
		.txd1_oe(txd1_oe),
		.sclk1_in(sclk1_in),
		.sclk1_out(sclk1_i),
		.sclk1_oe(sclk1_oe),

		.intrx(intrx),
		.inttx(inttx),
		.intrx_clear(intrx_clear),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wr_sio),
		.ss_rdata(ss_rd_sio),
		.pause_req(pause_req),
		.pause_ready(pr_sio)
	);

	assign txd0    = txd0_i;
	assign txd1    = txd1_i;
	assign sclk0_out = sclk0_i;
	assign sclk1_out = sclk1_i;

	// A/D converter: 0x60-0x67, 0x6D, tap words 0x1C-0x1F
	ngp_adc u_adc
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.sfr_addr(sfr_addr),
		.sfr_wdata(sfr_wdata),
		.sfr_wr(wr_adc),
		.sfr_rd(rd_s_adc),
		.sfr_rdata(rd_adc),

		.an0(an0),
		.intad(intad),
		.intad_clear(intad_clear),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wr_adc),
		.ss_rdata(ss_rd_adc),
		.pause_req(pause_req),
		.pause_ready(pr_adc)
	);

	// Watchdog: 0x6E, 0x6F, tap words 0x20-0x21.
	// INTWD is non-maskable but it is not the NMI pin: vector 0x24 at priority
	// 7, against the pin's 0x20 (datasheet p.12). It goes to the intc's INTWD
	// channel and never to nmi_n.
	ngp_wdt u_wdt
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.sfr_addr(sfr_addr),
		.sfr_wdata(sfr_wdata),
		.sfr_wr(wr_wdt),
		.sfr_rd(rd_s_wdt),
		.sfr_rdata(rd_wdt),

		.halted(halted),
		.haltm(haltm),
		.warm(wdt_warm),
		.intwd(intwd),
		.wdtout_n(wdtout_n),
		.int_reset(wdt_reset),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wr_wdt),
		.ss_rdata(ss_rd_wdt),
		.pause_req(pause_req),
		.pause_ready(pr_wdt)
	);

	// Ports and the pattern generators: 0x01-0x1F, 0x2C-0x2F, 0x4C-0x4E, every
	// unmapped gap, tap words 0x28-0x2F
	ngp_ports u_ports
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.sfr_addr(sfr_addr),
		.sfr_wdata(sfr_wdata),
		.sfr_wr(wr_ports),
		.sfr_rd(rd_s_ports),
		.sfr_rdata(rd_ports),

		.p1_in(p1_in), .p1_out(p1_out), .p1_oe(p1_oe),
		.p2_out(p2_out),
		.p5_in(p5_in), .p5_out(p5_out), .p5_oe(p5_oe),
		.p6_out(p6_out),
		.p7_in(p7_in), .p7_out(p7_out), .p7_oe(p7_oe),
		.p8_in(p8_in), .p8_out(p8_out), .p8_oe(p8_oe),
		.p9_in(p9_in),
		.pa_in(pa_in), .pa_out(pa_out_i), .pa_oe(pa_oe_i),
		.pb_in(pb_in), .pb_out(pb_out), .pb_oe(pb_oe),

		.to1(to1), .to3(to3),
		.to4(to4), .to5(to5), .to6(to6),
		.pg0t(pg0t), .pg1t(pg1t),
		.txd0(txd0_i), .sclk0(sclk0_i), .txd1(txd1_i), .sclk1(sclk1_i),
		.cs_n(cs_n_i),
		.ti0_pin(ti0_pin),
		.wait_pin(wait_pin),
		.pb_is_input(pb_is_input),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wr_ports),
		.ss_rdata(ss_rd_ports),
		.pause_req(pause_req),
		.pause_ready(pr_ports)
	);

	assign pa_out = pa_out_i;
	assign pa_oe  = pa_oe_i;

	// The Z80 interrupt line is the PA3 pin, not TFF3 directly: it carries what
	// the port drives while PACR<PA3C> makes it an output, and the pin's own
	// level otherwise. The BIOS arms the path with PAFC = 0x0C then
	// PACR = 0x0C.
	assign to3_z80 = pa_oe_i[3] ? pa_out_i[3] : pa_in[3];

	// PB0/PB1/PB4/PB5 reach ngp_t16 as capture inputs and ngp_ports as port
	// pins; the other four bits of pb_in are read by ngp_ports alone, so the
	// aggregate keeps the lint honest about which bits this level touches.
	wire unused_ok = &{1'b0, pb_in[7:6], pb_in[3:2]};

endmodule
