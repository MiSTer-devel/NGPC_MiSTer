// Copyright (c) 2026 Jamie Blanks

// TMP95C061 serial channels SC0 and SC1. SFRs 0x50-0x58.
//
// Each channel is a baud-rate generator, a transmit shift register with its
// own holding buffer, a double-buffered receive path, and a mode/status
// register pair (TMP95C061 datasheet pp.124-139). The external link-cable
// transport is not modelled - NGPC.sv supplies the pulled-high RXD0 and CTS0_n
// levels of an unplugged CON2 - but the registers, the shift registers and the
// baud divider here are real, and looping TxD back to RxD transfers a byte.
// The software RTS latch at 0xB2 is the separate outbound RTS_n signal.
//
// The baud tree. The BIOS VECT_COMINIT writes BR0CR = 0x05, i.e. BR0CK = 00 =
// phi_T0 and BR0S = 5, and SNK specifies the link at 19200 bps 8N1. In UART
// mode the rate is tap / divisor / 16 (datasheet p.135):
//
//     phi_T0 / 5 / 16 = 19200  =>  phi_T0 = 19200 * 5 * 16 = 1 536 000 Hz
//                                         = 6.144 MHz / 4 = fc / 4
//
// which is exact. That pins phi_T0 = fc/4 = 1.536 MHz and with it the whole
// prescaler structure in ngp_t8: the four serial taps consumed here are
// phi_T0 = fc/4, phi_T2 = fc/16, phi_T8 = fc/64 and phi_T32 = fc/256
// (datasheet p.135). The pre-COMINIT idle setting BR0CR = 0x15 is then
// phi_T2 / 5 / 16 = 4800 bps, also exact. This module never assumes a rate:
// it counts the tap pulses the prescaler hands it.
//
// Divisor encoding (datasheet p.135): BRnS divides by 2 to 16, code 0 means
// 16, and code 1 is prohibited - implemented here as divide by one, which is
// what the counter does; no source says what silicon does.
//
// Read side effects: the three error flags OERR, PERR and FERR in SCnCR are
// cleared by reading the register, and reading SCnBUF clears the channel's
// INTRXn request flip-flop in the interrupt controller (datasheet p.137),
// carried out of the module on intrx_clear. Both act on the sfr_rd strobe
// only, so observing sfr_rdata disturbs nothing.

module ngp_sio
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire [6:0]  sfr_addr,
	input  wire [7:0]  sfr_wdata,
	input  wire        sfr_wr,
	input  wire        sfr_rd,
	output wire [7:0]  sfr_rdata,

	input  wire        phi_t0,       // fc/4   -- from ngp_t8's prescaler
	input  wire        phi_t2,       // fc/16
	input  wire        phi_t8,       // fc/64
	input  wire        phi_t32,      // fc/256
	input  wire        to2_trg,      // timer 2 match clock option

	input  wire        rxd0,
	input  wire        cts0_n,
	output wire        txd0,
	output wire        txd0_oe,      // open drain when ODE0 = 1
	input  wire        sclk0_in,
	output wire        sclk0_out,
	output wire        sclk0_oe,
	input  wire        rxd1,
	output wire        txd1,
	output wire        txd1_oe,
	input  wire        sclk1_in,
	output wire        sclk1_out,
	output wire        sclk1_oe,

	output wire [1:0]  intrx,
	output wire [1:0]  inttx,
	output wire [1:0]  intrx_clear,  // SCnBUF read -> clear INTRXn in ngp_intc

	input  wire [7:0]  ss_reg_addr,  // words 0x18-0x1B
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [6:0] ADDR_ODE = 7'h58;

	localparam [7:0] SS_CH0_A = 8'h18;
	localparam [7:0] SS_CH0_B = 8'h19;
	localparam [7:0] SS_CH1_A = 8'h1A;
	localparam [7:0] SS_CH1_B = 8'h1B;

	// 0x50-0x57: bit 2 picks the channel, bits 1:0 pick BUF/CR/MOD/BRCR.
	wire       sel_sc  = (sfr_addr[6:3] == 4'b1010);
	wire       sel_ode = (sfr_addr == ADDR_ODE);
	wire       ch1     = sfr_addr[2];
	wire [1:0] regsel  = sfr_addr[1:0];

	wire wr_ch0 = sfr_wr && sel_sc && !ch1;
	wire wr_ch1 = sfr_wr && sel_sc &&  ch1;
	wire rd_ch0 = sfr_rd && sel_sc && !ch1;
	wire rd_ch1 = sfr_rd && sel_sc &&  ch1;

	// ODE: only bits 1 and 0 exist (datasheet p.182). Bits 7:2 are
	// unimplemented and read 0, which keeps the reset value 0x00
	// self-consistent.
	reg [1:0] ode;

	wire ss_sel_ch0a = (ss_reg_addr == SS_CH0_A);
	wire ss_sel_ch0b = (ss_reg_addr == SS_CH0_B);
	wire ss_sel_ch1a = (ss_reg_addr == SS_CH1_A);
	wire ss_sel_ch1b = (ss_reg_addr == SS_CH1_B);

	wire [7:0] rd_buf0, rd_cr0, rd_mod0, rd_brcr0;
	wire [7:0] rd_buf1, rd_cr1, rd_mod1, rd_brcr1;
	wire [7:0] ss_txbuf0, ss_rxbuf0, ss_cnt0;
	wire [7:0] ss_txbuf1, ss_rxbuf1, ss_cnt1;
	wire [5:0] ss_sr0, ss_sr1;
	wire       ss_txfull0, ss_rxfull0, ss_txfull1, ss_rxfull1;
	wire [3:0] ss_txphase0, ss_txphase1;
	wire       ss_rxdprev0, ss_rxdprev1;
	wire       ss_sclkprev0, ss_sclkprev1;
	wire       ss_iosclk0, ss_iosclk1;
	wire       idle0, idle1;

	ngp_sio_channel #(.HAS_CTS(1'b1)) u_ch0
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.wdata(sfr_wdata),
		.wr_buf(wr_ch0 && (regsel == 2'd0)),
		.wr_cr(wr_ch0 && (regsel == 2'd1)),
		.wr_mod(wr_ch0 && (regsel == 2'd2)),
		.wr_brcr(wr_ch0 && (regsel == 2'd3)),
		.rd_buf(rd_ch0 && (regsel == 2'd0)),
		.rd_cr(rd_ch0 && (regsel == 2'd1)),
		.rd_buf_data(rd_buf0),
		.rd_cr_data(rd_cr0),
		.rd_mod_data(rd_mod0),
		.rd_brcr_data(rd_brcr0),

		.phi_t0(phi_t0),
		.phi_t2(phi_t2),
		.phi_t8(phi_t8),
		.phi_t32(phi_t32),
		.to2_trg(to2_trg),
		.ode(ode[0]),

		.rxd(rxd0),
		.cts_n(cts0_n),
		.sclk_in(sclk0_in),
		.txd(txd0),
		.txd_oe(txd0_oe),
		.sclk_out(sclk0_out),
		.sclk_oe(sclk0_oe),

		.intrx(intrx[0]),
		.inttx(inttx[0]),

		.pause_req(pause_req),
		.idle(idle0),

		.ss_tx_buf(ss_txbuf0),
		.ss_rx_buf(ss_rxbuf0),
		.ss_sr(ss_sr0),
		.ss_cnt(ss_cnt0),
		.ss_tx_full(ss_txfull0),
		.ss_rx_full(ss_rxfull0),
		.ss_tx_phase(ss_txphase0),
		.ss_rxd_prev(ss_rxdprev0),
		.ss_sclk_prev(ss_sclkprev0),
		.ss_io_sclk(ss_iosclk0),
		.ss_wren_a(ss_wren && pause_ready && ss_sel_ch0a),
		.ss_wren_b(ss_wren && pause_ready && ss_sel_ch0b),
		.ss_wdata(ss_wdata)
	);

	ngp_sio_channel #(.HAS_CTS(1'b0)) u_ch1
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.wdata(sfr_wdata),
		.wr_buf(wr_ch1 && (regsel == 2'd0)),
		.wr_cr(wr_ch1 && (regsel == 2'd1)),
		.wr_mod(wr_ch1 && (regsel == 2'd2)),
		.wr_brcr(wr_ch1 && (regsel == 2'd3)),
		.rd_buf(rd_ch1 && (regsel == 2'd0)),
		.rd_cr(rd_ch1 && (regsel == 2'd1)),
		.rd_buf_data(rd_buf1),
		.rd_cr_data(rd_cr1),
		.rd_mod_data(rd_mod1),
		.rd_brcr_data(rd_brcr1),

		.phi_t0(phi_t0),
		.phi_t2(phi_t2),
		.phi_t8(phi_t8),
		.phi_t32(phi_t32),
		.to2_trg(to2_trg),
		.ode(ode[1]),

		.rxd(rxd1),
		.cts_n(1'b0),                 // channel 1 has no CTS pin (datasheet p.182)
		.sclk_in(sclk1_in),
		.txd(txd1),
		.txd_oe(txd1_oe),
		.sclk_out(sclk1_out),
		.sclk_oe(sclk1_oe),

		.intrx(intrx[1]),
		.inttx(inttx[1]),

		.pause_req(pause_req),
		.idle(idle1),

		.ss_tx_buf(ss_txbuf1),
		.ss_rx_buf(ss_rxbuf1),
		.ss_sr(ss_sr1),
		.ss_cnt(ss_cnt1),
		.ss_tx_full(ss_txfull1),
		.ss_rx_full(ss_rxfull1),
		.ss_tx_phase(ss_txphase1),
		.ss_rxd_prev(ss_rxdprev1),
		.ss_sclk_prev(ss_sclkprev1),
		.ss_io_sclk(ss_iosclk1),
		.ss_wren_a(ss_wren && pause_ready && ss_sel_ch1a),
		.ss_wren_b(ss_wren && pause_ready && ss_sel_ch1b),
		.ss_wdata(ss_wdata)
	);

	always @(posedge clk) begin
		if (reset) begin
			ode <= 2'b00;
		end else begin
			if (sfr_wr && sel_ode) begin
				ode <= sfr_wdata[1:0];
			end
			// ODE lives in channel 0's second tap word.
			if (ss_wren && pause_ready && ss_sel_ch0b) begin
				ode <= ss_wdata[17:16];
			end
		end
	end

	reg [7:0] sfr_rdata_r;
	always_comb begin
		if (sel_ode) begin
			sfr_rdata_r = {6'b000000, ode};
		end else if (sel_sc) begin
			case (regsel)
				2'd0:    sfr_rdata_r = ch1 ? rd_buf1  : rd_buf0;
				2'd1:    sfr_rdata_r = ch1 ? rd_cr1   : rd_cr0;
				2'd2:    sfr_rdata_r = ch1 ? rd_mod1  : rd_mod0;
				default: sfr_rdata_r = ch1 ? rd_brcr1 : rd_brcr0;
			endcase
		end else begin
			sfr_rdata_r = 8'hFF;
		end
	end

	assign sfr_rdata = sfr_rdata_r;

	// Reading SCnBUF clears the channel's INTRXn request flip-flop
	// (datasheet p.137). The buffer-empty half of the same side effect is
	// inside the channel; this is the half that has to leave the module.
	assign intrx_clear = { rd_ch1 && (regsel == 2'd0),
	                       rd_ch0 && (regsel == 2'd0) };

	// A byte in a shift register must finish before the enables stop.
	assign pause_ready = pause_req && idle0 && idle1;

	// Four tap words. The first word of each channel holds the buffers and
	// SCnCR/SCnMOD; the second holds BRnCR, buffer fullness, ODE, the shifter
	// low bits, the bit count and the BRG divider, plus RXD history, UART
	// phase, SCLK history and generated-clock polarity so the first edge after
	// a restore is exact.
	reg [31:0] ss_rdata_r;
	always_comb begin
		ss_rdata_r = 32'd0;
		if (ss_sel_ch0a) begin
			ss_rdata_r = {ss_txbuf0, ss_rxbuf0, rd_cr0, rd_mod0};
		end else if (ss_sel_ch0b) begin
			ss_rdata_r = {ss_rxdprev0, rd_brcr0[6:0], ss_txphase0,
			              ss_rxfull0, ss_txfull0, ode, ss_sclkprev0,
			              ss_iosclk0, ss_sr0, ss_cnt0};
		end else if (ss_sel_ch1a) begin
			ss_rdata_r = {ss_txbuf1, ss_rxbuf1, rd_cr1, rd_mod1};
		end else if (ss_sel_ch1b) begin
			// The two ODE positions remain zero for channel 1.
			ss_rdata_r = {ss_rxdprev1, rd_brcr1[6:0], ss_txphase1,
			              ss_rxfull1, ss_txfull1, 2'b00, ss_sclkprev1,
			              ss_iosclk1, ss_sr1, ss_cnt1};
		end
	end

	assign ss_rdata = ss_rdata_r;

endmodule


// One serial channel. HAS_CTS is 0 for channel 1, which has no CTS pin, so
// its SC1MOD<CTSE> reads back 0 (datasheet p.182).
module ngp_sio_channel
#(
	parameter [0:0] HAS_CTS = 1'b1
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire [7:0]  wdata,
	input  wire        wr_buf,
	input  wire        wr_cr,
	input  wire        wr_mod,
	input  wire        wr_brcr,
	input  wire        rd_buf,
	input  wire        rd_cr,
	output wire [7:0]  rd_buf_data,
	output wire [7:0]  rd_cr_data,
	output wire [7:0]  rd_mod_data,
	output wire [7:0]  rd_brcr_data,

	input  wire        phi_t0,
	input  wire        phi_t2,
	input  wire        phi_t8,
	input  wire        phi_t32,
	input  wire        to2_trg,
	input  wire        ode,

	input  wire        rxd,
	input  wire        cts_n,
	input  wire        sclk_in,
	output wire        txd,
	output wire        txd_oe,
	output wire        sclk_out,
	output wire        sclk_oe,

	output wire        intrx,
	output wire        inttx,

	input  wire        pause_req,
	output wire        idle,

	output wire [7:0]  ss_tx_buf,
	output wire [7:0]  ss_rx_buf,
	output wire [5:0]  ss_sr,
	output wire [7:0]  ss_cnt,
	output wire        ss_tx_full,
	output wire        ss_rx_full,
	output wire [3:0]  ss_tx_phase,
	output wire        ss_rxd_prev,
	output wire        ss_sclk_prev,
	output wire        ss_io_sclk,
	input  wire        ss_wren_a,
	input  wire        ss_wren_b,
	input  wire [31:0] ss_wdata
);

	localparam [1:0] SM_IO   = 2'b00;   // I/O interface (synchronous)
	localparam [1:0] SM_7BIT = 2'b01;
	localparam [1:0] SM_8BIT = 2'b10;
	localparam [1:0] SM_9BIT = 2'b11;

	localparam [1:0] SC_TO2  = 2'b00;   // timer 2 match
	localparam [1:0] SC_BRG  = 2'b01;   // baud rate generator
	localparam [1:0] SC_PHI1 = 2'b10;   // internal clock
	localparam [1:0] SC_EXT  = 2'b11;   // external SCLK

	localparam [1:0] TX_IDLE  = 2'd0;
	localparam [1:0] TX_START = 2'd1;
	localparam [1:0] TX_DATA  = 2'd2;
	localparam [1:0] TX_STOP  = 2'd3;

	localparam [1:0] RX_IDLE  = 2'd0;
	localparam [1:0] RX_START = 2'd1;
	localparam [1:0] RX_DATA  = 2'd2;
	localparam [1:0] RX_STOP  = 2'd3;

	// SCnCR:  [7] RB8 (R), [6] EVEN, [5] PE, [4] OERR, [3] PERR, [2] FERR,
	//         [1] SCLKS, [0] IOC
	// SCnMOD: [7] TB8, [6] CTSE, [5] RXE, [4] WU, [3:2] SM, [1:0] SC
	// BRnCR:  [7] fixed 0, [5:4] BRnCK, [3:0] BRnS
	reg [7:0] sccr;
	reg [7:0] scmod;
	reg [6:0] brcr;   // bit 7 is fixed 0 and is not stored

	reg [7:0] tx_buf;
	reg       tx_buf_full;
	reg [8:0] tx_sr;
	reg [3:0] tx_bit;
	reg [1:0] tx_state;
	reg [3:0] tx_phase;
	reg       txd_r;

	reg [7:0] rx_buf;
	reg       rx_buf_full;
	reg [8:0] rx_sr;
	reg [3:0] rx_bit;
	reg [1:0] rx_state;
	reg [3:0] rx_phase;
	reg [1:0] rx_ones;
	reg       rxd_prev;

	reg [3:0] brg_div;
	reg       sclk_prev;
	reg       io_sclk;

	reg       intrx_r;
	reg       inttx_r;

	wire [1:0] sm   = scmod[3:2];
	wire [1:0] sc   = scmod[1:0];
	wire       rxe  = scmod[5];
	wire       wu   = scmod[4];
	wire       ctse = scmod[6];
	wire       pe   = sccr[5];
	wire       even = sccr[6];
	wire       ioc  = sccr[0];
	wire       sclks = sccr[1];

	wire uart_mode = (sm != SM_IO);

	// Body bits per frame: data bits plus the parity bit when one is added.
	// Parity is only available in 7-bit and 8-bit UART mode (datasheet p.139).
	wire parity_on = pe && ((sm == SM_7BIT) || (sm == SM_8BIT));

	reg [3:0] data_len;
	always_comb begin
		case (sm)
			SM_7BIT: data_len = 4'd7;
			SM_9BIT: data_len = 4'd9;
			default: data_len = 4'd8;   // 8-bit UART and I/O interface
		endcase
	end

	wire [3:0] frame_len = data_len + (parity_on ? 4'd1 : 4'd0);

	// Baud rate generator
	wire [1:0] brck = brcr[5:4];

	reg brg_src;
	always_comb begin
		case (brck)
			2'b00:   brg_src = phi_t0;
			2'b01:   brg_src = phi_t2;
			2'b10:   brg_src = phi_t8;
			default: brg_src = phi_t32;
		endcase
	end

	// BRnS = 0 means divide by 16, so the counter wraps at 15.
	wire [3:0] brg_top = (brcr[3:0] == 4'd0) ? 4'd15 : (brcr[3:0] - 4'd1);
	wire       brg_out = brg_src && (brg_div == brg_top);

	// External SCLK edge, per SCnCR<SCLKS> (datasheet p.136).
	wire sclk_edge = sclks ? (sclk_prev && !sclk_in) : (!sclk_prev && sclk_in);

	// SIOCLK, the 16x oversampling clock of the UART receiver and transmitter
	// (datasheet p.136). SC = 10 selects the internal clock; no source gives
	// its exact rate, so one machine state is used.
	reg sioclk;
	always_comb begin
		case (sc)
			SC_TO2:  sioclk = to2_trg;
			SC_BRG:  sioclk = brg_out;
			SC_PHI1: sioclk = ce;
			SC_EXT:  sioclk = sclk_edge;
			default: sioclk = sclk_edge;
		endcase
	end

	// I/O interface mode: SCLK output is the baud generator divided by two,
	// SCLK input is the external edge (datasheet p.136).
	wire io_shift = ioc ? sclk_edge : (brg_out && !io_sclk);

	wire bit_tick = uart_mode ? (sioclk && (tx_phase == 4'd15)) : io_shift;

	// Transmitter
	// Parity is generated from the data written to the buffer and rides in
	// the bit above the data: SCnBUF<TB7> in 7-bit mode, SCnMOD<TB8> in
	// 8-bit mode (datasheet p.139).
	wire tx_par_even = ^tx_buf[6:0];
	wire tx_par_all  = ^tx_buf;
	wire tx_par      = (sm == SM_7BIT) ? (even ? tx_par_even : ~tx_par_even)
	                                   : (even ? tx_par_all  : ~tx_par_all);

	wire [6:0] tx_data7 = tx_buf[6:0];
	wire       tb8      = scmod[7];

	reg [8:0] tx_frame;
	always_comb begin
		case (sm)
			SM_7BIT: tx_frame = {1'b0, parity_on ? tx_par : 1'b1, tx_data7};
			SM_8BIT: tx_frame = {parity_on ? tx_par : tb8, tx_buf};
			SM_9BIT: tx_frame = {tb8, tx_buf};
			default: tx_frame = {1'b0, tx_buf};
		endcase
	end

	// CTS holds off the START of the next frame only; the interrupt that
	// asks for the next byte still fires (datasheet p.138).
	wire cts_ok     = (HAS_CTS == 1'b0) || !ctse || !cts_n;
	wire tx_load_ok = tx_buf_full && cts_ok && !pause_req;

	// Receiver
	// Each bit is sampled at SIOCLK counts 7, 8 and 9 and decided by
	// majority (datasheet p.136). rx_ones accumulates counts 7 and 8; count 9's
	// sample joins them here.
	wire rx_vote = rx_ones[1] || (rx_ones[0] && rxd);

	wire rx_sample = (rx_phase == 4'd7) || (rx_phase == 4'd8);
	wire rx_decide = (rx_phase == 4'd9);
	wire rx_endbit = (rx_phase == 4'd15);
	wire rx_lastbody = (rx_bit == (frame_len - 4'd1));

	// The receive interrupt and the buffer transfer happen at the centre of
	// the last body bit in 9-bit and 8-bit-plus-parity modes, and at the
	// centre of the stop bit otherwise (datasheet p.140 table).
	wire rx_early = (sm == SM_9BIT) || ((sm == SM_8BIT) && parity_on);

	wire rx_shift_uart = uart_mode && sioclk && (rx_state == RX_DATA) && rx_decide;
	wire rx_shift_io   = !uart_mode && io_shift && rxe && !pause_req;
	wire rx_shift_bit  = uart_mode ? rx_vote : rxd;
	wire [8:0] rx_sr_next = {rx_shift_bit, rx_sr[8:1]};
	wire [8:0] rx_frame_src = (rx_shift_uart || rx_shift_io) ? rx_sr_next : rx_sr;

	// Body bit 0 lands at position 9 - frame_len after frame_len shifts.
	wire [8:0] rx_src_sh1 = {1'b0,  rx_frame_src[8:1]};
	wire [8:0] rx_src_sh2 = {2'b00, rx_frame_src[8:2]};

	reg [8:0] rx_aligned;
	always_comb begin
		case (frame_len)
			4'd7:    rx_aligned = rx_src_sh2;
			4'd8:    rx_aligned = rx_src_sh1;
			default: rx_aligned = rx_frame_src;
		endcase
	end

	wire [7:0] rx_al_low8 = rx_aligned[7:0];
	wire [6:0] rx_al_low7 = rx_aligned[6:0];
	wire       rx_al_b8   = rx_aligned[8];
	wire       rb8_hold   = sccr[7];

	wire       rx_par_recv = (sm == SM_7BIT) ? rx_aligned[7] : rx_aligned[8];
	wire       rx_par_calc = (sm == SM_7BIT) ? (even ? ^rx_aligned[6:0] : ~^rx_aligned[6:0])
	                                         : (even ? ^rx_aligned[7:0] : ~^rx_aligned[7:0]);
	wire       rx_perr     = parity_on && (rx_par_recv != rx_par_calc);

	reg [7:0] rx_data;
	always_comb begin
		if ((sm == SM_7BIT) && !parity_on) begin
			rx_data = {1'b0, rx_al_low7};
		end else begin
			rx_data = rx_al_low8;
		end
	end

	reg rx_rb8;
	always_comb begin
		case (sm)
			SM_9BIT: rx_rb8 = rx_al_b8;
			SM_8BIT: rx_rb8 = parity_on ? rx_al_b8 : rb8_hold;
			default: rx_rb8 = rb8_hold;
		endcase
	end

	// Wake-up: in 9-bit mode with WU set, INTRX fires only for RB8 = 1
	// (datasheet p.137).
	wire rx_wu_block = (sm == SM_9BIT) && wu && !rx_rb8;

	wire rx_commit_uart = uart_mode && sioclk && rx_decide &&
	                      ((rx_early  && (rx_state == RX_DATA) && rx_lastbody) ||
	                       (!rx_early && (rx_state == RX_STOP)));
	wire rx_commit_io   = rx_shift_io && (rx_bit == 4'd7);
	wire rx_commit      = rx_commit_uart || rx_commit_io;

	// The receive register has hardware-set status and read-to-clear/free
	// side effects. Resolve both from the pre-edge state so a receive event
	// on the same edge as a CPU read is never discarded. A buffer read frees
	// the old byte before the new commit is accepted; without that read, a
	// commit into a full buffer is an overrun and preserves the old byte.
	wire       rx_ferr_set = uart_mode && sioclk && rx_decide &&
	                         (rx_state == RX_STOP) && !rx_vote;
	wire       rx_accept = rx_commit && (!rx_buf_full || rd_buf);
	wire       rx_overrun_set = rx_commit && rx_buf_full && !rd_buf;
	wire [2:0] rx_err_set = {rx_overrun_set, rx_accept && rx_perr, rx_ferr_set};
	wire [2:0] rx_err_keep = rd_cr ? 3'b000 : sccr[4:2];
	wire [2:0] sccr_err_next = rx_err_keep | rx_err_set;
	wire       rx_buf_full_next = rx_commit ? 1'b1 :
	                              (rd_buf ? 1'b0 : rx_buf_full);
	wire       channel_idle = (tx_state == TX_IDLE) &&
	                          (rx_state == RX_IDLE) && (rx_bit == 4'd0);
	wire       pause_hold = pause_req && channel_idle;

	// Sequential
	always @(posedge clk) begin
		intrx_r <= 1'b0;
		inttx_r <= 1'b0;

		if (reset) begin
			// Datasheet p.182: SCnCR = 0x00 with bit 7 undefined, SCnMOD = 0x00
			// with bit 7 undefined, BRnCR = 0x00.
			sccr        <= 8'h00;
			scmod       <= 8'h00;
			brcr        <= 7'h00;
			tx_buf      <= 8'h00;
			tx_buf_full <= 1'b0;
			tx_sr       <= 9'h000;
			tx_bit      <= 4'd0;
			tx_state    <= TX_IDLE;
			tx_phase    <= 4'd0;
			txd_r       <= 1'b1;
			rx_buf      <= 8'h00;
			rx_buf_full <= 1'b0;
			rx_sr       <= 9'h000;
			rx_bit      <= 4'd0;
			rx_state    <= RX_IDLE;
			rx_phase    <= 4'd0;
			rx_ones     <= 2'd0;
			rxd_prev    <= 1'b1;
			brg_div     <= 4'd0;
			sclk_prev   <= 1'b0;
			io_sclk     <= 1'b0;
		end else begin
			if (!pause_hold) begin
				sclk_prev <= sclk_in;
			end
			sccr[4:2] <= sccr_err_next;
			rx_buf_full <= rx_buf_full_next;

			if (brg_src && !pause_hold) begin
				brg_div <= (brg_div == brg_top) ? 4'd0 : (brg_div + 4'd1);
			end

			if (!uart_mode && !ioc && brg_out && !pause_hold) begin
				io_sclk <= ~io_sclk;
			end

			// Transmit
			if (uart_mode && sioclk && !pause_hold) begin
				tx_phase <= tx_phase + 4'd1;
			end

			if (bit_tick) begin
				case (tx_state)
					TX_START: begin
						txd_r    <= tx_sr[0];
						tx_sr    <= {1'b0, tx_sr[8:1]};
						tx_bit   <= 4'd1;
						tx_state <= TX_DATA;
					end
					TX_DATA: begin
						if (tx_bit == frame_len) begin
							// Every body bit is out: the shift register is
							// empty, which is where INTTX fires -- "just
							// before last bit is transmitted" (datasheet p.139).
							txd_r    <= 1'b1;
							inttx_r  <= 1'b1;
							tx_state <= uart_mode ? TX_STOP : TX_IDLE;
						end else begin
							txd_r  <= tx_sr[0];
							tx_sr  <= {1'b0, tx_sr[8:1]};
							tx_bit <= tx_bit + 4'd1;
						end
					end
					default: begin
						// TX_IDLE and the end of the stop bit behave the
						// same, so back-to-back frames have no extra gap.
						if (tx_load_ok) begin
							tx_buf_full <= 1'b0;
							if (uart_mode) begin
								tx_sr    <= tx_frame;
								tx_bit   <= 4'd0;
								txd_r    <= 1'b0;          // start bit
								tx_state <= TX_START;
							end else begin
								tx_sr    <= {1'b0, tx_frame[8:1]};
								tx_bit   <= 4'd1;
								txd_r    <= tx_frame[0];
								tx_state <= TX_DATA;
							end
							// The generated parity is stored back into TB8
							// in 8-bit mode (datasheet p.139).
							if (parity_on && (sm == SM_8BIT)) begin
								scmod[7] <= tx_par;
							end
						end else begin
							txd_r    <= 1'b1;
							tx_state <= TX_IDLE;
						end
					end
				endcase
			end

			// Receive
			if (uart_mode && sioclk && !pause_hold) begin
				rxd_prev <= rxd;

				if (rx_state == RX_IDLE) begin
					// A falling edge on an idle line opens a frame; this
					// SIOCLK is count 0 of the start bit.
					if (rxe && !pause_req && rxd_prev && !rxd) begin
						rx_phase <= 4'd1;
						rx_ones  <= 2'd0;
						rx_bit   <= 4'd0;
						rx_state <= RX_START;
					end
				end else begin
					rx_phase <= rx_phase + 4'd1;

					if (rx_sample) begin
						rx_ones <= rx_ones + {1'b0, rxd};
					end

					if (rx_decide) begin
						rx_ones <= 2'd0;

						case (rx_state)
							RX_START: begin
								// Two or more ones means the start bit was
								// noise (datasheet p.136).
								if (rx_vote) begin
									rx_state <= RX_IDLE;
									rx_phase <= 4'd0;
								end
							end
							RX_DATA: begin
								rx_sr <= rx_sr_next;
							end
							default: begin
								// Stop bit: a majority of zero is a framing
								// error (datasheet p.139).
								rx_state <= RX_IDLE;
								rx_phase <= 4'd0;
								rx_bit   <= 4'd0;
							end
						endcase
					end

					if (rx_endbit) begin
						if (rx_state == RX_START) begin
							rx_state <= RX_DATA;
							rx_bit   <= 4'd0;
						end else if (rx_state == RX_DATA) begin
							if (rx_lastbody) begin
								rx_state <= RX_STOP;
							end else begin
								rx_bit <= rx_bit + 4'd1;
							end
						end
					end
				end
			end

			if (rx_shift_io) begin
				rx_sr  <= rx_sr_next;
				rx_bit <= (rx_bit == 4'd7) ? 4'd0 : (rx_bit + 4'd1);
			end

			// One transfer site for both modes.
			// On overrun, the next-state logic sets OERR and leaves buffer 2
			// and RB8 untouched (datasheet p.139).
			if (rx_accept) begin
				rx_buf  <= rx_data;
				sccr[7] <= rx_rb8;
				if (!rx_wu_block) begin
					intrx_r <= 1'b1;
				end
			end

			// SFR access. Writes and read side effects are strobe-enabled,
			// never ce-gated. Receive-status and buffer collisions are
			// resolved by the explicit next-state logic above, with hardware
			// receive events taking priority over reads.
			if (wr_buf) begin
				tx_buf      <= wdata;
				tx_buf_full <= 1'b1;
			end

			if (wr_cr) begin
				// EVEN, PE, SCLKS and IOC only: RB8 is read-only and the
				// three error flags are set by hardware and cleared by
				// reading (datasheet p.182).
				sccr[6:5] <= wdata[6:5];
				sccr[1:0] <= wdata[1:0];
			end

			if (wr_mod) begin
				scmod <= wdata;
			end

			if (wr_brcr) begin
				brcr <= wdata[6:0];
			end

			// Savestate writes land last and are already qualified by
			// pause_ready at the parent.
			if (ss_wren_a) begin
				tx_buf <= ss_wdata[31:24];
				rx_buf <= ss_wdata[23:16];
				sccr   <= ss_wdata[15:8];
				scmod  <= ss_wdata[7:0];
			end

			if (ss_wren_b) begin
				brcr        <= ss_wdata[30:24];
				rxd_prev    <= ss_wdata[31];
				tx_phase    <= ss_wdata[23:20];
				rx_buf_full <= ss_wdata[19];
				tx_buf_full <= ss_wdata[18];
				sclk_prev   <= ss_wdata[15];
				io_sclk     <= ss_wdata[14];
				tx_sr       <= {3'b000, ss_wdata[13:8]};
				tx_bit      <= ss_wdata[7:4];
				brg_div     <= ss_wdata[3:0];
			end
		end
	end

	// Outputs
	// SCnBUF reads receive buffer 2; the transmit half of the address is
	// write-only and its shadow appears on the savestate tap only.
	assign rd_buf_data  = rx_buf;
	assign rd_cr_data   = sccr;
	assign rd_mod_data  = (HAS_CTS == 1'b1) ? scmod : {scmod[7], 1'b0, scmod[5:0]};
	assign rd_brcr_data = {1'b0, brcr};

	assign txd    = txd_r;
	assign txd_oe = ode ? ~txd_r : 1'b1;   // open drain drives the 0 only

	assign sclk_out = io_sclk;
	assign sclk_oe  = (sm == SM_IO) && !ioc;

	assign intrx = intrx_r;
	assign inttx = inttx_r;

	assign idle = channel_idle;

	assign ss_tx_buf  = tx_buf;
	assign ss_rx_buf  = rx_buf;
	assign ss_sr      = tx_sr[5:0];
	assign ss_cnt     = {tx_bit, brg_div};
	assign ss_tx_full = tx_buf_full;
	assign ss_rx_full = rx_buf_full;
	assign ss_tx_phase = tx_phase;
	assign ss_rxd_prev = rxd_prev;
	assign ss_sclk_prev = sclk_prev;
	assign ss_io_sclk = io_sclk;

	// Bits of the tap words this channel does not own: the parent restores
	// the ODE half of word 0x19 and word 0x1B's filler byte.
	wire unused_ok = &{1'b0, ss_wdata[17:16]};

endmodule
