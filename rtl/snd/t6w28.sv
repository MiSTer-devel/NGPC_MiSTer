// Copyright (c) 2026 Jamie Blanks

// T6W28 programmable sound generator, the PSG inside the SNK K2 chip.
//
// Not two SN76489s: one set of three tone generators plus one noise generator,
// with two banks of four attenuators, one bank per output channel.  The two
// write ports address different parts of that single machine.  No primary
// T6W28 document states this; every known emulator lineage does it this way.
//
//   port 0 (a0 = 0, Z80 0x4000, main CPU 0xA0)
//       the four right attenuators, the noise control register, and a private
//       tone-2 register whose only job is to set the mode-3 noise shift rate
//   port 1 (a0 = 1, Z80 0x4001, main CPU 0xA1)
//       the four left attenuators and the three tone frequency registers that
//       actually drive the generators
//
// Each port has its own first-byte latch.  Tone pitch is global and written
// through port 1; noise pitch and mode are global and written through port 0;
// only the volumes are per side, and noise attenuation is symmetric (port 1's
// noise attenuator feeds left, port 0's feeds right).  A mono driver writes
// the same byte to both ports.
//
// Everything else about the generators is the TI part, per the SN76489
// datasheet.  That datasheet calls D0 the most significant bit and numbers the
// ten period bits F0..F9 with F0 most significant (p.5 pin table, p.2 s.1);
// every bit range below is already translated into conventional bit-7-is-MSB
// terms, so only this comment has to hold the inversion.
//
// out_l/out_r carry the full 14-bit sum (four channels x a 12-bit volume ROM
// = 0..16380); the mixer scales.  ss_addr is 4 bits (ngp_snd taps 0x10-0x15
// arrive rebased to 0x0-0x5).
//
// Three model choices are parameters rather than a silent default: the LFSR_*
// group (15-bit model by default, versus a 16-bit one -- in periodic mode a
// single set bit circulates, so the two differ by a 16/15 pitch ratio, about
// 1.1 semitones), TONE_ZERO (what a period register of 0 means), and
// NOISE3_X2 (mode-3 noise interval).  Each is described at its declaration.
//
// One clock enable, ce = ce_3m072, which is the chip's CLOCK pin.  A 4-bit
// scaler qualified by ce produces the tick at 3.072 MHz / 16 = 192 kHz exactly
// (SN76489 datasheet p.2 s.1 "a 10 stage tone counter, which is decremented at
// a N/16 rate", p.8 block diagram), and every audible state change happens on
// a tick.  The scaler free-runs from reset and is never restarted by a write,
// which gives writes the sub-tick jitter the real part has, so freezing the
// chip for a savestate is just gating ce.

module t6w28
#(
	// --- noise LFSR model ---------------------------------------------
	parameter integer LFSR_WIDTH  = 15,
	parameter [15:0]  LFSR_RESET  = 16'h4000,
	parameter [15:0]  LFSR_TAPS   = 16'h0003,  // bits XORed into the new MSB
	parameter integer LFSR_INVERT = 0,         // invert the white-noise output
	// --- tone period register = 0 -------------------------------------
	parameter [1:0]   TONE_ZERO   = 2'd1,      // 0 = 1024, 1 = one, 2 = freeze
	// --- mode-3 noise interval ----------------------------------------
	parameter integer NOISE3_X2   = 1,         // 1 = 2n ticks, 0 = n ticks
	// --- which bit of the shift register is the noise OUTPUT ----------
	// 1 = output the post-shift bit (default); 0 = the bit leaving the
	// register, one shift interval earlier.  Inaudible either way -- same
	// sequence, offset one interval.
	parameter integer NOISE_OUT_POST = 1
)
(
	input  wire        clk,
	input  wire        ce,          // ce_3m072, the chip's CLOCK pin
	input  wire        rst_n,

	// bus pins (the SN76489 pin set: /CE, /WE, D0-D7, READY)
	input  wire        ce_n,
	input  wire        we_n,
	input  wire        a0,          // 0 = port 0 (0x4000), 1 = port 1 (0x4001)
	input  wire [7:0]  din,
	output wire        ready,       // open-drain model: low = busy

	input  wire        enable,      // 0xB8 sound enable; 0 mutes the outputs

	output wire [13:0] out_l,       // unsigned sum of 4 attenuated channels
	output wire [13:0] out_r,

	// savestate tap (ngp_snd taps 0x10-0x15 arrive here as 0x0-0x5)
	input  wire [3:0]  ss_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata
);

	localparam [3:0] SS_PERIOD = 4'h0;   // tone period registers
	localparam [3:0] SS_COUNT  = 4'h1;   // tone down counters
	localparam [3:0] SS_GEN    = 4'h2;   // LFSR, noise control, scaler, flops
	localparam [3:0] SS_NOISE  = 4'h3;   // noise counter and rate register
	localparam [3:0] SS_ATT    = 4'h4;   // both attenuator banks
	localparam [3:0] SS_MISC   = 4'h5;   // port latches, READY, enable shadow

	// The one shifted bit position, precomputed so no variable part select
	// or shift by a parameter appears in the datapath.
	localparam [15:0] LFSR_MSB = 16'd1 << (LFSR_WIDTH - 1);

	// State

	reg  [9:0]  tone_period [0:2];   // written through port 1 only
	reg  [9:0]  tone_cnt    [0:2];
	reg  [2:0]  tone_ff;

	reg  [9:0]  noise_period;        // port 0's private tone-2 register
	reg  [11:0] noise_cnt;
	reg  [15:0] lfsr;
	reg         noise_out;
	reg         noise_mode;          // 0 = periodic, 1 = white  (FB)
	reg  [1:0]  noise_rate;          // NF1 NF0

	reg  [3:0]  att_l [0:3];         // 0..2 = tones, 3 = noise
	reg  [3:0]  att_r [0:3];

	reg  [2:0]  latch0;              // each port has its own first-byte latch
	reg  [2:0]  latch1;

	reg  [3:0]  div16;               // the divide-by-16 clock scaler
	reg  [5:0]  ready_cnt;
	reg         stb_q;               // /CE + /WE history, for edge detection

	integer i;

	// Bus interface (SN76489 datasheet p.4 s.7, p.5 Table 5)
	// "When CE is true, the WE signal strobes the contents of the data bus to
	// the appropriate control register."  One write per assertion: the strobe
	// is edge detected in the ce domain so a host that holds /CE and /WE low
	// across several chip clocks still writes once.

	wire stb     = !ce_n && !we_n;
	wire wr_stb  = ce && stb && !stb_q;

	always @(posedge clk) begin
		if (!rst_n) begin
			stb_q <= 1'b0;
		end else if (ss_wren && (ss_addr == SS_MISC)) begin
			stb_q <= ss_wdata[13];
		end else if (ce) begin
			stb_q <= stb;
		end
	end

	// READY is an open-collector pin pulled low for the roughly 32 clocks the
	// part needs to load a control register (SN76489 datasheet p.4 s.7, p.5
	// pin table).  READY is modelled because the pin is real, but /WAIT is
	// tied high: a true 32-clock stall would wreck every driver's timing
	// budget.
	always @(posedge clk) begin
		if (!rst_n) begin
			ready_cnt <= 6'd0;
		end else if (ss_wren && (ss_addr == SS_MISC)) begin
			ready_cnt <= ss_wdata[11:6];
		end else if (wr_stb) begin
			ready_cnt <= 6'd32;
		end else if (ce && (ready_cnt != 6'd0)) begin
			ready_cnt <= ready_cnt - 6'd1;
		end
	end

	assign ready = (ready_cnt == 6'd0);

	// Write decode (SN76489 datasheet p.3 Table 4, p.4 s.6)
	// Latch byte (din[7] = 1):  din[6:4] = register address, din[3:0] = the
	//                           four LEAST significant period bits (F6..F9)
	// Data byte  (din[7] = 0):  din[6] don't care, din[5:0] = the six MOST
	//                           significant period bits (F0..F5), applied to
	//                           whichever register that port last latched
	//
	// din[6:4] decomposes as {channel[1:0], is_attenuator}:
	//   000 tone 1 freq   001 tone 1 att   010 tone 2 freq   011 tone 2 att
	//   100 tone 3 freq   101 tone 3 att   110 noise control 111 noise att
	//
	// A data byte arriving while a non-frequency register is latched re-applies
	// din[3:0] exactly as a latch byte would, including the LFSR reset for the
	// noise control register.  The datasheet defines the data byte only for
	// tone registers; this behaviour is inferred.

	wire       is_latch = din[7];
	wire [2:0] reg_sel  = is_latch ? din[6:4] : (a0 ? latch1 : latch0);
	wire [1:0] chan     = reg_sel[2:1];
	wire       is_att   = reg_sel[0];

	// The complete effect table (P = port, R = latched register):
	//   P0 R=000,010        nothing.  Port 0 accepts the write and drops it
	//   P0 R=001,011,101    right attenuator for tone 0/1/2 <- din[3:0]
	//   P0 R=100            private noise-rate register, 10 bits
	//   P0 R=110            noise control: mode, rate, LFSR <- LFSR_RESET
	//   P0 R=111            right noise attenuator <- din[3:0]
	//   P1 R=000,010,100    tone 0/1/2 period, 10 bits
	//   P1 R=001,011,101    left attenuator for tone 0/1/2 <- din[3:0]
	//   P1 R=110            nothing
	//   P1 R=111            left noise attenuator <- din[3:0]
	// The two "nothing" rows are not implemented as registers on purpose: the
	// chip is write-only so they are not observable anywhere, and giving them
	// storage would only add savestate surface.
	wire wr_att_l   = wr_stb &&  a0 && is_att;
	wire wr_att_r   = wr_stb && !a0 && is_att;
	wire wr_noise_c = wr_stb && !a0 && (reg_sel == 3'b110);
	wire wr_tone    = wr_stb &&  a0 && !is_att && (reg_sel != 3'b110);
	wire wr_nperiod = wr_stb && !a0 && (reg_sel == 3'b100);

	// Register file
	// Reset state.  The datasheet gives no power-on state; all eight
	// attenuators OFF is the only silent power-on state.  Tone periods 0,
	// counters 1, flip-flops 0, latches 0, noise periodic at rate 00, LFSR at
	// LFSR_RESET, scaler 0.

	always @(posedge clk) begin
		if (!rst_n) begin
			for (i = 0; i < 3; i = i + 1) begin
				tone_period[i] <= 10'd0;
			end
			for (i = 0; i < 4; i = i + 1) begin
				att_l[i] <= 4'd15;
				att_r[i] <= 4'd15;
			end
			noise_period <= 10'd0;
			noise_mode   <= 1'b0;
			noise_rate   <= 2'd0;
			latch0       <= 3'd0;
			latch1       <= 3'd0;
		end else if (ss_wren && (ss_addr == SS_PERIOD)) begin
			tone_period[0] <= ss_wdata[9:0];
			tone_period[1] <= ss_wdata[19:10];
			tone_period[2] <= ss_wdata[29:20];
		end else if (ss_wren && (ss_addr == SS_GEN)) begin
			noise_mode <= ss_wdata[18];
			noise_rate <= ss_wdata[17:16];
		end else if (ss_wren && (ss_addr == SS_NOISE)) begin
			noise_period <= ss_wdata[21:12];
		end else if (ss_wren && (ss_addr == SS_ATT)) begin
			for (i = 0; i < 4; i = i + 1) begin
				att_r[i] <= ss_wdata[i*4 +: 4];
				att_l[i] <= ss_wdata[16 + i*4 +: 4];
			end
		end else if (ss_wren && (ss_addr == SS_MISC)) begin
			latch0 <= ss_wdata[2:0];
			latch1 <= ss_wdata[5:3];
		end else begin
			// The register address is held on the chip, so repeated data bytes
			// keep landing in the same register - the mechanism the datasheet
			// calls out for fast frequency sweeps (SN76489 datasheet p.3 s.4).
			if (wr_stb && is_latch) begin
				if (a0) begin
					latch1 <= din[6:4];
				end else begin
					latch0 <= din[6:4];
				end
			end

			if (wr_att_l) begin
				att_l[chan] <= din[3:0];
			end
			if (wr_att_r) begin
				att_r[chan] <= din[3:0];
			end
			if (wr_noise_c) begin
				noise_mode <= din[2];
				noise_rate <= din[1:0];
			end
			if (wr_tone) begin
				if (is_latch) begin
					tone_period[chan][3:0] <= din[3:0];   // F6..F9
				end else begin
					tone_period[chan][9:4] <= din[5:0];   // F0..F5
				end
			end
			if (wr_nperiod) begin
				if (is_latch) begin
					noise_period[3:0] <= din[3:0];
				end else begin
					noise_period[9:4] <= din[5:0];
				end
			end
		end
	end

	// Divide-by-16 clock scaler (SN76489 datasheet p.2 s.1, p.8 block diagram)

	wire tick = ce && (div16 == 4'd15);

	always @(posedge clk) begin
		if (!rst_n) begin
			div16 <= 4'd0;
		end else if (ss_wren && (ss_addr == SS_GEN)) begin
			div16 <= ss_wdata[22:19];
		end else if (ce) begin
			div16 <= div16 + 4'd1;
		end
	end

	// Tone generators (SN76489 datasheet p.2 s.1)
	// Each generator holds a 10-bit period register n and a 10-bit down
	// counter decremented once per tick.  Reaching zero produces a borrow that
	// toggles the frequency flip-flop and reloads the counter, so the half
	// period is exactly n ticks and f = clock / (32n).  At 3.072 MHz that is
	// 96 kHz for n = 1 down to 93.9 Hz for n = 1023.
	//
	// The counter is written so that the "0 means 1024" model needs no special
	// case at all: loading 0 into a 10-bit counter and letting it wrap gives
	// 0, 1023, ... 2, reload - exactly 1024 ticks.  Only TONE_ZERO = 1 needs
	// the substitution below and only TONE_ZERO = 2 needs the hold.

	function automatic [9:0] period_eff(input [9:0] p);
	begin
		if ((TONE_ZERO == 2'd1) && (p == 10'd0)) begin
			period_eff = 10'd1;
		end else begin
			period_eff = p;
		end
	end
	endfunction

	function automatic frozen(input [9:0] p);
	begin
		frozen = (TONE_ZERO == 2'd2) && (p == 10'd0);
	end
	endfunction

	always @(posedge clk) begin
		if (!rst_n) begin
			for (i = 0; i < 3; i = i + 1) begin
				tone_cnt[i] <= 10'd1;
			end
			tone_ff <= 3'd0;
		end else if (ss_wren && (ss_addr == SS_COUNT)) begin
			tone_cnt[0] <= ss_wdata[9:0];
			tone_cnt[1] <= ss_wdata[19:10];
			tone_cnt[2] <= ss_wdata[29:20];
		end else if (ss_wren && (ss_addr == SS_GEN)) begin
			tone_ff <= ss_wdata[25:23];
		end else if (tick) begin
			for (i = 0; i < 3; i = i + 1) begin
				if (!frozen(tone_period[i])) begin
					if (tone_cnt[i] == 10'd1) begin
						tone_cnt[i] <= period_eff(tone_period[i]);
						tone_ff[i]  <= ~tone_ff[i];
					end else begin
						tone_cnt[i] <= tone_cnt[i] - 10'd1;
					end
				end
			end
		end
	end

	// Noise generator (SN76489 datasheet p.2 s.2, p.3 Tables 2 and 3)
	// A shift register with an exclusive-OR feedback network and "provisions
	// to protect the shift register from being locked in the zero state": the
	// noise control register write reloads a non-zero constant, which is the
	// same sentence's other half, "whenever the noise control register is
	// changed, the shift register is cleared".  Shift rate is N/512, N/1024,
	// N/2048 or "Tone Generator #3 Output".
	//
	// Mode 3 shifts every 2n ticks, not n.  The three fixed rates are 512, 1024
	// and 2048 chip clocks, exactly the full periods of tone dividers with
	// n = 16, 32 and 64, so "tone generator output" means one shift per full
	// cycle and the fixed rates are the same mechanism with hard-wired
	// divisors.  One shift per flip-flop toggle would put mode 3 an octave
	// above the fixed rates.  NOISE3_X2 = 0 selects that other reading.

	function automatic [11:0] noise_interval(input [1:0] r, input [9:0] np);
		reg [10:0] n_eff;
	begin
		if (np == 10'd0) begin
			// The same n = 0 question the tone counters have.  TONE_ZERO = 2
			// (freeze) is handled by noise_frozen below, not here.
			n_eff = (TONE_ZERO == 2'd0) ? 11'd1024 : 11'd1;
		end else begin
			n_eff = {1'b0, np};
		end
		case (r)
			2'd0:    noise_interval = 12'd32;    // N/512
			2'd1:    noise_interval = 12'd64;    // N/1024
			2'd2:    noise_interval = 12'd128;   // N/2048
			default: noise_interval = (NOISE3_X2 != 0) ? {n_eff, 1'b0}
			                                          : {1'b0, n_eff};
		endcase
	end
	endfunction

	wire noise_frozen = (noise_rate == 2'd3) && frozen(noise_period);

	// White noise: the new MSB is the XOR of the tapped bits.  Periodic: the
	// new MSB is bit 0, a plain rotate, so a single set bit circulates and the
	// output has a 1/LFSR_WIDTH duty cycle - which is why the mixer blocks DC
	// with a filter instead of subtracting a computed mean.
	wire [15:0] lfsr_tapped = lfsr & LFSR_TAPS;
	wire        fb_white    = ^lfsr_tapped;
	wire        fb_bit      = noise_mode ? fb_white : lfsr[0];
	wire [15:0] lfsr_next   = {1'b0, lfsr[15:1]} | (fb_bit ? LFSR_MSB : 16'd0);

	// The bit presented at the output stage after this shift.  NOISE_OUT_POST
	// = 1 is the post-shift bit; 0 is the bit leaving the register, one shift
	// interval earlier.
	wire        noise_bit   = (NOISE_OUT_POST != 0) ? lfsr_next[0] : lfsr[0];

	wire noise_shift = tick && !noise_frozen && (noise_cnt <= 12'd1);

	always @(posedge clk) begin
		if (!rst_n) begin
			noise_cnt <= 12'd32;
			lfsr      <= LFSR_RESET;
			noise_out <= 1'b0;
		end else if (ss_wren && (ss_addr == SS_NOISE)) begin
			noise_cnt <= ss_wdata[11:0];
		end else if (ss_wren && (ss_addr == SS_GEN)) begin
			lfsr      <= ss_wdata[15:0];
			noise_out <= ss_wdata[26];
		end else begin
			// A noise control write clears the shift register but deliberately
			// does not restart the interval counter: the datasheet sentence is
			// about the shift register only.
			if (wr_noise_c) begin
				lfsr <= LFSR_RESET;
			end else if (noise_shift) begin
				lfsr      <= lfsr_next;
				noise_out <= (noise_mode && (LFSR_INVERT != 0)) ? ~noise_bit
				                                                : noise_bit;
			end

			if (tick && !noise_frozen) begin
				if (noise_cnt <= 12'd1) begin
					noise_cnt <= noise_interval(noise_rate, noise_period);
				end else begin
					noise_cnt <= noise_cnt - 12'd1;
				end
			end
		end
	end

	// Attenuators (SN76489 datasheet p.2 Table 1, p.4 s.6, p.6 electrical characteristics)
	// Four bits of binary-weighted 16/8/4/2 dB steps with all-ones meaning OFF,
	// so attenuation in dB is 2 * att.  One 16-entry 12-bit ROM is shared by
	// all eight attenuators: value[i] = round(4095 * 10^(-2i/20)), value[15]=0.
	// Every neighbouring ratio lands between 1.2585 and 1.2615 against the
	// ideal 1.258925, within 0.02 dB - far tighter than the part's own +/-1 dB
	// tolerance.  The 12-bit domain is chosen so four channels sum to at most
	// 16380, which the mixer takes to full scale with a shift and no
	// multiplier.

	function automatic [11:0] vol_rom(input [3:0] att);
	begin
		case (att)
			4'd0:    vol_rom = 12'd4095;
			4'd1:    vol_rom = 12'd3253;
			4'd2:    vol_rom = 12'd2584;
			4'd3:    vol_rom = 12'd2052;
			4'd4:    vol_rom = 12'd1630;
			4'd5:    vol_rom = 12'd1295;
			4'd6:    vol_rom = 12'd1029;
			4'd7:    vol_rom = 12'd817;
			4'd8:    vol_rom = 12'd649;
			4'd9:    vol_rom = 12'd516;
			4'd10:   vol_rom = 12'd410;
			4'd11:   vol_rom = 12'd325;
			4'd12:   vol_rom = 12'd258;
			4'd13:   vol_rom = 12'd205;
			4'd14:   vol_rom = 12'd163;
			default: vol_rom = 12'd0;            // 15 = OFF
		endcase
	end
	endfunction

	// Each channel output is unipolar, as the datasheet's "conventional
	// operational amplifier summing circuit" implies (SN76489 datasheet p.3
	// s.3).  DC removal belongs to the coupling capacitor on the real board
	// (SN76489 datasheet p.7 Figure 2), and so to ngp_mixer here, not to this
	// module.

	wire [11:0] chl0 = tone_ff[0] ? vol_rom(att_l[0]) : 12'd0;
	wire [11:0] chl1 = tone_ff[1] ? vol_rom(att_l[1]) : 12'd0;
	wire [11:0] chl2 = tone_ff[2] ? vol_rom(att_l[2]) : 12'd0;
	wire [11:0] chl3 = noise_out  ? vol_rom(att_l[3]) : 12'd0;

	wire [11:0] chr0 = tone_ff[0] ? vol_rom(att_r[0]) : 12'd0;
	wire [11:0] chr1 = tone_ff[1] ? vol_rom(att_r[1]) : 12'd0;
	wire [11:0] chr2 = tone_ff[2] ? vol_rom(att_r[2]) : 12'd0;
	wire [11:0] chr3 = noise_out  ? vol_rom(att_r[3]) : 12'd0;

	wire [13:0] sum_l = {2'd0, chl0} + {2'd0, chl1} + {2'd0, chl2} + {2'd0, chl3};
	wire [13:0] sum_r = {2'd0, chr0} + {2'd0, chr1} + {2'd0, chr2} + {2'd0, chr3};

	// Sound disable mutes the output stage only; "disable, reprogram,
	// re-enable" therefore works.  The write path is deliberately always live.
	assign out_l = enable ? sum_l : 14'd0;
	assign out_r = enable ? sum_r : 14'd0;

	// Savestate tap (rebased to 0x0-0x5).  Bit 12 of SS_MISC echoes the
	// `enable` input and is ignored on write: 0xB8 lives in ngp_sysreg, which
	// owns and saves it.

	reg [31:0] ss_rdata_r;

	always @* begin
		case (ss_addr)
			SS_PERIOD: ss_rdata_r = {2'd0, tone_period[2], tone_period[1], tone_period[0]};
			SS_COUNT:  ss_rdata_r = {2'd0, tone_cnt[2], tone_cnt[1], tone_cnt[0]};
			SS_GEN:    ss_rdata_r = {5'd0, noise_out, tone_ff, div16, noise_mode, noise_rate, lfsr};
			SS_NOISE:  ss_rdata_r = {10'd0, noise_period, noise_cnt};
			SS_ATT:    ss_rdata_r = {att_l[3], att_l[2], att_l[1], att_l[0],
			                         att_r[3], att_r[2], att_r[1], att_r[0]};
			SS_MISC:   ss_rdata_r = {18'd0, stb_q, enable, ready_cnt, latch1, latch0};
			default:   ss_rdata_r = 32'd0;
		endcase
	end

	assign ss_rdata = ss_rdata_r;

	wire unused_ok = &{1'b1, ss_wdata, din[6]};

endmodule
