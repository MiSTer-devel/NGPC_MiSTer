// Copyright (c) 2026 Jamie Blanks

// NGP audio output stage: DC block, gain, sum, and saturate.
//
// On the real board the PSG's four attenuated channels are summed by an
// operational amplifier and leave the chip through a series capacitor into the
// volume pot (SN76489 datasheet p.3 s.3, p.7 Fig. 2), and the two 8-bit DACs
// join the same node.  This module is that node: the capacitor is a one-pole
// DC blocker and the amplifier is a pair of shifts.  Nothing here is a
// register the CPU can see.
//
//	per side:  psg sum          14 bits unsigned, 0..16380
//	           -> DC block      one pole, run on the 192 kHz tick
//	           -> psg gain      shift, trim-controlled
//	           +  (dac - 128) << dac gain
//	           -> 21-bit signed accumulator
//	           -> saturate to signed 16
//	           -> register on ce_3m072 -> audio_l / audio_r
//
// The framework sets AUDIO_S = 1, so these samples are signed.  Handing
// audio_out unsigned samples declared as signed makes it XOR the sign bit, and
// the mistake is silent.
//
// Every gain is a shift; there is no multiplier.  t6w28's 12-bit volume ROM is
// sized so four channels sum to at most 16380, full scale to within 2.5 dB, so
// nominal PSG gain is x1 and nominal DAC gain is a shift of 6.  Peak is then
// 24572 and saturation never fires in normal operation.  The accumulator is 21
// bits so three shifts of trim headroom on each path cannot wrap it ahead of
// the saturator.
//
// The DC blocker is a filter rather than a subtracted mean: a mean is exact
// only for a 50 percent square wave, wrong for periodic noise whose duty cycle
// is 1/LFSR_WIDTH, and it would suppress the switch-on transient.  The DAC
// path needs no blocker, since 0x80 is the documented silence level and
// dac - 128 is already centred, so a parked DAC holds a constant level.
//
// Reconstruction is not duplicated here: sys/audio_out.sv already runs its IIR
// at sys_top's 7.056 MHz `flt_ce`, and a second core-side low pass audibly
// degrades the PSG.  Outputs are registered on ce_3m072, holding each value
// 325 ns (about 8 CLK_AUDIO cycles), which meets the framework's two-cycle
// hold rule.
//
// The 192 kHz blocker tick comes from this module's own divide-by-16 of ce.
// DCB_SHIFT = 11 puts the corner near 15 Hz at that rate; running the blocker
// on ce_3m072 instead would put it at 239 Hz and thin the bass.  The divider
// phase, blocker and output history are all savestate state -- omitting them
// makes a restore resume as silence or static -- and reserved addresses read
// zero to keep the global layout fixed.

module ngp_mixer
#(
	// One-pole DC blocker, evaluated on the 192 kHz tick.
	// corner ~= 192e3 / (2*pi*2^DCB_SHIFT); 11 gives about 15 Hz.
	parameter integer DCB_SHIFT = 11
)
(
	input  wire        clk,           // clk_sys, 49.152 MHz
	input  wire        ce,            // ce_3m072
	input  wire        reset,         // active high

	input  wire [13:0] psg_l,         // t6w28 out_l, unsigned 0..16380
	input  wire [13:0] psg_r,
	input  wire [7:0]  dac_l,         // 0xA2 latch, unsigned, 0x80 = silence
	input  wire [7:0]  dac_r,         // 0xA3 latch

	// Trim, 4 = nominal, each step 6 dB.  Nominal is x1 for the PSG and a
	// shift of 6 for the DACs, which is the 0.5 ratio above.
	input  wire [2:0]  mix_psg_gain,
	input  wire [2:0]  mix_dac_gain,

	// Complete mixer history uses local words 0,1,4,5,6,9. The intervening
	// addresses remain reserved so the global savestate layout does not move.
	// Writes are one clk_sys pulse
	// during a parked restore and therefore must not be qualified by `ce`.
	input  wire [3:0]  ss_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,

	// Registered on ce; ngp_snd may pass them straight to the framework.
	output wire signed [15:0] audio_l,
	output wire signed [15:0] audio_r
);

	localparam integer ACCW  = 21;                     // mix accumulator
	localparam integer MIXW  = ACCW + 1;               // sign-extension guard
	localparam integer DCBW  = 18;                     // DC blocker output
	localparam integer DCBW2 = DCBW + DCB_SHIFT;       // + fractional bits
	wire unused_ss_ok = &{1'b1, ss_wdata[31:29]};

	// 192 kHz tick, the rate the DC blocker runs at

	reg [3:0] div16;

	always @(posedge clk) begin
		if (reset) begin
			div16 <= 4'd0;
		end else if (ss_wren && (ss_addr == 4'd0)) begin
			div16 <= ss_wdata[17:14];
		end else if (ce) begin
			div16 <= div16 + 4'd1;
		end
	end

	wire tick = ce && (div16 == 4'd15);

	// Gain shifts.  Pure functions of their arguments.

	function automatic signed [ACCW-1:0] gain_psg(input signed [DCBW-1:0] v,
	                                              input [2:0] trim);
		reg signed [ACCW-1:0] ve;
	begin
		ve = {{(ACCW-DCBW){v[DCBW-1]}}, v};
		case (trim)
			3'd0:    gain_psg = ve >>> 4;
			3'd1:    gain_psg = ve >>> 3;
			3'd2:    gain_psg = ve >>> 2;
			3'd3:    gain_psg = ve >>> 1;
			3'd4:    gain_psg = ve;          // nominal, x1
			3'd5:    gain_psg = ve <<< 1;
			3'd6:    gain_psg = ve <<< 2;
			default: gain_psg = ve <<< 3;
		endcase
	end
	endfunction

	function automatic signed [ACCW-1:0] gain_dac(input signed [8:0] v,
	                                              input [2:0] trim);
		reg signed [ACCW-1:0] ve;
	begin
		ve = {{(ACCW-9){v[8]}}, v};
		case (trim)
			3'd0:    gain_dac = ve <<< 2;
			3'd1:    gain_dac = ve <<< 3;
			3'd2:    gain_dac = ve <<< 4;
			3'd3:    gain_dac = ve <<< 5;
			3'd4:    gain_dac = ve <<< 6;    // nominal, DAC spans -8192..+8128
			3'd5:    gain_dac = ve <<< 7;
			3'd6:    gain_dac = ve <<< 8;
			default: gain_dac = ve <<< 9;
		endcase
	end
	endfunction

	function automatic signed [15:0] sat16(input signed [MIXW-1:0] v);
	begin
		if (v > $signed({{(MIXW-16){1'b0}}, 16'sh7FFF})) begin
			sat16 = 16'sh7FFF;
		end else if (v < $signed({{(MIXW-16){1'b1}}, 16'sh8000})) begin
			sat16 = 16'sh8000;
		end else begin
			sat16 = v[15:0];
		end
	end
	endfunction

	// One side of the mixer, instantiated twice

	wire [13:0] psg_in [0:1];
	wire [7:0]  dac_in [0:1];

	assign psg_in[0] = psg_l;
	assign psg_in[1] = psg_r;
	assign dac_in[0] = dac_l;
	assign dac_in[1] = dac_r;

	wire signed [15:0] out_s [0:1];
	wire [13:0] ss_x_prev [0:1];
	wire signed [DCBW2-1:0] ss_dcb [0:1];

	genvar g;
	generate
		for (g = 0; g < 2; g = g + 1) begin : gen_side
			localparam [3:0] SS_XPREV = (g == 0) ? 4'd0 : 4'd5;
			localparam [3:0] SS_DCB   = (g == 0) ? 4'd1 : 4'd6;
			localparam [3:0] SS_OUT   = (g == 0) ? 4'd4 : 4'd9;

			// --- DC blocker: y <= x - x_prev + y - (y >>> DCB_SHIFT) -----
			// The state carries DCB_SHIFT fractional bits.  Without them the
			// leak term truncates to zero for anything below 2^DCB_SHIFT and
			// the filter stops dead with up to 2047 counts of DC still on the
			// output -- 12 percent of full scale, a permanent offset rather
			// than a blocked one.  With them the residue is under one output
			// LSB in both directions.
			reg  [13:0]             x_prev;
			reg  signed [DCBW2-1:0] dcb;

			wire signed [DCBW-1:0] x_now = $signed({{(DCBW-14){1'b0}}, psg_in[g]});
			wire signed [DCBW-1:0] x_old = $signed({{(DCBW-14){1'b0}}, x_prev});
			wire signed [DCBW-1:0] dx    = x_now - x_old;

			wire signed [DCBW2-1:0] dx_e = {{(DCBW2-DCBW){dx[DCBW-1]}}, dx};

			// y = state >>> DCB_SHIFT is just the top DCBW bits.
			wire signed [DCBW-1:0] dcb_y = dcb[DCBW2-1 -: DCBW];

			always @(posedge clk) begin
				if (reset) begin
					x_prev <= 14'd0;
					dcb    <= {DCBW2{1'b0}};
				end else if (ss_wren && (ss_addr == SS_XPREV)) begin
					x_prev <= ss_wdata[13:0];
				end else if (ss_wren && (ss_addr == SS_DCB)) begin
					dcb <= ss_wdata[DCBW2-1:0];
				end else if (tick) begin
					x_prev <= psg_in[g];
					dcb    <= dcb + (dx_e <<< DCB_SHIFT) - (dcb >>> DCB_SHIFT);
				end
			end

			// --- gain and sum -------------------------------------------
			wire signed [8:0] dac_c = $signed({1'b0, dac_in[g]}) - 9'sd128;

			wire signed [ACCW-1:0] acc = gain_psg(dcb_y, mix_psg_gain) +
			                             gain_dac(dac_c, mix_dac_gain);

			wire signed [MIXW-1:0] acc_x = {acc[ACCW-1], acc};

			// --- saturate and register ----------------------------------
			reg signed [15:0] out_r;

			always @(posedge clk) begin
				if (reset) begin
					out_r <= 16'sd0;
				end else if (ss_wren && (ss_addr == SS_OUT)) begin
					out_r <= ss_wdata[15:0];
				end else if (ce) begin
					out_r <= sat16(acc_x);
				end
			end

			assign out_s[g] = out_r;
			assign ss_x_prev[g] = x_prev;
			assign ss_dcb[g] = dcb;
		end
	endgenerate

	reg [31:0] ss_rdata_r;
	always @* begin
		case (ss_addr)
			4'd0: ss_rdata_r = {14'd0, div16, ss_x_prev[0]};
			4'd1: ss_rdata_r = {{(32-DCBW2){ss_dcb[0][DCBW2-1]}}, ss_dcb[0]};
			4'd4: ss_rdata_r = {{16{out_s[0][15]}}, out_s[0]};
			4'd5: ss_rdata_r = {18'd0, ss_x_prev[1]};
			4'd6: ss_rdata_r = {{(32-DCBW2){ss_dcb[1][DCBW2-1]}}, ss_dcb[1]};
			4'd9: ss_rdata_r = {{16{out_s[1][15]}}, out_s[1]};
			default: ss_rdata_r = 32'd0;
		endcase
	end

	assign ss_rdata = ss_rdata_r;

	assign audio_l = out_s[0];
	assign audio_r = out_s[1];

endmodule
