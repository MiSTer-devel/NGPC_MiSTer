// Copyright (c) 2026 Jamie Blanks

// TMP95C061 10-bit A/D converter. SFRs 0x60-0x67 ADREG0-3 L/H, 0x6D ADMOD.
//
// Successive approximation against a 1024-step resistor ladder, a four-way
// input mux shared with port 9, and four result registers (TMP95C061 datasheet
// p.148). There is no sample and hold, so the result is taken from the input
// at the moment the conversion ends.
//
// On the NGP only AN0 is bonded, and it carries the main battery voltage. This
// block is boot critical: the BIOS starts a conversion with ADMOD = 0x04
// (start, single, high speed, channel 0) and its INTAD handler stores
// ADREG0 >> 6 at 0x6F80. Boot needs a value above 0x210; if INTAD never fires
// the battery state machine hangs and the unit shuts down. `an0` is therefore
// driven with a constant 0x3FF from the fabric, with the low-battery paths
// reachable by driving it lower.
//
// Timing (datasheet p.153): a conversion is 160 states with ADCS = 0 (high
// speed) and 320 with ADCS = 1. States are `ce` ticks and they are really
// counted. A down counter is loaded at the start so a mid-conversion ADCS
// change cannot corrupt the count.
//
// Read side effects (datasheet p.152 items (7) and (8)), both fired by the
// sfr_rd strobe only, so observing sfr_rdata disturbs nothing:
//   - reading any ADREG clears EOCF;
//   - reading an ADREGnH additionally clears the INTAD request flip-flop in
//     the interrupt controller, which leaves this module as `intad_clear`.
//
// Two behaviours the sources do not settle:
//   1. EOCF in repeat mode. The datasheet p.152 item (6) describes EOCF and
//      INTAD only for single mode. EOCF is set at every conversion end here,
//      including repeat, because that is the only way software can poll a
//      repeating conversion; ADBF and EOCF are therefore both set in repeat
//      mode.
//   2. ADCH = 11 in scan mode. Datasheet p.152 item (5) lists only AN0->AN1,
//      AN0->AN1->AN2 and AN0->AN1->AN2->AN3 for the four ADCH codes, so the
//      fourth code has no documented meaning. The full four-channel scan is
//      used for it (SCAN_ADCH11_LAST).

module ngp_adc
#(
	// AN1-AN3 are not bonded on the NGP board. What an unbonded analog input
	// converts to is a board fact we do not have, so it is a parameter rather
	// than a silent zero.
	parameter [9:0] AN_UNBONDED = 10'h000,
	// Last channel of a scan when ADCH = 11 (undocumented, see above).
	parameter [1:0] SCAN_ADCH11_LAST = 2'd3
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire [6:0]  sfr_addr,
	input  wire [7:0]  sfr_wdata,
	input  wire        sfr_wr,
	input  wire        sfr_rd,
	output wire [7:0]  sfr_rdata,

	input  wire [9:0]  an0,          // battery model; AN1-AN3 unbonded
	output wire        intad,
	output wire        intad_clear,  // ADREGnH read -> clear IADC in ngp_intc

	input  wire [7:0]  ss_reg_addr,  // words 0x1C-0x1F
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [6:0] ADDR_ADMOD = 7'h6D;

	localparam [8:0] CONV_STATES_FAST = 9'd160;
	localparam [8:0] CONV_STATES_SLOW = 9'd320;

	localparam [7:0] SS_ADREG01 = 8'h1C;
	localparam [7:0] SS_ADREG23 = 8'h1D;
	localparam [7:0] SS_MODE    = 8'h1E;
	localparam [7:0] SS_COUNT   = 8'h1F;

	// ADMOD writable half: [5] REPET, [4] SCAN, [3] ADCS, [2] ADS (always
	// reads 0), [1:0] ADCH. Bits 7 and 6 are the read-only EOCF and ADBF.
	reg  [5:0]  admod_w;
	reg         eocf;
	reg         adbf;

	reg  [9:0]  adreg0;
	reg  [9:0]  adreg1;
	reg  [9:0]  adreg2;
	reg  [9:0]  adreg3;

	reg  [1:0]  conv_ch;     // channel currently converting
	reg  [8:0]  conv_cnt;    // states remaining in this channel's conversion
	reg         intad_r;

	wire        repet = admod_w[5];
	wire        scan  = admod_w[4];
	wire        adcs  = admod_w[3];
	wire [1:0]  adch  = admod_w[1:0];

	// 0x60-0x67: address bit 0 picks H over L, bits 2:1 pick the channel.
	wire       sel_adreg = (sfr_addr[6:3] == 4'b1100);
	wire [1:0] adreg_ch  = sfr_addr[2:1];
	wire       adreg_hi  = sfr_addr[0];
	wire       sel_admod = (sfr_addr == ADDR_ADMOD);

	reg [9:0] adreg_sel;
	always_comb begin
		case (adreg_ch)
			2'd0:    adreg_sel = adreg0;
			2'd1:    adreg_sel = adreg1;
			2'd2:    adreg_sel = adreg2;
			default: adreg_sel = adreg3;
		endcase
	end

	// ADREGnL reads {result[1:0], 6'b111111}: the low six bits are fixed 1
	// (datasheet p.183). ADREGnH reads result[9:2]. ADS always reads back 0.
	wire [7:0] adreg_rd = adreg_hi ? adreg_sel[9:2] : {adreg_sel[1:0], 6'b111111};
	wire [7:0] admod_rd = {eocf, adbf, admod_w[5:3], 1'b0, admod_w[1:0]};

	reg [7:0] sfr_rdata_r;
	always_comb begin
		if (sel_adreg) begin
			sfr_rdata_r = adreg_rd;
		end else if (sel_admod) begin
			sfr_rdata_r = admod_rd;
		end else begin
			sfr_rdata_r = 8'hFF;
		end
	end

	// ADMOD bits 7 and 6 are the read-only EOCF and ADBF flags, so a write
	// ignores them. The marked ss_wdata bits are zero filler in the tap layout
	// and have nothing to restore into.
	wire unused_ok = &{1'b0, sfr_wdata[7:6], ss_wdata[31:26], ss_wdata[15:14]};

	assign sfr_rdata   = sfr_rdata_r;
	assign intad_clear = sfr_rd && sel_adreg && adreg_hi;
	assign intad       = intad_r;

	// Last channel of a scan (datasheet p.152 item (5)).
	reg [1:0] scan_last;
	always_comb begin
		case (adch)
			2'd0:    scan_last = 2'd1;   // AN0 -> AN1
			2'd1:    scan_last = 2'd2;   // AN0 -> AN1 -> AN2
			2'd2:    scan_last = 2'd3;   // AN0 -> AN1 -> AN2 -> AN3
			default: scan_last = SCAN_ADCH11_LAST;
		endcase
	end

	wire [1:0] first_ch   = scan ? 2'd0 : adch;
	wire [8:0] conv_load  = adcs ? CONV_STATES_SLOW : CONV_STATES_FAST;
	wire       last_ch    = !scan || (conv_ch == scan_last);

	// A conversion may only be launched when the converter is idle. Pausing
	// blocks a launch so an in-flight conversion drains and repeat mode stops
	// re-arming.
	wire start_req = sfr_wr && sel_admod && sfr_wdata[2] && !adbf && !pause_req;

	wire conv_done = ce && adbf && (conv_cnt == 9'd1);

	// No sample and hold (datasheet p.148): the ladder is compared against the pin
	// as it stands, so the result is taken at the end of the conversion.
	reg [9:0] an_sel;
	always_comb begin
		case (conv_ch)
			2'd0:    an_sel = an0;
			default: an_sel = AN_UNBONDED;
		endcase
	end

	wire ss_sel_reg01 = (ss_reg_addr == SS_ADREG01);
	wire ss_sel_reg23 = (ss_reg_addr == SS_ADREG23);
	wire ss_sel_mode  = (ss_reg_addr == SS_MODE);
	wire ss_sel_count = (ss_reg_addr == SS_COUNT);

	always @(posedge clk) begin
		intad_r <= 1'b0;

		if (reset) begin
			admod_w  <= 6'd0;             // channel 0, high speed, single, idle
			eocf     <= 1'b0;
			adbf     <= 1'b0;
			adreg0   <= 10'd0;
			adreg1   <= 10'd0;
			adreg2   <= 10'd0;
			adreg3   <= 10'd0;
			conv_ch  <= 2'd0;
			conv_cnt <= 9'd0;
		end else begin
			if (ce && adbf && (conv_cnt != 9'd0)) begin
				conv_cnt <= conv_cnt - 9'd1;
			end

			// Reading any ADREG clears EOCF, on the strobe only. This sits
			// ahead of the completion below so that a read landing on the
			// exact state a conversion ends in cannot swallow the new flag.
			if (sfr_rd && sel_adreg) begin
				eocf <= 1'b0;
			end

			if (conv_done) begin
				case (conv_ch)
					2'd0:    adreg0 <= an_sel;
					2'd1:    adreg1 <= an_sel;
					2'd2:    adreg2 <= an_sel;
					default: adreg3 <= an_sel;
				endcase

				if (last_ch) begin
					// End of the conversion the CPU asked for: flag it and
					// interrupt. In repeat mode the converter immediately
					// re-arms and ADBF never falls.
					eocf    <= 1'b1;
					intad_r <= 1'b1;
					if (repet && !pause_req) begin
						conv_ch  <= first_ch;
						conv_cnt <= conv_load;
					end else begin
						adbf     <= 1'b0;
						conv_cnt <= 9'd0;
					end
				end else begin
					conv_ch  <= conv_ch + 2'd1;
					conv_cnt <= conv_load;
				end
			end

			// SFR writes are strobe-enabled, never ce-gated.
			if (sfr_wr && sel_admod) begin
				admod_w <= {sfr_wdata[5:3], 1'b0, sfr_wdata[1:0]};
				if (start_req) begin
					adbf     <= 1'b1;
					eocf     <= 1'b0;
					conv_ch  <= sfr_wdata[4] ? 2'd0 : sfr_wdata[1:0];
					conv_cnt <= sfr_wdata[3] ? CONV_STATES_SLOW : CONV_STATES_FAST;
				end
			end

			// Savestate writes land last and only while parked.
			if (ss_wren && pause_ready) begin
				if (ss_sel_reg01) begin
					adreg1 <= ss_wdata[25:16];
					adreg0 <= ss_wdata[9:0];
				end
				if (ss_sel_reg23) begin
					adreg3 <= ss_wdata[25:16];
					adreg2 <= ss_wdata[9:0];
				end
				if (ss_sel_mode) begin
					admod_w <= ss_wdata[13:8];
					eocf    <= ss_wdata[7];
					adbf    <= ss_wdata[6];
					conv_ch <= ss_wdata[1:0];
				end
				if (ss_sel_count) begin
					conv_cnt <= ss_wdata[8:0];
				end
			end
		end
	end

	// A conversion in flight must finish before the enables stop.
	assign pause_ready = pause_req && !adbf;

	reg [31:0] ss_rdata_r;
	always_comb begin
		ss_rdata_r = 32'd0;
		if (ss_sel_reg01) begin
			ss_rdata_r = {6'h0, adreg1, 6'h0, adreg0};
		end else if (ss_sel_reg23) begin
			ss_rdata_r = {6'h0, adreg3, 6'h0, adreg2};
		end else if (ss_sel_mode) begin
			ss_rdata_r = {16'h0000, 2'b00, admod_w, eocf, adbf, 4'h0, conv_ch};
		end else if (ss_sel_count) begin
			ss_rdata_r = {23'h0, conv_cnt};
		end
	end

	assign ss_rdata = ss_rdata_r;

endmodule
