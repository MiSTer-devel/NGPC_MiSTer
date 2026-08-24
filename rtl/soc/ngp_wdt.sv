// Copyright (c) 2026 Jamie Blanks

// TMP95C061 watchdog timer (runaway detector). SFRs 0x6E WDMOD, 0x6F WDCR.
//
// The counter is a 22-stage binary chain clocked by phi = fc/2 (TMP95C061
// datasheet p.156), which is exactly one `ce` tick here, so the four detect
// times land on chain bits 15, 17, 19 and 21:
//
//   WDTP=00  2^16/fc  = 2^15 ce ticks  (10.7 ms at fc = 6.144 MHz)
//   WDTP=01  2^18/fc  = 2^17 ce ticks  (42.7 ms)
//   WDTP=10  2^20/fc  = 2^19 ce ticks  (171 ms)
//   WDTP=11  2^22/fc  = 2^21 ce ticks  (683 ms, the setting the BIOS arms at
//                                       game launch)
//
// Overflow raises INTWD. INTWD is non-maskable but it is not the NMI pin: its
// vector is 0x24 at priority 7, while the NMI pin's is 0x20 (datasheet p.12).
// The shell wires `intwd` to the INTWD channel of ngp_intc, never to NMI.
//
// Overflow also pulls WDTOUT low and, only when WDMOD<RESCR> is set, asserts
// the internal reset. The NGP BIOS leaves RESCR = 0, and no internal reset
// controller is built (see k2_soc's wdt_reset note).
//
// Enable/disable protocol (datasheet p.157):
//   - reset leaves the watchdog enabled (WDMOD = 0x80), which is why the
//     BIOS's first two peripheral writes turn it off;
//   - to disable, clear WDMOD<WDTE> and then write the disable code 0xB1 to
//     WDCR. 0xB1 written while WDTE is still 1 is ignored, so a runaway cannot
//     disable the watchdog with a single stray store;
//   - to re-enable, merely set WDMOD<WDTE> again;
//   - the clear code 0x4E zeroes the counter at any time.
//
// WDTE gates the counter here, so clearing it stops the count on its own and
// the 0xB1 code is the second half of a two-key lock rather than the only off
// switch. The datasheet wording can also be read as WDTE alone doing nothing,
// but no BIOS sequence distinguishes the two: every disable it performs writes
// both.
//
// The watchdog runs in RUN-mode HALT and stops in IDLE and STOP HALT
// (datasheet p.160); `halted` carries the CPU's own HALT state in from the
// shell, and `haltm` (WDMOD[3:2]) and `warm` (WDMOD[4]) leave for the power
// block, which picks the oscillator warm-up length from WARM (datasheet p.28
// note). They are outputs rather than duplicated registers because WDMOD is
// owned here.

module ngp_wdt
#(
	// Datasheet p.156: "The watchdog timer out pin (WDTOUT) outputs 0 to 8 to
	// 20 states and resets itself." The same page also says WDTOUT stays low
	// until the 4EH clear code is written; the two statements cannot both be
	// true, so both are honoured - the pulse self-clears after this many states
	// and a 4EH write clears it early. The exact length inside 8..20 is not
	// documented, hence the parameter.
	parameter [4:0] WDTOUT_STATES = 5'd8
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

	input  wire        halted,       // CPU is in HALT
	output wire [1:0]  haltm,        // WDMOD[3:2] for the standby controller
	output wire        warm,         // WDMOD[4] for the standby controller
	output wire        intwd,        // one-ce pulse on overflow
	output wire        wdtout_n,     // pin, low for WDTOUT_STATES after overflow
	output wire        int_reset,    // WDMOD<RESCR> qualified WDTOUT

	input  wire [7:0]  ss_reg_addr,  // words 0x20-0x21
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [6:0] ADDR_WDMOD = 7'h6E;
	localparam [6:0] ADDR_WDCR  = 7'h6F;

	localparam [7:0] CODE_CLEAR   = 8'h4E;
	localparam [7:0] CODE_DISABLE = 8'hB1;

	// The datasheet p.188 states HALTM = 01 outright ("the hold mode setting
	// register is set to the STOP mode (WDMOD <HALTM1,0> = 0,1)"). RUN = 00 is
	// inferred: it is the reset value and RUN is the mode that changes nothing.
	// The remaining codes are IDLE.
	localparam [1:0] HALTM_RUN = 2'b00;

	localparam [7:0] SS_WORD0 = 8'h20;
	localparam [7:0] SS_WORD1 = 8'h21;

	reg  [7:0]  wdmod;
	reg  [7:0]  wdcr_shadow;
	reg  [21:0] wdcnt;
	reg         wd_disabled;   // the 0xB1 disable latch
	reg  [4:0]  wdtout_cnt;
	reg         wdtout_low;
	reg         intwd_r;

	wire sel_wdmod = (sfr_addr == ADDR_WDMOD);
	wire sel_wdcr  = (sfr_addr == ADDR_WDCR);

	// The watchdog has no read side effects. sfr_rd belongs to the normative
	// SFR port group and is deliberately not used. ss_wdata[31:30] is the zero
	// filler at the top of tap word 0x20 and has nothing to restore into.
	wire unused_ok = &{1'b0, sfr_rd, ss_wdata[31:30]};

	// WDMOD is readable (datasheet p.181; the BIOS does `bit 4,(0x6E)`). WDCR
	// is write only, so it reads 0xFF like every other write-only SFR; its
	// shadow appears on the savestate tap only.
	assign sfr_rdata = sel_wdmod ? wdmod : 8'hFF;

	assign haltm = wdmod[3:2];
	assign warm  = wdmod[4];

	// Counting is gated four ways: the enable bit, the disable latch, the HALT
	// mode, and a requested pause (`run_ce` below).
	//
	// The pause term is load-bearing. Out of reset the watchdog is enabled with
	// WDTP = 00, a 2^15-tick period, and the BIOS does not disable it until its
	// first two stores. A pause raised in that window would let the watchdog
	// bite and vector the BIOS to INT_WATCHDOG instead of the cold workspace
	// initializer. A pause is a MiSTer construct that real silicon never sees,
	// so it must be invisible to the machine; hence run_ce.
	wire wd_enabled = wdmod[7] && !wd_disabled;
	wire wd_count   = wd_enabled && (!halted || (wdmod[3:2] == HALTM_RUN));

	// The whole counting domain, WDTOUT pulse included, stops with the machine.
	wire run_ce = ce && !pause_req;

	wire [21:0] wdcnt_next = wdcnt + 22'd1;

	// Tap select. The overflow is the rising edge of the selected chain bit;
	// the chain is zeroed on overflow so the detect time is also the repeat
	// period (a free-running chain would double it after the first bite).
	wire [3:0] taps_now  = {wdcnt[21],      wdcnt[19],      wdcnt[17],      wdcnt[15]};
	wire [3:0] taps_next = {wdcnt_next[21], wdcnt_next[19], wdcnt_next[17], wdcnt_next[15]};

	wire tap_now  = taps_now[wdmod[6:5]];
	wire tap_next = taps_next[wdmod[6:5]];

	wire wd_overflow = wd_count && tap_next && !tap_now;

	wire ss_sel0 = (ss_reg_addr == SS_WORD0);
	wire ss_sel1 = (ss_reg_addr == SS_WORD1);

	always @(posedge clk) begin
		// One clk_sys wide, raised on the ce tick the overflow happens on.
		intwd_r <= 1'b0;

		if (reset) begin
			wdmod       <= 8'h80;         // WDTE = 1: enabled out of reset
			wdcr_shadow <= 8'h00;
			wdcnt       <= 22'd0;
			wd_disabled <= 1'b0;
			wdtout_cnt  <= 5'd0;
			wdtout_low  <= 1'b0;
		end else begin
			if (run_ce) begin
				if (wd_count) begin
					if (wd_overflow) begin
						wdcnt      <= 22'd0;
						intwd_r    <= 1'b1;
						wdtout_low <= 1'b1;
						wdtout_cnt <= WDTOUT_STATES;
					end else begin
						wdcnt <= wdcnt_next;
					end
				end

				if (wdtout_low && !wd_overflow) begin
					if (wdtout_cnt <= 5'd1) begin
						wdtout_cnt <= 5'd0;
						wdtout_low <= 1'b0;
					end else begin
						wdtout_cnt <= wdtout_cnt - 5'd1;
					end
				end
			end

			// SFR writes are strobe-enabled, never ce-gated.
			if (sfr_wr && sel_wdmod) begin
				wdmod <= sfr_wdata;
				// Setting WDTE alone lifts the disable latch (datasheet p.157).
				if (sfr_wdata[7]) begin
					wd_disabled <= 1'b0;
				end
			end

			if (sfr_wr && sel_wdcr) begin
				wdcr_shadow <= sfr_wdata;
				case (sfr_wdata)
					CODE_CLEAR: begin
						wdcnt      <= 22'd0;
						wdtout_low <= 1'b0;
						wdtout_cnt <= 5'd0;
					end
					CODE_DISABLE: begin
						// Honoured only with WDTE already clear.
						if (!wdmod[7]) begin
							wd_disabled <= 1'b1;
						end
					end
					default: begin
						// Any other code is recorded and otherwise ignored.
					end
				endcase
			end

			// Savestate writes land last and only while parked.
			if (ss_wren && pause_ready) begin
				if (ss_sel0) begin
					wdmod <= ss_wdata[29:22];
					wdcnt <= ss_wdata[21:0];
				end
				if (ss_sel1) begin
					wdcr_shadow <= ss_wdata[15:8];
					wdtout_cnt  <= ss_wdata[5:1];
					wd_disabled <= ss_wdata[0];
					wdtout_low  <= (ss_wdata[5:1] != 5'd0);
				end
			end
		end
	end

	assign intwd     = intwd_r;
	assign wdtout_n  = ~wdtout_low;
	assign int_reset = wdmod[1] && wdtout_low;

	// Nothing is ever in flight: the counter is ce-gated, so freezing the
	// enables freezes it. Ready is reported only in answer to a request, the
	// same convention t900_cpu uses.
	assign pause_ready = pause_req;

	reg [31:0] ss_rdata_r;
	always_comb begin
		ss_rdata_r = 32'd0;
		if (ss_sel0) begin
			ss_rdata_r = {2'b00, wdmod, wdcnt};
		end else if (ss_sel1) begin
			ss_rdata_r = {16'h0000, wdcr_shadow, 2'b00, wdtout_cnt, wd_disabled};
		end
	end

	assign ss_rdata = ss_rdata_r;

endmodule
