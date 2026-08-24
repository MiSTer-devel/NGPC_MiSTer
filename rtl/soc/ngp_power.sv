// Copyright (c) 2026 Jamie Blanks

// NGP power / standby / wake controller.
//
// The machine's supervisor: the power-button hold stretcher, the OFF/WARM-UP/
// RUN/HALT-RUN state machine deciding when the main clock domain and the CPU
// may tick, the wake path that turns a button press or an RTC alarm arriving
// while the machine is frozen into a clean interrupt edge, and savestate
// internals word 92.  It owns no CPU-visible register: the power latches
// (0xB4-0xB7) and the clock gear (0x80) are ngp_sysreg's, the gear divider is
// ngp_clocks', WDMOD (and so HALTM and WARM) is ngp_wdt's, and the RTC counter
// chain and alarm comparator are ngp_rtc's.  This block never divides; it only
// freezes.
//
// `ce` is the UNGATED 6.144 MHz machine enable, because the warm-up timer and
// the button stretcher must keep counting while the machine is frozen.  The
// gates this block produces are applied by the parent at the point of use;
// feeding a gated enable back in would restart the divider phase on every wake.
// Single-clk_sys-cycle strobes (ss_wren, alarm_pulse, restore_standby) are
// therefore captured on plain clk, outside the `ce` block -- re-gating a
// one-cycle strobe by an enable that fires one cycle in 8 (128 at gear 4) drops
// most of them.  Conversely, an interrupt handed to ngp_intc must be a LEVEL
// that survives at least two of that controller's own enables: its pin-history
// sampler forms edges on the TLCS state enable, so a one-clk_sys pulse into
// int0 is invisible at every gear.  Hence INT0_HOLD_TICKS and the wake_nmi hold.
//
// States:
//   S_OFF       main clock domain frozen, LCD black, RTC still ticking.  Both
//               the entry state at core start and the standby state.
//   S_WARMUP    oscillator restart; clocks run, CPU stopped.  WDMOD<WARM> picks
//               2^14/fc (2.67 ms) or 2^16/fc (10.7 ms) (TMP95C061 datasheet
//               p.28).  Counted on the UNGEARED enable: an oscillator restart
//               is a physical time, not a CPU-clock count.
//   S_ARM       clocks and CPU running, wake interrupt still withheld for
//               ARM_TICKS enables with the pin idle, so ngp_intc's pin history
//               -- frozen with the rest of the machine -- refreshes before the
//               wake edge, instead of comparing against a stale sample.
//   S_SETTLE    the HALT has been seen; the machine keeps running while the CPU
//               retires it.
//   S_RUN       everything live.
//   S_HALT_RUN  the CPU executed HALT with HALTM = 00 (RUN): a plain CPU stall,
//               peripherals/timers/video keep running and the screen is not
//               blanked.  Games halt this way (WDMOD = 0xF0 at game bootup), so
//               treating it as standby would freeze a running game.
//
// Standby entry is `halt` AND HALTM != 00, on the RISING edge of `halted`.  The
// edge matters: after a wake the CPU stays halted until it accepts the
// interrupt, and a level-triggered entry would drop it back into standby before
// the wake could be delivered.
//
// S_SETTLE exists because `halted` rises one CPU STATE before the HALT retires:
// t900_seq sets halt_r in the execute commit but retires the instruction later,
// when retire_now holds, and for the BIOS's own `halt` at 0xFF1127 those are
// different states.  Freezing on the rising edge parks the CPU mid-instruction
// with the retirement lost, so a savestate taken there is not at an instruction
// boundary -- the one thing t900_cpu's pause contract promises.  The gear hides
// it: the shutdown routine sets gear 4 at 0xFF10B6 before halting, so one CPU
// state is 16 enables.  If an interrupt releases the HALT during the settle the
// machine was never asleep and returns to S_RUN.
//
// `power_off_latch` (0xB4 == 0xA0) is deliberately not an input: the latch must
// never stop the clock, because the CPU has to reach the `halt` two
// instructions after writing it.  It is a report, and reporting it is the
// parent's job.
//
// The power button is HELD, not tapped.  The colour BIOS gate, decoded:
//
//   00ff1a45  ld (0x6c1e),0x0             ; iteration counter
//   00ff1a4d  incw 0x1,(0x6c1e)
//   00ff1a51  cp (0x6c1e),0x28            ; 40 iterations
//   00ff1a57  jrl NC/UGE,0x00ff1a73       ; held all the way -> boot
//   00ff1a5a  calr 0x00ff1a6a             ; delay: ld BC,0xb4; nop; nop; djnz
//   00ff1a5d  ld A,(0x000000b1)           ; read the button
//   00ff1a60  and A,0x1
//   00ff1a63  or B,A                      ; ANY read of "released" sticks
//   00ff1a65  jr Z,0x00ff1a4d
//   00ff1a67  jrl T,0x00ff1074            ; released once -> power off
//
// Costed from this project's cycle table (NOP = 2 states, DJNZ = 4 states):
//
//   inner delay  180 x (nop + nop + djnz) = 180 x 8      = 1440 states
//   outer body   + ~35 states of counter/compare/read    ~ 1475 states
//   whole gate   x 40                                    ~ 59,000 states
//
// and the gate runs at CLOCK GEAR 4, because the shutdown routine sets gear 4
// at 0xFF10B6 before halting and the NMI path does not restore the gear
// (`ld (0x80),0x3` at 0xFF1A73) until after the gate is satisfied.  At gear 4
// the geared oscillator is 384 kHz and the execution-state rate 192 kHz, so
// 59,000 states is about 307 ms.  HOLD_TICKS must exceed that or a scripted tap
// or OSD press meets a machine that correctly refuses to boot.
//
// Cold start.  With `cold_run` = 0 the block starts in S_OFF with the CPU never
// having run, so the first wake has no standby context and no NMI is delivered:
// the CPU runs the reset vector, which cold-initialises everything and ends by
// jumping into the power-down routine at 0xFF1074 -- a hardware reset on this
// machine ends in standby.  That halt sets `warm_ctx`, so a cold start takes two
// presses.  With `cold_run` = 1, which is what k2_soc ties, the release of
// /RESET is itself that first release and the BIOS's power-down tail parks the
// machine in standby with `warm_ctx` set, so the user's first press is the warm
// one; on silicon /RESET releasing starts the CPU, and the BIOS reset path is
// written to end in a halt, which only makes sense if it runs without a press.
// Either way an NMI on the first press would fail a decoded check:
// 0xFF18A1/0xFF18AA demand 0x4800 <= XSP <= 0x6C00 and a Toshiba reset leaves
// XSP at 0x100, so the handler would power the machine straight back off.  After
// an NV restore the parent pulses `restore_standby`, setting `warm_ctx` up front
// so the very first press takes the warm path.
//
// The OFF-state wake is not gated by 0xB3 bit 2.  `nmi_n_i` arrives already
// gated by that bit inside ngp_sysreg and the gate applies while the machine
// runs, but in S_OFF the button wakes unconditionally: on a console whose
// sub-battery has just been fitted every register including 0xB3 is at its reset
// value, so an off->on transition depending on a programmed value could never
// happen.  The two never disagree in practice, because the BIOS arms the gate at
// 0xFF1114, five instructions before its `halt`.
//
// Choices the documents do not settle:
//   1. WAKE_CLEARS_LATCH, default 0.  A wake edge is described as clearing
//      `power_off_latch`, but the NMI handler does that itself: 0xFF18CB
//      word-compares 0xB4 against 0x000A and, when it does not match, writes
//      0x000A and runs a ~250-iteration settle delay before re-measuring the
//      battery.  If hardware cleared the latch that branch would be dead code,
//      so the default is that it does not.  With the parameter on,
//      `pwr_latch_clr` pulses at the wake edge; ngp_sysreg has no input for it
//      today, so the pulse is a report only.
//   2. IDLE_SKIPS_WARMUP, default 1.  IDLE (HALTM = 1x) is released without a
//      warm-up because there is no oscillator to restart (TMP95C061 datasheet
//      p.28 Table 3.4(2)); STOP (01) warms up.  Neither BIOS is known to use
//      IDLE, so this is untested software territory.
//   3. ARM_TICKS and INT0_HOLD_TICKS are implementation numbers with no
//      hardware meaning; both are sized against the slowest gear, where one
//      ce_cpu is 16 of this block's enables.
//   4. An alarm wakes the machine whether or not INTE0AD enables INT0; doing
//      otherwise would need an enable export from ngp_intc.  A masked INT0 then
//      leaves the machine awake with the CPU still halted, which is harmless
//      because the next power press is non-maskable.

module ngp_power
#(
	// Minimum time the button reads as pressed after a tap or an OSD press, in
	// 6.144 MHz enables.  One NGP frame is 515 x 199 = 102,485 of them, so this
	// is 24 frames = 400 ms, comfortably past the ~307 ms BIOS hold gate at
	// clock gear 4.  A held pad or OSD entry extends past it naturally; this
	// only bounds a release.
	parameter [21:0] HOLD_TICKS        = 22'd2_460_000,

	// Oscillator warm-up, WDMOD<WARM> = 0 and 1 (TMP95C061 datasheet p.28).
	parameter [16:0] WARM_TICKS_SHORT  = 17'd16384,   // 2^14/fc = 2.67 ms
	parameter [16:0] WARM_TICKS_LONG   = 17'd65536,   // 2^16/fc = 10.7 ms

	// Enables spent with the machine running and the wake interrupt still
	// withheld, so ngp_intc can refresh its pin history.  It needs >= 2 of its
	// own enables; at gear 4 one of those is 16 of ours.
	parameter [7:0]  ARM_TICKS         = 8'd32,

	// Enables the machine keeps running after the HALT is seen, so the CPU can
	// retire it (see the header).  The requirement is "at least one CPU state,
	// at the slowest gear" = 16 enables; 128 is eight of those, which covers a
	// retirement still waiting on a posted store, and is 20.8 us -- far below
	// anything software or a user can observe.
	parameter [7:0]  HALT_SETTLE_TICKS = 8'd128,

	// How long a wake or alarm INT0 is held.  Silicon's alarm output cannot be
	// shorter than one 32.768 kHz period (188 enables), which is also
	// comfortably more than the two ce_cpu samples ngp_intc needs at gear 4.
	parameter [7:0]  INT0_HOLD_TICKS   = 8'd192,

	// 1: IDLE is released without a warm-up.  See header choice 2.
	parameter        IDLE_SKIPS_WARMUP = 1'b1,
	// 1: pulse pwr_latch_clr at the wake edge.  See header choice 1.
	parameter        WAKE_CLEARS_LATCH = 1'b0
)
(
	input  wire        clk,              // clk_sys
	input  wire        ce,               // ce_6m144, UNGATED (see header)
	input  wire        reset,
	// 0 = the machine is OFF when /RESET releases and needs a press to run;
	// 1 = /RESET releasing starts it, and the BIOS's own reset path parks it
	// in standby.  A reset-time strap, not a runtime control (see header).
	input  wire        cold_run,

	// board
	input  wire        pwr_btn_raw_n,    // pad OR OSD entry, active low
	output wire        pwr_btn_n,        // stretched, to ngp_sysreg 0xB1/NMI

	// interrupt path
	input  wire        nmi_n_i,          // from ngp_sysreg, gated by 0xB3.2
	output wire        nmi_n,            // to t900_mcu
	input  wire        alarm_pulse,      // ngp_rtc alarm match, one clk_sys
	output wire        int0,             // to t900_mcu

	// machine status
	input  wire        halted,           // t900_cpu is in HALT
	input  wire [1:0]  haltm,            // WDMOD[3:2] from ngp_wdt
	input  wire        wdt_warm,         // WDMOD[4]

	// NV restore
	input  wire        restore_standby,  // one clk_sys pulse: a standby context
	                                     // has been fabricated

	// freeze and reporting
	output wire        main_clk_run,     // qualifies ce_6m144/_n, ce_3m072
	output wire        cpu_run,          // additionally qualifies ce_cpu
	output wire        standby,          // frozen: blank the LCD, report "off"
	output wire        nv_flush_req,     // one clk_sys pulse on standby entry
	output wire        machine_has_run,  // the CPU has been released at least once
	output wire        pwr_latch_clr,    // see header choice 1

	// savestate tap, internals word 92
	input  wire [7:0]  ss_reg_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [2:0] S_OFF      = 3'd0;
	localparam [2:0] S_WARMUP   = 3'd1;
	localparam [2:0] S_ARM      = 3'd2;
	localparam [2:0] S_RUN      = 3'd3;
	localparam [2:0] S_HALT_RUN = 3'd4;
	localparam [2:0] S_SETTLE   = 3'd5;

	// WDMOD<HALTM1,0>: 00 RUN, 01 STOP, 1x IDLE (TMP95C061 datasheet p.181).
	localparam [1:0] HALTM_RUN  = 2'b00;
	localparam [1:0] HALTM_STOP = 2'b01;

	// The block's only tap word: internals word 92, natively 64 bits because the
	// two counters do not fit in 32.
	//
	//   [2:0]    state              [37:16]  hold_cnt   (button stretcher)
	//   [3]      has_run            [54:38]  warm_cnt   (warm-up / arm /
	//                                                   halt settle)
	//   [4]      warm_ctx           [62:55]  int0_cnt   (INT0 hold)
	//   [5]      skip_warm          [15:10]  zero
	//   [6]      alarm_pend         [63]     zero
	//   [7]      wake_nmi
	//   [8]      halted_q           section 7.2's "power_off_latch" is NOT here:
	//   [9]      btn_held_q         0xB4 is ngp_sysreg's latch, saved in word 93.
	localparam [7:0] SS_PWR = 8'h00;

	reg  [2:0]  state;
	reg  [21:0] hold_cnt;
	reg  [16:0] warm_cnt;
	reg  [7:0]  int0_cnt;

	reg         has_run;
	reg         warm_ctx;      // a standby context exists -> wake by NMI
	reg         skip_warm;     // the standby was entered through IDLE
	reg         alarm_pend;    // an alarm fired while the machine was frozen
	reg         wake_nmi;      // hold the wake NMI until the CPU takes it
	reg         halted_q;
	reg         btn_held_q;
	reg         btn_pause_pend;
	reg         alarm_pause_pend;
	reg         restore_pause_pend;

	reg         nv_flush_r;
	reg         latch_clr_r;

	// A savestate pause must be invisible to the machine, so the block's own
	// timers stop with everything else.
	wire run_ce = ce && !pause_req;

	wire btn_pressed = ~pwr_btn_raw_n;
	wire btn_held    = btn_pressed || (hold_cnt != 22'd0);

	// Edges are taken at enable rate, so a button that is still held when the
	// machine goes back to standby cannot re-wake it -- the same rule the NMI
	// pin follows.
	wire wake_btn = btn_held && !btn_held_q;
	wire wake_any = wake_btn || alarm_pend;

	wire halt_edge = halted && !halted_q;

	// S_SETTLE is a running machine in every respect: the CPU has its enable
	// (that is the point) and its interrupts are still live.
	wire irq_live = (state == S_RUN) || (state == S_HALT_RUN) ||
	                (state == S_SETTLE);

	// Outputs

	assign standby         = (state == S_OFF);
	assign main_clk_run    = (state != S_OFF);
	assign cpu_run         = (state == S_ARM) || irq_live;
	assign machine_has_run = has_run;
	assign nv_flush_req    = nv_flush_r;
	assign pwr_latch_clr   = latch_clr_r;

	assign pwr_btn_n = ~btn_held;

	// Both interrupts are forced to their idle level until the machine is fully
	// awake, so the only edge ngp_intc can see is the one this block means.
	assign nmi_n = irq_live ? (nmi_n_i && !wake_nmi) : 1'b1;
	assign int0  = irq_live && (int0_cnt != 8'd0);

	// Nothing is ever in flight here; the pause tree only has to stop the
	// counters, which run_ce already does.
	assign pause_ready = 1'b1;

	wire ss_sel = (ss_reg_addr == SS_PWR);
	wire ss_wr  = ss_wren && ss_sel && pause_req;   // honoured only while paused

	// Zero filler in the tap word, and the one input this block reports on but
	// does not consume.
	wire unused_ok = &{1'b0, ss_wdata[63], ss_wdata[15:10]};

	always @(posedge clk) begin
		if (reset) begin
			// `cold_run` is sampled here and nowhere else, so it is a strap:
			// changing it while the machine runs does nothing.
			state       <= cold_run ? S_RUN : S_OFF;
			hold_cnt    <= 22'd0;
			warm_cnt    <= 17'd0;
			int0_cnt    <= 8'd0;
			has_run     <= cold_run;
			warm_ctx    <= 1'b0;
			skip_warm   <= 1'b0;
			alarm_pend  <= 1'b0;
			wake_nmi    <= 1'b0;
			halted_q    <= 1'b0;
			btn_held_q  <= 1'b0;
			btn_pause_pend     <= 1'b0;
			alarm_pause_pend   <= 1'b0;
			restore_pause_pend <= 1'b0;
			nv_flush_r  <= 1'b0;
			latch_clr_r <= 1'b0;
		end else begin
			nv_flush_r  <= 1'b0;
			latch_clr_r <= 1'b0;

			if (run_ce) begin
				halted_q   <= halted;
				btn_held_q <= btn_held;

				if (hold_cnt != 22'd0) begin
					hold_cnt <= hold_cnt - 22'd1;
				end

				if (int0_cnt != 8'd0) begin
					int0_cnt <= int0_cnt - 8'd1;
				end

				case (state)
					S_OFF: begin
						if (wake_any) begin
							state    <= S_WARMUP;
							warm_cnt <= (skip_warm && (IDLE_SKIPS_WARMUP != 0))
							            ? 17'd0
							            : (wdt_warm ? WARM_TICKS_LONG
							                        : WARM_TICKS_SHORT);
							// Only a machine that has a standby context to
							// return to is woken with an NMI; a cold one runs
							// the reset vector instead (see the header).
							if (warm_ctx && wake_btn) begin
								wake_nmi <= 1'b1;
							end
							if (WAKE_CLEARS_LATCH != 0) begin
								latch_clr_r <= 1'b1;
							end
						end
					end

					S_WARMUP: begin
						if (warm_cnt == 17'd0) begin
							state    <= S_ARM;
							warm_cnt <= {9'd0, ARM_TICKS};
						end else begin
							warm_cnt <= warm_cnt - 17'd1;
						end
					end

					S_ARM: begin
						if (warm_cnt == 17'd0) begin
							state   <= S_RUN;
							has_run <= 1'b1;
							// No seed of halted_q is needed: the CPU is still
							// parked in HALT and the top of this block has
							// already sampled that, so S_RUN starts with no
							// pending halt edge and the machine cannot fall
							// straight back into standby.
							if (alarm_pend) begin
								int0_cnt   <= INT0_HOLD_TICKS;
								alarm_pend <= 1'b0;
							end
						end else begin
							warm_cnt <= warm_cnt - 17'd1;
						end
					end

					S_RUN: begin
						// The wake NMI is released once the CPU has left HALT,
						// which is exactly "the interrupt was accepted".
						if (!halted) begin
							wake_nmi <= 1'b0;
						end

						if (halt_edge) begin
							if (haltm == HALTM_RUN) begin
								state <= S_HALT_RUN;
							end else begin
								// Not S_OFF yet: the CPU has not retired the
								// HALT (see the header).  warm_cnt is free
								// here and carries the settle, so the
								// savestate word does not grow a field.
								state    <= S_SETTLE;
								warm_cnt <= {9'd0, HALT_SETTLE_TICKS};
							end
						end
					end

					S_SETTLE: begin
						if (!halted) begin
							// An interrupt released the HALT before the
							// machine went down.  It was never asleep.
							state <= S_RUN;
						end else if (warm_cnt == 17'd0) begin
							state      <= S_OFF;
							warm_ctx   <= 1'b1;
							skip_warm  <= (haltm != HALTM_STOP);
							wake_nmi   <= 1'b0;
							nv_flush_r <= 1'b1;
						end else begin
							warm_cnt <= warm_cnt - 17'd1;
						end
					end

					S_HALT_RUN: begin
						if (!halted) begin
							state <= S_RUN;
						end
					end

					default: begin
						state <= S_OFF;
					end
				endcase
			end

			// clk_sys-domain events.  None of these may be re-gated by ce: a
			// one-cycle strobe would be dropped 7 times out of 8 at gear 0 and
			// 127 out of 128 at gear 4.  They must also not change word 92 while
			// a savestate transfer is paused, so they are latched outside the
			// serialized machine state and consumed on the first resumed clock.
			if (pause_req) begin
				if (btn_pressed) begin
					btn_pause_pend <= 1'b1;
				end
				if (alarm_pulse) begin
					alarm_pause_pend <= 1'b1;
				end
				if (restore_standby) begin
					restore_pause_pend <= 1'b1;
				end
			end else begin
				btn_pause_pend     <= 1'b0;
				alarm_pause_pend   <= 1'b0;
				restore_pause_pend <= 1'b0;

				// A level, not a strobe, but reloading it here also means a
				// press shorter than one enable period still arms the full hold.
				if (btn_pressed || btn_pause_pend) begin
					hold_cnt <= HOLD_TICKS;
				end

				if (alarm_pulse || alarm_pause_pend) begin
					if (irq_live) begin
						int0_cnt <= INT0_HOLD_TICKS;
					end else begin
						alarm_pend <= 1'b1;
					end
				end

				if (restore_standby || restore_pause_pend) begin
					warm_ctx <= 1'b1;
				end
			end

			// Savestate writes land last and only while parked.
			if (ss_wr) begin
				state      <= ss_wdata[2:0];
				has_run    <= ss_wdata[3];
				warm_ctx   <= ss_wdata[4];
				skip_warm  <= ss_wdata[5];
				alarm_pend <= ss_wdata[6];
				wake_nmi   <= ss_wdata[7];
				halted_q   <= ss_wdata[8];
				btn_held_q <= ss_wdata[9];
				hold_cnt   <= ss_wdata[37:16];
				warm_cnt   <= ss_wdata[54:38];
				int0_cnt   <= ss_wdata[62:55];
			end
		end
	end

	reg [63:0] ss_rdata_r;
	always @* begin
		if (ss_sel) begin
			ss_rdata_r = {1'b0, int0_cnt, warm_cnt, hold_cnt,
			              6'd0, btn_held_q, halted_q, wake_nmi, alarm_pend,
			              skip_warm, warm_ctx, has_run, state};
		end else begin
			ss_rdata_r = 64'd0;
		end
	end

	assign ss_rdata = ss_rdata_r;

endmodule
