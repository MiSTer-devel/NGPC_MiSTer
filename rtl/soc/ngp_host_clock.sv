// Copyright (c) 2026 Jamie Blanks

// Host wall clock: the missing half of the RTC seed.
//
// Main_MiSTer sends the RTC packet once at core start; the 60-second resend in
// user_io_poll() only runs for cores named "neogeo" or minimig, so NGPC never
// gets a second one.  ngp_setup_seed latches that single packet correctly but
// nothing advanced it, so loading a game twenty minutes after core start seeded
// a clock twenty minutes slow, compounding on every later load.  This block
// carries the single packet forward, so the seed value is "startup packet +
// elapsed".
//
// It is an interposer, not a new interface: in goes the hps_io RTC packet, out
// comes a packet in the identical 65-bit MSM6242B layout with the same bit-64
// completion-toggle convention, holding the current host time.  `emu` wires it
// between hps_io.RTC and ngp_setup_seed.hps_rtc, which needs no change.  The
// output toggle flips on a capture and once per host second, so the downstream
// shadow is never more than one second stale.
//
// hps_io exposes RTC and TIMESTAMP; this block takes RTC.  RTC is already local
// time (the HPS applies the timezone offset to it and not to TIMESTAMP, which is
// UTC Unix seconds the core is never given an offset for) and already BCD in the
// field order ngp_rtc and the BIOS want.  TIMESTAMP would need an
// epoch-to-calendar conversion, which means a sequential subtract-and-compare
// converter, since division may not appear in synthesizable code.
//
// Three constraints, each enforced structurally by an ABSENT PORT rather than a
// tie-off, because the host clock must sit outside the captured state:
//   (a) Not in savestates.  This is host time, not machine state; restoring it
//       would drag the host clock backwards and the next game load would seed
//       from the past.  The module has no savestate ports, so no walker can
//       reach it and it cannot be added to a state layout by accident.
//   (b) Not pause-gated.  k2_soc freezes ce_xt1 with machine_run once the chip
//       reports parked, so a wall clock riding the machine's 32 kHz enable would
//       lose 2.67 ms of host time on every savestate capture.  This module has
//       no pause_req input, its `ce` is tied to 1'b1 in `emu`, and it sits
//       outside ngp_mainboard entirely, so the pause tree has no path to it.
//   (c) Not machine-reset-gated.  An OSD reset or a new game load must not
//       restart the host clock, so the module has no reset input at all; its
//       power-up state comes from the `initial` block below, as
//       ngp_setup_seed's startup-packet shadow does.
//
// Drive: clk_sys (49.152 MHz) with a free-running enable, divided in two stages
// mirroring the real sub-crystal path:
//
//   ce -> [/1500] -> 32.768 kHz -> [/32768] -> 1 Hz -> BCD calendar chain
//
// 49.152 MHz is exactly 1500 x 32.768 kHz, so this is the same cadence
// ngp_clocks.ce_32k768 produces, from the same PLL, with the same accuracy.  The
// divider is duplicated rather than sourced from ngp_clocks because ce_32k768 is
// generated inside ngp_mainboard and every consumer of it inside k2_soc is gated
// by machine_run -- one careless AND away from breaking constraint (b).  Both
// divider stages are parameters so host time can be compressed for simulation;
// P_PRESCALE_LAST = 0 and P_SUBSEC_LAST = 0 give one host second per enabled
// clock.
//
// bcd_inc and month_last match ngp_rtc's definitions.  The one deliberate
// difference is `leap`: ngp_rtc keeps it as a free-running year-mod-4 counter
// because the BIOS composes and writes register 0x97, so software must own that
// field, whereas nothing writes this block and the leap state is derived from
// the BCD year instead, which cannot drift from the year it describes.  The rule
// is the same: year mod 4 == 0 is a leap year, no century exception needed,
// because the representable window is 1991-2090 (SNK SysCall reference) and its
// only century year, 2000, is a leap year.  Mod 4 of a BCD year is a pair of
// small case statements (10 mod 4 = 2) plus one 2-bit add, and every calendar
// limit is a compare against a literal BCD constant, so no division or modulus
// appears anywhere.

module ngp_host_clock
#(
	// clk_sys -> 32.768 kHz. 49.152 MHz / 1500, as ngp_clocks divides it.
	parameter [10:0] P_PRESCALE_LAST = 11'd1499,
	// 32.768 kHz -> 1 Hz, as ngp_rtc divides it.
	parameter [14:0] P_SUBSEC_LAST   = 15'd32767
)
(
	input  wire        clk,        // clk_sys
	// Free-running enable. This must NEVER be a machine enable: see (b) above.
	input  wire        ce,

	// hps_io RTC, MSM6242B layout, bit 64 the completion toggle.
	input  wire [64:0] hps_rtc,

	// The same packet shape, advanced to the current host time. Bit 64 toggles
	// on every capture and on every host second.
	output wire [64:0] host_rtc
);

	// BCD helpers -- pure functions of their arguments

	// Two-digit BCD increment with the ">9 -> +6" correction. 0x99 wraps to
	// 0x00, which is what the year stage wants.
	function automatic [7:0] bcd_inc(input [7:0] v);
		reg [3:0] lo;
		reg [3:0] hi;
	begin
		if (v[3:0] == 4'd9) begin
			lo = 4'd0;
			hi = (v[7:4] == 4'd9) ? 4'd0 : (v[7:4] + 4'd1);
		end else begin
			lo = v[3:0] + 4'd1;
			hi = v[7:4];
		end
		bcd_inc = {hi, lo};
	end
	endfunction

	// Last day of a BCD month, February taking the leap counter.
	function automatic [7:0] month_last(input [7:0] m, input [1:0] lp);
	begin
		case (m)
			8'h02:   month_last = (lp == 2'd0) ? 8'h29 : 8'h28;
			8'h04,
			8'h06,
			8'h09,
			8'h11:   month_last = 8'h30;
			default: month_last = 8'h31;
		endcase
	end
	endfunction

	// Year modulo four from two BCD digits. 10 mod 4 = 2. The explicit cases
	// also give a malformed HPS digit a deterministic zero contribution.
	function automatic [1:0] bcd_year_mod4(input [7:0] bcd_year);
		reg [1:0] tens_mod4;
		reg [1:0] ones_mod4;
	begin
		case (bcd_year[7:4])
			4'd0, 4'd2, 4'd4, 4'd6, 4'd8: tens_mod4 = 2'd0;
			4'd1, 4'd3, 4'd5, 4'd7, 4'd9: tens_mod4 = 2'd2;
			default:                      tens_mod4 = 2'd0;
		endcase
		case (bcd_year[3:0])
			4'd0, 4'd4, 4'd8: ones_mod4 = 2'd0;
			4'd1, 4'd5, 4'd9: ones_mod4 = 2'd1;
			4'd2, 4'd6:       ones_mod4 = 2'd2;
			4'd3, 4'd7:       ones_mod4 = 2'd3;
			default:          ones_mod4 = 2'd0;
		endcase
		bcd_year_mod4 = tens_mod4 + ones_mod4;
	end
	endfunction

	// State

	reg [10:0] pre_div;
	reg [14:0] sub_div;

	reg  [7:0] second;
	reg  [7:0] minute;
	reg  [7:0] hour;
	reg  [7:0] day;
	reg  [7:0] month;
	reg  [7:0] year;
	reg  [2:0] weekday;

	// hps_io's upper packet bits. Main_MiSTer sends a fixed 0x40 in the top
	// byte; nothing here interprets them, so they are carried through verbatim
	// rather than invented or dropped.
	reg [12:0] flags;

	reg        toggle_in;    // history of the INPUT completion toggle
	reg        toggle_out;   // the OUTPUT completion toggle

	// Power-up state, and the reason this module needs no reset input. The
	// values match ngp_setup_seed's startup shadow and ngp_rtc's reset state:
	// 2000-01-01 00:00:00, weekday 6 -- 2000-01-01 really was a Saturday, so a
	// core whose HPS never sends a packet still hands the BIOS a self-consistent
	// date instead of an invalid BCD digit.
	initial begin
		pre_div    = 11'd0;
		sub_div    = 15'd0;
		second     = 8'h00;
		minute     = 8'h00;
		hour       = 8'h00;
		day        = 8'h01;
		month      = 8'h01;
		year       = 8'h00;
		weekday    = 3'd6;
		flags      = {8'h40, 5'd0};
		toggle_in  = 1'b0;
		toggle_out = 1'b0;
	end

	// hps_io writes RTC in four 16-bit pieces and toggles bit 64 only once the
	// whole packet is visible, so this never captures an in-flight mixture.
	// A later packet, if one ever arrived, is authoritative and re-seeds.
	wire capture = (hps_rtc[64] != toggle_in);

	wire tick_32k = ce && (pre_div == P_PRESCALE_LAST);
	wire tick_1hz = tick_32k && (sub_div == P_SUBSEC_LAST);

	// Derived, not counted -- see the header. Leap year is "mod 4 == 0".
	wire [1:0] leap = bcd_year_mod4(year);

	// Each stage rolls over when it reaches its last value; the next stage
	// advances only on that rollover. Every comparison is against a literal BCD
	// constant, so no binary conversion appears anywhere.
	//
	// `day >= month_last(...)` rather than `==`, matching ngp_rtc: an out-of-
	// range day (say 31 in February) lands on the 1st of the next month instead
	// of running away through 0x39, 0x40 ... 0x99. The two are indistinguishable
	// for any in-range date, and the HPS only ever sends in-range dates.
	wire sec_roll  = (second == 8'h59);
	wire min_roll  = sec_roll && (minute == 8'h59);
	wire hour_roll = min_roll && (hour   == 8'h23);
	wire day_roll  = hour_roll && (day >= month_last(month, leap));
	wire mon_roll  = day_roll && (month == 8'h12);

	always @(posedge clk) begin
		if (capture) begin
			// The host just told us the time; the sub-second dividers restart so
			// the next second lands one full second after that instant. Same
			// choice ngp_rtc makes when software writes its seconds register.
			toggle_in  <= hps_rtc[64];
			toggle_out <= ~toggle_out;
			pre_div    <= 11'd0;
			sub_div    <= 15'd0;
			flags      <= hps_rtc[63:51];
			weekday    <= hps_rtc[50:48];
			year       <= hps_rtc[47:40];
			month      <= hps_rtc[39:32];
			day        <= hps_rtc[31:24];
			hour       <= hps_rtc[23:16];
			minute     <= hps_rtc[15:8];
			second     <= hps_rtc[7:0];
		end else begin
			if (ce) begin
				pre_div <= tick_32k ? 11'd0 : (pre_div + 11'd1);
			end

			if (tick_32k) begin
				sub_div <= tick_1hz ? 15'd0 : (sub_div + 15'd1);
			end

			if (tick_1hz) begin
				// Publishing on every host second keeps the downstream shadow at
				// most one second stale.
				toggle_out <= ~toggle_out;

				second <= sec_roll ? 8'h00 : bcd_inc(second);

				if (sec_roll) begin
					minute <= min_roll ? 8'h00 : bcd_inc(minute);
				end
				if (min_roll) begin
					hour <= hour_roll ? 8'h00 : bcd_inc(hour);
				end
				if (hour_roll) begin
					day     <= day_roll ? 8'h01 : bcd_inc(day);
					weekday <= (weekday == 3'd6) ? 3'd0 : (weekday + 3'd1);
				end
				if (day_roll) begin
					month <= mon_roll ? 8'h01 : bcd_inc(month);
				end
				if (mon_roll) begin
					year <= bcd_inc(year);   // 0x99 wraps to 0x00
				end
			end
		end
	end

	assign host_rtc = {toggle_out, flags, weekday,
	                   year, month, day, hour, minute, second};

endmodule
