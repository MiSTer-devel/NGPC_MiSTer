// Copyright (c) 2026 Jamie Blanks

// NGP real-time clock, internal I/O 0x000090-0x00009B.
//
// A 15-bit divider from the 32.768 kHz sub-crystal enable down to 1 Hz, feeding
// a six-stage BCD calendar chain with the classic ">9 -> +6" carry correction
// at every stage.  No binary/BCD conversion happens anywhere in this file.
//
//   ce (= ce_32k768) -> [/32768] -> tick_1hz
//     second -> minute -> hour -> day -> month -> year
//                                 |               |
//                                 +-> weekday     +-> leap counter (year mod 4)
//
// The block runs on the battery-backed 32 kHz domain, so it is never gated by
// the clock gear, by HALT, or by the standby freeze -- a real console keeps
// time while it is off.  It is gated by the savestate pause, so a pause is
// invisible to the machine.
//
// Register map:
//   0x90  R/W  control.  bit 1 = alarm enable.  bit 0 and bits 7-5 are written
//              in fixed BIOS idioms whose meaning is not deducible; they are
//              plain storage.  Nothing in this register gates the counter
//              chain -- see below.
//   0x91  R/W  year    BCD 00-99   (00-90 => 2000-2090, 91-99 => 1991-1999)
//   0x92  R/W  month   BCD 01-12
//   0x93  R/W  day     BCD 01-31
//   0x94  R/W  hour    BCD 00-23
//   0x95  R/W  minute  BCD 00-59
//   0x96  R/W  second  BCD 00-59
//   0x97  R/W  {2'b00, leap[1:0], 1'b0, weekday[2:0]}
//   0x98  R/W  alarm day, 0x99 hour, 0x9A minute, 0x9B weekday
//
// Three facts the register map depends on:
//   1. 0x90 bit 0 is NOT the counter enable.  The BIOS time-set path does
//      `and (0x90),0x02` before writing the time and `or (0x90),0xE0` after,
//      which leaves bit 0 clear forever; if bit 0 gated counting, the clock
//      would stop permanently the first time a user set it.  bit 1 is the alarm
//      enable -- cleared before the alarm registers change, set after.  No bit
//      of 0x90 gates anything here, and none should be added.
//   2. 0x97 is R/W, not read-only.  VECT_RTCSET composes the byte itself,
//      `weekday & 7 | ((leap & 3) << 4)` written at 0xFF136E, so the hardware's
//      fields must live exactly where the BIOS puts them or VECT_RTCGET returns
//      nonsense.
//   3. The register order is year-first (0x91 year ... 0x96 second), confirmed
//      by VECT_RTCSET at 0xFF1346-0xFF135B.
//
// The reset state is a real date rather than zeros -- 2000-01-01 00:00:00,
// weekday 6 (Saturday), leap 0 -- because the BIOS menu reads the clock every
// frame and an invalid BCD digit can hang a BCD-adjust loop.  2000-01-01 really
// was a Saturday, so the state is self-consistent.
//
// Calendar: 31/28/31/30/31/30/31/31/30/31/30/31, February 29 when `leap == 0`.
// The leap counter is year mod 4 kept as a 2-bit up-counter, so "leap year" is
// "leap == 0".  There is no century exception and none is needed: the
// representable window is 1991-2090 (SNK SysCall reference) and its only
// century year, 2000, is a leap year.  Weekday and the leap counter free-run
// off the day and year rollovers; the hardware does not derive either from the
// date.  The BIOS computes both and writes them into 0x97, which is only
// sensible if the hardware free-runs them, and its own cold init writes an
// inconsistent pair (year 1999 with weekday 0, when 1999-01-01 was a Friday)
// without caring.
//
// The alarm register assignment comes from the BIOS's own alarm programmer
// FUN_00ff1742 (colour; mono 0xFF16A5), whose validator limits 0x1F / 0x17 /
// 0x3B / 0x06 identify the four fields:
//
//   00ff1747  and (0x90),0xfd    ; DISARM first
//   00ff174b  or  (0x90),0x01
//   00ff1799  ld (0x98),A        ; QC = day of month  (skipped if 0xFF)
//   00ff17a1  ld (0x99),B        ; hour              (skipped if 0xFF)
//   00ff17a9  ld (0x9a),C        ; minute            (skipped if 0xFF)
//   00ff17b1  ld (0x9b),D        ; weekday           (skipped if 0xFF)
//   00ff17b4  or (0x90),0x02     ; ARM
//
// So 0x90 bit 1 is the arm bit, and the compare is day / hour / minute /
// weekday plus "second == 0", evaluated once a second on the 1 Hz tick and
// AFTER the counter chain has advanced -- which is why every term below is a
// `nxt_` value and not the current register.  Getting that wrong costs a whole
// minute of accuracy at every match.
//
// The output is a one-clk_sys pulse, consumed in the clk_sys domain by ngp_power
// (a wake source while the machine is frozen) and reaching the CPU through that
// block's held INT0 -- never directly as an interrupt pin, because ngp_intc's
// pin-history sampler runs on ce_cpu and a one-clk_sys pulse is invisible to it
// at every gear.
//
// Two alarm details the documents do not settle:
//   1. ALARM_FF_WILDCARD (default 1).  0xFF = "don't care" is documented for
//      the API (SNK SysCall reference, VECT_ALARMSET), but the BIOS implements
//      don't-care by not writing the register, so no 0xFF ever reaches the
//      hardware from either BIOS and there is no evidence silicon implements
//      it.  On by default for third-party code that writes the registers.
//   2. ALARM_MATCH_WEEKDAY (default 0).  0x9B is stored and read back but kept
//      out of the compare, because the BIOS implements don't-care by skipping
//      the register write and a stale 0x9B must not block an otherwise valid
//      alarm on six days out of seven.  Setting the parameter restores the
//      strict five-field compare.
//
// Two further local choices:
//   1. SEC_WRITE_RESETS_DIVIDER, default 1: writing 0x96 zeroes the sub-second
//      divider so the clock runs from the instant the BIOS finishes setting the
//      time.
//   2. The day rollover compares `day >= last day of month`, not `==`.
//      VECT_RTCSET range-checks hour, minute, year and month but not day, so
//      software can legally leave 31 in February.  With `==` such a value would
//      run away through 0x39, 0x40 ... 0x99 and wrap to 0x00; with `>=` it lands
//      on the 1st of the next month.  The two are indistinguishable for any
//      in-range date.

module ngp_rtc
#(
	// Writing the seconds register restarts the sub-second divider.
	parameter SEC_WRITE_RESETS_DIVIDER = 1'b1,
	// 0xFF in an alarm register means "don't care". See the header.
	parameter ALARM_FF_WILDCARD  = 1'b1,
	// 0x9B[2:0] participates in the compare when set. See the header.
	parameter ALARM_MATCH_WEEKDAY = 1'b0,
	// Value returned by the block's unmapped addresses.
	parameter [7:0] P_OPEN_BUS = 8'hFF
)
(
	input  wire        clk,          // clk_sys
	input  wire        ce,           // ce_32k768 (ce_xt1)
	input  wire        reset,

	// internal I/O bus.  io_addr = bus_addr[6:0]; the parent asserts the
	// strobes only for 0x90-0x9B.
	input  wire [6:0]  io_addr,
	input  wire [7:0]  io_wdata,
	input  wire        io_wr,        // one clk_sys pulse
	input  wire        io_rd,        // one clk_sys pulse
	output wire [7:0]  io_rdata,     // combinational

	// Alarm match: ONE clk_sys pulse, produced on the 1 Hz tick after the
	// counter chain has advanced. -> ngp_power (wake) -> INT0.
	output wire        alarm_pulse,

	// savestate tap, words 0x00-0x03
	input  wire [7:0]  ss_reg_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [7:0] A_CTRL  = 8'h90;
	localparam [7:0] A_YEAR  = 8'h91;
	localparam [7:0] A_MONTH = 8'h92;
	localparam [7:0] A_DAY   = 8'h93;
	localparam [7:0] A_HOUR  = 8'h94;
	localparam [7:0] A_MIN   = 8'h95;
	localparam [7:0] A_SEC   = 8'h96;
	localparam [7:0] A_WDAY  = 8'h97;
	localparam [7:0] A_AL_D  = 8'h98;
	localparam [7:0] A_AL_H  = 8'h99;
	localparam [7:0] A_AL_M  = 8'h9A;
	localparam [7:0] A_AL_W  = 8'h9B;

	localparam [7:0] SS_DATE  = 8'h00;   // 0x90, year, month, day
	localparam [7:0] SS_TIME  = 8'h01;   // hour, minute, second
	localparam [7:0] SS_WDIV  = 8'h02;   // composed 0x97 + sub-second divider
	localparam [7:0] SS_ALARM = 8'h03;   // 0x98-0x9B

	// One second of the sub-crystal.
	localparam [14:0] DIV_LAST = 15'd32767;

	reg  [14:0] div;
	reg  [7:0]  reg_ctrl;
	reg  [7:0]  year;
	reg  [7:0]  month;
	reg  [7:0]  day;
	reg  [7:0]  hour;
	reg  [7:0]  minute;
	reg  [7:0]  second;
	reg  [2:0]  weekday;
	reg  [1:0]  leap;
	reg  [7:0]  alarm_d;
	reg  [7:0]  alarm_h;
	reg  [7:0]  alarm_m;
	reg  [7:0]  alarm_w;

	// CPU-visible address, so every comparison reads like a listing.
	wire [7:0] a = {1'b1, io_addr};

	// 0x97 as software sees it.  Unused bits read 0.
	wire [7:0] wday_reg = {2'b00, leap, 1'b0, weekday};

	// BCD helpers -- pure functions of their arguments

	// Two-digit BCD increment with the ">9 -> +6" correction.  0x99 wraps to
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

	// One alarm field against one counter field. Pure function of its
	// arguments: the wildcard is a parameter, which is a constant.
	function automatic alarm_match(input [7:0] a_reg, input [7:0] cur);
	begin
		alarm_match = (a_reg == cur) ||
		              ((ALARM_FF_WILDCARD != 0) && (a_reg == 8'hFF));
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

	// Read mux

	reg [7:0] rdata_r;
	always @* begin
		case (a)
			A_CTRL:  rdata_r = reg_ctrl;
			A_YEAR:  rdata_r = year;
			A_MONTH: rdata_r = month;
			A_DAY:   rdata_r = day;
			A_HOUR:  rdata_r = hour;
			A_MIN:   rdata_r = minute;
			A_SEC:   rdata_r = second;
			A_WDAY:  rdata_r = wday_reg;
			A_AL_D:  rdata_r = alarm_d;
			A_AL_H:  rdata_r = alarm_h;
			A_AL_M:  rdata_r = alarm_m;
			A_AL_W:  rdata_r = alarm_w;
			default: rdata_r = P_OPEN_BUS;
		endcase
	end

	assign io_rdata = rdata_r;

	// Nothing is ever in flight here.
	assign pause_ready = 1'b1;

	wire ss_wr = ss_wren && pause_req;   // honoured only while paused

	wire ss_sel_date  = (ss_reg_addr == SS_DATE);
	wire ss_sel_time  = (ss_reg_addr == SS_TIME);
	wire ss_sel_wdiv  = (ss_reg_addr == SS_WDIV);
	wire ss_sel_alarm = (ss_reg_addr == SS_ALARM);

	// The chain must never move except on the 1 Hz tick, so a CPU word read
	// spanning two registers always sees a coherent pair.  Gating the tick with
	// pause_req keeps a savestate out of the machine's view.
	//
	// This is a double gate: k2_soc also withholds ce_xt1 once the whole chip
	// has reported parked.  They are not equivalent -- this one stops the clock
	// on the request, the parent's when the last block finishes draining, up to
	// a K2GE line later -- but the difference is invisible to software, because
	// the CPU parks at the next instruction boundary long before the K2GE does.
	// The local gate is kept so this block stays correct on its own terms for
	// any parent that wires `ce` straight from the crystal divider, which is
	// what standby does: ce_32k768 is never gated by main_clk_run.
	wire tick_ce  = ce && !pause_req;
	wire tick_1hz = tick_ce && (div == DIV_LAST);

	// Carry chain
	// Each stage rolls over when it hits its last value; the next stage
	// advances only on that rollover.  Every comparison is against a literal
	// BCD constant, so no binary conversion appears anywhere.

	wire sec_roll  = (second == 8'h59);
	wire min_roll  = sec_roll && (minute == 8'h59);
	wire hour_roll = min_roll && (hour   == 8'h23);
	wire day_roll  = hour_roll && (day >= month_last(month, leap));
	wire mon_roll  = day_roll && (month == 8'h12);

	// Alarm compare
	// The values the chain is ABOUT to take, so the compare sees the state the
	// software will see one clock later.  These are the same expressions the
	// counter block below assigns, written once here and used in both places.

	wire [7:0] nxt_second  = sec_roll  ? 8'h00 : bcd_inc(second);
	wire [7:0] nxt_minute  = !sec_roll ? minute
	                                   : (min_roll  ? 8'h00 : bcd_inc(minute));
	wire [7:0] nxt_hour    = !min_roll ? hour
	                                   : (hour_roll ? 8'h00 : bcd_inc(hour));
	wire [7:0] nxt_day     = !hour_roll ? day
	                                    : (day_roll ? 8'h01 : bcd_inc(day));
	wire [2:0] nxt_weekday = !hour_roll ? weekday
	                                    : ((weekday == 3'd6) ? 3'd0
	                                                         : (weekday + 3'd1));

	// 0x9B carries the weekday in its low three bits; the whole byte is
	// compared against 0xFF for the wildcard, because that is what software
	// writes when it means "any".
	wire alarm_wday_ok = (ALARM_MATCH_WEEKDAY == 0) ||
	                     (alarm_w[2:0] == nxt_weekday) ||
	                     ((ALARM_FF_WILDCARD != 0) && (alarm_w == 8'hFF));

	wire alarm_hit = reg_ctrl[1] &&                 // armed, 0x90 bit 1
	                 (nxt_second == 8'h00) &&
	                 alarm_match(alarm_m, nxt_minute) &&
	                 alarm_match(alarm_h, nxt_hour) &&
	                 alarm_match(alarm_d, nxt_day) &&
	                 alarm_wday_ok;

	reg alarm_pulse_r;
	assign alarm_pulse = alarm_pulse_r;

	// io_rd exists for symmetry with the rest of the internal I/O bus; this
	// block has no read side effects, so observing io_rdata disturbs nothing.
	wire unused_ok = &{1'b0, io_rd};

	always @(posedge clk) begin
		if (reset) begin
			div      <= 15'd0;
			reg_ctrl <= 8'h00;   // alarm disarmed
			year     <= 8'h00;   // 2000
			month    <= 8'h01;
			day      <= 8'h01;
			hour     <= 8'h00;
			minute   <= 8'h00;
			second   <= 8'h00;
			weekday  <= 3'd6;    // 2000-01-01 was a Saturday
			leap     <= 2'd0;    // and a leap year
			alarm_d  <= 8'h00;
			alarm_h  <= 8'h00;
			alarm_m  <= 8'h00;
			alarm_w  <= 8'h00;
			alarm_pulse_r <= 1'b0;
		end else begin
			// One clk_sys wide, raised on the tick the match happens on. NOT
			// re-gated by `ce`: the consumer is in the clk_sys domain and a
			// pulse narrower than the enable period is exactly what it wants.
			alarm_pulse_r <= tick_1hz && alarm_hit;

			if (tick_ce) begin
				div <= (div == DIV_LAST) ? 15'd0 : (div + 15'd1);
			end

			// The five `nxt_` wires are the compare's inputs as well, so the
			// alarm can never disagree with the chain about what the time is
			// about to be.
			if (tick_1hz) begin
				second  <= nxt_second;
				minute  <= nxt_minute;
				hour    <= nxt_hour;
				day     <= nxt_day;
				weekday <= nxt_weekday;

				if (day_roll) begin
					month <= mon_roll ? 8'h01 : bcd_inc(month);
				end
				if (mon_roll) begin
					year <= bcd_inc(year);   // 0x99 wraps to 0x00
					leap <= leap + 2'd1;     // free-running year mod 4
				end
			end

			// CPU writes.  One strobe, one path.  A write on the same clock as
			// a tick wins, because software asked for that value.
			if (io_wr) begin
				case (a)
					A_CTRL:  reg_ctrl <= io_wdata;
					A_YEAR:  year     <= io_wdata;
					A_MONTH: month    <= io_wdata;
					A_DAY:   day      <= io_wdata;
					A_HOUR:  hour     <= io_wdata;
					A_MIN:   minute   <= io_wdata;
					A_SEC:   second   <= io_wdata;
					A_WDAY:  begin
						weekday <= io_wdata[2:0];
						leap    <= io_wdata[5:4];
					end
					A_AL_D:  alarm_d <= io_wdata;
					A_AL_H:  alarm_h <= io_wdata;
					A_AL_M:  alarm_m <= io_wdata;
					A_AL_W:  alarm_w <= io_wdata;
					default: ;
				endcase

				if (SEC_WRITE_RESETS_DIVIDER && (a == A_SEC)) begin
					div <= 15'd0;
				end
			end

			// Savestate writes land last and only while parked.
			if (ss_wr) begin
				if (ss_sel_date) begin
					reg_ctrl <= ss_wdata[31:24];
					year     <= ss_wdata[23:16];
					month    <= ss_wdata[15:8];
					day      <= ss_wdata[7:0];
				end
				if (ss_sel_time) begin
					hour   <= ss_wdata[23:16];
					minute <= ss_wdata[15:8];
					second <= ss_wdata[7:0];
				end
				if (ss_sel_wdiv) begin
					// weekday and leap are owned by the composed 0x97 byte, so
					// exactly one word restores them.
					weekday <= ss_wdata[26:24];
					leap    <= ss_wdata[29:28];
					div     <= ss_wdata[14:0];
				end
				if (ss_sel_alarm) begin
					alarm_d <= ss_wdata[31:24];
					alarm_h <= ss_wdata[23:16];
					alarm_m <= ss_wdata[15:8];
					alarm_w <= ss_wdata[7:0];
				end
			end
		end
	end

	reg [31:0] ss_rdata_r;
	always @* begin
		case (ss_reg_addr)
			SS_DATE:  ss_rdata_r = {reg_ctrl, year, month, day};
			SS_TIME:  ss_rdata_r = {8'h00, hour, minute, second};
			SS_WDIV:  ss_rdata_r = {wday_reg, 8'h00, 1'b0, div};
			SS_ALARM: ss_rdata_r = {alarm_d, alarm_h, alarm_m, alarm_w};
			default:  ss_rdata_r = 32'd0;
		endcase
	end

	assign ss_rdata = ss_rdata_r;

endmodule
