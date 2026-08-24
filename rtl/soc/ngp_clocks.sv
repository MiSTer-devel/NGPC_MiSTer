// Copyright (c) 2026 Jamie Blanks

// NGP clock enable tree.
//
// clk_sys is 49.152 MHz, exactly 8 x 6.144 MHz (the main crystal on the real
// board) and exactly 1500 x 32.768 kHz (the sub crystal). Every machine domain
// is expressed as a clock enable on clk_sys:
//
//   ce_6m144   1-of-8     6.144 MHz oscillator / K2GE dot enable
//   ce_3m072   1-of-16    Z80 and T6W28 enable
//   ce_cpu     a subset of ce_6m144, thinned by the TLCS clock gear (register
//              0x80; values 0-4 give 6.144 MHz / 2^gear). This is the
//              documented CPU-clock rate geared peripherals consume.
//   ce_t900    TLCS execution-state enable. A Toshiba state is two oscillator
//              clocks, so this is ce_cpu / 2 and drives the sequencer and bus.
//   ce_32k768  1-of-1500 RTC tick; runs regardless of gear or CPU halt, as the
//              battery-backed 32 kHz domain does on hardware.
//
// The divider phase is machine state. div16 places ce_6m144 and ce_3m072,
// gear_count decides which of those ticks becomes a ce_cpu, and div32k places
// the 32 kHz tick relative to both; 1500 is not a multiple of 16, so that
// relationship walks. Two machines with identical registers but different
// divider phases interleave their enables differently and diverge.
//
// So the module keeps a parked copy of the three counters. park_div16 /
// park_gear / park_div32k track the live counters exactly while the machine
// runs and FREEZE when it parks, so the copy holds the phase the machine
// stopped on, does not move for the whole of a savestate capture, and is what
// the savestate word carries. The machine enables are withheld for as long as
// the machine is parked and the live counters are reloaded from the parked copy
// on release, which makes the pause transparent: the first enable after a
// resume is exactly the enable the freeze withheld, whether the freeze lasted
// two cycles or a million.
module ngp_clocks
#(
	// Savestate internals word. k2_soc answers 0-95, ngp_cart 96-103, and
	// 104-111 are spare. This module claims exactly one of them.
	parameter [9:0] SS_BASE = 10'd104
)
(
	input  wire       clk_sys,
	input  wire       reset,
	input  wire [2:0] gear,

	// The board's pause tree. `pause_req` asks the machine to park,
	// `phase_capture` is the earlier SoC-stop boundary, and `pause_ready` is the
	// whole board (cartridge included) reporting parked. This module never
	// withholds readiness -- it has nothing to drain. It freezes the phase at the
	// SoC boundary but accepts a restored phase only after the final board
	// handshake authorizes the state transfer.
	input  wire       pause_req,
	input  wire       phase_capture,
	input  wire       pause_ready,
	input  wire       restore_hold,

	output reg        ce_6m144,
	output reg        ce_3m072,
	output reg        ce_cpu,
	output reg        ce_t900,
	output reg        ce_32k768,

	// Savestate internals tap, one 64-bit word at SS_BASE. Packed so a state
	// file can be decoded by hand:
	//
	//   [3:0]    park_div16    the 1-of-16 master divider
	//   [7:4]    park_gear[3:0]  clock-gear thinning counter, low bits
	//   [18:8]   park_div32k     the 1-of-1500 sub-crystal divider
	//   [19]     park_gear[4]    clock-gear thinning counter, high bit
	//   [63:20]  zero
	//
	// i.e. byte 0 is {gear, div16} and bytes 1-2 are div32k in little-endian
	// order, 20 bits used of 64. Bit 19 extends the original four-bit gear
	// phase without moving any established field in the word.
	input  wire [9:0]  ss_bus_adr,
	input  wire [63:0] ss_bus_din,
	input  wire        ss_bus_wren,
	output wire [63:0] ss_bus_dout
);

	reg  [3:0]  div16;
	reg  [4:0]  gear_count;
	reg  [10:0] div32k;

	// The phase the machine is parked on. See the header.
	reg  [3:0]  park_div16;
	reg  [4:0]  park_gear;
	reg  [10:0] park_div32k;

	// `parked_q` latches the moment the SoC reports stopped. The final board
	// handshake can follow later because the cartridge deliberately observes
	// four stable full-rate clocks before it reports drained. `hold_q` gives
	// the release its two-cycle realign window:
	//
	//   cycle r     pause_req falls. hold_q is still 2, so the machine enables
	//               produced at the previous edge were already withheld and no
	//               free-phase enable can leak into the restored run.
	//   edge r->r+1 the counters are reloaded from the parked copy.
	//   cycle r+2   hold_q is 0 and the first enable of the resumed machine is
	//               computed from the reloaded phase.
	//
	// Two cycles is the minimum that works: the enables are registered, so an
	// enable presented at cycle n was decided at the edge into n.
	reg         parked_q;
	reg  [1:0]  hold_q;

	// Gear values above 4 do not exist on hardware; treat them as 4.
	reg [3:0] gear_mask;
	reg [4:0] state_mask;
	always_comb begin
		case (gear)
			3'd0: begin
				gear_mask  = 4'd0;
				state_mask = 5'd1;
			end
			3'd1: begin
				gear_mask  = 4'd1;
				state_mask = 5'd3;
			end
			3'd2: begin
				gear_mask  = 4'd3;
				state_mask = 5'd7;
			end
			3'd3: begin
				gear_mask  = 4'd7;
				state_mask = 5'd15;
			end
			default: begin
				gear_mask  = 4'd15;
				state_mask = 5'd31;
			end
		endcase
	end

	wire tick_6m144 = (div16[2:0] == 3'd7);
	wire tick_32k   = (div32k == 11'd1499);

	// The direct phase-capture term matters on the first stopped cycle:
	// `phase_capture` is generated from state that changes on this same clock,
	// so `parked_q` cannot reflect it until the following edge. The parked copy
	// already contains the next phase at that point; withholding its tracker on
	// the first observable capture cycle preserves exactly the enable the SoC
	// stopped before consuming.
	wire machine_hold = restore_hold ||
	                    (pause_req && (phase_capture || parked_q)) ||
	                    (hold_q == 2'd2);

	wire ss_hit = (ss_bus_adr == SS_BASE);
	wire ss_wr  = ss_bus_wren && ss_hit && (pause_ready || restore_hold);

	always @(posedge clk_sys) begin
		if (reset) begin
			div16         <= 4'd0;
			gear_count    <= 5'd0;
			div32k        <= 11'd0;
			park_div16    <= 4'd0;
			park_gear     <= 5'd0;
			park_div32k   <= 11'd0;
			parked_q      <= 1'b0;
			hold_q        <= 2'd0;
			ce_6m144      <= 1'b0;
			ce_3m072      <= 1'b0;
			ce_cpu        <= 1'b0;
			ce_t900       <= 1'b0;
			ce_32k768     <= 1'b0;
		end else begin
			// The machine dividers.
			div16 <= div16 + 4'd1;

			if (tick_6m144) begin
				gear_count <= gear_count + 5'd1;
			end

			if (tick_32k) div32k <= 11'd0;
			else          div32k <= div32k + 11'd1;

			// The machine enables. Both geared enables coincide with ce_6m144.
			// ce_cpu preserves the documented 6.144 MHz / gear clock for
			// serial and peripheral timing; ce_t900 advances the execution
			// engine once per two oscillator clocks, as Toshiba defines one
			// state. They must stay separate: halving ce_cpu would also halve
			// the serial baud rate.
			ce_6m144   <= !machine_hold && tick_6m144;
			ce_3m072   <= !machine_hold && (div16 == 4'd15);
			ce_cpu     <= !machine_hold && tick_6m144 &&
			              ((gear_count[3:0] & gear_mask) == 4'd0);
			ce_t900    <= !machine_hold && tick_6m144 &&
			              ((gear_count & state_mask) == 5'd0);
			ce_32k768  <= !machine_hold && tick_32k;

			// The parked phase tracks the counters cycle for cycle while the
			// machine runs, so the moment the hold starts it already holds the
			// phase of the enable the hold withheld.
			if (!machine_hold) begin
				park_div16  <= div16 + 4'd1;
				park_gear   <= tick_6m144 ? (gear_count + 5'd1) : gear_count;
				park_div32k <= tick_32k ? 11'd0 : (div32k + 11'd1);
			end

			// A restore overwrites it. The engine only writes while it holds
			// the machine, so this can never fight the tracking above.
			if (ss_wr) begin
				park_div16  <= ss_bus_din[3:0];
				park_gear   <= {ss_bus_din[19], ss_bus_din[7:4]};
				park_div32k <= ss_bus_din[18:8];
			end

			// Park and release.
			if (phase_capture) parked_q <= 1'b1;

			if (pause_req) begin
				hold_q <= parked_q ? 2'd2 : 2'd0;
			end else if (hold_q != 2'd0) begin
				hold_q <= hold_q - 2'd1;
				if (hold_q == 2'd2) begin
					// Later assignment wins over the free-running one above:
					// this is the reload, and it is the only place the live
					// counters ever move by anything but one.
					div16      <= park_div16;
					gear_count <= park_gear;
					div32k     <= park_div32k;
				end else begin
					parked_q <= 1'b0;
				end
			end
		end
	end

	assign ss_bus_dout = ss_hit ? {44'd0, park_gear[4], park_div32k,
	                               park_gear[3:0], park_div16}
	                            : 64'd0;

	// 20 bits of a 64-bit word are used; the rest is reserved and must read
	// back as zero so a later claim on it cannot be mistaken for old data.
	wire unused_ok = &{1'b0, ss_bus_din[63:20], 1'b0};

endmodule
