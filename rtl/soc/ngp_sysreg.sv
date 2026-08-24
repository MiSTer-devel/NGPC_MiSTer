// Copyright (c) 2026 Jamie Blanks

// SNK system register block, internal I/O 0x000080-0x0000BF minus the RTC.
//
// Not a Toshiba peripheral: this is the glue SNK put around the TMP95C061 on
// the mainboard -- the keypad, the power button and its NMI gate, the
// power-down latches, the clock-gear selector, the sound/Z80 control bytes, and
// the one-byte mailbox the Z80 reads at its own 0x8000.  Almost every register
// is a plain latch and the behaviour lives in what the latch is wired to, so
// most of this file is a decoder and a read mux.
//
// `io_addr` is bus_addr[6:0], so the block sees 0x00-0x3F and
// `a = {1'b1, io_addr}` reconstructs the CPU-visible 0x80-0xBF address, making
// every comparison below read as the address a disassembly listing shows.
//
// Register map:
//   0x80  R/W  clock gear, bits [2:0] = 0..4 -> 6.144/3.072/1.536/0.768/0.384
//              MHz.  ngp_clocks does the division and clamps > 4 to 4.
//              Reset 0 = full speed.
//   0x81-0x8F  unmapped, read P_OPEN_BUS
//   0x90-0x9B  owned by ngp_rtc, never selected here
//   0x9C-0x9F  unmapped, read P_OPEN_BUS
//   0xA0  W    PSG port 0, 0xA1 W PSG port 1.  Strobed out on `psg_wr` when
//              the main-CPU direct path is open
//   0xA2  W    DAC latch, left channel; 0xA3 right
//   0xB0  R    buttons, 1 = pressed, bit 7 reads 0
//   0xB1  R    {5'd0, cable absent, sub-battery OK, power button}
//   0xB2  R/W  COMM status / RTS, reset 0x01
//   0xB3  R/W  bit 2 = power-button NMI enable
//   0xB4  R/W  power latch low, 0xA0 = off request.  0xB5 high byte
//   0xB6  R/W  power latch 2 low, 0x50 = off.  0xB7 high byte
//   0xB8  R/W  sound enable, whole byte 0x55 = on
//   0xB9  R/W  Z80 run, whole byte 0x55 = run
//   0xBA  S    any write pulses the Z80 NMI, data ignored
//   0xBB       unmapped
//   0xBC  R/W  Z80 comm latch, one byte, last writer wins
//   0xBD-0xBF  unmapped
//
// Bus timing: io_rdata is combinational and nothing in the read path depends on
// ce; io_wr / io_rd are one-clk_sys strobes and are not re-gated by ce, because
// the strobe is the enable.  `ce` (= ce_cpu) times only the 0xBA pulse, the one
// thing here that models the passage of time, so freezing the machine is just
// gating the enable.
//
// Three word-write behaviours that must not be special-cased.  The SNK block is
// 8 bits wide, so the CPU's 16-bit stores are already two byte cycles
// (TMP95C061 datasheet p.52 Table 3.6(2)) and a plain byte decoder gets all
// three right for free: `ldw (0xB8),0x5555` turns sound on and starts the Z80;
// `ldw (0xB8),0xAA55` leaves sound on with the Z80 halted, which is what opens
// the main-CPU direct PSG path; `ldw (0xB4),0x00A0` puts 0xA0 in 0xB4 and 0x00
// in 0xB5.  A decoder that "helpfully" recognised word writes would break all
// three.
//
// 0xB4 and 0xB6 are also word-READABLE, which most published register maps get
// wrong: the BIOS does `cp (0xB4),0x000A` at 0xFF18CB and `cp (0xB6),0x0050` at
// 0xFF115C.  0xB5 and 0xB7 are therefore real readable latches, not
// don't-cares.
//
// Choices the documents do not settle:
//   1. Readback of write-only addresses: 0xA0-0xA3 and 0xBA read P_OPEN_BUS,
//      everything else reads its latch.
//   2. RTS polarity.  0xB2 bit 0 is "RTS, 1 = off" and the reset value 0x01 is
//      itself a choice.  `link_rts_n` mirrors bit 0 straight through, so the
//      reset state leaves the active-low output deasserted.  Nothing states the
//      pad polarity.
//   3. /MPOFF polarity.  `mpoff_n` is asserted (low) exactly when 0xB6 holds the
//      documented power-down value 0x50.  The pad exists on the board but its
//      polarity is not documented.
//   4. The direct-PSG gate is `snd_en & ~z80_run`.  The alternative reading,
//      (0xB8 == 0x55 && 0xB9 == 0xAA), differs only for 0xB9 values that are
//      neither 0x55 nor 0xAA.
//   5. Simultaneous 0xBC writes from the CPU and the Z80 cannot happen on
//      hardware (one bus, one arbiter).  If both strobes land on one clock
//      here, the CPU write wins.
//   6. `z80_nmi` is one `ce` (= ce_cpu) wide.  The Z80 side must re-latch it in
//      its own enable domain, which is slower than ce_cpu at gear 0 and faster
//      at gear 4.
//
// Savestate tap: three 32-bit words, 0x00-0x02, holding every latch in the
// block.  This block owns the 0xA2/0xA3 DAC latches.

module ngp_sysreg
#(
	// Power-on value of the 0xA2/0xA3 DAC latches.  A parameter because the two
	// candidate values are audibly different and neither is documented: the DACs
	// are unsigned and the mixer computes `dac - 128` with no DC blocker on that
	// path, so 0x00 puts a -8192 step on both channels from reset until software
	// first writes them, where mid-scale 0x80 would be silent.
	parameter [7:0] DAC_RESET = 8'h00,

	// Value returned by unmapped and write-only addresses in this block.  Same
	// constant as the fabric's open bus, so one hardware measurement corrects
	// both.
	parameter [7:0] P_OPEN_BUS = 8'hFF
)
(
	input  wire        clk,            // clk_sys
	input  wire        ce,             // ce_cpu
	input  wire        reset,

	// internal I/O bus.  io_addr = bus_addr[6:0]; the parent asserts the
	// strobes only for 0x80-0xBF minus the RTC window.
	input  wire [6:0]  io_addr,
	input  wire [7:0]  io_wdata,
	input  wire        io_wr,          // one clk_sys pulse
	input  wire        io_rd,          // one clk_sys pulse
	output wire [7:0]  io_rdata,       // combinational

	// pads / board
	input  wire [6:0]  btn_n,          // active low: Up Down Left Right A B Option
	input  wire        pwr_btn_n,      // active low
	input  wire        subbatt_ok,     // board ties 1
	output wire        link_rts_n,

	// chip-internal fan-out
	output wire [2:0]  gear,             // 0x80[2:0]
	output wire        nmi_n,            // to ngp_intc, already gated by 0xB3.2
	output wire        power_off_latch,  // 0xB4 == 0xA0
	output wire        mpoff_n,          // 0xB6 == 0x50
	output wire        snd_en,           // 0xB8 == 0x55
	output wire        z80_run,          // 0xB9 == 0x55
	output wire        z80_nmi,          // one-ce pulse on any write to 0xBA
	output wire [7:0]  comm_latch,       // 0xBC, main-CPU side
	input  wire [7:0]  comm_latch_z80,   // Z80 side write data
	input  wire        comm_latch_z80_wr,
	output wire        psg_wr,           // 0xA0/0xA1 write strobe
	output wire        psg_port,         // 0 = 0xA0, 1 = 0xA1
	output wire [7:0]  psg_data,
	output wire [7:0]  dac_l,            // 0xA2
	output wire [7:0]  dac_r,            // 0xA3

	// savestate tap, words 0x00-0x02
	input  wire [7:0]  ss_reg_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [7:0] A_GEAR = 8'h80;
	localparam [7:0] A_PSG0 = 8'hA0;
	localparam [7:0] A_PSG1 = 8'hA1;
	localparam [7:0] A_DAC0 = 8'hA2;
	localparam [7:0] A_DAC1 = 8'hA3;
	localparam [7:0] A_BTN  = 8'hB0;
	localparam [7:0] A_PWR  = 8'hB1;
	localparam [7:0] A_COMM = 8'hB2;
	localparam [7:0] A_NMIG = 8'hB3;
	localparam [7:0] A_PWL0 = 8'hB4;
	localparam [7:0] A_PWH0 = 8'hB5;
	localparam [7:0] A_PWL1 = 8'hB6;
	localparam [7:0] A_PWH1 = 8'hB7;
	localparam [7:0] A_SND  = 8'hB8;
	localparam [7:0] A_ZRUN = 8'hB9;
	localparam [7:0] A_ZNMI = 8'hBA;
	localparam [7:0] A_ZCOM = 8'hBC;

	localparam [7:0] SS_MISC = 8'h00;   // gear, 0xB2, 0xB3, 0xBC
	localparam [7:0] SS_PWR  = 8'h01;   // 0xB4, 0xB5, 0xB6, 0xB7
	localparam [7:0] SS_SND  = 8'h02;   // 0xB8, 0xB9, 0xA2, 0xA3

	// Whole-byte magic values, compared as bytes and never as single bits.
	localparam [7:0] V_ON      = 8'h55;   // sound on / Z80 run
	localparam [7:0] V_PWROFF  = 8'hA0;   // 0xB4 shutdown value
	localparam [7:0] V_MPOFF   = 8'h50;   // 0xB6 shutdown value

	reg [7:0] reg_gear;
	reg [7:0] reg_b2;
	reg [7:0] reg_b3;
	reg [7:0] reg_b4;
	reg [7:0] reg_b5;
	reg [7:0] reg_b6;
	reg [7:0] reg_b7;
	reg [7:0] reg_b8;
	reg [7:0] reg_b9;
	reg [7:0] reg_bc;
	reg [7:0] reg_a2;
	reg [7:0] reg_a3;
	reg       znmi_r;

	// CPU-visible address, so every comparison below reads like a listing.
	wire [7:0] a = {1'b1, io_addr};

	// Read mux
	// 0xB0 and 0xB1 are the two live pad readbacks.  0xB0 inverts the pads:
	// 1 = pressed, bit 7 = 0 (the D-button of an attached Neo Geo pad is
	// unbonded).  Getting this backwards makes the BIOS behave as if every
	// button is stuck, which is also what open bus would do.

	reg [7:0] rdata_r;
	always @* begin
		case (a)
			A_GEAR:  rdata_r = reg_gear;
			A_BTN:   rdata_r = {1'b0, ~btn_n};
			// Bit 2 is the active-low cable-detect input. There is no link
			// transport at the emu boundary yet, so it must read 1 = unplugged.
			// SNK Gals' Fighters reads exactly this bit at ROM 0x264350 and
			// enters its LINK FAILURE path when it is left at 0.
			A_PWR:   rdata_r = {5'd0, 1'b1, subbatt_ok, pwr_btn_n};
			A_COMM:  rdata_r = reg_b2;
			A_NMIG:  rdata_r = reg_b3;
			A_PWL0:  rdata_r = reg_b4;
			A_PWH0:  rdata_r = reg_b5;
			A_PWL1:  rdata_r = reg_b6;
			A_PWH1:  rdata_r = reg_b7;
			A_SND:   rdata_r = reg_b8;
			A_ZRUN:  rdata_r = reg_b9;
			A_ZCOM:  rdata_r = reg_bc;
			default: rdata_r = P_OPEN_BUS;
		endcase
	end

	assign io_rdata = rdata_r;

	// Fan-out

	assign gear            = reg_gear[2:0];
	assign link_rts_n      = reg_b2[0];
	assign nmi_n           = pwr_btn_n | ~reg_b3[2];
	assign power_off_latch = (reg_b4 == V_PWROFF);
	assign mpoff_n         = (reg_b6 != V_MPOFF);
	assign snd_en          = (reg_b8 == V_ON);
	assign z80_run         = (reg_b9 == V_ON);
	assign z80_nmi         = znmi_r;
	assign comm_latch      = reg_bc;

	// "CPU direct mode": the main CPU may drive the PSG only while sound is
	// enabled and the Z80 is not running, or both would drive one chip.
	wire psg_open = snd_en && !z80_run;

	assign psg_wr   = io_wr && psg_open && ((a == A_PSG0) || (a == A_PSG1));
	assign psg_port = a[0];
	assign psg_data = io_wdata;

	assign dac_l = reg_a2;
	assign dac_r = reg_a3;

	// Nothing is ever in flight here.
	assign pause_ready = 1'b1;

	wire ss_wr = ss_wren && pause_req;   // honoured only while paused

	wire ss_sel_misc = (ss_reg_addr == SS_MISC);
	wire ss_sel_pwr  = (ss_reg_addr == SS_PWR);
	wire ss_sel_snd  = (ss_reg_addr == SS_SND);

	// io_rd exists for symmetry with the rest of the internal I/O bus; this
	// block has no read side effects, so observing io_rdata disturbs nothing.
	wire unused_ok = &{1'b0, io_rd};

	always @(posedge clk) begin
		if (reset) begin
			reg_gear <= 8'h00;   // full speed
			reg_b2   <= 8'h01;   // RTS off: the safe "no link partner" state
			reg_b3   <= 8'h00;   // power NMI masked until the BIOS arms it
			reg_b4   <= 8'h00;
			reg_b5   <= 8'h00;
			reg_b6   <= 8'h00;
			reg_b7   <= 8'h00;
			reg_b8   <= 8'h00;
			reg_b9   <= 8'h00;   // Z80 held in reset until software writes 0x55
			reg_bc   <= 8'h00;
			reg_a2   <= DAC_RESET;
			reg_a3   <= DAC_RESET;
			znmi_r   <= 1'b0;
		end else begin
			// The 0xBA strobe is held until the next machine enable, so a
			// write landing between two ce edges is still one full ce wide.
			if (io_wr && (a == A_ZNMI)) begin
				znmi_r <= 1'b1;
			end else if (ce) begin
				znmi_r <= 1'b0;
			end

			// The Z80 side of the mailbox.  Loses a tie with the CPU, which
			// cannot happen on hardware.
			if (comm_latch_z80_wr) begin
				reg_bc <= comm_latch_z80;
			end

			// Plain byte decoder: every 16-bit store the BIOS makes to this
			// block is already two byte cycles, so no word case exists.
			if (io_wr) begin
				case (a)
					A_GEAR:  reg_gear <= io_wdata;
					A_DAC0:  reg_a2   <= io_wdata;
					A_DAC1:  reg_a3   <= io_wdata;
					A_COMM:  reg_b2   <= io_wdata;
					A_NMIG:  reg_b3   <= io_wdata;
					A_PWL0:  reg_b4   <= io_wdata;
					A_PWH0:  reg_b5   <= io_wdata;
					A_PWL1:  reg_b6   <= io_wdata;
					A_PWH1:  reg_b7   <= io_wdata;
					A_SND:   reg_b8   <= io_wdata;
					A_ZRUN:  reg_b9   <= io_wdata;
					A_ZCOM:  reg_bc   <= io_wdata;
					default: ;   // 0xA0/0xA1 are strobes, the rest is unmapped
				endcase
			end

			// Savestate writes land last and only while parked.
			if (ss_wr) begin
				if (ss_sel_misc) begin
					reg_gear <= ss_wdata[31:24];
					reg_b2   <= ss_wdata[23:16];
					reg_b3   <= ss_wdata[15:8];
					reg_bc   <= ss_wdata[7:0];
				end
				if (ss_sel_pwr) begin
					reg_b4 <= ss_wdata[31:24];
					reg_b5 <= ss_wdata[23:16];
					reg_b6 <= ss_wdata[15:8];
					reg_b7 <= ss_wdata[7:0];
				end
				if (ss_sel_snd) begin
					reg_b8 <= ss_wdata[31:24];
					reg_b9 <= ss_wdata[23:16];
					reg_a2 <= ss_wdata[15:8];
					reg_a3 <= ss_wdata[7:0];
				end
			end
		end
	end

	reg [31:0] ss_rdata_r;
	always @* begin
		case (ss_reg_addr)
			SS_MISC: ss_rdata_r = {reg_gear, reg_b2, reg_b3, reg_bc};
			SS_PWR:  ss_rdata_r = {reg_b4, reg_b5, reg_b6, reg_b7};
			SS_SND:  ss_rdata_r = {reg_b8, reg_b9, reg_a2, reg_a3};
			default: ss_rdata_r = 32'd0;
		endcase
	end

	assign ss_rdata = ss_rdata_r;

endmodule
