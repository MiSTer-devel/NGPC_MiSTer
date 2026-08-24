// Copyright (c) 2026 Jamie Blanks

// k2_soc -- the SNK K2-CHIP (T3W2xAF, 144-QFP), presented at its pads.
//
// This file is assembly: it holds no register and no state machine of its own.
// Every behaviour lives in a submodule with its own header. What this level
// owns is the wiring, the savestate address-space adapter and the pause chain.
//
// Deviations from the real pad list, all deliberate:
//   1. `mono_strap` is an added input: the fabric needs a runtime image select
//      and the board owns the strap, so the signal has to cross this boundary.
//      P_MONO is the K1-CHIP build switch and forces the mono image on its own,
//      because a K1 part has no colour BIOS.
//   2. `bios_wr / bios_sel / bios_addr / bios_data` are added inputs. On
//      silicon the BIOS is mask ROM with no write port; our images arrive over
//      ioctl and reach the block RAM through its port B.
//   3. `subbatt_ok` has no pad and is tied to 1 (healthy sub-battery).
//   4. `link_txd` has no output enable. Its value comes from Port 8's resolved
//      P80 function/direction pair, so GPIO input and the serial open-drain
//      release both become the pulled-up idle mark.
//   5. The two PSG output pins become one signed 16-bit audio_l / audio_r pair.
//      On silicon the T6W28's analog outputs and the two R-2R DACs are summed
//      by the amplifier on the board; we mix digitally instead, inside ngp_snd
//      with the PSG, because that is where the band limiting has to be.
//      dac_l / dac_r stay exported -- they are real digital-side latches, and
//      they feed that mixer.
//   6. `palette_frame_boundary` is an OSD service input, not a K2-CHIP pin. It
//      selects a display-only K2GE palette bank at native frame edges; CPU
//      palette writes and readback stay immediate.
//
// The external bus strobes. The fabric hands out ext_rd / ext_wr as one-clk_sys
// T2 strobes, the right shape for a register slave but not for a flash die,
// which latches a write on the RISING edge of nWE while nCE is low. So the
// cartridge strobes here are CYCLE LEVELS derived from bus_active and bus_we --
// asserted from T1, released when the cycle retires -- putting the latching
// edge at the end of the bus cycle. A real TLCS-900/H asserts /WR partway into
// the cycle; the two are indistinguishable to the die because address and data
// are stable across the whole cycle.
//
// Savestate internals adapter
//
// The chip presents one 10-bit / 64-bit internals slice; its blocks do not.
// t900_cpu and t900_mcu expose 32-bit taps with 8-bit local addresses, ngp_rtc
// and ngp_sysreg expose 32-bit taps, and k2ge is natively 64-bit. savestates.sv
// latches BUS_Dout exactly one clk_sys after it moves BUS_Adr, so the read path
// must be purely combinational and one tap port can answer only one slot per
// cycle. The adapter therefore pairs taps ACROSS blocks, never two slots of the
// same block:
//
//   word 0-47    [31:0]  t900_cpu, dense: words 0-19 -> tap 0x00-0x13 (the
//                        register file), 20-22 -> 0x20-0x22 (PC, SR, halt),
//                        23-34 -> 0x30-0x3B (micro-DMA). 35-47 read 0.
//                [63:32] t900_mcu, identity: word n -> tap n, 0x00-0x2F.
//   word 48-63   [31:0]  ngp_snd, one tap per word: 48-56 -> tap 0x00-0x08 (the
//                        block's control word and the Z80's eight architectural
//                        words), 57-62 -> tap 0x10-0x15 (the T6W28), 63
//                        reserved and reads zero.
//                [63:32] ngp_mixer's ten addresses; reserved ones read zero.
//   word 64-87   k2ge, native 64-bit, local ss_addr = word - 64.
//   word 88-91   ngp_rtc taps 0-3 in [31:0]: date, time, weekday plus
//                sub-second divider, alarm.
//   word 92      ngp_power, native 64-bit, local address 0x00: the state
//                machine, warm-up counter, button stretcher and INT0 hold. The
//                power LATCHES (0xB4-0xB7) are ngp_sysreg's and ride in 93.
//   word 93-95   ngp_sysreg taps 0-2 in [31:0].
//   word 96-103  cartridge -- answered at the BOARD, not here.
//
// The read mux is a priority chain of range compares, never a wired-OR: a stuck
// slave must fail visibly rather than corrupt everyone else's word.
//
// The pause chain and the freeze
//
// The chain is ORDERED, not a parallel AND, so the reported ready cannot bounce
// while savestates.sv counts its 16 settle cycles. The CPU is the only master
// and drains first, then the sound block at a Z80 instruction boundary, then
// the K2GE at a native frame boundary. The fabric, the MCU shell, ngp_sysreg
// and ngp_rtc hold nothing in flight the CPU has not already stopped feeding,
// so they take the raw request and only contribute a ready bit.
//
// Draining is half a pause; the other half is withholding the enables, which
// happens here rather than in ngp_clocks -- a gated generator would restart its
// divider phase on every resume and the freeze would perturb the machine. Two
// further points are load bearing. `machine_run` falls on the COMPLETED
// handshake, not the bare request, because a block with a multi-cycle
// transaction in flight needs its enable in order to drain and freezing on the
// request would deadlock the pause; there is no combinational loop, since every
// pause_ready here is a function of pause_req and of registers, never of `ce`.
// And the savestate paths are deliberately not gated: internals writes and
// memory-tap accesses run on plain clk_sys (the block RAM wrappers have no
// enable either), so the walker still works on a chip whose enables are all
// held low. The board's CRT presenter stays outside the gate for the same
// reason.
//
// ngp_power gates the same enables at the same point of use for a different
// reason, with `main_clk_run` (STOP/IDLE: the main oscillator is stopped) and
// `cpu_run` (the CPU additionally held through warm-up and arming):
//
//     ce_x1 / ce_x1_div2  &  machine_run  &  main_clk_run
//     ce_cpu / ce_t900    &  machine_run  &  main_clk_run & cpu_run
//     ce_xt1              &  machine_run
//
// ce_xt1 takes the pause gate but never the standby gate: the 32.768 kHz domain
// is battery backed and keeps time while the console is "off", but it must not
// advance across a savestate pause or the captured time would depend on how
// long the transfer took. ngp_power's own `ce` is the ungated ce_x1 off the
// pad, because its warm-up timer and button stretcher are what end the freeze;
// it gates itself on pause_req internally, so a savestate still stops it.
//
// A frozen machine cannot drain, so a frozen machine reports itself drained.
// Four blocks here need enables to reach their parked state -- t900_cpu must
// step from S_HALT to S_PAUSE, k2ge must reach a frame boundary, ngp_sio must
// finish a byte, ngp_adc a conversion -- and standby is exactly the condition
// that withholds those enables, so in standby pause_ready could never rise.
// While ngp_power withholds an enable this chip needs, the chip therefore
// reports parked without waiting for the chain: the CPU is parked in HALT and
// nothing can start anything, because the only bus master has no enable. The
// cost is that a capture taken in standby catches the K2GE mid-line and the CPU
// in S_HALT, so it cannot be restored onto a standby machine -- which it never
// has to be, because savestates.sv resets every chip before broadcasting the
// internals and a reset machine runs.
//
// `cold_run` is tied high, so releasing /RESET starts the machine, which is
// what the pad means on a TLCS-900/H. The BIOS reset path at 0xFF204A is
// written to end by jumping into the power-down routine at 0xFF1074 and
// halting, so a machine that did not run on /RESET release could never execute
// that tail. Driving it low restores a two-press cold start.
//
// `mcu.wdt_reset` is the WDMOD<RESCR>-qualified watchdog overflow, which on
// silicon asserts an internal reset. No internal reset controller is built:
// RESCR is WDMOD bit 1 and both BIOSes write WDMOD as 0x80 / 0x04 / 0x14 /
// 0xF0, bit 1 clear in all of them, so nothing here ever arms it, and building
// one would mean inventing its scope. A chip reset plainly must not reset
// ngp_rtc or the SNK latches (the battery-backed domain that survives a power
// cycle) and must not reset ngp_power itself, or a watchdog bite would look
// like a cold start; which of the remaining blocks it touches is undocumented.

module k2_soc
#(
	// 1 = K1-CHIP build (monochrome Neo Geo Pocket). Forces the mono BIOS image
	// and selects the K1GE display path.
	parameter        P_MONO     = 1'b0,
	// Value returned by any read nothing sources.
	parameter [7:0]  P_OPEN_BUS = 8'hFF,
	// Mixer trims, 4 = nominal and each step is 6 dB. Parameters rather than
	// pads because the volume control on the real board is an analog pot after
	// the amplifier, so there is no digital net to model.
	parameter [2:0]  P_MIX_PSG_GAIN = 3'd4,
	parameter [2:0]  P_MIX_DAC_GAIN = 3'd4,
	// 1 = the K2GE's Vint PAD is active low, so its falling edge is the start of
	// vertical blanking, which is the edge the BIOS's T4MOD<CAP12M> = 10
	// selects. See the pad bonding below. 0 is a defect, kept for A/B testing.
	parameter        P_VINT_PAD_ACTIVE_LOW = 1'b1
)
(
	// ---- clock and crystal pads ------------------------------------------
	input  wire        clk_sys,      // 49.152 MHz fabric clock
	input  wire        reset,        // /RESET pad, synchronous, active high
	input  wire        ce_x1,        // Y1  6.144 MHz enable
	input  wire        ce_x1_div2,   // 3.072 MHz, Z80 + PSG
	input  wire        ce_cpu,       // 6.144 MHz / gear CPU-clock enable
	input  wire        ce_t900,      // fosc/2 / gear TLCS state enable
	input  wire        ce_xt1,       // Y2  32.768 kHz enable
	output wire [2:0]  gear,         // SNK register 0x80, back to ngp_clocks

	// ---- image strap and BIOS loader (deviations 1 and 2) -----------------
	input  wire        mono_strap,   // 0 = colour image, 1 = mono; reset strap
	input  wire        palette_frame_boundary, // display-only OSD alternative
	input  wire        bios_wr,
	input  wire        bios_sel,     // which image the loader is filling
	input  wire [14:0] bios_addr,    // word address
	input  wire [15:0] bios_data,

	// ---- MiSTer cheat record stream ----------------------------------------
	// These are service-side signals, not physical K2 pads. The engine below
	// interposes only on the CPU's completed read-data mux.
	input  wire         cheat_load_begin,
	input  wire         cheat_invalidate,
	input  wire         cheat_commit_req,
	output wire         cheat_commit_done,
	input  wire [128:0] cheat_code,

	// ---- cartridge bus ----------------------------------------------------
	output wire [20:0] cart_a,       // A20..A0
	input  wire [7:0]  cart_d_i,     // resolved cart-edge value incl. pull-ups
	input  wire        cart_rd_ready, // selected cartridge read data is valid
	output wire [7:0]  cart_d_o,
	output wire        cart_d_oe,    // 1 = the SoC is driving cart_d_o
	output wire        cart_nce0,    // CS0 window 0x200000-0x3FFFFF
	output wire        cart_nce1,    // CS1 window 0x800000-0x9FFFFF
	output wire        cart_noe,
	output wire        cart_nwe,

	// ---- LCD port ----------------------------------------------------------
	output wire [3:0]  lcd_r,
	output wire [3:0]  lcd_g,
	output wire [3:0]  lcd_b,
	output wire        lcd_dclk_ce,
	output wire        lcd_de,
	output wire        lcd_hs,
	output wire        lcd_vs,
	output wire        lcd_lp,
	output wire        lcd_sp,

	// ---- audio pads --------------------------------------------------------
	// dac_l / dac_r are the real R-2R DAC pins and stay exported. The PSG's two
	// output pins do NOT: see deviation 5.
	output wire [7:0]  dac_l,        // 8-bit unsigned R-2R DAC, SNK 0xA2
	output wire [7:0]  dac_r,        // SNK 0xA3
	output wire signed [15:0] audio_l,   // mixed PSG + DAC, signed 16-bit PCM
	output wire signed [15:0] audio_r,

	// ---- panel, power and misc pads ---------------------------------------
	input  wire [6:0]  btn_n,        // {Option,B,A,Right,Left,Down,Up}, low = pressed
	input  wire        pwr_btn_n,    // POWER, active low
	output wire        mpoff_n,      // /MPOFF main-power-off latch
	output wire        bios_setup_ready, // cold init reached power-off HALT
	output wire        bios_mono_active, // model/BIOS strap latched at reset
	input  wire [9:0]  an0,          // AN0 battery sense, pre-digitised
	output wire        led,          // red LED D10 drive
	input  wire        inp0,         // reserved input pad, read at 0x87FE D6

	// ---- link port ---------------------------------------------------------
	output wire        link_txd,
	input  wire        link_rxd,
	output wire        link_rts_n,   // SNK register 0xB2 bit 0
	input  wire        link_cts_n,   // CTS0 pin

	// ---- savestate internals slice + memory taps --------------------------
	input  wire [9:0]  ss_bus_adr,
	input  wire [63:0] ss_bus_din,
	input  wire        ss_bus_wren,
	output wire [63:0] ss_bus_dout,
	input  wire [1:0]  ss_mem_type,  // 0 work RAM, 1 shared RAM, 2 video
	input  wire        ss_mem_active,
	input  wire [13:0] ss_mem_addr,
	input  wire [7:0]  ss_mem_wdata,
	input  wire        ss_mem_wren,
	input  wire        ss_mem_rden,
	output wire [7:0]  ss_mem_rdata,
	// High for the whole internals broadcast of a restore (savestates.sv's own
	// `loading_savestate`). Only `ngp_snd` consumes it, to hold the Z80 out of
	// reset while its register words are written; see that module's header.
	input  wire        loading_savestate,
	input  wire        pause_req,
	output wire        pause_ready
);

	// CPU bus (the t900_biu contract; see rtl/t900/t900_biu.sv)

	wire [23:0] plan_addr;
	wire        bus_width8;
	wire        bus_req;
	wire        bus_we;
	wire [23:0] bus_addr;
	wire [1:0]  bus_be;
	wire [15:0] bus_wdata;
	wire [15:0] bus_rdata_raw;
	wire [15:0] bus_rdata;
	wire        bus_rdy;
	wire        cheat_available;

	// Fabric fan-out

	wire [6:0]  io_addr;
	wire [7:0]  io_wdata;
	wire        sfr_wr,    sfr_rd;
	wire [7:0]  sfr_rdata;
	wire        sysreg_wr, sysreg_rd;
	wire [7:0]  sysreg_rdata;
	wire        rtc_wr,    rtc_rd;
	wire [7:0]  rtc_rdata;

	wire [13:0] gfx_addr;
	wire [15:0] gfx_wdata;
	wire        gfx_cs, gfx_rd;
	wire [2:0]  gfx_wait;   // K2GE drawing-period VRAM contention
	wire        bus_wait_gfx; // that contention, as a per-state level
	wire [1:0]  gfx_we;
	wire [15:0] gfx_rdata;

	wire [23:0] ext_addr;
	wire [7:0]  ext_wdata;
	wire        ext_oe, ext_rd, ext_wr;

	wire        bus_active;
	wire        csc_plan_width8;
	wire [2:0]  csc_waits;

	// MCU shell fan-out

	wire        int_req;
	wire [2:0]  int_level;
	wire [7:0]  int_vector;
	// The identity half of the interrupt handshake (t900_cpu's header,
	// "THE ACKNOWLEDGE CARRIES AN IDENTITY").
	wire [4:0]  int_id;
	wire        int_ack;
	wire [4:0]  int_ack_id;
	wire [3:0]  dma_req, dma_ack, dma_end;
	wire        halt_release;
	wire        halted;
	wire [1:0]  haltm;
	wire        wdtout_n;
	wire        wdt_reset;
	wire [3:0]  plan_cs_n;
	wire [3:0]  cs_n;
	wire        cart_selected = !(&cs_n[1:0]);
	wire        to3_z80;

	wire        txd0, txd0_oe;
	wire        sclk0_out, sclk0_oe;
	wire        txd1, txd1_oe;
	wire        sclk1_out, sclk1_oe;

	wire [7:0]  p1_out, p1_oe;
	wire [7:0]  p2_out;
	wire [7:0]  p5_out, p5_oe;
	wire [7:0]  p6_out;
	wire [7:0]  p7_out, p7_oe;
	wire [7:0]  p8_out, p8_oe;
	wire [3:0]  pa_out, pa_oe;
	wire [7:0]  pb_out, pb_oe;

	// SNK block and video fan-out

	// The NMI path runs pad -> power (stretch) -> sysreg (0xB1 + the 0xB3.2
	// gate) -> power (wake mux) -> mcu, so it needs three names.
	wire        pwr_btn_held_n;    // stretched button, to ngp_sysreg 0xB1
	wire        nmi_n_gated;       // ngp_sysreg's output, gated by 0xB3.2
	wire        nmi_n;             // ngp_power's output, to the MCU

	wire        main_clk_run;
	wire        cpu_run;
	wire        standby;
	wire        nv_flush_req;
	wire        machine_has_run;
	wire        pwr_latch_clr;
	wire        alarm_pulse;
	wire        wdt_warm;

	wire        power_off_latch;
	wire        snd_en, z80_run, z80_nmi;
	wire [7:0]  comm_latch;
	wire [7:0]  comm_latch_z80;
	wire        comm_latch_z80_wr;
	wire        psg_wr, psg_port;
	wire [7:0]  psg_data;

	// Z80 side of the shared RAM, k2_soc_fabric's `zram` port B.
	wire [11:0] z80_ram_addr;
	wire        z80_ram_wr;
	wire [7:0]  z80_ram_wdata;
	wire [7:0]  z80_ram_rdata;

	// The chip-internal ACTIVE-HIGH V-blank event. The PAD it drives is active
	// low; the inversion lives at the pin bonding further down. Do not invert
	// here and do not invert inside k2ge_vtimer: `vint_q` is a savestate bit and
	// its polarity is part of the state format.
	wire        vint;               // -> INT4 pad, through vint_pad
	wire        ti0;                // K2GE HINT alias -> PA1/TI0

	// Savestate adapter and pause chain nets (driven further down)

	wire [31:0] cpu_ss_rdata;
	wire [31:0] mcu_ss_rdata;
	wire [31:0] rtc_ss_rdata;
	wire [31:0] sysreg_ss_rdata;
	wire [31:0] snd_ss_rdata;
	wire [31:0] snd_mix_ss_rdata;
	wire [63:0] gfx_ss_rdata;

	reg  [7:0]  cpu_ss_addr;
	reg         cpu_slot;
	reg  [7:0]  snd_ss_addr;
	wire [7:0]  mcu_ss_addr;
	wire [7:0]  rtc_ss_addr;
	wire [7:0]  sysreg_ss_addr;

	wire        ss_wr_cpu;
	wire        ss_wr_mcu;
	wire        ss_wr_gfx;
	wire        ss_wr_rtc;
	wire        ss_wr_sysreg;
	wire        ss_wr_snd;

	wire        cpu_pause_rdy;
	wire        mcu_pause_rdy;
	wire        fab_pause_rdy;
	wire        sysreg_pause_rdy;
	wire        rtc_pause_rdy;
	wire        snd_pause_rdy;
	wire        snd_pause_req;
	wire        gfx_pause_rdy;
	wire        gfx_pause_req;

	wire [63:0] pwr_ss_rdata;
	wire        ss_wr_pwr;
	wire [7:0]  pwr_ss_addr;
	wire        pwr_pause_rdy;

	// Pause chain and the freeze (see the header)
	//
	// The chain is ordered, not a parallel AND: the CPU is the only bus master
	// so it drains first, the sound block stops at a Z80 instruction boundary
	// next, and the K2GE -- which needs a native frame boundary -- is asked
	// last. Every other block holds nothing in flight that the CPU has not
	// already stopped feeding, so it takes the raw request and contributes its
	// ready bit.

	assign snd_pause_req = loading_savestate ||
		(pause_req && cpu_pause_rdy);
	assign gfx_pause_req = loading_savestate ||
		(pause_req && cpu_pause_rdy && snd_pause_rdy);

	// The blocks that drain by running cannot drain while ngp_power is holding
	// their enables low, and a machine that is already stopped has nothing left
	// to drain. See "A frozen machine cannot drain" in the header.
	wire power_frozen = ~main_clk_run | ~cpu_run;

	assign pause_ready = pause_req &&
	                     (loading_savestate || power_frozen ||
	                      (cpu_pause_rdy && mcu_pause_rdy &&
	                       fab_pause_rdy && sysreg_pause_rdy && rtc_pause_rdy &&
	                       snd_pause_rdy && gfx_pause_rdy));

	// `machine_run` is low only once the whole chain has reported parked, so
	// nothing is frozen before it has finished draining. `main_clk_run` and
	// `cpu_run` AND into the same lines; ce_xt1 takes the pause gate and never
	// the standby one.
	wire machine_run  = ~pause_ready && !loading_savestate;

	wire ce_x1_g      = ce_x1      & machine_run & main_clk_run;
	wire ce_x1_div2_g = ce_x1_div2 & machine_run & main_clk_run;
	wire ce_cpu_g     = ce_cpu     & machine_run & main_clk_run & cpu_run;
	wire ce_t900_g    = ce_t900    & machine_run & main_clk_run & cpu_run;
	wire ce_xt1_g     = ce_xt1     & machine_run;

	// The savestate memory tap is byte serial and region selected. Type 2 is
	// the video region, which the K2GE now owns; the fabric answers types 0
	// and 1 and reads 0 for type 2 (its own header says so), so the two read
	// paths are selected by ss_mem_type directly. That is safe because the
	// type names the region being walked and is constant for the whole of it,
	// while the two sources have different read latencies (the fabric answers
	// one clk after the address, the K2GE two).
	wire        gfx_ss_mem_active = ss_mem_active && (ss_mem_type == 2'd2);
	wire [7:0]  gfx_ss_mem_rdata;
	wire [7:0]  fab_ss_mem_rdata;

	assign ss_mem_rdata = (ss_mem_type == 2'd2) ? gfx_ss_mem_rdata
	                                            : fab_ss_mem_rdata;

	// Retirement trace, an observation point only

	wire        trace_valid;
	wire [23:0] trace_pc;
	wire [15:0] trace_sr;
	wire [7:0]  trace_f;

	// Interrupt pin sources
	//
	// INT0 is the RTC alarm, through `power`: ngp_rtc's match is one clk_sys
	// wide and ngp_intc samples its pin history on ce_cpu (its external-pin
	// edge detector), so the pulse is turned into a held level there and never
	// wired straight across. INT4 is the K2GE VBlank and INT5 is the Z80's write
	// to its own 0xC000 region, held by `ngp_snd` for one ce_cpu for the same
	// reason. INT6 is unbonded on the NGP.
	//
	// INT7 is the panel-switch line. No document states what SNK put on it; the
	// model here is that any panel input raises it, which is what the BIOS's
	// clock-gear auto-regeneration needs in order to notice that the user is
	// awake. It is the PIN level, not an edge -- ngp_intc owns the edge, as it
	// does for every other external interrupt on this chip -- and the pad is
	// active low, so "any button down" is `~&btn_n`.
	//
	// INT7 is deliberately not a wake source: STOP and IDLE are released by NMI,
	// INT0 and reset only (TMP95C061 datasheet p.28 Table 3.4(2)).

	// The K2GE's Vint pad is active low
	//
	// `vint` out of k2ge is the active-HIGH event: high from the start of raster
	// line WBA.V+WSI.V (152 for a full window) to the start of line 0, which is
	// the vertical blanking period. The pad inverts it, so the falling edge is
	// the START of blanking, which is where software expects its V-blank
	// service. Two facts force that polarity:
	//
	//  - the TMP95C061 datasheet's 16-bit timer 4/5 capture control says "Only
	//    in this setting, interrupt INT4/INT6 occurs at fall edge", the setting
	//    being T4MOD<CAP12M> = 10; the pulse-width-measurement page repeats that
	//    in other modes it occurs at the rising edge. ngp_intc implements that;
	//  - the colour BIOS writes T4MOD = 0x30 (CAP12M = 10) at 0x00FF2454 in its
	//    MCU init table, and T5MOD = 0x30 three bytes later. So INT4 is a
	//    falling-edge interrupt from early in the boot sequence onward.
	//
	// Active high instead puts every V-blank handler 47 raster lines late
	// (47 x 515 = 24,205 CPU clocks). P_VINT_PAD_ACTIVE_LOW = 0 restores that
	// for A/B testing; it is a defect, not an option.
	//
	// PB0 is the same pad (see pb_in below), so its readback inverts with it.
	// Neither BIOS reads port B -- all four accesses to SFR 0x1F in the colour
	// BIOS are writes -- and no disassembled cartridge reads it either, so no
	// software on this console can see the inversion.

	wire int0;                        // driven by `power`
	wire vint_pad = P_VINT_PAD_ACTIVE_LOW ? ~vint : vint;
	wire int4 = vint_pad;
	wire int5;                        // driven by `snd`
	wire int6 = 1'b0;
	wire int7 = ~&btn_n;

	// NGP-specific port pin bonding
	//
	// PA0 = the WAIT pin. Nothing on the NGP board drives it, and it is active
	//       low, so it reads 1 = no wait requested.
	// PA1 = TI0, the external clock input of 8-bit timer 0, not an interrupt
	//       (TLCS-900 8-bit timer manual p.3). K2GE exposes this bond as `ti0`,
	//       which aliases the documented 152-pulse HINT waveform.
	// PA2 = TO1, PA3 = TO3 (the Z80 interrupt). Both are driven by the shell
	//       when PACR/PAFC arm them; the pin reads back 1 when it is not.
	// PB0/PB1/PB4/PB5 carry INT4/INT5/INT6/INT7 and PB7 carries INT0. They are
	//       fed internally on the NGP rather than by a package pin, so the pin
	//       view is the same signal the interrupt controller sees.
	// P5/P7/P9 are unbonded and read 1, the level an unconnected CMOS input on
	//       this board sits at.

	// P1 reads 0, not the pull-up. P1 is a real general-purpose 8-bit port here,
	// not a bus half: it shares D8-D15 only when AM8/16 = 0 and the NGP straps
	// AM8/16 = 1 (TMP95C061 datasheet p.7 section 3.1.2, pp.174-177). Reset is
	// input mode with latch 0x00, so a read returns the PIN, and nothing on the
	// board drives that pin -- an unused CMOS input cannot be left floating, so
	// it is tied to the ground plane.
	//
	// Dynamite Slugger (Japan, Europe) (En,Ja) is the software that can see the
	// difference: at 0x218ACD it reads a WORD straddling 0x000000-0x000001 and
	// at 0x218C0D compares it against a freshly zeroed scene-object selector.
	// With P1 at 0xFF the compare fails, the selector store at 0x218C18 is
	// skipped, and the Option menu dispatches to quit-to-title instead of the
	// pinch-hitter menu. With P1 at 0 it behaves.
	//
	// This is scoped to P1 alone; P5/P7/P9 keep the pull-up because nothing
	// covers them either way.
	wire [7:0] p1_in = 8'h00;
	wire [7:0] p5_in = 8'hFF;
	wire [7:0] p7_in = 8'hFF;
	wire [3:0] p9_in = 4'hF;

	// P8: bit0 TxD0, bit1 RxD0, bit2 SCLK0, bit3 TxD1, bit4 RxD1, bit5 SCLK1
	// (see rtl/soc/ngp_ports.sv). Only RxD0 is a real net on this board.
	wire [7:0] p8_in = {2'b11, 1'b1, 1'b1, 1'b1, 1'b1, link_rxd, 1'b1};

	wire [3:0] pa_in = {1'b1, 1'b1, ti0, 1'b1};
	wire [7:0] pb_in = {int0, 1'b1, int7, int6, 1'b1, 1'b1, int5, int4};

	// BIOS image strap
	//
	// The image the CPU sees is strapped at reset and is not switchable while
	// running -- flipping it mid-run would change the code under the PC. The
	// fabric takes a live input, so the sample happens here: one flop that
	// follows the pad while reset is asserted and holds afterwards. A K1-CHIP
	// build has no colour image to select, so P_MONO forces the mono side.

	reg mono_q;

	always @(posedge clk_sys) begin
		if (reset) mono_q <= mono_strap | (P_MONO != 0);
	end

	// cpu : t900_cpu

	// SPLIT_RF_READ = 1 is the synthesis configuration: it holds the register
	// file's read ports (and the prefetch queue byte) across the machine half
	// period so the decode -> register file -> ALU -> write-back chain becomes
	// two timed paths instead of one 25 ns path that never fitted in a 20.3 ns
	// clk_sys period. See t900_cpu's header.
	//
	// It requires that `ce` is never high on two consecutive clk_sys cycles,
	// which ce_t900_g satisfies: ce_t900 is one clk_sys cycle in sixteen before
	// the clock gear thins it further, and the gates above only remove pulses.
	// Do not add a gating term that could produce two adjacent enables.
	t900_cpu
	#(
		.SPLIT_RF_READ (1)
	)
	cpu
	(
		.clk          (clk_sys),
		.ce           (ce_t900_g),
		.reset        (reset),

		.plan_addr    (plan_addr),
		.bus_width8   (bus_width8),
		.bus_wait_gfx (bus_wait_gfx),
		.bus_req      (bus_req),
		.bus_we       (bus_we),
		.bus_addr     (bus_addr),
		.bus_be       (bus_be),
		.bus_wdata    (bus_wdata),
		.bus_rdata    (bus_rdata),
		.bus_rdy      (bus_rdy),

		.int_req      (int_req),
		.int_level    (int_level),
		.int_vector   (int_vector),
		.int_id       (int_id),
		.int_ack      (int_ack),
		.int_ack_id   (int_ack_id),
		.halt_release (halt_release),

		.dma_req      (dma_req),
		.dma_ack      (dma_ack),
		.dma_end      (dma_end),

		.trace_valid  (trace_valid),
		.trace_pc     (trace_pc),
		.trace_sr     (trace_sr),
		.trace_f      (trace_f),
		.halted       (halted),

		.ss_reg_addr  (cpu_ss_addr),
		.ss_wdata     (ss_bus_din[31:0]),
		.ss_wren      (ss_wr_cpu),
		.ss_rdata     (cpu_ss_rdata),
		.restore_hold (loading_savestate),
		.pause_req    (pause_req),
		.pause_ready  (cpu_pause_rdy)
	);

	// MiSTer cheat engine -- after the complete fabric read mux
	//
	// CPU instruction prefetches and data reads share this one seam, so ROM and
	// RAM cheats behave alike. Writes, memory taps and cartridge backing traffic
	// never pass through it. bus_rdy is observed only to rescan compare records
	// once the raw response is valid; the engine never changes readiness.
	ngp_cheat_engine cheat_engine
	(
		.clk_sys_i        (clk_sys),
		.load_begin_i     (cheat_load_begin),
		.invalidate_i     (cheat_invalidate),
		.commit_req_i     (cheat_commit_req),
		// Reset is also a safe atomic table-update boundary. This is deliberately
		// local to cheats; the savestate pause contract still requires pause_ready.
		.paused_i         (pause_ready | reset),
		.enable_i         (bus_req && !bus_we),
		.response_ready_i (bus_rdy),
		.code_i           (cheat_code),
		.addr_i           (bus_addr),
		.be_i             (bus_be),
		.data_i           (bus_rdata_raw),
		.data_o           (bus_rdata),
		.available_o      (cheat_available),
		.commit_done_o    (cheat_commit_done)
	);

	// fabric : k2_soc_fabric -- the internal bus and the on-chip memories

	k2_soc_fabric #(.P_OPEN_BUS(P_OPEN_BUS)) fabric
	(
		.clk             (clk_sys),
		.ce              (ce_t900_g),
		.reset           (reset),

		.plan_addr       (plan_addr),
		.bus_width8      (bus_width8),
		.bus_wait_gfx    (bus_wait_gfx),
		.bus_req         (bus_req),
		.bus_we          (bus_we),
		.bus_addr        (bus_addr),
		.bus_be          (bus_be),
		.bus_wdata       (bus_wdata),
		.bus_rdata       (bus_rdata_raw),
		.bus_rdy         (bus_rdy),

		.csc_plan_width8 (csc_plan_width8),
		.csc_waits       (csc_waits),
		.bus_active      (bus_active),

		.io_addr         (io_addr),
		.io_wdata        (io_wdata),
		.sfr_wr          (sfr_wr),
		.sfr_rd          (sfr_rd),
		.sfr_rdata       (sfr_rdata),

		.sysreg_wr       (sysreg_wr),
		.sysreg_rd       (sysreg_rd),
		.sysreg_rdata    (sysreg_rdata),
		.rtc_wr          (rtc_wr),
		.rtc_rd          (rtc_rd),
		.rtc_rdata       (rtc_rdata),

		.gfx_addr        (gfx_addr),
		.gfx_wdata       (gfx_wdata),
		.gfx_cs          (gfx_cs),
		.gfx_rd          (gfx_rd),
		.gfx_we          (gfx_we),
		.gfx_rdata       (gfx_rdata),
		.gfx_wait        (gfx_wait),

		.ext_addr        (ext_addr),
		.ext_wdata       (ext_wdata),
		.ext_oe          (ext_oe),
		.ext_rd          (ext_rd),
		.ext_wr          (ext_wr),
		.ext_rdata       (cart_d_i),
		.cart_selected   (cart_selected),
		.cart_rd_ready   (cart_rd_ready),

		.z80_ram_addr    (z80_ram_addr),
		.z80_ram_wr      (z80_ram_wr),
		.z80_ram_wdata   (z80_ram_wdata),
		.z80_ram_rdata   (z80_ram_rdata),

		.mono_strap      (mono_q),
		.bios_wr         (bios_wr),
		.bios_sel        (bios_sel),
		.bios_addr       (bios_addr),
		.bios_data       (bios_data),

		.ss_mem_type     (ss_mem_type),
		.ss_mem_active   (ss_mem_active),
		.ss_mem_addr     (ss_mem_addr),
		.ss_mem_wdata    (ss_mem_wdata),
		.ss_mem_wren     (ss_mem_wren),
		.ss_mem_rden     (ss_mem_rden),
		.ss_mem_rdata    (fab_ss_mem_rdata),
		.restore_hold    (loading_savestate),
		.pause_req       (pause_req),
		.pause_ready     (fab_pause_rdy)
	);

	// mcu : t900_mcu -- SFRs 0x000000-0x00007F

	t900_mcu mcu
	(
		.clk          (clk_sys),
		.ce           (ce_t900_g),
		.ce_periph    (ce_cpu_g),
		.ce_timer     (ce_x1_g),
		.reset        (reset),

		.sfr_addr     (io_addr),
		.sfr_wdata    (io_wdata),
		.sfr_wr       (sfr_wr),
		.sfr_rd       (sfr_rd),
		.sfr_rdata    (sfr_rdata),

		.plan_addr    (plan_addr),
		.plan_width8  (csc_plan_width8),
		.plan_cs_n    (plan_cs_n),
		.bus_addr     (bus_addr),
		.bus_active   (bus_active),
		.bus_waits    (csc_waits),
		.cs_n         (cs_n),

		.int_req      (int_req),
		.int_level    (int_level),
		.int_vector   (int_vector),
		.int_id       (int_id),
		.int_ack      (int_ack),
		.int_ack_id   (int_ack_id),
		.dma_req      (dma_req),
		.dma_ack      (dma_ack),
		.dma_end      (dma_end),
		.halt_release (halt_release),
		.halted       (halted),
		.haltm        (haltm),
		.wdt_warm     (wdt_warm),
		.wdtout_n     (wdtout_n),
		.wdt_reset    (wdt_reset),

		.nmi_n        (nmi_n),
		.int0         (int0),
		.int4         (int4),
		.int5         (int5),
		.int6         (int6),
		.int7         (int7),
		.to3_z80      (to3_z80),

		.an0          (an0),

		.rxd0         (link_rxd),
		.cts0_n       (link_cts_n),
		.txd0         (txd0),
		.txd0_oe      (txd0_oe),
		.sclk0_in     (1'b1),
		.sclk0_out    (sclk0_out),
		.sclk0_oe     (sclk0_oe),
		.rxd1         (1'b1),
		.txd1         (txd1),
		.txd1_oe      (txd1_oe),
		.sclk1_in     (1'b1),
		.sclk1_out    (sclk1_out),
		.sclk1_oe     (sclk1_oe),

		.p1_in        (p1_in), .p1_out(p1_out), .p1_oe(p1_oe),
		.p2_out       (p2_out),
		.p5_in        (p5_in), .p5_out(p5_out), .p5_oe(p5_oe),
		.p6_out       (p6_out),
		.p7_in        (p7_in), .p7_out(p7_out), .p7_oe(p7_oe),
		.p8_in        (p8_in), .p8_out(p8_out), .p8_oe(p8_oe),
		.p9_in        (p9_in),
		.pa_in        (pa_in), .pa_out(pa_out), .pa_oe(pa_oe),
		.pb_in        (pb_in), .pb_out(pb_out), .pb_oe(pb_oe),

		.ss_reg_addr  (mcu_ss_addr),
		.ss_wdata     (ss_bus_din[63:32]),
		.ss_wren      (ss_wr_mcu),
		.ss_rdata     (mcu_ss_rdata),
		.restore_hold (loading_savestate),
		.pause_req    (pause_req),
		.pause_ready  (mcu_pause_rdy)
	);

	// sysreg : ngp_sysreg -- the SNK block 0x80-0x8F, 0x9C-0xBF

	ngp_sysreg #(.P_OPEN_BUS(P_OPEN_BUS)) sysreg
	(
		.clk               (clk_sys),
		.ce                (ce_t900_g),
		.reset             (reset),

		.io_addr           (io_addr),
		.io_wdata          (io_wdata),
		.io_wr             (sysreg_wr),
		.io_rd             (sysreg_rd),
		.io_rdata          (sysreg_rdata),

		.btn_n             (btn_n),
		.pwr_btn_n         (pwr_btn_held_n),
		.subbatt_ok        (1'b1),          // deviation 3
		.link_rts_n        (link_rts_n),

		.gear              (gear),
		.nmi_n             (nmi_n_gated),
		.power_off_latch   (power_off_latch),
		.mpoff_n           (mpoff_n),
		.snd_en            (snd_en),
		.z80_run           (z80_run),
		.z80_nmi           (z80_nmi),
		.comm_latch        (comm_latch),
		.comm_latch_z80    (comm_latch_z80),
		.comm_latch_z80_wr (comm_latch_z80_wr),
		.psg_wr            (psg_wr),
		.psg_port          (psg_port),
		.psg_data          (psg_data),
		.dac_l             (dac_l),
		.dac_r             (dac_r),

		.ss_reg_addr       (sysreg_ss_addr),
		.ss_wdata          (ss_bus_din[31:0]),
		.ss_wren           (ss_wr_sysreg),
		.ss_rdata          (sysreg_ss_rdata),
		.pause_req         (pause_req),
		.pause_ready       (sysreg_pause_rdy)
	);

	// power : ngp_power -- standby, wake and the enable freeze
	// Four wiring rules, each of which is a defect if it is broken:
	//
	//   1. `ce` is the RAW ce_x1 off the pad, not ce_x1_g. The warm-up timer
	//      and the button stretcher are what END the freeze, so a frozen
	//      machine still has to clock them. (The block gates itself on
	//      pause_req, so a savestate still stops it.)
	//   2. It sits BETWEEN ngp_sysreg and the MCU on the NMI path. sysreg owns
	//      0xB1 and the 0xB3 bit 2 gate; this block owns the wake, which in
	//      S_OFF is deliberately NOT gated by 0xB3 -- a console whose battery
	//      has just been fitted has 0xB3 at its reset value and must still turn
	//      on.
	//   3. The stretched button, not the pad, is what ngp_sysreg reads back at
	//      0xB1. The BIOS's hold gate at 0xFF1A45 reads 0xB1 forty times and a
	//      single "released" aborts the boot, so the stretcher has to be
	//      upstream of the register or an OSD press is invisible to it.
	//   4. `restore_standby` is tied low. What it exists to do -- make the FIRST
	//      press take the warm path -- is already true here through `cold_run`,
	//      because the BIOS's own reset path halts and sets the context itself.
	//      The port stays so an NV restore can drive it without moving anything.

	ngp_power power
	(
		.clk             (clk_sys),
		.ce              (ce_x1),          // rule 1: UNGATED
		.reset           (reset),
		.cold_run        (1'b1),           // see the header

		.pwr_btn_raw_n   (pwr_btn_n),      // the pad
		.pwr_btn_n       (pwr_btn_held_n), // rule 3: -> ngp_sysreg 0xB1

		.nmi_n_i         (nmi_n_gated),    // rule 2
		.nmi_n           (nmi_n),
		.alarm_pulse     (alarm_pulse),
		.int0            (int0),

		.halted          (halted),
		.haltm           (haltm),
		.wdt_warm        (wdt_warm),

		.restore_standby (1'b0),           // rule 4

		.main_clk_run    (main_clk_run),
		.cpu_run         (cpu_run),
		.standby         (standby),
		.nv_flush_req    (nv_flush_req),
		.machine_has_run (machine_has_run),
		.pwr_latch_clr   (pwr_latch_clr),

		.ss_reg_addr     (pwr_ss_addr),
		.ss_wdata        (ss_bus_din),
		.ss_wren         (ss_wr_pwr),
		.ss_rdata        (pwr_ss_rdata),
		.pause_req       (pause_req),
		.pause_ready     (pwr_pause_rdy)   // always 1; nothing to drain
	);

	// rtc : ngp_rtc -- the SNK block 0x90-0x9B, 32.768 kHz domain

	ngp_rtc #(.P_OPEN_BUS(P_OPEN_BUS)) rtc
	(
		.clk         (clk_sys),
		.ce          (ce_xt1_g),
		.reset       (reset),

		.io_addr     (io_addr),
		.io_wdata    (io_wdata),
		.io_wr       (rtc_wr),
		.io_rd       (rtc_rd),
		.io_rdata    (rtc_rdata),

		.alarm_pulse (alarm_pulse),

		.ss_reg_addr (rtc_ss_addr),
		.ss_wdata    (ss_bus_din[31:0]),
		.ss_wren     (ss_wr_rtc),
		.ss_rdata    (rtc_ss_rdata),
		.pause_req   (pause_req),
		.pause_ready (rtc_pause_rdy)
	);

	// gfx : k2ge -- the graphics engine, 0x008000-0x00BFFF
	// vint reaches the CPU as INT4 and the HINT alias reaches TI0 (the PA1 pin).
	// Both are boot-critical: the cold reset path waits on 0x8010 bit 6 before
	// it can halt, and the post-wake main loop is VBlank driven. The `hint`
	// output is left unconnected -- HINT reaches the interrupt controller
	// through the TI0 bond and the timer, not as a pin of its own.

	k2ge gfx
	(
		.clk_sys       (clk_sys),
		.ce_6m144      (ce_x1_g),
		.rst           (reset),
		.main_clk_run  (main_clk_run),
		.palette_frame_boundary (palette_frame_boundary),

		.cpu_a         (gfx_addr),
		.cpu_din       (gfx_wdata),
		.cpu_dout      (gfx_rdata),
		.cpu_cs        (gfx_cs),
		.cpu_rd        (gfx_rd),
		.cpu_we        (gfx_we),
		.cpu_wait_states (gfx_wait),

		.vint          (vint),
		.hint          (),
		.ti0           (ti0),

		.lcd_r         (lcd_r),
		.lcd_g         (lcd_g),
		.lcd_b         (lcd_b),
		.lcd_dclk_ce   (lcd_dclk_ce),
		.lcd_de        (lcd_de),
		.lcd_hs        (lcd_hs),
		.lcd_vs        (lcd_vs),
		.lcd_lp        (lcd_lp),
		.lcd_sp        (lcd_sp),

		.led           (led),
		.inp0          (inp0),
		.mono_strap    (mono_q),

		.ss_addr       (ss_bus_adr[4:0]),
		.ss_wdata      (ss_bus_din),
		.ss_wren       (ss_wr_gfx),
		.ss_rdata      (gfx_ss_rdata),

		.ss_mem_active (gfx_ss_mem_active),
		.ss_mem_addr   (ss_mem_addr),
		.ss_mem_wdata  (ss_mem_wdata),
		.ss_mem_wren   (ss_mem_wren),
		.ss_mem_rden   (ss_mem_rden),
		.ss_mem_rdata  (gfx_ss_mem_rdata),
		.restore_hold  (loading_savestate),

		.pause_req     (gfx_pause_req),
		.pause_ready   (gfx_pause_rdy)
	);

	// Cartridge pads
	//
	// The chip selects come from the real CS/WAIT controller inside the MCU
	// shell, never from a hardcoded decode: the BIOS programs MSAR0/MAMR0 and
	// MSAR1/MAMR1 itself. CS2 and CS3 exist as register state and as ngp_ports
	// P6 functions but are not bonded to the cartridge connector on this board.

	assign cart_a    = ext_addr[20:0];
	assign cart_d_o  = ext_wdata;
	assign cart_d_oe = ext_oe;
	assign cart_nce0 = cs_n[0];
	assign cart_nce1 = cs_n[1];
	assign cart_noe  = ~(bus_active && !bus_we);
	assign cart_nwe  = ~(bus_active &&  bus_we);

	// Link port pad (deviation 4)

	assign link_txd = p8_oe[0] ? p8_out[0] : 1'b1;

	// snd : ngp_snd -- the Z80, the T6W28 and the mixer
	// Everything the sound block needs from the main CPU already exists as an
	// ngp_sysreg fan-out, so this is pure wiring:
	//
	//   snd_en / z80_run / z80_nmi / comm_latch    0xB8 / 0xB9 / 0xBA / 0xBC
	//   psg_wr / psg_port / psg_data               the 0xA0-0xA1 direct path,
	//                                              already gated by
	//                                              (snd_en && !z80_run)
	//   dac_l / dac_r                              the 0xA2 / 0xA3 latches
	//   comm_z80_wdata / _wr                       the Z80's half of 0xBC
	//
	// and two machine nets: `to3_z80`, the PA3 pin the MCU's timer 3 drives,
	// and `int5`, which goes back into the interrupt controller through the PB1
	// pin view like every other external interrupt on this chip.
	//
	// `ce_x1_div2` is the 3.072 MHz enable; the block has no other clock.
	//
	// P_OPEN_BUS is deliberately NOT passed down. The chip's open bus and the
	// Z80's are different buses: the Z80's 0xFF is chosen so a runaway core
	// executes RST 38h into its own IM1 handler, which is what a floating bus
	// does. One measurement does not settle both.

	ngp_snd snd
	(
		.clk               (clk_sys),
		.reset             (reset),
		.ce_3m072          (ce_x1_div2_g),
		.ce_cpu            (ce_t900_g),

		.snd_en            (snd_en),
		.z80_run           (z80_run),
		.z80_nmi           (z80_nmi),
		.comm_latch        (comm_latch),
		.psg_wr            (psg_wr),
		.psg_port          (psg_port),
		.psg_data          (psg_data),
		.dac_l_in          (dac_l),
		.dac_r_in          (dac_r),

		.comm_z80_wdata    (comm_latch_z80),
		.comm_z80_wr       (comm_latch_z80_wr),

		.zram_addr         (z80_ram_addr),
		.zram_wr           (z80_ram_wr),
		.zram_wdata        (z80_ram_wdata),
		.zram_rdata        (z80_ram_rdata),

		.to3               (to3_z80),
		.int5              (int5),

		.audio_l           (audio_l),
		.audio_r           (audio_r),
		.mix_psg_gain      (P_MIX_PSG_GAIN),
		.mix_dac_gain      (P_MIX_DAC_GAIN),

		.ss_reg_addr       (snd_ss_addr),
		.ss_wdata          (ss_bus_din[31:0]),
		.ss_wren           (ss_wr_snd),
		.ss_rdata          (snd_ss_rdata),
		.ss_mix_wdata      (ss_bus_din[63:32]),
		.ss_mix_rdata      (snd_mix_ss_rdata),
		.loading_savestate (loading_savestate),
		.pause_req         (snd_pause_req),
		.pause_ready       (snd_pause_rdy)
	);

	// Savestate internals adapter (see the header for the map and the reason)

	wire ss_sel_cpu = (ss_bus_adr <  10'd48);
	wire ss_sel_snd = (ss_bus_adr >= 10'd48) && (ss_bus_adr < 10'd64);
	wire ss_sel_gfx = (ss_bus_adr >= 10'd64) && (ss_bus_adr < 10'd88);
	wire ss_sel_rtc = (ss_bus_adr >= 10'd88) && (ss_bus_adr < 10'd92);
	wire ss_sel_pwr = (ss_bus_adr == 10'd92);
	wire ss_sel_sys = (ss_bus_adr >= 10'd93) && (ss_bus_adr < 10'd96);

	// The CPU's tap map is sparse (0x00-0x13, 0x20-0x22, 0x30-0x3B); the
	// internals words are dense. Adds of literals only -- no division, no
	// modulus, one LUT level per arm.
	always @* begin
		if (ss_bus_adr < 10'd20) begin
			cpu_ss_addr = {3'd0, ss_bus_adr[4:0]};             // 0x00-0x13
			cpu_slot    = 1'b1;
		end else if (ss_bus_adr < 10'd23) begin
			cpu_ss_addr = 8'h20 + (ss_bus_adr[7:0] - 8'd20);   // 0x20-0x22
			cpu_slot    = 1'b1;
		end else if (ss_bus_adr < 10'd35) begin
			cpu_ss_addr = 8'h30 + (ss_bus_adr[7:0] - 8'd23);   // 0x30-0x3B
			cpu_slot    = 1'b1;
		end else begin
			cpu_ss_addr = 8'hFF;                               // no slot
			cpu_slot    = 1'b0;
		end
	end

	// ngp_snd's tap is dense in two runs: 0x00-0x08 then 0x10-0x15. Word 63 is
	// reserved, and it is steered at 0xFF -- an address the block decodes as
	// nothing at all -- so both halves read zero and a write reaches no register.
	always @* begin
		if (ss_bus_adr < 10'd57)
			snd_ss_addr = 8'h00 + (ss_bus_adr[7:0] - 8'd48);   // 0x00-0x08
		else if (ss_bus_adr < 10'd63)
			snd_ss_addr = 8'h10 + (ss_bus_adr[7:0] - 8'd57);   // 0x10-0x15
		else
			snd_ss_addr = 8'hFF;                               // 63, reserved
	end

	assign mcu_ss_addr    = ss_bus_adr[7:0];                   // identity
	assign rtc_ss_addr    = ss_bus_adr[7:0] - 8'd88;
	assign sysreg_ss_addr = ss_bus_adr[7:0] - 8'd93;

	// ngp_power owns exactly one word and decodes it as local 0x00. Every
	// other address is steered to 0xFF, which the block decodes as nothing, so
	// it reads zero and a write there reaches no register.
	assign pwr_ss_addr    = ss_sel_pwr ? 8'h00 : 8'hFF;

	assign ss_wr_cpu    = ss_bus_wren && ss_sel_cpu && cpu_slot;
	assign ss_wr_mcu    = ss_bus_wren && ss_sel_cpu;
	assign ss_wr_snd    = ss_bus_wren && ss_sel_snd;
	assign ss_wr_gfx    = ss_bus_wren && ss_sel_gfx;
	assign ss_wr_rtc    = ss_bus_wren && ss_sel_rtc;
	assign ss_wr_pwr    = ss_bus_wren && ss_sel_pwr;
	assign ss_wr_sysreg = ss_bus_wren && ss_sel_sys;

	// A priority chain of range compares, never a wired-OR.
	reg [63:0] ss_dout_r;
	always @* begin
		if (ss_sel_gfx)      ss_dout_r = gfx_ss_rdata;
		else if (ss_sel_rtc) ss_dout_r = {32'd0, rtc_ss_rdata};
		else if (ss_sel_pwr) ss_dout_r = pwr_ss_rdata;      // native 64-bit
		else if (ss_sel_sys) ss_dout_r = {32'd0, sysreg_ss_rdata};
		else if (ss_sel_snd) ss_dout_r = {snd_mix_ss_rdata, snd_ss_rdata};
		else if (ss_sel_cpu) ss_dout_r = {mcu_ss_rdata,
		                                  cpu_slot ? cpu_ss_rdata : 32'd0};
		else                 ss_dout_r = 64'd0;
	end

	assign ss_bus_dout = ss_dout_r;

	// Both BIOSes commit the power-off latch and then HALT after their cold
	// initialization pass, but ngp_power still has to consume that HALT edge and
	// finish S_SETTLE before the machine is actually off. Starting the setup
	// pause from the BIOS pair alone freezes that transition, and the following
	// automatic Power edge is then consumed before S_OFF can accept it. So this
	// is qualified with the power block's public standby state: the overlay and
	// its wake request both happen only after standby has completed.
	assign bios_setup_ready = standby && halted && power_off_latch;
	assign bios_mono_active = mono_q;

	// Deliberately unread at this level
	//
	// Each of these is a real pad or block output with no consumer on this
	// board. They are wired to names rather than left dangling so a consumer is
	// a one-line change.
	//   nv_flush_req     the standby-entry flush trigger. There is no NV slot to
	//                    flush, but this is the only moment the file's contents
	//                    would be meaningful.
	//   machine_has_run  the "do not restore over a running machine" interlock.
	//   pwr_latch_clr    the alternative reading in which a wake edge clears
	//                    0xB4; off by default, and ngp_sysreg has no input for
	//                    it. See ngp_power's header.
	//   pwr_pause_rdy    ngp_power reports ready unconditionally -- it holds
	//                    nothing in flight -- so ANDing it into the chain would
	//                    only add a constant.
	//   wdt_reset        the watchdog's internal reset; see the header for why
	//                    no reset controller is built.
	wire unused_ok = &{1'b0,
	                   nv_flush_req, machine_has_run, pwr_latch_clr,
	                   pwr_pause_rdy, cheat_available,
	                   wdtout_n, wdt_reset, plan_cs_n, cs_n[3:2],
	                   ext_rd, ext_wr, ext_addr[23:21],
	                   txd0, txd0_oe, sclk0_out, sclk0_oe,
	                   txd1, txd1_oe, sclk1_out, sclk1_oe,
	                   p1_out, p1_oe, p2_out, p5_out, p5_oe, p6_out,
	                   p7_out, p7_oe, p8_out[7:1], p8_oe[7:1], pa_out, pa_oe,
	                   pb_out, pb_oe,
	                   trace_valid, trace_pc, trace_sr, trace_f,
	                   1'b0};

endmodule
