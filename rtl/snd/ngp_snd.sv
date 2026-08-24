// Copyright (c) 2026 Jamie Blanks

// NGP/NGPC sound subsystem wrapper.
//
// Holds the Z80 (`ngp_z80` around vendored tv80), the T6W28, the mixer, the
// PSG write-port owner mux, the TO3 interrupt latch, the NMI stretcher, the
// INT5 generator and the Z80's address decode.
//
// The 4 KB shared RAM is `zram` in `k2_soc_fabric` (port A is the main CPU's
// 0x007000-0x007FFF window, this block drives port B through `zram_*`), and
// ports 0xB8, 0xB9, 0xBA, 0xBC and 0xA0-0xA3 are latches in `ngp_sysreg`,
// which owns their savestate words.
//
// The Z80's memory map:
//   0x0000-0x3FFF  R/W  4 KB shared RAM, mirrored four times
//   0x4000-0x7FFF  W    T6W28; A0 picks the port
//   0x8000-0xBFFF  R/W  the one-byte mailbox (= main 0xBC)
//   0xC000-0xFFFF  W    any write raises INT5 on the main CPU
//   I/O space, any port, write -> clears this Z80's own interrupt latch
//
// The decode is coarse (A15:A14 for the region, A0 for the PSG port);
// `P_COARSE_DECODE` selects exact-address decode instead.  Reads of the
// write-only regions return `P_OPEN_BUS`.
//
// Everything audible runs on `snd_ce`, which is `ce_3m072` gated off while the
// block is parked, so freezing the subsystem is exactly removing the enable.
// `ce_cpu` is used only to stretch the INT5 request: the interrupt controller
// edge-detects on its own `ce`, which at clock gear 4 is 1-in-256 while the
// Z80 runs at 1-in-16, so a request lasting one Z80 cycle would be invisible.
// `psg_wr`, `z80_nmi`, `ss_wren` and the savestate strobes are single
// `clk_sys` cycles and must never be re-gated by an enable, which would drop
// them silently.
//
// Savestate word map, in the low halves of internals words 48-63:
//   internals 48     ss_reg_addr 0x00        this block's control word
//   internals 49-56  ss_reg_addr 0x01-0x08   the Z80's architectural registers,
//                                            one register pair per word as
//                                            tv80's register file needs
//   internals 57-62  ss_reg_addr 0x10-0x15   the T6W28
//   internals 63                             reserved, reads zero
// The upper halves of words 48-57 carry ngp_mixer's ten-address layout, of
// which six addresses hold state and the rest read zero.
//
// Two restore-order rules.  `ss_wren` is honoured unconditionally rather than
// qualified by `pause_ready`, because the engine's order is power-on reset,
// then broadcast internals, and a broadcast arriving after the pause handshake
// released would silently drop its writes.  And the Z80's reset is forced
// inactive while `loading_savestate` is high, or `z80_run` starts at 0 after
// the power-on reset and tv80's asynchronous reset clobbers every restored
// register word before `z80_run` itself is restored.

module ngp_snd
#(
	// Value returned by the Z80's reads of write-only or unmapped regions.
	// 0xFF makes a runaway Z80 execute RST 38h and land in its own IM1
	// handler, which is what a floating bus does.
	parameter [7:0]  P_OPEN_BUS       = 8'hFF,

	// 1 = decode A15:A14 only, so the RAM mirrors four times and the PSG
	//     covers 0x4000-0x7FFF (the default).
	// 0 = exact-address decode.
	parameter        P_COARSE_DECODE  = 1,

	// What clears the TO3 interrupt latch.
	// 1 = any Z80 I/O WRITE clears the latch (default);
	// 0 = the interrupt acknowledge clears it.
	parameter        P_IRQ_CLR_ON_OUT = 1,

	// 1 = only a write to the 0xC000 region raises INT5 (default);
	// 0 = any access does.
	parameter        P_INT5_ON_WRITE  = 1,

	// T6W28 model parameters, passed straight through so one hardware
	// measurement can settle them from the top of the tree.  Defaults match
	// t6w28.sv's own.
	parameter integer LFSR_WIDTH  = 15,
	parameter [15:0]  LFSR_RESET  = 16'h4000,
	parameter [15:0]  LFSR_TAPS   = 16'h0003,
	parameter integer LFSR_INVERT = 0,
	parameter [1:0]   TONE_ZERO   = 2'd1,
	parameter integer NOISE3_X2   = 1,

	// PSG DC-blocker corner. MiSTer's standard audio_out owns reconstruction.
	parameter integer DCB_SHIFT = 11
)
(
	input  wire        clk,             // clk_sys, 49.152 MHz
	input  wire        reset,           // machine reset, active high
	input  wire        ce_3m072,        // 1-in-16 of clk_sys
	input  wire        ce_cpu,          // fosc/2 / gear machine-state enable

	// ---- from ngp_sysreg ---------------------------------------------------
	input  wire        snd_en,          // 0xB8 == 0x55
	input  wire        z80_run,         // 0xB9 == 0x55
	input  wire        z80_nmi,         // one-ce_cpu pulse on any write to 0xBA
	input  wire [7:0]  comm_latch,      // 0xBC, current value
	input  wire        psg_wr,          // 0xA0/0xA1 write, already gated by
	                                    // (snd_en && !z80_run)
	input  wire        psg_port,        // 0 = 0xA0 -> PSG port 0, 1 = 0xA1
	input  wire [7:0]  psg_data,
	input  wire [7:0]  dac_l_in,        // 0xA2 latch
	input  wire [7:0]  dac_r_in,        // 0xA3 latch

	// ---- to ngp_sysreg -----------------------------------------------------
	output wire [7:0]  comm_z80_wdata,  // -> comm_latch_z80
	output wire        comm_z80_wr,     // -> comm_latch_z80_wr

	// ---- shared RAM, Z80 side (port B of k2_soc_fabric's `zram`) -----------
	output wire [11:0] zram_addr,
	output wire        zram_wr,
	output wire [7:0]  zram_wdata,
	input  wire [7:0]  zram_rdata,      // valid the clk after zram_addr

	// ---- machine wiring ----------------------------------------------------
	input  wire        to3,             // timer 3 output pin from t900_mcu
	output wire        int5,            // -> ngp_intc, held for one ce_cpu

	// ---- audio -------------------------------------------------------------
	output wire signed [15:0] audio_l,
	output wire signed [15:0] audio_r,
	input  wire [2:0]  mix_psg_gain,    // 4 = nominal, each step 6 dB
	input  wire [2:0]  mix_dac_gain,

	// ---- savestate ---------------------------------------------------------
	input  wire [7:0]  ss_reg_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire [31:0] ss_mix_wdata,
	output wire [31:0] ss_mix_rdata,
	input  wire        loading_savestate,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [7:0] SS_CTL = 8'h00;

	// Declared up front so every net exists before the first instance that
	// reads it.
	wire ss_sel_ctl = (ss_reg_addr == SS_CTL);
	wire ss_sel_z80 = (ss_reg_addr[7:4] == 4'h0) && (ss_reg_addr[3:0] != 4'h0);
	wire ss_sel_psg = (ss_reg_addr[7:4] == 4'h1);
	wire ss_wr_ctl  = ss_wren && ss_sel_ctl;

	reg       irq_latch;
	reg       to3_q;
	reg [1:0] nmi_cnt;
	reg       nmi_rearm;
	wire      nmi_pending = |nmi_cnt;

	// Pause and the one enable
	// `pause_ready` may only rise between Z80 instructions, or while the Z80
	// is held in reset.  Nothing else in this block can be mid-transaction:
	// every write here is a single-cycle register or BRAM write and nothing
	// talks to SDRAM or DDR, so draining costs nothing.

	wire z80_reset;
	wire instr_boundary;

	reg  parked;
	wire snd_ce = ce_3m072 && !parked;

	always @(posedge clk) begin
		if (reset) begin
			parked <= 1'b0;
		end else if (!pause_req) begin
			parked <= 1'b0;
		end else if (z80_reset || (instr_boundary && !z80_int_taken)) begin
			parked <= 1'b1;
		end
	end

	assign pause_ready = parked;

	// While a pause is pending but not yet taken, mask new pin samples. A
	// request sampled before the mask may still be accepted on the collision
	// boundary; park admission detects `z80_int_taken` and lets that acceptance
	// reach the next representable instruction boundary.
	wire int_mask = pause_req;

	// Z80 reset and the restore-ordering safeguard
	assign z80_reset = reset || (!z80_run && !loading_savestate);

	// Z80 core
	wire [15:0] z80_addr;
	wire [7:0]  z80_dout;
	wire        z80_m1_n, z80_mreq_n, z80_iorq_n, z80_rd_n, z80_wr_n;
	wire        z80_rfsh_n, z80_halt_n;
	reg  [7:0]  z80_din;
	wire [31:0] z80_ss_rdata;

	// Named rather than left as empty port connections so the file lints
	// clean; simulation-only.
	wire        z80_naive_boundary;
	wire        z80_prefix_active;
	wire        z80_int_taken;
	wire [6:0]  z80_mc;
	wire [6:0]  z80_ts;

	// Region decode.  Coarse by default; P_COARSE_DECODE = 0 selects the
	// exact-address model.
	wire [1:0] z80_rgn = z80_addr[15:14];
	wire sel_ram  = P_COARSE_DECODE ? (z80_rgn == 2'b00)
	                                : (z80_addr[15:12] == 4'h0);
	wire sel_psg  = P_COARSE_DECODE ? (z80_rgn == 2'b01)
	                                : (z80_addr[15:1] == 15'h2000);
	wire sel_comm = P_COARSE_DECODE ? (z80_rgn == 2'b10)
	                                : (z80_addr == 16'h8000);
	wire sel_int5 = P_COARSE_DECODE ? (z80_rgn == 2'b11)
	                                : (z80_addr == 16'hC000);

	// Memory strobes.  /MREQ is also pulsed during the M1 refresh half-cycle
	// with the I:R address on the bus, so every access below is qualified by
	// /RD or /WR as well -- refresh drives neither.
	wire z80_mem_rd = !z80_mreq_n && !z80_rd_n;
	wire z80_mem_wr = !z80_mreq_n && !z80_wr_n;
	wire z80_io_wr  = !z80_iorq_n && !z80_wr_n;
	// An interrupt acknowledge asserts /IORQ with /M1 and neither /RD nor
	// /WR, so qualifying on /WR excludes it automatically
	// (Z80 manual p.12, interrupt request/acknowledge cycle).
	wire z80_inta   = !z80_iorq_n && !z80_m1_n;

	always @* begin
		if (z80_mem_rd && sel_ram)       z80_din = zram_rdata;
		else if (z80_mem_rd && sel_comm) z80_din = comm_latch;
		else               z80_din = P_OPEN_BUS;
	end

	ngp_z80 u_z80
	(
		.clk            (clk),
		.ce             (snd_ce),
		.reset          (z80_reset),

		.int_n          (~(irq_latch && !int_mask)),
		.nmi_n          (int_mask || nmi_rearm || !nmi_pending),
		// The T6W28's READY is modelled but not wired to /WAIT: 32 chip
		// clocks per write is 10.4 us and real drivers issue PSG writes
		// about 4.2 us apart, so an honest stall would wreck their timing.
		.wait_n         (1'b1),

		.addr           (z80_addr),
		.din            (z80_din),
		.dout           (z80_dout),
		.m1_n           (z80_m1_n),
		.mreq_n         (z80_mreq_n),
		.iorq_n         (z80_iorq_n),
		.rd_n           (z80_rd_n),
		.wr_n           (z80_wr_n),
		.rfsh_n         (z80_rfsh_n),
		.halt_n         (z80_halt_n),

		.instr_boundary (instr_boundary),
		.naive_boundary (z80_naive_boundary),
		.prefix_active  (z80_prefix_active),
		.int_taken      (z80_int_taken),
		.mc             (z80_mc),
		.ts             (z80_ts),

		.ss_addr        (ss_reg_addr[3:0]),
		.ss_wdata       (ss_wdata),
		.ss_wren        (ss_wren && ss_sel_z80),
		.ss_rdata       (z80_ss_rdata)
	);

	// Shared RAM, Z80 side
	// The address is registered into the BRAM, so `zram_rdata` is valid the
	// clock after `zram_addr` changes.  The Z80 holds its address stable from
	// T1 through T3 -- at least 32 clk_sys cycles at ce_3m072 -- so the data
	// is ready long before it is sampled.  No wait state is needed
	// (SysPro p.6 specifies 0 wait states for this RAM).
	assign zram_addr  = z80_addr[11:0];
	// Gated by `!parked` so the savestate memory walker owns port B outright
	// while the machine is stopped.  The Z80 can only park between
	// instructions, so no write is ever in flight when this drops.
	assign zram_wr    = sel_ram && z80_mem_wr && !parked;
	assign zram_wdata = z80_dout;

	// Mailbox, Z80 side
	// One shared byte; last writer wins, no full/empty flags.  ngp_sysreg owns
	// the register, so this block only forwards the Z80's writes to it.
	assign comm_z80_wdata = z80_dout;
	assign comm_z80_wr    = sel_comm && z80_mem_wr;

	// INT5 -- Z80 write to the 0xC000 region
	// Held until the interrupt controller's own enable has had a chance to
	// sample it.  Two Z80 writes inside one `ce_cpu` therefore merge into one
	// request, which is also what an edge-latched controller would do if the
	// CPU had not serviced the first.
	wire int5_set = sel_int5 && (P_INT5_ON_WRITE ? z80_mem_wr
	                                             : (z80_mem_wr || z80_mem_rd));

	reg int5_hold;
	always @(posedge clk) begin
		if (reset || z80_reset) begin
			int5_hold <= 1'b0;
		end else if (ss_wr_ctl) begin
			int5_hold <= ss_wdata[25];
		end else if (int5_set) begin
			int5_hold <= 1'b1;
		end else if (ce_cpu && !parked) begin
			int5_hold <= 1'b0;
		end
	end

	assign int5 = int5_hold;

	// TO3 -> Z80 /INT, an S/R latch
	// Set on the rising edge of TO3, cleared by a Z80 I/O write.  Set wins, so
	// a timer tick landing on the same clock as the clearing OUT is not lost.
	// An IM1 handler that never executes an OUT therefore re-enters forever.
	// `P_IRQ_CLR_ON_OUT = 0` selects the acknowledge-clears model instead.
	//
	// TO3 arrives in the CPU's enable domain, so the edge detector samples in
	// clk_sys and is NOT gated by snd_ce -- at clock gear 0 the CPU is faster
	// than this block's enable and a gated detector would miss short pulses.

	wire irq_clr = P_IRQ_CLR_ON_OUT ? z80_io_wr : (z80_inta && z80_rd_n && z80_wr_n);

	always @(posedge clk) begin
		if (reset) begin
			to3_q     <= 1'b0;
			irq_latch <= 1'b0;
		end else begin
			if (!loading_savestate) to3_q <= to3;

			// The Z80 reset clears the latch, so a freshly started driver
			// does not take a stale interrupt.
			if (!loading_savestate) begin
				if (z80_reset)               irq_latch <= 1'b0;
				else if (to3 && !to3_q)      irq_latch <= 1'b1;
				else if (irq_clr)            irq_latch <= 1'b0;
			end

			if (ss_wr_ctl) begin
				to3_q     <= ss_wdata[30];
				irq_latch <= ss_wdata[26];
			end
		end
	end

	// NMI stretcher
	// `z80_nmi` is one `ce_cpu` wide.  Hold /NMI asserted for two Z80 enables
	// so an edge-triggered input sees a full low period followed by a high
	// one.  A second request while it is still low restarts the counter, which
	// merges the two -- the same thing a hardware edge detector would do with
	// two edges inside one Z80 clock.
	// tv80 reset clears its previous-pin sample low, so a restored live counter
	// first presents one sampled high enable. The counter is held for that
	// enable, then `nmi_rearm` releases the complete restored low pulse.

	always @(posedge clk) begin
		if (reset || z80_reset) begin
			nmi_cnt   <= 2'd0;
			nmi_rearm <= 1'b0;
		end else begin
			if (z80_nmi) begin
				nmi_cnt   <= 2'd2;
				nmi_rearm <= 1'b0;
			end else if (snd_ce && nmi_rearm) begin
				nmi_rearm <= 1'b0;
			end else if (snd_ce && (nmi_cnt != 0)) begin
				nmi_cnt <= nmi_cnt - 2'd1;
			end

			if (ss_wr_ctl) begin
				nmi_cnt   <= ss_wdata[29:28];
				nmi_rearm <= |ss_wdata[29:28];
			end
		end
	end

	// PSG write-port owner
	// `psg_wr` is a one-clk_sys strobe from ngp_sysreg and the PSG only looks
	// at its pins on a `ce` tick, so the main-CPU write is latched and held
	// until one enable has passed.  Setting wins over clearing, so a write
	// landing on the same clock as the enable is never lost.
	//
	// Because ngp_sysreg already gates `psg_wr` with (snd_en && !z80_run), the
	// Z80 is guaranteed to be in reset whenever the main path is live, so a
	// plain two-way mux is faithful and no arbiter is needed.
	//
	// 0xA1 drives PSG port 1.

	// A two-deep queue, because the main CPU can issue two byte writes closer
	// together than one `ce_3m072` period -- a 16-bit store to 0x0000A0 is
	// exactly that, two byte cycles to 0xA0 then 0xA1 -- while the PSG can
	// only accept one write per enable.  Without the queue the second write
	// is swallowed, which for a 0xA0/0xA1 pair means one whole port is never
	// programmed.  Two entries is enough: each main-CPU byte cycle is at
	// least two machine states, so no more than two can land inside one
	// enable period even at clock gear 0.
	//
	// The strobe is presented for exactly one enable and then forced low for
	// one, because the PSG edge-detects its /WE and a strobe held across two
	// enables would count as one write. Pause drains until the park boundary
	// and then holds queue/strobe state; reset flushes both entries. Switching
	// to Z80 ownership holds queued main bytes and forces their strobe low.

	reg       main_go;
	reg [1:0] qcnt;
	reg [8:0] q0;                        // {port, data}
	reg [8:0] q1;

	wire       pop  = snd_ce && main_go && !z80_run;
	wire       push = psg_wr && !z80_run && ((qcnt != 2'd2) || pop);
	wire [8:0] item = {psg_port, psg_data};

	always @(posedge clk) begin
		if (reset) begin
			main_go <= 1'b0;
			qcnt    <= 2'd0;
			q0      <= 9'd0;
			q1      <= 9'd0;
		end else begin
			// Strobe: high for one owned enable, then low for one. Ownership
			// cannot consume a queued byte unless that byte reached the PSG pins.
			if (snd_ce) begin
				if (z80_run) main_go <= 1'b0;
				else         main_go <= (qcnt != 2'd0) && !main_go;
			end

			// Queue.  Push and pop can land on the same clock, and push must
			// win over nothing -- both are honoured.
			case ({push, pop})
				2'b10: begin
					if (qcnt == 2'd0) q0 <= item;
					else              q1 <= item;
					qcnt <= qcnt + 2'd1;
				end
				2'b01: begin
					q0   <= q1;
					qcnt <= qcnt - 2'd1;
				end
				2'b11: begin
					q0 <= (qcnt == 2'd1) ? item : q1;
					if (qcnt == 2'd2) q1 <= item;
				end
				default: ;
			endcase

			if (ss_wr_ctl) begin
				main_go <= ss_wdata[27];
				qcnt    <= ss_wdata[23:22];
				q0      <= {ss_wdata[31], ss_wdata[7:0]};
				q1      <= {ss_wdata[21], ss_wdata[20:13]};
			end
		end
	end

	wire z80_psg_wr = z80_run && sel_psg && z80_mem_wr;
	wire psg_active = z80_run ? z80_psg_wr : main_go;

	wire       psg_ce_n = ~psg_active;
	wire       psg_we_n = ~psg_active;
	wire       psg_a0   = z80_run ? z80_addr[0] : q0[8];
	wire [7:0] psg_din  = z80_run ? z80_dout    : q0[7:0];

	// The PSG
	wire [13:0] psg_out_l;
	wire [13:0] psg_out_r;
	wire [31:0] psg_ss_rdata;
	wire        psg_ready;

	t6w28 #(
		.LFSR_WIDTH  (LFSR_WIDTH),
		.LFSR_RESET  (LFSR_RESET),
		.LFSR_TAPS   (LFSR_TAPS),
		.LFSR_INVERT (LFSR_INVERT),
		.TONE_ZERO   (TONE_ZERO),
		.NOISE3_X2   (NOISE3_X2)
	) u_psg
	(
		.clk      (clk),
		.ce       (snd_ce),
		.rst_n    (~reset),

		.ce_n     (psg_ce_n),
		.we_n     (psg_we_n),
		.a0       (psg_a0),
		.din      (psg_din),
		.ready    (psg_ready),

		// Default: 0xB8 mutes the output stage only; Z80 writes still land.
		.enable   (snd_en),

		.out_l    (psg_out_l),
		.out_r    (psg_out_r),

		.ss_addr  (ss_reg_addr[3:0]),
		.ss_wdata (ss_wdata),
		.ss_wren  (ss_wren && ss_sel_psg),
		.ss_rdata (psg_ss_rdata)
	);

	// Mixer
	// The low sound tap has a split address range. Collapse that range into the
	// dense global-word index used by the upper-half mixer tap. Removed filter
	// state addresses remain reserved and read zero, preserving the layout.
	reg [3:0] mix_ss_addr;
	always @* begin
		if (ss_reg_addr <= 8'h08)
			mix_ss_addr = ss_reg_addr[3:0];
		else if ((ss_reg_addr >= 8'h10) && (ss_reg_addr <= 8'h15))
			mix_ss_addr = 4'd9 + ss_reg_addr[3:0];
		else
			mix_ss_addr = 4'hF;
	end

	ngp_mixer #(
		.DCB_SHIFT (DCB_SHIFT)
	) u_mix
	(
		.clk          (clk),
		.ce           (snd_ce),
		.reset        (reset),

		.psg_l        (psg_out_l),
		.psg_r        (psg_out_r),
		.dac_l        (dac_l_in),
		.dac_r        (dac_r_in),

		.mix_psg_gain (mix_psg_gain),
		.mix_dac_gain (mix_dac_gain),
		.ss_addr      (mix_ss_addr),
		.ss_wdata     (ss_mix_wdata),
		.ss_wren      (ss_wren),
		.ss_rdata     (ss_mix_rdata),

		.audio_l      (audio_l),
		.audio_r      (audio_r)
	);

	// Savestate tap
	reg [31:0] ss_rdata_r;
	always @* begin
		if (ss_sel_ctl) begin
			// [31]    q0 port          [30]    to3_q
			// [29:28] nmi stretch      [27]    main_go
			// [26]    irq_latch        [25]    int5_hold
			// [24]    parked (reported, never restored -- pause state, not
			//         machine state)
			// [23:22] queue occupancy  [21]    q1 port
			// [20:13] q1 data          [7:0]   q0 data
			ss_rdata_r = {q0[8], to3_q, nmi_cnt,
			              main_go, irq_latch, int5_hold, parked,
			              qcnt, q1[8], q1[7:0], 5'd0, q0[7:0]};
		end else if (ss_sel_z80) begin
			ss_rdata_r = z80_ss_rdata;
		end else if (ss_sel_psg) begin
			ss_rdata_r = psg_ss_rdata;
		end else begin
			ss_rdata_r = 32'd0;
		end
	end

	assign ss_rdata = ss_rdata_r;

	// `ready`, /RFSH and /HALT are real pins that nothing on this board
	// watches, so they are modelled but unread.
	wire unused_ok = &{1'b1, psg_ready, z80_rfsh_n, z80_halt_n,
	                   z80_naive_boundary, z80_prefix_active,
	                   z80_mc, z80_ts};

endmodule
