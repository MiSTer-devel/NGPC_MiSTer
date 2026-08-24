// Copyright (c) 2026 Jamie Blanks

// Z80 sound-CPU adapter.
//
// One `tv80_core` (vendored under rtl/snd/z80/tv80/, MIT) plus the bus-strobe
// decode that turns its `iorq / no_read / write / mc / ts` encoding into the
// five separate Z80 strobes the rest of the sound block decodes.
//
// The decode is functionally the upstream `tv80s` wrapper with three changes.
// It is gated by `ce`: `tv80s` ties the core's `cen` to 1 and runs its strobe
// register off the raw clock, which on a 1-in-16 enable would advance the
// strobes sixteen times per T state -- hence `tv80_core` directly rather than
// `tv80s`.  `TV80_REFRESH` must be defined by the build, or the core has no R
// register at all (`LD A,R` returns zero, `LD R,A` is a no-op) and `rfsh_n` is
// tied high.  And `instr_boundary` is brought out for the pause protocol.
//
// Mode/IOWait/T2Write are pinned to the real-Z80 settings rather than the
// core's own defaults, which select Mode 1 ("Fast Z80").
//
// Everything clocked here advances only on `ce` (= ce_3m072), except the
// savestate path, which passes straight through to the core un-gated.
// `reset` is active high and is inverted onto the core's asynchronous
// `reset_n`, which is what lets the core be held reset while `ce` is low, as
// the savestate restore path needs.  `wait_n` is a real input even though
// `ngp_snd` ties it high.

module ngp_z80
(
	input  wire        clk,            // clk_sys
	input  wire        ce,             // ce_3m072
	input  wire        reset,          // active high; ~z80_run

	input  wire        int_n,
	input  wire        nmi_n,
	input  wire        wait_n,

	output wire [15:0] addr,
	input  wire [7:0]  din,
	output wire [7:0]  dout,
	output wire        m1_n,
	output wire        mreq_n,
	output wire        iorq_n,
	output wire        rd_n,
	output wire        wr_n,
	output wire        rfsh_n,
	output wire        halt_n,

	output wire        instr_boundary, // 1 on the ce tick that retires an insn

	// Simulation-only observability ports; consumed by the simulation
	// harness, unused in the shipped design.
	output wire        naive_boundary,
	output wire        prefix_active,
	output wire        int_taken,
	output wire [6:0]  mc,
	output wire [6:0]  ts,

	// savestate tap, architectural state only (words 0x1-0x8)
	input  wire [3:0]  ss_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata
);

	wire       core_iorq;
	wire       core_no_read;
	wire       core_write;
	wire       core_intcycle_n;
	wire [7:0] core_di;

	// Core outputs this machine has no use for.  Named rather than left as
	// empty port connections so the lint is clean with no waiver: the NGP has
	// no bus master to acknowledge, nothing reads the interrupt-enable pin,
	// and `stop` belongs to the core's Game Boy mode.
	wire       core_busak_n;
	wire       core_inte;
	wire       core_stop;
	wire       unused_ok = &{1'b1, core_busak_n, core_inte, core_stop};

	reg  [7:0] di_reg;
	reg        mreq_n_r;
	reg        iorq_n_r;
	reg        rd_n_r;
	reg        wr_n_r;

	assign mreq_n = mreq_n_r;
	assign iorq_n = iorq_n_r;
	assign rd_n   = rd_n_r;
	assign wr_n   = wr_n_r;
	assign core_di = di_reg;

	tv80_core #(
		.Mode   (0),   // 0 = Z80.  The core's own default is 1, "Fast Z80".
		.IOWait (1)    // 1 = the real 4 T I/O cycle with its automatic wait
	) u_core
	(
		.reset_n        (~reset),
		.clk            (clk),
		.cen            (ce),
		.wait_n         (wait_n),
		.int_n          (int_n),
		.nmi_n          (nmi_n),
		.busrq_n        (1'b1),

		.m1_n           (m1_n),
		.iorq           (core_iorq),
		.no_read        (core_no_read),
		.write          (core_write),
		.rfsh_n         (rfsh_n),
		.halt_n         (halt_n),
		.busak_n        (core_busak_n),
		.A              (addr),
		.dinst          (din),      // combinational, for the opcode decode
		.di             (core_di),  // registered, for data reads
		.dout           (dout),
		.mc             (mc),
		.ts             (ts),
		.intcycle_n     (core_intcycle_n),
		.IntE           (core_inte),
		.stop           (core_stop),

		.instr_boundary (instr_boundary),
		.naive_boundary (naive_boundary),
		.prefix_active  (prefix_active),
		.int_taken      (int_taken),
		.ss_addr        (ss_addr),
		.ss_wdata       (ss_wdata),
		.ss_wren        (ss_wren),
		.ss_rdata       (ss_rdata)
	);

	// T2Write = 1: /WR falls in T2, which is what the Z80 manual's write
	// cycle shows (Z80 manual p.11, "Memory Write Cycle").
	always @(posedge clk) begin
		if (reset) begin
			rd_n_r   <= 1'b1;
			wr_n_r   <= 1'b1;
			iorq_n_r <= 1'b1;
			mreq_n_r <= 1'b1;
			di_reg   <= 8'h00;
		end else if (ce) begin
			rd_n_r   <= 1'b1;
			wr_n_r   <= 1'b1;
			iorq_n_r <= 1'b1;
			mreq_n_r <= 1'b1;

			if (mc[0]) begin
				// M1.  An interrupt-acknowledge cycle asserts IORQ with M1
				// and neither RD nor WR, which is what lets the TO3 latch
				// clear ignore it.
				if (ts[1] || (ts[2] && !wait_n)) begin
					rd_n_r   <= ~core_intcycle_n;
					mreq_n_r <= ~core_intcycle_n;
					iorq_n_r <= core_intcycle_n;
				end
`ifdef TV80_REFRESH
				// The refresh half of M1: MREQ pulses in T3 with the I:R
				// address on the bus and no RD or WR.  Nothing in this
				// machine watches it, but the pin is real.
				if (ts[3]) begin
					mreq_n_r <= 1'b0;
				end
`endif
			end else begin
				if ((ts[1] || (ts[2] && !wait_n)) && !core_no_read && !core_write) begin
					rd_n_r   <= 1'b0;
					iorq_n_r <= ~core_iorq;
					mreq_n_r <= core_iorq;
				end
				if ((ts[1] || (ts[2] && !wait_n)) && core_write) begin
					wr_n_r   <= 1'b0;
					iorq_n_r <= ~core_iorq;
					mreq_n_r <= core_iorq;
				end
			end

			if (ts[2] && wait_n && !core_write && !core_no_read) begin
				di_reg <= din;
			end
		end
	end

endmodule
