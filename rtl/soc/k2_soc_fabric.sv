// Copyright (c) 2026 Jamie Blanks

// k2_soc_fabric -- the K2-CHIP's internal bus fabric and its on-chip memories.
//
// The CPU is the ONLY master on the CPU-side bus, so there is no arbiter and no
// arbitration latency on that path. Every other agent -- the BIOS image loader,
// the savestate walker, the Z80 -- reaches memory through port B of the block
// RAM it wants. Two rules follow, both load-bearing:
//   1. no fabric agent may drive a memory's port A; anything that must see the
//      CPU's own view takes the pause path instead;
//   2. the only true arbitration point in the machine is the cart SDRAM, and it
//      lives inside ngp_cart_sdram. Nothing here arbitrates.
//
// The internal decode runs AHEAD of ngp_csc, unconditionally. ngp_csc answers
// for the external bus only: MSAR/MAMR reach A8 and describe external windows,
// so they cannot decode the internal regions and must not be consulted for
// them. B2CS resets to 0x10 -- CS2 enabled, B2M = 0 -- which claims
// 0x000080-0xFFFFFF with a wait field of 00 = 2 waits (TMP95C061 datasheet
// pp.186-187), and the BIOS does not disable it until write 19 of its init
// table at 0xFF2093. A fabric that asked ngp_csc first would therefore run the
// first ~19 instruction fetches as 8-bit 2-wait EXTERNAL cycles and be wrong
// from the reset vector onward. Internal-first makes the BIOS region 16-bit
// 0-wait from the very first fetch, which is what it is (SysPro p.6).
//
// This module inserts exactly csc_waits fixed states and no more: the BIOS
// programs the cart windows B0CS = B1CS = 0x17 for zero waits, and queue-fill
// cost on the 8-bit cart bus is already charged by t900_biu, so adding the
// asymmetric "fetch wait 3" from silicon calibration here would double-count it
// and make cart code roughly twice too slow. A selected cartridge read may
// still extend until `cart_rd_ready`, which is backing-store response latency
// rather than a bus wait. ngp_csc's CART_EXTRA_WAIT parameter exists to test
// the other reading and defaults to 0.
//
// The bus contract this module serves (see rtl/t900/t900_biu.sv):
//   - plan_addr is the address of the NEXT cycle the BIU intends to issue. The
//     fabric answers bus_width8 for it COMBINATIONALLY, in the same cycle. A
//     consumer that answers one cycle late silently issues 16-bit cycles inside
//     the 8-bit cart region and erases the entire cart-fetch penalty.
//   - bus_req holds bus_addr / bus_be / bus_we / bus_wdata stable from T1 until
//     bus_rdy.
//   - The CSC minimum behind bus_rdy is registered and sampled in T2. A
//     zero-wait region reaches that state right after issue (2 states per
//     cycle); N wait states hold it low for N more states. Selected cartridge
//     reads additionally qualify the live T2 result with cart_rd_ready, so a
//     response already present in that nominal window does not cost a third
//     state and a late SDRAM response still extends the cycle.
//   - 8-bit regions transfer on bus_rdata[7:0] / bus_wdata[7:0] with
//     bus_be = 2'b01. In 16-bit regions an odd single byte rides bits [15:8]
//     with be = 2'b10, an even single byte rides [7:0] with be = 2'b01, and an
//     aligned word uses be = 2'b11.
//   - byte writes arrive REPLICATED on both lanes, so the memories may take the
//     byte from whichever lane their byte enable selects.
//   - bus_rdata only has to be valid at the T2 ce edge, not at T1, which is
//     what lets every block RAM here use the wrapper's own output register:
//     the address is stable from the T1 edge, the RAM answers one clk later,
//     and T2 is eight clk cycles away at full gear.
//
// Choices that are choices rather than documented facts:
//   - Open bus reads P_OPEN_BUS = 0xFF per byte. It is a stateless constant, so
//     the fabric holds no open-bus register and needs no savestate word. In an
//     8-bit region the high lane is undriven and reads it too, though the BIU
//     never looks (width8 forces be = 01).
//   - An EXTERNAL read that no chip select claims is a third address class and
//     reads P_EXT_UNSEL = 0x00, not the cartridge edge. See the read mux.
//   - The internal I/O page tail 0x0000C0-0x0000FF stays INTERNAL (8-bit, zero
//     wait, reads 0xFF, writes dropped) rather than falling through to the
//     external bus. SysPro p.8 makes the whole 256-byte page "Internal I/O" but
//     documents nothing above 0xBF.
//   - CPU writes into the BIOS window are dropped. It is mask ROM.
//
// `region` is a PURE function of its argument: it reads no module register, so
// the continuous assigns that call it re-evaluate correctly. Keep it that way.

module k2_soc_fabric
#(
	// Value returned by any read the fabric itself does not source. See the
	// choices note above.
	parameter [7:0] P_OPEN_BUS = 8'hFF,

	// Value an EXTERNAL read takes when the CSC asserts no chip select, i.e.
	// nothing on the board is addressed and the cartridge's nOE never falls.
	// This is NOT the cartridge edge's pulled-up 0xFF: that value belongs to a
	// SELECTED cart cycle no die answers, which is a different situation.
	parameter [7:0] P_EXT_UNSEL = 8'h00
)
(
	input  wire        clk,            // clk_sys, 49.152 MHz
	input  wire        ce,             // ce_t900, fosc/2 / gear state enable
	input  wire        reset,

	// ---- CPU bus, the t900_biu contract ---------------------------------
	input  wire [23:0] plan_addr,
	output wire        bus_width8,     // combinational answer for plan_addr
	// High on exactly those states the cycle in flight is being held by the
	// K2GE composition pass rather than by the memory itself.  See the
	// `bus_wait_gfx` note below the wait-state FSM.
	output wire        bus_wait_gfx,
	input  wire        bus_req,
	input  wire        bus_we,
	input  wire [23:0] bus_addr,
	input  wire [1:0]  bus_be,
	input  wire [15:0] bus_wdata,
	output wire [15:0] bus_rdata,
	output wire        bus_rdy,

	// ---- ngp_csc, which lives inside t900_mcu ---------------------------
	// The external bus only. Both inputs are ignored whenever the internal
	// decode hits -- see the decode-order note in the header.
	input  wire        csc_plan_width8,
	input  wire [2:0]  csc_waits,
	output wire        bus_active,     // chip-select qualifier, external only

	// ---- MCU SFR bus, 0x000000-0x00007F ----------------------------------
	// sfr_wr / sfr_rd are the one-clk_sys T2 strobes the SFR bus convention
	// requires, so no adapter is needed anywhere.
	output wire [6:0]  io_addr,        // = bus_addr[6:0]
	output wire [7:0]  io_wdata,
	output wire        sfr_wr,
	output wire        sfr_rd,
	input  wire [7:0]  sfr_rdata,      // combinational

	// ---- SNK block, 0x000080-0x0000BF ------------------------------------
	// 0x80-0x8F, 0x9C-0x9F and 0xA0-0xBF -> ngp_sysreg; 0x90-0x9B -> ngp_rtc.
	// io_addr reaches both as 0x00-0x3F within the block.
	output wire        sysreg_wr,
	output wire        sysreg_rd,
	input  wire [7:0]  sysreg_rdata,
	output wire        rtc_wr,
	output wire        rtc_rd,
	input  wire [7:0]  rtc_rdata,

	// ---- K2GE register/VRAM window, 0x008000-0x00BFFF --------------------
	output wire [13:0] gfx_addr,
	output wire [15:0] gfx_wdata,
	output wire        gfx_cs,
	output wire        gfx_rd,
	output wire [1:0]  gfx_we,         // [0] even byte, [1] odd byte
	input  wire [15:0] gfx_rdata,
	// Wait states the K2GE owes the cycle currently on the bus. Zero for every
	// internal region except character/scroll/sprite VRAM during the drawing
	// period; `gfx_cs` is false for the others, so this is safe to apply to
	// every internal cycle.
	input  wire [2:0]  gfx_wait,

	// ---- external bus: cart CS0/CS1 and every reserved range -------------
	output wire [23:0] ext_addr,
	output wire [7:0]  ext_wdata,
	output wire        ext_oe,         // 1 = the SoC drives ext_wdata
	output wire        ext_rd,         // T2 strobe
	output wire        ext_wr,         // T2 strobe
	input  wire [7:0]  ext_rdata,      // resolved value incl. connector pull-ups
	input  wire        cart_selected,  // live CSC answer: CS0 or CS1 asserted
	input  wire        cart_rd_ready,  // selected cart read response is valid

	// ---- Z80 side of the shared RAM (port B) -----------------------------
	// The sound block drives these; see the zram instance for the arbitration
	// rule. rdata is valid one clk after addr, exactly like the CPU side.
	input  wire [11:0] z80_ram_addr,
	input  wire        z80_ram_wr,
	input  wire [7:0]  z80_ram_wdata,
	output wire [7:0]  z80_ram_rdata,

	// ---- BIOS image loader (ioctl side) ----------------------------------
	input  wire        mono_strap,     // 0 = colour image, 1 = mono; reset strap
	input  wire        bios_wr,
	input  wire        bios_sel,       // which image the loader is filling
	input  wire [14:0] bios_addr,      // word address
	input  wire [15:0] bios_data,

	// ---- savestate region tap --------------------------------------------
	// Byte serial, one clk of latency, on port B of the memory selected by
	// ss_mem_type. Type 2 (video) is answered by k2ge, not here; this port
	// reads 0 for it.
	input  wire [1:0]  ss_mem_type,    // 0 work RAM, 1 shared RAM, 2 video
	input  wire        ss_mem_active,
	input  wire [13:0] ss_mem_addr,
	input  wire [7:0]  ss_mem_wdata,
	input  wire        ss_mem_wren,
	input  wire        ss_mem_rden,
	output wire [7:0]  ss_mem_rdata,
	input  wire        restore_hold,
	input  wire        pause_req,
	output wire        pause_ready
);

	// Region decode
	//
	// Equality compares only -- no adders, no magnitude chains -- so the
	// combinational plan_addr -> bus_width8 path is one LUT level.
	//
	//   0x000000-0x00007F  MCU SFRs           8-bit  0 wait
	//   0x000080-0x0000BF  SNK block          8-bit  0 wait
	//   0x0000C0-0x0000FF  internal I/O tail  8-bit  0 wait  (open bus)
	//   0x004000-0x006FFF  work RAM 12 KB    16-bit  0 wait
	//   0x007000-0x007FFF  Z80 shared RAM     8-bit  0 wait
	//   0x008000-0x00BFFF  K2GE              16-bit  0 wait
	//   0xFF0000-0xFFFFFF  BIOS ROM 64 KB    16-bit  0 wait
	//   everything else    external          ngp_csc answers

	localparam [2:0] RGN_EXT    = 3'd0;
	localparam [2:0] RGN_SFR    = 3'd1;
	localparam [2:0] RGN_SNK    = 3'd2;
	localparam [2:0] RGN_IOTAIL = 3'd3;
	localparam [2:0] RGN_WRAM   = 3'd4;
	localparam [2:0] RGN_ZRAM   = 3'd5;
	localparam [2:0] RGN_GFX    = 3'd6;
	localparam [2:0] RGN_BIOS   = 3'd7;

	// Pure function of its argument; see the header.
	//
	// a[5:0] takes no part in the decode: the smallest region here is the
	// 64-byte SNK block, so nothing below A6 can change which region an address
	// lands in. Sub-decodes that DO need those bits (the RTC inside the SNK
	// block, the byte lane inside a memory) read them directly from bus_addr.
	/* verilator lint_off UNUSEDSIGNAL */
	function automatic [2:0] region(input [23:0] a);
	/* verilator lint_on UNUSEDSIGNAL */
		reg pg0;
		reg io;
		reg lo16;
	begin
		pg0  = (a[23:16] == 8'h00);
		io   = pg0 && (a[15:8] == 8'h00);
		lo16 = pg0 && (a[15:14] == 2'b01);

		if (io) begin
			// 0x00-0x7F SFRs, 0x80-0xBF SNK block, 0xC0-0xFF the tail.
			region = !a[7] ? RGN_SFR : (!a[6] ? RGN_SNK : RGN_IOTAIL);
		end else if (lo16) begin
			// 0x4000-0x6FFF work RAM, 0x7000-0x7FFF Z80 shared RAM.
			region = (a[13:12] == 2'b11) ? RGN_ZRAM : RGN_WRAM;
		end else if (pg0 && (a[15:14] == 2'b10)) begin
			region = RGN_GFX;
		end else if (a[23:16] == 8'hFF) begin
			region = RGN_BIOS;
		end else begin
			region = RGN_EXT;
		end
	end
	endfunction

	// The width answer is about the address the BIU is ABOUT to drive.
	wire [2:0] plan_rgn      = region(plan_addr);
	wire       plan_internal = (plan_rgn != RGN_EXT);
	// RGN_ZRAM is deliberately not here: the Z80 shared RAM is 16-bit to the
	// CPU, as measured -- see the width note on its instance below. RGN_SNK is
	// 16-bit to the CPU too: the 3-byte short-absolute word store `ld (0xA2),WA`
	// costs 2.00 states marginal on console against the 3.98 a byte-serial model
	// charges, so the word transfers in ONE internal cycle. The byte-serial
	// ngp_sysreg/ngp_rtc interfaces are fed by the two-phase lane shim below.
	// SFR stays 8-bit (documented on the TMP95C061); IOTAIL stays 8-bit, with
	// nothing mapped there to measure.
	wire       plan_int_w8   = (plan_rgn == RGN_SFR) ||
	                           (plan_rgn == RGN_IOTAIL);

	// Internal first, unconditionally. The external answer is ngp_csc's, and on
	// this board it is always 8-bit because the AM8/16 pin is tied high
	// (TMP95C061 datasheet p.7 section 3.1.2, p.51 item 2). It is kept as a real
	// signal rather than a constant because ngp_csc is implemented honestly and
	// a bench can untie AM8/16.
	assign bus_width8 = plan_internal ? plan_int_w8 : csc_plan_width8;

	// Everything except the width answer decodes the address ON the bus.
	wire [2:0] bus_rgn      = region(bus_addr);
	wire       bus_internal = (bus_rgn != RGN_EXT);

	wire sel_sfr    = (bus_rgn == RGN_SFR);
	wire sel_snk    = (bus_rgn == RGN_SNK);
	wire sel_iotail = (bus_rgn == RGN_IOTAIL);
	wire sel_wram   = (bus_rgn == RGN_WRAM);
	wire sel_zram   = (bus_rgn == RGN_ZRAM);
	wire sel_gfx    = (bus_rgn == RGN_GFX);
	wire sel_ext    = (bus_rgn == RGN_EXT);
	// There is deliberately no sel_bios: the BIOS is mask ROM, so it has no
	// write strobe, and the read mux selects it from bus_rgn directly.

	// SNK block sub-decode: 0x90-0x9B is the RTC, the rest of 0x80-0xBF is
	// ngp_sysreg.
	wire sel_rtc    = sel_snk && (bus_addr[5:4] == 2'b01) &&
	                  (bus_addr[3:0] <= 4'hB);
	wire sel_sysreg = sel_snk && !sel_rtc;

	// Work RAM is two instances; the seam is at 0x006000.
	wire sel_wram_hi = sel_wram && bus_addr[13];
	wire sel_wram_lo = sel_wram && !bus_addr[13];

	// Wait-state generation
	//
	// One cycle in flight at a time; the BIU never overlaps.
	//
	// Wait encodings come from the winning block's BxWx field, resolved inside
	// ngp_csc (TMP95C061 datasheet p.52): 00 -> 2, 01 -> 1, 10 -> 1 plus
	// WAIT-pin extension, 11 -> 0. Nothing on the NGP board drives the WAIT pin,
	// so mode 10 degenerates to 1. The fabric applies csc_waits verbatim and
	// adds nothing; `cart_rd_ready` is an independent response-valid condition
	// and may extend only a selected read.

	reg       cyc_active;
	reg       bus_rdy_q;
	reg [2:0] wcnt;

	// Every internal region is zero-wait except character/scroll/sprite VRAM
	// while the K2GE's composition pass owns it, which costs the CPU two states.
	// The K2GE reports that on `gfx_wait` and already returns 0 for its own
	// registers, its palette and every cycle outside the drawing period -- but
	// the region gate here is the fabric's own decode, so no other internal
	// region can ever be charged by it even if that pin misbehaves.
	wire [2:0] internal_waits = sel_gfx ? gfx_wait : 3'd0;

	// `cart_selected` is the live CSC decision for this stable bus cycle. It
	// deliberately is not inferred from address ranges here: firmware can
	// reprogram the external chip-select windows. Writes, internal cycles, and
	// unselected/reserved external reads never consume the cartridge response
	// handshake.
	wire selected_cart_read = !bus_internal && !bus_we && cart_selected;
	wire cart_response_ok    = !selected_cart_read || cart_rd_ready;

	always @(posedge clk) begin
		if (reset) begin
			cyc_active <= 1'b0;
			bus_rdy_q  <= 1'b0;
			wcnt       <= 3'd0;
		end else if (ce) begin
			if (!cyc_active && bus_req) begin
				// T1: latch the wait budget of the region on the bus.
				cyc_active <= 1'b1;
				wcnt       <= bus_internal ? internal_waits : csc_waits;
				bus_rdy_q  <= bus_internal ? (internal_waits == 3'd0) :
				              (csc_waits == 3'd0);
			end else if (cyc_active && bus_rdy_q && cart_response_ok) begin
				// T2 consumed by the BIU.
				cyc_active <= 1'b0;
				bus_rdy_q  <= 1'b0;
			end else if (cyc_active) begin
				if (wcnt != 3'd0) begin
					if (wcnt == 3'd1) begin
						wcnt <= 3'd0;
						bus_rdy_q <= 1'b1;
					end else begin
						wcnt <= wcnt - 3'd1;
					end
				end else if (cart_response_ok) begin
					bus_rdy_q <= 1'b1;
				end
			end
		end
	end

	assign bus_rdy = bus_rdy_q && cart_response_ok;

	// Which wait states are ARBITRATION rather than MEMORY
	//
	// The K2GE's drawing-period hold is not a property of the memory: the
	// composition pass owns the array and hands it back. Every other wait on
	// this bus -- a CSC-programmed cart wait, an 8-bit region's second bus
	// cycle -- is the memory being slow, and an instruction has to pay for it.
	//
	// The sequencer needs the two separated because a repeat form absorbs the
	// first and not the second: on silicon a block move within the gfx window
	// costs exactly 0 extra states per iteration, while the same instruction
	// sourcing from the 8-bit cartridge runs 2 states per iteration slower.
	// Charging both, or exempting both, contradicts one of those two results;
	// see `mem_over_charge` in rtl/t900/t900_seq.sv.
	//
	// This is a LEVEL over the wait states of a gfx cycle: T1 leaves `bus_rdy_q`
	// low and the FSM raises it on the last wait state, so the level is high for
	// exactly `gfx_wait` states.
	assign bus_wait_gfx = cyc_active && sel_gfx && !bus_rdy_q;

	// The T2 strobes -- the only writes in the fabric
	//
	// One clk_sys cycle wide, landing on the T2 ce edge. The SFR bus convention
	// is exactly this shape, so the MCU shell and the SNK block need no adapter.
	// A register write is NOT additionally gated by ce anywhere downstream: the
	// strobe is the enable.

	wire cycle_end = ce && cyc_active && bus_rdy_q && cart_response_ok;
	wire acc_wr    = cycle_end && bus_we;
	wire acc_rd    = cycle_end && !bus_we;

	// ---- the SNK word-cycle lane shim --------------------------------
	// The region is 16-bit on the bus (one 2-state word cycle, be = 11)
	// while ngp_sysreg/ngp_rtc keep their byte-serial faces: the LOW lane
	// is delivered in the cycle's FIRST state (the T1 ce edge) and the
	// HIGH lane in its second (the T2 edge).  A byte access in the 16-bit
	// region selects its lane from the byte enables; the SFR page is
	// untouched.  The read side latches the low byte at the T1 edge and
	// combines at T2 (both sources are combinational in io_addr).
	wire acc_t1      = ce && !cyc_active && bus_req && cart_response_ok;
	wire snk_word    = (bus_rgn == RGN_SNK) && (bus_be == 2'b11);
	wire snk_hi_lane = snk_word ? cyc_active : bus_be[1];
	wire snk_t1_wr   = acc_t1 && bus_we && snk_word;

	assign io_addr   = (bus_rgn == RGN_SNK)
	                   ? {bus_addr[6:1], snk_hi_lane}
	                   : bus_addr[6:0];
	assign io_wdata  = ((bus_rgn == RGN_SNK) && snk_hi_lane)
	                   ? bus_wdata[15:8]
	                   : bus_wdata[7:0];  // 8-bit regions: the BIU replicates

	reg [7:0] snk_lo_q;
	always @(posedge clk) begin
		if (acc_t1 && !bus_we && snk_word)
			snk_lo_q <= sel_rtc ? rtc_rdata : sysreg_rdata;
	end

	assign sfr_wr    = acc_wr && sel_sfr;
	assign sfr_rd    = acc_rd && sel_sfr;
	assign sysreg_wr = (acc_wr || snk_t1_wr) && sel_sysreg;
	assign sysreg_rd = acc_rd && sel_sysreg;
	assign rtc_wr    = (acc_wr || snk_t1_wr) && sel_rtc;
	assign rtc_rd    = acc_rd && sel_rtc;

	// K2GE. cpu_cs is a level held for the whole cycle; the byte enables carry
	// the write strobe, which is what the register file samples.
	assign gfx_addr  = bus_addr[13:0];
	assign gfx_wdata = bus_wdata;
	assign gfx_cs    = bus_req && sel_gfx;
	assign gfx_rd    = acc_rd && sel_gfx;
	assign gfx_we    = {2{acc_wr && sel_gfx}} & bus_be;

	// External bus. Address and write data are levels valid from T1; the
	// strobes mark T2.
	assign ext_addr  = bus_addr;
	assign ext_wdata = bus_wdata[7:0];
	assign ext_oe    = bus_req && sel_ext && bus_we;
	assign ext_rd    = acc_rd && sel_ext;
	assign ext_wr    = acc_wr && sel_ext;

	// Connector coherence -- the chip select may not strobe over a bus that
	// has not settled yet
	// Only an external cycle may assert a chip select; an internal access never
	// leaves the die. But WHEN it may assert is not simply "as soon as bus_req
	// is high", and this is the one place in the machine where that matters.
	//
	// Every other consumer of the CPU bus samples it on a `ce` edge -- this
	// module's cycle latch and wait budget, its T2 strobes, every memory address
	// port. `ngp_cart` does not: it is instantiated with `ce = 1'b1` on purpose,
	// so its dies behave like the asynchronous parts they are and see every
	// strobe edge. It therefore watches this bus BETWEEN ce edges, and between
	// ce edges the bus is not coherent: t900_cpu holds its bus outputs in a
	// register on the opposite phase of `ce` (SPLIT_BUS_OUT), fed through the
	// register file's own opposite-phase hold (SPLIT_RF_READ), and those holds
	// reload on every non-`ce` clk_sys. So for up to 5 clk_sys after each `ce`
	// edge, bus_req / bus_we / bus_addr are a MIXTURE of the cycle that is
	// ending and the one that is starting.
	//
	// Presented to the connector, that mixture is a PHANTOM CYCLE: a chip select
	// over an address no software issued -- a spurious flash command write, or a
	// read of a cartridge address the CPU is only passing through on its way to
	// an internal one. The die cannot recall a fetch it has started, so the byte
	// that comes back belongs to the phantom and is handed to the CPU as the
	// NEXT read's answer.
	//
	// The rule: the connector is shown a cycle only once the bus carrying it has
	// settled. `conn_*_q` below is a copy of the bus that refuses to load during
	// the unsettled window, and the chip select is qualified by the live bus
	// still agreeing with it, so a value that exists only inside the window can
	// never be copied in and never be presented.
	//
	// This costs the CPU nothing, structurally rather than by luck. At every
	// `ce` edge the hold is current, so `bus_active` at a `ce` edge is
	// bit-for-bit what it would be without the gate -- and a `ce` edge is the
	// only time anything on the die looks at it. What changes is only the
	// sub-state waveform the cartridge sees: a cycle is presented SETTLE_CLKS+1
	// clk_sys into its T1 instead of one or two. A zero-wait cartridge cycle is
	// 32 clk_sys long and a die's array fetch is a flat 8 clk_sys from read
	// event to `rd_ready`, so the fetch still finishes with more than half the
	// cycle to spare.
	//
	// SETTLE_CLKS is one clk_sys past the observed maximum. It is a property of
	// the CPU's output pipeline -- two opposite-phase holds -- not of any
	// program, so it does not vary with the software being run.
	//
	// The `ce` term below keeps this correct when `ce` is FASTER than the
	// window; a bench may run it one clk_sys in four, and t900_cpu only promises
	// four. A `ce` edge is coherent by definition (it is the end of a state and
	// the holds have long since loaded), so it always loads. At that ratio the
	// counter never arms and the connector is presented from the state the
	// fabric latched the cycle in, which is late but never wrong. At the ratio
	// the machine actually runs, one clk_sys in sixteen, the counter arms first
	// and the `ce` term is redundant.
	localparam [2:0] SETTLE_CLKS = 3'd5;

	reg  [2:0]  settle_cnt;
	reg  [23:0] conn_addr_q;
	reg         conn_req_q;
	reg         conn_we_q;

	wire bus_settled = ce || (settle_cnt >= SETTLE_CLKS);

	always @(posedge clk) begin
		if (reset) begin
			settle_cnt  <= 3'd7;
			conn_addr_q <= 24'd0;
			conn_req_q  <= 1'b0;
			conn_we_q   <= 1'b0;
		end else begin
			if (ce)                       settle_cnt <= 3'd0;
			else if (settle_cnt != 3'd7)  settle_cnt <= settle_cnt + 3'd1;

			if (bus_settled) begin
				conn_addr_q <= bus_addr;
				conn_req_q  <= bus_req;
				conn_we_q   <= bus_we;
			end
		end
	end

	// The live bus agreeing with the settled copy. Deliberately NOT gated by
	// `bus_settled` itself: a cycle already in flight must stay presented
	// through the unsettled window at the top of each of its states, and it
	// does, because nothing about it has changed. Only a bus that has MOVED
	// inside the window is held off, and it is held off until it settles.
	wire conn_coherent = (bus_addr == conn_addr_q) &&
	                     (bus_req  == conn_req_q)  &&
	                     (bus_we   == conn_we_q);

	assign bus_active = bus_req && sel_ext && conn_coherent;

	// Pause participation
	//
	// The fabric holds only the in-flight cycle state, so this is a check, not
	// a wait: it cannot be busy if the CPU is not. The memories are
	// single-cycle with no queue and are ready immediately, which is why they
	// contribute no term. Same convention as the other blocks: pause_ready is
	// low unless pause_req is asserted.

	assign pause_ready = pause_req && !cyc_active;

	// A restore is honoured only while the whole fabric is parked. The reset term
	// is for the deterministic cold-game-load clear: the machine is held in
	// reset while ngp_wram_clear walks this same port B. It does not make a live
	// savestate restore legal without a pause.
	wire ss_wr_eff = ss_mem_active && ss_mem_wren &&
		(pause_ready || reset || restore_hold);
	wire ss_wr_wram = ss_wr_eff && (ss_mem_type == 2'd0);
	wire ss_wr_zram = ss_wr_eff && (ss_mem_type == 2'd1);

	// The savestate tap is byte serial on a 16-bit memory, so the byte lane is
	// the low address bit and the write data is replicated onto both lanes.
	wire [1:0]  ss_be    = ss_mem_addr[0] ? 2'b10 : 2'b01;
	wire [15:0] ss_wdata = {ss_mem_wdata, ss_mem_wdata};

	// Work RAM -- 12 KB, 16-bit, zero wait  (SysPro p.6)
	//
	// 12 KB is not a power of two and the supplied wrapper is power-of-two only
	// (NUM_WORDS = 1 << ADDR_WIDTH), so a single instance would be 8192 x 16 =
	// 16 M10K with a quarter wasted. Two instances cost 12 blocks:
	//
	//   wram_lo  0x004000-0x005FFF  4096 x 16   8 M10K
	//   wram_hi  0x006000-0x006FFF  2048 x 16   4 M10K
	//
	// Byte enables are mandatory: the BIOS does byte writes and read-modify-
	// write bit operations all over its workspace (`res 6,(0x6F83)`,
	// `ld (0x6C7C),0xA55A`).
	//
	// Port A is the CPU. Port B is the savestate byte tap and nothing else.

	wire [15:0] wram_lo_q;
	wire [15:0] wram_lo_qb;

	cache_ram_dp_be #(.ADDR_WIDTH(12), .DATA_WIDTH(16)) wram_lo
	(
		.clk_i     (clk),
		.addr_a_i  (bus_addr[12:1]),
		.wren_a_i  (acc_wr && sel_wram_lo),
		.be_a_i    (bus_be),
		.wdata_a_i (bus_wdata),
		.q_a_o     (wram_lo_q),
		.addr_b_i  (ss_mem_addr[12:1]),
		.wren_b_i  (ss_wr_wram && !ss_mem_addr[13]),
		.be_b_i    (ss_be),
		.wdata_b_i (ss_wdata),
		.q_b_o     (wram_lo_qb)
	);

	wire [15:0] wram_hi_q;
	wire [15:0] wram_hi_qb;

	cache_ram_dp_be #(.ADDR_WIDTH(11), .DATA_WIDTH(16)) wram_hi
	(
		.clk_i     (clk),
		.addr_a_i  (bus_addr[11:1]),
		.wren_a_i  (acc_wr && sel_wram_hi),
		.be_a_i    (bus_be),
		.wdata_a_i (bus_wdata),
		.q_a_o     (wram_hi_q),
		.addr_b_i  (ss_mem_addr[11:1]),
		.wren_b_i  (ss_wr_wram && ss_mem_addr[13]),
		.be_b_i    (ss_be),
		.wdata_b_i (ss_wdata),
		.q_b_o     (wram_hi_qb)
	);

	// Z80 shared RAM -- 4 KB, 16-bit to the CPU, 8-bit to the Z80, zero wait
	// The CPU port width is measured, not documented. SysPro p.6 lists this RAM
	// as "4KB (8bit, 0 wait, accessible through the main CPU)", but that line
	// sits inside the document's SOUND CONTROL CPU block, so "8bit" reads at
	// least as easily as the Z80's port width -- a tautology for a Z80 -- as it
	// does the main CPU's. Zero wait is not in dispute; only the width is. The
	// Toshiba part cannot settle it either: a stock TMP95C061 has "Internal RAM:
	// None, Internal ROM: None" (datasheet p.1 section 1), so every byte of NGPC
	// RAM is SNK silicon inside the K2-CHIP, and the board carries no external
	// SRAM part whose package could force a byte-wide port.
	//
	// On silicon, identical code staged at 0x005000 and at 0x007400 executes at
	// identical speed -- 0.00 states per instruction apart over two console
	// passes -- while an 8-bit CPU port makes the sound-RAM copy 2.43 states per
	// instruction slower. So the CPU port is the same width as work RAM.
	//
	// The Z80 side is genuinely 8-bit: it addresses bytes and selects its lane
	// below.
	//
	// Port B carries the savestate tap and the Z80, sharing it through the mux
	// below rather than through a third BRAM port. THE WALKER WINS while
	// ss_mem_active. That is a static priority, not an arbiter, and it is safe
	// for one reason: the Z80 is stopped whenever a savestate is taken.
	// `ngp_snd` parks its Z80 at an instruction boundary (or holds it in reset)
	// before pause_ready rises and drops z80_ram_wr while parked, so no Z80
	// write can be in flight when the walker takes the port. Giving the Z80
	// priority instead would let a runaway core corrupt a restore, which is the
	// failure that cannot be recovered from.
	//
	// The walker's select is ss_mem_active alone and not the type compare: a
	// walk of the work-RAM region also holds this port, which costs nothing (the
	// Z80 is stopped for all of it) and keeps the mux one comparison deep.

	wire [15:0] zram_q;
	wire [15:0] zram_qb;

	// Port B is byte serial for both of its users, so it presents a word
	// address, a one-hot byte enable and replicated write data -- exactly the
	// convention `ss_be` / `ss_wdata` already use for work RAM above.
	wire        zram_b_walker = ss_mem_active;
	wire [11:0] zram_b_byte   = zram_b_walker ? ss_mem_addr[11:0] : z80_ram_addr;
	wire        zram_b_wren   = zram_b_walker ? ss_wr_zram        : z80_ram_wr;
	wire [7:0]  zram_b_byte_d = zram_b_walker ? ss_mem_wdata      : z80_ram_wdata;
	wire [1:0]  zram_b_be     = zram_b_byte[0] ? 2'b10 : 2'b01;
	wire [15:0] zram_b_wdata  = {zram_b_byte_d, zram_b_byte_d};

	cache_ram_dp_be #(.ADDR_WIDTH(11), .DATA_WIDTH(16)) zram
	(
		.clk_i     (clk),
		.addr_a_i  (bus_addr[11:1]),
		.wren_a_i  (acc_wr && sel_zram),
		.be_a_i    (bus_be),
		.wdata_a_i (bus_wdata),
		.q_a_o     (zram_q),
		.addr_b_i  (zram_b_byte[11:1]),
		.wren_b_i  (zram_b_wren),
		.be_b_i    (zram_b_be),
		.wdata_b_i (zram_b_wdata),
		.q_b_o     (zram_qb)
	);

	// The block RAM answers one clk after its address, so the byte lane has to
	// travel WITH the data rather than be sampled fresh -- the same rule
	// `ss_lane_q` follows below. Selecting on the live address bit would return
	// the right byte only when two consecutive accesses happened to share lane
	// parity, which is an intermittent corruption that survives casual testing.
	reg zram_b_lane_q;
	always @(posedge clk) zram_b_lane_q <= zram_b_byte[0];

	wire [7:0] zram_b_byte_q = zram_b_lane_q ? zram_qb[15:8] : zram_qb[7:0];

	// One port, one output register: the Z80 and the walker read the same wire.
	// They never both need it, for the reason above.
	assign z80_ram_rdata = zram_b_byte_q;

	// BIOS ROM -- two resident 64 KB images, 16-bit, zero wait  (SysPro p.6)
	// 64 KB at 16 bits wide is exactly 2^15 words, so there is no waste and no
	// split: 32768 x 16 = 64 M10K each, 128 for both.
	//
	// Port A is the CPU, read only -- writes into 0xFF0000-0xFFFFFF are dropped
	// because it is mask ROM. The image the CPU sees is strapped at reset and is
	// NOT switchable while running: a mid-run switch would change the code under
	// the PC.
	//
	// Port B is the ioctl loader. With WIDE(1) the framework delivers
	// {file[n+1], file[n]} on a 16-bit port, which is exactly the little-endian
	// word the TLCS-900/H reads, so the parent writes ioctl_dout verbatim and
	// bios_addr is the word address ioctl_addr[15:1]. The check that catches a
	// byte swap immediately is the reset vector: bytes 4A 20 FF 00 at 0xFFFF00
	// must read back as 0x00FF204A.
	//
	// These BRAMs are NOT savestate: their contents come deterministically from
	// boot0.rom / boot1.rom at every core start.

	wire [15:0] bios_col_q;
	wire [15:0] bios_col_qb;

	cache_ram_dp #(.ADDR_WIDTH(15), .DATA_WIDTH(16)) bios_col
	(
		.clk_i     (clk),
		.addr_a_i  (bus_addr[15:1]),
		.wren_a_i  (1'b0),
		.wdata_a_i (16'd0),
		.q_a_o     (bios_col_q),
		.addr_b_i  (bios_addr),
		.wren_b_i  (bios_wr && !bios_sel),
		.wdata_b_i (bios_data),
		.q_b_o     (bios_col_qb)
	);

	wire [15:0] bios_mono_q;
	wire [15:0] bios_mono_qb;

	cache_ram_dp #(.ADDR_WIDTH(15), .DATA_WIDTH(16)) bios_mono
	(
		.clk_i     (clk),
		.addr_a_i  (bus_addr[15:1]),
		.wren_a_i  (1'b0),
		.wdata_a_i (16'd0),
		.q_a_o     (bios_mono_q),
		.addr_b_i  (bios_addr),
		.wren_b_i  (bios_wr && bios_sel),
		.wdata_b_i (bios_data),
		.q_b_o     (bios_mono_qb)
	);

	wire [15:0] bios_q = mono_strap ? bios_mono_q : bios_col_q;

	// CPU read mux
	//
	// A case, never a wired-OR, for the same reason the savestate bus and the
	// SFR bus use one: a stuck slave must fail visibly rather than corrupt
	// everyone else's reads.
	//
	// Every source is valid at the T2 ce edge. The block RAMs answer one clk
	// after their address, which is stable from T1; the SFR, SNK and K2GE
	// sources are combinational functions of an address that is equally stable.

	reg [15:0] rdata_r;

	always @* begin
		case (bus_rgn)
			RGN_SFR:  rdata_r = {P_OPEN_BUS, sfr_rdata};
			// 16-bit region: a word read combines the T1-latched low byte
			// with the live high byte; a byte read rides its own lane.
			RGN_SNK:  rdata_r = snk_word
			                    ? {(sel_rtc ? rtc_rdata : sysreg_rdata), snk_lo_q}
			                    : (snk_hi_lane
			                       ? {(sel_rtc ? rtc_rdata : sysreg_rdata), P_OPEN_BUS}
			                       : {P_OPEN_BUS, (sel_rtc ? rtc_rdata : sysreg_rdata)});
			RGN_WRAM: rdata_r = bus_addr[13] ? wram_hi_q : wram_lo_q;
			RGN_ZRAM: rdata_r = zram_q;
			RGN_GFX:  rdata_r = gfx_rdata;
			RGN_BIOS: rdata_r = bios_q;
			// An unselected external read returns 0x00: the K2's internal data
			// bus is weakly pulled low, and with no chip select asserted the
			// external gates stay closed, so nothing off-chip reaches the CPU.
			// Do not substitute the connector's pull-ups (0xFF) -- that value
			// belongs to a SELECTED cart cycle no die answers -- and do not
			// substitute a last-value-driven model. Biomotor Unitron (USA,
			// Europe) discriminates: it dereferences an unmapped descriptor at
			// 0x050005/0x050006 during the attract sequence's meteor impact and
			// uses the two bytes as a sprite's width and height. 0x00/0x00
			// writes one invisible word, which is what a real handheld shows;
			// 0xFF/0xFF writes the 27 x 27 block that clipping leaves of a
			// 255 x 255 object, and last-value-driven hands the plotter a width
			// of 33 and draws its own stippled block.
			RGN_EXT:  rdata_r = {P_OPEN_BUS,
			                     cart_selected ? ext_rdata : P_EXT_UNSEL};
			// RGN_IOTAIL and anything unreachable: nothing is mapped there.
			default:  rdata_r = {P_OPEN_BUS, P_OPEN_BUS};
		endcase
	end

	assign bus_rdata = rdata_r;

	// Savestate region read path
	//
	// The block RAM answers one clk after its address, so the lane, instance
	// and type selects have to travel with the data rather than being sampled
	// fresh. These three flops track the wrapper's own latency and therefore
	// carry no clock enable -- the wrapper has none either, and gating them
	// would desynchronise the tap from the memory it reads. The savestate
	// walker runs on clk_sys, not on ce.

	reg       ss_lane_q;
	reg       ss_whi_q;
	reg [1:0] ss_type_q;

	always @(posedge clk) begin
		ss_lane_q <= ss_mem_addr[0];
		ss_whi_q  <= ss_mem_addr[13];
		ss_type_q <= ss_mem_type;
	end

	wire [15:0] ss_wram_word = ss_whi_q ? wram_hi_qb : wram_lo_qb;
	wire [7:0]  ss_wram_byte = ss_lane_q ? ss_wram_word[15:8] : ss_wram_word[7:0];

	reg [7:0] ss_rdata_r;

	always @* begin
		case (ss_type_q)
			2'd0:    ss_rdata_r = ss_wram_byte;
			// The shared RAM is a 16-bit memory, so its savestate byte comes
			// through the same lane select the Z80 read uses. Byte N of the walk
			// is still byte N of the region: the lane convention here matches
			// `ss_be`, so savestate byte order is unchanged.
			2'd1:    ss_rdata_r = zram_b_byte_q;
			// Type 2 (video) is answered by k2ge, not here; this port reads 0
			// for it. Type 3 is cart flash and never reaches this module.
			default: ss_rdata_r = 8'h00;
		endcase
	end

	assign ss_mem_rdata = ss_rdata_r;

	// Deliberately unread
	//
	// ss_mem_rden: the block RAM answers unconditionally one clk after its
	//   address, so there is nothing for a read enable to gate. It is kept in
	//   the port list because the savestate engine drives it and the cart region
	//   does need it.
	// bios_col_qb / bios_mono_qb: the loader writes and never reads back.
	// sel_iotail: the read mux reaches it through the case default, and the
	//   strobe fan-out deliberately drops its writes.
	// ss_mem_addr[13] on the shared-RAM path: 4 KB needs 12 bits.
	wire unused_ok = &{1'b0, ss_mem_rden, bios_col_qb, bios_mono_qb,
	                   sel_iotail, 1'b0};

endmodule
