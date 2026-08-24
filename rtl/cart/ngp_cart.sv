// Copyright (c) 2026 Jamie Blanks

// The cartridge as a board: a connector, one or two flash dies, the dirty
// bitmaps, and one mailbox out to whatever holds the cart image.
//
// There is no flash logic here -- every command, every status byte and every
// block boundary lives in flash_die.
//
//   CS0 (0x200000-0x3FFFFF)  ->  die0.nCE
//   CS1 (0x800000-0x9FFFFF)  ->  die1.nCE
//   /OE, /WE, A[20:0], D_in  ->  both dies
//
// The BIOS programs MSAR0/MAMR0 and MSAR1/MAMR1 itself, so the chip selects
// come from ngp_csc, never from a hardcoded decode.
//
// mem_* is the whole storage interface. The adapter on the other end must
// honour:
//
//   1. mem_req is held with its payload stable until mem_done, and mem_tag
//      toggles on every transaction, including a repeat of the same address:
//      ngp_cart_sdram's p0 read hold is invalidated only by the request
//      dropping or the tag changing, so read X / program X / read X under one
//      tag would return the pre-program value.
//   2. Reads return the whole 16-bit word containing mem_addr in mem_rdata,
//      qualified by mem_rvalid at or before mem_done. SDRAM p1's read port is
//      byte-lane selected and returns {8'hff, byte}: the adapter must put that
//      byte back into the half mem_lane names before presenting mem_rdata.
//   3. mem_flash routes the transaction: 1 = an embedded program or erase
//      (SDRAM p1), 0 = a CPU cart read (SDRAM p0). A read is never issued
//      while the die that wants it is busy, so the two never contend.
//   4. Writes commit the lanes named by mem_be, visible to the very next read.
//   5. mem_addr is cart-linear: die 0 at offset 0, die 1 at +0x200000.
//
// d_out is the resolved cartridge-edge value including the pull-ups, so the
// SoC can consume it unconditionally; d_oe reports whether silicon is actually
// driving, for the open-bus model. An absent die drives nothing and the
// connector reads 0xFF, which is both a pulled-up bus and blank flash, so the
// BIOS licence-string check at 0x200000 fails and it lands in the setup path.
// Forcing 0x00 there would be wrong.
//
// Cart size is decided by a magnitude comparison on the downloaded byte count
// with no fall-through (first bucket <= 512 KB -> 0xAB), not by a prog_addr
// high-water mark, which reports a 2 MB device for small homebrew. Die 1 is a
// full peer of die 0 -- same module, same program and erase path, same backing
// store, its own dirty bitmap -- because a read-only second die loses saves on
// the 4 MB titles.
//
// Every die address, including the top-tail save blocks, traverses mem_*, so
// no part of the cartridge image ever has a second live copy.

module ngp_cart
#(
	parameter [7:0]   MANUFACTURER_ID   = 8'h98,
	parameter [7:0]   ID_PROTECT_BYTE   = 8'h02,
	parameter [7:0]   ID_TRAILER_BYTE   = 8'h80,
	parameter [21:0]  ERASE_WORD_PERIOD = 22'd75,
	parameter [9:0]   SS_BASE           = 10'd96   // internals 96-103
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	// --- die population, decoded from the loaded image -------------------
	input  wire [24:0] image_bytes,   // byte count of the image just loaded
	input  wire        config_load,   // one-tick strobe: latch the population
	// Header-qualified physical-capacity exception for original Delta Warp.
	// Its 512-KiB dump executes save paths in an 8-Mbit device's upper blocks.
	input  wire        force_8m_die0,
	// Successful sparse restore: force both idle dies to read-array state.
	input  wire        force_flash_read,
	output wire [1:0]  size_code0,    // BIOS 0x6C58/59 code: 0 absent, 1 4 Mbit,
	output wire [1:0]  size_code1,    //   2 8 Mbit, 3 16 Mbit
	output wire [24:0] cart_bytes,    // total backing bytes behind the connector
	output wire        cart_present,

	// --- cartridge edge ---------------------------------------------------
	input  wire [20:0] A,
	input  wire [7:0]  d_in,
	input  wire        nCE0,
	input  wire        nCE1,
	input  wire        nOE,
	input  wire        nWE,
	output wire [7:0]  d_out,         // resolved, pull-ups included
	output wire        d_oe,          // 1 = a die is actually driving
	output wire        rd_ready,      // 1 = d_out is valid for this read cycle

	// --- backing store ----------------------------------------------------
	output wire        mem_req,
	output reg         mem_we,
	output reg  [24:0] mem_addr,
	output reg  [15:0] mem_wdata,
	output reg  [1:0]  mem_be,
	output reg         mem_lane,
	output reg         mem_tag,
	output reg         mem_flash,
	input  wire [15:0] mem_rdata,
	input  wire        mem_rvalid,
	input  wire        mem_done,

	// --- save layer -------------------------------------------------------
	output reg         dirty_pulse,
	output wire [34:0] dirty0,
	output wire [34:0] dirty1,
	// Exact completion evidence for sparse-overlay snapshot invalidation. The
	// coalesced dirty_pulse remains the MiSTer autosave trigger, but it cannot
	// identify which staged physical block needs a retry.
	output wire        dirty0_event,
	output wire [5:0]  dirty0_block,
	output wire        dirty1_event,
	output wire [5:0]  dirty1_block,
	input  wire        dirty_clear,
	output wire        flash_busy,
	output wire [1:0]  die_busy,

	// --- savestate --------------------------------------------------------
	input  wire [9:0]  ss_bus_adr,
	input  wire [63:0] ss_bus_din,
	input  wire        ss_bus_wren,
	input  wire        ss_bus_rst,
	output wire [63:0] ss_bus_dout,
	input  wire        ss_restore_is_rewind,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [1:0] GR_NONE = 2'd0;
	localparam [1:0] GR_DIE0 = 2'd1;
	localparam [1:0] GR_DIE1 = 2'd2;

	// Die population
	// Densities and block tables come from the FlashMem datasheet (p.5-6).
	// Odd-sized homebrew rounds UP into the next bucket.

	reg        present0_q, present1_q;
	reg [7:0]  dev0_q, dev1_q;
	reg [20:0] mask0_q, mask1_q;
	reg [1:0]  code0_q, code1_q;
	reg [24:0] cart_bytes_q;

	always @(posedge clk) begin
		if (reset) begin
			present0_q   <= 1'b0;
			present1_q   <= 1'b0;
			dev0_q       <= 8'h2F;
			dev1_q       <= 8'h2F;
			mask0_q      <= 21'h1FFFFF;
			mask1_q      <= 21'h1FFFFF;
			code0_q      <= 2'd0;
			code1_q      <= 2'd0;
			cart_bytes_q <= 25'd0;
		end else if (ce && config_load) begin
			if (image_bytes == 25'd0) begin
				present0_q   <= 1'b0;
				present1_q   <= 1'b0;
				dev0_q       <= 8'h2F;
				dev1_q       <= 8'h2F;
				mask0_q      <= 21'h1FFFFF;
				mask1_q      <= 21'h1FFFFF;
				code0_q      <= 2'd0;
				code1_q      <= 2'd0;
				cart_bytes_q <= 25'd0;
			end else if ((image_bytes <= 25'h080000) && !force_8m_die0) begin
				present0_q   <= 1'b1;
				present1_q   <= 1'b0;
				dev0_q       <= 8'hAB;
				dev1_q       <= 8'h2F;
				mask0_q      <= 21'h07FFFF;
				mask1_q      <= 21'h1FFFFF;
				code0_q      <= 2'd1;
				code1_q      <= 2'd0;
				cart_bytes_q <= 25'h080000;
			end else if (image_bytes <= 25'h100000) begin
				present0_q   <= 1'b1;
				present1_q   <= 1'b0;
				dev0_q       <= 8'h2C;
				dev1_q       <= 8'h2F;
				mask0_q      <= 21'h0FFFFF;
				mask1_q      <= 21'h1FFFFF;
				code0_q      <= 2'd2;
				code1_q      <= 2'd0;
				cart_bytes_q <= 25'h100000;
			end else if (image_bytes <= 25'h200000) begin
				present0_q   <= 1'b1;
				present1_q   <= 1'b0;
				dev0_q       <= 8'h2F;
				dev1_q       <= 8'h2F;
				mask0_q      <= 21'h1FFFFF;
				mask1_q      <= 21'h1FFFFF;
				code0_q      <= 2'd3;
				code1_q      <= 2'd0;
				cart_bytes_q <= 25'h200000;
			end else begin
				present0_q   <= 1'b1;
				present1_q   <= 1'b1;
				dev0_q       <= 8'h2F;
				dev1_q       <= 8'h2F;
				mask0_q      <= 21'h1FFFFF;
				mask1_q      <= 21'h1FFFFF;
				code0_q      <= 2'd3;
				code1_q      <= 2'd3;
				cart_bytes_q <= 25'h400000;
			end
		end
	end

	assign size_code0   = code0_q;
	assign size_code1   = code1_q;
	assign cart_bytes   = cart_bytes_q;
	assign cart_present = present0_q;

	// The two dies

	wire [7:0]  d0_dq, d1_dq;
	wire        d0_oe, d1_oe;
	wire        d0_rdy, d1_rdy;

	wire        d0_mreq, d1_mreq;
	wire        d0_mwe,  d1_mwe;
	wire [20:0] d0_maddr, d1_maddr;
	wire [15:0] d0_mwdata, d1_mwdata;
	wire [1:0]  d0_mbe, d1_mbe;
	wire        d0_mlane, d1_mlane;
	wire        d0_mtag, d1_mtag;

	wire        d0_busy, d1_busy;
	wire        d0_dirty, d1_dirty;
	wire [5:0]  d0_dblk, d1_dblk;
	wire [63:0] d0_ssdout, d1_ssdout;
	wire        d0_pause_rdy, d1_pause_rdy;

	reg  [1:0]  grant_q;

	// Canonical backing-store transaction
	// req_q is the arbiter's sole outstanding-transaction flag; every command
	// waits for mem_done.

	reg req_q;

	assign mem_req = req_q;

	wire        d0_mdone   = req_q && mem_done && (grant_q == GR_DIE0);
	wire        d1_mdone   = req_q && mem_done && (grant_q == GR_DIE1);
	wire        d0_mrvalid = req_q && mem_rvalid && (grant_q == GR_DIE0);
	wire        d1_mrvalid = req_q && mem_rvalid && (grant_q == GR_DIE1);

	flash_die
	#(
		.MANUFACTURER_ID(MANUFACTURER_ID),
		.ID_PROTECT_BYTE(ID_PROTECT_BYTE),
		.ID_TRAILER_BYTE(ID_TRAILER_BYTE),
		.ERASE_WORD_PERIOD(ERASE_WORD_PERIOD),
		.SS_BASE(SS_BASE)
	)
	die0
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),
		.force_read_i(force_flash_read),
		.present(present0_q),
		.cfg_device_id(dev0_q),
		.cfg_size_mask(mask0_q),
		.A(A),
		.DQ_in(d_in),
		.DQ_out(d0_dq),
		.DQ_oe(d0_oe),
		.rd_ready(d0_rdy),
		.nCE(nCE0),
		.nOE(nOE),
		.nWE(nWE),
		.mem_req(d0_mreq),
		.mem_we(d0_mwe),
		.mem_addr(d0_maddr),
		.mem_wdata(d0_mwdata),
		.mem_be(d0_mbe),
		.mem_lane(d0_mlane),
		.mem_tag(d0_mtag),
		.mem_rdata(mem_rdata),
		.mem_rvalid(d0_mrvalid),
		.mem_done(d0_mdone),
		.busy(d0_busy),
		.dirty_pulse(d0_dirty),
		.dirty_block(d0_dblk),
		.ss_bus_adr(ss_bus_adr),
		.ss_bus_din(ss_bus_din),
		.ss_bus_wren(ss_bus_wren),
		.ss_bus_rst(ss_bus_rst),
		.ss_bus_dout(d0_ssdout),
		.ss_restore_is_rewind(ss_restore_is_rewind),
		.pause_req(pause_req),
		.pause_ready(d0_pause_rdy)
	);

	flash_die
	#(
		.MANUFACTURER_ID(MANUFACTURER_ID),
		.ID_PROTECT_BYTE(ID_PROTECT_BYTE),
		.ID_TRAILER_BYTE(ID_TRAILER_BYTE),
		.ERASE_WORD_PERIOD(ERASE_WORD_PERIOD),
		.SS_BASE(SS_BASE + 10'd4)
	)
	die1
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),
		.force_read_i(force_flash_read),
		.present(present1_q),
		.cfg_device_id(dev1_q),
		.cfg_size_mask(mask1_q),
		.A(A),
		.DQ_in(d_in),
		.DQ_out(d1_dq),
		.DQ_oe(d1_oe),
		.rd_ready(d1_rdy),
		.nCE(nCE1),
		.nOE(nOE),
		.nWE(nWE),
		.mem_req(d1_mreq),
		.mem_we(d1_mwe),
		.mem_addr(d1_maddr),
		.mem_wdata(d1_mwdata),
		.mem_be(d1_mbe),
		.mem_lane(d1_mlane),
		.mem_tag(d1_mtag),
		.mem_rdata(mem_rdata),
		.mem_rvalid(d1_mrvalid),
		.mem_done(d1_mdone),
		.busy(d1_busy),
		.dirty_pulse(d1_dirty),
		.dirty_block(d1_dblk),
		.ss_bus_adr(ss_bus_adr),
		.ss_bus_din(ss_bus_din),
		.ss_bus_wren(ss_bus_wren),
		.ss_bus_rst(ss_bus_rst),
		.ss_bus_dout(d1_ssdout),
		.ss_restore_is_rewind(ss_restore_is_rewind),
		.pause_req(pause_req),
		.pause_ready(d1_pause_rdy)
	);

	// Bus resolution and pull-ups

	assign d_out    = d0_oe ? d0_dq : d1_oe ? d1_dq : 8'hFF;
	assign d_oe     = d0_oe || d1_oe;
	assign rd_ready = d0_oe ? d0_rdy : d1_oe ? d1_rdy : 1'b1;

	assign flash_busy = d0_busy || d1_busy;
	assign die_busy = {d1_busy, d0_busy};
	assign dirty0_event = d0_dirty;
	assign dirty0_block = d0_dblk;
	assign dirty1_event = d1_dirty;
	assign dirty1_block = d1_dblk;

	// Backing-store arbitration
	// A die that is not busy can only be asking for array data for a bus
	// read, so it wins: that is the latency-critical path, and with the
	// hardware busy model (whole die answers status) a busy die never asks
	// for array data at all. mem_tag flips once per issued transaction,
	// whatever the source -- see contract item 1 in the header.

	// A die's own tag identifies its transaction, so the arbiter services each
	// transaction exactly once: a request still standing with an unchanged tag
	// is the tail of one already answered, not a new one.
	reg d0_tag_seen_q, d1_tag_seen_q;

	wire d0_fresh = d0_mreq && (d0_mtag != d0_tag_seen_q);
	wire d1_fresh = d1_mreq && (d1_mtag != d1_tag_seen_q);

	wire d0_read = d0_fresh && !d0_busy;
	wire d1_read = d1_fresh && !d1_busy;

	always @(posedge clk) begin
		if (reset) begin
			req_q         <= 1'b0;
			mem_we        <= 1'b0;
			mem_addr      <= 25'd0;
			mem_wdata     <= 16'd0;
			mem_be        <= 2'b00;
			mem_lane      <= 1'b0;
			mem_tag       <= 1'b0;
			mem_flash     <= 1'b0;
			grant_q       <= GR_NONE;
			d0_tag_seen_q <= 1'b0;
			d1_tag_seen_q <= 1'b0;
		end else if (ce) begin
			if (req_q) begin
				if (mem_done) begin
					req_q   <= 1'b0;
					grant_q <= GR_NONE;
				end
			end else if (d0_read || (!d1_read && d0_fresh)) begin
				req_q     <= 1'b1;
				mem_we    <= d0_mwe;
				mem_addr  <= {4'd0, d0_maddr};
				mem_wdata <= d0_mwdata;
				mem_be    <= d0_mbe;
				mem_lane  <= d0_mlane;
				mem_flash <= d0_busy;
				mem_tag   <= ~mem_tag;
				grant_q   <= GR_DIE0;
				d0_tag_seen_q <= d0_mtag;
			end else if (d1_fresh) begin
				req_q     <= 1'b1;
				mem_we    <= d1_mwe;
				mem_addr  <= {3'd0, 1'b1, d1_maddr};   // die 1 at +0x200000
				mem_wdata <= d1_mwdata;
				mem_be    <= d1_mbe;
				mem_lane  <= d1_mlane;
				mem_flash <= d1_busy;
				mem_tag   <= ~mem_tag;
				grant_q   <= GR_DIE1;
				d1_tag_seen_q <= d1_mtag;
			end
		end
	end

	// Dirty bitmaps. Not for sector skipping: they coalesce the save-slot dirty
	// pulse and let a savestate load re-sync the .sav. The only thing that sets
	// a bit is the completion edge of an embedded program or erase, so command
	// sequences, reads, reset commands, loader traffic and rejected operations
	// never mark a block dirty.

	reg [34:0] dirty0_q, dirty1_q;

	assign dirty0 = dirty0_q;
	assign dirty1 = dirty1_q;

	wire ss_hit_c0 = (ss_bus_adr == (SS_BASE + 10'd2));
	wire ss_hit_c1 = (ss_bus_adr == (SS_BASE + 10'd6));

	always @(posedge clk) begin
		if (reset || ss_bus_rst) begin
			dirty0_q     <= 35'd0;
			dirty1_q     <= 35'd0;
			dirty_pulse  <= 1'b0;
		end else if (ss_bus_wren && (ss_hit_c0 || ss_hit_c1)) begin
			// A rewind slot carries no flash array, so its bitmap means
			// nothing; leave the live one alone.
			if (!ss_restore_is_rewind) begin
				if (ss_hit_c0) dirty0_q <= ss_bus_din[34:0];
				else           dirty1_q <= ss_bus_din[34:0];
			end
		end else if (ce) begin
			dirty_pulse <= d0_dirty || d1_dirty;

			if (dirty_clear || config_load) begin
				dirty0_q     <= 35'd0;
				dirty1_q     <= 35'd0;
			end else begin
				if (d0_dirty) dirty0_q <= dirty0_q | (35'd1 << d0_dblk);
				if (d1_dirty) dirty1_q <= dirty1_q | (35'd1 << d1_dblk);
			end
		end
	end

	// Savestate mux (internals SS_BASE+0 .. SS_BASE+7)
	// Each die owns two words; the connector owns the bitmap word and a
	// read-only configuration echo per die. Muxed, never wired-OR.

	wire [63:0] ss_cart0 = {18'd0, code0_q, dev0_q, present0_q, dirty0_q};
	wire [63:0] ss_cart1 = {18'd0, code1_q, dev1_q, present1_q, dirty1_q};
	wire [63:0] ss_cfg0  = {18'd0, cart_bytes_q, mask0_q};
	wire [63:0] ss_cfg1  = {18'd0, cart_bytes_q, mask1_q};

	assign ss_bus_dout = ((ss_bus_adr == SS_BASE) ||
	                      (ss_bus_adr == (SS_BASE + 10'd1)))   ? d0_ssdout :
	                     ((ss_bus_adr == (SS_BASE + 10'd4)) ||
	                      (ss_bus_adr == (SS_BASE + 10'd5)))   ? d1_ssdout :
	                     (ss_bus_adr == (SS_BASE + 10'd2))     ? ss_cart0  :
	                     (ss_bus_adr == (SS_BASE + 10'd3))     ? ss_cfg0   :
	                     (ss_bus_adr == (SS_BASE + 10'd6))     ? ss_cart1  :
	                     (ss_bus_adr == (SS_BASE + 10'd7))     ? ss_cfg1   : 64'd0;

	// Pause drain
	// Both dies drained AND no transaction of ours outstanding, held stable
	// for four clk_sys ticks before it counts as drained. The stability
	// requirement exists because the SDRAM bridge's idle indication is an
	// unregistered mix of two clock domains and can glitch true
	// mid-transaction, so a savestate drain term must never be a
	// single-cycle sample.

	reg [2:0] drain_cnt_q;

	wire drained_w = d0_pause_rdy && d1_pause_rdy && !req_q;

	always @(posedge clk) begin
		if (reset) begin
			drain_cnt_q <= 3'd0;
		end else if (ce) begin
			if (!drained_w)              drain_cnt_q <= 3'd0;
			else if (drain_cnt_q != 3'd4) drain_cnt_q <= drain_cnt_q + 3'd1;
		end
	end

	assign pause_ready = pause_req && drained_w && (drain_cnt_q == 3'd4);

endmodule
