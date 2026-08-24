// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps
`default_nettype none

// The storage behind ngp_cart's mailbox: the whole 4 MB cart window living in
// SDRAM at 0x000000-0x3FFFFF, die 0 at offset 0 and die 1 at +0x200000.
//
// A thin adapter over rtl/mem/cart_sdram.sv, which already implements the
// three-port SDRAM service and its clock-domain crossings; what it lacks is
// the NGPC's port allocation and cart-linear address map.
//
//   p0  read-only   CPU cart reads          -- the latency-critical path
//   p1  read/write  program RMW and erase fill commits
//   p2  read/write  image loader and 0xFF prefill, then background save and
//                   savestate traffic (the module serialises loader first)
//
// All three ports stay 16 bits wide. p0 must not be narrowed to 8 bits to
// match the die: sdram.sv's per-port single-entry read cache exists only for
// 8-bit ports, and it would put a coherency surface between p0 reads and p1
// flash writes. At 16 bits there is no cache, a p1 write is visible to the
// very next p0 read, and p0 hands the connector both bytes for free.
//
// ngp_cart already registers mem_req/addr/data/tag, holds them until mem_done,
// and flips mem_tag once per transaction, which is exactly what cart_sdram's
// p0 and p1 clients must do, so the request side is pure wiring and adds no
// latency. The one exception is the p1 read lane: p1 returns a lane-selected
// byte as {8'hff, byte} and the die expects it in the half mem_lane names, so
// it is re-expanded below.
//
// mem_done and mem_rvalid follow the bridge's held valid terms and stay
// asserted until ngp_cart drops mem_req; a pulse would be lost whenever
// ngp_cart's ce is low.
//
// mem_tag alternates on every transaction whatever port it lands on, so two p0
// transactions separated by an even number of p1 transactions carry the same
// tag, and the bridge's held response is keyed on the tag. That is still safe:
// a same-tag repeat needs an intervening p1 transaction, during which
// read_req_i is low for a full SDRAM round trip, which clears
// read_hold_valid_ram_q and then read_data_valid_sys_q, so the second p0
// transaction starts against an empty hold. Back-to-back p0 transactions,
// where mem_req is low for only one clk_sys cycle, always carry different tags.
module ngp_cart_sdram
#(
	// The real clk_ram frequency. See cart_sdram's header: at 120 MHz
	// against a 98.304 MHz clock the refresh interval silently becomes
	// 9.52 us against a 7.8125 us requirement.
	parameter int unsigned CLK_FREQ_HZ = 98_304_000
)
(
	input  wire        clk_sys,
	input  wire        clk_ram,
	input  wire        reset_i,

	// --- ngp_cart mailbox, clk_sys ---------------------------------------
	input  wire        mem_req_i,
	input  wire        mem_we_i,
	input  wire [24:0] mem_addr_i,     // cart-linear byte address
	input  wire [15:0] mem_wdata_i,
	input  wire [1:0]  mem_be_i,
	input  wire        mem_lane_i,
	input  wire        mem_tag_i,
	input  wire        mem_flash_i,    // 1 = program/erase commit (p1), 0 = CPU read (p0)
	output wire [15:0] mem_rdata_o,
	output wire        mem_rvalid_o,
	output wire        mem_done_o,

	// --- loader and prefill mailbox (p2), clk_sys -------------------------
	// One-shot: pulse load_req_i for a single cycle while load_ready_o is
	// high, then wait for load_done_o.
	input  wire        load_req_i,
	input  wire [24:0] load_addr_i,
	input  wire [15:0] load_data_i,
	output wire        load_ready_o,
	output wire        load_done_o,

	// --- background mailbox (p2), clk_sys ---------------------------------
	// Sparse overlay and savestate Type3 movers. Same one-shot rule: pulse, do
	// not hold across transactions.
	input  wire        bg_req_i,
	input  wire        bg_we_i,
	input  wire [24:0] bg_addr_i,
	input  wire [15:0] bg_wdata_i,
	input  wire [1:0]  bg_be_i,
	output wire        bg_ready_o,
	output wire        bg_done_o,
	output wire [15:0] bg_rdata_o,

	inout  wire [15:0] SDRAM_DQ,
	output wire [12:0] SDRAM_A,
	output wire        SDRAM_DQML,
	output wire        SDRAM_DQMH,
	output wire  [1:0] SDRAM_BA,
	output wire        SDRAM_nCS,
	output wire        SDRAM_nWE,
	output wire        SDRAM_nRAS,
	output wire        SDRAM_nCAS,
	output wire        SDRAM_CKE,
	output wire        SDRAM_CLK
);

	// Request routing

	wire to_p0_w = mem_req_i && !mem_flash_i;
	wire to_p1_w = mem_req_i &&  mem_flash_i;

	wire [25:0] mem_sdram_addr_w = {1'b0, mem_addr_i};

	wire        p0_valid_w;
	wire [15:0] p0_data_w;
	wire [15:0] p1_data_w;
	wire        p1_read_valid_w;
	wire        p1_done_w;

	// p1 hands back {8'hff, byte}. Put the byte in the half mem_lane names so
	// the die's own lane select finds it; the partner half keeps the bridge's
	// 0xff, which is both "not fetched" and what an unwritten flash byte reads
	// as. A lane-1 read is therefore just a byte swap of what p1 returned.
	wire [15:0] p1_expanded_w = mem_lane_i ? {p1_data_w[7:0], p1_data_w[15:8]}
	                                       : p1_data_w;

	assign mem_rdata_o  = to_p1_w ? p1_expanded_w : p0_data_w;
	assign mem_rvalid_o = to_p1_w ? p1_read_valid_w : p0_valid_w;
	assign mem_done_o   = to_p1_w ? p1_done_w : p0_valid_w;

	// The supplied three-port SDRAM service

	cart_sdram
	#(
		.CLK_FREQ_HZ(CLK_FREQ_HZ)
	)
	u_sdram
	(
		.clk_sys(clk_sys),
		.clk_ram(clk_ram),
		.reset_i(reset_i),

		// p0: CPU cart reads.
		.read_req_i(to_p0_w),
		.read_visible_req_i(to_p0_w),
		.read_tag_i(mem_tag_i),
		.read_addr_i(mem_sdram_addr_w),
		.read_valid_o(p0_valid_w),
		.read_data_o(p0_data_w),

		// p2 loader half: image load and the 0xFF tail prefill.
		.write_req_i(load_req_i),
		.write_addr_i({1'b0, load_addr_i}),
		.write_data_i(load_data_i),
		.write_ready_o(load_ready_o),
		.write_done_o(load_done_o),

		// p1: program read-modify-write commits and erase fill words.
		.sram_req_i(to_p1_w),
		.sram_we_i(mem_we_i),
		.sram_addr_i(mem_sdram_addr_w),
		.sram_write_data_i(mem_wdata_i),
		.sram_be_i(mem_be_i),
		.sram_read_lane_i(mem_lane_i),
		.sram_tag_i(mem_tag_i),
		.sram_read_data_o(p1_data_w),
		.sram_read_valid_o(p1_read_valid_w),
		.sram_access_done_o(p1_done_w),

		// p2 background half: save staging and the savestate walker.
		.sram_bg_req_i(bg_req_i),
		.sram_bg_we_i(bg_we_i),
		.sram_bg_addr_i({1'b0, bg_addr_i}),
		.sram_bg_write_data_i(bg_wdata_i),
		.sram_bg_be_i(bg_be_i),
		.sram_bg_ready_o(bg_ready_o),
		.sram_bg_done_o(bg_done_o),
		.sram_bg_read_data_o(bg_rdata_o),
		.foreground_idle_o(),

		.SDRAM_DQ(SDRAM_DQ),
		.SDRAM_A(SDRAM_A),
		.SDRAM_DQML(SDRAM_DQML),
		.SDRAM_DQMH(SDRAM_DQMH),
		.SDRAM_BA(SDRAM_BA),
		.SDRAM_nCS(SDRAM_nCS),
		.SDRAM_nWE(SDRAM_nWE),
		.SDRAM_nRAS(SDRAM_nRAS),
		.SDRAM_nCAS(SDRAM_nCAS),
		.SDRAM_CKE(SDRAM_CKE),
		.SDRAM_CLK(SDRAM_CLK)
	);

endmodule

`default_nettype wire
