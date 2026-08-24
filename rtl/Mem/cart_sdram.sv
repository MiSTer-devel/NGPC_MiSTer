// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps
`default_nettype none

// Cartridge SDRAM service.
//
// Runtime CPU requests remain asserted until READY, so their complete bundles
// are sampled twice into the faster SDRAM domain before controller admission.
// ROM and SRAM responses are held by tag until the 49.152 MHz side captures
// them. Loader and paused/background requests use one-entry bundled-data toggle
// mailboxes. Port 0 serves runtime ROM reads, port 1 serves runtime cartridge
// SRAM, and port 2 serializes ROM-loader and background SRAM traffic. The
// controller's busy outputs qualify command admission; ready reports only a
// completed read or write and is never used as refresh/init availability.
//
// CLK_FREQ_HZ must be the real frequency of clk_ram. It is not cosmetic: every
// ns-derived constant in sdram.sv is computed from it, and while all the others
// merely become conservative when the parameter is too high, T_REFI is an
// interval the controller must not exceed. A parameter of 120 MHz against a
// real 98.304 MHz clock gives 936 cycles = 9.52 us between refreshes against
// the AS4C32M16SB's 7.8125 us requirement -- a 78 ms refresh pass against a
// 64 ms retention spec, i.e. temperature-dependent data loss in the cart image.
// Nothing catches it: sdram.sv's elaboration guards only fire on an illegal
// CAS latency, and they sit inside `ifndef SYNTHESIS.
module cart_sdram
#(
	parameter int unsigned CLK_FREQ_HZ = 98_304_000
)
(
	input  wire        clk_sys,
	input  wire        clk_ram,
	input  wire        reset_i,

	input  wire        read_req_i,
	input  wire        read_visible_req_i,
	input  wire        read_tag_i,
	input  wire [25:0] read_addr_i,
	output wire        read_valid_o,
	output wire [15:0] read_data_o,

	input  wire        write_req_i,
	input  wire [25:0] write_addr_i,
	input  wire [15:0] write_data_i,
	output wire        write_ready_o,
	output reg         write_done_o,

	input  wire        sram_req_i,
	input  wire        sram_we_i,
	input  wire [25:0] sram_addr_i,
	input  wire [15:0] sram_write_data_i,
	input  wire [1:0]  sram_be_i,
	input  wire        sram_read_lane_i,
	input  wire        sram_tag_i,
	output wire [15:0] sram_read_data_o,
	output wire        sram_read_valid_o,
	output wire        sram_access_done_o,

	input  wire        sram_bg_req_i,
	input  wire        sram_bg_we_i,
	input  wire [25:0] sram_bg_addr_i,
	input  wire [15:0] sram_bg_write_data_i,
	input  wire [1:0]  sram_bg_be_i,
	output wire        sram_bg_ready_o,
	output reg         sram_bg_done_o,
	output wire [15:0] sram_bg_read_data_o,
	output wire        foreground_idle_o,

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

	localparam [1:0] P2_OWNER_NONE   = 2'd0;
	localparam [1:0] P2_OWNER_LOADER = 2'd1;
	localparam [1:0] P2_OWNER_SRAM   = 2'd2;

	reg [1:0] reset_sys_pipe_q = 2'b11;
	reg [1:0] reset_ram_pipe_q = 2'b11;

	always @(posedge clk_sys or posedge reset_i) begin
		if (reset_i) reset_sys_pipe_q <= 2'b11;
		else reset_sys_pipe_q <= {reset_sys_pipe_q[0], 1'b0};
	end

	always @(posedge clk_ram or posedge reset_i) begin
		if (reset_i) reset_ram_pipe_q <= 2'b11;
		else reset_ram_pipe_q <= {reset_ram_pipe_q[0], 1'b0};
	end

	wire reset_sys_w = reset_sys_pipe_q[1];
	wire reset_ram_w = reset_ram_pipe_q[1];

	// Loader request mailbox, clk_sys side.
	reg        write_toggle_sys_q = 1'b0;
	reg        write_pending_sys_q = 1'b0;
	reg [25:0] write_addr_sys_q = 26'd0;
	reg [15:0] write_data_sys_q = 16'd0;
	reg        write_done_toggle_seen_sys_q = 1'b0;

	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg write_done_toggle_sys_meta_q = 1'b0;
	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg write_done_toggle_sys_sync_q = 1'b0;

	// Background SRAM mailbox, clk_sys side.
	reg        sram_bg_toggle_sys_q = 1'b0;
	reg        sram_bg_pending_sys_q = 1'b0;
	reg        sram_bg_req_seen_sys_q = 1'b0;
	reg        sram_bg_we_sys_q = 1'b0;
	reg [25:0] sram_bg_addr_sys_q = 26'd0;
	reg [15:0] sram_bg_data_sys_q = 16'd0;
	reg [1:0]  sram_bg_be_sys_q = 2'b11;
	reg        sram_bg_done_toggle_seen_sys_q = 1'b0;
	reg [15:0] sram_bg_read_data_sys_q = 16'hffff;

	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg sram_bg_done_toggle_sys_meta_q = 1'b0;
	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg sram_bg_done_toggle_sys_sync_q = 1'b0;
	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg init_ready_sys_meta_q = 1'b0;
	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg init_ready_sys_sync_q = 1'b0;
	reg init_done_sys_q = 1'b0;

	wire write_done_seen_sys_w =
		write_done_toggle_sys_sync_q != write_done_toggle_seen_sys_q;
	wire sram_bg_done_seen_sys_w =
		sram_bg_done_toggle_sys_sync_q != sram_bg_done_toggle_seen_sys_q;
	wire [1:0] sram_bg_be_effective_w =
		(sram_bg_be_i == 2'b00) ? 2'b11 : sram_bg_be_i;

	assign write_ready_o = init_done_sys_q && !write_pending_sys_q &&
		!reset_sys_w;
	assign sram_bg_ready_o = init_done_sys_q && !sram_bg_pending_sys_q &&
		!reset_sys_w;
	assign sram_bg_read_data_o = sram_bg_read_data_sys_q;

	// Loader mailbox synchronizers, clk_ram side.
	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg write_toggle_ram_meta_q = 1'b0;
	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg write_toggle_ram_sync_q = 1'b0;
	reg [25:0] write_addr_ram_meta_q = 26'd0;
	reg [25:0] write_addr_ram_sync_q = 26'd0;
	reg [15:0] write_data_ram_meta_q = 16'd0;
	reg [15:0] write_data_ram_sync_q = 16'd0;
	reg        write_toggle_seen_ram_q = 1'b0;
	reg        write_pending_ram_q = 1'b0;
	reg [25:0] write_addr_ram_q = 26'd0;
	reg [15:0] write_data_ram_q = 16'd0;
	reg        write_done_toggle_ram_q = 1'b0;

	// Background SRAM mailbox synchronizers, clk_ram side.
	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg sram_bg_toggle_ram_meta_q = 1'b0;
	(* preserve, altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg sram_bg_toggle_ram_sync_q = 1'b0;
	reg        sram_bg_we_ram_meta_q = 1'b0;
	reg        sram_bg_we_ram_sync_q = 1'b0;
	reg [25:0] sram_bg_addr_ram_meta_q = 26'd0;
	reg [25:0] sram_bg_addr_ram_sync_q = 26'd0;
	reg [15:0] sram_bg_data_ram_meta_q = 16'd0;
	reg [15:0] sram_bg_data_ram_sync_q = 16'd0;
	reg [1:0]  sram_bg_be_ram_meta_q = 2'b11;
	reg [1:0]  sram_bg_be_ram_sync_q = 2'b11;
	reg        sram_bg_toggle_seen_ram_q = 1'b0;
	reg        sram_bg_pending_ram_q = 1'b0;
	reg        sram_bg_we_ram_q = 1'b0;
	reg [25:0] sram_bg_addr_ram_q = 26'd0;
	reg [15:0] sram_bg_data_ram_q = 16'd0;
	reg [1:0]  sram_bg_be_ram_q = 2'b11;
	reg        sram_bg_done_toggle_ram_q = 1'b0;
	reg [15:0] sram_bg_read_data_ram_q = 16'hffff;

	// Physical controller ports.
	wire        p0_busy_w;
	wire        p0_ready_w;
	wire [63:0] p0_dout_w;
	wire        p1_busy_w;
	wire        p1_ready_w;
	wire [63:0] p1_dout_w;
	wire        p2_busy_w;
	wire        p2_ready_w;
	wire [63:0] p2_dout_w;
	wire        init_ready_ram_w = !p0_busy_w && !p1_busy_w && !p2_busy_w;

	// Runtime inputs are stable until READY. clk_ram (98.304 MHz) is 2x clk_sys
	// from the same PLL, so the domains are phase-aligned and two capture stages
	// (meta -> sync) resolve the crossing without metastability. Control (req/tag)
	// now shares the same 2-stage depth as the held-stable payload, so the whole
	// bundle is coherent at the sync stage. This keeps live 49.152 MHz decode
	// logic out of the controller and SDRAM pin cones.
	//
	// The synchronizers are not what makes this safe. The response paths below
	// (read_hold_* and sram_*_hold_*) cross clk_ram -> clk_sys with no
	// synchronizer at all; they are correct only because the two clocks are
	// synchronous, integer-ratio, phase-aligned and fully timed, and because
	// control and payload are launched on the same edge through equal-depth
	// paths. False-pathing or clock-grouping clk_sys against clk_ram does not
	// merely cost margin, it tears multi-bit bundles.
	reg        read_req_ram_meta_q = 1'b0;
	reg        read_req_ram_sync_q = 1'b0;
	reg        read_tag_ram_meta_q = 1'b0;
	reg        read_tag_ram_sync_q = 1'b0;
	reg [25:0] read_addr_ram_meta_q = 26'd0;
	reg [25:0] read_addr_ram_sync_q = 26'd0;
	reg        sram_req_ram_meta_q = 1'b0;
	reg        sram_req_ram_sync_q = 1'b0;
	reg        sram_we_ram_meta_q = 1'b0;
	reg        sram_we_ram_sync_q = 1'b0;
	reg [25:0] sram_addr_ram_meta_q = 26'd0;
	reg [25:0] sram_addr_ram_sync_q = 26'd0;
	reg [15:0] sram_data_ram_meta_q = 16'd0;
	reg [15:0] sram_data_ram_sync_q = 16'd0;
	reg [1:0]  sram_be_ram_meta_q = 2'b11;
	reg [1:0]  sram_be_ram_sync_q = 2'b11;
	reg        sram_lane_ram_meta_q = 1'b0;
	reg        sram_lane_ram_sync_q = 1'b0;
	reg        sram_tag_ram_meta_q = 1'b0;
	reg        sram_tag_ram_sync_q = 1'b0;

	// Runtime ROM request and held response.
	reg        read_active_tag_ram_q = 1'b0;
	reg        read_active_ram_q = 1'b0;
	reg        read_hold_valid_ram_q = 1'b0;
	reg        read_hold_tag_ram_q = 1'b0;
	reg [15:0] read_hold_data_ram_q = 16'hffff;
	reg        read_data_valid_sys_q = 1'b0;
	reg        read_data_tag_sys_q = 1'b0;
	reg [15:0] read_data_sys_q = 16'd0;

	wire read_offer_ram_w = !read_active_ram_q && read_req_ram_sync_q &&
		(!read_hold_valid_ram_q ||
		 (read_tag_ram_sync_q != read_hold_tag_ram_q));
	wire read_accept_ram_w = read_offer_ram_w && !p0_busy_w;

	assign read_valid_o = !reset_sys_w && read_visible_req_i &&
		read_data_valid_sys_q && (read_tag_i == read_data_tag_sys_q);
	assign read_data_o = read_data_sys_q;

	// Runtime SRAM path uses the same registered request/held response contract.
	reg        sram_seen_valid_ram_q = 1'b0;
	reg        sram_seen_tag_ram_q = 1'b0;
	reg        sram_read_active_ram_q = 1'b0;
	reg        sram_read_active_tag_ram_q = 1'b0;
	reg        sram_read_active_lane_ram_q = 1'b0;
	reg        sram_write_active_ram_q = 1'b0;
	reg        sram_write_active_tag_ram_q = 1'b0;
	reg        sram_access_hold_valid_ram_q = 1'b0;
	reg        sram_access_hold_tag_ram_q = 1'b0;
	reg        sram_data_hold_valid_ram_q = 1'b0;
	reg        sram_data_hold_tag_ram_q = 1'b0;
	reg [15:0] sram_data_hold_ram_q = 16'hffff;
	reg        sram_access_valid_sys_q = 1'b0;
	reg        sram_access_tag_sys_q = 1'b0;
	reg        sram_data_valid_sys_q = 1'b0;
	reg        sram_data_tag_sys_q = 1'b0;
	reg [15:0] sram_data_sys_q = 16'hffff;

	wire [1:0] sram_be_effective_w =
		(sram_be_ram_sync_q == 2'b00) ? 2'b11 : sram_be_ram_sync_q;
	wire sram_offer_ram_w = !sram_seen_valid_ram_q &&
		!sram_read_active_ram_q && !sram_write_active_ram_q &&
		sram_req_ram_sync_q;
	wire sram_accept_ram_w = sram_offer_ram_w && !p1_busy_w;

	assign sram_read_data_o = sram_data_sys_q;
	assign sram_read_valid_o = !reset_sys_w && sram_req_i && !sram_we_i &&
		sram_data_valid_sys_q && (sram_tag_i == sram_data_tag_sys_q);
	assign sram_access_done_o = !reset_sys_w && sram_req_i &&
		sram_access_valid_sys_q && (sram_tag_i == sram_access_tag_sys_q);
	// Unregistered, and a mix of both clock domains: read_req_i/sram_req_i are
	// clk_sys while the three *_ram_q terms are clk_ram. It can glitch, and it
	// can read true for an instant while a transaction is in flight, which is
	// fine for a coarse transfer-allowed heuristic but unusable raw as a
	// savestate pause_ready term. ngp_cart_sdram registers it in clk_sys and
	// requires four stable ticks before calling it drained.
	assign foreground_idle_o = !read_req_i && !sram_req_i &&
		!read_active_ram_q && !sram_read_active_ram_q &&
		!sram_write_active_ram_q;

	// Port 2 serializes the two mailbox clients. Payload is latched before req.
	reg        p2_req_q = 1'b0;
	reg        p2_active_q = 1'b0;
	reg [1:0]  p2_owner_q = P2_OWNER_NONE;
	reg        p2_we_q = 1'b0;
	reg [25:0] p2_addr_q = 26'd0;
	reg [15:0] p2_data_q = 16'd0;
	reg [1:0]  p2_be_q = 2'b11;

	// System-domain completion and held response capture.
	always @(posedge clk_sys) begin
		write_done_toggle_sys_meta_q <= write_done_toggle_ram_q;
		write_done_toggle_sys_sync_q <= write_done_toggle_sys_meta_q;
		sram_bg_done_toggle_sys_meta_q <= sram_bg_done_toggle_ram_q;
		sram_bg_done_toggle_sys_sync_q <= sram_bg_done_toggle_sys_meta_q;
		init_ready_sys_meta_q <= init_ready_ram_w;
		init_ready_sys_sync_q <= init_ready_sys_meta_q;
		write_done_o <= 1'b0;
		sram_bg_done_o <= 1'b0;

		if (reset_sys_w) begin
			read_data_valid_sys_q <= 1'b0;
			read_data_tag_sys_q <= 1'b0;
			read_data_sys_q <= 16'd0;
			sram_access_valid_sys_q <= 1'b0;
			sram_access_tag_sys_q <= 1'b0;
			sram_data_valid_sys_q <= 1'b0;
			sram_data_tag_sys_q <= 1'b0;
			sram_data_sys_q <= 16'hffff;
			write_toggle_sys_q <= 1'b0;
			write_pending_sys_q <= 1'b0;
			write_addr_sys_q <= 26'd0;
			write_data_sys_q <= 16'd0;
			write_done_toggle_seen_sys_q <= 1'b0;
			sram_bg_toggle_sys_q <= 1'b0;
			sram_bg_pending_sys_q <= 1'b0;
			sram_bg_req_seen_sys_q <= 1'b0;
			sram_bg_we_sys_q <= 1'b0;
			sram_bg_addr_sys_q <= 26'd0;
			sram_bg_data_sys_q <= 16'd0;
			sram_bg_be_sys_q <= 2'b11;
			sram_bg_done_toggle_seen_sys_q <= 1'b0;
			sram_bg_read_data_sys_q <= 16'hffff;
			init_done_sys_q <= 1'b0;
		end else begin
			if (!sram_bg_req_i) begin
				sram_bg_req_seen_sys_q <= 1'b0;
			end

			if (read_hold_valid_ram_q) begin
				read_data_valid_sys_q <= 1'b1;
				read_data_tag_sys_q <= read_hold_tag_ram_q;
				read_data_sys_q <= read_hold_data_ram_q;
			end else if (!read_visible_req_i ||
				(read_tag_i != read_data_tag_sys_q)) begin
				read_data_valid_sys_q <= 1'b0;
			end

			if (sram_access_hold_valid_ram_q) begin
				sram_access_valid_sys_q <= 1'b1;
				sram_access_tag_sys_q <= sram_access_hold_tag_ram_q;
			end else if (!sram_req_i ||
				(sram_tag_i != sram_access_tag_sys_q)) begin
				sram_access_valid_sys_q <= 1'b0;
			end

			if (sram_data_hold_valid_ram_q) begin
				sram_data_valid_sys_q <= 1'b1;
				sram_data_tag_sys_q <= sram_data_hold_tag_ram_q;
				sram_data_sys_q <= sram_data_hold_ram_q;
			end else if (!sram_req_i || sram_we_i ||
				(sram_tag_i != sram_data_tag_sys_q)) begin
				sram_data_valid_sys_q <= 1'b0;
			end

			if (init_ready_sys_sync_q) init_done_sys_q <= 1'b1;

			if (write_done_seen_sys_w) begin
				write_done_toggle_seen_sys_q <= write_done_toggle_sys_sync_q;
				write_pending_sys_q <= 1'b0;
				write_done_o <= 1'b1;
			end

			if (write_req_i && write_ready_o) begin
				write_addr_sys_q <= write_addr_i;
				write_data_sys_q <= write_data_i;
				write_toggle_sys_q <= ~write_toggle_sys_q;
				write_pending_sys_q <= 1'b1;
			end

			if (sram_bg_done_seen_sys_w) begin
				sram_bg_done_toggle_seen_sys_q <=
					sram_bg_done_toggle_sys_sync_q;
				sram_bg_pending_sys_q <= 1'b0;
				sram_bg_read_data_sys_q <= sram_bg_read_data_ram_q;
				sram_bg_done_o <= 1'b1;
			end

			if (sram_bg_req_i && !sram_bg_req_seen_sys_q &&
				sram_bg_ready_o) begin
				sram_bg_we_sys_q <= sram_bg_we_i;
				sram_bg_addr_sys_q <= sram_bg_addr_i;
				sram_bg_data_sys_q <= sram_bg_write_data_i;
				sram_bg_be_sys_q <= sram_bg_be_effective_w;
				sram_bg_toggle_sys_q <= ~sram_bg_toggle_sys_q;
				sram_bg_pending_sys_q <= 1'b1;
				sram_bg_req_seen_sys_q <= 1'b1;
			end
		end
	end

	// RAM-domain mailbox capture, runtime ownership, and port-2 sequencing.
	always @(posedge clk_ram) begin
		read_req_ram_meta_q <= read_req_i;
		read_req_ram_sync_q <= read_req_ram_meta_q;
		read_tag_ram_meta_q <= read_tag_i;
		read_tag_ram_sync_q <= read_tag_ram_meta_q;
		read_addr_ram_meta_q <= read_addr_i;
		read_addr_ram_sync_q <= read_addr_ram_meta_q;
		sram_req_ram_meta_q <= sram_req_i;
		sram_req_ram_sync_q <= sram_req_ram_meta_q;
		sram_we_ram_meta_q <= sram_we_i;
		sram_we_ram_sync_q <= sram_we_ram_meta_q;
		sram_addr_ram_meta_q <= sram_addr_i;
		sram_addr_ram_sync_q <= sram_addr_ram_meta_q;
		sram_data_ram_meta_q <= sram_write_data_i;
		sram_data_ram_sync_q <= sram_data_ram_meta_q;
		sram_be_ram_meta_q <= sram_be_i;
		sram_be_ram_sync_q <= sram_be_ram_meta_q;
		sram_lane_ram_meta_q <= sram_read_lane_i;
		sram_lane_ram_sync_q <= sram_lane_ram_meta_q;
		sram_tag_ram_meta_q <= sram_tag_i;
		sram_tag_ram_sync_q <= sram_tag_ram_meta_q;
		write_toggle_ram_meta_q <= write_toggle_sys_q;
		write_toggle_ram_sync_q <= write_toggle_ram_meta_q;
		write_addr_ram_meta_q <= write_addr_sys_q;
		write_addr_ram_sync_q <= write_addr_ram_meta_q;
		write_data_ram_meta_q <= write_data_sys_q;
		write_data_ram_sync_q <= write_data_ram_meta_q;
		sram_bg_toggle_ram_meta_q <= sram_bg_toggle_sys_q;
		sram_bg_toggle_ram_sync_q <= sram_bg_toggle_ram_meta_q;
		sram_bg_we_ram_meta_q <= sram_bg_we_sys_q;
		sram_bg_we_ram_sync_q <= sram_bg_we_ram_meta_q;
		sram_bg_addr_ram_meta_q <= sram_bg_addr_sys_q;
		sram_bg_addr_ram_sync_q <= sram_bg_addr_ram_meta_q;
		sram_bg_data_ram_meta_q <= sram_bg_data_sys_q;
		sram_bg_data_ram_sync_q <= sram_bg_data_ram_meta_q;
		sram_bg_be_ram_meta_q <= sram_bg_be_sys_q;
		sram_bg_be_ram_sync_q <= sram_bg_be_ram_meta_q;

		if (reset_ram_w) begin
			read_req_ram_meta_q <= 1'b0;
			read_req_ram_sync_q <= 1'b0;
			read_tag_ram_meta_q <= 1'b0;
			read_tag_ram_sync_q <= 1'b0;
			read_addr_ram_meta_q <= 26'd0;
			read_addr_ram_sync_q <= 26'd0;
			sram_req_ram_meta_q <= 1'b0;
			sram_req_ram_sync_q <= 1'b0;
			sram_we_ram_meta_q <= 1'b0;
			sram_we_ram_sync_q <= 1'b0;
			sram_addr_ram_meta_q <= 26'd0;
			sram_addr_ram_sync_q <= 26'd0;
			sram_data_ram_meta_q <= 16'd0;
			sram_data_ram_sync_q <= 16'd0;
			sram_be_ram_meta_q <= 2'b11;
			sram_be_ram_sync_q <= 2'b11;
			sram_lane_ram_meta_q <= 1'b0;
			sram_lane_ram_sync_q <= 1'b0;
			sram_tag_ram_meta_q <= 1'b0;
			sram_tag_ram_sync_q <= 1'b0;
			write_toggle_seen_ram_q <= 1'b0;
			write_pending_ram_q <= 1'b0;
			write_addr_ram_q <= 26'd0;
			write_data_ram_q <= 16'd0;
			write_done_toggle_ram_q <= 1'b0;
			sram_bg_toggle_seen_ram_q <= 1'b0;
			sram_bg_pending_ram_q <= 1'b0;
			sram_bg_we_ram_q <= 1'b0;
			sram_bg_addr_ram_q <= 26'd0;
			sram_bg_data_ram_q <= 16'd0;
			sram_bg_be_ram_q <= 2'b11;
			sram_bg_done_toggle_ram_q <= 1'b0;
			sram_bg_read_data_ram_q <= 16'hffff;
			read_active_tag_ram_q <= 1'b0;
			read_active_ram_q <= 1'b0;
			read_hold_valid_ram_q <= 1'b0;
			read_hold_tag_ram_q <= 1'b0;
			read_hold_data_ram_q <= 16'hffff;
			sram_seen_valid_ram_q <= 1'b0;
			sram_seen_tag_ram_q <= 1'b0;
			sram_read_active_ram_q <= 1'b0;
			sram_read_active_tag_ram_q <= 1'b0;
			sram_read_active_lane_ram_q <= 1'b0;
			sram_write_active_ram_q <= 1'b0;
			sram_write_active_tag_ram_q <= 1'b0;
			sram_access_hold_valid_ram_q <= 1'b0;
			sram_access_hold_tag_ram_q <= 1'b0;
			sram_data_hold_valid_ram_q <= 1'b0;
			sram_data_hold_tag_ram_q <= 1'b0;
			sram_data_hold_ram_q <= 16'hffff;
			p2_req_q <= 1'b0;
			p2_active_q <= 1'b0;
			p2_owner_q <= P2_OWNER_NONE;
			p2_we_q <= 1'b0;
			p2_addr_q <= 26'd0;
			p2_data_q <= 16'd0;
			p2_be_q <= 2'b11;
		end else begin
			if ((write_toggle_ram_sync_q != write_toggle_seen_ram_q) &&
				!write_pending_ram_q) begin
				write_toggle_seen_ram_q <= write_toggle_ram_sync_q;
				write_pending_ram_q <= 1'b1;
				write_addr_ram_q <= write_addr_ram_sync_q;
				write_data_ram_q <= write_data_ram_sync_q;
			end

			if ((sram_bg_toggle_ram_sync_q != sram_bg_toggle_seen_ram_q) &&
				!sram_bg_pending_ram_q) begin
				sram_bg_toggle_seen_ram_q <= sram_bg_toggle_ram_sync_q;
				sram_bg_pending_ram_q <= 1'b1;
				sram_bg_we_ram_q <= sram_bg_we_ram_sync_q;
				sram_bg_addr_ram_q <= sram_bg_addr_ram_sync_q;
				sram_bg_data_ram_q <= sram_bg_data_ram_sync_q;
				sram_bg_be_ram_q <= sram_bg_be_ram_sync_q;
			end

			if (read_hold_valid_ram_q &&
				(!read_req_ram_sync_q ||
				 (read_tag_ram_sync_q != read_hold_tag_ram_q))) begin
				read_hold_valid_ram_q <= 1'b0;
			end
			if (p0_ready_w && read_active_ram_q) begin
				read_hold_valid_ram_q <= 1'b1;
				read_hold_tag_ram_q <= read_active_tag_ram_q;
				read_hold_data_ram_q <= p0_dout_w[15:0];
				read_active_ram_q <= 1'b0;
			end
			if (read_accept_ram_w) begin
				read_active_tag_ram_q <= read_tag_ram_sync_q;
				read_active_ram_q <= 1'b1;
			end

			if (sram_seen_valid_ram_q &&
				(!sram_req_ram_sync_q ||
				 (sram_tag_ram_sync_q != sram_seen_tag_ram_q))) begin
				sram_seen_valid_ram_q <= 1'b0;
			end
			if (sram_access_hold_valid_ram_q &&
				(!sram_req_ram_sync_q ||
				 (sram_tag_ram_sync_q != sram_access_hold_tag_ram_q))) begin
				sram_access_hold_valid_ram_q <= 1'b0;
			end
			if (sram_data_hold_valid_ram_q &&
				(!sram_req_ram_sync_q || sram_we_ram_sync_q ||
				 (sram_tag_ram_sync_q != sram_data_hold_tag_ram_q))) begin
				sram_data_hold_valid_ram_q <= 1'b0;
			end
			if (sram_accept_ram_w) begin
				sram_seen_valid_ram_q <= 1'b1;
				sram_seen_tag_ram_q <= sram_tag_ram_sync_q;
				if (sram_we_ram_sync_q) begin
					sram_write_active_ram_q <= 1'b1;
					sram_write_active_tag_ram_q <= sram_tag_ram_sync_q;
				end else begin
					sram_read_active_ram_q <= 1'b1;
					sram_read_active_tag_ram_q <= sram_tag_ram_sync_q;
					sram_read_active_lane_ram_q <= sram_lane_ram_sync_q;
				end
			end
			if (p1_ready_w && sram_read_active_ram_q) begin
				sram_read_active_ram_q <= 1'b0;
				sram_access_hold_valid_ram_q <= 1'b1;
				sram_access_hold_tag_ram_q <= sram_read_active_tag_ram_q;
				sram_data_hold_valid_ram_q <= 1'b1;
				sram_data_hold_tag_ram_q <= sram_read_active_tag_ram_q;
				sram_data_hold_ram_q <= sram_read_active_lane_ram_q ?
					{8'hff, p1_dout_w[15:8]} :
					{8'hff, p1_dout_w[7:0]};
			end
			if (p1_ready_w && sram_write_active_ram_q) begin
				sram_write_active_ram_q <= 1'b0;
				sram_access_hold_valid_ram_q <= 1'b1;
				sram_access_hold_tag_ram_q <= sram_write_active_tag_ram_q;
			end

			if (!p2_active_q && !p2_req_q) begin
				if (write_pending_ram_q) begin
					p2_req_q <= 1'b1;
					p2_owner_q <= P2_OWNER_LOADER;
					p2_we_q <= 1'b1;
					p2_addr_q <= write_addr_ram_q;
					p2_data_q <= write_data_ram_q;
					p2_be_q <= 2'b11;
				end else if (sram_bg_pending_ram_q) begin
					p2_req_q <= 1'b1;
					p2_owner_q <= P2_OWNER_SRAM;
					p2_we_q <= sram_bg_we_ram_q;
					p2_addr_q <= sram_bg_addr_ram_q;
					p2_data_q <= sram_bg_data_ram_q;
					p2_be_q <= sram_bg_be_ram_q;
				end
			end else if (p2_req_q && !p2_busy_w) begin
				p2_req_q <= 1'b0;
				p2_active_q <= 1'b1;
				if (p2_owner_q == P2_OWNER_LOADER) begin
					write_pending_ram_q <= 1'b0;
				end else if (p2_owner_q == P2_OWNER_SRAM) begin
					sram_bg_pending_ram_q <= 1'b0;
				end
			end else if (p2_active_q && p2_ready_w) begin
				p2_active_q <= 1'b0;
				if (p2_owner_q == P2_OWNER_LOADER) begin
					write_done_toggle_ram_q <= ~write_done_toggle_ram_q;
				end else if (p2_owner_q == P2_OWNER_SRAM) begin
					if (!p2_we_q) begin
						sram_bg_read_data_ram_q <= p2_dout_w[15:0];
					end
					sram_bg_done_toggle_ram_q <=
						~sram_bg_done_toggle_ram_q;
				end
				p2_owner_q <= P2_OWNER_NONE;
			end
		end
	end

	sdram #(
		.CLK_FREQ_HZ(CLK_FREQ_HZ),
		.SDRAM_TIMING_GRADE(7),
		.PORT0_SIZE(1),
		.PORT1_SIZE(1),
		.PORT2_SIZE(1),
		.AUTO_REFRESH(1'b1)
	) u_core (
		.clk(clk_ram),
		.reset(reset_ram_w),
		.refresh(1'b0),
		.p0_req(read_offer_ram_w),
		.p0_we(1'b0),
		.p0_addr(read_addr_ram_sync_q),
		.p0_din(64'd0),
		.p0_byte_en(8'hff),
		.p0_dout(p0_dout_w),
		.p0_busy(p0_busy_w),
		.p0_ready(p0_ready_w),
		.p1_req(sram_offer_ram_w),
		.p1_we(sram_we_ram_sync_q),
		.p1_addr(sram_addr_ram_sync_q),
		.p1_din({48'd0, sram_data_ram_sync_q}),
		.p1_byte_en({6'd0, sram_be_effective_w}),
		.p1_dout(p1_dout_w),
		.p1_busy(p1_busy_w),
		.p1_ready(p1_ready_w),
		.p2_req(p2_req_q),
		.p2_we(p2_we_q),
		.p2_addr(p2_addr_q),
		.p2_din({48'd0, p2_data_q}),
		.p2_byte_en({6'd0, p2_be_q}),
		.p2_dout(p2_dout_w),
		.p2_busy(p2_busy_w),
		.p2_ready(p2_ready_w),
		.SDRAM_CLK(SDRAM_CLK),
		.SDRAM_CKE(SDRAM_CKE),
		.SDRAM_A(SDRAM_A),
		.SDRAM_BA(SDRAM_BA),
		.SDRAM_DQ(SDRAM_DQ),
		.SDRAM_DQML(SDRAM_DQML),
		.SDRAM_DQMH(SDRAM_DQMH),
		.SDRAM_nCS(SDRAM_nCS),
		.SDRAM_nCAS(SDRAM_nCAS),
		.SDRAM_nRAS(SDRAM_nRAS),
		.SDRAM_nWE(SDRAM_nWE)
	);

endmodule

`default_nettype wire
