// Copyright (c) 2026 Jamie Blanks

// TMP95C061-class chip select / wait controller, and the DRAM controller
// register stub that shares the same external-bus register block.
//
// SFRs owned: 0x3C-0x3F, 0x5A-0x5F, 0x68-0x6C (TMP95C061 datasheet
// pp.186-187).
//
// This is not the system address decoder. MSAR/MAMR only reach A8 and only
// describe external windows, so the internal regions - SFRs 0x000000-0x0000FF,
// work RAM, Z80 shared RAM, K2GE, BIOS - are decoded by the k2_soc fabric
// ahead of this module, with their own fixed widths and zero waits. What
// arrives here is whatever the fabric did not claim.
//
// The controller is a masked bitwise comparator (datasheet p.58), so there is
// no arithmetic in the decode. MSARn supplies the start bits for A23-A16
// (S15-S8 are hardwired 0) and MAMRn disables the comparison of individual
// address bits (1 = don't care):
//
//   CS0  MAMR0 bit  7    6    5    4    3    2    1       0
//                  A20  A19  A18  A17  A16  A15  A14-A9  A8
//        A23, A22, A21 are always compared.
//   CS1  the same window shifted one bit up: bit 7 = V21 ... bit 2 = V16,
//        bit 1 covers A15-A9, bit 0 = A8; only A23/A22 always compared.
//   CS2/CS3  eight individual mask bits for A22-A15, A23 always compared;
//        A14-A8 are never compared.
//
// The CS0 mapping matches the datasheet's own worked example (MSAR0 = 0x01,
// MAMR0 = 0x07 -> 64 KB at 0x010000, p.59). The CS1 and CS2/CS3 orderings are
// inferred from the p.59 prose, since the table giving them is an unreadable
// image; the NGP never uses CS2/CS3 as windows.
//
// DO NOT ADD WAITS HERE. The BIOS programs B0CS = B1CS = 0x17 = zero waits
// (datasheet p.52, encoding 11). A CS/WAIT controller cannot distinguish a
// fetch from a data access, so the asymmetric "cart fetch wait 3, cart data
// wait 0" measurement is really prefetch-queue fill cost on the 8-bit cart
// bus, which t900_biu already charges. CART_EXTRA_WAIT exists to test the
// other reading; it defaults to 0.
//
// SFR bus: sfr_rdata is combinational, and sfr_wr is a one-clk_sys strobe
// that is deliberately not re-gated by ce (the strobe is the enable).

module ngp_csc
#(
	// AM8/16 is bonded high on the NGP, so every external area is 8-bit and
	// the B*BUS bits are stored and read back but have no effect (datasheet
	// p.7 section 3.1.2, p.51 item (2)). The parameter keeps the general
	// TMP95C061 behavior available.
	parameter [0:0] AM8_16 = 1'b1,

	// Extra wait states for the cart windows (CS0/CS1). See the wait note
	// above; default 0.
	parameter [2:0] CART_EXTRA_WAIT = 3'd0
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire [6:0]  sfr_addr,
	input  wire [7:0]  sfr_wdata,
	input  wire        sfr_wr,
	input  wire        sfr_rd,
	output wire [7:0]  sfr_rdata,

	// The address the BIU is about to drive. The width answer must be
	// combinational from this address, not from the one on the bus: a consumer
	// that answers one cycle late issues 16-bit cycles inside an 8-bit region.
	input  wire [23:0] plan_addr,
	output wire        plan_width8,
	output wire [3:0]  plan_cs_n,

	// The address actually on the bus. The wait answer comes from this one.
	input  wire [23:0] bus_addr,
	input  wire        bus_active,
	input  wire        wait_pin,      // PA0, unbonded on the NGP board
	output wire [2:0]  bus_waits,
	output wire [3:0]  cs_n,

	input  wire [7:0]  ss_reg_addr,   // words 0x22-0x27
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	// Registers
	reg [7:0] msar0, mamr0, msar1, mamr1;
	reg [7:0] msar2, mamr2, msar3, mamr3;
	reg [7:0] b0cs, b1cs, b2cs, b3cs, bexcs;

	wire [7:0] drefcr;
	wire [7:0] dmemcr;

	// Bit positions inside a B*CS byte (datasheet pp.51-52).
	localparam integer BCS_ENABLE = 4;   // B*E
	localparam integer BCS_MODE   = 3;   // B2M (CS2) / B3CAS (CS3)
	localparam integer BCS_BUS    = 2;   // B*BUS, 1 = 8-bit

	// WAIT pin sampling. Encoding 10 inserts one wait and then holds the cycle
	// while the pin is low (datasheet p.52). Silicon samples the pin on a state
	// boundary, so the sample is taken on ce (one ce = one Toshiba state). On
	// the NGP the pin is PA0 and nothing on the board drives it, so this path
	// never fires.
	reg wait_pin_q;

	always @(posedge clk) begin
		if (reset) begin
			wait_pin_q <= 1'b1;             // idle high: no extension
		end else if (ce) begin
			wait_pin_q <= wait_pin;
		end
	end

	// Decode: expand a mask register into a per-address-bit mask for A23..A8,
	// where a 1 means "do not compare this address bit".
	function automatic [15:0] expand_mask0(input [7:0] m);
		expand_mask0 = { 3'b000,          // A23, A22, A21 always compared
		                 m[7:3],          // A20..A16
		                 m[2],            // A15
		                 {6{m[1]}},       // A14..A9 share one mask bit
		                 m[0] };          // A8
	endfunction

	function automatic [15:0] expand_mask1(input [7:0] m);
		expand_mask1 = { 2'b00,           // A23, A22 always compared
		                 m[7:2],          // A21..A16
		                 {7{m[1]}},       // A15..A9 share one mask bit
		                 m[0] };          // A8
	endfunction

	function automatic [15:0] expand_mask23(input [7:0] m);
		expand_mask23 = { 1'b0,           // A23 always compared
		                  m[7:0],         // A22..A15
		                  7'h7f };        // A14..A8 never compared
	endfunction

	// Per-block hit, already gated by the block enable bit. Only A23-A7 reach
	// the decode: A23-A8 are what the comparators see, and A7 is only there
	// for CS2's "whole 16 MB space above 0x000080" mode.
	//
	// Every function in this file takes what it reads as an argument. A
	// function that reads module registers directly is a simulation hazard: a
	// continuous assignment calling it re-evaluates when its arguments change
	// and not when the registers do, so the answer silently goes stale.
	function automatic hit_cs0(input [15:0] a_hi, input [7:0] msar,
	                           input [7:0] mamr, input [7:0] ctl);
		hit_cs0 = ctl[BCS_ENABLE] &&
		          (((a_hi ^ {msar, 8'h00}) & ~expand_mask0(mamr)) == 16'h0000);
	endfunction

	function automatic hit_cs1(input [15:0] a_hi, input [7:0] msar,
	                           input [7:0] mamr, input [7:0] ctl);
		hit_cs1 = ctl[BCS_ENABLE] &&
		          (((a_hi ^ {msar, 8'h00}) & ~expand_mask1(mamr)) == 16'h0000);
	endfunction

	function automatic hit_cs3(input [15:0] a_hi, input [7:0] msar,
	                           input [7:0] mamr, input [7:0] ctl);
		hit_cs3 = ctl[BCS_ENABLE] &&
		          (((a_hi ^ {msar, 8'h00}) & ~expand_mask23(mamr)) == 16'h0000);
	endfunction

	// CS2 has the extra B2M mode: 0 ignores MSAR2/MAMR2 and claims the whole
	// 16 MB space from 0x000080 up (datasheet p.53 item (6), p.58). "at or above
	// 0x80" is a plain test of the bits above A6, not arithmetic.
	function automatic hit_cs2(input [16:0] a_hi, input [7:0] msar,
	                           input [7:0] mamr, input [7:0] ctl);
		hit_cs2 = ctl[BCS_ENABLE] &&
		          (ctl[BCS_MODE]
		             ? (((a_hi[16:1] ^ {msar, 8'h00}) & ~expand_mask23(mamr)) == 16'h0000)
		             : (a_hi != 17'h00000));
	endfunction

	// "If the set address areas overlap or CS2 is enabled for the 16 MB
	// area, the one with a smaller CS number is selected" (datasheet p.54).
	function automatic [3:0] cs_select_n(input [3:0] hits);
	begin
		if (hits[0])      cs_select_n = 4'b1110;
		else if (hits[1]) cs_select_n = 4'b1101;
		else if (hits[2]) cs_select_n = 4'b1011;
		else if (hits[3]) cs_select_n = 4'b0111;
		else              cs_select_n = 4'b1111;
	end
	endfunction

	// The control byte that governs the cycle. Anything no block claims is
	// governed by BEXCS, which has no enable bit (datasheet p.53 item (5)).
	function automatic [7:0] sel_ctl(input [3:0] hits, input [7:0] c0,
	                                 input [7:0] c1, input [7:0] c2,
	                                 input [7:0] c3, input [7:0] cex);
	begin
		if (hits[0])      sel_ctl = c0;
		else if (hits[1]) sel_ctl = c1;
		else if (hits[2]) sel_ctl = c2;
		else if (hits[3]) sel_ctl = c3;
		else              sel_ctl = cex;
	end
	endfunction

	// Wait states from the block's B*W* field (datasheet p.52):
	//   00 = 2 waits, 01 = 1 wait, 10 = 1 wait then extend while WAIT is low,
	//   11 = 0 waits.
	// Encoding 10's extension is reported as the maximum count while the pin is
	// low, the only channel the port list offers. On the NGP the pin is never
	// driven, so that case is unreachable here.
	function automatic [2:0] wait_count(input [1:0] bw, input is_cart, input wait_high);
		reg [2:0] base;
		reg [3:0] sum;
	begin
		case (bw)
			2'b00:   base = 3'd2;
			2'b01:   base = 3'd1;
			2'b10:   base = wait_high ? 3'd1 : 3'd7;
			default: base = 3'd0;
		endcase
		sum = {1'b0, base} + (is_cart ? {1'b0, CART_EXTRA_WAIT} : 4'd0);
		wait_count = (sum > 4'd7) ? 3'd7 : sum[2:0];
	end
	endfunction

	wire [3:0] plan_hits = { hit_cs3(plan_addr[23:8], msar3, mamr3, b3cs),
	                         hit_cs2(plan_addr[23:7], msar2, mamr2, b2cs),
	                         hit_cs1(plan_addr[23:8], msar1, mamr1, b1cs),
	                         hit_cs0(plan_addr[23:8], msar0, mamr0, b0cs) };

	wire [3:0] bus_hits  = { hit_cs3(bus_addr[23:8], msar3, mamr3, b3cs),
	                         hit_cs2(bus_addr[23:7], msar2, mamr2, b2cs),
	                         hit_cs1(bus_addr[23:8], msar1, mamr1, b1cs),
	                         hit_cs0(bus_addr[23:8], msar0, mamr0, b0cs) };

	wire [7:0] plan_ctl  = sel_ctl(plan_hits, b0cs, b1cs, b2cs, b3cs, bexcs);
	wire [7:0] bus_ctl   = sel_ctl(bus_hits,  b0cs, b1cs, b2cs, b3cs, bexcs);

	// AM8/16 = 1 forces every external area to 8 bits regardless of B*BUS
	// (datasheet p.51 item (2)).
	assign plan_width8 = AM8_16 ? 1'b1 : plan_ctl[BCS_BUS];
	assign plan_cs_n   = cs_select_n(plan_hits);

	// The chip select strobes low only for a cycle that is actually on the
	// bus; plan_cs_n is the same answer for the cycle being planned.
	assign cs_n      = bus_active ? cs_select_n(bus_hits) : 4'b1111;
	assign bus_waits = wait_count(bus_ctl[1:0], bus_hits[0] | bus_hits[1], wait_pin_q);

	// SFR reads: combinational, zero wait states. Every register here is R/W,
	// so there are no write-only shadows. Addresses this module does not own
	// read 0xFF.
	reg [7:0] rd;

	always_comb begin
		case (sfr_addr)
			7'h3c:   rd = msar0;
			7'h3d:   rd = mamr0;
			7'h3e:   rd = msar1;
			7'h3f:   rd = mamr1;
			7'h5a:   rd = drefcr;
			7'h5b:   rd = dmemcr;
			7'h5c:   rd = msar2;
			7'h5d:   rd = mamr2;
			7'h5e:   rd = msar3;
			7'h5f:   rd = mamr3;
			7'h68:   rd = b0cs;
			7'h69:   rd = b1cs;
			7'h6a:   rd = b2cs;
			7'h6b:   rd = b3cs;
			7'h6c:   rd = bexcs;
			default: rd = 8'hff;
		endcase
	end

	assign sfr_rdata = rd;

	// Savestate tap. Nothing here is in flight across a pause, so the module is
	// ready the moment it is asked; ss_wren is honored only while paused.
	assign pause_ready = pause_req;

	wire ss_hit_22 = ss_wren && pause_ready && (ss_reg_addr == 8'h22);
	wire ss_hit_23 = ss_wren && pause_ready && (ss_reg_addr == 8'h23);
	wire ss_hit_24 = ss_wren && pause_ready && (ss_reg_addr == 8'h24);
	wire ss_hit_25 = ss_wren && pause_ready && (ss_reg_addr == 8'h25);

	reg [31:0] ss_rd;

	always_comb begin
		case (ss_reg_addr)
			8'h22:   ss_rd = {msar0, mamr0, msar1, mamr1};
			8'h23:   ss_rd = {msar2, mamr2, msar3, mamr3};
			8'h24:   ss_rd = {b0cs, b1cs, b2cs, b3cs};
			8'h25:   ss_rd = {bexcs, drefcr, dmemcr, 8'h00};
			default: ss_rd = 32'h00000000;   // 0x26-0x27 reserved
		endcase
	end

	assign ss_rdata = ss_rd;

	// Register file. Reset values are the chip reset values (datasheet p.58,
	// pp.186-187), not the state the BIOS leaves behind: every window register
	// all-ones, every block disabled except CS2, which comes up enabled with
	// B2M = 0 so it covers 0x000080-0xFFFFFF, and every wait field 00 = two
	// wait states.
	always @(posedge clk) begin
		if (reset) begin
			msar0 <= 8'hff;
			mamr0 <= 8'hff;
			msar1 <= 8'hff;
			mamr1 <= 8'hff;
			msar2 <= 8'hff;
			mamr2 <= 8'hff;
			msar3 <= 8'hff;
			mamr3 <= 8'hff;
			b0cs  <= 8'h00;
			b1cs  <= 8'h00;
			b2cs  <= 8'h10;                 // B2E = 1, B2M = 0
			b3cs  <= 8'h00;
			bexcs <= 8'h00;
		end else begin
			if (sfr_wr) begin
				case (sfr_addr)
					7'h3c: msar0 <= sfr_wdata;
					7'h3d: mamr0 <= sfr_wdata;
					7'h3e: msar1 <= sfr_wdata;
					7'h3f: mamr1 <= sfr_wdata;
					7'h5c: msar2 <= sfr_wdata;
					7'h5d: mamr2 <= sfr_wdata;
					7'h5e: msar3 <= sfr_wdata;
					7'h5f: mamr3 <= sfr_wdata;
					7'h68: b0cs  <= sfr_wdata;
					7'h69: b1cs  <= sfr_wdata;
					7'h6a: b2cs  <= sfr_wdata;
					7'h6b: b3cs  <= sfr_wdata;
					7'h6c: bexcs <= sfr_wdata;
					default: ;                  // 0x5A/0x5B live in ngp_dramc
				endcase
			end

			if (ss_hit_22) begin
				msar0 <= ss_wdata[31:24];
				mamr0 <= ss_wdata[23:16];
				msar1 <= ss_wdata[15:8];
				mamr1 <= ss_wdata[7:0];
			end
			if (ss_hit_23) begin
				msar2 <= ss_wdata[31:24];
				mamr2 <= ss_wdata[23:16];
				msar3 <= ss_wdata[15:8];
				mamr3 <= ss_wdata[7:0];
			end
			if (ss_hit_24) begin
				b0cs <= ss_wdata[31:24];
				b1cs <= ss_wdata[23:16];
				b2cs <= ss_wdata[15:8];
				b3cs <= ss_wdata[7:0];
			end
			if (ss_hit_25) begin
				bexcs <= ss_wdata[31:24];
			end
		end
	end

	ngp_dramc dramc
	(
		.clk(clk),
		.reset(reset),
		.wr_drefcr(sfr_wr && (sfr_addr == 7'h5a)),
		.wr_dmemcr(sfr_wr && (sfr_addr == 7'h5b)),
		.wdata(sfr_wdata),
		.ss_wren(ss_hit_25),
		.ss_wdata(ss_wdata[23:8]),
		.drefcr(drefcr),
		.dmemcr(dmemcr)
	);

	// Tie-off for inputs this controller does not look at:
	//  - sfr_rd is in the normative port list but no register here has a read
	//    side effect;
	//  - A6-A0 never reach the comparators. Start addresses are 64 KB granular
	//    and the masks reach down to A8, with A7 used only by CS2's
	//    whole-space mode (datasheet p.58);
	//  - only the wait field of the live block's control byte is needed for
	//    bus_waits; its enable, mode and bus-size bits are consumed in
	//    plan_hits / bus_hits and plan_width8.
	wire unused_ok = &{1'b0, sfr_rd, plan_addr[6:0], bus_addr[6:0], bus_ctl[7:2]};

endmodule

// DRAM controller stub. It lives inside ngp_csc because it is the other half
// of the same external bus controller (datasheet p.60).
//
// DREFCR (0x5A) and DMEMCR (0x5B) are readable, writable storage with no
// behavior: there is no DRAM on the Neo Geo Pocket board and the BIOS never
// touches either register, so modelling refresh timing would be inventing
// hardware nothing can observe.
//
// No ce port: the only thing this stub does is latch a register on a write
// strobe, and the strobe is itself the enable.
module ngp_dramc
(
	input  wire        clk,
	input  wire        reset,
	input  wire        wr_drefcr,
	input  wire        wr_dmemcr,
	input  wire [7:0]  wdata,
	input  wire        ss_wren,
	input  wire [15:0] ss_wdata,       // {DREFCR, DMEMCR}
	output reg  [7:0]  drefcr,
	output reg  [7:0]  dmemcr
);

	always @(posedge clk) begin
		if (reset) begin
			drefcr <= 8'h00;               // datasheet p.187
			dmemcr <= 8'h80;               // datasheet p.187
		end else begin
			if (wr_drefcr) drefcr <= wdata;
			if (wr_dmemcr) dmemcr <= wdata;
			if (ss_wren) begin
				drefcr <= ss_wdata[15:8];
				dmemcr <= ss_wdata[7:0];
			end
		end
	end

endmodule
