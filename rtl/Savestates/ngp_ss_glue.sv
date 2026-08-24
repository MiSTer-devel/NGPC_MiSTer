// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_ss_glue -- routing for the byte-serial memory tap of `savestates.sv`.
//
//   type 0  work RAM 12 KiB      ) behind ngp_mainboard's flat tap, byte wide
//   type 1  Z80 shared RAM 4 KiB )
//   type 2  video region 16 KiB  )
//   type 3  unreachable at SAVETYPESCOUNT = 3
//
// Cartridge flash never traverses this tap: it is an externally managed sparse
// Type3 manifest plus the physical blocks its frozen ledger selects, so no
// unconditional cartridge image is stored here.
//
// The engine walks one byte at a time and will not advance until
// `(memory_wait_q >= 7) && Save_RAMReady` (rtl/Savestates/savestates.sv), so
// every backend has at least eight clk_sys cycles of slack per byte and
// `Ready` can be tied high.
module ngp_ss_glue
(
	input  wire        clk,
	input  wire        reset,

	// ---- savestates.sv byte-serial memory tap ------------------------------
	input  wire [2:0]  ss_ram_type,
	input  wire [24:0] ss_ram_addr,
	input  wire        ss_ram_rden,
	input  wire        ss_ram_wren,
	input  wire [7:0]  ss_ram_wdata,
	output wire [7:0]  ss_ram_rdata,
	output wire        ss_ram_ready,

	// ---- ngp_mainboard flat block-RAM tap (types 0-2) ----------------------
	output wire [1:0]  mem_type,
	output wire        mem_active,
	output wire [13:0] mem_addr,
	output wire [7:0]  mem_wdata,
	output wire        mem_wren,
	output wire        mem_rden,
	input  wire [7:0]  mem_rdata
);

	wire active_w = ss_ram_rden || ss_ram_wren;

	assign mem_type   = ss_ram_type[1:0];
	assign mem_active = active_w;
	assign mem_addr   = ss_ram_addr[13:0];
	assign mem_wdata  = ss_ram_wdata;
	assign mem_wren   = ss_ram_wren;
	assign mem_rden   = ss_ram_rden;

	assign ss_ram_rdata = mem_rdata;

	// Every active backend is block RAM and answers unconditionally, and the
	// engine waits eight cycles before it looks, so nothing here can be late.
	assign ss_ram_ready = 1'b1;

	// ss_ram_addr's high bits exist because the engine's tap is 25 bits wide
	// for cores whose regions live in a large external memory. Every region
	// here is at most 16 KiB, and only memory types 0..2 exist.
	wire unused_ok = &{1'b0, ss_ram_addr[24:14], ss_ram_type[2], 1'b0};

endmodule

`default_nettype wire
