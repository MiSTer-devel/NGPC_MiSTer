// Copyright (c) 2026 Jamie Blanks

// Deterministic 12 KiB work-RAM clear for a new MiSTer game load.
//
// This is a MiSTer session-management bridge, not console hardware. A new
// cartridge is a new cold session: the machine is held in reset while this
// walker zeros CPU addresses 0x4000-0x6FFF through the existing savestate
// port B. The BIOS then performs its normal cold initialization, after which
// ngp_setup_seed overlays the menu-selected setup record and HPS RTC.
//
// Ordinary soft reset never starts this block. Keeping that distinction makes
// resets within one loaded game console-like while every new game begins from
// the same blank retained-RAM state.

module ngp_wram_clear
(
	input  wire        clk,
	input  wire        reset,
	input  wire        start,

	output reg         busy,
	output reg         done,
	output wire [1:0]  ss_mem_type,
	output wire        ss_mem_active,
	output wire [13:0] ss_mem_addr,
	output wire [7:0]  ss_mem_wdata,
	output wire        ss_mem_wren,
	output wire        ss_mem_rden
);

	localparam [13:0] WRAM_LAST = 14'h2FFF; // 0x4000 + 0x2FFF = 0x6FFF

	reg [13:0] addr_q;

	assign ss_mem_type   = 2'd0;
	assign ss_mem_active = busy;
	assign ss_mem_addr   = addr_q;
	assign ss_mem_wdata  = 8'h00;
	assign ss_mem_wren   = busy;
	assign ss_mem_rden   = 1'b0;

	always @(posedge clk) begin
		if (reset) begin
			busy   <= 1'b0;
			done   <= 1'b0;
			addr_q <= 14'd0;
		end else begin
			done <= 1'b0;

			if (start) begin
				busy   <= 1'b1;
				addr_q <= 14'd0;
			end else if (busy) begin
				if (addr_q == WRAM_LAST) begin
					busy   <= 1'b0;
					done   <= 1'b1;
					addr_q <= 14'd0;
				end else begin
					addr_q <= addr_q + 14'd1;
				end
			end
		end
	end

endmodule
