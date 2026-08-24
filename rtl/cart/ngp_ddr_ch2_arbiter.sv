// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_ddr_ch2_arbiter -- one outstanding owner for ddram.v channel 2.
//
// ddram.v remembers only a pending bit; it does not preserve the bundled
// address/data fields of a requester.  The selected owner is therefore held
// from request admission through ready.  The cartridge shadow loader has
// strict priority while a cartridge download is active.  The sparse-overlay
// mover receives the channel only between loader words and after the loader
// has gone idle.
module ngp_ddr_ch2_arbiter
(
	input  wire        clk,
	input  wire        reset,
	input  wire        loader_prefer_i,

	input  wire [27:1] loader_addr_i,
	input  wire [63:0] loader_din_i,
	input  wire        loader_req_i,
	input  wire        loader_rnw_i,
	input  wire [7:0]  loader_be_i,
	output wire [63:0] loader_dout_o,
	output wire        loader_ready_o,

	input  wire [27:1] overlay_addr_i,
	input  wire [63:0] overlay_din_i,
	input  wire        overlay_req_i,
	input  wire        overlay_rnw_i,
	input  wire [7:0]  overlay_be_i,
	output wire [63:0] overlay_dout_o,
	output wire        overlay_ready_o,

	output wire [27:1] ddr_addr_o,
	output wire [63:0] ddr_din_o,
	output wire        ddr_req_o,
	output wire        ddr_rnw_o,
	output wire [7:0]  ddr_be_o,
	input  wire [63:0] ddr_dout_i,
	input  wire        ddr_ready_i
);

	localparam [1:0] OWNER_NONE    = 2'd0;
	localparam [1:0] OWNER_LOADER  = 2'd1;
	localparam [1:0] OWNER_OVERLAY = 2'd2;

	reg [1:0] owner_q;

	// In the idle cycle ddram samples the selected request. It becomes owned
	// at that same edge, then every field remains selected until its ready.
	wire choose_loader_w = loader_req_i && (loader_prefer_i || !overlay_req_i);
	wire choose_overlay_w = !choose_loader_w && overlay_req_i;
	wire loader_selected_w = (owner_q == OWNER_LOADER) ||
		((owner_q == OWNER_NONE) && choose_loader_w);

	assign ddr_addr_o = loader_selected_w ? loader_addr_i : overlay_addr_i;
	assign ddr_din_o  = loader_selected_w ? loader_din_i  : overlay_din_i;
	// ddram.v OR-latches a pending bit. A level-held request would therefore
	// be sampled again on the completion edge and launch a duplicate transfer.
	// Emit exactly one admission pulse; keep only the bundle/owner selected
	// until ready returns.
	assign ddr_req_o  = (owner_q == OWNER_NONE) &&
	                    (choose_loader_w || choose_overlay_w);
	assign ddr_rnw_o  = loader_selected_w ? loader_rnw_i  : overlay_rnw_i;
	assign ddr_be_o   = loader_selected_w ? loader_be_i   : overlay_be_i;

	assign loader_dout_o  = ddr_dout_i;
	assign overlay_dout_o = ddr_dout_i;
	assign loader_ready_o = ddr_ready_i && (owner_q == OWNER_LOADER);
	assign overlay_ready_o = ddr_ready_i && (owner_q == OWNER_OVERLAY);

	always @(posedge clk) begin
		if (reset) begin
			owner_q <= OWNER_NONE;
		end else begin
			case (owner_q)
				OWNER_NONE: begin
					if (choose_loader_w) owner_q <= OWNER_LOADER;
					else if (choose_overlay_w) owner_q <= OWNER_OVERLAY;
				end

				default: begin
					if (ddr_ready_i) owner_q <= OWNER_NONE;
				end
			endcase
		end
	end

endmodule

`default_nettype wire
