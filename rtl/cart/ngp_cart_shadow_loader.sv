// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_cart_shadow_loader -- duplicate each physical cartridge-load word into
// the canonical external-SDRAM image and the immutable DDR3 pristine image.
//
// `ngp_cart_rom` already serializes its payload and 0xFF physical-tail writes.
// It must not be told that a word has completed until BOTH destinations have
// accepted it: cart_ready then means that every byte which the flash dies can
// read also has a pristine source for later sparse-overlay restoration.
//
// This is intentionally a one-word fanout rather than a FIFO. The loader has
// one outstanding request by contract, a DDR3 write has no read turnaround,
// and a one-entry sequencer is small enough to keep the acknowledgement proof
// local. `ddram` latches only a pending bit, so the held DDR payload is a
// requirement, not an optimisation.
module ngp_cart_shadow_loader
(
	input  wire        clk,
	input  wire        reset,

	// ---- loader side ------------------------------------------------------
	input  wire        load_req_i,
	input  wire [24:0] load_addr_i,
	input  wire [15:0] load_data_i,
	// Pulse at the start of a new cart download. The fanout sees raw image
	// words followed by the loader's 0xFF tail, so its CRC identifies the full
	// physical flash view rather than merely the file payload.
	input  wire        identity_reset_i,
	output wire        load_ready_o,
	output wire        load_done_o,
	output wire [31:0] pristine_crc32_o,

	// ---- canonical live-cart SDRAM p2 ------------------------------------
	output wire        live_req_o,
	output wire [24:0] live_addr_o,
	output wire [15:0] live_data_o,
	input  wire        live_ready_i,
	input  wire        live_done_i,

	// ---- pristine-shadow DDR3 client (rtl/mem/ddram.v channel 2) ---------
	output wire [27:1] ddr_addr_o,
	output wire [63:0] ddr_din_o,
	output wire        ddr_req_o,
	output wire        ddr_rnw_o,
	output wire [7:0]  ddr_be_o,
	input  wire        ddr_ready_i
);

	localparam [1:0] ST_IDLE       = 2'd0;
	localparam [1:0] ST_SHADOW_REQ = 2'd1;
	localparam [1:0] ST_WAIT       = 2'd2;

	reg [1:0]  state_q;
	reg [24:1] addr_q;
	reg [15:0] data_q;
	reg        live_done_q;
	reg        shadow_done_q;
	reg        done_q;
	reg [31:0] pristine_crc32_q;

	function automatic [31:0] crc32_byte;
		input [31:0] crc_i;
		input [7:0] data_i;
		reg [31:0] crc_v;
		integer bit_i;
		begin
			crc_v = crc_i ^ {24'd0, data_i};
			for (bit_i = 0; bit_i < 8; bit_i = bit_i + 1) begin
				if (crc_v[0]) crc_v = (crc_v >> 1) ^ 32'hEDB88320;
				else          crc_v = crc_v >> 1;
			end
			crc32_byte = crc_v;
		end
	endfunction

	function automatic [31:0] crc32_word;
		input [31:0] crc_i;
		input [15:0] data_i;
		begin
			crc32_word = crc32_byte(crc32_byte(crc_i, data_i[7:0]), data_i[15:8]);
		end
	endfunction

	// The live p2 mailbox must accept the request in the same cycle as the
	// loader's pulse. DDR3 may be busy behind channel 1, but ddram retains the
	// channel-2 request and the sequencer holds its payload until ready.
	assign load_ready_o = (state_q == ST_IDLE) && live_ready_i;
	assign live_req_o   = load_req_i && load_ready_o;
	assign live_addr_o  = load_addr_i;
	assign live_data_o  = load_data_i;
	assign load_done_o  = done_q;
	assign pristine_crc32_o = ~pristine_crc32_q;

	// ddram's helper address is eight-byte aligned. The physical aperture
	// begins at 0x30000000, so a cart byte address A maps to shadow byte A.
	// A cartridge load is WIDE/even-addressed, but all four lanes are stated so
	// a future byte loader cannot silently corrupt its adjacent byte.
	assign ddr_addr_o = {3'd0, addr_q[24:3], 2'b00};
	assign ddr_req_o  = (state_q == ST_SHADOW_REQ);
	assign ddr_rnw_o  = 1'b0;

	reg [63:0] ddr_din_q;
	reg [7:0]  ddr_be_q;

	always @* begin
		ddr_din_q = 64'd0;
		ddr_be_q  = 8'd0;
		case (addr_q[2:1])
			2'd0: begin
				ddr_din_q[15:0] = data_q;
				ddr_be_q = 8'h03;
			end
			2'd1: begin
				ddr_din_q[31:16] = data_q;
				ddr_be_q = 8'h0C;
			end
			2'd2: begin
				ddr_din_q[47:32] = data_q;
				ddr_be_q = 8'h30;
			end
			default: begin
				ddr_din_q[63:48] = data_q;
				ddr_be_q = 8'hC0;
			end
		endcase
	end

	assign ddr_din_o = ddr_din_q;
	assign ddr_be_o  = ddr_be_q;

	always @(posedge clk) begin
		done_q <= 1'b0;

		if (reset) begin
			state_q       <= ST_IDLE;
			addr_q        <= 24'd0;
			data_q        <= 16'd0;
			live_done_q   <= 1'b0;
			shadow_done_q <= 1'b0;
			done_q        <= 1'b0;
			pristine_crc32_q <= 32'hFFFFFFFF;
		end else begin
			if (identity_reset_i) pristine_crc32_q <= 32'hFFFFFFFF;
			else if (load_req_i && load_ready_o)
				pristine_crc32_q <= crc32_word(pristine_crc32_q, load_data_i);

			case (state_q)
				ST_IDLE: begin
					if (load_req_i && load_ready_o) begin
						addr_q        <= load_addr_i[24:1];
						data_q        <= load_data_i;
						live_done_q   <= live_done_i;
						shadow_done_q <= 1'b0;
						state_q       <= ST_SHADOW_REQ;
					end
				end

				ST_SHADOW_REQ: begin
					if (live_done_i) live_done_q <= 1'b1;
					if (ddr_ready_i) begin
						shadow_done_q <= 1'b1;
						state_q <= ST_WAIT;
					end
				end

				default: begin
					if (live_done_i) live_done_q <= 1'b1;
					if (live_done_q && shadow_done_q) begin
						done_q  <= 1'b1;
						state_q <= ST_IDLE;
					end
				end
			endcase
		end
	end

endmodule

`default_nettype wire
