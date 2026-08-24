// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps
`default_nettype none

// The cartridge loader: HPS ioctl in, SDRAM cart window out.
//
// ioctl_index is 16 bits: [5:0] is the F-slot / boot sub-index, [15:6] is the
// CONF_STR extension index.
//
//   16'h0000                   boot0.rom = NGPC colour BIOS
//   16'h0040                   boot1.rom = NGP mono BIOS
//   ioctl_index[5:0] == 6'd1   FS1 cartridge          <- this module
//   ioctl_index[7:0] == 8'd255 cheat blob             <- ignored everywhere
//
// BIOS routing elsewhere must compare all 16 bits, because boot1.rom's low six
// bits equal boot0.rom's and decoding [5:0] alone silently loads the mono BIOS
// over the colour BIOS at every core start. Cart routing must compare [5:0]
// only, because [15:6] varies with which of the three extensions (NGP/NGC/NPC)
// matched the file and a full-word compare would reject two of them. Index 255
// cannot collide with the cart (its [5:0] is 6'd63) but is excluded explicitly:
// it is a two-byte zero download re-sent on every cheat toggle and ROM open.
//
// A cart download must not extend the integrator's machine reset -- only the
// two BIOS indices do that. A cart swap happens in standby on the real console,
// and resetting the CPU, work RAM, RTC or the K2GE here would skip the BIOS
// warm path. A download holds down the cart path only:
//
//   cart_download_start_o -> sparse overlay identity/ledger replacement
//   config_load_o with image_bytes_o = 0 -> both dies go absent, bitmaps clear
//   cart_ready_o low until the tail prefill finishes
//
// Blank flash reads 0xFF, so any backing byte the image does not cover must be
// 0xFF before the CPU can see it. Only the tail [image_bytes, cart_bytes) is
// prefilled, once the download has fallen and the die population is known:
// 0 ms for an exact power-of-two image, 13 ms for ~100 KB of tail, 65 ms for
// the worst realistic homebrew case. Filling the whole 4 MB window instead
// would cost 430-533 ms of ioctl_wait at ~205-254 ns per p2 round trip, and
// would have to happen before the image arrived or it would erase it. The
// tiny-image-on-a-huge-die case cannot occur: the die is sized from the image
// and everything above the die size folds back through SIZE_MASK.
module ngp_cart_rom
(
	input  wire        clk_sys,
	input  wire        reset_i,

	// --- HPS ioctl --------------------------------------------------------
	input  wire        ioctl_download_i,
	input  wire        ioctl_wr_i,
	input  wire [26:0] ioctl_addr_i,
	input  wire [15:0] ioctl_dout_i,
	// The full framework index is taken, but only [7:0] is decoded: [15:6] is
	// the CONF_STR extension index and comparing it would reject two of the
	// three cart extensions. See the header.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [15:0] ioctl_index_i,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        ioctl_wait_o,

	// --- loader mailbox into ngp_cart_sdram (p2) --------------------------
	output wire        load_req_o,
	output wire [24:0] load_addr_o,
	output wire [15:0] load_data_o,
	input  wire        load_ready_i,
	input  wire        load_done_i,

	// --- cart configuration ----------------------------------------------
	output wire [24:0] image_bytes_o,       // byte count of the loaded image
	// IEEE CRC32 of the raw downloaded image. This is accumulated only from
	// accepted ioctl image words -- never from the physical 0xFF tail -- and is
	// therefore a stable identity for a mounted sparse overlay.
	output wire [31:0] image_crc32_o,
	output wire        config_load_o,       // latch the die population in ngp_cart
	input  wire [24:0] cart_bytes_i,        // ngp_cart's decoded backing size
	// Header bytes used by the BIOS setup seed's optional resume launch. The
	// packed title is little-address-first: bits [7:0] are cart byte 0x24.
	output wire        header_valid_o,
	output wire [15:0] header_catalog_o,
	output wire [7:0]  header_subcatalog_o,
	output wire [95:0] header_title_o,
	output wire        cart_download_o,     // our download is in progress
	output wire        cart_download_start_o,
	// 1 = the backing store matches what the dies will be asked for. High out of
	// reset with no cart at all, because an empty slot has no backing store to
	// wait for: both dies are absent and every read is answered by the
	// connector's pull-ups without touching SDRAM. It drops at the start of a
	// download and comes back when the tail prefill finishes.
	output wire        cart_ready_o
);

	// The cart window is 4 MB. An image larger than that is not a cart, so its
	// writes are dropped rather than allowed to scribble past the window.
	localparam [24:0] CART_WINDOW_BYTES = 25'h400000;

	// config_load is documented as a one-tick strobe, but ngp_cart samples it
	// under its own clock enable. Holding it for a few ticks is idempotent (the
	// decode is a pure function of image_bytes and the bitmap clear is
	// repeatable) and survives a divided ce, so hold it.
	localparam [2:0] CFG_HOLD_TICKS = 3'd3;

	localparam [2:0] ST_IDLE    = 3'd0;
	localparam [2:0] ST_INVAL   = 3'd1;
	localparam [2:0] ST_LOAD    = 3'd2;
	localparam [2:0] ST_DRAIN   = 3'd3;
	localparam [2:0] ST_CONFIG  = 3'd4;
	localparam [2:0] ST_SETTLE  = 3'd5;
	localparam [2:0] ST_FILL    = 3'd6;
	localparam [2:0] ST_READY   = 3'd7;

	reg  [2:0]  state_q;
	reg  [2:0]  hold_q;
	reg         old_download_q;
	reg         load_pending_q;
	reg  [24:0] image_bytes_q;
	reg  [31:0] image_crc32_q;
	reg  [24:0] fill_addr_q;
	reg         cart_ready_q;
	reg  [7:0]  header_words_q;
	reg  [15:0] header_catalog_q;
	reg  [7:0]  header_subcatalog_q;
	reg  [95:0] header_title_q;

	wire cart_sel_w = (ioctl_index_i[5:0] == 6'd1) &&
	                  (ioctl_index_i[7:0] != 8'd255);
	wire cart_download_w = ioctl_download_i && cart_sel_w;
	wire download_start_w = cart_download_w && !old_download_q;

	// Everything below 0x400000 has these bits clear.
	wire in_window_w = (ioctl_addr_i[26:22] == 5'd0);

	// hps_io runs this interface WIDE, so every ioctl_wr carries a full 16-bit
	// word and ioctl_addr steps by two. The running maximum is the byte count
	// and does not depend on ioctl_addr staying valid after the download
	// falls.
	wire [26:0] next_bytes_w = ioctl_addr_i + 27'd2;

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
			// ioctl is little-endian: dout[7:0] is the byte at ioctl_addr.
			crc32_word = crc32_byte(crc32_byte(crc_i, data_i[7:0]), data_i[15:8]);
		end
	endfunction

	wire loader_free_w = !load_pending_q && load_ready_i;

	wire load_accept_w = (state_q == ST_LOAD) && cart_download_w &&
		old_download_q && ioctl_wr_i && in_window_w && loader_free_w;

	wire fill_more_w   = (fill_addr_q < cart_bytes_i);
	wire fill_accept_w = (state_q == ST_FILL) && fill_more_w && loader_free_w;

	// All three back-pressure terms are kept -- the first download cycle, an
	// outstanding mailbox write, and p2 not ready -- plus a fourth: any state
	// that is not ST_LOAD cannot accept a word, and a word arriving unstalled
	// would be dropped on the floor. That covers the four-tick ST_INVAL window
	// at the head of every download.
	assign ioctl_wait_o = cart_download_w &&
		(!old_download_q || (state_q != ST_LOAD) ||
		 load_pending_q || !load_ready_i);

	assign load_req_o  = load_accept_w || fill_accept_w;
	assign load_addr_o = (state_q == ST_FILL) ? fill_addr_q : ioctl_addr_i[24:0];
	assign load_data_o = (state_q == ST_FILL) ? 16'hFFFF : ioctl_dout_i;

	assign image_bytes_o         = image_bytes_q;
	assign image_crc32_o         = ~image_crc32_q;
	assign config_load_o         = (state_q == ST_INVAL) || (state_q == ST_CONFIG);
	assign header_valid_o        = &header_words_q;
	assign header_catalog_o      = header_catalog_q;
	assign header_subcatalog_o   = header_subcatalog_q;
	assign header_title_o        = header_title_q;
	assign cart_download_o       = cart_download_w;
	assign cart_download_start_o = download_start_w;
	assign cart_ready_o          = cart_ready_q;

	// The image ends on a word boundary because the transfer is 16 bits wide,
	// but round up anyway so an odd count can never let the fill overwrite the
	// last image byte.
	wire [24:0] fill_first_w = image_bytes_q[0] ? (image_bytes_q + 25'd1)
	                                            : image_bytes_q;

	always @(posedge clk_sys) begin
		if (reset_i) begin
			state_q        <= ST_IDLE;
			hold_q         <= 3'd0;
			old_download_q <= 1'b0;
			load_pending_q <= 1'b0;
			image_bytes_q  <= 25'd0;
			image_crc32_q  <= 32'hFFFFFFFF;
			fill_addr_q    <= 25'd0;
			cart_ready_q   <= 1'b1;
			header_words_q <= 8'd0;
			header_catalog_q <= 16'd0;
			header_subcatalog_q <= 8'd0;
			header_title_q <= 96'd0;
		end else begin
			old_download_q <= cart_download_w;

			if (load_done_i) load_pending_q <= 1'b0;
			if (load_req_o)  load_pending_q <= 1'b1;

			// A download can start from any state, including the middle of a
			// prefill for the previous cart.
			if (download_start_w) begin
				state_q       <= ST_INVAL;
				hold_q        <= CFG_HOLD_TICKS;
				image_bytes_q <= 25'd0;
				image_crc32_q <= 32'hFFFFFFFF;
				cart_ready_q  <= 1'b0;
				header_words_q <= 8'd0;
				header_catalog_q <= 16'd0;
				header_subcatalog_q <= 8'd0;
				header_title_q <= 96'd0;
			end else begin
				// Capture only accepted words from the real start of the image.
				// ioctl is WIDE: dout[7:0] is the byte at ioctl_addr and
				// dout[15:8] the following byte. An eight-bit completion mask
				// keeps a truncated or sparse image from looking resume-safe.
				if (load_accept_w && (ioctl_addr_i[26:6] == 21'd0)) begin
					case (ioctl_addr_i[5:0])
						6'h20: begin
							header_catalog_q <= ioctl_dout_i;
							header_words_q[0] <= 1'b1;
						end
						6'h22: begin
							header_subcatalog_q <= ioctl_dout_i[7:0];
							header_words_q[1] <= 1'b1;
						end
						6'h24: begin header_title_q[15:0]  <= ioctl_dout_i; header_words_q[2] <= 1'b1; end
						6'h26: begin header_title_q[31:16] <= ioctl_dout_i; header_words_q[3] <= 1'b1; end
						6'h28: begin header_title_q[47:32] <= ioctl_dout_i; header_words_q[4] <= 1'b1; end
						6'h2A: begin header_title_q[63:48] <= ioctl_dout_i; header_words_q[5] <= 1'b1; end
						6'h2C: begin header_title_q[79:64] <= ioctl_dout_i; header_words_q[6] <= 1'b1; end
						6'h2E: begin header_title_q[95:80] <= ioctl_dout_i; header_words_q[7] <= 1'b1; end
						default: ;
					endcase
				end

				case (state_q)
				// Nothing to do until a cart arrives.
				ST_IDLE: ;

				ST_INVAL: begin
					// config_load with a zero byte count: both dies go absent
					// and the dirty bitmaps clear while the image is replaced.
					if (hold_q != 3'd0) hold_q <= hold_q - 3'd1;
					else state_q <= ST_LOAD;
				end

				ST_LOAD: begin
					if (load_accept_w) begin
						image_bytes_q <= (next_bytes_w > {2'd0, CART_WINDOW_BYTES}) ?
							CART_WINDOW_BYTES : next_bytes_w[24:0];
						// The loader has accepted this exact word, so the CRC advances
						// in precisely the same order as the image backing store.
						image_crc32_q <= crc32_word(image_crc32_q, ioctl_dout_i);
					end
					if (!cart_download_w) state_q <= ST_DRAIN;
				end

				ST_DRAIN: begin
					// Let the last mailbox write land before the population is
					// decoded, so cart_ready cannot rise over an unfinished image.
					if (loader_free_w) begin
						state_q <= ST_CONFIG;
						hold_q  <= CFG_HOLD_TICKS;
					end
				end

				ST_CONFIG: begin
					if (hold_q != 3'd0) begin
						hold_q <= hold_q - 3'd1;
					end else begin
						fill_addr_q <= fill_first_w;
						hold_q      <= CFG_HOLD_TICKS;
						state_q     <= ST_SETTLE;
					end
				end

				ST_SETTLE: begin
					// cart_bytes_i is registered on the far side of config_load.
					if (hold_q != 3'd0) hold_q <= hold_q - 3'd1;
					else state_q <= ST_FILL;
				end

				ST_FILL: begin
					// An exact power-of-two image leaves this empty and falls
					// straight through.
					if (!fill_more_w) begin
						state_q      <= ST_READY;
						cart_ready_q <= 1'b1;
					end else if (fill_accept_w) begin
						fill_addr_q <= fill_addr_q + 25'd2;
					end
				end

				ST_READY: ;

				default: state_q <= ST_READY;
				endcase
			end
		end
	end

endmodule

`default_nettype wire
