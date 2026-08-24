// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// ngp_cart_overlay_geometry -- immutable FlashMem erase geometry for sparse
// persistence. All values are selected with bounded case logic: there is no
// divide/modulus or title-specific write allowlist in the hardware path.
module ngp_cart_overlay_geometry
(
	input  wire [1:0] size_code_i,  // 1/2/3 = 4/8/16 Mbit, 0 = no die
	input  wire [5:0] block_i,
	output reg        valid_o,
	output reg [20:0] base_o,       // die-relative byte address
	output reg [16:0] bytes_o,      // complete physical erase-block byte count
	output reg [15:0] words_o       // complete 16-bit word count
);

	always @* begin
		valid_o = 1'b0;
		base_o  = 21'd0;
		bytes_o = 17'd0;
		words_o = 16'd0;

		case (size_code_i)
			2'd1: begin
				if (block_i < 6'd7) begin
					valid_o = 1'b1;
					base_o  = {block_i[4:0], 16'd0};
					bytes_o = 17'd65536;
					words_o = 16'd32768;
				end else begin
					case (block_i)
						6'd7:  begin valid_o = 1'b1; base_o = 21'h070000; bytes_o = 17'd32768; words_o = 16'd16384; end
						6'd8:  begin valid_o = 1'b1; base_o = 21'h078000; bytes_o = 17'd8192;  words_o = 16'd4096; end
						6'd9:  begin valid_o = 1'b1; base_o = 21'h07A000; bytes_o = 17'd8192;  words_o = 16'd4096; end
						6'd10: begin valid_o = 1'b1; base_o = 21'h07C000; bytes_o = 17'd16384; words_o = 16'd8192; end
						default: ;
					endcase
				end
			end

			2'd2: begin
				if (block_i < 6'd15) begin
					valid_o = 1'b1;
					base_o  = {block_i[4:0], 16'd0};
					bytes_o = 17'd65536;
					words_o = 16'd32768;
				end else begin
					case (block_i)
						6'd15: begin valid_o = 1'b1; base_o = 21'h0F0000; bytes_o = 17'd32768; words_o = 16'd16384; end
						6'd16: begin valid_o = 1'b1; base_o = 21'h0F8000; bytes_o = 17'd8192;  words_o = 16'd4096; end
						6'd17: begin valid_o = 1'b1; base_o = 21'h0FA000; bytes_o = 17'd8192;  words_o = 16'd4096; end
						6'd18: begin valid_o = 1'b1; base_o = 21'h0FC000; bytes_o = 17'd16384; words_o = 16'd8192; end
						default: ;
					endcase
				end
			end

			2'd3: begin
				if (block_i < 6'd31) begin
					valid_o = 1'b1;
					base_o  = {block_i[4:0], 16'd0};
					bytes_o = 17'd65536;
					words_o = 16'd32768;
				end else begin
					case (block_i)
						6'd31: begin valid_o = 1'b1; base_o = 21'h1F0000; bytes_o = 17'd32768; words_o = 16'd16384; end
						6'd32: begin valid_o = 1'b1; base_o = 21'h1F8000; bytes_o = 17'd8192;  words_o = 16'd4096; end
						6'd33: begin valid_o = 1'b1; base_o = 21'h1FA000; bytes_o = 17'd8192;  words_o = 16'd4096; end
						6'd34: begin valid_o = 1'b1; base_o = 21'h1FC000; bytes_o = 17'd16384; words_o = 16'd8192; end
						default: ;
					endcase
				end
			end

			default: ;
		endcase
	end

endmodule

`default_nettype wire
