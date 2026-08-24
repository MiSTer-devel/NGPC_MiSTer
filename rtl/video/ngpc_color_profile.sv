// Copyright (c) 2026 Jamie Blanks

// Presentation-only saturation profiles for the NGPC RGB output.
//
// The physical NGPC panel has not been measured well enough to justify an
// NGPC-specific colour matrix or transfer curve. These are therefore named and
// implemented as saturation choices, not as chip behaviour or an "accurate LCD"
// claim.
//
// The luma and the three convex blends use only shifts and adds. The weights
// sum exactly to a power of two, so black, white and neutral gray remain exact;
// every result is also bounded by its two inputs and needs no saturation clamp.

module ngpc_color_profile
(
	input  wire [1:0] mode,
	input  wire [7:0] r_in,
	input  wire [7:0] g_in,
	input  wire [7:0] b_in,

	output wire [7:0] r_out,
	output wire [7:0] g_out,
	output wire [7:0] b_out
);

	// Y = (2R + 5G + B + 4) >> 3. The +4 provides nearest-integer
	// rounding. Maximum sum is 8*255+4 = 2044, so eleven bits are exact.
	wire [7:0] luma = 8'(({2'b00, r_in, 1'b0} +
	                      {1'b0,  g_in, 2'b00} +
	                      {3'b000, g_in} +
	                      {3'b000, b_in} + 11'd4) >> 3);

	function automatic [7:0] profile_channel(
		input [7:0] channel,
		input [7:0] y,
		input [1:0] profile
	);
		begin
			case (profile)
				2'd0: begin                         // 75% chroma (default)
					profile_channel = 8'(({1'b0, channel, 1'b0} +
					                      {2'b00, channel} +
					                      {2'b00, y} + 10'd2) >> 2);
				end
				2'd1: begin                         // 50% chroma
					profile_channel = 8'(({2'b00, channel} +
					                      {2'b00, y} + 10'd1) >> 1);
				end
				2'd2: begin                         // 25% chroma
					profile_channel = 8'(({2'b00, channel} +
					                      {1'b0, y, 1'b0} +
					                      {2'b00, y} + 10'd2) >> 2);
				end
				default: begin                      // 100% chroma
					profile_channel = channel;
				end
			endcase
		end
	endfunction

	assign r_out = profile_channel(r_in, luma, mode);
	assign g_out = profile_channel(g_in, luma, mode);
	assign b_out = profile_channel(b_in, luma, mode);

endmodule
