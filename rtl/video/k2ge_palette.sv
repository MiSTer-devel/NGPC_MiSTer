// Copyright (c) 2026 Jamie Blanks

// K2GE colour lookup: the mono LUT stage, the palette address formation and the
// mono (K1GE) palette bypass.  Turns one line-buffer entry into one RGB444 dot.
// It owns no memory -- the 256 x 16 palette RAM lives in k2ge_vram (port B) and
// this module drives its address and consumes its data -- and no raster, since
// k2ge_scanout decides which dot is produced, whether it is inside the window
// and when the pipeline steps.
//
// Address formation (checked against K2GEpal)
//   K2GE mode   pal_idx = { layer[1:0], CP.C[3:0], pix[1:0] }      entries   0..191
//   compat mode pal_idx = { 2'b11, layer[1:0], P.C, LUT[2:0] }     entries 192..239
//   background  pal_idx = { 5'b11110, BGC[2:0] }                   entries 240..247
//   window      pal_idx = { 5'b11111, OOWC[2:0] }                  entries 248..255
//
//   layer  00 sprite, 01 scroll 1, 10 scroll 2
//
// The layer is derived from the line-buffer entry's rank and P.F rather than
// stored, which is what makes the documented 9-bit entry wide enough.  The P.F
// here must be the copy the composition pass used for that row, one raster line
// behind the composition-side latch, or rank and layer disagree for a line.  P.F = 0 puts plane 1 in front (K1GE 3-4-8):
//
//   rank 1, 3, 5  sprite            -> layer 00
//   rank 2        back  plane       -> layer 10 if P.F = 0, else 01
//   rank 4        front plane       -> layer 01 if P.F = 0, else 10
//
// Entry ranges are those of K2GEpal: sprites 0..63 at 0x8200, scroll 1 at
// 0x8280, scroll 2 at 0x8300, compat sprites 192..207 at 0x8380, compat scroll 1
// at 0x83A0, compat scroll 2 at 0x83C0, background 240..247 at 0x83E0, window
// 248..255 at 0x83F0.  K2GE Table 19 contains end-address typos; K2GEpal is the
// source of record.
//
// The mono LUT stays in the pipeline in compatibility mode.  The 2-bit character
// pixel goes through SPPLT / SC1PLT / SC2PLT, selected by layer and by P.C, and
// the 3-bit LUT output is what indexes the compat palette region: "The value set
// in this sprite palette associated with the K1GE upper palette compatible mode
// sprite color palette is the actual colors displayed" (K2GE 4-12).  In K2GE
// mode the LUT registers are readable and writable but have no display effect
// (K2GE Table 14).
//
// mono_lut is the flat 18 x 3-bit vector k2ge_mmr exports, index 0 = 0x8101:
//
//   0.. 2  SPPLT.01/02/03    3.. 5  SPPLT.11/12/13
//   6.. 8  SC1PLT.01/02/03   9..11  SC1PLT.11/12/13
//  12..14  SC2PLT.01/02/03  15..17  SC2PLT.11/12/13
//
// so the flat index is layer*6 + P.C*3 + (pix-1) and the bit offset three times
// that.  There is no entry for pixel 00 -- a clear dot never reaches the palette
// (K1GE 3-7 Table 11; K2GE 5-1, "00 b is always treated as clear color") and
// k2ge_linebuf never stores one -- so pixel 00 maps onto the pixel-01 entry
// purely to keep the mux total, and is unreachable.
//
// A real K1GE has no colour palette RAM at all: the mono LUT's 3-bit output IS
// the gray level driving the panel (K1GE 3-7-1), and the mono BIOS never writes
// 0x8200-0x83FF.  Compatibility mode only looks right on colour hardware because
// the colour BIOS preloads entries 192..239.  So with `mono_strap` high this
// module bypasses the palette RAM entirely and maps the 3-bit code through a
// built-in ramp; without that an NGP build boots to a black screen through an
// uninitialised palette.  Code 0 is lightest and code 7 darkest ("the rear most
// screen has the lightest color", K1GE 3-7-1), so the code is inverted and
// expanded 3 -> 4 bits by replicating the top bit, which maps 0 -> 0xF and
// 7 -> 0x0 exactly at both ends.  The bypass emits a neutral gray: the
// OSD-selectable tint is applied downstream, so this chip's RGB444 output stays
// presentation-free.
//
// Background and window colour in the mono bypass
//   P_MONO_BG_RAMP puts the window OOWC code through the same 3-bit gradation
//   ramp as the LUT output.  That is an inference: K1GE defines OOWC as a colour
//   but gives the gradation scale explicitly only for character palettes.  Set
//   it to 0 to make the outside-window fill black instead.
//   A native K1GE does have 0x8118, and an invalid BGON there selects the
//   lightest screen rather than black.  K2GEres ("K1GE Resister Table", v1.0,
//   1998.11.18) lists "Background Color BG, 0x8118, D7:6 = BGON, D2:0 = BGC,
//   W (R), timing H, initial value 07" with "D7 = 1, D6 = 0" as the valid
//   combination, and K2GE 5-1 lists the background display function among those
//   unchanged from the K1GE.  "BGC not valid -> black" is a K2GE sentence
//   (K2GE 4-6) and stays on the K2GE-facing path in `bg_black` below; the same
//   black rule is scoped to K1GE compatibility mode on a colour part in
//   `compat_bgon_rule`.
//
// One registered stage, which is the block's real timing risk: rank -> layer,
// the mode mux, the 18:1 LUT mux, the window and background substitution and
// the concatenation all feed a block-RAM address, so `pal_idx` is REGISTERED per
// dot and the RAM sees a clean flop-to-RAM path.  Nothing lets a line-buffer
// output reach the palette address combinationally.
//
//   `form`  one clk_sys cycle, asserted by k2ge_scanout when `lb_entry`,
//           `in_win` and the latched registers are valid for this dot.  The
//           address appears on `pal_rd_addr` from the next cycle and holds
//           until the next `form`.
//   pal_r/g/b are combinational from `pal_rd_data` plus the flags captured at
//           `form`, so they are valid in the cycle where the palette RAM
//           answers.  k2ge_scanout registers them there (its P3 stage).
//
// `ce` is the chip's 49.152 MHz enable, tied high in normal operation and
// dropped to freeze the block.  The savestate write path is deliberately not
// gated by `ce`, because the engine walks state while the chip is frozen.  This
// module claims bits 63:48 of internals word 13 and reads 0 elsewhere;
// k2ge_scanout claims bits 47:0 and reads 0 elsewhere, so the chip parent merges
// the two images with a plain OR.

module k2ge_palette
#(
	// Mono bypass: 1 = the window code uses the same 3-bit gradation ramp as
	// the character LUT, 0 = outside-window fill is black.  The native-K1GE
	// clear background always uses the ramp and is never black; it is BGC when
	// BGON is valid and code 0 otherwise.  See the header, notes 1 and 2.
	parameter P_MONO_BG_RAMP = 1'b1,

	// P_BG_LINE_LATCH -- resolve the BACKGROUND colour once per line
	// 0 = the background colour is read from palette RAM per dot; 1 = fetched
	// once per line and held.  Per-dot tears mid-line on titles that rewrite
	// entry 0x83E0 from a per-line interrupt handler, so 1 is the production
	// choice.  Scope is the background source only: sprite and plane palette
	// entries keep per-dot visibility.
	parameter P_BG_LINE_LATCH = 1'b0
)
(
	input  wire        clk_sys,      // 49.152 MHz
	input  wire        ce,           // chip enable, tie 1 (see header)
	input  wire        rst,          // synchronous power-on reset

	// --- one dot in, from k2ge_scanout -------------------------------------
	input  wire        form,         // capture and form the address this cycle
	input  wire [8:0]  lb_entry,     // {rank[2:0], code[3:0], pix[1:0]}
	input  wire        in_win,       // this dot is inside the display window

	// --- raster line boundary (k2ge_vtimer), for P_BG_LINE_LATCH ------------
	input  wire        line_start,   // entering dot 0 of a raster line

	// --- latched / immediate registers (k2ge_mmr) --------------------------
	input  wire        p_f,          // 0x8030 D7, per-line latched
	input  wire        mode_compat,  // 0x87E2 D7, immediate
	input  wire [53:0] mono_lut,     // 0x8101-0x8117, 18 x 3 bits, captured
	                                 // for the row being painted (k2ge_mmr's
	                                 // P_MONO_LUT_LINE_LATCH)
	input  wire [1:0]  bgon,         // 0x8118 D7:6, per-line latched
	input  wire [2:0]  bgc,          // 0x8118 D2:0, per-line latched
	input  wire [2:0]  oowc,         // 0x8012 D2:0, per-line latched

	// --- system build strap -------------------------------------------------
	input  wire        mono_strap,   // 1 = NGP (K1GE) system: no palette RAM

	// --- palette RAM read port (k2ge_vram port B) ---------------------------
	output wire [7:0]  pal_rd_addr,  // registered, one entry per dot
	input  wire [15:0] pal_rd_data,  // D3:0 R, D7:4 G, D11:8 B, D15:12 read 0

	// --- one dot out, to k2ge_scanout P3 ------------------------------------
	output wire [3:0]  pal_r,
	output wire [3:0]  pal_g,
	output wire [3:0]  pal_b,

	// --- savestate internals, local word 13 bits 63:48 (see header) --------
	output wire [15:0] ss_state,
	input  wire [15:0] ss_wdata,
	input  wire        ss_wren
);

	// The line-buffer entry
	// { rank[2:0], code[3:0], pix[1:0] }, where code
	// is CP.C in K2GE mode and {3'b000, P.C} in compatibility mode.

	wire [2:0] rank = lb_entry[8:6];
	wire [3:0] code = lb_entry[5:2];
	wire [1:0] pix  = lb_entry[1:0];
	wire       p_c  = lb_entry[2];        // code[0]

	// Source of this dot
	// Window fill beats everything - the area outside WBA/WSI is not drawn at
	// all, it is filled with OOWC (K1GE 3-4-10; K2GE 4-5).  Inside the
	// window, rank 0 means nothing was ever written to the line buffer, which
	// is how the design avoids a background fill pass; the background colour
	// is substituted here instead.

	wire src_win = !in_win;
	wire src_bg  =  in_win && (rank == 3'd0);

	// (K2GE 4-6) says only 2'b10 makes BGC valid and other values make the
	// background black.  The K2GE invalid-BGON black rule is applied only in
	// K1GE compatibility mode.  Applying it in native colour mode blanks
	// titles that keep 0x8118 = 0x00 and blank the background by writing
	// 0x0000 to entry 0x83E0 instead.  A true K1GE takes neither branch: it
	// HAS the register (header note 2) but an invalid BGON there is light
	// rather than black, so its background is handled entirely by
	// `mono_code` below.
	wire compat_bgon_rule = mode_compat;
	wire bg_black = src_bg && (bgon != 2'b10) && compat_bgon_rule;

	// Layer, from rank and the per-line latched P.F

	reg [1:0] layer;

	always @* begin
		case (rank)
			3'd2:    layer = p_f ? 2'b01 : 2'b10;   // back plane
			3'd4:    layer = p_f ? 2'b10 : 2'b01;   // front plane
			default: layer = 2'b00;                 // ranks 1, 3, 5: sprite
		endcase
	end

	// Mono LUT lookup
	// Flat index = layer*6 + P.C*3 + (pix-1), bit offset = 3 x that.  Written
	// as two constant muxes and one add rather than a multiply.

	reg [5:0] lut_layer_ofs;
	reg [5:0] lut_pix_ofs;

	always @* begin
		case (layer)
			2'b01:   lut_layer_ofs = 6'd18;    // scroll 1
			2'b10:   lut_layer_ofs = 6'd36;    // scroll 2
			default: lut_layer_ofs = 6'd0;     // sprite
		endcase
	end

	always @* begin
		case (pix)
			2'd2:    lut_pix_ofs = 6'd3;
			2'd3:    lut_pix_ofs = 6'd6;
			default: lut_pix_ofs = 6'd0;       // pixel 01, and unreachable 00
		endcase
	end

	wire [5:0] lut_ofs = lut_layer_ofs + (p_c ? 6'd9 : 6'd0) + lut_pix_ofs;
	wire [2:0] lut_val = mono_lut[lut_ofs +: 3];

	// Address formation
	// A mono build always takes the compatibility-mode path: the K1GE has no
	// CP.C and no K2GE palette region, so MODE is forced.
	// The address is unused there anyway - the RAM is bypassed below -
	// but forming it keeps the register meaningful in a savestate and keeps
	// the two paths from needing separate reset values.

	wire compat = mode_compat || mono_strap;

	reg [7:0] pal_idx;

	always @* begin
		if (src_win)     pal_idx = {5'b11111, oowc};             // 248..255
		else if (src_bg) pal_idx = {5'b11110, bgc};              // 240..247
		else if (compat) pal_idx = {2'b11, layer, p_c, lut_val}; // 192..239
		else             pal_idx = {layer, code, pix};           //   0..191
	end

	// Mono gray level
	// Code 0 is lightest and code 7 darkest (K1GE 3-7-1), so the level is
	// the inverted code, expanded 3 -> 4 bits by replicating the top bit:
	// 0 -> 0xF and 7 -> 0x0 exactly.

	reg [2:0] mono_code;
	reg       mono_black;

	always @* begin
		if (src_win) begin
			mono_code  = oowc;
			mono_black = (P_MONO_BG_RAMP == 0);
		end else if (src_bg) begin
			// 0x8118 exists on a K1GE and selects the clear-background
			// gradation code when BGON is valid.  An invalid BGON falls back
			// to code 0 - the rear-most, lightest screen (K1GE 3-7-1) -
			// and never to black.  See the header, note 2.
			mono_code  = (bgon == 2'b10) ? bgc : 3'd0;
			mono_black = 1'b0;
		end else begin
			mono_code  = lut_val;
			mono_black = 1'b0;
		end
	end

	wire [2:0] mono_lvl3 = ~mono_code;
	wire [3:0] mono_lvl4 = {mono_lvl3, mono_lvl3[2]};

	// The one registered stage

	reg [7:0] idx_q;
	reg [3:0] lvl_q;
	reg       mono_q;
	reg       black_q;
	reg       bg_q;      // this dot's colour comes from the background entry

	// Per-line background colour (P_BG_LINE_LATCH)
	// The first background dot of each line is read from palette RAM and its
	// colour held for the rest of the line, so a CPU write part way along the
	// line first shows up on the next line.

	reg [11:0] bg_line_q;   // the colour this line's background dots use
	reg        bg_valid_q;  // a background dot has already been resolved

	always @(posedge clk_sys) begin
		if (rst) begin
			bg_line_q  <= 12'd0;
			bg_valid_q <= 1'b0;
		end else if (ce) begin
			if (line_start) begin
				bg_valid_q <= 1'b0;
			end else if (form && bg_q && !bg_valid_q) begin
				// Sampled ON a `form`, which is what aligns it. `bg_q` and
				// `pal_rd_data` both describe the PREVIOUS dot at that moment:
				// idx_q is latched at the end of a form, the address appears the
				// next cycle, and the RAM answers the cycle after that, so the
				// data for dot P is on the bus from two cycles after its form
				// until two cycles after the following one.
				//
				// This relies on forms being at least two cycles apart, which is
				// the same assumption the module's existing output stage already
				// makes: `black_q`/`lvl_q` are latched on the form and combined
				// with `pal_rd_data` after the read latency.
				bg_line_q  <= pal_rd_data[11:0];
				bg_valid_q <= 1'b1;
			end
		end
	end

	always @(posedge clk_sys) begin
		if (rst) begin
			idx_q   <= 8'd0;
			lvl_q   <= 4'd0;
			mono_q  <= 1'b0;
			black_q <= 1'b0;
			bg_q    <= 1'b0;
		end else if (ss_wren) begin
			idx_q   <= ss_wdata[15:8];
			lvl_q   <= ss_wdata[7:4];
			mono_q  <= ss_wdata[3];
			black_q <= ss_wdata[2];
			bg_q    <= ss_wdata[1];
		end else if (ce && form) begin
			idx_q   <= pal_idx;
			lvl_q   <= mono_lvl4;
			mono_q  <= mono_strap;
			black_q <= mono_strap ? mono_black : bg_black;
			bg_q    <= src_bg;
		end
	end

	assign pal_rd_addr = idx_q;

	// Output dot
	// Palette entry format: D3:0 = R, D7:4 = G, D11:8 = B, D15:12 do not exist
	// and read 0 (K2GE 4-15 Table 18).  k2ge_vram masks them on the write
	// path, so no read-side mask is needed - but the bits are still ignored
	// here rather than trusted.

	// A background dot takes the per-line fetched word; every other source still
	// reads palette RAM per dot.  See P_BG_LINE_LATCH in the port list.
	wire [11:0] ram_rgb  = ((P_BG_LINE_LATCH != 0) && bg_q && bg_valid_q)
	                       ? bg_line_q : pal_rd_data[11:0];
	wire [11:0] mono_rgb = {lvl_q, lvl_q, lvl_q};

	wire [11:0] out_rgb = black_q ? 12'h000 : (mono_q ? mono_rgb : ram_rgb);

	assign pal_r = out_rgb[3:0];
	assign pal_g = out_rgb[7:4];
	assign pal_b = out_rgb[11:8];

	// Savestate image

	// Bit 1 carries `bg_q`.  `bg_line_q`/`bg_valid_q` are not saved: a
	// restore lands with `bg_valid_q` clear and the next background dot
	// resolves live.
	assign ss_state = {idx_q, lvl_q, mono_q, black_q, bg_q, 1'b0};

	// Deliberately unread inputs
	// The palette entry is twelve bits wide; D15:12 do not exist.  Savestate
	// word bit 0 is still spare and reads back 0 (bit 1 is now `bg_q`).
	wire unused_ok = &{1'b0, pal_rd_data[15:12], ss_wdata[0], 1'b0};

endmodule
