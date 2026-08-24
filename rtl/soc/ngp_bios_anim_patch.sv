// Copyright (c) 2026 Jamie Blanks

// ngp_bios_anim_patch -- the `Skip BIOS Animation` option, done on the BIOS's
// NORMAL launch path.
//
// This is a MiSTer session-management bridge, not console hardware, and it sits
// beside the BIOS loader for the same reason the loader itself is not console
// hardware: a real NPK2 board has a mask ROM and no download port at all. All
// this block does is substitute ONE 16-bit word of the loaded BIOS image while
// the machine is held in reset.
//
// The colour BIOS boot decision, decoded:
//
//   00ff1b28  bit 0x7,(0x00006f86)   ; User_Answer bit 7 = resume request
//   00ff1b2c  jrl NZ,0x00ff1d07      ; set   -> RESUME path -> launch 0xFF1E51
//   00ff1b2f  ...                    ; clear -> NORMAL path
//   00ff1b96  ld A,0x3               ; \ VECT_SYSFONTSET: char RAM 0xA000-0xAFFF
//   00ff1b98  calr 0x00ff8d8a        ; /   (this is what puts the SNK logo tiles
//                                    ;      at 0xA1C0 -- see below)
//   00ff1bc0  bit 0x4,(0x00006f83)   ; first-boot setup UI requested?
//   00ff1be2  ld A,0x2
//   00ff1be4  calr 0x00ff239d        ; install VBlank handler variant 2
//   00ff1be7  calr 0x00ff530c        ; <<< THE EYE-CATCH. UNCONDITIONAL.
//   00ff1bf4  calr 0x00ff6840        ; low-battery screen (also on the resume path)
//   00ff1c3f  calr 0x00ff2e77        ; the normal launch join
//   00ff1c61  set 0x6,(0x00006f84)   ; User_Boot bit 6 = Power-ON
//   00ff1c65  ld (0x00006f86),0x0    ; User_Answer cleared
//   00ff1c7a  pushw (0x00006c02) ... ; the normal launch
//
// There is no work-RAM flag that skips 0xFF530C. The only conditional between
// 0xFF1B2F and 0xFF1C3F is the first-boot setup UI at 0xFF1BC0, and the
// eye-catch call is not inside it, so on the normal path the animation can only
// be skipped by stopping the CPU from executing that call.
//
// This block does the smallest possible version of that: it turns the FIRST
// BYTE of the eye-catch routine into `ret` (0x0E). The call still happens and
// returns immediately, the boot continues down the normal path, and the game
// is launched at 0xFF1C7A with User_Boot = 0x40 (Power-ON) and User_Answer =
// 0x00 -- exactly the documented cold-boot state. Nothing false is told to the
// cartridge. Reaching the same skip by writing the BIOS's own resume request
// into work RAM instead would launch through 0xFF1E1D with User_Boot bit 5
// (Resume) set, which is a claim the game acts on and which is false on a cold
// boot, because the work RAM the game is told to restore was just cleared.
//
//   colour BIOS  eye-catch entry 0xFF530C  = file offset 0x530C = word 0x2986
//   mono   BIOS  eye-catch entry 0xFF4618  = file offset 0x4618 = word 0x230C
//
// Both routines begin `f1 20 80 00 00  ld (0x00008020),0x0`, so the word that
// carries the entry byte reads 0x20F1 in BOTH images, and the patched word is
// 0x200E. The high byte is left alone; it is the second byte of an instruction
// that is now unreachable. Neither entry is referenced by any pointer, and each
// has exactly one `calr` to it.
//
// The eye-catch is often said to leave the SNK logo tiles at char RAM 0xA1C0
// that Metal Slug - 2nd Mission checks. Measured here, those tiles are
// VECT_SYSFONTSET's, not the eye-catch's: VECT_SYSFONTSET (0xFF8D8A) expands the
// 1bpp font at 0xFF8DCF into char RAM 0xA000-0xAFFF at palette 3, so glyphs
// 0x1C-0x1F land at 0xA1C0-0xA1FF and reproduce the 64-byte reference pattern
// Metal Slug 2 carries at its own 0x28DCC4 byte for byte; the BIOS calls it on
// the normal path (0xFF1B98) as well as the resume path (0xFF1D69); and all the
// eye-catch does at 0xFF5522-0xFF554E is re-derive those 64 bytes in place,
// keeping every pixel whose colour index has bit 0 set, which on palette-3 font
// data (indices 0 and 3 only) is the identity. Skipping the eye-catch therefore
// cannot fail the check.
//
// The target word alone is not an image identity: an unrelated or modified
// 64-KiB image can carry 0x20F1 at the same address. The patch therefore also
// requires three loader words outside the patched target:
//
//   byte offset       0x0000  0x2000  0x4000
//   colour word       0x31F3  0xF0F1  0x1923
//   mono word         0x31F3  0xD104  0xF1F5
//
// All three arrive before both eye-catch targets in the loader's ascending
// stream. Only their match bits are retained; no BIOS data is buffered. An
// unknown or incomplete image stays disarmed, even when its target word is
// 0x20F1. The loader's word-0 write starts a fresh verdict for that image.
//
// Writes are issued only while `reset` is asserted, and never on a cycle the
// loader is using the port, so the selected value takes effect on the next reset
// or cartridge load and the write stays off a running CPU's fetch path.

module ngp_bios_anim_patch
(
	input  wire        clk,
	input  wire        reset,           // machine reset; also high during a BIOS download

	// The OSD option, active high = skip the animation.
	input  wire        skip_animation,

	// Loader snoop. These are the same wires the BIOS BRAM port B sees.
	input  wire        bios_wr,
	input  wire        bios_sel,        // 0 = colour image, 1 = mono image
	input  wire [14:0] bios_addr,       // word address
	input  wire [15:0] bios_data,

	// One-word write request, to be merged into the loader port with the
	// LOADER taking priority.
	output wire        patch_wr,
	output wire        patch_sel,
	output wire [14:0] patch_addr,
	output wire [15:0] patch_data
);

	// Colour BIOS 0xFF530C, mono BIOS 0xFF4618: the eye-catch entry points.
	localparam [14:0] W_ADDR_COL = 15'h2986;
	localparam [14:0] W_ADDR_MONO = 15'h230C;

	// Bounded image signature, as word addresses.
	localparam [14:0] W_SIG_ADDR0 = 15'h0000;   // byte 0x0000
	localparam [14:0] W_SIG_ADDR1 = 15'h1000;   // byte 0x2000
	localparam [14:0] W_SIG_ADDR2 = 15'h2000;   // byte 0x4000
	localparam [15:0] W_SIG_COL0  = 16'h31F3;
	localparam [15:0] W_SIG_COL1  = 16'hF0F1;
	localparam [15:0] W_SIG_COL2  = 16'h1923;
	localparam [15:0] W_SIG_MONO0 = 16'h31F3;
	localparam [15:0] W_SIG_MONO1 = 16'hD104;
	localparam [15:0] W_SIG_MONO2 = 16'hF1F5;

	// `f1 20` = the first two bytes of `ld (0x00008020),0x0`, little endian.
	localparam [15:0] W_ORIGINAL = 16'h20F1;
	// Low byte replaced by `ret`; the high byte is now unreachable padding.
	localparam [15:0] W_PATCHED  = 16'h200E;

	// Armed when the loader was seen to deliver the expected original word to
	// the expected address. No reset: this is download-derived state, exactly
	// like the cartridge header capture, and it must survive machine reset
	// because reset is asserted for the whole download.
	reg armed_col;
	reg armed_mono;
	// What this block last wrote into each image. Meaningless until armed.
	reg applied_col;
	reg applied_mono;
	// One bit per non-target signature word. These are comparison results,
	// not captured BIOS contents.
	reg [2:0] signature_col;
	reg [2:0] signature_mono;

	initial begin
		armed_col    = 1'b0;
		armed_mono   = 1'b0;
		applied_col  = 1'b0;
		applied_mono = 1'b0;
		signature_col  = 3'b000;
		signature_mono = 3'b000;
	end

	wire load_col  = bios_wr && !bios_sel;
	wire load_mono = bios_wr &&  bios_sel;

	wire arm_col  = load_col  && (bios_addr == W_ADDR_COL) &&
	                (bios_data == W_ORIGINAL) && (signature_col == 3'b111);
	wire arm_mono = load_mono && (bios_addr == W_ADDR_MONO) &&
	                (bios_data == W_ORIGINAL) && (signature_mono == 3'b111);

	// The loader always starts a fresh image at word 0.
	wire rearm_col  = load_col  && (bios_addr == W_SIG_ADDR0);
	wire rearm_mono = load_mono && (bios_addr == W_SIG_ADDR0);

	wire want_col  = armed_col  && (applied_col  != skip_animation);
	wire want_mono = armed_mono && (applied_mono != skip_animation);

	// Loader has priority; one image at a time, colour first.
	wire issue      = reset && !bios_wr && (want_col || want_mono);
	wire issue_mono = want_col ? 1'b0 : 1'b1;

	assign patch_wr   = issue;
	assign patch_sel  = issue_mono;
	assign patch_addr = issue_mono ? W_ADDR_MONO : W_ADDR_COL;
	assign patch_data = skip_animation ? W_PATCHED : W_ORIGINAL;

	always @(posedge clk) begin
		if (rearm_col) begin
			armed_col   <= 1'b0;
			applied_col <= 1'b0;
			signature_col <= {2'b00, (bios_data == W_SIG_COL0)};
		end else begin
			if (load_col && (bios_addr == W_SIG_ADDR1))
				signature_col[1] <= (bios_data == W_SIG_COL1);
			if (load_col && (bios_addr == W_SIG_ADDR2))
				signature_col[2] <= (bios_data == W_SIG_COL2);
			if (arm_col) begin
				armed_col   <= 1'b1;
				applied_col <= 1'b0;
			end else if (issue && !issue_mono) begin
				applied_col <= skip_animation;
			end
		end

		if (rearm_mono) begin
			armed_mono   <= 1'b0;
			applied_mono <= 1'b0;
			signature_mono <= {2'b00, (bios_data == W_SIG_MONO0)};
		end else begin
			if (load_mono && (bios_addr == W_SIG_ADDR1))
				signature_mono[1] <= (bios_data == W_SIG_MONO1);
			if (load_mono && (bios_addr == W_SIG_ADDR2))
				signature_mono[2] <= (bios_data == W_SIG_MONO2);
			if (arm_mono) begin
				armed_mono   <= 1'b1;
				applied_mono <= 1'b0;
			end else if (issue && issue_mono) begin
				applied_mono <= skip_animation;
			end
		end
	end

endmodule
