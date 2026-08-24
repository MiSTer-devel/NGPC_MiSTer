// Copyright (c) 2026 Jamie Blanks

// K2GE register file.  Owns the CPU-visible registers of the graphics chip --
// everything in 0x8000-0x81FF and 0x8400-0x87FF -- and the three effect-timing
// latch groups that decide WHEN a written value reaches the picture.  It does
// not own palette RAM (0x8200-0x83FF) or any VRAM region; those belong to
// k2ge_vram and are decoded around this module.
//
// Immediate  the interrupt enables 0x8000, MODE 0x87E2, the unlock byte 0x87F0
//            and the LED pair 0x8400/0x8402.  "The value set in this register
//            takes effect immediately" (K1GE 3-4-4, 3-8, 3-9; K2GE 4-12..4-15).
//            The eighteen mono LUT bytes 0x8101-0x8117 carry the same sentence
//            (K1GE 3-7) but are treated as per line on the scanout side --
//            see P_MONO_LUT_LINE_LATCH.
// Per line   0x8012 NEG/OOWC, 0x8020/21 PO, 0x8030 P.F, 0x8032-35 the scroll
//            offsets and 0x8118 BGC/BGON.  "Setting in this register is
//            reflected in the next line being drawn on screen" (K1GE 3-6;
//            K2GE 4-6, 4-11).  Also 0x8002 WBA.H and 0x8004 WSI.H, which Metal
//            Slug 1/2 update after RAS.V reaches 128 to draw the lower status
//            bar and which therefore have to reach the next line.
// Per frame  0x8003 WBA.V, 0x8005 WSI.V and 0x8006 REF.  The vertical window
//            fields derive Vint and keep the TechRef's next-frame rule
//            (K1GE 3-4-10); REF is per frame under every reading.
//
// The live register is what the CPU reads back; the latched copy is what the
// render and scanout pipelines consume.  A CPU write landing on the same
// clk_sys edge as a latch strobe is NOT captured by that strobe -- both flops
// sample the same edge, so the copy takes the old value and the write lands one
// line (or frame) later.  That is what two flip-flops sharing a clock do.
//
// Parameters
//   P_WINDOW_H_LATCH / P_WINDOW_V_LATCH  window latch timing, per axis.  The
//     TechRefs say next frame (K1GE 3-4-10); the SNK register table (K2GEres)
//     marks WBA/WSI "H" (horizontal blanking, i.e. per line), the same marking
//     it gives the scroll offsets.  Split per axis because the horizontal
//     choice is settled by game behaviour and the vertical one is not.
//   P_RENDER_LATCH  per-line latch phase for the composition-side registers.
//     0 = the start of the pass, so a write during raster line L is latched at
//     the start of line L+1 and reaches display row L+1 (K1GE 3-4-9).  1 closes
//     the latch at the end of the previous line's drawing period instead, so a
//     write inside dots 484..514 of line L slips to display row L+2.
//   P_BGC_RESET / P_MONO_BGC_RESET  BGC reset value.  The K2GE TechRef states
//     0x00 while the K1GE-titled register table prints 0x07 with BGON D7:6
//     fixed to 10, so each part takes the reset its own document prints
//     (K2GE 4-6; K2GEres).
//
// Write-only registers read 0xFF on the CPU bus and appear only on the
// savestate tap: 0x87E0 (2D reset strobe), 0x87F0 (the unlock byte),
// 0x8700-0x870C (the seven mono-BIOS LCD-init bytes) and 0x87F2/0x87F4.
//
// Everything mapped but not listed reads 0x00 and ignores writes, including the
// "Access not allowed" holes at 0x8100/04/08/0C/10/14 (K2GE 4-12) and the whole
// of the unlisted address space (K1GE 3-2, "PLEASE DO NOT ACCESS").  Reading
// 0x00 there is this design's choice, not a documented behaviour.
//
// CPU bus: zero wait, always.  The bus is sampled on the clk_sys cycle where
// ce_6m144 is high and writes commit on that edge.  Reads are a combinational
// mux of flip-flops which the chip-level read mux registers at the sampling
// tick; there are eight clk_sys cycles inside one dot period, so that path has
// seven cycles of slack.  The 16-bit bus is byte-laned -- cpu_din[7:0] is the
// even byte and cpu_din[15:8] the odd, enabled by cpu_we[0] and cpu_we[1] --
// and every register lives at a single byte address, so in exactly one lane.

module k2ge_mmr
#(
	// 0 = latch per frame (K1GE 3-4-10), 1 = per line (K2GEres).
	// Production uses horizontal per-line and vertical per-frame.
	parameter       P_WINDOW_H_LATCH = 1'b1,
	parameter       P_WINDOW_V_LATCH = 1'b0,
	// 0 = composition-side registers latch at the start of the pass (a write
	//     in raster line L reaches display row L+1); 1 = at the end of the
	//     previous line's drawing period, which drops writes in dots 484..514.
	parameter       P_RENDER_LATCH = 1'b0,
	// Mono LUT effect timing.  1 = the LUT feeding the colour lookup is the
	// copy captured for the row being PAINTED, on the same two-stage
	// scanout-side path as P.F/NEG/OOWC/BGC.  0 = the live register applied
	// per dot, the literal reading of "takes effect immediately" (K1GE 3-7).
	// This reads that sentence as "not deferred to the next frame" -- the
	// contrast it draws with the per-line and per-frame groups -- rather than
	// a claim that the LUT is resampled per dot inside a row.
	//
	// Why 1 is production: KOF R-1 switches scroll 1 from status-bar mode to
	// playfield mode once per frame at raster line 31, writing SC1SO.H and
	// four SC1PLT bytes as ONE action.  SC1SO.H is per-line latched and lands
	// on the next composed row; with 0 the LUT bytes land partway through the
	// row being painted (hdot 385-444 is visible x~128-148 at three dots per
	// pixel) and recolour its tail, so the two halves of one decision reach
	// different rows.  Same defect class as the P.F layer latch.  It became
	// visible when the panel phase moved from `vline - 2` to `vline - 1`,
	// which put the split on display row 30, the bottom of the status bar.
	//
	// NOT confirmed against silicon.  Real NGPC hardware running KOF R-1 in
	// K1GE compatibility mode would settle it: with 0 the last row of the
	// status bar under the right-hand portraits carries a few pixels of the
	// wrong grey, drifting frame to frame; with 1 it is clean.
	parameter       P_MONO_LUT_LINE_LATCH = 1'b1,
	// K2GE 0x8118 reset (K2GE 4-6).
	parameter [7:0] P_BGC_RESET      = 8'h00,
	// True-NGP reset selected by mono_strap (K2GEres).  See header note 3.
	parameter [7:0] P_MONO_BGC_RESET = 8'h07
)
(
	input  wire        clk_sys,        // 49.152 MHz
	input  wire        ce_6m144,       // dot-clock enable, 1 of 8
	input  wire        rst,            // synchronous power-on reset
	input  wire        mono_strap,     // reset-latched chip variant: 1 = K1GE

	// --- CPU bus face -----------------------------------------------------
	input  wire [13:0] cpu_a,          // byte address inside 0x8000-0xBFFF
	input  wire [15:0] cpu_din,
	output wire [15:0] cpu_dout,
	output wire        cpu_hit,        // this module answers the address
	input  wire        cpu_cs,         // window select from the SoC decoder
	input  wire        cpu_rd,         // read strobe (no side effects here)
	input  wire [1:0]  cpu_we,         // [0] = even byte, [1] = odd byte

	// --- raster interface (k2ge_vtimer) -----------------------------------
	input  wire [7:0]  ras_h,          // 0x8008 readback
	input  wire [7:0]  ras_v,          // 0x8009 readback
	input  wire        blnk,           // 0x8010 D6
	input  wire        covr,           // 0x8010 D7
	input  wire        line_start,     // entering dot 0 of any line
	input  wire        frame_start,    // entering dot 0 of line 0
	input  wire        hdraw_end,      // entering dot P_HDRAW_END

	// --- back to the raster timer -----------------------------------------
	output wire        vi_e,           // 0x8000 D7
	output wire        hi_e,           // 0x8000 D6
	output wire [7:0]  ref_last,       // 0x8006, per-frame latched
	output wire [8:0]  vint_line,      // WBA.V + WSI.V, NINE bits - see the assign
	output wire        soft_reset,     // 0x87E0 = 0x52 wrote this cycle

	// --- latched register set for the render and scanout blocks ------------
	output wire [7:0]  win_wba_h,
	output wire [7:0]  win_wba_v,
	output wire [7:0]  win_wsi_h,
	output wire [7:0]  win_wsi_v,
	output wire [7:0]  ln_po_h,
	output wire [7:0]  ln_po_v,
	output wire [7:0]  ln_s1so_h,
	output wire [7:0]  ln_s1so_v,
	output wire [7:0]  ln_s2so_h,
	output wire [7:0]  ln_s2so_v,
	output wire        ln_p_f,         // 0 = plane 1 in front, composition side
	output wire        sc_p_f,         // the same field one line later, for the
	                                   // colour lookup that paints that row
	output wire [53:0] sc_mono_lut,    // mono LUT on that same paint-side path
	                                   // when P_MONO_LUT_LINE_LATCH = 1
	output wire        ln_neg,
	output wire [2:0]  ln_oowc,
	output wire [1:0]  ln_bgon,        // valid only when == 2'b10
	output wire [2:0]  ln_bgc,

	// --- immediate-effect outputs -----------------------------------------
	output wire        mode_compat,    // 0x87E2 D7: 1 = K1GE upper-palette mode
	output wire [53:0] mono_lut,       // 18 x 3 bits, index 0 = 0x8101

	// --- misc chip pads ----------------------------------------------------
	input  wire        inp0,           // read at 0x87FE D6
	output wire        led,            // LED drive pad

	// --- savestate internals slice ----------------------------------------
	// Words 0..5, 7, 8, 15 and 16.  Word 0 is shared with k2ge_vtimer on a
	// disjoint bit split: this module drives 63:53 and reads 0 elsewhere, the
	// timer drives 2:0, and the parent ORs the images.
	input  wire [4:0]  ss_addr,
	input  wire [63:0] ss_wdata,
	input  wire        ss_wren,
	output wire [63:0] ss_rdata,

	// --- pause / drain handshake -------------------------------------------
	input  wire        pause_req,
	output wire        pause_ready
);

	// Address decode
	// The register file is 0x8000-0x81FF and 0x8400-0x87FF.  0x8200-0x83FF is
	// palette RAM and 0x8800 and up is VRAM; both belong to k2ge_vram, so this
	// module reports `cpu_hit` low there and the chip mux picks another source.

	// The lane, not address bit 0, says which byte a register sees, so cpu_a[0]
	// takes no part in the decode.
	wire [9:0]  wa   = cpu_a[10:1];                        // word address
	wire [10:0] ba_e = {wa, 1'b0};                         // even-lane byte addr
	wire [10:0] ba_o = {wa, 1'b1};                         // odd-lane byte addr

	assign cpu_hit = (cpu_a[13:11] == 3'b000) && (cpu_a[10:9] != 2'b01);

	wire [7:0] d_e = cpu_din[7:0];
	wire [7:0] d_o = cpu_din[15:8];

	wire wr_e = ce_6m144 && cpu_cs && cpu_hit && cpu_we[0];
	wire wr_o = ce_6m144 && cpu_cs && cpu_hit && cpu_we[1];

	// The mono LUT occupies 0x8100-0x8117 as six groups of four bytes, the
	// first byte of each group being an "Access not allowed" hole (K1GE
	// 3-7).  Flatten (group, column) to a 0..17 index: idx = group*3 + col - 1.
	// The multiply by three is a shift and an add, not a DSP.
	wire        lut_reg   = (wa[9:4] == 6'h08) && (wa[3:1] <= 3'd5);
	wire [2:0]  lut_grp   = wa[3:1];
	wire [4:0]  lut_base  = {2'b00, lut_grp} + {1'b0, lut_grp, 1'b0};
	wire [4:0]  lut_idx_e = lut_base + 5'd1;               // column 2
	wire [4:0]  lut_idx_o = lut_base + {3'b000, wa[0], 1'b1} - 5'd1;
	wire        lut_hit_e = lut_reg && wa[0];              // 0x8102, 0x8106, ...
	wire        lut_hit_o = lut_reg;                       // 0x8101, 0x8103, ...

	// The seven undocumented mono-BIOS LCD-init bytes sit at 0x8700-0x870C on
	// even addresses only.
	wire        lcd_hit_e = (wa[9:3] == 7'h70) && (wa[2:0] <= 3'd6);
	wire [2:0]  lcd_idx   = wa[2:0];

	// The register file

	reg        vi_e_q;                                     // 0x8000 D7
	reg        hi_e_q;                                     // 0x8000 D6
	reg [5:0]  int_dc_q;                                   // 0x8000 D5:0, D.C

	reg [7:0]  wba_h_q, wba_v_q, wsi_h_q, wsi_v_q;         // 0x8002-0x8005
	reg [7:0]  ref_q;                                      // 0x8006
	reg [7:0]  ctl2d_q;                                    // 0x8012
	reg [7:0]  po_h_q, po_v_q;                             // 0x8020, 0x8021
	reg [7:0]  pf_q;                                       // 0x8030
	reg [7:0]  s1so_h_q, s1so_v_q, s2so_h_q, s2so_v_q;     // 0x8032-0x8035
	reg [2:0]  lut_q [0:17];                               // 0x8101-0x8117
	reg [7:0]  bgc_q;                                      // 0x8118
	reg [4:0]  ledon_q;                                    // 0x8400 D7:3
	reg [7:0]  ledfrq_q;                                   // 0x8402
	reg [7:0]  lcd_q [0:6];                                // 0x8700-0x870C
	reg        mode_q;                                     // 0x87E2 D7
	reg [7:0]  unlock_q;                                   // 0x87F0
	reg [7:0]  undoc_f2_q, undoc_f4_q;                     // 0x87F2, 0x87F4

	// Latched copies.
	reg [7:0]  l_po_h, l_po_v;
	reg [7:0]  l_s1so_h, l_s1so_v, l_s2so_h, l_s2so_v;
	reg        l_p_f;
	reg        l_neg;
	reg [2:0]  l_oowc;
	reg [1:0]  l_bgon;
	reg [2:0]  l_bgc;
	reg [7:0]  f_wba_h, f_wba_v, f_wsi_h, f_wsi_v;
	reg [7:0]  f_ref;

	// The SECOND line of latch on the scanout-side group; see "The two groups
	// are one raster line apart" below.
	reg        s_p_f;
	reg [53:0] l_lut;                                      // composition side
	reg [53:0] s_lut;                                      // paint side
	reg        s_neg;
	reg [2:0]  s_oowc;
	reg [1:0]  s_bgon;
	reg [2:0]  s_bgc;

	// LED flasher.
	reg [15:0] led_div;
	reg [7:0]  led_phase;
	reg        led_q;

	reg        pause_ready_q;

	integer i;

	// 0x8400 always reads with D2:0 = 1: "D0 ~ D2 is set to a constant 1 to
	// avoid LED from going off due to a program crash" (K1GE 3-8).
	wire [7:0] ledctl = {ledon_q, 3'b111};

	// 0x87E0 accepts only 0x52; "Value other than 0X52 is ignored" (K1GE
	// 3-10).  The BIOS writes 0x53 and 0x47 here, so this path is barely
	// exercised on a real boot.
	assign soft_reset = wr_e && (ba_e == 11'h7E0) && (d_e == 8'h52);

	// 0x87E2 and REF 0x8006 are writable only while the unlock byte holds
	// 0xAA.  No SNK document mentions 0x87F0 at all, but the BIOS uses it, so
	// the gate stays.
	//
	// The gate is applied in both modes.  Neither the K1GE TechRef nor K2GEres
	// documents 0x8006 at all, but REF = 0 collapses the raster identically on
	// either part and no mono title is known to write it.
	//
	// (K2GE p.21) marks REF "Locked; priority user only" -- the same words it
	// uses for MODE.  An ungated REF is not a harmless liberty: REF is the
	// last line number, so REF = 0 collapses the raster to ONE line, which
	// stops BLNK, latches Vint high with no further rising edge, and starves
	// the CRT presenter's frame capture until it faults and parks the whole
	// machine through the pause tree.
	//
	// Some homebrew titles reach REF by accident: a misaligned 16-bit store to
	// 0x8005 lands its high byte on 0x8006 and zeroes REF.  Those titles run on
	// physical hardware, which is the evidence that silicon ignores the write.
	// No retail title writes REF.
	wire regs_unlocked = (unlock_q == 8'hAA);

	// 0x87E2 MODE does not exist on a K1GE.  It is a K2GE addition - (K2GE
	// 5-1) item 4 lists the mode-selection register among the things ADDED to
	// the K1GE, and the K1GE memory map puts 0x87E2 inside the "open area ...
	// PLEASE DO NOT ACCESS. (Future expansion)" of the register block
	// (K1GE 3-2).  On a mono build the write is therefore dropped and the
	// address reads back as unmapped (0x00), which is this design's stated
	// behaviour for an address no register owns.  The DISPLAY path never
	// depended on the flop here anyway: k2ge.sv forces compatibility mode from
	// the strap through `mode_compat_eff`, and k2ge_palette does the same for
	// itself, so `mode_q` reading 0 in mono changes nothing that is drawn.
	wire mode_reg_exists = !mono_strap;

	// Savestate tap

	wire ss_wr = ss_wren && pause_req;                     // honoured while paused

	// Flat image of the mono LUT: index 0 (= 0x8101) in the high three bits.
	wire [53:0] lut_flat = {lut_q[ 0], lut_q[ 1], lut_q[ 2], lut_q[ 3], lut_q[ 4],
	                        lut_q[ 5], lut_q[ 6], lut_q[ 7], lut_q[ 8], lut_q[ 9],
	                        lut_q[10], lut_q[11], lut_q[12], lut_q[13], lut_q[14],
	                        lut_q[15], lut_q[16], lut_q[17]};

	wire [55:0] lcd_flat = {lcd_q[0], lcd_q[1], lcd_q[2], lcd_q[3],
	                        lcd_q[4], lcd_q[5], lcd_q[6]};

	reg [63:0] ss_rdata_r;

	always @* begin
		case (ss_addr)
			// Word 0, this module's eleven bits.  k2ge_vtimer owns 2:0.
			5'd0:    ss_rdata_r = {vi_e_q, hi_e_q, mode_q, unlock_q, 53'd0};
			5'd1:    ss_rdata_r = {wba_h_q, wba_v_q, wsi_h_q, wsi_v_q, ref_q,
			                       18'd0, int_dc_q};
			5'd2:    ss_rdata_r = {s1so_h_q, s1so_v_q, s2so_h_q, s2so_v_q,
			                       pf_q, 24'd0};
			5'd3:    ss_rdata_r = {po_h_q, po_v_q, ctl2d_q, bgc_q, 32'd0};
			5'd4:    ss_rdata_r = {10'd0, lut_flat};
			5'd5:    ss_rdata_r = {ledctl, ledfrq_q, 8'd0, led_phase,
			                       led_div, 15'd0, led_q};
			5'd7:    ss_rdata_r = {l_s1so_h, l_s1so_v, l_s2so_h, l_s2so_v,
			                       l_po_h, l_po_v, l_p_f, l_neg, l_oowc,
			                       l_bgon, l_bgc, 6'd0};
			// vint_line is DERIVED from the four window bytes above and is
			// never restored from this word; it is published so a savestate
			// dump shows the derived sum.
			5'd8:    ss_rdata_r = {f_wba_h, f_wba_v, f_wsi_h, f_wsi_v, f_ref,
			                       vint_line, 15'd0};
			5'd15:   ss_rdata_r = {lcd_flat, 8'd0};
			5'd16:   ss_rdata_r = {undoc_f2_q, undoc_f4_q, 48'd0};
			default: ss_rdata_r = 64'd0;
		endcase
	end

	assign ss_rdata = ss_rdata_r;

	// Live registers

	wire [7:0] bgc_reset = mono_strap ? P_MONO_BGC_RESET : P_BGC_RESET;
	// One block, one driver per flop.  Priority: hardware reset, the 2D
	// software reset ("All register settings are set to the initialization
	// values when this register is used" (K1GE 3-10 caution 2)), a savestate
	// restore, then the CPU.

	always @(posedge clk_sys) begin
		if (rst || soft_reset) begin
			vi_e_q     <= 1'b0;
			hi_e_q     <= 1'b0;
			int_dc_q   <= 6'd0;
			wba_h_q    <= 8'h00;
			wba_v_q    <= 8'h00;
			wsi_h_q    <= 8'hFF;
			wsi_v_q    <= 8'hFF;
			ref_q      <= 8'hC6;
			ctl2d_q    <= 8'h00;
			po_h_q     <= 8'h00;
			po_v_q     <= 8'h00;
			pf_q       <= 8'h00;
			s1so_h_q   <= 8'h00;
			s1so_v_q   <= 8'h00;
			s2so_h_q   <= 8'h00;
			s2so_v_q   <= 8'h00;
			bgc_q      <= bgc_reset;
			ledon_q    <= 5'h1F;                           // 0x8400 = 0xFF
			ledfrq_q   <= 8'h80;                           // 1.3 s
			mode_q     <= 1'b0;
			unlock_q   <= 8'h00;
			undoc_f2_q <= 8'h00;
			undoc_f4_q <= 8'h00;
			for (i = 0; i < 18; i = i + 1) lut_q[i] <= 3'h7;
			for (i = 0; i < 7;  i = i + 1) lcd_q[i] <= 8'h00;
		end else if (ss_wr) begin
			case (ss_addr)
				5'd0: begin
					vi_e_q   <= ss_wdata[63];
					hi_e_q   <= ss_wdata[62];
					mode_q   <= ss_wdata[61];
					unlock_q <= ss_wdata[60:53];
				end
				5'd1: begin
					wba_h_q  <= ss_wdata[63:56];
					wba_v_q  <= ss_wdata[55:48];
					wsi_h_q  <= ss_wdata[47:40];
					wsi_v_q  <= ss_wdata[39:32];
					ref_q    <= ss_wdata[31:24];
					int_dc_q <= ss_wdata[5:0];
				end
				5'd2: begin
					s1so_h_q <= ss_wdata[63:56];
					s1so_v_q <= ss_wdata[55:48];
					s2so_h_q <= ss_wdata[47:40];
					s2so_v_q <= ss_wdata[39:32];
					pf_q     <= ss_wdata[31:24];
				end
				5'd3: begin
					po_h_q   <= ss_wdata[63:56];
					po_v_q   <= ss_wdata[55:48];
					ctl2d_q  <= ss_wdata[47:40];
					bgc_q    <= ss_wdata[39:32];
				end
				5'd4: begin
					for (i = 0; i < 18; i = i + 1)
						lut_q[i] <= ss_wdata[53 - i*3 -: 3];
				end
				5'd5: begin
					ledon_q  <= ss_wdata[63:59];
					ledfrq_q <= ss_wdata[55:48];
				end
				5'd15: begin
					for (i = 0; i < 7; i = i + 1)
						lcd_q[i] <= ss_wdata[63 - i*8 -: 8];
				end
				5'd16: begin
					undoc_f2_q <= ss_wdata[63:56];
					undoc_f4_q <= ss_wdata[55:48];
				end
				default: ;
			endcase
		end else begin
			if (wr_e) begin
				case (ba_e)
					11'h000: {vi_e_q, hi_e_q, int_dc_q} <= d_e;
					11'h002: wba_h_q  <= d_e;
					11'h004: wsi_h_q  <= d_e;
					11'h006: if (regs_unlocked) ref_q <= d_e;
					11'h012: ctl2d_q  <= d_e;
					11'h020: po_h_q   <= d_e;
					11'h030: pf_q     <= d_e;
					11'h032: s1so_h_q <= d_e;
					11'h034: s2so_h_q <= d_e;
					11'h118: bgc_q    <= d_e;
					11'h400: ledon_q  <= d_e[7:3];
					11'h402: ledfrq_q <= d_e;
					11'h7E2: if (regs_unlocked && mode_reg_exists)
					             mode_q <= d_e[7];
					11'h7F0: unlock_q   <= d_e;
					11'h7F2: undoc_f2_q <= d_e;
					11'h7F4: undoc_f4_q <= d_e;
					default: ;
				endcase
				if (lut_hit_e) lut_q[lut_idx_e] <= d_e[2:0];
				if (lcd_hit_e) lcd_q[lcd_idx]   <= d_e;
			end
			if (wr_o) begin
				case (ba_o)
					11'h003: wba_v_q  <= d_o;
					11'h005: wsi_v_q  <= d_o;
					11'h021: po_v_q   <= d_o;
					11'h033: s1so_v_q <= d_o;
					11'h035: s2so_v_q <= d_o;
					default: ;
				endcase
				if (lut_hit_o) lut_q[lut_idx_o] <= d_o[2:0];
			end
		end
	end

	// Latch groups
	// Scanout-side per-line registers (NEG, OOWC, BGC/BGON) are captured at the
	// start of the line they affect.  Composition-side per-line registers
	// (scroll offsets, PO, P.F) are captured at the start of the composition
	// pass, or one drawing period earlier when P_RENDER_LATCH is 1 - see the
	// header.
	//
	// The scanout-side per-line group takes a SECOND stage so the value used
	// while painting display row R is the one captured for row R.  P.F needs
	// both copies: the ranks are decided during the composition pass, so they
	// take the first stage, while the palette derives the layer from the rank
	// while the row is being painted a line later and must see the value that
	// composition used.  Feeding the first stage to both makes the two halves
	// of one decision disagree for a line whenever P.F changes.  Work one
	// write at dot 200 of raster line X through both pipelines: the
	// composition group is latched at the start of line X+1, and the pass
	// there composes display row X+1; the scanout during line X+1 paints
	// display row X.  Without the second stage the same write would reach two
	// different displayed rows, while the documents give both groups the same
	// next-line contract (K1GE 3-4-9 / K2GE 4-4-8 "reflected in the next line
	// being drawn on screen"; K2GEres marks NEG, OOWC, BGC/BGON, WBA.H and
	// WSI.H "H" in its Display Timing column) and titles that program both
	// groups from one Timer/HINT handler depend on it.
	//
	// The WINDOW does not take that second stage, in either axis.  The clip is
	// applied while the panel paints, not while the pass composes, so the same
	// single per-line latch already lands it on the row being painted -- which
	// is one displayed row EARLIER than the same latch reaches the composition
	// group.  Samurai Shodown 2 is the case that settles it: its HINT handler
	// collapses WSI.H to 0 and resets the scroll to the HUD in one go on raster
	// line 136, and on hardware the collapsed window blanks display row 136
	// while the scroll takes effect on row 137.  Forcing the two to agree left
	// row 136 painting the leftover playfield, which is a coloured line under
	// the play area.  The chip has one latch; the one-row split between a
	// paint-time and a compose-time register is what that pipeline does.
	//
	// Horizontal and vertical window fields have independent latches.  REF
	// does not: it is per frame in both readings, and moving the total line
	// count mid-frame is not something either document describes.

	wire latch_line   = line_start;
	wire latch_render = P_RENDER_LATCH ? hdraw_end : line_start;
	wire latch_win_h  = P_WINDOW_H_LATCH ? line_start : frame_start;
	wire latch_win_v  = P_WINDOW_V_LATCH ? line_start : frame_start;

	always @(posedge clk_sys) begin
		if (rst || soft_reset) begin
			l_po_h   <= 8'h00;
			l_po_v   <= 8'h00;
			l_s1so_h <= 8'h00;
			l_s1so_v <= 8'h00;
			l_s2so_h <= 8'h00;
			l_s2so_v <= 8'h00;
			l_p_f    <= 1'b0;
			l_neg    <= 1'b0;
			l_oowc   <= 3'd0;
			l_bgon   <= bgc_reset[7:6];
			l_bgc    <= bgc_reset[2:0];
			f_wba_h  <= 8'h00;
			f_wba_v  <= 8'h00;
			f_wsi_h  <= 8'hFF;
			f_wsi_v  <= 8'hFF;
			f_ref    <= 8'hC6;
			s_p_f    <= 1'b0;
			l_lut    <= {18{3'h7}};
			s_lut    <= {18{3'h7}};
			s_neg    <= 1'b0;
			s_oowc   <= 3'd0;
			s_bgon   <= bgc_reset[7:6];
			s_bgc    <= bgc_reset[2:0];
		end else if (ss_wr && (ss_addr == 5'd4)) begin
			// Seed both latch stages from the same words, matching the
			// convention the second-stage group already uses, so the
			// restored machine is self-consistent immediately.
			l_lut <= ss_wdata[53:0];
			s_lut <= ss_wdata[53:0];
		end else if (ss_wr && (ss_addr == 5'd7)) begin
			l_s1so_h <= ss_wdata[63:56];
			l_s1so_v <= ss_wdata[55:48];
			l_s2so_h <= ss_wdata[47:40];
			l_s2so_v <= ss_wdata[39:32];
			l_po_h   <= ss_wdata[31:24];
			l_po_v   <= ss_wdata[23:16];
			l_p_f    <= ss_wdata[15];
			l_neg    <= ss_wdata[14];
			l_oowc   <= ss_wdata[13:11];
			l_bgon   <= ss_wdata[10:9];
			l_bgc    <= ss_wdata[8:6];
			// The second stage is NOT in the savestate word, and deliberately
			// so: adding it would change the state format and invalidate every
			// existing .ss file, to buy back at most one raster line of stale
			// background/NEG at the instant of a restore -- a restore already
			// resumes on a frame boundary, so the whole first line it can
			// affect is off-panel.  Seeding both stages from the same words
			// makes the machine self-consistent immediately.
			s_p_f    <= ss_wdata[15];
			s_neg    <= ss_wdata[14];
			s_oowc   <= ss_wdata[13:11];
			s_bgon   <= ss_wdata[10:9];
			s_bgc    <= ss_wdata[8:6];
		end else if (ss_wr && (ss_addr == 5'd8)) begin
			f_wba_h  <= ss_wdata[63:56];
			f_wba_v  <= ss_wdata[55:48];
			f_wsi_h  <= ss_wdata[47:40];
			f_wsi_v  <= ss_wdata[39:32];
			f_ref    <= ss_wdata[31:24];
		end else begin
			if (latch_line) begin
				l_neg  <= ctl2d_q[7];
				l_oowc <= ctl2d_q[2:0];
				l_bgon <= bgc_q[7:6];
				l_bgc  <= bgc_q[2:0];
			end
			if (latch_render) begin
				l_po_h   <= po_h_q;
				l_po_v   <= po_v_q;
				l_p_f    <= pf_q[7];
				l_s1so_h <= s1so_h_q;
				l_s1so_v <= s1so_v_q;
				l_s2so_h <= s2so_h_q;
				l_s2so_v <= s2so_v_q;
			end
			if (latch_win_h) begin
				f_wba_h <= wba_h_q;
				f_wsi_h <= wsi_h_q;
			end
			if (latch_win_v) begin
				f_wba_v <= wba_v_q;
				f_wsi_v <= wsi_v_q;
			end
			if (frame_start) begin
				f_ref <= ref_q;
			end
			// The second scanout-side stage. One line behind the first, so the
			// value in use while the panel paints display row R is the one
			// captured for row R.
			if (line_start) begin
				s_p_f   <= l_p_f;
				l_lut   <= lut_flat;
				s_lut   <= l_lut;
				s_neg   <= l_neg;
				s_oowc  <= l_oowc;
				s_bgon  <= l_bgon;
				s_bgc   <= l_bgc;
			end
		end
	end

	// LED flasher: a plain 16-bit divider off the dot clock.  6.144 MHz / 2^16 =
	// 93.75 Hz = 10.667 ms against the documented 10.6 ms unit, and 0x80 of
	// those is 1.365 s against the documented 1.3 s (K1GE 3-8, 3-9).  The
	// 32.768 kHz sub-clock has no power-of-two divide anywhere near 10.6 ms
	// (2^8 = 7.8 ms, 2^9 = 15.6 ms).  A consequence is that the LED stops
	// flashing in STOP-mode standby, when the main oscillator halts.
	//
	// The document is ambiguous with itself about duty: 0x8400 says "LED
	// flashes with (register value x 10.6 mS)
	// apart.  Flash cycle is set in 3-9" while 0x8402 says "Sets the LED flash
	// cycle ... 0X80 (1.3 S)" (K1GE 3-8, 3-9).  Read literally that is an
	// on-time from 0x8400 inside a cycle from 0x8402, which is what this does:
	// the phase counter runs 0..LEDFRQ-1 in 10.667 ms ticks and the LED is on
	// while the phase is below the 0x8400 value.  The alternative reading -
	// a fixed 50% duty with 0x8400 selecting only on/off/flash - is not
	// implemented.  0x8400 = 0xFF is a documented special case: constantly on.
	//
	// There is no "off" setting: D2:0 read as 1, so the smallest programmable
	// value is 0x07, which flashes for seven ticks per cycle.  That is
	// deliberate on SNK's part ("to avoid LED from going off due to a program
	// crash").

	wire led_tick  = ce_6m144 && (led_div == 16'hFFFF);
	// Cycle length in ticks.  LEDFRQ = 0 gives the full 256-tick wrap, which is
	// the natural behaviour of the counter and is not documented either way.
	wire [7:0] led_phase_last = ledfrq_q - 8'd1;

	always @(posedge clk_sys) begin
		if (rst || soft_reset) begin
			led_div   <= 16'd0;
			led_phase <= 8'd0;
			led_q     <= 1'b1;                             // reset value is "on"
		end else if (ss_wr && (ss_addr == 5'd5)) begin
			led_phase <= ss_wdata[39:32];
			led_div   <= ss_wdata[31:16];
			led_q     <= ss_wdata[0];
		end else begin
			if (ce_6m144) begin
				led_div <= led_div + 16'd1;
				if (led_tick)
					led_phase <= (led_phase >= led_phase_last) ? 8'd0
					                                           : (led_phase + 8'd1);
				led_q <= (ledctl == 8'hFF) ? 1'b1 : (led_phase < ledctl);
			end
		end
	end

	assign led = led_q;

	// Pause / drain
	// Report ready at a line boundary, so the whole video block freezes with
	// the render pass and the scanout pipeline in a defined state.  Not
	// ce-gated, because once the parent sees `pause_ready`
	// it gates ce_6m144 and this flop must hold.

	always @(posedge clk_sys) begin
		if (rst)             pause_ready_q <= 1'b0;
		else if (!pause_req) pause_ready_q <= 1'b0;
		else if (line_start) pause_ready_q <= 1'b1;
	end

	assign pause_ready = pause_ready_q;

	// CPU readback
	// A mux of flip-flops, one per byte lane.  Unmapped addresses read 0x00;
	// write-only registers read 0xFF on the CPU bus and appear only on the
	// savestate tap.

	// Both lanes are decoded by one combinational block that walks the two byte
	// addresses of the addressed word.  A two-pass loop rather than a function
	// called twice: a function that reads module state from inside a continuous
	// assignment is not re-evaluated when that state changes in every
	// simulator, and the mono LUT is read out of `lut_flat` rather than the
	// array for the same reason.

	reg  [7:0]  rd_lane [0:1];
	reg  [10:0] rd_ba;
	reg  [2:0]  rd_g;
	reg  [1:0]  rd_c;
	reg  [4:0]  rd_ix;
	integer     ln;

	always @* begin
		for (ln = 0; ln < 2; ln = ln + 1) begin
			rd_ba = {wa, ln[0]};
			rd_g  = rd_ba[4:2];
			rd_c  = rd_ba[1:0];
			rd_ix = ({2'b00, rd_g} + {1'b0, rd_g, 1'b0}) + {3'b000, rd_c} - 5'd1;

			if ((rd_ba[10:5] == 6'h08) && (rd_g <= 3'd5) && (rd_c != 2'b00)) begin
				// Mono LUT.  "D7:3 read 0" is our reading of the D.C columns;
				// the documents mark them Do-Not-Care rather than give a value.
				rd_lane[ln] = {5'd0, lut_flat[53 - rd_ix*3 -: 3]};
			end else if ((rd_ba[10:4] == 7'h70) && (rd_ba[3:1] <= 3'd6)
			             && !rd_ba[0]) begin
				rd_lane[ln] = 8'hFF;                       // 0x8700-0x870C
			end else begin
				case (rd_ba)
					11'h000: rd_lane[ln] = {vi_e_q, hi_e_q, int_dc_q};
					11'h002: rd_lane[ln] = wba_h_q;
					11'h003: rd_lane[ln] = wba_v_q;
					11'h004: rd_lane[ln] = wsi_h_q;
					11'h005: rd_lane[ln] = wsi_v_q;
					11'h006: rd_lane[ln] = ref_q;
					11'h008: rd_lane[ln] = ras_h;
					11'h009: rd_lane[ln] = ras_v;
					11'h010: rd_lane[ln] = {covr, blnk, 6'd0};
					11'h012: rd_lane[ln] = ctl2d_q;
					11'h020: rd_lane[ln] = po_h_q;
					11'h021: rd_lane[ln] = po_v_q;
					11'h030: rd_lane[ln] = pf_q;
					11'h032: rd_lane[ln] = s1so_h_q;
					11'h033: rd_lane[ln] = s1so_v_q;
					11'h034: rd_lane[ln] = s2so_h_q;
					11'h035: rd_lane[ln] = s2so_v_q;
					11'h118: rd_lane[ln] = bgc_q;
					11'h400: rd_lane[ln] = ledctl;
					11'h402: rd_lane[ln] = ledfrq_q;
					11'h7E0: rd_lane[ln] = 8'hFF;          // write-only strobe
					11'h7E2: rd_lane[ln] = mode_reg_exists ? {mode_q, 7'd0}
					                                       : 8'h00;
					11'h7F0: rd_lane[ln] = 8'hFF;          // write-only shadow
					11'h7F2: rd_lane[ln] = 8'hFF;
					11'h7F4: rd_lane[ln] = 8'hFF;
					11'h7FE: rd_lane[ln] = {1'b0, inp0, 6'b111111};
					default: rd_lane[ln] = 8'h00;
				endcase
			end
		end
	end

	assign cpu_dout = cpu_hit ? {rd_lane[1], rd_lane[0]} : 16'h0000;

	// Outputs

	assign vi_e        = vi_e_q;
	assign hi_e        = hi_e_q;
	assign ref_last    = f_ref;

	// "Vint occurs when the line WBA.V + WSI.V is drawn" (K1GE 3-4-11).
	// WSI is a size, so this is the first line past the window.
	//
	// Nine bits, not eight: an out-of-range WBA.V + WSI.V must stay out of
	// range so a consumer can see it, rather than aliasing onto a valid line.
	// Every sum below 256 is unchanged.
	assign vint_line   = {1'b0, f_wba_v} + {1'b0, f_wsi_v};

	// Per-line window fields go through the second stage; per-frame ones do
	// not (see the latch-group note above).
	assign win_wba_h   = f_wba_h;
	assign win_wsi_h   = f_wsi_h;
	assign win_wba_v   = f_wba_v;
	assign win_wsi_v   = f_wsi_v;

	assign ln_po_h     = l_po_h;
	assign ln_po_v     = l_po_v;
	assign ln_s1so_h   = l_s1so_h;
	assign ln_s1so_v   = l_s1so_v;
	assign ln_s2so_h   = l_s2so_h;
	assign ln_s2so_v   = l_s2so_v;
	assign ln_p_f      = l_p_f;
	assign sc_p_f      = s_p_f;
	assign sc_mono_lut = P_MONO_LUT_LINE_LATCH ? s_lut : lut_flat;
	assign ln_neg      = s_neg;
	assign ln_oowc     = s_oowc;
	assign ln_bgon     = s_bgon;
	assign ln_bgc      = s_bgc;

	assign mode_compat = mode_q;
	assign mono_lut    = lut_flat;

	// Deliberately unread inputs
	// Reads have no side effects here, so cpu_rd is not needed; only some bits
	// of each savestate word are ours.
	wire unused_ok = &{1'b0, cpu_rd, cpu_a[13:11], cpu_a[0], ss_wdata, 1'b0};

endmodule
