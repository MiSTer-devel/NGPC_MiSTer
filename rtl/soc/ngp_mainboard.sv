// Copyright (c) 2026 Jamie Blanks

// ngp_mainboard -- the NPK2 printed circuit board.
//
// The real board carries exactly two digital parts: the SNK K2-CHIP in a
// 144-pin QFP and an analog LCD bias supply. So this file is wiring and nothing
// else. Every piece of logic lives inside k2_soc, or inside ngp_cart on the
// other side of the connector; anything here that starts to look like a state
// machine belongs in the chip.
//
// What the board is allowed to contain, and does:
//   - the button inversions (the panel switches close against a common SWCOM
//     net, so a pressed button pulls its pad LOW);
//   - tie-offs for nets that carry no signal on this board (INP0, the link
//     connector with nothing plugged in, the battery divider, the sub-battery
//     sense);
//   - the clock-enable tree, the one deliberate boundary deviation: on silicon
//     the clock-gear divider is inside the chip, but ngp_clocks lives here and
//     `gear` crosses back out of the SoC to reach it;
//   - the savestate address split between the chip and the cartridge.
//
// Savestate address split. The internals slice is one 10-bit / 64-bit space:
// k2_soc answers words 0-95 (see its header for the map), ngp_cart answers
// 96-103 through its own native 64-bit tap with SS_BASE = 96, and ngp_clocks
// answers word 104, the divider phase -- the one piece of machine state that
// lives on this board rather than in a part. The read is a mux on the range,
// never a wired-OR.
//
// The pause chain is ordered: the SoC drains first (CPU, then the K2GE at a
// native frame boundary), and only then is the cartridge asked, because the CPU
// is what starts cart transactions. The freeze itself is inside the chip, at
// the point of use, and not on these wires: ngp_power's standby gates are
// applied at that same point, so a board-level gate would split one enable tree
// across two files, and the cartridge deliberately keeps its `ce` tied high and
// would need an exception anyway. Separately, ngp_clocks holds the machine
// enables while the machine is parked and reloads its dividers from the parked
// phase on release, so a freeze does not shift the enable interleaving.
//
// `lcd`, the board's public raster, is the one part of the machine that must
// never freeze. It runs on clk_sys directly and takes neither reset nor a
// machine clock enable, so the framework keeps seeing CE_PIXEL and syncs
// whatever the machine is doing.

module ngp_mainboard
#(
	// The board strap default when nothing drives mono_strap.
	parameter        P_MONO_DEFAULT = 1'b0,
	parameter [7:0]  P_OPEN_BUS     = 8'hFF,
	// Mixer trims, handed straight to k2_soc; 4 = nominal, each step 6 dB.
	parameter [2:0]  P_MIX_PSG_GAIN = 3'd4,
	parameter [2:0]  P_MIX_DAC_GAIN = 3'd4
)
(
	input  wire        clk_sys,
	input  wire        reset,
	input  wire        restore_reset,
	input  wire        mono_strap,       // OSD System select, sampled at reset
	input  wire        lcd_persistence,  // presentation-only panel response
	input  wire        palette_frame_boundary, // display lookup alternative

	// ---- video to the framework -------------------------------------------
	output wire        ce_pix,
	output wire [7:0]  vga_r,
	output wire [7:0]  vga_g,
	output wire [7:0]  vga_b,
	output wire        hsync,
	output wire        vsync,
	output wire        hblank,
	output wire        vblank,

	// ---- audio to the framework -------------------------------------------
	// Two's-complement 16-bit PCM; `emu` sets AUDIO_S = 1. Declared unsigned
	// here only because that is what NGPC.sv's own nets are: the bits are the
	// signed value k2_soc produced and nothing on this path does arithmetic.
	output wire [15:0] audio_l,
	output wire [15:0] audio_r,

	// ---- panel inputs from the emu-level mapper ---------------------------
	input  wire [6:0]  btn,              // active HIGH here; the board inverts
	input  wire        power_btn,        // active HIGH, held >= 8 frames

	// ---- CON2 link connector ----------------------------------------------
	// The board carries all four SC0 signals without inventing a partner. The
	// emu boundary supplies the unplugged RXD0/CTS0_n pull levels; keeping TXD0
	// and RTS_n visible here makes a later cable transport a wiring change.
	output wire        link_txd,
	input  wire        link_rxd,
	output wire        link_rts_n,
	input  wire        link_cts_n,

	output wire        led_user,
	output wire        bios_setup_ready,
	output wire        bios_mono_active,

	// ---- BIOS image loader -------------------------------------------------
	input  wire        bios_wr,
	input  wire        bios_sel,         // 0 = colour image, 1 = mono image
	input  wire [14:0] bios_addr,        // word address
	input  wire [15:0] bios_data,
	// OSD `Skip BIOS Animation`. Consumed by ngp_bios_anim_patch below, which
	// is part of the loader path, not part of the chip.
	input  wire        skip_bios_animation,

	// ---- MiSTer cheat record stream ----------------------------------------
	input  wire         cheat_load_begin,
	input  wire         cheat_invalidate,
	input  wire         cheat_commit_req,
	output wire         cheat_commit_done,
	input  wire [128:0] cheat_code,

	// ---- cartridge: population and backing store --------------------------
	input  wire [24:0] cart_image_bytes,
	input  wire        cart_config_load,
	input  wire        cart_force_8m_die0,
	input  wire        cart_force_flash_read,
	output wire [1:0]  cart_size_code0,
	output wire [1:0]  cart_size_code1,
	output wire [24:0] cart_bytes,
	output wire        cart_present,
	output wire        cart_mem_req,
	output wire        cart_mem_we,
	output wire [24:0] cart_mem_addr,
	output wire [15:0] cart_mem_wdata,
	output wire [1:0]  cart_mem_be,
	output wire        cart_mem_lane,
	output wire        cart_mem_tag,
	output wire        cart_mem_flash,
	input  wire [15:0] cart_mem_rdata,
	input  wire        cart_mem_rvalid,
	input  wire        cart_mem_done,
	output wire        cart_dirty_pulse,
	output wire [34:0] cart_dirty0,
	output wire [34:0] cart_dirty1,
	output wire        cart_dirty0_event,
	output wire [5:0]  cart_dirty0_block,
	output wire        cart_dirty1_event,
	output wire [5:0]  cart_dirty1_block,
	input  wire        cart_dirty_clear,
	output wire        cart_flash_busy,
	output wire [1:0]  cart_die_busy,

	// ---- savestate bus and pause ------------------------------------------
	input  wire [9:0]  ss_bus_adr,
	input  wire [63:0] ss_bus_din,
	input  wire        ss_bus_wren,
	input  wire        ss_bus_rst,
	input  wire        ss_restore_is_rewind,
	output wire [63:0] ss_bus_dout,
	input  wire [1:0]  ss_mem_type,
	input  wire        ss_mem_active,
	input  wire [13:0] ss_mem_addr,
	input  wire [7:0]  ss_mem_wdata,
	input  wire        ss_mem_wren,
	input  wire        ss_mem_rden,
	output wire [7:0]  ss_mem_rdata,
	// High for the whole internals broadcast of a restore. Only ngp_snd reads
	// it, to hold the Z80 out of reset-order trouble while its registers are
	// being rewritten (see rtl/snd/ngp_snd.sv).
	input  wire        loading_savestate,
	input  wire        pause_req,
	output wire        pause_ready
);

	// Nets between the two parts

	wire [2:0]  gear;
	wire        ce_6m144, ce_3m072, ce_cpu, ce_t900, ce_32k768;

	wire [20:0] cart_a;
	wire [7:0]  cart_d_o;
	wire        cart_d_oe;
	wire        cart_nce0, cart_nce1, cart_noe, cart_nwe;
	wire [7:0]  cart_d_i;
	wire        cart_edge_oe;
	wire        cart_rd_ready;

	wire [3:0]  lcd_r, lcd_g, lcd_b;
	wire        lcd_dclk_ce, lcd_de, lcd_hs, lcd_vs, lcd_lp, lcd_sp;
	// The presentation raster's own DE. The framework computes its DE from the
	// two blanks (sys/video_mixer.sv), so this one is retained as the explicit
	// observation point for DE == ~(HBlank | VBlank).
	wire        lcd_vga_de;

	wire [7:0]  dac_l, dac_r;

	wire        mpoff_n;
	wire        led;

	wire [63:0] soc_ss_dout;
	wire [63:0] cart_ss_dout;
	wire [63:0] clocks_ss_dout;
	wire        soc_pause_ready;
	wire        cart_pause_ready;
	wire        cart_pause_req;
	wire        video_pause_req;
	wire        video_reset_hold;
	wire        machine_pause_req;
	wire        machine_pause_ready;
	wire        machine_reset;
	reg         restore_release_q;

	assign machine_reset = reset | restore_reset | video_reset_hold;

	// clocks : ngp_clocks -- the one deliberate boundary deviation
	// On silicon the clock-gear divider is inside the chip; here it is on the
	// board, so `gear` crosses back out of the SoC. It is also the only thing on
	// this board carrying savestate state (internals word 104, the divider
	// phase), and the pause tree is wired IN rather than out: ngp_clocks has
	// nothing to drain, so it is not a link in the chain, it only watches it so
	// the phase survives a freeze. The phase stops at the SoC-ready edge;
	// board-final ready stays separate so cartridge drain is still required
	// before a restore can write the word.

	ngp_clocks clocks
	(
		.clk_sys       (clk_sys),
		.reset         (machine_reset),
		.gear          (gear),

		.pause_req     (machine_pause_req),
		.phase_capture (soc_pause_ready),
		.pause_ready   (machine_pause_ready),
		.restore_hold  (loading_savestate),

		.ce_6m144      (ce_6m144),
		.ce_3m072      (ce_3m072),
		.ce_cpu        (ce_cpu),
		.ce_t900       (ce_t900),
		.ce_32k768     (ce_32k768),

		.ss_bus_adr    (ss_bus_adr),
		.ss_bus_din    (ss_bus_din),
		.ss_bus_wren   (ss_bus_wren),
		.ss_bus_dout   (clocks_ss_dout)
	);

	// bios_anim_patch : ngp_bios_anim_patch -- the `Skip BIOS Animation` option
	// Loader-path wiring, not chip logic. A real NPK2 board has a mask ROM and
	// no download port, so `bios_wr/sel/addr/data` are already a MiSTer bridge
	// rather than a board net; the option substitutes exactly one word of the
	// image as it passes through. The loader always wins the port; the patch
	// takes an idle cycle.

	wire        patch_wr;
	wire        patch_sel;
	wire [14:0] patch_addr;
	wire [15:0] patch_data;

	ngp_bios_anim_patch bios_anim_patch
	(
		.clk            (clk_sys),
		.reset          (machine_reset),
		.skip_animation (skip_bios_animation),
		.bios_wr        (bios_wr),
		.bios_sel       (bios_sel),
		.bios_addr      (bios_addr),
		.bios_data      (bios_data),
		.patch_wr       (patch_wr),
		.patch_sel      (patch_sel),
		.patch_addr     (patch_addr),
		.patch_data     (patch_data)
	);

	// Steered by patch_wr, which is asserted only on cycles the loader itself
	// is idle, so the substitution can never displace a loader word.
	wire        soc_bios_wr   = bios_wr | patch_wr;
	wire        soc_bios_sel  = patch_wr ? patch_sel  : bios_sel;
	wire [14:0] soc_bios_addr = patch_wr ? patch_addr : bios_addr;
	wire [15:0] soc_bios_data = patch_wr ? patch_data : bios_data;

	// soc : k2_soc -- the 144-QFP part
	// Board-side tie-offs:
	//   an0        the main-battery divider, tied to a healthy 10'h3FF. An OSD
	//              low-battery option can lower it later.
	//   inp0       tied to 1. The pad exists (K1GE p.24) but no net on this
	//              board reaches it.
	//   link_*     pass through to the emu boundary, which supplies the
	//              unplugged input levels.
	//   btn_n      SW1-SW4 plus the A/B/OPTION dome contacts, all closing to
	//              SWCOM, so the pad is LOW when pressed and the board inverts
	//              the active-high mapper output.
	//   pwr_btn_n  the POWER switch through the Q5C/SD8C network, same polarity.

	k2_soc
	#(
		.P_MONO         (P_MONO_DEFAULT),
		.P_OPEN_BUS     (P_OPEN_BUS),
		.P_MIX_PSG_GAIN (P_MIX_PSG_GAIN),
		.P_MIX_DAC_GAIN (P_MIX_DAC_GAIN)
	)
	soc
	(
		.clk_sys       (clk_sys),
		.reset         (machine_reset),
		.ce_x1         (ce_6m144),
		.ce_x1_div2    (ce_3m072),
		.ce_cpu        (ce_cpu),
		.ce_t900       (ce_t900),
		.ce_xt1        (ce_32k768),
		.gear          (gear),

		.mono_strap    (mono_strap),
		.palette_frame_boundary (palette_frame_boundary),
		.bios_wr       (soc_bios_wr),
		.bios_sel      (soc_bios_sel),
		.bios_addr     (soc_bios_addr),
		.bios_data     (soc_bios_data),
		.cheat_load_begin  (cheat_load_begin),
		.cheat_invalidate  (cheat_invalidate),
		.cheat_commit_req  (cheat_commit_req),
		.cheat_commit_done (cheat_commit_done),
		.cheat_code        (cheat_code),

		.cart_a        (cart_a),
		.cart_d_i      (cart_d_i),
		.cart_rd_ready (cart_rd_ready),
		.cart_d_o      (cart_d_o),
		.cart_d_oe     (cart_d_oe),
		.cart_nce0     (cart_nce0),
		.cart_nce1     (cart_nce1),
		.cart_noe      (cart_noe),
		.cart_nwe      (cart_nwe),

		.lcd_r         (lcd_r),
		.lcd_g         (lcd_g),
		.lcd_b         (lcd_b),
		.lcd_dclk_ce   (lcd_dclk_ce),
		.lcd_de        (lcd_de),
		.lcd_hs        (lcd_hs),
		.lcd_vs        (lcd_vs),
		.lcd_lp        (lcd_lp),
		.lcd_sp        (lcd_sp),

		.dac_l         (dac_l),
		.dac_r         (dac_r),
		// Already mixed into signed 16-bit PCM inside the chip, so the
		//
		// amplifier this board would otherwise model is one wire. The framework
		// takes the two's-complement bits verbatim; NGPC.sv sets AUDIO_S = 1.
		.audio_l       (audio_l),
		.audio_r       (audio_r),

		.btn_n         (~btn),
		.pwr_btn_n     (~power_btn),
		.mpoff_n       (mpoff_n),
		.bios_setup_ready (bios_setup_ready),
		.bios_mono_active (bios_mono_active),
		.an0           (10'h3FF),
		.led           (led),
		.inp0          (1'b1),

		.link_txd      (link_txd),
		.link_rxd      (link_rxd),
		.link_rts_n    (link_rts_n),
		.link_cts_n    (link_cts_n),

		.ss_bus_adr    (ss_bus_adr),
		.ss_bus_din    (ss_bus_din),
		.ss_bus_wren   (ss_bus_wren),
		.ss_bus_dout   (soc_ss_dout),
		.ss_mem_type   (ss_mem_type),
		.ss_mem_active (ss_mem_active),
		.ss_mem_addr   (ss_mem_addr),
		.ss_mem_wdata  (ss_mem_wdata),
		.ss_mem_wren   (ss_mem_wren),
		.ss_mem_rden   (ss_mem_rden),
		.ss_mem_rdata  (ss_mem_rdata),
		.loading_savestate (loading_savestate),
		.pause_req     (machine_pause_req),
		.pause_ready   (soc_pause_ready)
	);

	// cart : ngp_cart -- the 36-pin cartridge connector and its dies
	// `ce` is tied high on purpose. The cartridge is asynchronous logic on real
	// hardware, and sampling its pins at the full 49.152 MHz clock catches every
	// strobe edge of a 6.144 MHz bus cycle with margin (see rtl/cart/
	// flash_die.sv). Gating it with a bus enable would make the die miss the
	// edges it exists to detect.
	//
	// The reset is the board input, not `machine_reset`: the latter includes the
	// CRT presenter's private hold until public frame wrap, cartridge population
	// is re-latched while that hold is active, and keeping the external flash in
	// reset would discard the configuration strobe and leave every loaded image
	// absent. On the physical board the flash is outside the K2 chip, so a
	// presentation-only extension must not reset it either.

	ngp_cart cart
	(
		.clk                  (clk_sys),
		.ce                   (1'b1),
		.reset                (reset | restore_reset),

		.image_bytes          (cart_image_bytes),
		.config_load          (cart_config_load),
		.force_8m_die0        (cart_force_8m_die0),
		.force_flash_read     (cart_force_flash_read),
		.size_code0           (cart_size_code0),
		.size_code1           (cart_size_code1),
		.cart_bytes           (cart_bytes),
		.cart_present         (cart_present),

		.A                    (cart_a),
		.d_in                 (cart_d_o),
		.nCE0                 (cart_nce0),
		.nCE1                 (cart_nce1),
		.nOE                  (cart_noe),
		.nWE                  (cart_nwe),
		.d_out                (cart_d_i),
		.d_oe                 (cart_edge_oe),
		.rd_ready             (cart_rd_ready),

		.mem_req              (cart_mem_req),
		.mem_we               (cart_mem_we),
		.mem_addr             (cart_mem_addr),
		.mem_wdata            (cart_mem_wdata),
		.mem_be               (cart_mem_be),
		.mem_lane             (cart_mem_lane),
		.mem_tag              (cart_mem_tag),
		.mem_flash            (cart_mem_flash),
		.mem_rdata            (cart_mem_rdata),
		.mem_rvalid           (cart_mem_rvalid),
		.mem_done             (cart_mem_done),

		.dirty_pulse          (cart_dirty_pulse),
		.dirty0               (cart_dirty0),
		.dirty1               (cart_dirty1),
		.dirty0_event         (cart_dirty0_event),
		.dirty0_block         (cart_dirty0_block),
		.dirty1_event         (cart_dirty1_event),
		.dirty1_block         (cart_dirty1_block),
		.dirty_clear          (cart_dirty_clear),
		.flash_busy           (cart_flash_busy),
		.die_busy             (cart_die_busy),

		.ss_bus_adr           (ss_bus_adr),
		.ss_bus_din           (ss_bus_din),
		.ss_bus_wren          (ss_bus_wren),
		.ss_bus_rst           (ss_bus_rst),
		.ss_bus_dout          (cart_ss_dout),
		.ss_restore_is_rewind (ss_restore_is_rewind),
		.pause_req            (cart_pause_req),
		.pause_ready          (cart_pause_ready)
	);

	// Savestate split and the pause chain

	wire ss_sel_cart   = (ss_bus_adr >= 10'd96) && (ss_bus_adr < 10'd104);
	wire ss_sel_clocks = (ss_bus_adr == 10'd104);

	assign ss_bus_dout = ss_sel_cart   ? cart_ss_dout   :
	                     ss_sel_clocks ? clocks_ss_dout : soc_ss_dout;

	// The CPU is what starts cartridge transactions, so the connector is only
	// asked once the chip is quiet. The presentation coordinator can extend an
	// external pause through the next public frame edge, or request the same
	// ordered drain after an exceptional frame-ownership fault. The external
	// savestate client still sees ready only in response to its own request.
	// Startup and core reset are a separate hold-in-reset path so CPU prefetch
	// cannot be parked half-started.
	assign machine_pause_req = pause_req | video_pause_req;
	assign cart_pause_req = machine_pause_req && soc_pause_ready;
	// A restore invalidates the partial display generation and asks the
	// presenter to re-enter at a public frame boundary. The machine is already
	// safely stopped by the restore service hold, so retain that ready result
	// until the presenter's request falls. Dropping it with
	// loading_savestate would deadlock: ngp_clocks is parked, while K2GE would
	// need those same clocks to drain through the normal pause chain.
	always @(posedge clk_sys) begin
		if (reset)
			restore_release_q <= 1'b0;
		else if (loading_savestate)
			restore_release_q <= 1'b1;
		else if (!video_pause_req)
			restore_release_q <= 1'b0;
	end

	assign machine_pause_ready = loading_savestate || restore_release_q ||
		(machine_pause_req && soc_pause_ready && cart_pause_ready);
	assign pause_ready = pause_req && machine_pause_ready;

	// lcd : ngpc_crt_framebuffer -- LCD capture and framework video face
	// The K2GE remains a native 515 x 199 machine. This board-level store takes
	// complete RGB444 generations, expands them losslessly into RGB888
	// presentation banks, and presents a separate 262-line progressive raster
	// whose 819,880-clk frame period exactly equals the native period.
	// Ping-pong ownership changes only at the public frame boundary; exceptional
	// holds align the native and public boundaries, and equal frame totals then
	// keep normal gameplay continuous. The public raster uses clk_sys directly
	// and therefore continues through every machine pause.

	ngpc_crt_framebuffer lcd
	(
		.clk_sys              (clk_sys),
		.rst                  (reset),

		.lcd_dclk_ce          (lcd_dclk_ce),
		.lcd_r                (lcd_r),
		.lcd_g                (lcd_g),
		.lcd_b                (lcd_b),
		.lcd_de               (lcd_de),
		.lcd_sp               (lcd_sp),

		.external_pause_req   (pause_req),
		.machine_pause_ready  (machine_pause_ready),
		.loading_savestate    (loading_savestate),
		.video_pause_req      (video_pause_req),
		.video_reset_hold     (video_reset_hold),

		.tint                 (3'd0),
		.lcd_persistence      (lcd_persistence),

		.ce_pixel             (ce_pix),
		.vga_r                (vga_r),
		.vga_g                (vga_g),
		.vga_b                (vga_b),
		.vga_de               (lcd_vga_de),
		.vga_hbl              (hblank),
		.vga_vbl              (vblank),
		.vga_hs               (hsync),
		.vga_vs               (vsync)
	);

	// On silicon the PSG's analog outputs and the two R-2R DACs are summed by an
	// amplifier on this board. We mix digitally instead, inside `ngp_snd` with
	// the PSG, so the SoC hands the board finished PCM and the amplifier is a
	// wire. The volume pot and the speaker are analog and are not modelled.

	// The red LED D10 is driven by the K2GE's LED block.
	assign led_user = led;

	// Deliberately unread at this level
	//   lcd_vga_de           see its declaration: the framework takes the two
	//                        blanks, not DE.
	//   dac_l, dac_r         mixed inside the chip; observation points here.
	//   mpoff_n              the /MPOFF test pad drives the DC-DC enable
	//                        network, which is not modelled electrically.
	//   cart_edge_oe         whether silicon is actually driving. The connector
	//                        already folds the pull-ups into d_out, so the SoC
	//                        consumes the resolved value unconditionally.
	//   cart_d_oe            the SoC's own drive enable; the connector takes
	//                        d_in unconditionally and gates on nWE instead,
	//                        which is what the die pins do.
	//   lcd_hs/lcd_vs/lcd_lp the native LCD flex timing; the presenter derives
	//                        its own raster from lcd_dclk_ce, lcd_de and lcd_sp.
	wire unused_ok = &{1'b0,
	                   lcd_vga_de,
	                   dac_l, dac_r,
	                   mpoff_n,
	                   lcd_hs, lcd_vs, lcd_lp,
	                   cart_edge_oe, cart_d_oe,
	                   1'b0};

endmodule
