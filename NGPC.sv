//============================================================================
//
//  Neo Geo Pocket / Neo Geo Pocket Color for MiSTer -- core top level.
//
//  Copyright (c) 2026 Jamie Blanks
//
//  This file is wiring. Console behaviour lives in rtl/soc/ngp_mainboard.sv
//  and in the cartridge backing store on the other side of the connector.
//
//  Instance names are load-bearing: the Verilator harness reaches internal
//  signals by hierarchical reference, e.g. emu.mainboard.soc.cpu.trace_pc.
//
//  Status-bit allocation. The table stays complete even for bits with no
//  CONF_STR entry: a bit that moves silently changes the meaning of every
//  user's saved OSD settings.
//
//   Bits       Directive        Meaning
//   [0]        T[0] / R[0]      Reset
//   [2:1]      O[2:1]           System: NGPC / Auto(ext) / NGP -> strap
//   [3]        O[3]             Language English/Japanese (BIOS seed)
//   [6:4]      O[6:4]           video_freak scale
//   [8:7]      O[8:7]           AUDIO_MIX stereo cross-mix
//   [10]       P3O[10]          Autosave on/off
//   [11]       P3R[11]          Save Backup RAM (manual)
//   [12]       P3R[12]          Load Backup RAM (manual)
//   [13]       T[13]            Power button (virtual press)
//   [16:14]    H0P2O[16:14]     Monochrome-game palette (BIOS seed)
//   [17]       P3O[17]          RTC: MiSTer time / BIOS default
//   [18]       P3O[18]          Automatic power-on after setup
//   [19]       P2O[19]          Skip BIOS eye-catch (normal launch)
//   [20]       P1O[20]          LCD response Raw/Panel
//   [22:21]    H0P1O[22:21]     Saturation 75%/50%/25%/100%
//   [23]       H0P3O[23]        Palette updates Immediate/Frame
//   [24]       O[24]            Serial route Internal/SNAC
//   [42]       d7R[42]          Save state
//   [43]       d7R[43]          Load state
//   [44]       d7P3O[44]        Savestate disk-write off
//   [46:45]    d7O[46:45]       Savestate slot 1-4
//   [47]       reserved         Rewind capture on/off (rewind not built)
//   [122:121]  P1O[122:121]     Aspect ratio
//
//   status_menumask[7] is `allow_ss` and gates the d7 savestate block.
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

//////////////////////// Framework ports not used yet ////////////////////////

assign ADC_BUS  = 'Z;
assign UART_DTR = 1'b0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

// DDR3 hosts the four savestate slots, the pristine cartridge shadow and the
// sparse staging aperture.
assign DDRAM_CLK = clk_sys;

assign VGA_SL       = 0;
assign VGA_F1       = 0;
assign VGA_SCALER   = 0;
assign VGA_DISABLE  = 0;
assign HDMI_FREEZE  = 0;
assign HDMI_BLACKOUT= 0;
assign HDMI_BOB_DEINT = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

//////////////////////////////// CONF_STR ////////////////////////////////////

// The `SS32000000:800000` token reserves the DDR savestate region (4 x 8 MB
// slots, SAVESTATE_SHIFT = 21). The adjacent `UART19200` token advertises the
// one rate supported by the physical NGPC link. Both are non-OSD capabilities
// and therefore share CONF_STR item 1, separated by a comma as Main expects.
//
// Entries whose function does not exist yet are deliberately absent; the bit
// allocation for them is in the table at the top of this file.

`include "build_id.v"
localparam CONF_STR = {
	"NGPC;SS32000000:800000,UART19200;",
	"FS1,NGPNGCNPC,Load Cartridge;",
	"-;",
	"C,Cheats;",
	"-;",
	"O[2:1],System,NGPC,Auto,NGP;",
	"-;",
	"d7O[46:45],Savestate Slot,1,2,3,4;",
	"d7R[42],Save State (Alt-F1..F4);",
	"d7R[43],Restore State (F1..F4);",
	"-;",
	"P1,Audio & Video;",
	"P1-;",
	"P1O[20],LCD Response,Raw,Panel;",
	"H0P1O[22:21],Saturation,75%,50%,25%,100%;",
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[6:4],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P1O[8:7],Stereo Mix,None,25%,50%,100%;",
	"P2,BIOS Settings;",
	"P2-;",
	"P2O[3],Language,English,Japanese;",
	"H0P2O[16:14],NGP Palette,B&W,Red,Green,Blue,Classic;",
	"P2O[19],Skip BIOS Animation,On,Off;",
	"P3,Advanced;",
	"P3-;",
	"P3O[17],RTC,MiSTer Time,BIOS Default;",
	"P3O[18],Automatic Power,On,Off;",
	"P3O[24],Serial Route,Internal,SNAC;",
	//"H0P3O[23],Palette Updates,Immediate,Frame Boundary;",
	"P3-;",
	"P3O[10],Autosave,On,Off;",
	"P3R[11],Save Backup;",
	"P3R[12],Load Backup;",
	"P3-;",
	"d7P3O[44],Savestate to SD,On,Off;",
	"-;",
	"R[0],Reset;",
	"J1,A,B,Option,Power,Savestates;",
	"jn,A,B,Start,Select,L;",
	// OSD info banner lines, indexed 1-based by savestate_ui's ss_info.
	// The module emits 5 (help), 6+slot (slot selected) and 10+{slot,load}
	// (saved / restored), so entries 1-4 are never requested by this core:
	// they are where a rewind build's toasts would live, and rewind is not
	// built. They are present because the index is a position in this list,
	// not a name.
	"I,",
	"-,",
	"-,",
	"-,",
	"-,",
	"Slot=L/R Load=U Save=D,",
	"Active Slot 1,",
	"Active Slot 2,",
	"Active Slot 3,",
	"Active Slot 4,",
	"Save to state 1,",
	"Restore state 1,",
	"Save to state 2,",
	"Restore state 2,",
	"Save to state 3,",
	"Restore state 3,",
	"Save to state 4,",
	"Restore state 4;",
	"v,11;",
	"V,v",`BUILD_DATE
};

//////////////////////////////// Clocks //////////////////////////////////////

// clk_sys = 49.152 MHz = 8 x 6.144 MHz, the console's X1 crystal times eight.
// clk_ram = 98.304 MHz = 2 x clk_sys from the same PLL, phase aligned, for the
// cartridge SDRAM. NGPC.sdc must never false-path between them, and
// rtl/mem/sdram.sdc expects the SDRAM clock on PLL counter c1.

wire clk_sys;
wire clk_ram;
wire pll_locked;

pll pll
(
	.refclk   (CLK_50M),
	.rst      (1'b0),
	.outclk_0 (clk_sys),
	.outclk_1 (clk_ram),
	.locked   (pll_locked)
);

///////////////////////////// Cartridge nets /////////////////////////////////

wire [24:0] cart_image_bytes;
wire [31:0] cart_image_crc32;
wire [31:0] cart_pristine_crc32;
wire        cart_loader_config;
wire [1:0]  cart_size_code0;
wire [1:0]  cart_size_code1;
wire [24:0] cart_bytes;
wire        cart_present;
wire        cart_mem_req;
wire        cart_mem_we;
wire [24:0] cart_mem_addr;
wire [15:0] cart_mem_wdata;
wire [1:0]  cart_mem_be;
wire        cart_mem_lane;
wire        cart_mem_tag;
wire        cart_mem_flash;
wire [15:0] cart_mem_rdata;
wire        cart_mem_rvalid;
wire        cart_mem_done;
wire        cart_dirty_pulse;
wire [34:0] cart_dirty0;
wire [34:0] cart_dirty1;
wire        cart_dirty0_event;
wire [5:0]  cart_dirty0_block;
wire        cart_dirty1_event;
wire [5:0]  cart_dirty1_block;
wire        cart_flash_busy;
wire [1:0]  cart_die_busy;
wire        cart_header_valid;
wire [15:0] cart_header_catalog;
wire [7:0]  cart_header_subcatalog;
wire [95:0] cart_header_title;

// Original Delta Warp is a 512-KiB dump whose retail code writes 8-Mbit
// blocks 16/17. Match all header identity bytes before overriding geometry.
wire cart_force_8m_die0 = cart_header_valid &&
	(cart_image_bytes == 25'h080000) &&
	(cart_header_catalog == 16'h0103) &&
	(cart_header_subcatalog == 8'h05) &&
	(cart_header_title[7:0]   == 8'h44) && // D
	(cart_header_title[15:8]  == 8'h45) && // E
	(cart_header_title[23:16] == 8'h4C) && // L
	(cart_header_title[31:24] == 8'h54) && // T
	(cart_header_title[39:32] == 8'h41) && // A
	(cart_header_title[47:40] == 8'h20) && // space
	(cart_header_title[55:48] == 8'h57) && // W
	(cart_header_title[63:56] == 8'h41) && // A
	(cart_header_title[71:64] == 8'h52) && // R
	(cart_header_title[79:72] == 8'h50) && // P
	(cart_header_title[87:80] == 8'h20) && // space
	(cart_header_title[95:88] == 8'h00);   // terminator

wire        cart_load_req;
wire [24:0] cart_load_addr;
wire [15:0] cart_load_data;
wire        cart_load_ready;
wire        cart_load_done;
wire        cart_live_load_req;
wire [24:0] cart_live_load_addr;
wire [15:0] cart_live_load_data;
wire        cart_live_load_ready;
wire        cart_live_load_done;
wire        cart_download;
wire        cart_download_start;
wire        cart_ready;
wire [15:0] cart_bg_rdata;
wire        cart_bg_ready;
wire        cart_bg_done;

wire        overlay_pause_req;
wire        overlay_boot_hold;
wire        overlay_force_flash_read;
wire        overlay_pending;
wire        overlay_mounted_writable;
wire        overlay_save_done;
wire        overlay_save_rejected;
wire        overlay_load_done;
wire        overlay_load_rejected;
wire [34:0] overlay_live0;
wire [34:0] overlay_live1;
wire [34:0] overlay_pending0;
wire [34:0] overlay_pending1;
wire [34:0] overlay_file0;
wire [34:0] overlay_file1;

wire        overlay_p2_req;
wire        overlay_p2_we;
wire [24:0] overlay_p2_addr;
wire [15:0] overlay_p2_wdata;
wire [1:0]  overlay_p2_be;
wire        overlay_p2_ready;
wire        overlay_p2_done;

wire        state_p2_req;
wire        state_p2_we;
wire [24:0] state_p2_addr;
wire [15:0] state_p2_wdata;
wire [1:0]  state_p2_be;
wire        state_p2_ready;
wire        state_p2_done;

wire        state_ledger_adopt;
wire [34:0] state_ledger0;
wire [34:0] state_ledger1;
wire        state_force_flash_read;

//////////////////////////////// hps_io //////////////////////////////////////

// WIDE(1): 16-bit ioctl_dout matching both BIOS BRAM ports and the cartridge
//          loader, with ioctl_addr incrementing by 2 per word.
// VDNUM(2): S0 = cartridge .sav, S1 reserved for a system-NVRAM image so the
//          slot arrays and status-bit allocations never have to move. S1 is
//          tied off below.
// BLKSZ(2): 512-byte sectors.

// hps_io's core-extension bus. Nothing in this core listens on it, so the two
// core-driven fields are held at a defined "no extension" value rather than
// left floating: hps_io muxes HPS_BUS[15:0] off EXT_BUS[32].
wire  [35:0] ext_bus;
assign ext_bus[32]   = 1'b0;
assign ext_bus[15:0] = 16'd0;

wire         forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [21:0] gamma_bus;
wire  [31:0] joystick_0;
wire  [64:0] hps_rtc;
wire  [10:0] ps2_key;
wire   [7:0] hps_uart_mode;
wire  [31:0] hps_uart_speed;

wire         ioctl_download;
wire  [15:0] ioctl_index;
wire         ioctl_wr;
wire  [26:0] ioctl_addr;
wire  [15:0] ioctl_dout;
wire         ioctl_wait;
wire         cart_ioctl_wait;
wire         bios_mono_active;
wire         pause_ready;
wire         reset;

// S0 = cartridge .sav, driven below. S1 = the reserved system-NVRAM slot,
// unused; it keeps its position so the allocation never has to move.
wire [31:0] sd_lba[2];
wire  [5:0] sd_blk_cnt[2];
wire  [1:0] sd_rd;
wire  [1:0] sd_wr;
wire [15:0] sd_buff_din[2];
wire  [1:0] sd_ack;
wire [12:0] sd_buff_addr;
wire [15:0] sd_buff_dout;
wire        sd_buff_wr;
wire  [1:0] img_mounted;
wire        img_readonly;
wire [63:0] img_size;

wire        overlay_sd_rd;
wire        overlay_sd_wr;
wire [31:0] overlay_sd_lba;
wire [15:0] overlay_sd_buff_din;

// Declared here because hps_io consumes them and the blocks that drive them sit
// further down the file, next to the cartridge they depend on.
wire [127:0] status_in;
wire         status_set;
wire         ss_info_req;
wire   [7:0] ss_info;
wire         cart_dirty_clear;

assign sd_lba[0]      = overlay_sd_lba;
assign sd_lba[1]      = 32'd0;
// The sparse overlay bridge transfers exactly one 512-byte sector per request.
assign sd_blk_cnt[0]  = 6'd0;
assign sd_blk_cnt[1]  = 6'd0;
assign sd_rd          = {1'b0, overlay_sd_rd};
assign sd_wr          = {1'b0, overlay_sd_wr};
assign sd_buff_din[0] = overlay_sd_buff_din;
assign sd_buff_din[1] = 16'd0;

// Menumask 0 hides the colour-BIOS seed palette on true monochrome hardware.
// The latched SoC mode is used rather than the live System option, so the menu
// describes the machine that is actually running. Bit 7 is the savestate gate.
//
// `!overlay_busy` is not tidiness: sparse S0 and sparse manual Type3 share the
// cartridge SDRAM background mailbox and DDR3 staging aperture, so they must
// never run at once. `!seed_busy` closes the other seam -- the BIOS
// setup seeder owns the same internals bus and the same flat memory tap, and a
// state load landing on top of a half-written setup record would leave a
// checksum that the BIOS then rejects.
wire overlay_busy;
wire seed_busy;
wire savestate_busy;
wire ss_save_req;
wire ss_load_req;
wire persistent_transaction_busy;
wire overlay_operation_enable;
wire allow_ss = cart_present && cart_ready && !cart_download &&
				!cart_dl_raw && !base_reset && !overlay_busy &&
				!savestate_busy && !seed_busy;
wire [15:0] status_menumask = {8'd0, allow_ss, 6'd0, bios_mono_active};

hps_io #(.CONF_STR(CONF_STR), .WIDE(1), .VDNUM(2), .BLKSZ(2)) hps_io
(
	.clk_sys            (clk_sys),
	.HPS_BUS            (HPS_BUS),
	.EXT_BUS            (ext_bus),
	.gamma_bus          (gamma_bus),

	.forced_scandoubler (forced_scandoubler),
	.buttons            (buttons),
	.status             (status),
	.status_menumask    (status_menumask),
	.status_in          (status_in),
	.status_set         (status_set),

	.info_req           (ss_info_req),
	.info               (ss_info),

	.joystick_0         (joystick_0),
	.ps2_key            (ps2_key),
	.RTC                (hps_rtc),
	.uart_mode          (hps_uart_mode),
	.uart_speed         (hps_uart_speed),

	.ioctl_download     (ioctl_download),
	.ioctl_index        (ioctl_index),
	.ioctl_wr           (ioctl_wr),
	.ioctl_addr         (ioctl_addr),
	.ioctl_dout         (ioctl_dout),
	.ioctl_wait         (ioctl_wait),

	.sd_lba             (sd_lba),
	.sd_blk_cnt         (sd_blk_cnt),
	.sd_rd              (sd_rd),
	.sd_wr              (sd_wr),
	.sd_ack             (sd_ack),
	.sd_buff_addr       (sd_buff_addr),
	.sd_buff_dout       (sd_buff_dout),
	.sd_buff_din        (sd_buff_din),
	.sd_buff_wr         (sd_buff_wr),

	.img_mounted        (img_mounted),
	.img_readonly       (img_readonly),
	.img_size           (img_size)
);

///////////////////////////// ioctl decode ///////////////////////////////////

// The BIOS comparisons are full 16-bit compares: boot0.rom autoloads at index
// 0x0000 and boot1.rom at 0x0040, the same low six bits, so decoding
// ioctl_index[5:0] alone would load the mono BIOS over the colour one on a
// machine that has both images installed.
//
// The cart comparison is [5:0] only, because [15:6] carries which of the three
// CONF_STR extensions (NGP / NGC / NPC) matched the file; a full-word compare
// would accept one extension and reject the other two.
//
// Index 255 is the standard cheat blob. It feeds only the loader below and
// stays out of the cartridge loader and every reset term: toggling a cheat
// pauses the machine, swaps the active table and resumes, with no reset.

wire bios0_dl = ioctl_download && (ioctl_index == 16'h0000);        // boot0.rom, colour
wire bios1_dl = ioctl_download && (ioctl_index == 16'h0040);        // boot1.rom, mono
wire cheat_dl = ioctl_download && (ioctl_index == 16'h00FF);         // standard MiSTer cheats
wire cart_dl_raw = ioctl_download && (ioctl_index[5:0] == 6'd1) && !cheat_dl;
// Do not replace the live cart, pristine shadow, or identity beneath an
// accepted sparse S0/manual-state transaction. hps_io remains back-pressured
// until the current transaction reaches its atomic commit boundary.
wire cart_dl = cart_dl_raw && !persistent_transaction_busy;
wire cart_transaction_wait = cart_dl_raw && persistent_transaction_busy;

wire         cheat_wait;
wire         cheat_pause_req;
wire         cheat_load_begin;
wire         cheat_invalidate;
wire         cheat_commit_req;
wire         cheat_commit_done;
wire [128:0] cheat_code;
wire         cheat_safe_stopped;

ngp_cheat_loader cheat_loader
(
	.clk_sys_i        (clk_sys),
	.cheat_download_i (cheat_dl),
	.cart_download_i  (cart_dl),
	.paused_i          (cheat_safe_stopped),
	.commit_done_i     (cheat_commit_done),
	.ioctl_wr_i       (ioctl_wr),
	.ioctl_addr_i     (ioctl_addr[3:0]),
	.ioctl_dout_i     (ioctl_dout),
	.wait_o           (cheat_wait),
	.pause_req_o      (cheat_pause_req),
	.load_begin_o     (cheat_load_begin),
	.invalidate_o     (cheat_invalidate),
	.commit_req_o     (cheat_commit_req),
	.code_o           (cheat_code)
);

// Cartridge setup and atomic cheat reloads share hps_io's general download
// back-pressure pin. A cheat payload is held until the ordered machine pause
// is complete, so staging writes cannot steal a table read port from a live
// CPU access. Neither wait participates in the console reset equation.
assign ioctl_wait = cart_ioctl_wait | cheat_wait | cart_transaction_wait;

// BIOS loader. Both images are 64 KB = 32768 16-bit words, so the word address
// is ioctl_addr[15:1] under a byte-address guard: a file longer than 64 KB must
// not wrap and corrupt the start of the image. No back-pressure is needed here,
// as a BRAM port accepts a word per clock and the HPS is far slower.
//
// With WIDE(1) the HPS delivers {file[n+1], file[n]}, which is already the
// little-endian word the TLCS-900/H reads, so words are written verbatim. A
// byte swap shows up at the reset vector: 4A 20 FF 00 at 0xFFFF00 must read
// back as 0x00FF204A.

wire        bios_wr   = (bios0_dl || bios1_dl) && ioctl_wr && (ioctl_addr < 27'h10000);
wire        bios_sel  = bios1_dl;                                   // 0 = colour, 1 = mono
wire [14:0] bios_addr = ioctl_addr[15:1];
wire [15:0] bios_data = ioctl_dout;

//////////////////////////////// Reset ///////////////////////////////////////

// Reset is extended across a BIOS download. A cartridge download defines a
// deterministic cold session: hold the machine in reset, clear all 12 KiB of
// work RAM through its existing port B, then let the selected BIOS
// cold-initialize that blank bank. Ordinary OSD reset does not start the
// clear walker. Index-255 cheat downloads pause and atomically reload only
// the cheat table; they remain absent from this reset path.
//
// The power button is NOT reset. It is the NMI/standby flow, and on this
// machine "power" and "reset" are emphatically different things.

// The cartridge loader and its SDRAM backing store sit OUTSIDE the machine
// reset. A soft reset is a console reset, not a cartridge removal: wiping the
// loader's byte count would make an inserted cart vanish on every OSD reset.
// It is also the reset for this file's own small helpers, which must not be
// cleared by the machine reset they help produce.
wire hard_reset = RESET | ~pll_locked;

localparam [7:0] BIOS_RESET_TAIL = 8'hFF;   // clk_sys ticks held after the download falls

wire bios_dl = bios0_dl || bios1_dl;

// The tail keeps the machine in reset for a moment after the last BIOS word
// lands, so the first fetch cannot race the loader's final BRAM write.
reg [7:0] bios_reset_cnt;

always @(posedge clk_sys) begin
	if (hard_reset || bios_dl) bios_reset_cnt <= BIOS_RESET_TAIL;
	else if (bios_reset_cnt != 8'd0) bios_reset_cnt <= bios_reset_cnt - 8'd1;
end

wire bios_reset = bios_dl || (bios_reset_cnt != 8'd0);

wire base_reset = RESET | status[0] | buttons[1] | ~pll_locked | bios_reset;

wire        wram_clear_busy;
wire        wram_clear_done;
wire [1:0]  clear_ss_mem_type;
wire        clear_ss_mem_active;
wire [13:0] clear_ss_mem_addr;
wire [7:0]  clear_ss_mem_wdata;
wire        clear_ss_mem_wren;
wire        clear_ss_mem_rden;

ngp_wram_clear wram_clear
(
	.clk           (clk_sys),
	.reset         (hard_reset),
	.start         (cart_download_start),
	.busy          (wram_clear_busy),
	.done          (wram_clear_done),
	.ss_mem_type   (clear_ss_mem_type),
	.ss_mem_active (clear_ss_mem_active),
	.ss_mem_addr   (clear_ss_mem_addr),
	.ss_mem_wdata  (clear_ss_mem_wdata),
	.ss_mem_wren   (clear_ss_mem_wren),
	.ss_mem_rden   (clear_ss_mem_rden)
);

// cart_dl asserts before the clear start pulse, so the CPU cannot get one
// running edge ahead of the walker. The final byte is written on an edge where
// wram_clear_busy is still high; reset releases only for the following edge.
//
// `ss_core_reset` is a one-cycle destructive restore-init pulse. It is passed
// separately to the mainboard so it cannot restart/stretch the CRT reset
// coordinator. `loading_savestate` then provides a non-destructive service
// hold: all machine enables remain off while sparse flash, internals and RAM
// are restored serially. Holding ordinary synchronous reset for that interval
// would erase every word again on the next clock.
wire ss_core_reset;

// Cartridge replacement remains a cold, reset-held transaction through the
// mounted-overlay decision.  Releasing reset and substituting a normal pause
// here parks the CPU before its first BIOS instruction while the K2GE drains;
// the reset-default watchdog can then expire before the BIOS disables it.
assign reset = base_reset | cart_dl | wram_clear_busy | overlay_boot_hold;

// Main sends its standard index-255 initialization while a freshly loaded core
// can still be held in synchronous reset. Reset already makes every machine
// client quiescent, but it also clears the ordered pause-ready flops; waiting
// for those flops in that interval would hold HPS ioctl_wait forever. Use reset
// only as a safe cheat staging/commit boundary. Cheat state remains absent from
// every reset equation, and live reloads still take the full ordered pause.
assign cheat_safe_stopped = reset | pause_ready;

//////////////////////////////// Inputs //////////////////////////////////////

// joystick_0[3:0] is {up, down, left, right} in bits 3,2,1,0 -- the framework's
// SNES-virtual layout is RIGHT=0, LEFT=1, DOWN=2, UP=3 -- with bits 4 and up
// assigned in J1 order. 0xB0's bit order is Up,Down,Left,Right,A,B,Option,
// per the SYSTEM WORK REFERENCE MANUAL p.4; the 1 = pressed sense is applied
// inside ngp_sysreg.
//
//   joystick_0 bit  function            destination
//   0               Right               btn[3]
//   1               Left                btn[2]
//   2               Down                btn[1]
//   3               Up                  btn[0]
//   4               A                   btn[4]
//   5               B                   btn[5]
//   6               Option              btn[6]
//   7               Power               the press stretcher below
//   8               Savestates modifier suppresses the d-pad (chord)
//   9               Rewind              reserved, rewind not built
//
// `btn` is ACTIVE HIGH here. The board inverts it to the pads' active-low
// SWCOM wiring, so do not invert twice.

wire joy_ss_chord = joystick_0[8];              // rtl/Savestates/savestate_ui.sv joySS

// The d-pad is suppressed while the savestate chord is held, because
// joySS + Up/Down/Left/Right is load / save / slot-select. Without this the
// game moves while the user picks a slot.
wire dpad_en = ~joy_ss_chord;

wire [6:0] btn;
assign btn[0] = joystick_0[3] & dpad_en;        // Up
assign btn[1] = joystick_0[2] & dpad_en;        // Down
assign btn[2] = joystick_0[1] & dpad_en;        // Left
assign btn[3] = joystick_0[0] & dpad_en;        // Right
assign btn[4] = joystick_0[4];                  // A
assign btn[5] = joystick_0[5];                  // B
assign btn[6] = joystick_0[6];                  // Option

// The power press is held, never pulsed: the BIOS hold gate at 0xFF1A45 runs
// 40 iterations and a single read of "released" aborts the boot back to
// power-off, so an unstretched tap looks like a machine ignoring its power
// button. The gate is ~59,000 states at clock gear 4, which is ~154 ms if a
// state is one 6.144 MHz enable and ~307 ms if it is two oscillator periods;
// 24 frames clears both readings and overshooting costs nothing.
//
// The OSD entry T[13] pulses the same stretcher, so the OSD and the pad
// button are one path with one hold rule.

localparam [24:0] PWR_HOLD_CLKS = 25'd19_677_144; // 24 frames at 59.9503 Hz, 49.152 MHz clock

reg  [24:0] pwr_hold_q;
reg         osd_pwr_q;
reg         auto_pwr_pending_q;

wire pwr_pad  = joystick_0[7];
wire osd_pwr  = status[13] ^ osd_pwr_q;           // T[] toggles on selection
wire seed_done;
wire bios_setup_ready;
// No cartridge is a valid launch target: it is how the stock BIOS reaches its
// built-in utilities. A present cartridge must finish loading first, and any
// active download blocks both cases so the press cannot race an HPS transfer.
wire launch_target_ready = !cart_download && (!cart_present || cart_ready);
wire auto_pwr = auto_pwr_pending_q && bios_setup_ready && launch_target_ready;

always @(posedge clk_sys) begin
	if (hard_reset || cart_download_start) begin
		// Capture, do not clear: clearing would manufacture a toggle edge -- a
		// phantom power press -- if the bit happened to be set at release.
		osd_pwr_q  <= status[13];
		pwr_hold_q <= 25'd0;
		auto_pwr_pending_q <= 1'b0;
	end else begin
		osd_pwr_q <= status[13];

		if (pwr_pad || osd_pwr || auto_pwr) pwr_hold_q <= PWR_HOLD_CLKS;
		else if (pwr_hold_q != 25'd0) pwr_hold_q <= pwr_hold_q - 25'd1;

		// The HPS may still be streaming the cartridge when the BIOS reaches
		// its first standby. Remember the completed setup and wake only once
		// both sides are ready. A manual press cancels the pending automatic
		// press so a later cartridge load cannot turn an already-running
		// machine back off.
		if (status[18] || pwr_pad || osd_pwr || auto_pwr)
			auto_pwr_pending_q <= 1'b0;
		else if (seed_done)
			auto_pwr_pending_q <= 1'b1;
	end
end

wire power_btn = pwr_pad || (pwr_hold_q != 25'd0);

// System select. O[2:1]: 0 = NGPC (colour), 1 = Auto, 2 = NGP (mono).
// For FS1, ioctl_index[15:6] is the zero-based position of the matching
// extension in `NGP,NGC,NPC`: 0 selects NGP, while 1 and 2 select NGPC. Keep
// that choice in one flop because ioctl_index belongs to the active framework
// transfer, not permanently to the inserted cartridge. A hard reset with no
// cartridge defaults Auto to NGPC. The explicit menu choices remain
// authoritative. k2_soc latches mono_strap while reset is asserted, so the
// new extension choice is in place throughout the cartridge-load reset.
reg cart_ext_mono_q;

always @(posedge clk_sys) begin
	if (hard_reset) cart_ext_mono_q <= 1'b0;
	else if (cart_download_start)
		cart_ext_mono_q <= (ioctl_index[15:6] == 10'd0);
end

wire mono_strap = (status[2:1] == 2'd2) ||
	((status[2:1] == 2'd1) && cart_ext_mono_q);

// A machine reset clears ngp_cart's die population (rtl/cart/ngp_cart.sv, the
// config_load register block), but the loader and the SDRAM image survive it
// because they sit on `hard_reset`. Re-strap the board from the retained byte
// count once the machine reset releases, so a cartridge does not disappear
// when the user hits Reset. This is wiring, not a hardware claim.
reg [3:0] cart_reconfig_q;

always @(posedge clk_sys) begin
	if (reset || ss_core_reset) cart_reconfig_q <= 4'd8;
	else if (cart_reconfig_q != 4'd0) cart_reconfig_q <= cart_reconfig_q - 4'd1;
end

wire cart_config_load = cart_loader_config || (!reset && (cart_reconfig_q != 4'd0));

// Persistence identity cannot come from ngp_cart's resettable flash-command
// registers. S0 boot application intentionally holds that block in reset, and
// a manual-state load does the same until its sparse flash overlay is complete.
// Derive the physical geometry from the retained loader/header identity so it
// remains valid for the entire reset-held transaction.
wire [1:0] persist_die0_code = (cart_image_bytes == 25'd0) ? 2'd0 :
	((cart_image_bytes <= 25'h080000) && !cart_force_8m_die0) ? 2'd1 :
	(cart_image_bytes <= 25'h100000) ? 2'd2 : 2'd3;
wire [1:0] persist_die1_code = (cart_image_bytes > 25'h200000) ? 2'd3 : 2'd0;
wire [24:0] persist_cart_bytes = (cart_image_bytes == 25'd0) ? 25'd0 :
	((cart_image_bytes <= 25'h080000) && !cart_force_8m_die0) ? 25'h080000 :
	(cart_image_bytes <= 25'h100000) ? 25'h100000 :
	(cart_image_bytes <= 25'h200000) ? 25'h200000 : 25'h400000;
wire persist_geometry_ready = cart_ready && (persist_die0_code != 2'd0);

///////////////////////// The machine: mainboard /////////////////////////////

wire        ce_pix;
wire  [7:0] vga_r, vga_g, vga_b;
wire  [7:0] profile_r, profile_g, profile_b;
wire        hsync, vsync, hblank, vblank;
wire [15:0] audio_l, audio_r;
wire        led_user;

// CON2 can be routed either through the MiSTer HPS UART (Internal) or directly
// through the USER port (SNAC). Main enables the HPS connection and sets its
// peripheral to the one link rate advertised in CONF_STR. A disabled or
// mismatched internal host is electrically indistinguishable from no cable:
// TMP95C061 Port 8 pulls P81/RXD0 and P82/CTS0 high (datasheet pp.41-42),
// leaving RXD at mark/idle and active-low CTS deasserted.
localparam [0:0] LINK_RXD_UNPLUGGED   = 1'b1;
localparam [0:0] LINK_CTS_N_UNPLUGGED = 1'b1;
localparam [31:0] LINK_UART_BAUD       = 32'd19200;

wire link_txd;
wire link_rts_n;
wire serial_route_snac = status[24];
wire link_host_enable = !serial_route_snac && (hps_uart_mode != 8'd0) &&
	                    (hps_uart_speed == LINK_UART_BAUD);

// The HPS and USER-port inputs are asynchronous to clk_sys. The serial receiver
// oversamples at 16x, so two synchronizer stages add no meaningful bit-time
// error and keep metastability out of the SC0 state.
(* ASYNC_REG = "TRUE" *) reg [1:0] hps_link_rxd_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] hps_link_cts_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] snac_link_rxd_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] snac_link_cts_sync;

always @(posedge clk_sys) begin
	if (hard_reset) begin
		hps_link_rxd_sync <= 2'b11;
		hps_link_cts_sync <= 2'b11;
		snac_link_rxd_sync <= 2'b11;
		snac_link_cts_sync <= 2'b11;
	end else begin
		hps_link_rxd_sync <= {hps_link_rxd_sync[0], UART_RXD};
		hps_link_cts_sync <= {hps_link_cts_sync[0], UART_CTS};
		snac_link_rxd_sync <= {snac_link_rxd_sync[0], USER_IN[1]};
		snac_link_cts_sync <= {snac_link_cts_sync[0], USER_IN[4]};
	end
end

wire link_rxd = serial_route_snac ? snac_link_rxd_sync[1] :
	                link_host_enable ? hps_link_rxd_sync[1] : LINK_RXD_UNPLUGGED;
wire link_cts_n = serial_route_snac ? snac_link_cts_sync[1] :
	                  link_host_enable ? hps_link_cts_sync[1] : LINK_CTS_N_UNPLUGGED;
wire link_present = serial_route_snac || link_host_enable;

// sys_top crosses the endpoint-oriented names: core UART_TXD feeds HPS RXD,
// and core UART_RTS feeds HPS CTS. Both handshake signals use their native
// active-low wire levels, so no polarity inversion belongs here.
assign UART_TXD = link_host_enable ? link_txd   : 1'b1;
assign UART_RTS = link_host_enable ? link_rts_n : 1'b1;

// USER_IO[1] and [4] are inputs and therefore remain released. All unused pins
// are released as well, including while the Internal route is selected.
assign USER_OUT[0] = 1'b1;
assign USER_OUT[1] = 1'b1;
assign USER_OUT[2] = serial_route_snac ? link_txd   : 1'b1;
assign USER_OUT[3] = 1'b1;
assign USER_OUT[4] = 1'b1;
assign USER_OUT[5] = 1'b1;
assign USER_OUT[6] = serial_route_snac ? link_rts_n : 1'b1;

wire [63:0] ss_bus_dout;
wire  [7:0] ss_mem_rdata;
wire  [9:0] seed_ss_bus_adr;
wire [63:0] seed_ss_bus_din;
wire        seed_ss_bus_wren;
wire  [1:0] seed_ss_mem_type;
wire        seed_ss_mem_active;
wire [13:0] seed_ss_mem_addr;
wire  [7:0] seed_ss_mem_wdata;
wire        seed_ss_mem_wren;
wire        seed_ss_mem_rden;
wire        seed_pause_req;

// The savestate engine's half of the same two buses, plus its pause request.
wire  [9:0] eng_ss_bus_adr;
wire [63:0] eng_ss_bus_din;
wire        eng_ss_bus_wren;
wire        eng_ss_bus_rst;
wire  [1:0] eng_ss_mem_type;
wire        eng_ss_mem_active;
wire [13:0] eng_ss_mem_addr;
wire  [7:0] eng_ss_mem_wdata;
wire        eng_ss_mem_wren;
wire        eng_ss_mem_rden;
wire        eng_pause_req;
wire        loading_savestate;
wire        ss_restore_is_rewind;

ngp_persistence_admission persistence_admission
(
	.clk               (clk_sys),
	.ce                (1'b1),
	.reset             (hard_reset),
	.cart_download_i   (cart_dl_raw),
	.overlay_busy_i    (overlay_busy),
	.savestate_busy_i  (savestate_busy),
	.seed_busy_i       (seed_busy),
	.ss_save_i         (ss_save_req),
	.ss_load_i         (ss_load_req),
	.state_reserved_o  (),
	.persistent_busy_o (persistent_transaction_busy),
	.overlay_enable_o  (overlay_operation_enable)
);

// Three owners share the internals bus and the flat memory tap. They are
// separated in time rather than by an arbiter, so a static priority is enough:
//
//   1. the work-RAM clear walker, which only runs while `reset` is held for a
//      cartridge download and nothing else can be running at all;
//   2. the savestate engine, because it has already frozen the machine and a
//      restore that lost a word would be silently wrong;
//   3. the BIOS setup seeder, the only one that can be preempted harmlessly --
//      it retries at the next standby.
//
// `allow_ss` additionally refuses a savestate request while the seeder is busy,
// so rank 2 over rank 3 is a belt-and-braces ordering rather than the thing
// that keeps them apart.
wire ss_eng_owner = savestate_busy;

wire  [9:0] mb_ss_bus_adr    = ss_eng_owner ? eng_ss_bus_adr  : seed_ss_bus_adr;
wire [63:0] mb_ss_bus_din    = ss_eng_owner ? eng_ss_bus_din  : seed_ss_bus_din;
wire        mb_ss_bus_wren   = ss_eng_owner ? eng_ss_bus_wren : seed_ss_bus_wren;
wire        mb_ss_bus_rst    = ss_eng_owner ? eng_ss_bus_rst  : 1'b0;

wire  [1:0] mb_ss_mem_type   = wram_clear_busy ? clear_ss_mem_type :
                               ss_eng_owner    ? eng_ss_mem_type   : seed_ss_mem_type;
wire        mb_ss_mem_active = wram_clear_busy ? clear_ss_mem_active :
                               ss_eng_owner    ? eng_ss_mem_active   : seed_ss_mem_active;
wire [13:0] mb_ss_mem_addr   = wram_clear_busy ? clear_ss_mem_addr :
                               ss_eng_owner    ? eng_ss_mem_addr   : seed_ss_mem_addr;
wire  [7:0] mb_ss_mem_wdata  = wram_clear_busy ? clear_ss_mem_wdata :
                               ss_eng_owner    ? eng_ss_mem_wdata   : seed_ss_mem_wdata;
wire        mb_ss_mem_wren   = wram_clear_busy ? clear_ss_mem_wren :
                               ss_eng_owner    ? eng_ss_mem_wren   : seed_ss_mem_wren;
wire        mb_ss_mem_rden   = wram_clear_busy ? clear_ss_mem_rden :
                               ss_eng_owner    ? eng_ss_mem_rden   : seed_ss_mem_rden;

// The host wall clock sits between hps_io and the seed engine. Main_MiSTer
// sends this core its RTC exactly once, at core start: its 60-second resend
// only runs for cores named "neogeo" or "minimig", so without this block a
// game loaded twenty minutes later would get a clock twenty minutes slow.
// ngp_host_clock latches that one packet, carries it forward, and re-publishes
// it in the same packet shape the seed engine already consumes.
//
// `ce` is tied high ON PURPOSE and must stay that way. This is host time, not
// machine state: it is deliberately outside the pause tree (a savestate capture
// must not stop the host clock), outside the savestate layout (restoring host
// time would drag it backwards), and outside `reset` (an OSD reset or a new
// game load must not restart it). The module has no pause, savestate or reset
// port at all, so those three properties cannot be lost by miswiring here.
wire [64:0] host_rtc;

ngp_host_clock host_clock
(
	.clk      (clk_sys),
	.ce       (1'b1),
	.hps_rtc  (hps_rtc),
	.host_rtc (host_rtc)
);

ngp_setup_seed setup_seed
(
	.clk                    (clk_sys),
	.reset                  (reset),
	.setup_ready            (bios_setup_ready),
	.mono                   (bios_mono_active),
	.osd_language_japanese  (status[3]),
	.osd_palette            (status[16:14]),
	.use_hps_rtc            (!status[17]),
	.hps_rtc                (host_rtc),
	.skip_bios_animation    (!status[19]),
	.cart_header_valid      (cart_header_valid),
	.cart_catalog           (cart_header_catalog),
	.cart_subcatalog        (cart_header_subcatalog),
	.cart_title             (cart_header_title),
	.pause_req              (seed_pause_req),
	.pause_ready            (pause_ready),
	.ss_bus_adr             (seed_ss_bus_adr),
	.ss_bus_din             (seed_ss_bus_din),
	.ss_bus_wren            (seed_ss_bus_wren),
	.ss_mem_type            (seed_ss_mem_type),
	.ss_mem_active          (seed_ss_mem_active),
	.ss_mem_addr            (seed_ss_mem_addr),
	.ss_mem_wdata           (seed_ss_mem_wdata),
	.ss_mem_wren            (seed_ss_mem_wren),
	.ss_mem_rden            (seed_ss_mem_rden),
	.seed_busy              (seed_busy),
	.seed_done              (seed_done)
);

ngp_mainboard mainboard
(
	.clk_sys              (clk_sys),
	.reset                (reset),
	.restore_reset        (ss_core_reset),
	.mono_strap           (mono_strap),
	.lcd_persistence      (status[20]),
	.palette_frame_boundary (status[23]),

	.ce_pix               (ce_pix),
	.vga_r                (vga_r),
	.vga_g                (vga_g),
	.vga_b                (vga_b),
	.hsync                (hsync),
	.vsync                (vsync),
	.hblank               (hblank),
	.vblank               (vblank),

	.audio_l              (audio_l),
	.audio_r              (audio_r),

	.btn                  (btn),
	.power_btn            (power_btn),

	.link_txd             (link_txd),
	.link_rxd             (link_rxd),
	.link_rts_n           (link_rts_n),
	.link_cts_n           (link_cts_n),
	.link_present         (link_present),

	.led_user             (led_user),
	.bios_setup_ready     (bios_setup_ready),
	.bios_mono_active     (bios_mono_active),

	.bios_wr              (bios_wr),
	.bios_sel             (bios_sel),
	.bios_addr            (bios_addr),
	.bios_data            (bios_data),
	.skip_bios_animation  (!status[19]),
	.cheat_load_begin     (cheat_load_begin),
	.cheat_invalidate     (cheat_invalidate),
	.cheat_commit_req     (cheat_commit_req),
	.cheat_commit_done    (cheat_commit_done),
	.cheat_code           (cheat_code),

	.cart_image_bytes     (cart_image_bytes),
	.cart_config_load     (cart_config_load),
	.cart_force_8m_die0   (cart_force_8m_die0),
	.cart_force_flash_read(overlay_force_flash_read),
	.cart_size_code0      (cart_size_code0),
	.cart_size_code1      (cart_size_code1),
	.cart_bytes           (cart_bytes),
	.cart_present         (cart_present),
	.cart_mem_req         (cart_mem_req),
	.cart_mem_we          (cart_mem_we),
	.cart_mem_addr        (cart_mem_addr),
	.cart_mem_wdata       (cart_mem_wdata),
	.cart_mem_be          (cart_mem_be),
	.cart_mem_lane        (cart_mem_lane),
	.cart_mem_tag         (cart_mem_tag),
	.cart_mem_flash       (cart_mem_flash),
	.cart_mem_rdata       (cart_mem_rdata),
	.cart_mem_rvalid      (cart_mem_rvalid),
	.cart_mem_done        (cart_mem_done),
	.cart_dirty_pulse     (cart_dirty_pulse),
	.cart_dirty0          (cart_dirty0),
	.cart_dirty1          (cart_dirty1),
	.cart_dirty0_event    (cart_dirty0_event),
	.cart_dirty0_block    (cart_dirty0_block),
	.cart_dirty1_event    (cart_dirty1_event),
	.cart_dirty1_block    (cart_dirty1_block),
	.cart_dirty_clear     (cart_dirty_clear),
	.cart_flash_busy      (cart_flash_busy),
	.cart_die_busy        (cart_die_busy),

	// The internals bus and the flat memory tap, shared by the work-RAM clear
	// walker, the savestate engine and the BIOS setup seeder (see the priority
	// note where these muxes are built).
	.ss_bus_adr           (mb_ss_bus_adr),
	.ss_bus_din           (mb_ss_bus_din),
	.ss_bus_wren          (mb_ss_bus_wren),
	.ss_bus_rst           (mb_ss_bus_rst),
	.ss_restore_is_rewind (ss_restore_is_rewind),
	.ss_bus_dout          (ss_bus_dout),
	.ss_mem_type          (mb_ss_mem_type),
	.ss_mem_active        (mb_ss_mem_active),
	.ss_mem_addr          (mb_ss_mem_addr),
	.ss_mem_wdata         (mb_ss_mem_wdata),
	.ss_mem_wren          (mb_ss_mem_wren),
	.ss_mem_rden          (mb_ss_mem_rden),
	.ss_mem_rdata         (ss_mem_rdata),
	.loading_savestate    (loading_savestate),
	.pause_req            (seed_pause_req | eng_pause_req | cheat_pause_req |
	                       overlay_pause_req),
	.pause_ready          (pause_ready)
);

//////////////////// Cartridge backing store and loader //////////////////////

// ngp_cart_rom drives the cartridge part of ioctl_wait, and that matters: its
// 0xFF tail prefill costs up to ~65 ms on an odd-sized image, and the first
// cycle of every download is stalled while the die population is invalidated.
// The HPS must be held for it or words are dropped on the floor.
//
// ioctl_download_i is handed the already-decoded cart term. The module makes
// the same [5:0] / index-255 comparison internally, so this is belt and braces,
// and it keeps the whole download decode visible in one place in this file.

ngp_cart_rom cart_loader
(
	.clk_sys               (clk_sys),
	.reset_i               (hard_reset),

	.ioctl_download_i      (cart_dl),
	.ioctl_wr_i            (ioctl_wr),
	.ioctl_addr_i          (ioctl_addr),
	.ioctl_dout_i          (ioctl_dout),
	.ioctl_index_i         (ioctl_index),
	.ioctl_wait_o          (cart_ioctl_wait),

	.load_req_o            (cart_load_req),
	.load_addr_o           (cart_load_addr),
	.load_data_o           (cart_load_data),
	.load_ready_i          (cart_load_ready),
	.load_done_i           (cart_load_done),

	.image_bytes_o         (cart_image_bytes),
	.image_crc32_o         (cart_image_crc32),
	.config_load_o         (cart_loader_config),
	.cart_bytes_i          (cart_bytes),
	.header_valid_o        (cart_header_valid),
	.header_catalog_o      (cart_header_catalog),
	.header_subcatalog_o   (cart_header_subcatalog),
	.header_title_o        (cart_header_title),
	.cart_download_o       (cart_download),
	.cart_download_start_o (cart_download_start),
	.cart_ready_o          (cart_ready)
);

// The loader's p2 stream carries both ROM words and its physical-capacity
// 0xFF tail. Fan it out before either destination sees it so `cart_ready`
// proves the live SDRAM image and DDR3 pristine shadow are identical.
wire [27:1] shadow_ddr_addr;
wire [63:0] shadow_ddr_din;
wire        shadow_ddr_req;
wire        shadow_ddr_rnw;
wire [7:0]  shadow_ddr_be;

ngp_cart_shadow_loader cart_shadow_loader
(
	.clk          (clk_sys),
	.reset        (hard_reset),

	.load_req_i   (cart_load_req),
	.load_addr_i  (cart_load_addr),
	.load_data_i  (cart_load_data),
	.identity_reset_i (cart_download_start),
	.load_ready_o (cart_load_ready),
	.load_done_o  (cart_load_done),
	.pristine_crc32_o (cart_pristine_crc32),

	.live_req_o   (cart_live_load_req),
	.live_addr_o  (cart_live_load_addr),
	.live_data_o  (cart_live_load_data),
	.live_ready_i (cart_live_load_ready),
	.live_done_i  (cart_live_load_done),

	.ddr_addr_o   (shadow_ddr_addr),
	.ddr_din_o    (shadow_ddr_din),
	.ddr_req_o    (shadow_ddr_req),
	.ddr_rnw_o    (shadow_ddr_rnw),
	.ddr_be_o     (shadow_ddr_be),
	.ddr_ready_i  (shadow_ddr_ready)
);

// The background mailbox is the sole live-cart maintenance path. Sparse S0
// and manual-state transactions are statically exclusive and share it below.
wire cart_bg_owner_state = savestate_busy;
wire cart_bg_req = cart_bg_owner_state ? state_p2_req : overlay_p2_req;
wire cart_bg_we = cart_bg_owner_state ? state_p2_we : overlay_p2_we;
wire [24:0] cart_bg_addr = cart_bg_owner_state ? state_p2_addr : overlay_p2_addr;
wire [15:0] cart_bg_wdata = cart_bg_owner_state ? state_p2_wdata : overlay_p2_wdata;
wire [1:0] cart_bg_be = cart_bg_owner_state ? state_p2_be : overlay_p2_be;

assign overlay_p2_ready = !cart_bg_owner_state && cart_bg_ready;
assign overlay_p2_done = !cart_bg_owner_state && cart_bg_done;
assign state_p2_ready = cart_bg_owner_state && cart_bg_ready;
assign state_p2_done = cart_bg_owner_state && cart_bg_done;

ngp_cart_sdram #(.CLK_FREQ_HZ(98_304_000)) cart_sdram
(
	.clk_sys              (clk_sys),
	.clk_ram              (clk_ram),
	.reset_i              (hard_reset),

	.mem_req_i            (cart_mem_req),
	.mem_we_i             (cart_mem_we),
	.mem_addr_i           (cart_mem_addr),
	.mem_wdata_i          (cart_mem_wdata),
	.mem_be_i             (cart_mem_be),
	.mem_lane_i           (cart_mem_lane),
	.mem_tag_i            (cart_mem_tag),
	.mem_flash_i          (cart_mem_flash),
	.mem_rdata_o          (cart_mem_rdata),
	.mem_rvalid_o         (cart_mem_rvalid),
	.mem_done_o           (cart_mem_done),

	.load_req_i           (cart_live_load_req),
	.load_addr_i          (cart_live_load_addr),
	.load_data_i          (cart_live_load_data),
	.load_ready_o         (cart_live_load_ready),
	.load_done_o          (cart_live_load_done),

	.bg_req_i             (cart_bg_req),
	.bg_we_i              (cart_bg_we),
	.bg_addr_i            (cart_bg_addr),
	.bg_wdata_i           (cart_bg_wdata),
	.bg_be_i              (cart_bg_be),
	.bg_ready_o           (cart_bg_ready),
	.bg_done_o            (cart_bg_done),
	.bg_rdata_o           (cart_bg_rdata),

	.SDRAM_DQ             (SDRAM_DQ),
	.SDRAM_A              (SDRAM_A),
	.SDRAM_DQML           (SDRAM_DQML),
	.SDRAM_DQMH           (SDRAM_DQMH),
	.SDRAM_BA             (SDRAM_BA),
	.SDRAM_nCS            (SDRAM_nCS),
	.SDRAM_nWE            (SDRAM_nWE),
	.SDRAM_nRAS           (SDRAM_nRAS),
	.SDRAM_nCAS           (SDRAM_nCAS),
	.SDRAM_CKE            (SDRAM_CKE),
	.SDRAM_CLK            (SDRAM_CLK)
);

/////////////////// Sparse cartridge-flash overlay (.sav) ///////////////////

// The controller persists complete selected physical erase blocks. The live
// array stays in cartridge SDRAM; loader data and the physical 0xFF tail have
// already built the immutable DDR3 shadow used by restore-before-apply.
reg overlay_save_toggle_q;
reg overlay_load_toggle_q;

always @(posedge clk_sys) begin
	if (base_reset || cart_download_start) begin
		overlay_save_toggle_q <= status[11];
		overlay_load_toggle_q <= status[12];
	end else begin
		overlay_save_toggle_q <= status[11];
		overlay_load_toggle_q <= status[12];
	end
end

wire overlay_manual_save = (status[11] ^ overlay_save_toggle_q) &&
	!savestate_busy;
wire overlay_manual_load = (status[12] ^ overlay_load_toggle_q) &&
	!savestate_busy;

// Manual-state requests win simultaneous admission. The overlay itself
// retains mount/manual requests while disabled, so no request is lost and no
// two transaction controllers can own p2/staging on the same edge.
wire [27:1] overlay_s0_ddr_addr;
wire [63:0] overlay_s0_ddr_din;
wire        overlay_s0_ddr_req;
wire        overlay_s0_ddr_rnw;
wire [7:0]  overlay_s0_ddr_be;
wire [63:0] overlay_s0_ddr_dout;
wire        overlay_s0_ddr_ready;
// A boot overlay runs while the machine is held in reset. Runtime save/load
// operations use the ordered pause acknowledgement instead.
wire        overlay_safe_stopped = reset | pause_ready;

ngp_cart_overlay cart_overlay
(
	.clk                         (clk_sys),
	// A console/OSD reset does not replace or rewrite the cartridge SDRAM.
	// Preserve its overlay ledgers and mounted-directory generation as well.
	.reset                       (hard_reset),
	.cart_replace_i              (cart_download_start),
	.cart_ready_i                (cart_ready),
	.identity_raw_crc32_i        (cart_image_crc32),
	.identity_raw_bytes_i        ({7'd0, cart_image_bytes}),
	.identity_pristine_crc32_i   (cart_pristine_crc32),
	.identity_physical_bytes_i   ({7'd0, persist_cart_bytes}),
	.identity_die0_code_i        (persist_die0_code),
	.identity_die1_code_i        (persist_die1_code),
	.identity_catalog_i          (cart_header_catalog),
	.identity_subcatalog_i       (cart_header_subcatalog),
	.identity_title_i            (cart_header_title),
	.event0_i                    (cart_dirty0_event),
	.block0_i                    (cart_dirty0_block),
	.event1_i                    (cart_dirty1_event),
	.block1_i                    (cart_dirty1_block),
	.die_busy_i                  (cart_die_busy),
	.mount_i                     (img_mounted[0]),
	.mount_readonly_i            (img_readonly),
	.mount_size_i                (img_size),
	.save_i                      (overlay_manual_save),
	.load_i                      (overlay_manual_load),
	.operation_enable_i          (overlay_operation_enable),
	.autosave_disable_i          (status[10]),
	.osd_open_i                  (OSD_STATUS),
	.state_adopt_i               (state_ledger_adopt),
	.state_map0_i                (state_ledger0),
	.state_map1_i                (state_ledger1),
	.state_force_flash_read_i    (state_force_flash_read),
	.pause_ready_i               (overlay_safe_stopped),
	.pause_req_o                 (overlay_pause_req),
	.force_flash_read_o          (overlay_force_flash_read),
	.busy_o                      (overlay_busy),
	.boot_hold_o                 (overlay_boot_hold),
	.pending_o                   (overlay_pending),
	.mounted_writable_o          (overlay_mounted_writable),
	.save_done_o                 (overlay_save_done),
	.save_rejected_o             (overlay_save_rejected),
	.load_done_o                 (overlay_load_done),
	.load_rejected_o             (overlay_load_rejected),
	.live0_o                     (overlay_live0),
	.live1_o                     (overlay_live1),
	.pending0_o                  (overlay_pending0),
	.pending1_o                  (overlay_pending1),
	.file0_o                     (overlay_file0),
	.file1_o                     (overlay_file1),
	.sd_lba_o                    (overlay_sd_lba),
	.sd_rd_o                     (overlay_sd_rd),
	.sd_wr_o                     (overlay_sd_wr),
	.sd_ack_i                    (sd_ack[0]),
	.sd_buff_addr_i              (sd_buff_addr),
	.sd_buff_dout_i              (sd_buff_dout),
	.sd_buff_wr_i                (sd_buff_wr),
	.sd_buff_din_o               (overlay_sd_buff_din),
	.p2_req_o                    (overlay_p2_req),
	.p2_we_o                     (overlay_p2_we),
	.p2_addr_o                   (overlay_p2_addr),
	.p2_wdata_o                  (overlay_p2_wdata),
	.p2_be_o                     (overlay_p2_be),
	.p2_ready_i                  (overlay_p2_ready),
	.p2_done_i                   (overlay_p2_done),
	.p2_rdata_i                  (cart_bg_rdata),
	.ddr_addr_o                  (overlay_s0_ddr_addr),
	.ddr_din_o                   (overlay_s0_ddr_din),
	.ddr_req_o                   (overlay_s0_ddr_req),
	.ddr_rnw_o                   (overlay_s0_ddr_rnw),
	.ddr_be_o                    (overlay_s0_ddr_be),
	.ddr_dout_i                  (overlay_s0_ddr_dout),
	.ddr_ready_i                 (overlay_s0_ddr_ready)
);

// This bitmap is now diagnostic/controller-internal state only. Clear it at a
// completed directory commit; the canonical live/pending/file ledgers above
// have independent lifetimes and are not cleared by this pulse.
assign cart_dirty_clear = overlay_save_done;

////////////////////////////// Savestates ////////////////////////////////////

// Four manual slots at physical 0x32000000, 8 MB apart, exactly matching the
// `SS32000000:800000` token in the title string. The generic engine writes
// internals and memory types 0..2; a manifest plus selected complete flash
// blocks forms the dynamically sized Type3 tail. It never carries an
// unconditional copy of the cartridge image.
//
// No automatic-history ring is allocated: `ngp_savestate` ties `rewind_on`
// and `rewind_active` low, so nothing schedules a background capture and
// `TIME_CAPTURE` never fires.

wire  [1:0] ss_slot;
wire        ss_status_update;
wire        ss_load_done;

// savestate_ui owns the slot number once the user changes it with a pad chord
// or an F-key, and writes it back into status[46:45] so the OSD entry agrees.
assign status_in  = {status[127:47], ss_slot, status[44:0]};
assign status_set = ss_status_update;

// The NGP pad has no Start button, and savestate_ui's save/load condition is
// `(joySaveState || (joySS && joyStart))`. Driving BOTH `joySS` and
// `joySaveState` from the one Savestates button collapses that to the button
// itself, so chord+Down saves and chord+Up loads with no Start in the
// picture. `joyStart` is therefore tied low rather than borrowed from
// Option -- Option is a real console button and a game is entitled to it.
//
//   chord + Right / Left   next / previous slot, clamped to 0..3
//   chord + Down / Up      save / load
//   Alt + F1..F4           save to slot 1..4      F1..F4  load from slot 1..4
//   holding the chord      the slot help toast, after the module's timeout
//
// The DIRECTIONS come from `joystick_0` raw, not from `btn`. `btn` has the
// d-pad suppressed while the chord is held, so the game does not walk while
// the user picks a slot; the chord itself has to see exactly the directions
// the machine must not.
savestate_ui savestate_ui
(
	.clk           (clk_sys),
	.ps2_key       (ps2_key[10:0]),
	.allow_ss      (allow_ss),
	.joySS         (joystick_0[8]),
	.joyRight      (joystick_0[0]),
	.joyLeft       (joystick_0[1]),
	.joyDown       (joystick_0[2]),
	.joyUp         (joystick_0[3]),
	.joyStart      (1'b0),
	.joySaveState  (joystick_0[8]),
	.status_slot   (status[46:45]),
	.OSD_saveload  (status[43:42]),
	.ss_save       (ss_save_req),
	.ss_load       (ss_load_req),
	.ss_info_req   (ss_info_req),
	.ss_info       (ss_info),
	.statusUpdate  (ss_status_update),
	.selected_slot (ss_slot)
);

wire [63:0] shadow_ddr_dout;
wire        shadow_ddr_ready;

wire [27:1] state_sparse_ddr_addr;
wire [63:0] state_sparse_ddr_din;
wire        state_sparse_ddr_req;
wire        state_sparse_ddr_rnw;
wire [7:0]  state_sparse_ddr_be;
wire [63:0] state_sparse_ddr_dout;
wire        state_sparse_ddr_ready;

wire [27:1] persistent_ddr_addr;
wire [63:0] persistent_ddr_din;
wire        persistent_ddr_req;
wire        persistent_ddr_rnw;
wire [7:0]  persistent_ddr_be;
wire [63:0] persistent_ddr_dout;
wire        persistent_ddr_ready;

wire [63:0] ddr_ch2_dout;
wire        ddr_ch2_ready;
wire [27:1] ddr_ch2_addr;
wire [63:0] ddr_ch2_din;
wire        ddr_ch2_req;
wire        ddr_ch2_rnw;
wire [7:0]  ddr_ch2_be;

// S0 and sparse manual-state transactions are excluded at request admission,
// but the arbiter makes the single-outstanding bundled-data ownership
// structural. S0 has priority if a future integration mistake raises both.
ngp_ddr_ch2_arbiter persistent_ddr_arbiter
(
	.clk             (clk_sys),
	.reset           (hard_reset),
	.loader_prefer_i (overlay_busy),
	.loader_addr_i   (overlay_s0_ddr_addr),
	.loader_din_i    (overlay_s0_ddr_din),
	.loader_req_i    (overlay_s0_ddr_req),
	.loader_rnw_i    (overlay_s0_ddr_rnw),
	.loader_be_i     (overlay_s0_ddr_be),
	.loader_dout_o   (overlay_s0_ddr_dout),
	.loader_ready_o  (overlay_s0_ddr_ready),
	.overlay_addr_i  (state_sparse_ddr_addr),
	.overlay_din_i   (state_sparse_ddr_din),
	.overlay_req_i   (state_sparse_ddr_req),
	.overlay_rnw_i   (state_sparse_ddr_rnw),
	.overlay_be_i    (state_sparse_ddr_be),
	.overlay_dout_o  (state_sparse_ddr_dout),
	.overlay_ready_o (state_sparse_ddr_ready),
	.ddr_addr_o      (persistent_ddr_addr),
	.ddr_din_o       (persistent_ddr_din),
	.ddr_req_o       (persistent_ddr_req),
	.ddr_rnw_o       (persistent_ddr_rnw),
	.ddr_be_o        (persistent_ddr_be),
	.ddr_dout_i      (persistent_ddr_dout),
	.ddr_ready_i     (persistent_ddr_ready)
);

// Cartridge download has strict priority over every mutable staging client;
// cart_ready cannot rise until every pristine-shadow write is acknowledged.
ngp_ddr_ch2_arbiter ddr_ch2_arbiter
(
	.clk             (clk_sys),
	.reset           (hard_reset),
	.loader_prefer_i (cart_download),

	.loader_addr_i   (shadow_ddr_addr),
	.loader_din_i    (shadow_ddr_din),
	.loader_req_i    (shadow_ddr_req),
	.loader_rnw_i    (shadow_ddr_rnw),
	.loader_be_i     (shadow_ddr_be),
	.loader_dout_o   (shadow_ddr_dout),
	.loader_ready_o  (shadow_ddr_ready),

	.overlay_addr_i  (persistent_ddr_addr),
	.overlay_din_i   (persistent_ddr_din),
	.overlay_req_i   (persistent_ddr_req),
	.overlay_rnw_i   (persistent_ddr_rnw),
	.overlay_be_i    (persistent_ddr_be),
	.overlay_dout_o  (persistent_ddr_dout),
	.overlay_ready_o (persistent_ddr_ready),

	.ddr_addr_o      (ddr_ch2_addr),
	.ddr_din_o       (ddr_ch2_din),
	.ddr_req_o       (ddr_ch2_req),
	.ddr_rnw_o       (ddr_ch2_rnw),
	.ddr_be_o        (ddr_ch2_be),
	.ddr_dout_i      (ddr_ch2_dout),
	.ddr_ready_i     (ddr_ch2_ready)
);
wire [27:1] ddr_ch1_addr;
wire [63:0] ddr_ch1_din;
wire [63:0] ddr_ch1_dout;
wire        ddr_ch1_req;
wire        ddr_ch1_rnw;
wire  [7:0] ddr_ch1_be;
wire        ddr_ch1_ready;

ngp_savestate
#(
	.SAVESTATE_ADDR  (32'h0080_0000),   // (0x32000000 - 0x30000000) / 4, dwords
	.SAVESTATE_SHIFT (21)               // 2^21 dwords = 8 MB, = ss_size
)
savestate
(
	.clk                   (clk_sys),
	// Keep the transaction controller alive across a console/OSD reset. The
	// generic engine does not abort a busy restore on reset_in, so resetting
	// only the sparse half would allow a partial cart apply to resume. A hard
	// reset remains the coordinated abort boundary.
	.reset                 (hard_reset),

	.ss_save               (ss_save_req),
	.ss_load               (ss_load_req),
	.ss_slot               (ss_slot),
	// status[44] = "Savestate to SD: Off". With the header count frozen the
	// engine writes only the size word, leaving the HPS change detector alone,
	// so the state lives in DDR and is never flushed to disk.
	.increase_header_count (!status[44]),

	.pause_req             (eng_pause_req),
	.paused                (pause_ready),
	.restore_safe_stopped_i(reset | pause_ready),
	.core_reset            (ss_core_reset),
	.loading_savestate     (loading_savestate),
	.busy                  (savestate_busy),
	.load_done             (ss_load_done),
	.restore_is_rewind     (ss_restore_is_rewind),

	.identity_raw_crc32_i      (cart_image_crc32),
	.identity_raw_bytes_i      ({7'd0, cart_image_bytes}),
	.identity_pristine_crc32_i (cart_pristine_crc32),
	.identity_physical_bytes_i ({7'd0, persist_cart_bytes}),
	.identity_die0_code_i      (persist_die0_code),
	.identity_die1_code_i      (persist_die1_code),
	.cart_geometry_ready_i     (persist_geometry_ready),
	.identity_catalog_i        (cart_header_catalog),
	.identity_subcatalog_i     (cart_header_subcatalog),
	.identity_title_i          (cart_header_title),
	.live_ledger0_i            (overlay_live0),
	.live_ledger1_i            (overlay_live1),
	.die_busy_i                (cart_die_busy),
	.event0_i                  (cart_dirty0_event),
	.block0_i                  (cart_dirty0_block),
	.event1_i                  (cart_dirty1_event),
	.block1_i                  (cart_dirty1_block),
	.state_ledger_adopt_o      (state_ledger_adopt),
	.state_ledger0_o           (state_ledger0),
	.state_ledger1_o           (state_ledger1),
	.state_force_flash_read_o  (state_force_flash_read),

	.ss_bus_adr            (eng_ss_bus_adr),
	.ss_bus_din            (eng_ss_bus_din),
	.ss_bus_wren           (eng_ss_bus_wren),
	.ss_bus_rst            (eng_ss_bus_rst),
	.ss_bus_dout           (ss_bus_dout),

	.ss_mem_type           (eng_ss_mem_type),
	.ss_mem_active         (eng_ss_mem_active),
	.ss_mem_addr           (eng_ss_mem_addr),
	.ss_mem_wdata          (eng_ss_mem_wdata),
	.ss_mem_wren           (eng_ss_mem_wren),
	.ss_mem_rden           (eng_ss_mem_rden),
	.ss_mem_rdata          (ss_mem_rdata),

	.ddr_addr              (ddr_ch1_addr),
	.ddr_din               (ddr_ch1_din),
	.ddr_dout              (ddr_ch1_dout),
	.ddr_req               (ddr_ch1_req),
	.ddr_rnw               (ddr_ch1_rnw),
	.ddr_be                (ddr_ch1_be),
	.ddr_ready             (ddr_ch1_ready),

	.sparse_ddr_addr_o     (state_sparse_ddr_addr),
	.sparse_ddr_din_o      (state_sparse_ddr_din),
	.sparse_ddr_req_o      (state_sparse_ddr_req),
	.sparse_ddr_rnw_o      (state_sparse_ddr_rnw),
	.sparse_ddr_be_o       (state_sparse_ddr_be),
	.sparse_ddr_dout_i     (state_sparse_ddr_dout),
	.sparse_ddr_ready_i    (state_sparse_ddr_ready),
	.p2_req_o              (state_p2_req),
	.p2_we_o               (state_p2_we),
	.p2_addr_o             (state_p2_addr),
	.p2_wdata_o            (state_p2_wdata),
	.p2_be_o               (state_p2_be),
	.p2_ready_i            (state_p2_ready),
	.p2_done_i             (state_p2_done),
	.p2_rdata_i            (cart_bg_rdata)
);

ddram ddram
(
	.DDRAM_CLK        (clk_sys),
	.DDRAM_BUSY       (DDRAM_BUSY),
	.DDRAM_BURSTCNT   (DDRAM_BURSTCNT),
	.DDRAM_ADDR       (DDRAM_ADDR),
	.DDRAM_DOUT       (DDRAM_DOUT),
	.DDRAM_DOUT_READY (DDRAM_DOUT_READY),
	.DDRAM_RD         (DDRAM_RD),
	.DDRAM_DIN        (DDRAM_DIN),
	.DDRAM_BE         (DDRAM_BE),
	.DDRAM_WE         (DDRAM_WE),

	.ch1_addr         (ddr_ch1_addr),
	.ch1_dout         (ddr_ch1_dout),
	.ch1_din          (ddr_ch1_din),
	.ch1_req          (ddr_ch1_req),
	.ch1_rnw          (ddr_ch1_rnw),
	.ch1_be           (ddr_ch1_be),
	.ch1_ready        (ddr_ch1_ready),

	// Channel 2 holds the immutable cartridge shadow and sparse staging traffic.
	// Both join through the explicit owner mux above, never as parallel clients.
	.ch2_addr         (ddr_ch2_addr),
	.ch2_dout         (ddr_ch2_dout),
	.ch2_din          (ddr_ch2_din),
	.ch2_req          (ddr_ch2_req),
	.ch2_rnw          (ddr_ch2_rnw),
	.ch2_be           (ddr_ch2_be),
	.ch2_ready        (ddr_ch2_ready)
);

//////////////////////////////// Video ///////////////////////////////////////

// The machine remains native -- 160x152 inside 515 x 199 dots at 6.144 MHz,
// 59.95 Hz -- while the board expands complete RGB444 generations by nibble
// replication into an RGB888 ping-pong presentation store. The framework sees
// the native 160x152 active picture one-for-one inside a 262-line progressive
// CRT raster at about 15.707 kHz. Both frames are exactly 819,880 clk_sys
// clocks long, so after boundary admission they run continuously in lockstep
// without changing machine speed or inserting a per-frame pause.
//
// The reader bank changes only at the public frame wrap. CE_PIXEL, active-high
// syncs and blanks are generated by that free-running public raster, including
// while the emulated machine is parked for startup, a savestate, or exceptional
// resynchronization.

assign CLK_VIDEO = clk_sys;

wire [1:0] ar = status[122:121];
wire       vga_de_mixed;
wire       freeze_sync;

// The K2GE retains raw RGB444 truth. Raw presentation expands it losslessly to
// RGB888; Panel may retain sub-nibble optical-response history in those output
// bytes. Saturation is an off-chip presentation choice immediately before the
// standard MiSTer gamma/mixer stage. A stale colour-mode status value is
// forcibly bypassed on true NGP hardware, matching the H0 menu visibility
// contract.
ngpc_color_profile color_profile
(
	.mode  (bios_mono_active ? 2'd3 : status[22:21]),
	.r_in  (vga_r),
	.g_in  (vga_g),
	.b_in  (vga_b),
	.r_out (profile_r),
	.g_out (profile_g),
	.b_out (profile_b)
);

video_mixer #(.LINE_LENGTH(160), .HALF_DEPTH(0), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO   (CLK_VIDEO),
	.CE_PIXEL    (CE_PIXEL),
	.ce_pix      (ce_pix),

	.scandoubler (forced_scandoubler),
	.hq2x        (1'b0),

	.gamma_bus   (gamma_bus),

	.R           (profile_r),
	.G           (profile_g),
	.B           (profile_b),

	.HSync       (hsync),
	.VSync       (vsync),
	.HBlank      (hblank),
	.VBlank      (vblank),

	.HDMI_FREEZE (HDMI_FREEZE),
	.freeze_sync (freeze_sync),

	.VGA_R       (VGA_R),
	.VGA_G       (VGA_G),
	.VGA_B       (VGA_B),
	.VGA_VS      (VGA_VS),
	.VGA_HS      (VGA_HS),
	.VGA_DE      (vga_de_mixed)
);

// Aspect: the LCD is 160 x 152 square pixels, so "Original" is 20:19.
// video_freak owns VIDEO_ARX/ARY because integer scaling sets their [12] flag.

video_freak video_freak
(
	.CLK_VIDEO   (CLK_VIDEO),
	.CE_PIXEL    (CE_PIXEL),
	.VGA_VS      (VGA_VS),
	.HDMI_WIDTH  (HDMI_WIDTH),
	.HDMI_HEIGHT (HDMI_HEIGHT),
	.VGA_DE      (VGA_DE),
	.VIDEO_ARX   (VIDEO_ARX),
	.VIDEO_ARY   (VIDEO_ARY),

	.VGA_DE_IN   (vga_de_mixed),
	.ARX         ((ar == 2'd0) ? 12'd20 : {10'd0, ar - 2'd1}),
	.ARY         ((ar == 2'd0) ? 12'd19 : 12'd0),
	.CROP_SIZE   (12'd0),
	.CROP_OFF    (5'd0),
	.SCALE       (status[6:4])
);

/////////////////////////////// Audio ////////////////////////////////////////

// rtl/snd/ngp_mixer.sv produces SIGNED two's-complement samples (its header
// says so and its outputs are declared `signed`), so AUDIO_S is 1. Declaring
// unsigned samples as signed is silent: sys/audio_out.sv XORs the sign bit.
// The samples are registered on ce_3m072 inside the machine, which holds each
// value for about eight CLK_AUDIO cycles -- past the framework's "hold at least
// two" rule.

assign AUDIO_L   = audio_l;
assign AUDIO_R   = audio_r;
assign AUDIO_S   = 1'b1;
assign AUDIO_MIX = status[8:7];

/////////////////////////////// LEDs /////////////////////////////////////////

// The red LED D10 is driven by the machine, so the framework LED follows it.
assign LED_USER = led_user;

//////////////////// Deliberately unread at this level ///////////////////////
//
//   cart_dirty0/1            legacy diagnostic per-block bitmaps. Persistence
//                            owns separate live/pending/file ledgers.
//   cart_size_code0/1        live flash-command geometry; persistence derives
//                            retained geometry independently across reset.
//   cart_flash_busy          legacy coalesced activity indication. Sparse
//                            capture uses the per-die busy/event signals.
//   sd_buff_addr[12:8]       only 256 words of a 512-byte sector can move
//   freeze_sync              HDMI freeze handshake
//   CLK_AUDIO                an input clock for the framework audio path only;
//                            core logic must never be clocked from it
//   sdram_sz, TIMESTAMP      left unconnected on hps_io; RTC is consumed by
//                            the setup seed

wire unused_ok = &{1'b0,
	ext_bus,
	cart_dirty_pulse, cart_dirty0, cart_dirty1,
	cart_size_code0, cart_size_code1, cart_flash_busy,
	sd_buff_addr[12:8],
	wram_clear_done,
	freeze_sync,
	buttons[0], joystick_0[31:9], status[127:123],
	status[120:47], status[41:25], status[9],
	img_mounted[1], sd_ack[1], ss_load_done,
	overlay_pending, overlay_mounted_writable,
	overlay_save_rejected, overlay_load_done,
	overlay_load_rejected,
	overlay_pending0, overlay_pending1,
	overlay_file0, overlay_file1,
	shadow_ddr_dout,
	CLK_AUDIO, SD_MISO, SD_CD,
	UART_DSR, USER_IN[6:5], USER_IN[3:2], USER_IN[0],
	1'b0};

endmodule
