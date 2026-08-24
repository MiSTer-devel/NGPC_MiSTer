// Copyright (c) 2026 Jamie Blanks

// K2GE video memory block and its render fetch arbiter.  Owns every memory the
// graphics engine reads while it renders, the CPU face of those memories and
// the savestate flat tap that walks them.  No rendering logic lives here: it
// hands words to k2ge_scroll, k2ge_obj and k2ge_palette and gets out of the way.
//
// Documented internal memory (K1GE 3-1; K2GE 4-1), on the supplied block-RAM
// wrappers (an M10K stores 8192 usable bits at x16, so 512 words per block):
//
//   u_chram  4096 x 16  character RAM 8 KB   0xA000-0xBFFF   8 M10K
//   u_scr1   1024 x 16  scroll plane 1 2 KB  0x9000-0x97FF   2 M10K
//   u_scr2   1024 x 16  scroll plane 2 2 KB  0x9800-0x9FFF   2 M10K
//   u_oam     128 x 16  sprite table 256 B   0x8800-0x88FF   1 M10K
//   u_pal     256 x 16  colour palette 512 B 0x8200-0x83FF   1 M10K
//   u_pal_display 256x16 display-only palette snapshot       1 M10K
//   CP.C      64 x 4    sprite palette code  0x8C00-0x8C3F   256 flops
//                                                           15 M10K
//
// CP.C is held as one packed 256-bit vector rather than an unpacked array so
// that the CPU mux and the sprite evaluator can both read it with a plain part
// select.  The evaluator needs the nibble in the same cycle as the attribute
// word, which is why it is not a sixth block RAM.  The line buffer is charged
// to k2ge_linebuf, where its read-modify-write and clear sweep live, so the
// chip total is 15 here plus 1 there.
//
// Character RAM is the only memory with more than one render-side consumer and
// so the only one needing arbitration.  clk_sys runs at 8x the dot clock, so
// each dot has eight cycles and the rotation spends two per slot:
//
//   cycles 0-1  slot 0  scroll 1 character fetch
//   cycles 2-3  slot 1  sprite   character fetch
//   cycles 4-5  slot 2  scroll 2 character fetch
//   cycles 6-7  slot 3  sprite   character fetch
//
// Each slot is one drive cycle (present addr_b_i) and one capture cycle: the
// wrapper is a one-clock unregistered-output RAM, so q_b_o is valid the cycle
// after the address.  The rotation is a free-running counter locked to ce_6m144
// and re-armed at `line_start`, so the phase is a constant of the design,
// identical on every line and every frame.  It is a fixed schedule rather than
// a request/grant arbiter: a consumer with nothing to fetch idles in its slot,
// and no consumer can starve another because none can take another's cycles.
//
// Handshake seen by a consumer:
//
//   *_ch_slot   high for the one cycle in which this consumer may be taken
//   *_ch_req    consumer holds it with *_ch_addr until it sees the ack
//   *_ch_ack    = *_ch_slot & *_ch_req; the address was latched by the RAM
//   *_ch_valid  pulses two cycles after the ack, with *_ch_data
//
// Scroll map RAM, the sprite table and the palette have a single render-side
// consumer each, so their port B is owned outright and free-running: present an
// address, take the word next cycle.  No arbitration, no handshake.
//
// CPU access rules
//  1. Palette RAM is zero-wait always (SysPro p.6-7; K2GE 4-15 caution 5).
//     Character, scroll and sprite VRAM cost the CPU two wait states while the
//     composition pass owns the memory and none outside it (measured ~ +2.04
//     states per store).  `cpu_wait_states` below carries it.
//  2. One machine state.  The bus is sampled on the clk_sys cycle where
//     ce_6m144 is high, that address drives port A for the rest of the dot, the
//     write pulse is one cycle wide, and cpu_dout is a mux of already registered
//     RAM outputs stable from the third cycle of the dot until the next
//     ce_6m144 tick.  Eight cycles of budget, two used.
//  3. Palette RAM is 16-bit access only: "Using 8 bit access will produce
//     unreliable values" (K2GE 4-15 caution 1).  Hardware retains the
//     word-written colour after a byte store, so byte writes are ignored;
//     P_PAL_BYTE_WRITE = 1 writes the addressed lane instead.  D15:12 do not
//     exist and are masked to 0 on every write path, so they read 0 without a
//     read-side mask.
//  4. Character, scroll and sprite VRAM accept 8-bit and 16-bit access
//     (SysPro p.7), so cpu_we maps straight onto the wrapper's byte enables.
//  5. CP.C reads its stored nibble in both K2GE and compatibility mode, with
//     the high nibble fixed 0 (K2GE 4-3-3).  Keeping the RAM live is what
//     avoids losing data across a mode switch.
//  6. A CPU access never disturbs a render fetch here: the CPU owns port A and
//     the renderer owns port B.  Real silicon delays the renderer instead, and
//     this build does not model that contention.
//
// Savestate flat tap, TYPE2, 16384 bytes as a flat byte array.  The layout is
// keyed to the CPU window, so the flat address IS the CPU window offset for
// every region and one decode serves both.  The tap steals port A while
// `ss_mem_active` is asserted, which is legal because the engine only walks
// memory while the chip is paused and the CPU quiesced.
//
//   0x0000-0x01FF  MMR window       reads 0, writes ignored
//   0x0200-0x03FF  palette          u_pal, byte lane on addr[0]
//   0x0400-0x07FF  MMR window       reads 0, writes ignored
//   0x0800-0x08FF  sprite table     u_oam, byte lane on addr[0]
//   0x0900-0x0B7F  line buffer      forwarded to k2ge_linebuf
//   0x0B80-0x0BFF  vacant           reads 0, writes ignored
//   0x0C00-0x0C3F  sprite CP.C      low nibble, high nibble reads 0
//   0x0C40-0x0E3F  display palette  u_pal_display, byte lane on addr[0]
//   0x0E40-0x0FFF  vacant           reads 0, writes ignored
//   0x1000-0x1FFF  scroll planes    u_scr1 then u_scr2
//   0x2000-0x3FFF  character RAM    u_chram
//
// Present ss_mem_addr with ss_mem_rden and ss_mem_rdata is valid two clk_sys
// cycles later; the engine allows at least seven clocks per byte.  Writes
// commit on the cycle ss_mem_wren is asserted.  The tap deliberately bypasses
// the palette's 16-bit-only rule, because a restore must put back exactly what
// it saved.  The line-buffer window is forwarded rather than absorbed, so
// k2ge_linebuf must accept ss_lb_addr/ss_lb_wdata/ss_lb_wren and return
// ss_lb_rdata one cycle after the address, exactly like a block RAM.
//
// What consumers must provide
//   k2ge_scroll   x2  drives s1_/s2_map_addr and reads the word next cycle;
//                     holds s1_/s2_ch_req with an address until ack, then takes
//                     the word on valid.
//   k2ge_obj          drives oam_addr and reads the word next cycle; drives
//                     cpc_idx and reads the nibble in the SAME cycle; uses the
//                     obj character handshake, which gets two slots per dot.
//   k2ge_palette      drives pal_rd_addr and reads the entry next cycle.  It
//                     must register that address per dot; this module does not
//                     register it for you.
//   k2ge_linebuf      services the savestate line-buffer window above.
//
// `ce` is the chip's own 49.152 MHz enable, tied high in normal operation;
// dropping it freezes the arbiter, the fetch pipeline and the CPU face
// together.  It deliberately does not gate the savestate tap, which has to work
// while the rest of the chip is frozen.  `ce_6m144` is the dot enable and is
// always qualified by `ce`.

module k2ge_vram
#(
	// Palette byte-write behaviour.
	// 1 = write the addressed lane, 0 = ignore byte writes.
	parameter       P_PAL_BYTE_WRITE  = 1'b0,
	// CPU wait states charged for a character/scroll/sprite VRAM access
	// during the drawing period.  Two wait states, measured; 1 and 3 are
	// excluded by ten counts either way.
	parameter [2:0] P_VRAM_CPU_WAIT   = 3'd2,
	// Does the SPRITE TABLE (0x8800-0x88FF) pay P_VRAM_CPU_WAIT too?
	// 1 = measured behaviour (shipping); 0 = exempt the sprite table
	// (diagnostic).
	parameter       P_VRAM_WAIT_OAM   = 1'b1
)
(
	input  wire        clk_sys,       // 49.152 MHz
	input  wire        ce,            // chip enable, tie 1 (see header)
	input  wire        ce_6m144,      // dot-clock enable, 1 of 8
	input  wire        rst,           // synchronous power-on reset
	input  wire        mono_strap,    // 1 = K1GE part: no palette RAM, no CP.C
	input  wire        line_start,    // k2ge_vtimer: entering dot 0 of a line
	input  wire        run,           // SoC main oscillator is running
	input  wire        render_owns,   // composition pass owns character VRAM
	input  wire        frame_start,   // entering dot 0 of raster line 0
	input  wire        palette_sync_start, // entering the final blank raster line
	input  wire        palette_frame_boundary, // raw OSD request, latched per frame
	input  wire [11:0] palette_ss_wdata,
	input  wire        palette_ss_wren,
	output wire [11:0] palette_ss_state,

	// --- CPU bus face ------------------------------------------------------
	input  wire [13:0] cpu_a,         // byte address inside 0x8000-0xBFFF
	input  wire [15:0] cpu_din,
	output wire [15:0] cpu_dout,
	input  wire        cpu_cs,        // window select from the SoC decoder
	input  wire        cpu_rd,        // read strobe (no side effects here)
	input  wire [1:0]  cpu_we,        // [0] = even byte, [1] = odd byte
	output wire [2:0]  cpu_wait_states, // wait states owed to this CPU cycle

	// --- scroll plane 1 ------------------------------------------------------
	input  wire [9:0]  s1_map_addr,   // free-running map port
	output wire [15:0] s1_map_q,
	input  wire        s1_ch_req,
	input  wire [11:0] s1_ch_addr,
	output wire        s1_ch_ack,
	output wire        s1_ch_valid,
	output wire [15:0] s1_ch_data,

	// --- scroll plane 2 ------------------------------------------------------
	input  wire [9:0]  s2_map_addr,
	output wire [15:0] s2_map_q,
	input  wire        s2_ch_req,
	input  wire [11:0] s2_ch_addr,
	output wire        s2_ch_ack,
	output wire        s2_ch_valid,
	output wire [15:0] s2_ch_data,

	// --- sprite evaluator and drawer -----------------------------------------
	input  wire [6:0]  oam_addr,      // free-running sprite table port
	output wire [15:0] oam_q,
	input  wire [5:0]  cpc_idx,       // CP.C read, same-cycle
	output wire [3:0]  cpc_q,
	input  wire        ob_ch_req,
	input  wire [11:0] ob_ch_addr,
	output wire        ob_ch_ack,
	output wire        ob_ch_valid,
	output wire [15:0] ob_ch_data,

	// --- palette read port (k2ge_palette) ------------------------------------
	input  wire [7:0]  pal_rd_addr,
	output wire [15:0] pal_rd_data,

	// --- savestate flat memory tap (TYPE2, 16384 bytes) ----------------------
	input  wire        ss_mem_active,
	input  wire [13:0] ss_mem_addr,
	input  wire [7:0]  ss_mem_wdata,
	input  wire        ss_mem_wren,
	input  wire        ss_mem_rden,
	output wire [7:0]  ss_mem_rdata,

	// --- savestate line-buffer window, forwarded to k2ge_linebuf -------------
	output wire        ss_lb_sel,     // the tap is addressing the line buffer
	output wire [9:0]  ss_lb_addr,    // byte offset 0..639 inside the window
	output wire [7:0]  ss_lb_wdata,
	output wire        ss_lb_wren,
	input  wire [7:0]  ss_lb_rdata    // must arrive one cycle after the address
);

	// Region decode
	// Written out twice rather than wrapped in a function: a function reading
	// module state is not added to the sensitivity list of the assign that
	// calls it.  These are pure address decodes, so a function would be safe
	// - but two copies of five comparators is cheaper to read than the
	// argument about which functions are safe.

	// Colour palette RAM and the sprite CP.C nibbles are K2GE ADDITIONS - the
	// K2GE document lists both under "K1GE -> K2GE expanded function summary"
	// as address ranges "added" (K2GE 5-1 items 1 and 3) - and neither
	// exists on a K1GE.  The K1GE memory map has 0x8200-0x83FF inside the
	// register block's "open area ... PLEASE DO NOT ACCESS. (Future
	// expansion)" and 0x8900-0x8FFF, which contains 0x8C00, marked "vacant"
	// (K1GE 3-2).  So on a mono build this module stops CLAIMING those two
	// windows: writes are dropped with the rest of an unowned address and the
	// CPU read is forced to 0x0000, which is what k2ge.sv's header already
	// says an unowned address does.
	//
	// The RAMs themselves stay in the netlist and stay reachable from the
	// savestate tap, which is why the hide is applied here and at `cpu_dout`
	// rather than on the `sel_*` decode below: `sel_*` also serves the tap, and
	// a mono savestate must still round-trip whatever the blocks happen to
	// hold.  Restoring a colour state into a mono-strapped core is a strap
	// mismatch, not something this decode should try to repair.
	wire k2ge_only_regs = ~mono_strap;

	wire hit_pal = (cpu_a[13:9]  == 5'd1)  && k2ge_only_regs;  // 0x8200-0x83FF
	wire hit_oam = (cpu_a[13:8]  == 6'd8);                     // 0x8800-0x88FF
	wire hit_cpc = (cpu_a[13:6]  == 8'd48) && k2ge_only_regs;  // 0x8C00-0x8C3F
	wire hit_scr = (cpu_a[13:12] == 2'b01);                    // 0x9000-0x9FFF
	wire hit_ch  = cpu_a[13];                                  // 0xA000-0xBFFF

	// This module answers the address; the chip parent's read mux relies on
	// k2ge_vram returning 0x0000 for an address it does not own.
	wire cpu_hit = hit_pal | hit_oam | hit_cpc | hit_scr | hit_ch;

	// CPU bus sampling
	// One machine state: capture the bus at the ce_6m144 tick, hold it for the
	// whole dot, and emit a single-cycle write pulse.  Holding the address is
	// what makes cpu_dout stable for the rest of the dot without a second
	// output register.

	reg [13:0] a_q;
	reg [15:0] d_q;
	reg [1:0]  we_q;
	reg        wr_q;

	always @(posedge clk_sys) begin
		if (rst) begin
			a_q  <= 14'd0;
			d_q  <= 16'd0;
			we_q <= 2'd0;
			wr_q <= 1'b0;
		end else if (ce) begin
			wr_q <= 1'b0;
			if (ce_6m144) begin
				a_q  <= cpu_a;
				d_q  <= cpu_din;
				we_q <= cpu_we;
				wr_q <= cpu_cs && cpu_hit && (|cpu_we);
			end
		end
	end

	// Port A source: the CPU, or the savestate tap when it is walking
	// The flat tap address and the CPU window offset are the same number for
	// every region (the layout was chosen that way), so one
	// decode covers both and the mux is on the address, not on the decode.

	wire [13:0] pa     = ss_mem_active ? ss_mem_addr : a_q;
	wire        pa_wr  = ss_mem_active ? ss_mem_wren : wr_q;
	wire [1:0]  pa_be  = ss_mem_active ? {ss_mem_addr[0], ~ss_mem_addr[0]} : we_q;
	wire [15:0] pa_din = ss_mem_active ? {ss_mem_wdata, ss_mem_wdata} : d_q;

	wire pa_pal = (pa[13:9]  == 5'd1);
	wire pa_oam = (pa[13:8]  == 6'd8);
	wire pa_cpc = (pa[13:6]  == 8'd48);
	wire pa_scr = (pa[13:12] == 2'b01);
	wire pa_ch  = pa[13];
	wire pa_dpal = ss_mem_active && (pa >= 14'h0C40) && (pa < 14'h0E40);
	wire [13:0] pa_dpal_off = pa - 14'h0C40;

	// The line-buffer window is only reachable through the tap; there is no CPU
	// address for it (K2GE 4-2 marks 0x8900-0x8BFF vacant).
	wire pa_lb  = ss_mem_active && (ss_mem_addr >= 14'h0900)
	                            && (ss_mem_addr <  14'h0B80);

	// Palette D15:12 do not exist (K2GE 4-15 Table 18).  Masking on the
	// write path means every read - CPU, render and tap - sees 0 there without
	// a read-side mask, and a savestate round trip is idempotent.
	wire [15:0] pal_din = {4'h0, pa_din[11:0]};

	// The 16-bit-only rule, as a parameter.  The tap is
	// exempt: a restore must put back exactly what was saved.
	wire pal_lane_ok = ss_mem_active || (P_PAL_BYTE_WRITE != 0) || (pa_be == 2'b11);

	wire wr_pal = pa_wr && pa_pal && pal_lane_ok;
	wire wr_dpal = pa_wr && pa_dpal;
	wire wr_oam = pa_wr && pa_oam;
	wire wr_cpc = pa_wr && pa_cpc;
	wire wr_sc1 = pa_wr && pa_scr && !pa[11];
	wire wr_sc2 = pa_wr && pa_scr &&  pa[11];
	wire wr_chr = pa_wr && pa_ch;

	// Character RAM - 4096 x 16, 8 M10K

	reg  [11:0] ch_addr_b;
	wire [15:0] ch_q_a;
	wire [15:0] ch_q_b;

	cache_ram_dp_be #(
		.ADDR_WIDTH (12),
		.DATA_WIDTH (16)
	) u_chram (
		.clk_i     (clk_sys),
		.addr_a_i  (pa[12:1]),
		.wren_a_i  (wr_chr),
		.be_a_i    (pa_be),
		.wdata_a_i (pa_din),
		.q_a_o     (ch_q_a),
		.addr_b_i  (ch_addr_b),
		.wren_b_i  (1'b0),
		.be_b_i    (2'b00),
		.wdata_b_i (16'h0000),
		.q_b_o     (ch_q_b)
	);

	// Scroll plane RAMs - 1024 x 16 each, 2 M10K each

	wire [15:0] s1_q_a;
	wire [15:0] s2_q_a;

	cache_ram_dp_be #(
		.ADDR_WIDTH (10),
		.DATA_WIDTH (16)
	) u_scr1 (
		.clk_i     (clk_sys),
		.addr_a_i  (pa[10:1]),
		.wren_a_i  (wr_sc1),
		.be_a_i    (pa_be),
		.wdata_a_i (pa_din),
		.q_a_o     (s1_q_a),
		.addr_b_i  (s1_map_addr),
		.wren_b_i  (1'b0),
		.be_b_i    (2'b00),
		.wdata_b_i (16'h0000),
		.q_b_o     (s1_map_q)
	);

	cache_ram_dp_be #(
		.ADDR_WIDTH (10),
		.DATA_WIDTH (16)
	) u_scr2 (
		.clk_i     (clk_sys),
		.addr_a_i  (pa[10:1]),
		.wren_a_i  (wr_sc2),
		.be_a_i    (pa_be),
		.wdata_a_i (pa_din),
		.q_a_o     (s2_q_a),
		.addr_b_i  (s2_map_addr),
		.wren_b_i  (1'b0),
		.be_b_i    (2'b00),
		.wdata_b_i (16'h0000),
		.q_b_o     (s2_map_q)
	);

	// Sprite table - 128 x 16, 1 M10K

	wire [15:0] oam_q_a;

	cache_ram_dp_be #(
		.ADDR_WIDTH (7),
		.DATA_WIDTH (16)
	) u_oam (
		.clk_i     (clk_sys),
		.addr_a_i  (pa[7:1]),
		.wren_a_i  (wr_oam),
		.be_a_i    (pa_be),
		.wdata_a_i (pa_din),
		.q_a_o     (oam_q_a),
		.addr_b_i  (oam_addr),
		.wren_b_i  (1'b0),
		.be_b_i    (2'b00),
		.wdata_b_i (16'h0000),
		.q_b_o     (oam_q)
	);

	// Colour palette - canonical 256 x 16 (12 used), 1 M10K

	wire [15:0] pal_q_a;
	wire [15:0] pal_live_q_b;

	// The canonical RAM remains the CPU-visible K2GE palette.  Frame Boundary
	// is an explicitly user-selected display alternative: it does not delay a
	// store, change readback, or claim that the physical K2GE has this second
	// bank.
	//
	// The display bank is rebuilt during the final blank raster interval.  One
	// ISSUE cycle reads canonical port B and one WRITE cycle commits display
	// port B.  CPU writes win collisions and are mirrored only while that final
	// synchronization window is open.  The bank is therefore immutable for the
	// complete active frame.
	reg        palette_frame_q;
	reg        pal_sync_window_q;
	reg        pal_copy_active_q;
	reg        pal_copy_write_q;
	reg [7:0]  pal_copy_idx_q;

	wire normal_pal_wr = wr_pal && !ss_mem_active;
	wire [7:0] normal_pal_addr = pa[8:1];
	wire palette_sync_requested_w = palette_frame_q ||
	                                palette_frame_boundary;
	wire palette_sync_ready_w = pal_sync_window_q &&
	                            !pal_copy_active_q;
	wire mirror_pal_wr = normal_pal_wr
	                     && (pal_sync_window_q ||
	                         (palette_sync_start && palette_sync_requested_w));
	// Commit only on a controller-authorized clock.  In particular, a copier
	// parked in WRITE must not keep driving the display RAM while standby or the
	// savestate tap freezes the rest of K2GE.
	wire copy_pal_wr = run && ce && !ss_mem_active
	                   && pal_copy_active_q && pal_copy_write_q
	                   && !normal_pal_wr;

	// Saved phase is diagnostic; restore deliberately restarts a pending word
	// at ISSUE because the block RAM's registered read output is not state.
	assign palette_ss_state = {palette_frame_q, pal_sync_window_q,
	                           pal_copy_active_q, pal_copy_write_q,
	                           pal_copy_idx_q};

	always @(posedge clk_sys) begin
		if (rst) begin
			palette_frame_q  <= 1'b0;
			pal_sync_window_q <= 1'b0;
			pal_copy_active_q <= 1'b0;
			pal_copy_write_q <= 1'b0;
			pal_copy_idx_q   <= 8'd0;
		end else if (palette_ss_wren) begin
			// The menu is external configuration.  A restore happens while public
			// video is held, so applying its current request here cannot split a
			// displayed frame.
			palette_frame_q  <= palette_frame_boundary;
			pal_sync_window_q <= palette_ss_wdata[10];
			pal_copy_active_q <= palette_ss_wdata[9];
			pal_copy_write_q <= 1'b0;
			pal_copy_idx_q   <= palette_ss_wdata[7:0];
		end else if (run && ce) begin
			if (frame_start) begin
				// Disabling is always safe at a boundary. Enabling requires a
				// completed final-line sweep; a request arriving too late waits one
				// more frame instead of selecting stale display RAM.
				if (!palette_frame_boundary)
					palette_frame_q <= 1'b0;
				else if (palette_sync_ready_w)
					palette_frame_q <= 1'b1;
				pal_sync_window_q <= 1'b0;
				pal_copy_active_q <= 1'b0;
				pal_copy_write_q <= 1'b0;
			end else if (palette_sync_start && palette_sync_requested_w) begin
				pal_sync_window_q <= 1'b1;
				pal_copy_active_q <= 1'b1;
				pal_copy_write_q <= 1'b0;
				pal_copy_idx_q   <= 8'd0;

				// A write accepted on the opening edge is already mirrored.  If it
				// is entry zero, do not issue a mixed-port read of that entry.
				if (normal_pal_wr && (normal_pal_addr == 8'd0)) begin
					pal_copy_idx_q <= 8'd1;
				end
			end else if (pal_copy_active_q) begin
				if (!pal_copy_write_q) begin
					// ISSUE.  A same-entry CPU write is the authoritative value and
					// makes this entry complete without consuming the ambiguous read.
					if (normal_pal_wr && (normal_pal_addr == pal_copy_idx_q)) begin
						if (pal_copy_idx_q == 8'hFF) begin
							pal_copy_active_q <= 1'b0;
						end else begin
							pal_copy_idx_q <= pal_copy_idx_q + 8'd1;
						end
					end else begin
						pal_copy_write_q <= 1'b1;
					end
				end else if (normal_pal_wr) begin
					// WRITE collision.  A same-entry write completes the entry; a
					// different-entry write consumes the display port and retries.
					if (normal_pal_addr == pal_copy_idx_q) begin
						pal_copy_write_q <= 1'b0;
						if (pal_copy_idx_q == 8'hFF) begin
							pal_copy_active_q <= 1'b0;
						end else begin
							pal_copy_idx_q <= pal_copy_idx_q + 8'd1;
						end
					end
				end else begin
					// Normal WRITE: display port B commits pal_live_q_b below.
					pal_copy_write_q <= 1'b0;
					if (pal_copy_idx_q == 8'hFF) begin
						pal_copy_active_q <= 1'b0;
					end else begin
						pal_copy_idx_q <= pal_copy_idx_q + 8'd1;
					end
				end
			end
		end
	end

	wire [7:0] pal_live_addr_b = pal_copy_active_q
	                              ? pal_copy_idx_q : pal_rd_addr;

	cache_ram_dp_be #(
		.ADDR_WIDTH (8),
		.DATA_WIDTH (16)
	) u_pal (
		.clk_i     (clk_sys),
		.addr_a_i  (pa[8:1]),
		.wren_a_i  (wr_pal),
		.be_a_i    (pa_be),
		.wdata_a_i (pal_din),
		.q_a_o     (pal_q_a),
		.addr_b_i  (pal_live_addr_b),
		.wren_b_i  (1'b0),
		.be_b_i    (2'b00),
		.wdata_b_i (16'h0000),
		.q_b_o     (pal_live_q_b)
	);

	// Display palette - 256 x 16 (12 used), 1 M10K
	// Port A is reserved for the explicit TYPE2 savestate image at
	// 0x0C40-0x0E3F.  Port B is scanout except during the wholly blank final
	// raster interval, when CPU-write mirroring has priority over the sweep.

	wire [15:0] pal_display_q_a;
	wire [15:0] pal_display_q_b;
	wire [7:0] pal_display_addr_b = mirror_pal_wr ? normal_pal_addr
	                                  : copy_pal_wr ? pal_copy_idx_q
	                                  :               pal_rd_addr;
	wire [1:0] pal_display_be_b = mirror_pal_wr ? pa_be : 2'b11;
	wire [15:0] pal_display_din_b = mirror_pal_wr ? pal_din : pal_live_q_b;

	cache_ram_dp_be #(
		.ADDR_WIDTH (8),
		.DATA_WIDTH (16)
	) u_pal_display (
		.clk_i     (clk_sys),
		.addr_a_i  (pa_dpal_off[8:1]),
		.wren_a_i  (wr_dpal),
		.be_a_i    (pa_be),
		.wdata_a_i (pal_din),
		.q_a_o     (pal_display_q_a),
		.addr_b_i  (pal_display_addr_b),
		.wren_b_i  (mirror_pal_wr || copy_pal_wr),
		.be_b_i    (pal_display_be_b),
		.wdata_b_i (pal_display_din_b),
		.q_b_o     (pal_display_q_b)
	);

	assign pal_rd_data = palette_frame_q ? pal_display_q_b : pal_live_q_b;

`ifndef SYNTHESIS
	always @(posedge clk_sys) begin
		if (!rst && run && ce && frame_start && pal_copy_active_q)
			$fatal(1, "k2ge_vram: palette copy did not finish before frame_start");
	end
`endif

	// Sprite CP.C - 64 x 4 flops, held as one packed vector
	// Entry n lives at byte address 0x8C00 + n, so the even lane carries entry
	// {pa[5:1],0} and the odd lane entry {pa[5:1],1}.  Bit offset = n * 4.

	reg [255:0] cpc_bits;

	wire [7:0] cpc_bit_e = {pa[5:1], 3'b000};   // ({pa[5:1],1'b0}) * 4
	wire [7:0] cpc_bit_o = {pa[5:1], 3'b100};   // ({pa[5:1],1'b1}) * 4

	always @(posedge clk_sys) begin
		if (rst) begin
			cpc_bits <= 256'd0;
		end else if (wr_cpc) begin
			if (pa_be[0]) cpc_bits[cpc_bit_e +: 4] <= pa_din[3:0];
			if (pa_be[1]) cpc_bits[cpc_bit_o +: 4] <= pa_din[11:8];
		end
	end

	// The sprite evaluator reads a nibble in the same cycle it reads the
	// attribute word.  A 64:1 four-bit mux, which is why this is flops.
	assign cpc_q = cpc_bits[{cpc_idx, 2'b00} +: 4];

	// Readback - one registered decode, one mux of registered RAM outputs
	// The region decode is registered at the
	// sampling tick so the readback is a clean 8:1 mux and never a path from
	// cpu_a into a RAM address.  `sel_*` is the decode of whatever address the
	// RAMs are answering right now - one cycle behind `pa` - so it serves the
	// CPU (whose address is held for the dot) and the tap (whose address can
	// move every cycle) with the same logic.

	reg       sel_pal;
	reg       sel_dpal;
	reg       sel_oam;
	reg       sel_cpc;
	reg       sel_scr;
	reg       sel_ch;
	reg       sel_lb;
	reg       sel_plane;
	reg       sel_lane;
	reg [7:0] sel_cpc_e;
	reg [7:0] sel_cpc_o;

	always @(posedge clk_sys) begin
		if (rst) begin
			sel_pal   <= 1'b0;
			sel_dpal  <= 1'b0;
			sel_oam   <= 1'b0;
			sel_cpc   <= 1'b0;
			sel_scr   <= 1'b0;
			sel_ch    <= 1'b0;
			sel_lb    <= 1'b0;
			sel_plane <= 1'b0;
			sel_lane  <= 1'b0;
			sel_cpc_e <= 8'd0;
			sel_cpc_o <= 8'd0;
		end else begin
			sel_pal   <= pa_pal && !pa_lb;
			sel_dpal  <= pa_dpal;
			sel_oam   <= pa_oam && !pa_lb;
			sel_cpc   <= pa_cpc && !pa_lb;
			sel_scr   <= pa_scr && !pa_lb;
			sel_ch    <= pa_ch  && !pa_lb;
			sel_lb    <= pa_lb;
			sel_plane <= pa[11];
			sel_lane  <= pa[0];
			sel_cpc_e <= cpc_bit_e;
			sel_cpc_o <= cpc_bit_o;
		end
	end

	reg [15:0] rd_word;

	always @* begin
		if (sel_ch)       rd_word = ch_q_a;
		else if (sel_scr) rd_word = sel_plane ? s2_q_a : s1_q_a;
		else if (sel_oam) rd_word = oam_q_a;
		else if (sel_pal) rd_word = pal_q_a;
		else if (sel_dpal) rd_word = pal_display_q_a;
		else if (sel_cpc) rd_word = {4'h0, cpc_bits[sel_cpc_o +: 4],
		                             4'h0, cpc_bits[sel_cpc_e +: 4]};
		else              rd_word = 16'h0000;
	end

	// Unmapped addresses read 0.  The chip parent selects between this module
	// and k2ge_mmr with `cpu_hit`, so no gating is needed here - and gating on
	// a live signal while the data comes from the held address would be wrong
	// anyway.
	//
	// The one exception is the mono hide (see the region decode): `rd_word`
	// still answers the palette and CP.C windows because the savestate tap
	// reads through it, so the CPU-facing copy masks them instead.  `sel_pal`
	// and `sel_cpc` are the REGISTERED decode of the same held address the
	// data came from, so this mask has the same phase as the data and does not
	// reintroduce the live-signal problem the paragraph above rules out.
	wire cpu_region_hidden = mono_strap && (sel_pal || sel_cpc);

	assign cpu_dout = cpu_region_hidden ? 16'h0000 : rd_word;

	// Savestate tap read path
	// Address at cycle T, RAM answers at T+1, byte registered at T+1 and
	// readable from T+2.  The engine allows at least seven clocks per byte.
	// Deliberately not gated by `ce`: the tap runs while the chip is frozen.

	reg [7:0] ss_rdata_q;
	reg       ss_rden_q;

	wire [7:0] tap_byte = sel_lb ? ss_lb_rdata
	                             : (sel_lane ? rd_word[15:8] : rd_word[7:0]);

	always @(posedge clk_sys) begin
		if (rst) begin
			ss_rden_q  <= 1'b0;
			ss_rdata_q <= 8'd0;
		end else begin
			ss_rden_q <= ss_mem_active && ss_mem_rden;
			if (ss_rden_q) ss_rdata_q <= tap_byte;
		end
	end

	assign ss_mem_rdata = ss_rdata_q;

	assign ss_lb_sel   = pa_lb;
	assign ss_lb_addr  = ss_mem_addr[9:0] - 10'h100;   // 0x0900 -> 0
	assign ss_lb_wdata = ss_mem_wdata;
	assign ss_lb_wren  = pa_lb && ss_mem_wren;

	// The four-slot rotation
	// `cyc_q` names the clk_sys cycle inside the current dot.  ce_6m144 marks
	// cycle 0, so forcing the counter to 1 on that tick both advances it and
	// re-locks the phase every dot; `line_start` does the same thing at dot 0.
	// The counter therefore cannot drift, and after reset it locks within one
	// dot.
	//
	// The slot phase needs no savestate restore path for exactly that reason:
	// it is a function of the dot clock, and one dot after any restore it is
	// back in step.

	reg [2:0] cyc_q;

	always @(posedge clk_sys) begin
		if (rst)                                cyc_q <= 3'd0;
		else if (ce && (ce_6m144 || line_start)) cyc_q <= 3'd1;
		else if (ce)                             cyc_q <= cyc_q + 3'd1;
	end

	wire [1:0] slot  = cyc_q[2:1];
	wire       drive = ~cyc_q[0];               // first cycle of a slot

	wire s1_ch_slot = ce && drive && (slot == 2'd0);
	wire ob_ch_slot = ce && drive && (slot[0] == 1'b1);     // slots 1 and 3
	wire s2_ch_slot = ce && drive && (slot == 2'd2);

	assign s1_ch_ack = s1_ch_slot && s1_ch_req;
	assign s2_ch_ack = s2_ch_slot && s2_ch_req;
	assign ob_ch_ack = ob_ch_slot && ob_ch_req;

	// The character RAM's port B address.  A registered value out of one
	// consumer through a 3:1 mux into a RAM address - the shortest path the
	// arbiter can be built from, and stable across both cycles of the slot
	// because `slot` only moves on even cycles.
	always @* begin
		case (slot)
			2'd0:    ch_addr_b = s1_ch_addr;
			2'd2:    ch_addr_b = s2_ch_addr;
			default: ch_addr_b = ob_ch_addr;
		endcase
	end

	// Capture pipeline: ack in the drive cycle, RAM data in the capture cycle,
	// valid one cycle after that.
	reg        s1_pend, s2_pend, ob_pend;
	reg        s1_v_q,  s2_v_q,  ob_v_q;
	reg [15:0] s1_d_q,  s2_d_q,  ob_d_q;

	always @(posedge clk_sys) begin
		if (rst) begin
			s1_pend <= 1'b0;
			s2_pend <= 1'b0;
			ob_pend <= 1'b0;
			s1_v_q  <= 1'b0;
			s2_v_q  <= 1'b0;
			ob_v_q  <= 1'b0;
			s1_d_q  <= 16'd0;
			s2_d_q  <= 16'd0;
			ob_d_q  <= 16'd0;
		end else if (ce) begin
			s1_pend <= s1_ch_ack;
			s2_pend <= s2_ch_ack;
			ob_pend <= ob_ch_ack;

			s1_v_q  <= s1_pend;
			s2_v_q  <= s2_pend;
			ob_v_q  <= ob_pend;

			if (s1_pend) s1_d_q <= ch_q_b;
			if (s2_pend) s2_d_q <= ch_q_b;
			if (ob_pend) ob_d_q <= ch_q_b;
		end
	end

	assign s1_ch_valid = s1_v_q;
	assign s2_ch_valid = s2_v_q;
	assign ob_ch_valid = ob_v_q;
	assign s1_ch_data  = s1_d_q;
	assign s2_ch_data  = s2_d_q;
	assign ob_ch_data  = ob_d_q;

	// CPU wait states.
	// "Sprite VRAM, scroll VRAM, and character RAM read/write access invokes
	// the adjustment circuitry.  (Accessing the registers does not have an
	// effect.)" (K1GE section 4 / K2GE section 6).  Palette RAM is excluded
	// on the strength of (K2GE 4-15 caution 5); CP.C is excluded because the
	// sentence names three regions and CP.C is not one of them, which is a
	// judgement call.
	//
	// While the composition pass owns character/scroll/sprite VRAM, a CPU
	// cycle into those regions waits for it.  This is a LEVEL, not a one-dot
	// pulse: the fabric samples it in T1 to size the cycle, so it has to be
	// true for the whole state.
	//
	// LDIR into character RAM pays the wait and still costs 7n+1, because LDIR
	// spends only 4 of its 7 states per iteration on the bus and a 2-state wait
	// fits in the slack with a state to spare.  That same slack bounds the wait
	// from above: 4 or more would exceed it and lengthen the instruction.
	//
	// All three regions named by the manuals pay the same +1.96 states/access
	// -- character writes, character reads, sprite-table writes and scroll-map
	// writes -- with palette and work RAM flat.
	wire wait_region = hit_ch || hit_scr || (P_VRAM_WAIT_OAM && hit_oam);

	assign cpu_wait_states = (cpu_cs && wait_region
	                          && render_owns) ? P_VRAM_CPU_WAIT : 3'd0;

	// Deliberately unread inputs
	// Reads have no side effects here, so cpu_rd is not needed.  Only twelve
	// bits of a palette write survive, and the tap's upper address bits are
	// decoded rather than carried into the line-buffer offset.
	wire unused_ok = &{1'b0, cpu_rd, pa_din[15:12], ss_mem_addr[13:10], 1'b0};

endmodule
