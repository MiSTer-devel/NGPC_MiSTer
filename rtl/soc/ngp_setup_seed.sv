// Copyright (c) 2026 Jamie Blanks

// MiSTer-facing BIOS setup seed.
//
// This is not console hardware. It is a small compatibility bridge: let the
// original BIOS finish its cold initialization, pause it in standby, then
// create the same setup-valid bytes that its first-boot UI would have
// committed. The checksum rule is decoded directly from both BIOS images:
//
//   NGPC = 0x6F87 + 0x6C25..0x6C2B + 0x6F94
//   NGP  = 0x6F87 + 0x6C25..0x6C2B
//
// The initial seed fixes 0x6C25 at 0xDC and the other six interrupt shadows at
// zero, so the function reduces to 0xDC + language + optional palette. It is
// still kept as one named function so initial seeding and later OSD updates
// cannot acquire different arithmetic.
//
// MiSTer hps_io.RTC is already local-time BCD in MSM6242B order. No epoch
// conversion is performed here. The payload is reordered into ngp_rtc's four
// existing savestate words while the machine is paused. Year modulo four is
// the low two bits of (decimal tens * 2 + decimal ones); this uses one short
// adder and no division or modulus.
//
// The BIOS resume request is deliberately not seeded (P_ALLOW_RESUME_SEED = 0).
// Seeding User_Answer bit 7 at 0x6F86, with the last-run catalogue,
// sub-catalogue and title the BIOS compares against the cartridge, does skip the
// eye-catch, but it makes the BIOS launch the game with User_Boot bit 5 (Resume)
// set, and that bit is not decoration:
//
//   "check User_Boot -- Alarm startup (bit7) -> alarm handler; Resume (bit5)
//    -> validate saved 0x4000-0x5FFF data (checksum is the game's job); else
//    normal startup" (SysPro p.10)
//
//   "game sets User_Answer bit7; on power-off the work RAM 0x4000-0x5FFF is
//    preserved 'as is' (battery-backed); on next power-on with the same
//    software ... User_Boot bit5 is set and the system eye-catch is skipped"
//    (SysWork p.6-7 / SysPro p.17)
//
// The request is the game's to make, and on a cold boot the game has never run
// and the work RAM it is told to restore was just cleared, so a seeded resume
// tells every cartridge it is resuming a session that does not exist.  Games act
// on it: Faselei! and Pocket Tennis Color both accept the cleared RAM as a valid
// session, return early from their init, and draw over tilemaps they never
// filled.
//
// The mechanism is kept and gated rather than deleted: P_ALLOW_RESUME_SEED goes
// to 1 once work RAM 0x4000-0x5FFF really does survive a power cycle. With it at
// 0 the seed writes User_Answer = 0x00.

module ngp_setup_seed
#(
	// 1 = allow the BIOS resume request to be seeded on a cold boot. See the
	// header: this is only honest once work RAM 0x4000-0x5FFF survives a power
	// cycle, which nothing in this design provides yet.
	parameter P_ALLOW_RESUME_SEED = 1'b0
)
(
	input  wire        clk,
	input  wire        reset,

	// The BIOS has written its power-off latch and entered HALT.
	input  wire        setup_ready,
	input  wire        mono,

	// OSD values. Language is ordered English/Japanese for friendly defaults;
	// the BIOS byte is ordered Japanese=0/English=1, so it is inverted here.
	input  wire        osd_language_japanese,
	input  wire [2:0]  osd_palette,
	input  wire        use_hps_rtc,
	input  wire [64:0] hps_rtc,

	// The BIOS resume path skips both eye-catch animations, but it validates
	// these cartridge-header mirrors before it launches. Never assert the
	// resume request without a complete header captured by ngp_cart_rom.
	input  wire        skip_bios_animation,
	input  wire        cart_header_valid,
	input  wire [15:0] cart_catalog,
	input  wire [7:0]  cart_subcatalog,
	input  wire [95:0] cart_title,

	// Existing pause tree and savestate memory/register paths. The seed engine
	// adds no RAM port and owns these only while seed_busy is asserted.
	output wire        pause_req,
	input  wire        pause_ready,
	output reg  [9:0]  ss_bus_adr,
	output reg  [63:0] ss_bus_din,
	output reg         ss_bus_wren,
	output reg  [1:0]  ss_mem_type,
	output reg         ss_mem_active,
	output reg  [13:0] ss_mem_addr,
	output reg  [7:0]  ss_mem_wdata,
	output reg         ss_mem_wren,
	output reg         ss_mem_rden,

	output wire        seed_busy,
	output reg         seed_done
);

	localparam [2:0] ST_IDLE      = 3'd0;
	localparam [2:0] ST_WAIT      = 3'd1;
	localparam [2:0] ST_MEM       = 3'd2;
	localparam [2:0] ST_RTC_DATE  = 3'd3;
	localparam [2:0] ST_RTC_TIME  = 3'd4;
	localparam [2:0] ST_RTC_WDAY  = 3'd5;
	localparam [2:0] ST_RTC_ALARM = 3'd6;

	localparam [13:0] W_CSUM_LO = 14'h2C14; // CPU 0x6C14 - 0x4000
	localparam [13:0] W_CSUM_HI = 14'h2C15;
	localparam [13:0] W_LAST_CATALOG = 14'h2C04;
	localparam [13:0] W_LAST_SUBCAT  = 14'h2C06;
	localparam [13:0] W_LAST_TITLE   = 14'h2C08;
	localparam [13:0] W_INTE45  = 14'h2C25;
	localparam [13:0] W_MARK_P  = 14'h2E95;
	localparam [13:0] W_MARK_N  = 14'h2E96;
	localparam [13:0] W_STATE   = 14'h2F83;
	localparam [13:0] W_USER_ANSWER = 14'h2F86;
	localparam [13:0] W_LANG    = 14'h2F87;
	localparam [13:0] W_PALETTE = 14'h2F94;

	localparam [9:0] SS_RTC_DATE  = 10'd88;
	localparam [9:0] SS_RTC_TIME  = 10'd89;
	localparam [9:0] SS_RTC_WDAY  = 10'd90;
	localparam [9:0] SS_RTC_ALARM = 10'd91;

	function automatic [7:0] bios_language(input japanese);
	begin
		bios_language = japanese ? 8'h00 : 8'h01;
	end
	endfunction

	function automatic [7:0] bios_palette(input [2:0] palette);
	begin
		case (palette)
			3'd0: bios_palette = 8'h00; // black and white
			3'd1: bios_palette = 8'h01; // red
			3'd2: bios_palette = 8'h02; // green
			3'd3: bios_palette = 8'h03; // blue
			3'd4: bios_palette = 8'h04; // classic
			default: bios_palette = 8'h00;
		endcase
	end
	endfunction

	function automatic [7:0] bios_initial_checksum
	(
		input       mono_bios,
		input [7:0] language,
		input [7:0] palette
	);
		reg [9:0] total;
	begin
		// 0x6C25=0xDC; 0x6C26-0x6C2B=0. The NGPC BIOS alone adds
		// 0x6F94. Assignment of total[7:0] makes the 8-bit wrap explicit.
		total = {2'b00, 8'hDC} + {2'b00, language};
		if (!mono_bios) begin
			total = total + {2'b00, palette};
		end
		bios_initial_checksum = total[7:0];
	end
	endfunction

	function automatic [1:0] bcd_year_mod4(input [7:0] bcd_year);
		reg [1:0] tens_mod4;
		reg [1:0] ones_mod4;
	begin
		// 10 mod 4 = 2. The explicit BCD cases also give malformed HPS
		// digits a deterministic zero contribution.
		case (bcd_year[7:4])
			4'd0, 4'd2, 4'd4, 4'd6, 4'd8: tens_mod4 = 2'd0;
			4'd1, 4'd3, 4'd5, 4'd7, 4'd9: tens_mod4 = 2'd2;
			default:                      tens_mod4 = 2'd0;
		endcase
		case (bcd_year[3:0])
			4'd0, 4'd4, 4'd8: ones_mod4 = 2'd0;
			4'd1, 4'd5, 4'd9: ones_mod4 = 2'd1;
			4'd2, 4'd6:       ones_mod4 = 2'd2;
			4'd3, 4'd7:       ones_mod4 = 2'd3;
			default:          ones_mod4 = 2'd0;
		endcase
		bcd_year_mod4 = tens_mod4 + ones_mod4;
	end
	endfunction

	wire [7:0] live_language = bios_language(osd_language_japanese);
	wire [7:0] live_palette  = bios_palette(osd_palette);
	// Gated by P_ALLOW_RESUME_SEED -- see the header. The two runtime terms are
	// kept so the condition still reads as the feature it is, and so the day a
	// preserved session exists only the parameter has to move.
	wire       live_resume   = (P_ALLOW_RESUME_SEED != 0) &&
	                           skip_bios_animation && cart_header_valid;

	reg [2:0] state;
	reg [4:0] mem_step;
	reg       need_seed;

	reg       mono_q;
	reg [7:0] language_q;
	reg [7:0] palette_q;
	reg [7:0] checksum_q;
	reg       use_rtc_q;
	reg       resume_q;
	reg [15:0] catalog_q;
	reg [7:0]  subcatalog_q;
	reg [95:0] title_q;

	reg [7:0] rtc_year_q;
	reg [7:0] rtc_month_q;
	reg [7:0] rtc_day_q;
	reg [7:0] rtc_hour_q;
	reg [7:0] rtc_minute_q;
	reg [7:0] rtc_second_q;
	reg [2:0] rtc_weekday_q;
	reg [1:0] rtc_leap_q;
	// This capture intentionally runs through machine reset. Main may deliver
	// its startup RTC packet while the BIOS and work RAM are still held reset;
	// clearing the toggle history there would lose the only completion edge.
	reg [63:0] rtc_shadow_q;
	reg        rtc_toggle_q;

	initial begin
		rtc_shadow_q = {8'h40, 8'h06, 8'h00, 8'h01,
		                8'h01, 8'h00, 8'h00, 8'h00};
		rtc_toggle_q = 1'b0;
	end

	reg       applied_mono_q;
	reg [7:0] applied_language_q;
	reg [7:0] applied_palette_q;
	reg       applied_rtc_q;
	reg       applied_resume_q;
	reg [15:0] applied_catalog_q;
	reg [7:0]  applied_subcatalog_q;
	reg [95:0] applied_title_q;

	wire settings_changed = (mono != applied_mono_q) ||
	                        (live_language != applied_language_q) ||
	                        (live_palette != applied_palette_q) ||
	                        (use_hps_rtc != applied_rtc_q) ||
	                        (live_resume != applied_resume_q) ||
	                        (live_resume &&
	                         ((cart_catalog != applied_catalog_q) ||
	                          (cart_subcatalog != applied_subcatalog_q) ||
	                          (cart_title != applied_title_q)));

	wire latched_still_current = (mono == mono_q) &&
	                             (live_language == language_q) &&
	                             (live_palette == palette_q) &&
	                             (use_hps_rtc == use_rtc_q) &&
	                             (live_resume == resume_q) &&
	                             (!resume_q ||
	                              ((cart_catalog == catalog_q) &&
	                               (cart_subcatalog == subcatalog_q) &&
	                               (cart_title == title_q)));

	wire [7:0] rtc_wday = {2'b00, rtc_leap_q, 1'b0, rtc_weekday_q};
	wire       rtc_update = (hps_rtc[64] != rtc_toggle_q);
	wire [63:0] rtc_seed_value = rtc_update ? hps_rtc[63:0] : rtc_shadow_q;

	// hps_io writes RTC in four 16-bit pieces and toggles bit 64 only after
	// the complete packet is visible. This block therefore never captures an
	// in-flight mixture of old and new fields, including during machine reset.
	always @(posedge clk) begin
		if (rtc_update) begin
			rtc_shadow_q <= hps_rtc[63:0];
			rtc_toggle_q <= hps_rtc[64];
		end
	end

	assign pause_req = (state != ST_IDLE);
	assign seed_busy = (state != ST_IDLE);

	// One byte per clock on the existing work-RAM port B. W_STATE is volatile
	// BIOS workspace, not part of the checksummed setup record. Blank-RAM cold
	// initialization leaves its bit 4 set to request the first-boot UI; clear
	// the byte while the BIOS is paused in that same standby state. User_Answer
	// bit 7 is the BIOS-owned resume request. The matching cartridge identity is
	// written only when a complete header was captured; otherwise the request is
	// explicitly cleared. The BIOS clears bit 3 itself at every power-on and the
	// other known runtime bits are inactive in standby. The checksum remains the
	// final write and therefore the commit point of the persistent setup record.
	always @* begin
		ss_bus_adr    = 10'd0;
		ss_bus_din    = 64'd0;
		ss_bus_wren   = 1'b0;
		ss_mem_type   = 2'd0;
		ss_mem_active = 1'b0;
		ss_mem_addr   = 14'd0;
		ss_mem_wdata  = 8'd0;
		ss_mem_wren   = 1'b0;
		ss_mem_rden   = 1'b0;

		if (state == ST_MEM) begin
			ss_mem_active = 1'b1;
			ss_mem_wren   = 1'b1;
			case (mem_step)
				5'd0:  begin ss_mem_addr = W_INTE45;               ss_mem_wdata = 8'hDC; end
				5'd1:  begin ss_mem_addr = W_INTE45 + 14'd1;       ss_mem_wdata = 8'h00; end
				5'd2:  begin ss_mem_addr = W_INTE45 + 14'd2;       ss_mem_wdata = 8'h00; end
				5'd3:  begin ss_mem_addr = W_INTE45 + 14'd3;       ss_mem_wdata = 8'h00; end
				5'd4:  begin ss_mem_addr = W_INTE45 + 14'd4;       ss_mem_wdata = 8'h00; end
				5'd5:  begin ss_mem_addr = W_INTE45 + 14'd5;       ss_mem_wdata = 8'h00; end
				5'd6:  begin ss_mem_addr = W_INTE45 + 14'd6;       ss_mem_wdata = 8'h00; end
				5'd7:  begin ss_mem_addr = W_LANG;                 ss_mem_wdata = language_q; end
				5'd8:  begin
					ss_mem_addr  = W_PALETTE;
					ss_mem_wdata = palette_q;
					ss_mem_wren  = !mono_q;
				end
				5'd9:  begin
					ss_mem_addr  = W_MARK_P;
					ss_mem_wdata = 8'h50;
					ss_mem_wren  = !mono_q;
				end
				5'd10: begin
					ss_mem_addr  = W_MARK_N;
					ss_mem_wdata = 8'h4E;
					ss_mem_wren  = !mono_q;
				end
				5'd11: begin ss_mem_addr = W_STATE;                ss_mem_wdata = 8'h00; end
				5'd12: begin ss_mem_addr = W_LAST_CATALOG;         ss_mem_wdata = catalog_q[7:0];  ss_mem_wren = resume_q; end
				5'd13: begin ss_mem_addr = W_LAST_CATALOG + 14'd1; ss_mem_wdata = catalog_q[15:8]; ss_mem_wren = resume_q; end
				5'd14: begin ss_mem_addr = W_LAST_SUBCAT;          ss_mem_wdata = subcatalog_q;    ss_mem_wren = resume_q; end
				5'd15: begin ss_mem_addr = W_LAST_TITLE;           ss_mem_wdata = title_q[7:0];    ss_mem_wren = resume_q; end
				5'd16: begin ss_mem_addr = W_LAST_TITLE + 14'd1;   ss_mem_wdata = title_q[15:8];   ss_mem_wren = resume_q; end
				5'd17: begin ss_mem_addr = W_LAST_TITLE + 14'd2;   ss_mem_wdata = title_q[23:16];  ss_mem_wren = resume_q; end
				5'd18: begin ss_mem_addr = W_LAST_TITLE + 14'd3;   ss_mem_wdata = title_q[31:24];  ss_mem_wren = resume_q; end
				5'd19: begin ss_mem_addr = W_LAST_TITLE + 14'd4;   ss_mem_wdata = title_q[39:32];  ss_mem_wren = resume_q; end
				5'd20: begin ss_mem_addr = W_LAST_TITLE + 14'd5;   ss_mem_wdata = title_q[47:40];  ss_mem_wren = resume_q; end
				5'd21: begin ss_mem_addr = W_LAST_TITLE + 14'd6;   ss_mem_wdata = title_q[55:48];  ss_mem_wren = resume_q; end
				5'd22: begin ss_mem_addr = W_LAST_TITLE + 14'd7;   ss_mem_wdata = title_q[63:56];  ss_mem_wren = resume_q; end
				5'd23: begin ss_mem_addr = W_LAST_TITLE + 14'd8;   ss_mem_wdata = title_q[71:64];  ss_mem_wren = resume_q; end
				5'd24: begin ss_mem_addr = W_LAST_TITLE + 14'd9;   ss_mem_wdata = title_q[79:72];  ss_mem_wren = resume_q; end
				5'd25: begin ss_mem_addr = W_LAST_TITLE + 14'd10;  ss_mem_wdata = title_q[87:80];  ss_mem_wren = resume_q; end
				5'd26: begin ss_mem_addr = W_LAST_TITLE + 14'd11;  ss_mem_wdata = title_q[95:88];  ss_mem_wren = resume_q; end
				5'd27: begin ss_mem_addr = W_USER_ANSWER;          ss_mem_wdata = resume_q ? 8'h80 : 8'h00; end
				5'd28: begin ss_mem_addr = W_CSUM_HI;              ss_mem_wdata = 8'h00; end
				5'd29: begin ss_mem_addr = W_CSUM_LO;              ss_mem_wdata = checksum_q; end
				default: ss_mem_wren = 1'b0;
			endcase
		end

		case (state)
			ST_RTC_DATE: begin
				ss_bus_adr  = SS_RTC_DATE;
				// Bit 1 stays clear (alarm disabled). Bit 0 matches the BIOS's
				// post-time-set control idiom; it does not gate this RTC counter.
				ss_bus_din  = {32'd0, 8'h01, rtc_year_q, rtc_month_q, rtc_day_q};
				ss_bus_wren = 1'b1;
			end
			ST_RTC_TIME: begin
				ss_bus_adr  = SS_RTC_TIME;
				ss_bus_din  = {32'd0, 8'h00, rtc_hour_q, rtc_minute_q, rtc_second_q};
				ss_bus_wren = 1'b1;
			end
			ST_RTC_WDAY: begin
				ss_bus_adr  = SS_RTC_WDAY;
				ss_bus_din  = {32'd0, rtc_wday, 8'h00, 16'h0000};
				ss_bus_wren = 1'b1;
			end
			ST_RTC_ALARM: begin
				ss_bus_adr  = SS_RTC_ALARM;
				ss_bus_din  = 64'd0;
				ss_bus_wren = 1'b1;
			end
			default: ;
		endcase
	end

	always @(posedge clk) begin
		if (reset) begin
			state              <= ST_IDLE;
			mem_step           <= 5'd0;
			need_seed          <= 1'b1;
			seed_done          <= 1'b0;
			mono_q             <= 1'b0;
			language_q         <= 8'h01;
			palette_q          <= 8'h00;
			checksum_q         <= 8'hDD;
			use_rtc_q          <= 1'b1;
			resume_q           <= 1'b0;
			catalog_q          <= 16'd0;
			subcatalog_q       <= 8'd0;
			title_q            <= 96'd0;
			rtc_year_q         <= 8'h00;
			rtc_month_q        <= 8'h01;
			rtc_day_q          <= 8'h01;
			rtc_hour_q         <= 8'h00;
			rtc_minute_q       <= 8'h00;
			rtc_second_q       <= 8'h00;
			rtc_weekday_q      <= 3'd6;
			rtc_leap_q         <= 2'd0;
			applied_mono_q     <= 1'b0;
			applied_language_q <= 8'h01;
			applied_palette_q  <= 8'h00;
			applied_rtc_q      <= 1'b1;
			applied_resume_q   <= 1'b0;
			applied_catalog_q  <= 16'd0;
			applied_subcatalog_q <= 8'd0;
			applied_title_q    <= 96'd0;
		end else begin
			seed_done <= 1'b0;

			if ((state == ST_IDLE) && settings_changed) begin
				need_seed <= 1'b1;
			end

			case (state)
				ST_IDLE: begin
					if (setup_ready && need_seed) begin
						mono_q        <= mono;
						language_q    <= live_language;
						palette_q     <= live_palette;
						checksum_q    <= bios_initial_checksum(mono, live_language, live_palette);
						use_rtc_q     <= use_hps_rtc;
						resume_q      <= live_resume;
						catalog_q     <= cart_catalog;
						subcatalog_q  <= cart_subcatalog;
						title_q       <= cart_title;
						rtc_year_q    <= rtc_seed_value[47:40];
						rtc_month_q   <= rtc_seed_value[39:32];
						rtc_day_q     <= rtc_seed_value[31:24];
						rtc_hour_q    <= rtc_seed_value[23:16];
						rtc_minute_q  <= rtc_seed_value[15:8];
						rtc_second_q  <= rtc_seed_value[7:0];
						rtc_weekday_q <= rtc_seed_value[50:48];
						rtc_leap_q    <= bcd_year_mod4(rtc_seed_value[47:40]);
						state         <= ST_WAIT;
					end
				end

				ST_WAIT: begin
					if (pause_ready) begin
						mem_step <= 5'd0;
						state    <= ST_MEM;
					end
				end

				ST_MEM: begin
					if (mem_step == 5'd29) begin
						if (use_rtc_q) begin
							state <= ST_RTC_DATE;
						end else begin
							state              <= ST_IDLE;
							seed_done          <= 1'b1;
							need_seed          <= !latched_still_current;
							applied_mono_q     <= mono_q;
							applied_language_q <= language_q;
							applied_palette_q  <= palette_q;
							applied_rtc_q      <= use_rtc_q;
							applied_resume_q   <= resume_q;
							applied_catalog_q  <= catalog_q;
							applied_subcatalog_q <= subcatalog_q;
							applied_title_q    <= title_q;
						end
					end else begin
						mem_step <= mem_step + 5'd1;
					end
				end

				ST_RTC_DATE:  state <= ST_RTC_TIME;
				ST_RTC_TIME:  state <= ST_RTC_WDAY;
				ST_RTC_WDAY:  state <= ST_RTC_ALARM;
				ST_RTC_ALARM: begin
					state              <= ST_IDLE;
					seed_done          <= 1'b1;
					need_seed          <= !latched_still_current;
					applied_mono_q     <= mono_q;
					applied_language_q <= language_q;
					applied_palette_q  <= palette_q;
					applied_rtc_q      <= use_rtc_q;
					applied_resume_q   <= resume_q;
					applied_catalog_q  <= catalog_q;
					applied_subcatalog_q <= subcatalog_q;
					applied_title_q    <= title_q;
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

	// HPS flags and the upper weekday bits have no NGPC equivalent.
	wire unused_ok = &{1'b0, rtc_seed_value[63:51]};

endmodule
