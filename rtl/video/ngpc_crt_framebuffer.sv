// Copyright (c) 2026 Jamie Blanks

// Tear-free NGPC presentation store and frame-exact 262-line CRT raster.
//
// The K1GE/K2GE continues to run at its native 515 x 199-dot cadence. One
// native frame is 515 * 199 * 8 = 819,880 clk_sys clocks. The public raster
// below has the same exact period: 180 short lines of 3,129 clocks and 82 long
// lines of 3,130 clocks. All 152 visible lines are short; the 82 compensating
// clocks are placed on vertical-blank lines 152..233, followed by 28 short
// blank lines before the next picture. This avoids a horizontal-period change
// near the middle of the active picture while preserving exact frame lock.
// The extra raw clock is after the last blanking sample, so active pixels
// retain a uniform clk_sys/8 service cadence.
//
// Two RGB888 presentation banks carry strict ping-pong ownership. Native
// RGB444 is expanded by nibble replication before it enters either bank:
//
//   displayed  read for the whole public frame
//   write      built, then held complete until the next public frame wrap
//
// The roles never alias. A bank becomes displayed only at public frame wrap;
// completion cannot tear the current scanout. Exceptional holds park the K2GE
// at its native frame boundary and release it at the public frame boundary, so
// the completed bank always publishes before the next native write begins.
//
// A hold is exceptional. Reset/startup keeps the machine in reset until a
// public frame wrap; it must not park the CPU between reset-vector fetch and
// first prefetch. External savestate activity, public underflow, producer
// overflow, or an incomplete native capture uses the ordered pause chain and
// releases after both drain completion and public frame wrap. The public
// raster never stops, and the last displayed complete frame repeats.

module ngpc_crt_framebuffer
(
	input  wire        clk_sys,
	input  wire        rst,

	// Native LCD pads. RGB is the final K1GE/K2GE RGB444 stream.
	input  wire        lcd_dclk_ce,
	input  wire [3:0]  lcd_r,
	input  wire [3:0]  lcd_g,
	input  wire [3:0]  lcd_b,
	input  wire        lcd_de,
	input  wire        lcd_sp,

	// Existing machine pause chain. An external request is held across its
	// release until the public raster reaches frame start.
	input  wire        external_pause_req,
	input  wire        machine_pause_ready,
	input  wire        loading_savestate,
	output wire        video_pause_req,
	output wire        video_reset_hold,

	// Presentation-only tint. Zero is the colour-core bypass.
	input  wire [2:0]  tint,
	// Approximate the reflective LCD's optical response. This is presentation
	// state only: canonical K1GE/K2GE palette writes/readback remain immediate;
	// an optional display bank is selected upstream at K2GE's palette lookup.
	input  wire        lcd_persistence,

	// Framework video bus. Syncs are active high.
	output wire        ce_pixel,
	output wire [7:0]  vga_r,
	output wire [7:0]  vga_g,
	output wire [7:0]  vga_b,
	output wire        vga_de,
	output wire        vga_hbl,
	output wire        vga_vbl,
	output wire        vga_hs,
	output wire        vga_vs
);

	localparam [14:0] FRAME_LAST_ADDR = 15'd24319;

	// Public horizontal samples at 6.144 MHz:
	//   29 sync + 111 back porch + 160 active + 91 front porch = 391.
	// The native pixels are emitted one-for-one. The framework therefore sees
	// the original 160-pixel width; the rest of the CRT line is true blanking.
	//
	// A CRT phases its sweep from the sync leading edge, so the 160-dot island
	// must sit where RS-170A puts the centre of active video -- not in the
	// middle of whatever is left after sync. RS-170A runs active video from
	// 9.4 us to 62.06 us after that edge, centred at 35.73 us, i.e. 0.5621 of
	// the 63.556 us line. 0.5621 * 391 = 219.8 samples, so the 160-dot window
	// starts at 219.8 - 80 = 139.8 -> 140. The active window is deliberately
	// late in the line because the standard back porch (4.7 us) is far longer
	// than the standard front porch (1.5 us).
	//
	// The former 132 centred the island in the 359 post-sync samples (100 back
	// / 99 front), which ignored that asymmetry and left the picture 8 samples
	// -- 1.30 us, about 2.0% of a line -- left of where a standard picture
	// lands. Confirmed on a Sony consumer set in
	// references/artifacts/IMG_1658.jpg: after correcting the photograph for
	// perspective, the picture centre sat 4.7% of the visible width left of
	// the tube centre. Any residual after this change is the set's own H-phase
	// adjustment, not the raster.
	//
	// The sync pulse is 4.7 us +/- 0.1 in RS-170A. At 6.144 MHz only 29
	// samples (4.720 us) is in spec; 28 is 4.557 and 30 is 4.883. The former
	// 32 was 5.208 us, 11% wide. It is worth more than tidiness: sys/sys_top.v
	// builds composite sync from this pulse and derives the serration width
	// during vertical sync from its measured length, so an over-wide hsync
	// widens the serrations too.
	localparam [8:0] H_SYNC_END     = 9'd29;
	localparam [8:0] H_ACTIVE_START = 9'd140;
	localparam [8:0] H_ACTIVE_END   = 9'd300;

	// The native 152 rows are also emitted one-for-one. The remaining 110
	// lines are true vertical blank: 53 front, 3 sync, and 54 back.
	localparam [8:0] V_ACTIVE_END = 9'd152;
	localparam [8:0] V_SYNC_START = 9'd205;
	localparam [8:0] V_SYNC_END   = 9'd208;
	localparam [8:0] V_LAST       = 9'd261;

	// 180 * 3129 + 82 * 3130 = 819,880 exactly. RAW_SERVICE_CLOCKS is
	// 391 samples * eight raw clocks; the terminal one/two clocks are blank.
	localparam [11:0] RAW_SERVICE_CLOCKS = 12'd3128;
	localparam [11:0] RAW_SHORT_LAST     = 12'd3128;
	localparam [11:0] RAW_LONG_LAST      = 12'd3129;
	localparam [8:0]  LONG_LINE_COUNT    = 9'd82;
	localparam [8:0]  V_LONG_END         = V_ACTIVE_END + LONG_LINE_COUNT;

	// Public raster: free-running and exactly one native frame long

	// Power-up initialization is the timing generator's only reset. The core's
	// `rst` input below belongs to the emulated machine and must never stop or
	// phase-jump the public signal.
	reg [11:0] raw_line_clock_q;
	reg [8:0]  raster_y_q;

	// Keep the visible raster at one constant line period. Fractional-frame
	// compensation lives wholly in vertical blank, and the last 28 blank lines
	// return to the visible cadence before the next active row begins.
	wire long_line_w = (raster_y_q >= V_ACTIVE_END) &&
		(raster_y_q < V_LONG_END);
	wire [11:0] raw_line_last_w = long_line_w ?
		RAW_LONG_LAST : RAW_SHORT_LAST;
	wire raw_line_wrap_w = raw_line_clock_q == raw_line_last_w;
	wire frame_wrap_w = raw_line_wrap_w && (raster_y_q == V_LAST);
	wire sample_fire_w = (raw_line_clock_q < RAW_SERVICE_CLOCKS) &&
		(raw_line_clock_q[2:0] == 3'd0);
	wire [8:0] sample_x_w = raw_line_clock_q[11:3];

	always @(posedge clk_sys) begin
		if (raw_line_wrap_w) begin
			raw_line_clock_q <= 12'd0;
			raster_y_q       <= (raster_y_q == V_LAST) ?
				9'd0 : (raster_y_q + 9'd1);
		end else begin
			raw_line_clock_q <= raw_line_clock_q + 12'd1;
		end
	end

	// Generation ownership and exceptional frame-boundary admission

	reg        video_pause_req_q;
	reg        video_reset_hold_q;
	reg        display_valid_q;
	reg        pending_valid_q;
	reg        capture_active_q;
	reg        frame_locked_q;
	reg        sp_armed_q;
	reg        display_bank_q;
	reg        pending_bank_q;
	reg        build_bank_q;
	reg        response_history_valid_q;
	reg        capture_response_q;
	reg [14:0] write_addr_q;

	assign video_pause_req = video_pause_req_q;
	assign video_reset_hold = video_reset_hold_q;

	wire native_frame_start_w = lcd_dclk_ce && lcd_sp && sp_armed_q;
	wire capture_write_w = lcd_dclk_ce && lcd_de && capture_active_q &&
		!rst && !loading_savestate;
	wire capture_last_w = capture_write_w &&
		(write_addr_q == FRAME_LAST_ADDR);
	wire source_capture_allowed_w = !video_pause_req_q ||
		!machine_pause_ready;

	// If publication and a fresh native frame coincide, the retired display
	// bank is the new write bank. Otherwise the write bank is simply the bank
	// opposite the one displayed.
	wire start_bank_w = (frame_wrap_w && pending_valid_q) ?
		display_bank_q : !display_bank_q;

	wire ownership_fault_w = capture_active_q &&
		((build_bank_q == display_bank_q) ||
		 (pending_valid_q && (build_bank_q == pending_bank_q)));
	wire incomplete_fault_w = native_frame_start_w && capture_active_q;
	wire blocked_start_fault_w = native_frame_start_w && pending_valid_q &&
		!video_pause_req_q && !external_pause_req &&
		!(frame_wrap_w && pending_valid_q);
	wire overflow_fault_w = capture_last_w && pending_valid_q && !frame_wrap_w;
	wire underflow_fault_w = frame_wrap_w && frame_locked_q &&
		!video_pause_req_q && !external_pause_req &&
		!pending_valid_q && !capture_last_w;
	wire presentation_fault_w = ownership_fault_w || incomplete_fault_w ||
		blocked_start_fault_w || overflow_fault_w || underflow_fault_w;

	// Reset admission is not an ordinary pause. Holding the machine in reset
	// until public frame wrap starts every native counter at its canonical zero
	// phase and avoids parking the CPU between reset-vector fetch and its first
	// instruction prefetch.
	always @(posedge clk_sys) begin
		if (rst)
			video_reset_hold_q <= 1'b1;
		else if (video_reset_hold_q && frame_wrap_w)
			video_reset_hold_q <= 1'b0;
	end

	// Pause admission is separate from buffer ownership. During an ordinary
	// pause request the native raster is allowed to finish its current frame,
	// and that complete generation may still publish while the machine drains.
	always @(posedge clk_sys) begin
		if (rst) begin
			video_pause_req_q <= 1'b0;
		end else if (external_pause_req || loading_savestate ||
		             presentation_fault_w) begin
			video_pause_req_q <= 1'b1;
		end else if (video_pause_req_q && machine_pause_ready && frame_wrap_w) begin
			// Startup, resume and recovery all re-enter on this one boundary.
			video_pause_req_q <= 1'b0;
		end
	end

	always @(posedge clk_sys) begin
		if (rst) begin
			pending_valid_q  <= 1'b0;
			capture_active_q <= 1'b0;
			frame_locked_q   <= 1'b0;
			sp_armed_q       <= 1'b0;
			pending_bank_q   <= 1'b0;
			build_bank_q     <= !display_bank_q;
			response_history_valid_q <= 1'b0;
			capture_response_q <= 1'b0;
			write_addr_q     <= 15'd0;
		end else if (loading_savestate || presentation_fault_w) begin
			// A restore replaces mid-frame machine state. Keep the last displayed
			// complete bank, but a partial or unpublished generation is not valid.
			pending_valid_q  <= 1'b0;
			capture_active_q <= 1'b0;
			frame_locked_q   <= 1'b0;
			sp_armed_q       <= 1'b0;
			response_history_valid_q <= 1'b0;
			capture_response_q <= 1'b0;
			write_addr_q     <= 15'd0;
		end else begin
			// A frozen source can leave SP high. It is a pulse only after it has
			// first been observed low, matching the LCD pad arm-on-low rule.
			if (video_pause_req_q && machine_pause_ready && !capture_active_q)
				sp_armed_q <= 1'b0;
			else if (!lcd_sp)
				sp_armed_q <= 1'b1;
			else if (native_frame_start_w)
				sp_armed_q <= 1'b0;

			// Publication has priority over a simultaneous new build. No read
			// bank can change anywhere except this public frame-wrap branch.
			if (frame_wrap_w && pending_valid_q) begin
				display_bank_q   <= pending_bank_q;
				display_valid_q  <= 1'b1;
				pending_valid_q  <= 1'b0;
				frame_locked_q   <= 1'b1;
			end else if (frame_wrap_w && capture_last_w) begin
				display_bank_q   <= build_bank_q;
				display_valid_q  <= 1'b1;
				pending_valid_q  <= 1'b0;
				frame_locked_q   <= 1'b1;
			end

			if (capture_last_w) begin
				capture_active_q <= 1'b0;
				response_history_valid_q <= 1'b1;
				write_addr_q     <= 15'd0;
				if (pending_valid_q && frame_wrap_w) begin
					// Publish the older generation and queue the one completing now.
					pending_valid_q <= 1'b1;
					pending_bank_q  <= build_bank_q;
				end else if (!frame_wrap_w) begin
					pending_valid_q <= 1'b1;
					pending_bank_q  <= build_bank_q;
				end
			end else if (capture_write_w) begin
				write_addr_q <= write_addr_q + 15'd1;
			end

			if (native_frame_start_w && !capture_active_q &&
			    source_capture_allowed_w &&
			    (!pending_valid_q || frame_wrap_w)) begin
				build_bank_q      <= start_bank_w;
				capture_response_q <= lcd_persistence &&
					response_history_valid_q;
				capture_active_q  <= 1'b1;
				write_addr_q      <= 15'd0;
			end
		end
	end

	// Two split-depth RGB888 M10K banks

	// Port A of the displayed bank prefetches the old value at write_addr_q.
	// Native pixels are 24 clk_sys clocks apart, so the synchronous M10K output
	// is stable well before capture_write_w. The opposite bank is written on
	// that same edge. This preserves the proven completion/publication timing
	// and keeps strict two-bank ownership.
	wire [23:0] capture_raw_w = {
		{lcd_r, lcd_r}, {lcd_g, lcd_g}, {lcd_b, lcd_b}
	};
	wire write_low_w  = capture_write_w && !write_addr_q[14];
	wire write_high_w = capture_write_w &&  write_addr_q[14];

	reg [14:0] read_addr_q;
	wire [13:0] read_low_addr_w = read_addr_q[13:0];
	wire [12:0] read_high_addr_w = read_addr_q[12:0];

	wire [23:0] bank0_low_a_q, bank0_low_b_q;
	wire [23:0] bank0_high_a_q, bank0_high_b_q;
	wire [23:0] bank1_low_a_q, bank1_low_b_q;
	wire [23:0] bank1_high_a_q, bank1_high_b_q;
	wire        unused_ok;

	wire [23:0] previous_low_w = display_bank_q ?
		bank1_low_a_q : bank0_low_a_q;
	wire [23:0] previous_high_w = display_bank_q ?
		bank1_high_a_q : bank0_high_a_q;
	wire [23:0] previous_pixel_w = write_addr_q[14] ?
		previous_high_w : previous_low_w;

	// A one-pole optical-response approximation uses the preceding complete
	// generation as history. Target-directed tie rounding avoids a persistent
	// one-code floor on falling transitions while still converging rising ramps.
	// Keeping the recursive history at eight bits permits sub-RGB444 optical
	// levels instead of quantizing every intermediate frame back to a nibble.
	wire [8:0] response_r_sum_w =
		{1'b0, previous_pixel_w[23:16]} +
		{1'b0, capture_raw_w[23:16]} +
		{8'd0, capture_raw_w[23:16] >= previous_pixel_w[23:16]};
	wire [8:0] response_g_sum_w =
		{1'b0, previous_pixel_w[15:8]} +
		{1'b0, capture_raw_w[15:8]} +
		{8'd0, capture_raw_w[15:8] >= previous_pixel_w[15:8]};
	wire [8:0] response_b_sum_w =
		{1'b0, previous_pixel_w[7:0]} +
		{1'b0, capture_raw_w[7:0]} +
		{8'd0, capture_raw_w[7:0] >= previous_pixel_w[7:0]};
	wire [23:0] response_data_w = {
		response_r_sum_w[8:1],
		response_g_sum_w[8:1],
		response_b_sum_w[8:1]
	};
	wire [23:0] capture_data_w = capture_response_q ?
		response_data_w : capture_raw_w;

	cache_ram_dp #(.ADDR_WIDTH(14), .DATA_WIDTH(24)) u_bank0_low
	(
		.clk_i     (clk_sys),
		.addr_a_i  (write_addr_q[13:0]),
		.wren_a_i  (write_low_w && !build_bank_q),
		.wdata_a_i (capture_data_w),
		.q_a_o     (bank0_low_a_q),
		.addr_b_i  (read_low_addr_w),
		.wren_b_i  (1'b0),
		.wdata_b_i (24'd0),
		.q_b_o     (bank0_low_b_q)
	);

	cache_ram_dp #(.ADDR_WIDTH(13), .DATA_WIDTH(24)) u_bank0_high
	(
		.clk_i     (clk_sys),
		.addr_a_i  (write_addr_q[12:0]),
		.wren_a_i  (write_high_w && !build_bank_q),
		.wdata_a_i (capture_data_w),
		.q_a_o     (bank0_high_a_q),
		.addr_b_i  (read_high_addr_w),
		.wren_b_i  (1'b0),
		.wdata_b_i (24'd0),
		.q_b_o     (bank0_high_b_q)
	);

	cache_ram_dp #(.ADDR_WIDTH(14), .DATA_WIDTH(24)) u_bank1_low
	(
		.clk_i     (clk_sys),
		.addr_a_i  (write_addr_q[13:0]),
		.wren_a_i  (write_low_w && build_bank_q),
		.wdata_a_i (capture_data_w),
		.q_a_o     (bank1_low_a_q),
		.addr_b_i  (read_low_addr_w),
		.wren_b_i  (1'b0),
		.wdata_b_i (24'd0),
		.q_b_o     (bank1_low_b_q)
	);

	cache_ram_dp #(.ADDR_WIDTH(13), .DATA_WIDTH(24)) u_bank1_high
	(
		.clk_i     (clk_sys),
		.addr_a_i  (write_addr_q[12:0]),
		.wren_a_i  (write_high_w && build_bank_q),
		.wdata_a_i (capture_data_w),
		.q_a_o     (bank1_high_a_q),
		.addr_b_i  (read_high_addr_w),
		.wren_b_i  (1'b0),
		.wdata_b_i (24'd0),
		.q_b_o     (bank1_high_b_q)
	);

	wire [23:0] bank0_pixel_w = read_addr_q[14] ?
		bank0_high_b_q : bank0_low_b_q;
	wire [23:0] bank1_pixel_w = read_addr_q[14] ?
		bank1_high_b_q : bank1_low_b_q;
	wire [23:0] read_pixel_w = !display_bank_q ? bank0_pixel_w :
		bank1_pixel_w;

	// Native 160 x 152 one-for-one presentation reader

	reg [14:0] source_line_base_q;

	always @(posedge clk_sys) begin
		if (raw_line_wrap_w) begin
			if (raster_y_q == V_LAST) begin
				source_line_base_q <= 15'd0;
			end else if (raster_y_q < (V_ACTIVE_END - 9'd1)) begin
				source_line_base_q <= source_line_base_q + 15'd160;
			end
		end
	end

	// Attenuation is presentation-only and uses shifts, not multipliers.
	function automatic [7:0] atten(input [7:0] value, input [1:0] weight);
		case (weight)
			2'd0:    atten = value;
			2'd1:    atten = {1'b0, value[7:1]};
			2'd2:    atten = {2'b00, value[7:2]};
			default: atten = 8'd0;
		endcase
	endfunction

	function automatic [5:0] tint_weights(input [2:0] tint_sel);
		case (tint_sel)
			3'd1:    tint_weights = {2'd0, 2'd2, 2'd2};
			3'd2:    tint_weights = {2'd2, 2'd0, 2'd2};
			3'd3:    tint_weights = {2'd2, 2'd2, 2'd0};
			3'd4:    tint_weights = {2'd1, 2'd0, 2'd1};
			default: tint_weights = {2'd0, 2'd0, 2'd0};
		endcase
	endfunction

	wire [5:0] tint_w = tint_weights(tint);
	wire [7:0] pixel_r_w = atten(read_pixel_w[23:16], tint_w[5:4]);
	wire [7:0] pixel_g_w = atten(read_pixel_w[15:8],  tint_w[3:2]);
	wire [7:0] pixel_b_w = atten(read_pixel_w[7:0],   tint_w[1:0]);

	reg [7:0] vga_r_q;
	reg [7:0] vga_g_q;
	reg [7:0] vga_b_q;
	reg       vga_de_q;
	reg       vga_hbl_q;
	reg       vga_vbl_q;
	reg       vga_hs_q;
	reg       vga_vs_q;
	reg       ce_pixel_q;

	// Cyclone V configuration initializes these flops once. Core reset is
	// intentionally absent: it is not allowed to disturb the public raster.
	initial begin
		video_pause_req_q = 1'b0;
		video_reset_hold_q = 1'b1;
		display_valid_q   = 1'b0;
		pending_valid_q   = 1'b0;
		capture_active_q  = 1'b0;
		frame_locked_q    = 1'b0;
		sp_armed_q        = 1'b0;
		display_bank_q    = 1'b0;
		pending_bank_q    = 1'b0;
		build_bank_q      = 1'b1;
		response_history_valid_q = 1'b0;
		capture_response_q = 1'b0;
		write_addr_q      = 15'd0;
		raw_line_clock_q = 12'd0;
		raster_y_q       = 9'd0;
		read_addr_q      = 15'd0;
		source_line_base_q = 15'd0;
		vga_r_q          = 8'd0;
		vga_g_q          = 8'd0;
		vga_b_q          = 8'd0;
		vga_de_q         = 1'b0;
		vga_hbl_q        = 1'b1;
		vga_vbl_q        = 1'b1;
		vga_hs_q         = 1'b0;
		vga_vs_q         = 1'b0;
		ce_pixel_q       = 1'b0;
	end

	wire sample_hactive_w = (sample_x_w >= H_ACTIVE_START) &&
		(sample_x_w < H_ACTIVE_END);
	wire sample_vactive_w = raster_y_q < V_ACTIVE_END;
	wire sample_de_w = sample_hactive_w && sample_vactive_w;

	always @(posedge clk_sys) begin
		ce_pixel_q <= sample_fire_w;
		if (sample_fire_w) begin
			vga_hbl_q <= !sample_hactive_w;
			vga_vbl_q <= !sample_vactive_w;
			vga_de_q  <= sample_de_w;
			vga_hs_q  <= sample_x_w < H_SYNC_END;
			vga_vs_q  <= (raster_y_q >= V_SYNC_START) &&
			             (raster_y_q < V_SYNC_END);

			if (sample_de_w && display_valid_q) begin
				vga_r_q <= pixel_r_w;
				vga_g_q <= pixel_g_w;
				vga_b_q <= pixel_b_w;
			end else begin
				vga_r_q <= 8'd0;
				vga_g_q <= 8'd0;
				vga_b_q <= 8'd0;
			end

			// The blank sample immediately before active video prefetches
			// source pixel zero. Every active sample then advances once, so
			// each native pixel is presented exactly once.
			if ((sample_x_w + 9'd1) == H_ACTIVE_START)
				read_addr_q <= source_line_base_q;
			else if (sample_hactive_w)
				read_addr_q <= read_addr_q + 15'd1;
		end
	end

	assign ce_pixel = ce_pixel_q;
	assign vga_r    = vga_r_q;
	assign vga_g    = vga_g_q;
	assign vga_b    = vga_b_q;
	assign vga_de   = vga_de_q;
	assign vga_hbl  = vga_hbl_q;
	assign vga_vbl  = vga_vbl_q;
	assign vga_hs   = vga_hs_q;
	assign vga_vs   = vga_vs_q;

	assign unused_ok = &{1'b0,
		response_r_sum_w[0], response_g_sum_w[0], response_b_sum_w[0],
		1'b0};

endmodule
