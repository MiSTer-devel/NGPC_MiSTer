// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// One-sector (512 byte) HPS S0 bridge for the sparse cartridge overlay.
//
// The controller owns port A and fills it strictly in word order before a
// save request can be accepted. Port B is the MiSTer HPS endpoint. A read
// command stays owned after sd_ack falls until the controller has drained the
// received sector and pulses ctrl_rx_release_i. This prevents a later command
// from overwriting a sector that has not reached DDR3 staging yet.
//
// `sd_buff_addr_i` is the WIDE(1) 13-bit word address supplied by hps_io.
// A one-sector request accepts only addresses 0..255; any other write, an
// out-of-order word, or a short HPS transfer is reported as rx_error_o.
module ngp_cart_overlay_sector_bridge
(
	input  wire        clk,
	input  wire        reset,

	// Overlay-controller port A. Reads have the native one-clock M10K latency.
	input  wire        ctrl_fill_start_i,
	input  wire [7:0]  ctrl_buf_addr_i,
	input  wire        ctrl_buf_wr_i,
	input  wire [15:0] ctrl_buf_wdata_i,
	output wire [15:0] ctrl_buf_rdata_o,
	output wire        ctrl_fill_ready_o,
	output wire        ctrl_buf_wready_o,
	output wire        ctrl_buf_full_o,
	output wire        ctrl_fill_fault_o,

	// HPS request handshakes. A request is accepted only when its matching
	// ready signal is high. Requests must remain asserted until accepted.
	input  wire [31:0] ctrl_lba_i,
	input  wire        ctrl_save_req_i,
	output wire        ctrl_save_ready_o,
	input  wire        ctrl_load_req_i,
	output wire        ctrl_load_ready_o,

	// A completed HPS-to-core sector remains valid until this acknowledgement.
	input  wire        ctrl_rx_release_i,
	output wire        ctrl_rx_valid_o,
	output wire        ctrl_rx_error_o,
	output wire        ctrl_busy_o,

	// MiSTer S0 block interface.
	output wire [31:0] sd_lba_o,
	output wire        sd_rd_o,
	output wire        sd_wr_o,
	input  wire        sd_ack_i,
	input  wire [12:0] sd_buff_addr_i,
	input  wire [15:0] sd_buff_dout_i,
	input  wire        sd_buff_wr_i,
	output wire [15:0] sd_buff_din_o
);

	localparam [2:0] ST_IDLE        = 3'd0;
	localparam [2:0] ST_WRITE_ACK_HI = 3'd1;
	localparam [2:0] ST_WRITE_ACK_LO = 3'd2;
	localparam [2:0] ST_READ_ACK_HI  = 3'd3;
	localparam [2:0] ST_READ_ACK_LO  = 3'd4;
	localparam [2:0] ST_READ_DRAIN    = 3'd5;

	reg [2:0]  state_q;
	reg [31:0] lba_q;
	reg [7:0]  fill_expect_q;
	reg        fill_full_q;
	reg        fill_fault_q;
	reg [7:0]  rx_expect_q;
	reg        rx_full_q;
	reg        rx_fault_q;
	reg        rx_valid_q;
	reg        rx_error_q;

	// A fill is a deterministic 0..255 pass. That is both cheaper than a
	// 256-bit valid map and a direct assertion of the sequential p2 mover
	// contract. The controller must restart after any bad address.
	wire bridge_idle_w = (state_q == ST_IDLE) && !rx_valid_q;
	wire ctrl_fill_start_w = ctrl_fill_start_i && bridge_idle_w;
	wire ctrl_buf_wr_w = ctrl_buf_wr_i && bridge_idle_w && !fill_full_q &&
		!ctrl_fill_start_i;
	wire ctrl_fill_word_good_w = ctrl_buf_wr_w &&
		(ctrl_buf_addr_i == fill_expect_q) && !fill_fault_q;

	// hps_io addresses a WIDE sector as 256 sixteen-bit words. Writes are
	// accepted only while an S0 read acknowledgement is high.
	// hps_io registers its buffer-write strobe. The final word can therefore
	// trail sd_ack by one clk; retain ownership for a drain cycle and accept the
	// pipelined strobe instead of classifying a complete sector as short.
	wire hps_read_phase_w =
		((state_q == ST_READ_ACK_HI) && sd_ack_i) ||
		(state_q == ST_READ_ACK_LO) || (state_q == ST_READ_DRAIN);
	wire hps_buf_wr_w = hps_read_phase_w && sd_buff_wr_i;
	wire hps_word_good_w = hps_buf_wr_w && (sd_buff_addr_i[12:8] == 5'd0) &&
		(sd_buff_addr_i[7:0] == rx_expect_q) && !rx_fault_q;

	// Do not allow two commands to win one idle cycle. A previously received
	// sector must first be explicitly released by the controller.
	assign ctrl_fill_ready_o = bridge_idle_w;
	assign ctrl_buf_wready_o = bridge_idle_w && !fill_full_q && !ctrl_fill_start_i;
	assign ctrl_buf_full_o = fill_full_q;
	assign ctrl_fill_fault_o = fill_fault_q;
	assign ctrl_save_ready_o = bridge_idle_w && fill_full_q && !sd_ack_i &&
		!ctrl_fill_start_i && !ctrl_load_req_i;
	assign ctrl_load_ready_o = bridge_idle_w && !sd_ack_i && !ctrl_fill_start_i &&
		!ctrl_buf_wr_i && !ctrl_save_req_i;
	assign ctrl_rx_valid_o = rx_valid_q;
	assign ctrl_rx_error_o = rx_error_q;
	assign ctrl_busy_o = (state_q != ST_IDLE) || rx_valid_q;

	assign sd_lba_o = lba_q;
	assign sd_wr_o = (state_q == ST_WRITE_ACK_HI);
	assign sd_rd_o = (state_q == ST_READ_ACK_HI);

	cache_ram_dp_be #(
		.ADDR_WIDTH (8),
		.DATA_WIDTH (16)
	) u_sector (
		.clk_i     (clk),
		.addr_a_i  (ctrl_buf_addr_i),
		.wren_a_i  (ctrl_buf_wr_w),
		.be_a_i    (2'b11),
		.wdata_a_i (ctrl_buf_wdata_i),
		.q_a_o     (ctrl_buf_rdata_o),
		.addr_b_i  (sd_buff_addr_i[7:0]),
		.wren_b_i  (hps_buf_wr_w),
		.be_b_i    (2'b11),
		.wdata_b_i (sd_buff_dout_i),
		.q_b_o     (sd_buff_din_o)
	);

	always @(posedge clk) begin
		if (reset) begin
			state_q <= ST_IDLE;
			lba_q <= 32'd0;
			fill_expect_q <= 8'd0;
			fill_full_q <= 1'b0;
			fill_fault_q <= 1'b0;
			rx_expect_q <= 8'd0;
			rx_full_q <= 1'b0;
			rx_fault_q <= 1'b0;
			rx_valid_q <= 1'b0;
			rx_error_q <= 1'b0;
		end else begin
			// An accepted receive is held until its DDR3 drain is complete.
			if (rx_valid_q && ctrl_rx_release_i) begin
				rx_valid_q <= 1'b0;
				rx_error_q <= 1'b0;
			end

			if (ctrl_fill_start_w) begin
				fill_expect_q <= 8'd0;
				fill_full_q <= 1'b0;
				fill_fault_q <= 1'b0;
			end else if (ctrl_buf_wr_w) begin
				if (ctrl_fill_word_good_w) begin
					if (fill_expect_q == 8'hFF) begin
						fill_full_q <= 1'b1;
					end else begin
						fill_expect_q <= fill_expect_q + 8'd1;
					end
				end else begin
					fill_fault_q <= 1'b1;
				end
			end

			if (hps_buf_wr_w) begin
				if (hps_word_good_w) begin
					if (rx_expect_q == 8'hFF) begin
						rx_full_q <= 1'b1;
					end else begin
						rx_expect_q <= rx_expect_q + 8'd1;
					end
				end else begin
					rx_fault_q <= 1'b1;
				end
			end

			case (state_q)
				ST_IDLE: begin
					if (ctrl_save_req_i && ctrl_save_ready_o) begin
						lba_q <= ctrl_lba_i;
						state_q <= ST_WRITE_ACK_HI;
					end else if (ctrl_load_req_i && ctrl_load_ready_o) begin
						lba_q <= ctrl_lba_i;
						state_q <= ST_READ_ACK_HI;
						fill_full_q <= 1'b0;
						rx_expect_q <= 8'd0;
						rx_full_q <= 1'b0;
						rx_fault_q <= 1'b0;
						rx_valid_q <= 1'b0;
						rx_error_q <= 1'b0;
					end
				end

				ST_WRITE_ACK_HI: begin
					if (sd_ack_i) state_q <= ST_WRITE_ACK_LO;
				end

				ST_WRITE_ACK_LO: begin
					if (!sd_ack_i) state_q <= ST_IDLE;
				end

				ST_READ_ACK_HI: begin
					if (sd_ack_i) state_q <= ST_READ_ACK_LO;
				end

				ST_READ_ACK_LO: begin
					if (!sd_ack_i) begin
						state_q <= ST_READ_DRAIN;
					end
				end

				ST_READ_DRAIN: begin
					state_q <= ST_IDLE;
					if (rx_full_q && !rx_fault_q) begin
						rx_valid_q <= 1'b1;
					end else begin
						rx_error_q <= 1'b1;
					end
				end

				default: state_q <= ST_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
