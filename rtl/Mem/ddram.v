//
// ddram.v
// Copyright (c) 2019 Sorgelig
//
// Single-channel 64-bit bridge into the MiSTer DDR3 window.

module ddram
(
	input  wire        DDRAM_CLK,
	input  wire        DDRAM_BUSY,
	output wire [7:0]  DDRAM_BURSTCNT,
	output wire [28:0] DDRAM_ADDR,
	input  wire [63:0] DDRAM_DOUT,
	input  wire        DDRAM_DOUT_READY,
	output wire        DDRAM_RD,
	output wire [63:0] DDRAM_DIN,
	output wire [7:0]  DDRAM_BE,
	output wire        DDRAM_WE,

	input  wire [27:1] ch1_addr,
	output wire [63:0] ch1_dout,
	input  wire [63:0] ch1_din,
	input  wire        ch1_req,
	input  wire        ch1_rnw,
	input  wire [7:0]  ch1_be,
	output wire        ch1_ready,

	input  wire [27:1] ch2_addr,
	output wire [63:0] ch2_dout,
	input  wire [63:0] ch2_din,
	input  wire        ch2_req,
	input  wire        ch2_rnw,
	input  wire [7:0]  ch2_be,
	output wire        ch2_ready
);

	localparam STATE_IDLE = 1'b0;
	localparam STATE_WAIT_READ = 1'b1;

	reg [63:0] read_data_q;
	reg [63:0] write_data_q;
	reg [24:0] address_q;
	reg        read_q = 1'b0;
	reg        write_q = 1'b0;
	reg [7:0]  byte_enable_q;
	reg        ch1_ready_q = 1'b0;
	reg        ch2_ready_q = 1'b0;
	reg        state_q = STATE_IDLE;
	reg        ch1_pending_q = 1'b0;
	reg        ch2_pending_q = 1'b0;
	reg        active_ch2_q = 1'b0;

	// One helper address unit is four physical bytes. Address bits 2:1 select
	// bytes within a 64-bit beat and are intentionally discarded by the bridge.
	assign DDRAM_BURSTCNT = 8'd1;
	assign DDRAM_ADDR = {4'b0011, address_q};
	assign DDRAM_RD = read_q;
	assign DDRAM_DIN = write_data_q;
	assign DDRAM_BE = read_q ? 8'hff : byte_enable_q;
	assign DDRAM_WE = write_q;
	assign ch1_dout = read_data_q;
	assign ch2_dout = read_data_q;
	assign ch1_ready = ch1_ready_q;
	assign ch2_ready = ch2_ready_q;

	always @(posedge DDRAM_CLK) begin
		ch1_pending_q <= ch1_pending_q || ch1_req;
		ch2_pending_q <= ch2_pending_q || ch2_req;
		ch1_ready_q <= 1'b0;
		ch2_ready_q <= 1'b0;

		if (!DDRAM_BUSY) begin
			write_q <= 1'b0;
			read_q <= 1'b0;

			case (state_q)
				STATE_IDLE: begin
					if (ch1_pending_q || ch1_req) begin
						ch1_pending_q <= 1'b0;
						write_data_q <= ch1_din;
						byte_enable_q <= ch1_be;
						address_q <= ch1_addr[27:3];
						active_ch2_q <= 1'b0;
						if (ch1_rnw) begin
							read_q <= 1'b1;
							state_q <= STATE_WAIT_READ;
						end else begin
							write_q <= 1'b1;
							ch1_ready_q <= 1'b1;
						end
					end else if (ch2_pending_q || ch2_req) begin
						ch2_pending_q <= 1'b0;
						write_data_q <= ch2_din;
						byte_enable_q <= ch2_be;
						address_q <= ch2_addr[27:3];
						active_ch2_q <= 1'b1;
						if (ch2_rnw) begin
							read_q <= 1'b1;
							state_q <= STATE_WAIT_READ;
						end else begin
							write_q <= 1'b1;
							ch2_ready_q <= 1'b1;
						end
					end
				end

				STATE_WAIT_READ: begin
					if (DDRAM_DOUT_READY) begin
						read_data_q <= DDRAM_DOUT;
						if (active_ch2_q) begin
							ch2_ready_q <= 1'b1;
						end else begin
							ch1_ready_q <= 1'b1;
						end
						state_q <= STATE_IDLE;
					end
				end
			endcase
		end
	end

endmodule
