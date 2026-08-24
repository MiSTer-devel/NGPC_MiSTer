// Copyright (c) 2026 Jamie Blanks

module savestate_ui
#(
	parameter INFO_TIMEOUT_BITS = 25
)
(
	input  wire        clk,
	input  wire [10:0] ps2_key,
	input  wire        allow_ss,
	input  wire        joySS,
	input  wire        joyRight,
	input  wire        joyLeft,
	input  wire        joyDown,
	input  wire        joyUp,
	input  wire        joyStart,
	input  wire        joySaveState,
	input  wire [1:0]  status_slot,
	input  wire [1:0]  OSD_saveload,
	output reg         ss_save,
	output reg         ss_load,
	output reg         ss_info_req,
	output reg  [7:0]  ss_info,
	output reg         statusUpdate,
	output wire [1:0]  selected_slot
);
	reg [1:0] selected_slot_q = 2'd0;
	reg last_right_q = 1'b0;
	reg last_left_q = 1'b0;
	reg last_down_q = 1'b0;
	reg last_up_q = 1'b0;
	reg [INFO_TIMEOUT_BITS-1:0] info_wait_q =
		{INFO_TIMEOUT_BITS{1'b0}};
	reg slot_switched_q = 1'b0;
	reg [1:0] last_status_slot_q = 2'd0;
	reg key_toggle_q = 1'b0;
	reg alt_q = 1'b0;
	reg [1:0] last_osd_saveload_q = 2'd0;

	wire key_pressed_w = ps2_key[9];

	assign selected_slot = selected_slot_q;

	always @(posedge clk) begin
		key_toggle_q <= ps2_key[10];
		last_right_q <= joyRight;
		last_left_q <= joyLeft;
		last_down_q <= joyDown;
		last_up_q <= joyUp;
		last_status_slot_q <= status_slot;

		slot_switched_q <= 1'b0;
		ss_save <= 1'b0;
		ss_load <= 1'b0;
		ss_info_req <= 1'b0;
		statusUpdate <= 1'b0;

		if (allow_ss) begin
			if (key_toggle_q != ps2_key[10]) begin
				case (ps2_key[7:0])
					8'h11: alt_q <= key_pressed_w;
					8'h05: begin
						ss_save <= key_pressed_w && alt_q;
						ss_load <= key_pressed_w && !alt_q;
						selected_slot_q <= 2'd0;
						statusUpdate <= 1'b1;
					end
					8'h06: begin
						ss_save <= key_pressed_w && alt_q;
						ss_load <= key_pressed_w && !alt_q;
						selected_slot_q <= 2'd1;
						statusUpdate <= 1'b1;
					end
					8'h04: begin
						ss_save <= key_pressed_w && alt_q;
						ss_load <= key_pressed_w && !alt_q;
						selected_slot_q <= 2'd2;
						statusUpdate <= 1'b1;
					end
					8'h0c: begin
						ss_save <= key_pressed_w && alt_q;
						ss_load <= key_pressed_w && !alt_q;
						selected_slot_q <= 2'd3;
						statusUpdate <= 1'b1;
					end
					default: begin
					end
				endcase
			end

			if (last_status_slot_q != status_slot) begin
				selected_slot_q <= status_slot;
				statusUpdate <= 1'b1;
			end

			if (joySS || joySaveState) begin
				info_wait_q <= info_wait_q +
					{{(INFO_TIMEOUT_BITS-1){1'b0}}, 1'b1};
				if (info_wait_q[INFO_TIMEOUT_BITS-1]) begin
					ss_info <= 8'd5;
					ss_info_req <= 1'b1;
					info_wait_q <= {INFO_TIMEOUT_BITS{1'b0}};
				end

				if (joySS && joyRight && !last_right_q &&
					(selected_slot_q < 2'd3)) begin
					selected_slot_q <= selected_slot_q + 2'd1;
					statusUpdate <= 1'b1;
					slot_switched_q <= 1'b1;
					info_wait_q <= {INFO_TIMEOUT_BITS{1'b0}};
				end
				if (joySS && joyLeft && !last_left_q &&
					(selected_slot_q != 2'd0)) begin
					selected_slot_q <= selected_slot_q - 2'd1;
					statusUpdate <= 1'b1;
					slot_switched_q <= 1'b1;
					info_wait_q <= {INFO_TIMEOUT_BITS{1'b0}};
				end

				if ((joySaveState || (joySS && joyStart)) &&
					joyDown && !last_down_q) begin
					ss_save <= 1'b1;
					info_wait_q <= {INFO_TIMEOUT_BITS{1'b0}};
				end
				if ((joySaveState || (joySS && joyStart)) &&
					joyUp && !last_up_q) begin
					ss_load <= 1'b1;
					info_wait_q <= {INFO_TIMEOUT_BITS{1'b0}};
				end
			end else begin
				info_wait_q <= {INFO_TIMEOUT_BITS{1'b0}};
			end

			last_osd_saveload_q <= OSD_saveload;
			if (last_osd_saveload_q[0] ^ OSD_saveload[0]) begin
				ss_save <= OSD_saveload[0];
			end
			if (last_osd_saveload_q[1] ^ OSD_saveload[1]) begin
				ss_load <= OSD_saveload[1];
			end

			if (slot_switched_q) begin
				ss_info <= 8'd6 + {6'd0, selected_slot_q};
				ss_info_req <= 1'b1;
			end
			if (ss_load || ss_save) begin
				ss_info <= 8'd10 + {5'd0, selected_slot_q, ss_load};
				ss_info_req <= 1'b1;
			end
		end
	end
endmodule
