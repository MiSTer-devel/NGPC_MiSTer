// Copyright (c) 2026 Jamie Blanks

// Builds a 16-byte MiSTer cheat record from WIDE(1) IOCTL halfword writes and
// coordinates an atomic live reload. The HPS is held with wait_o until the
// ordered machine pause has parked; pause_req_o then remains high through the
// engine's commit acknowledgement. That release boundary flushes the CPU's
// prefetch queue, so no instruction can mix old and newly patched bytes.
//
// The HPS streams records little-endian in the field order method/width,
// address, compare, value; the assembled record uses the layout documented in
// ngp_cheat_engine.sv. Bit 128 strobes for one clock when a record completes.
//
// Record assembly follows the standard MiSTer record layout and WIDE(1)
// transfer cadence. The pause and atomic-commit handshake is specific to this
// core's CPU read interposer.
module ngp_cheat_loader
(
	input  wire         clk_sys_i,
	input  wire         cheat_download_i,
	input  wire         cart_download_i,
	input  wire         paused_i,
	input  wire         commit_done_i,
	input  wire         ioctl_wr_i,
	input  wire [3:0]   ioctl_addr_i,
	input  wire [15:0]  ioctl_dout_i,
	output wire         wait_o,
	output reg          pause_req_o,
	output reg          load_begin_o,
	output reg          invalidate_o,
	output reg          commit_req_o,
	output reg  [128:0] code_o
);

	reg cheat_download_q;
	reg cart_download_q;

	// hps_io samples this combinationally. No payload halfword is accepted
	// until every machine client has reached the existing ordered pause point.
	assign wait_o = cheat_download_i && !paused_i;

	// Quartus maps these explicit initial values into FPGA power-up state. A
	// console reset is intentionally absent: live reset must not remove cheats.
	initial begin
		pause_req_o = 1'b0;
		load_begin_o = 1'b0;
		invalidate_o = 1'b0;
		commit_req_o = 1'b0;
		code_o = 129'd0;
		cheat_download_q = 1'b0;
		cart_download_q = 1'b0;
	end

	always @(posedge clk_sys_i) begin
		code_o[128] <= 1'b0;
		load_begin_o <= 1'b0;
		invalidate_o <= 1'b0;
		cheat_download_q <= cheat_download_i;
		cart_download_q <= cart_download_i;

		// Commit acknowledgement is the only normal release. The engine does not
		// acknowledge until the last wide record has expanded into the staging
		// bank, and it samples this request while the machine is still parked.
		if (commit_done_i) begin
			pause_req_o <= 1'b0;
			commit_req_o <= 1'b0;
		end

		// A new cheat transfer begins a fresh inactive generation but leaves the
		// active table untouched. Back-pressure above prevents its payload from
		// arriving before pause_ready.
		if (cheat_download_i && !cheat_download_q) begin
			pause_req_o <= 1'b1;
			load_begin_o <= 1'b1;
			commit_req_o <= 1'b0;
		end

		// Falling download means the complete payload has arrived. Keep this
		// level asserted until commit_done_i; a pulse could outrun a pending
		// four-byte expansion in the engine.
		if (!cheat_download_i && cheat_download_q) begin
			commit_req_o <= 1'b1;
		end

		// Cartridge replacement already holds the machine in reset. Invalidate
		// both transaction state and the active generation immediately.
		if (cart_download_i && !cart_download_q) begin
			pause_req_o <= 1'b0;
			commit_req_o <= 1'b0;
			invalidate_o <= 1'b1;
		end

		if (cheat_download_i && paused_i && ioctl_wr_i) begin
			case (ioctl_addr_i)
				4'd0:  code_o[111:96]  <= ioctl_dout_i;
				4'd2:  code_o[127:112] <= ioctl_dout_i;
				4'd4:  code_o[79:64]   <= ioctl_dout_i;
				4'd6:  code_o[95:80]   <= ioctl_dout_i;
				4'd8:  code_o[47:32]   <= ioctl_dout_i;
				4'd10: code_o[63:48]   <= ioctl_dout_i;
				4'd12: code_o[15:0]    <= ioctl_dout_i;
				4'd14: begin
					code_o[31:16] <= ioctl_dout_i;
					code_o[128]   <= 1'b1;
				end
				default: ;
			endcase
		end
	end

endmodule
