// Copyright (c) 2026 Jamie Blanks

// TLCS-900/H general register file.
//
// Physical storage is 20 x 32-bit flops: banks 0-3 of XWA/XBC/XDE/XHL plus
// XIX/XIY/XIZ/XSP. Flops were chosen over memory blocks deliberately: the
// era-faithful structure is a small multi-ported register array, reads stay
// combinational for the sequencer, and the savestate tap is a plain mux.
//
// Register-code byte map (CPU900H p.45):
//   code[7:4] = 0-3 : absolute bank  (bank = code[5:4])
//   code[7:4] = D   : previous bank  (RFP - 1)
//   code[7:4] = E   : current bank   (RFP)
//   code[7:4] = F   : XIX/XIY/XIZ/XSP
//   code[3:2] = register within the group (XWA/XBC/XDE/XHL or XIX/XIY/XIZ/XSP)
//   code[1:0] = byte lane; word codes are multiples of 2, long of 4.
// Byte lanes of XWA are b0=A, b1=W, b2=QA, b3=QW (CPU900H p.29).
// Codes 40-7C are undefined; reads return zero here and writes drop.
// An invalid register code reaching this file is silently absorbed; only
// the decode tables raise INTUNDEF.
//
// Data convention: read data and write data are right-justified in 32 bits
// for every size. SIZE_B moves code[1:0]'s lane, SIZE_W moves the half
// selected by code[1], SIZE_L moves the whole register.

module t900_regfile
(
	input  wire        clk,

	input  wire [1:0]  rfp,          // current bank from SR

	// Read port A (combinational)
	input  wire [7:0]  ra_code,
	input  wire [1:0]  ra_size,      // 0=byte 1=word 2=long
	output wire [31:0] ra_data,

	// Read port B (combinational)
	input  wire [7:0]  rb_code,
	input  wire [1:0]  rb_size,
	output wire [31:0] rb_data,

	// Write port (registered)
	input  wire        wr_en,
	input  wire [7:0]  wr_code,
	input  wire [1:0]  wr_size,
	input  wire [31:0] wr_data,

	// Savestate tap: direct physical-index access, active only while the
	// core is paused. ss_wren writes a whole 32-bit entry.
	input  wire        ss_wren,
	input  wire [4:0]  ss_addr,
	input  wire [31:0] ss_wdata,
	output wire [31:0] ss_rdata
);

	localparam [1:0] SIZE_B = 2'd0;
	localparam [1:0] SIZE_W = 2'd1;
	localparam [1:0] SIZE_L = 2'd2;

	reg [31:0] regs [0:19];

	// Register-code bits [7:2] to physical index 0-19. Invalid codes report
	// !valid and index 0; callers treat the data as zero.
	function automatic [5:0] phys_index(input [5:0] chi, input [1:0] cur_rfp);
		reg [4:0] idx;
		reg       valid;
	begin
		valid = 1'b1;
		idx   = 5'd0;
		casez (chi[5:2])
			4'b00??: idx = {1'b0, chi[3:2], chi[1:0]};                      // absolute banks 0-3
			4'hd:    idx = {1'b0, (cur_rfp - 2'd1), chi[1:0]};              // previous bank
			4'he:    idx = {1'b0, cur_rfp, chi[1:0]};                       // current bank
			4'hf:    idx = {3'b100, chi[1:0]};                              // XIX/XIY/XIZ/XSP
			default: valid = 1'b0;                                          // 40-7C undefined
		endcase
		phys_index = {valid, idx};
	end
	endfunction

	function automatic [31:0] lane_read(input [31:0] full, input [1:0] lane, input [1:0] size);
	begin
		lane_read = 32'd0;
		case (size)
			SIZE_B: begin
				case (lane)
					2'd0: lane_read = {24'd0, full[7:0]};
					2'd1: lane_read = {24'd0, full[15:8]};
					2'd2: lane_read = {24'd0, full[23:16]};
					2'd3: lane_read = {24'd0, full[31:24]};
				endcase
			end
			SIZE_W:  lane_read = lane[1] ? {16'd0, full[31:16]} : {16'd0, full[15:0]};
			SIZE_L:  lane_read = full;
			default: lane_read = full;
		endcase
	end
	endfunction

	wire [5:0] ra_phys = phys_index(ra_code[7:2], rfp);
	wire [5:0] rb_phys = phys_index(rb_code[7:2], rfp);
	wire [5:0] wr_phys = phys_index(wr_code[7:2], rfp);

	assign ra_data = ra_phys[5] ? lane_read(regs[ra_phys[4:0]], ra_code[1:0], ra_size) : 32'd0;
	assign rb_data = rb_phys[5] ? lane_read(regs[rb_phys[4:0]], rb_code[1:0], rb_size) : 32'd0;

	wire ss_addr_ok = (ss_addr < 5'd20);

	always @(posedge clk) begin
		if (ss_wren && ss_addr_ok) begin
			regs[ss_addr] <= ss_wdata;
		end else if (wr_en && wr_phys[5]) begin
			case (wr_size)
				SIZE_B: begin
					case (wr_code[1:0])
						2'd0: regs[wr_phys[4:0]][7:0]   <= wr_data[7:0];
						2'd1: regs[wr_phys[4:0]][15:8]  <= wr_data[7:0];
						2'd2: regs[wr_phys[4:0]][23:16] <= wr_data[7:0];
						2'd3: regs[wr_phys[4:0]][31:24] <= wr_data[7:0];
					endcase
				end
				SIZE_W: begin
					if (wr_code[1]) begin
						regs[wr_phys[4:0]][31:16] <= wr_data[15:0];
					end else begin
						regs[wr_phys[4:0]][15:0] <= wr_data[15:0];
					end
				end
				default: regs[wr_phys[4:0]] <= wr_data;
			endcase
		end
	end

	assign ss_rdata = ss_addr_ok ? regs[ss_addr] : 32'd0;

endmodule
