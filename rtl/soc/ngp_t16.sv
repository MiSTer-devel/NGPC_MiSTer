// Copyright (c) 2026 Jamie Blanks

// TMP95C061-class 16-bit timers T4 and T5.
//
// Owns SFRs 0x30-0x3A and 0x40-0x49. The implementation follows the
// manufacturer's timer block and register diagrams (TMP95C061 datasheet
// pp.92-103): two 16-bit counters, four comparators, four capture registers,
// the TREG4/TREG6 double buffers, and timer flip-flops TFF4/TFF5/TFF6.
//
// Bus convention: combinational readback and one-clk_sys bus strobes that are
// not additionally ce-gated. Passage of time is ce-gated. Write-only timer
// registers remain visible only through the savestate tap.

module ngp_t16
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	input  wire [6:0]  sfr_addr,
	input  wire [7:0]  sfr_wdata,
	input  wire        sfr_wr,
	input  wire        sfr_rd,
	output wire [7:0]  sfr_rdata,

	input  wire        ti4,
	input  wire        ti5,
	input  wire        ti6,
	input  wire        ti7,
	input  wire        tff1,
	input  wire        phi_t1,
	input  wire        phi_t4,
	input  wire        phi_t16,
	input  wire        prrun,
	input  wire        t4run,
	input  wire        t5run,
	output wire        to4,
	output wire        to5,
	output wire        to6,
	output wire [3:0]  inttr,
	output wire [1:0]  cap12m,
	output wire [1:0]  cap34m,
	output wire        pg0t,
	output wire        pg1t,

	input  wire [7:0]  ss_reg_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        pause_req,
	output wire        pause_ready
);

	localparam [6:0] A_TREG4L = 7'h30;
	localparam [6:0] A_TREG4H = 7'h31;
	localparam [6:0] A_TREG5L = 7'h32;
	localparam [6:0] A_TREG5H = 7'h33;
	localparam [6:0] A_CAP1L  = 7'h34;
	localparam [6:0] A_CAP1H  = 7'h35;
	localparam [6:0] A_CAP2L  = 7'h36;
	localparam [6:0] A_CAP2H  = 7'h37;
	localparam [6:0] A_T4MOD  = 7'h38;
	localparam [6:0] A_T4FFCR = 7'h39;
	localparam [6:0] A_T45CR  = 7'h3A;
	localparam [6:0] A_TREG6L = 7'h40;
	localparam [6:0] A_TREG6H = 7'h41;
	localparam [6:0] A_TREG7L = 7'h42;
	localparam [6:0] A_TREG7H = 7'h43;
	localparam [6:0] A_CAP3L  = 7'h44;
	localparam [6:0] A_CAP3H  = 7'h45;
	localparam [6:0] A_CAP4L  = 7'h46;
	localparam [6:0] A_CAP4H  = 7'h47;
	localparam [6:0] A_T5MOD  = 7'h48;
	localparam [6:0] A_T5FFCR = 7'h49;

	// TREG4/TREG6 are the active comparator registers. Their buffers share
	// the CPU addresses and transfer on the upper-comparator match when the
	// corresponding T45CR enable is set (datasheet p.102).
	reg [15:0] treg4, treg5, treg6, treg7;
	reg [15:0] treg4_buf, treg6_buf;
	reg [15:0] cap1, cap2, cap3, cap4;
	reg [15:0] uc4, uc5;
	reg [7:0]  t4mod, t4ffcr, t45cr;
	reg [7:0]  t5mod, t5ffcr;
	reg        tff4, tff5, tff6;
	reg        ti4_q, ti5_q, ti6_q, ti7_q, tff1_q;

	// Fixed readback bits (datasheet pp.95-101). CAP1IN/CAP3IN are write-only
	// software-capture commands and always read one. Flip-flop command fields
	// always read 11. Unimplemented bits read zero.
	wire [7:0] t4mod_rb  = {t4mod[7:6], 1'b1, t4mod[4:0]};
	wire [7:0] t5mod_rb  = {2'b00, 1'b1, t5mod[4:0]};
	wire [7:0] t4ffcr_rb = {2'b11, t4ffcr[5:2], 2'b11};
	wire [7:0] t5ffcr_rb = {2'b00, t5ffcr[5:2], 2'b11};
	wire [7:0] t45cr_rb  = {4'b0000, t45cr[3:0]};

	reg [7:0] rdata;
	always_comb begin
		case (sfr_addr)
			A_CAP1L:  rdata = cap1[7:0];
			A_CAP1H:  rdata = cap1[15:8];
			A_CAP2L:  rdata = cap2[7:0];
			A_CAP2H:  rdata = cap2[15:8];
			A_T4MOD:  rdata = t4mod_rb;
			A_T4FFCR: rdata = t4ffcr_rb;
			A_T45CR:  rdata = t45cr_rb;
			A_CAP3L:  rdata = cap3[7:0];
			A_CAP3H:  rdata = cap3[15:8];
			A_CAP4L:  rdata = cap4[7:0];
			A_CAP4H:  rdata = cap4[15:8];
			A_T5MOD:  rdata = t5mod_rb;
			A_T5FFCR: rdata = t5ffcr_rb;
			default:  rdata = 8'hFF;
		endcase
	end
	assign sfr_rdata = rdata;

	assign cap12m = t4mod[4:3];
	assign cap34m = t5mod[4:3];
	assign pg0t   = t45cr[2];
	assign pg1t   = t45cr[3];
	assign to4    = tff4;
	assign to5    = tff5;
	assign to6    = tff6;

	// External inputs and TFF1 are levels sampled on ce, matching the
	// interrupt controller's external-pin convention. Internal phi inputs are
	// already one-clk_sys enables produced on a ce tick by ngp_t8.
	wire ti4_rise  = ce && ti4  && !ti4_q;
	wire ti4_fall  = ce && !ti4 &&  ti4_q;
	wire ti5_rise  = ce && ti5  && !ti5_q;
	wire ti6_rise  = ce && ti6  && !ti6_q;
	wire ti6_fall  = ce && !ti6 &&  ti6_q;
	wire ti7_rise  = ce && ti7  && !ti7_q;
	wire tff1_rise = ce && tff1  && !tff1_q;
	wire tff1_fall = ce && !tff1 &&  tff1_q;

	reg t4_clk_sel, t5_clk_sel;
	always_comb begin
		case (t4mod[1:0])
			2'b00:   t4_clk_sel = ti4_rise;
			2'b01:   t4_clk_sel = phi_t1;
			2'b10:   t4_clk_sel = phi_t4;
			default: t4_clk_sel = phi_t16;
		endcase
		case (t5mod[1:0])
			2'b00:   t5_clk_sel = ti6_rise;
			2'b01:   t5_clk_sel = phi_t1;
			2'b10:   t5_clk_sel = phi_t4;
			default: t5_clk_sel = phi_t16;
		endcase
	end

	wire t4_tick = ce && t4run && t4_clk_sel;
	wire t5_tick = ce && t5run && t5_clk_sel;
	wire [15:0] uc4_next = uc4 + 16'd1;
	wire [15:0] uc5_next = uc5 + 16'd1;
	wire match4 = t4_tick && (uc4_next == treg4);
	wire match5 = t4_tick && (uc4_next == treg5);
	wire match6 = t5_tick && (uc5_next == treg6);
	wire match7 = t5_tick && (uc5_next == treg7);

	// INTTR4-7 are comparator-match strobes, ordered low bit first to match
	// ngp_intc's vector slots 0x50-0x5C (datasheet pp.12, 103).
	assign inttr = {match7, match6, match5, match4};

	// Capture timing (datasheet pp.96, 99, 103). Mode 00 disables capture; mode
	// 01 captures the two external rising edges; mode 10 captures both edges of
	// TI4/TI6; mode 11 captures both edges of TFF1.
	reg cap1_hw, cap2_hw, cap3_hw, cap4_hw;
	always_comb begin
		cap1_hw = 1'b0;
		cap2_hw = 1'b0;
		cap3_hw = 1'b0;
		cap4_hw = 1'b0;
		case (t4mod[4:3])
			2'b01: begin cap1_hw = ti4_rise;  cap2_hw = ti5_rise;  end
			2'b10: begin cap1_hw = ti4_rise;  cap2_hw = ti4_fall;  end
			2'b11: begin cap1_hw = tff1_rise; cap2_hw = tff1_fall; end
			default: begin cap1_hw = 1'b0; cap2_hw = 1'b0; end
		endcase
		case (t5mod[4:3])
			2'b01: begin cap3_hw = ti6_rise;  cap4_hw = ti7_rise;  end
			2'b10: begin cap3_hw = ti6_rise;  cap4_hw = ti6_fall;  end
			2'b11: begin cap3_hw = tff1_rise; cap4_hw = tff1_fall; end
			default: begin cap3_hw = 1'b0; cap4_hw = 1'b0; end
		endcase
	end

	// A zero written to CAP1IN/CAP3IN takes a software capture. Toshiba says
	// the shared prescaler must be running (datasheet p.103), hence prrun rather
	// than the individual timer run bit qualifies these bus actions.
	wire swcap1 = sfr_wr && (sfr_addr == A_T4MOD) && !sfr_wdata[5] && prrun;
	wire swcap3 = sfr_wr && (sfr_addr == A_T5MOD) && !sfr_wdata[5] && prrun;
	wire cap1_load = cap1_hw || swcap1;
	wire cap3_load = cap3_hw || swcap3;

	// Hardware flip triggers (datasheet pp.96-100). Multiple simultaneous enabled
	// sources form one trigger and invert once, as the block diagrams show.
	wire flip4 = (t4ffcr[5] && cap2_hw)  || (t4ffcr[4] && cap1_load) ||
	             (t4ffcr[3] && match5)   || (t4ffcr[2] && match4);
	wire flip5 = (t4mod[7]  && cap2_hw)  || (t4mod[6]  && match5);
	wire flip6 = (t5ffcr[5] && cap4_hw)  || (t5ffcr[4] && cap3_load) ||
	             (t5ffcr[3] && match7)   || (t5ffcr[2] && match6);

	wire t4ffcr_wr = sfr_wr && (sfr_addr == A_T4FFCR);
	wire t5ffcr_wr = sfr_wr && (sfr_addr == A_T5FFCR);

	always @(posedge clk) begin
		if (reset) begin
			treg4     <= 16'h0000;
			treg5     <= 16'h0000;
			treg6     <= 16'h0000;
			treg7     <= 16'h0000;
			treg4_buf <= 16'h0000;
			treg6_buf <= 16'h0000;
			cap1      <= 16'h0000;
			cap2      <= 16'h0000;
			cap3      <= 16'h0000;
			cap4      <= 16'h0000;
			uc4       <= 16'h0000;
			uc5       <= 16'h0000;
			t4mod     <= 8'h20;
			t4ffcr    <= 8'h00;
			t45cr     <= 8'h00;
			t5mod     <= 8'h20;
			t5ffcr    <= 8'h00;
			tff4      <= 1'b0;
			tff5      <= 1'b0;
			tff6      <= 1'b0;
			ti4_q     <= 1'b0;
			ti5_q     <= 1'b0;
			ti6_q     <= 1'b0;
			ti7_q     <= 1'b0;
			tff1_q    <= 1'b0;
		end else begin
			// Time-advancing state. A stopped timer clears its counter on ce;
			// CLE only makes the upper comparator clear (datasheet pp.101-103).
			if (ce) begin
				ti4_q  <= ti4;
				ti5_q  <= ti5;
				ti6_q  <= ti6;
				ti7_q  <= ti7;
				tff1_q <= tff1;

				if      (!t4run)             uc4 <= 16'h0000;
				else if (match5 && t4mod[2]) uc4 <= 16'h0000;
				else if (t4_tick)            uc4 <= uc4_next;

				if      (!t5run)             uc5 <= 16'h0000;
				else if (match7 && t5mod[2]) uc5 <= 16'h0000;
				else if (t5_tick)            uc5 <= uc5_next;
			end

			// The low comparator register buffer transfers at the upper match.
			if (match5 && t45cr[0]) treg4 <= treg4_buf;
			if (match7 && t45cr[1]) treg6 <= treg6_buf;

			if (cap1_load) cap1 <= uc4;
			if (cap2_hw)   cap2 <= uc4;
			if (cap3_load) cap3 <= uc5;
			if (cap4_hw)   cap4 <= uc5;

			if (flip4) tff4 <= ~tff4;
			if (flip5) tff5 <= ~tff5;
			if (flip6) tff6 <= ~tff6;

			// CPU writes are bus strobes, not machine events. Writes to the low
			// timer register always update its buffer; DBxEN chooses whether the
			// active comparator register is written as well (datasheet p.102).
			if (sfr_wr) begin
				case (sfr_addr)
					A_TREG4L: begin
						treg4_buf[7:0] <= sfr_wdata;
						if (!t45cr[0]) treg4[7:0] <= sfr_wdata;
					end
					A_TREG4H: begin
						treg4_buf[15:8] <= sfr_wdata;
						if (!t45cr[0]) treg4[15:8] <= sfr_wdata;
					end
					A_TREG5L: treg5[7:0]  <= sfr_wdata;
					A_TREG5H: treg5[15:8] <= sfr_wdata;
					A_T4MOD:  t4mod <= {sfr_wdata[7:6], 1'b1, sfr_wdata[4:0]};
					A_T4FFCR: t4ffcr <= {2'b00, sfr_wdata[5:2], 2'b00};
					A_T45CR:  t45cr <= {4'b0000, sfr_wdata[3:0]};
					A_TREG6L: begin
						treg6_buf[7:0] <= sfr_wdata;
						if (!t45cr[1]) treg6[7:0] <= sfr_wdata;
					end
					A_TREG6H: begin
						treg6_buf[15:8] <= sfr_wdata;
						if (!t45cr[1]) treg6[15:8] <= sfr_wdata;
					end
					A_TREG7L: treg7[7:0]  <= sfr_wdata;
					A_TREG7H: treg7[15:8] <= sfr_wdata;
					A_T5MOD:  t5mod <= {2'b00, 1'b1, sfr_wdata[4:0]};
					A_T5FFCR: t5ffcr <= {2'b00, sfr_wdata[5:2], 2'b00};
					default: ;
				endcase
			end

			// Software flip-flop commands win over hardware triggers in the same
			// clk_sys cycle, matching ngp_t8's command priority.
			if (t4ffcr_wr) begin
				case (sfr_wdata[1:0])
					2'b00:   tff4 <= ~tff4;
					2'b01:   tff4 <= 1'b1;
					2'b10:   tff4 <= 1'b0;
					default: tff4 <= tff4;
				endcase
				case (sfr_wdata[7:6])
					2'b00:   tff5 <= ~tff5;
					2'b01:   tff5 <= 1'b1;
					2'b10:   tff5 <= 1'b0;
					default: tff5 <= tff5;
				endcase
			end
			if (t5ffcr_wr) begin
				case (sfr_wdata[1:0])
					2'b00:   tff6 <= ~tff6;
					2'b01:   tff6 <= 1'b1;
					2'b10:   tff6 <= 1'b0;
					default: tff6 <= tff6;
				endcase
			end

			// Words 0x10-0x17: registers, captures, counters, mode bytes,
			// flip-flop and edge history, and the two comparator buffers.
			if (ss_wren) begin
				case (ss_reg_addr)
					8'h10: begin treg5 <= ss_wdata[31:16]; treg4 <= ss_wdata[15:0]; end
					8'h11: begin treg7 <= ss_wdata[31:16]; treg6 <= ss_wdata[15:0]; end
					8'h12: begin cap2 <= ss_wdata[31:16]; cap1 <= ss_wdata[15:0]; end
					8'h13: begin cap4 <= ss_wdata[31:16]; cap3 <= ss_wdata[15:0]; end
					8'h14: begin uc5 <= ss_wdata[31:16]; uc4 <= ss_wdata[15:0]; end
					8'h15: begin
						t4mod  <= {ss_wdata[31:30], 1'b1, ss_wdata[28:24]};
						t4ffcr <= {2'b00, ss_wdata[21:18], 2'b00};
						t5mod  <= {2'b00, 1'b1, ss_wdata[12:8]};
						t5ffcr <= {2'b00, ss_wdata[5:2], 2'b00};
					end
					8'h16: begin
						tff4   <= ss_wdata[31];
						tff5   <= ss_wdata[30];
						tff6   <= ss_wdata[29];
						ti4_q  <= ss_wdata[28];
						ti5_q  <= ss_wdata[27];
						ti6_q  <= ss_wdata[26];
						ti7_q  <= ss_wdata[25];
						tff1_q <= ss_wdata[24];
						t45cr  <= {4'b0000, ss_wdata[3:0]};
					end
					8'h17: begin treg6_buf <= ss_wdata[31:16]; treg4_buf <= ss_wdata[15:0]; end
					default: ;
				endcase
			end
		end
	end

	reg [31:0] ss_rd;
	always_comb begin
		case (ss_reg_addr)
			8'h10:   ss_rd = {treg5, treg4};
			8'h11:   ss_rd = {treg7, treg6};
			8'h12:   ss_rd = {cap2, cap1};
			8'h13:   ss_rd = {cap4, cap3};
			8'h14:   ss_rd = {uc5, uc4};
			8'h15:   ss_rd = {t4mod_rb, t4ffcr_rb, t5mod_rb, t5ffcr_rb};
			8'h16:   ss_rd = {tff4, tff5, tff6, ti4_q, ti5_q, ti6_q, ti7_q,
			                  tff1_q, 16'h0000, t45cr_rb};
			8'h17:   ss_rd = {treg6_buf, treg4_buf};
			default: ss_rd = 32'h0000_0000;
		endcase
	end
	assign ss_rdata = ss_rd;

	// This timer has no transaction to drain. The parent withholds ce after
	// every participant reports ready, so counters and edge history freeze.
	assign pause_ready = 1'b1;

	wire unused_ok = &{1'b0, sfr_rd, pause_req,
	                   t4mod[5], t45cr[7:4], t5mod[7:5],
	                   t4ffcr[7:6], t4ffcr[1:0], t5ffcr[7:6], t5ffcr[1:0]};

endmodule
