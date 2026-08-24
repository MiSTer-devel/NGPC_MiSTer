`timescale 1ns/1ps
`default_nettype none

// MiSTer SDR SDRAM controller.
//
// Sits inside an `emu`-side wrapper and connects directly to either the
// primary `SDRAM_*` pins or the reduced secondary `SDRAM2_*` pins from
// `sys/emu_ports.vh`. The DQM/CKE outputs exist for the primary compatibility
// interface; on secondary SDRAM leave them unconnected and wire only the real
// `SDRAM2_*` pins.
//
// Clocking:
// - `clk` is the SDRAM command/state clock. On Cyclone V an internal ALTDDIO
//   output cell drives an inverted `SDRAM_CLK` and the matching input cell
//   captures DQ at the end of each SDRAM data window; the wrapper supplies no
//   second clock.
// - `reset` is asynchronous, active high. All client busy outputs stay high
//   until the JEDEC-style init sequence completes.
// - This module always drives the SDRAM pins. Absent-board tri-state behavior
//   such as `SDRAM2_EN` belongs in the wrapper.
//
// Client contract:
// - Each port is a busy/request interface. A client may present `req` when
//   its port's `busy` is low, which is the only "may pulse req now"
//   condition: `busy` is also high for init, refresh, close timing, or a
//   higher-priority port. The controller samples `req`, `we`, `addr`, `din`
//   and `byte_en` on the accepted rising `clk` edge, then raises `busy`.
//   Deassert `req` after `busy` rises unless deliberately issuing a second
//   request behind the current one.
// - Port priority is fixed: p0 beats p1, and p1 beats p2.
// - PORTx_SIZE selects the access width: 0 = 8, 1 = 16, 2 = 32, 3 = 64 bits.
//   Data always rides the 64-bit `din`/`dout` buses; narrow reads are
//   zero-extended and narrow writes use the low bits of `din`.
// - `byte_en[7:0]` is active high over successive bytes of `din`. Bits above
//   the port width are ignored; for an 8-bit port byte_en[0] controls the
//   byte selected by addr[0]. Reads ignore byte_en.
// - `ready` reports completion only, never acceptance. It falls when a
//   request is accepted and rises when read data is published or the final
//   write beat completes; reset, init, arbitration, close timing and refresh
//   do not lower it. Consult `busy` before issuing a request.
// - `dout` is valid on the same edge that raises `ready`; a write is
//   client-complete on that edge too.
// - The controller may internally accept one pending request after the
//   previous transfer completes but before tRC/tDAL close timing has elapsed.
//   That pending request still reports busy until its own transfer completes.
//
// Address contract:
// - `addr` is a byte address in the 64 MB, 16-bit SDRAM space.
// - Mapping is BA = addr[25:24], row = addr[23:11], column = addr[10:1].
// - Low address bits are masked to the port width before a command issues:
//   none for 8-bit, bit 0 for 16-bit, bits 1:0 for 32-bit, bits 2:0 for
//   64-bit. For 8-bit accesses only, addr[0] selects the byte lane.
//
// Hardware/timing model:
// - MiSTer SDRAM boards do not have independent DQM FPGA pins; cores encode
//   the chip byte masks through A12:A11. This controller therefore drives
//   `SDRAM_DQML = SDRAM_A[11]` and `SDRAM_DQMH = SDRAM_A[12]` at all times.
//   During READ/WRITE column and write-data cycles A12:A11 carry DQM values;
//   during row/mode/precharge commands they carry address values and DQM is
//   irrelevant because no data beat is being sampled.
// - All command, address, DQ data and DQ OE outputs are driven from registers
//   for MiSTer's pad-register/IOE timing expectations. DQM aliases registered
//   address bits, so it follows the same path.
// - Access sequencing is closed-page: ACTIVATE, wait tRCD, READ/WRITE with
//   auto-precharge, complete the client-visible transfer, then satisfy the
//   aggregate tRC/write-recovery close target before launching the next
//   command from idle or the pending slot.
// - The mode register is programmed for the largest configured read burst.
//   Write-burst mode is disabled; 32/64-bit writes are adjacent single WRITE
//   commands with auto-precharge on the final beat.
// - Read capture includes one extra cycle for the registered command/pad path
//   used on MiSTer SDRAM pins.
// - `SDRAM_TIMING_GRADE` selects the AS4C32M16SB timing profile:
//     7 = -7TIN/-7TCN, 143 MHz CL3 part; the default MiSTer profile.
//     6 = -6TIN, 166 MHz CL3 part.
//   CL2 is selected up to 100 MHz, CL3 above it. The profile's upper CL3
//   frequency is documented by the datasheet but not enforced here; choosing
//   a legal SDRAM clock and board timing strategy is the integrator's job.
module sdram #(
	parameter int unsigned CLK_FREQ_HZ = 100_000_000,
	parameter int unsigned SDRAM_TIMING_GRADE = 7,
	parameter int unsigned PORT0_SIZE = 1,
	parameter int unsigned PORT1_SIZE = 1,
	parameter int unsigned PORT2_SIZE = 1,
	parameter bit AUTO_REFRESH = 1'b1
) (
	input  wire        clk,
	input  wire        reset,

	input  wire        refresh,

	input  wire        p0_req,
	input  wire        p0_we,
	input  wire [25:0] p0_addr,
	input  wire [63:0] p0_din,
	input  wire [7:0]  p0_byte_en,
	output logic [63:0] p0_dout,
	output wire        p0_busy,
	output logic       p0_ready,

	input  wire        p1_req,
	input  wire        p1_we,
	input  wire [25:0] p1_addr,
	input  wire [63:0] p1_din,
	input  wire [7:0]  p1_byte_en,
	output logic [63:0] p1_dout,
	output wire        p1_busy,
	output logic       p1_ready,

	input  wire        p2_req,
	input  wire        p2_we,
	input  wire [25:0] p2_addr,
	input  wire [63:0] p2_din,
	input  wire [7:0]  p2_byte_en,
	output logic [63:0] p2_dout,
	output wire        p2_busy,
	output logic       p2_ready,

	output wire        SDRAM_CLK,
	output logic       SDRAM_CKE,
	output logic [12:0] SDRAM_A,
	output logic [1:0] SDRAM_BA,
	inout  wire [15:0] SDRAM_DQ,
	output wire        SDRAM_DQML,
	output wire        SDRAM_DQMH,
	output logic       SDRAM_nCS,
	output logic       SDRAM_nCAS,
	output logic       SDRAM_nRAS,
	output logic       SDRAM_nWE
);

	localparam logic [3:0] CMD_DESELECT      = 4'b1111;
	localparam logic [3:0] CMD_NOP           = 4'b0111;
	localparam logic [3:0] CMD_ACTIVE        = 4'b0011;
	localparam logic [3:0] CMD_READ          = 4'b0101;
	localparam logic [3:0] CMD_WRITE         = 4'b0100;
	localparam logic [3:0] CMD_PRECHARGE     = 4'b0010;
	localparam logic [3:0] CMD_AUTO_REFRESH  = 4'b0001;
	localparam logic [3:0] CMD_LOAD_MODE     = 4'b0000;

	// Elaboration-time timing helper. The division here is constant-folded
	// into localparams; it is not a runtime hardware divider.
	function automatic [15:0] cycles_for_ns;
		input [31:0] freq_hz;
		input [31:0] ns;
		reg [63:0] num;
		reg [63:0] cycles;
		begin
			num = ({32'd0, freq_hz} * {32'd0, ns}) + 64'd999_999_999;
			cycles = num / 64'd1_000_000_000;
			if (cycles < 64'd1) cycles_for_ns = 16'd1;
			else cycles_for_ns = cycles[15:0];
		end
	endfunction

	// After issuing a command in the current enabled cycle, wait this many
	// additional enabled cycles before the next dependent command/action.
	function automatic [15:0] count_after_command;
		input [15:0] cycles;
		begin
			if (cycles > 16'd1) count_after_command = cycles - 16'd1;
			else count_after_command = 16'd0;
		end
	endfunction

	function automatic [15:0] max16;
		input [15:0] a;
		input [15:0] b;
		begin
			if (a > b) max16 = a;
			else max16 = b;
		end
	endfunction

	function automatic [15:0] add16_sat;
		input [15:0] a;
		input [15:0] b;
		reg [16:0] sum;
		begin
			sum = {1'b0, a} + {1'b0, b};
			if (sum[16]) add16_sat = 16'hFFFF;
			else add16_sat = sum[15:0];
		end
	endfunction

	function automatic [2:0] size_to_beats;
		input [1:0] size;
		begin
			unique case (size)
				2'd2: size_to_beats = 3'd2;
				2'd3: size_to_beats = 3'd4;
				default: size_to_beats = 3'd1;
			endcase
		end
	endfunction

	function automatic [2:0] max3;
		input [2:0] a;
		input [2:0] b;
		input [2:0] c;
		reg [2:0] m;
		begin
			m = a;
			if (b > m) m = b;
			if (c > m) m = c;
			max3 = m;
		end
	endfunction

	function automatic [2:0] burst_code;
		input [2:0] beats;
		begin
			unique case (beats)
				3'd4: burst_code = 3'b010;
				3'd2: burst_code = 3'b001;
				default: burst_code = 3'b000;
			endcase
		end
	endfunction

	function automatic [25:0] align_addr;
		input [1:0] size;
		input [25:0] addr;
		begin
			unique case (size)
				2'd0: align_addr = addr;
				2'd1: align_addr = {addr[25:1], 1'b0};
				2'd2: align_addr = {addr[25:2], 2'b00};
				default: align_addr = {addr[25:3], 3'b000};
			endcase
		end
	endfunction

	function automatic [15:0] write_beat_data;
		input [1:0] size;
		input [63:0] data;
		input [1:0] beat;
		begin
			unique case (size)
				2'd0: write_beat_data = {data[7:0], data[7:0]};
				2'd1: write_beat_data = data[15:0];
				2'd2: begin
					if (beat[0]) write_beat_data = data[31:16];
					else write_beat_data = data[15:0];
				end
				default: begin
					unique case (beat)
					2'd0: write_beat_data = data[15:0];
					2'd1: write_beat_data = data[31:16];
					2'd2: write_beat_data = data[47:32];
				default: write_beat_data = data[63:48];
					endcase
				end
			endcase
		end
	endfunction

	function automatic [9:0] beat_col;
		input [9:0] col;
		input [1:0] beat;
		begin
			beat_col = col + {8'd0, beat};
		end
	endfunction

	function automatic [1:0] write_beat_dqm;
		input [1:0] size;
		input addr_bit;
		input [7:0] byte_en;
		input [1:0] beat;
		begin
			if (size == 2'd0) begin
				if (addr_bit) write_beat_dqm = {~byte_en[0], 1'b1};
				else write_beat_dqm = {1'b1, ~byte_en[0]};
			end else begin
				unique case (beat)
					2'd0: write_beat_dqm = {~byte_en[1], ~byte_en[0]};
					2'd1: write_beat_dqm = {~byte_en[3], ~byte_en[2]};
					2'd2: write_beat_dqm = {~byte_en[5], ~byte_en[4]};
					default: write_beat_dqm = {~byte_en[7], ~byte_en[6]};
				endcase
			end
		end
	endfunction

	function automatic [63:0] format_read_data;
		input [1:0] size;
		input addr_bit;
		input [63:0] data;
		begin
			unique case (size)
				2'd0: begin
					if (addr_bit) format_read_data = {56'd0, data[15:8]};
					else format_read_data = {56'd0, data[7:0]};
				end
				2'd1: format_read_data = {48'd0, data[15:0]};
				2'd2: format_read_data = {32'd0, data[31:0]};
				default: format_read_data = data;
			endcase
		end
	endfunction

	function automatic [63:0] set_read_beat;
		input [63:0] data;
		input [1:0] beat;
		input [15:0] word_data;
		begin
			set_read_beat = data;
			unique case (beat)
				2'd0: set_read_beat[15:0] = word_data;
				2'd1: set_read_beat[31:16] = word_data;
				2'd2: set_read_beat[47:32] = word_data;
				default: set_read_beat[63:48] = word_data;
			endcase
		end
	endfunction

	function automatic [63:0] cache_read_data;
		input addr_bit;
		input [15:0] data;
		begin
			if (addr_bit) cache_read_data = {56'd0, data[15:8]};
			else cache_read_data = {56'd0, data[7:0]};
		end
	endfunction

	localparam logic [2:0] PORT0_BEATS = size_to_beats(PORT0_SIZE[1:0]);
	localparam logic [2:0] PORT1_BEATS = size_to_beats(PORT1_SIZE[1:0]);
	localparam logic [2:0] PORT2_BEATS = size_to_beats(PORT2_SIZE[1:0]);

	// Program the chip for the widest read port. Write-burst mode stays
	// disabled, matching the common MiSTer controller pattern: wider writes
	// are emitted as adjacent single WRITE commands and the final WRITE uses
	// auto-precharge.
	localparam logic [2:0] MODE_BEATS = max3(PORT0_BEATS, PORT1_BEATS, PORT2_BEATS);
	localparam logic [2:0] MODE_BURST = burst_code(MODE_BEATS);
	// AS4C32M16SB supports CL2 or CL3. For both -6TIN and -7TIN, CL2 is
	// valid through 100 MHz; above that this controller programs CL3. The
	// CL3 frequency ceiling is not enforced here.
	localparam logic [2:0] CAS_LATENCY = (CLK_FREQ_HZ <= 100_000_000) ? 3'd2 : 3'd3;
	localparam logic MODE_WRITE_SINGLE = 1'b1;
	localparam logic [12:0] MODE_REG = {3'b000, MODE_WRITE_SINGLE, 2'b00, CAS_LATENCY, 1'b0, MODE_BURST};
	// Read capture pipeline, counted from the rising state-clock edge that
	// registers the READ command to the rising edge on which the state machine
	// may consume the beat:
	//   +0.5  the registered command reaches SDRAM on the next rising pin-clock
	//         edge, which is half a state-clock cycle later because SDRAM_CLK is
	//         the inverted state clock;
	//   +CL   the SDRAM drives the beat CAS_LATENCY pin clocks after it samples
	//         the command;
	//   +0.5  the input DDIO cell captures that beat on the falling state-clock
	//         edge sitting inside the beat's valid window (after tAC, before
	//         tOH of the following edge);
	//   +0.5  the DDIO cell realigns the captured value onto `dataout_l` at the
	//         following rising edge;
	//   +1    `dataout_l` is a registered output, so the first rising edge that
	//         can observe the new value is the one after that realignment edge.
	// The half cycles sum to a whole clock, giving CAS_LATENCY + 3. This holds
	// for CL2 and CL3 at every supported frequency, so it stays a constant
	// rather than a frequency-dependent expression. CAS_LATENCY + 2 omits the
	// DDIO realign register, samples one clock early, and returns the previous
	// idle or Hi-Z bus beat on real hardware.
	localparam logic [15:0] DQ_CAPTURE_PIPELINE = 16'd3;
	localparam logic [15:0] CAS_READ_CYCLES =
		{13'd0, CAS_LATENCY} + DQ_CAPTURE_PIPELINE;

	// AS4C32M16SB timing values, rounded up from nanoseconds using
	// CLK_FREQ_HZ. `SDRAM_TIMING_GRADE=7` matches MiSTer's default
	// AS4C32M16SB-7TIN/-7TCN boards. `6` selects the faster -6TIN profile.
	// Other values intentionally fall back to the conservative -7 profile.
	localparam logic TIMING_GRADE_6 = (SDRAM_TIMING_GRADE == 6);
	localparam logic [31:0] T_RCD_NS = TIMING_GRADE_6 ? 32'd18 : 32'd21;
	localparam logic [31:0] T_RP_NS = TIMING_GRADE_6 ? 32'd18 : 32'd21;
	localparam logic [31:0] T_RC_NS = TIMING_GRADE_6 ? 32'd60 : 32'd63;
	localparam logic [31:0] T_RFC_NS = TIMING_GRADE_6 ? 32'd60 : 32'd63;
	localparam logic [31:0] T_WR_NS = TIMING_GRADE_6 ? 32'd12 : 32'd14;
	localparam logic [31:0] T_MRD_NS = TIMING_GRADE_6 ? 32'd12 : 32'd14;

`ifndef SYNTHESIS
	// Elaboration guards. The CAS choice above is derived from CLK_FREQ_HZ, but
	// nothing otherwise stops an integrator from configuring a clock the part
	// cannot sustain at that latency. Catch it at elaboration rather than as
	// intermittent read corruption on a board.
	localparam int unsigned CL2_MAX_HZ = 100_000_000;
	localparam int unsigned CL3_MAX_HZ = TIMING_GRADE_6 ? 166_000_000 : 143_000_000;
	initial begin
		if ((CAS_LATENCY == 3'd2) && (CLK_FREQ_HZ > CL2_MAX_HZ)) begin
			$error("sdram: CL2 is not valid above %0d Hz", CL2_MAX_HZ);
		end
		if ((CAS_LATENCY == 3'd3) && (CLK_FREQ_HZ > CL3_MAX_HZ)) begin
			$error("sdram: CLK_FREQ_HZ %0d exceeds the CL3 ceiling %0d for the configured speed grade",
				CLK_FREQ_HZ, CL3_MAX_HZ);
		end
		if ((SDRAM_TIMING_GRADE != 6) && (SDRAM_TIMING_GRADE != 7)) begin
			$display("sdram: SDRAM_TIMING_GRADE %0d is unknown; using the conservative -7 profile",
				SDRAM_TIMING_GRADE);
		end
	end
`endif

	// Keep these as localparams so synthesis sees constants.
	// CKE stays low for the full 200 us datasheet startup interval; one NOP
	// cycle after CKE rises is sufficient before PRECHARGE ALL.
	localparam logic [15:0] T_POWER_WAIT = cycles_for_ns(CLK_FREQ_HZ[31:0], 32'd200_000);
	localparam logic [15:0] T_CKE_NOP = 16'd1;
	localparam logic [15:0] T_RCD = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RCD_NS);
	localparam logic [15:0] T_RP = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RP_NS);
	localparam logic [15:0] T_RC = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RC_NS);
	localparam logic [15:0] T_RFC = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RFC_NS);
	localparam logic [15:0] T_DAL = cycles_for_ns(CLK_FREQ_HZ[31:0], T_WR_NS + T_RP_NS);
	localparam logic [15:0] T_MRD = cycles_for_ns(CLK_FREQ_HZ[31:0], T_MRD_NS);
	// Use a conservative interval below the 64 ms / 8192-row average. Rounding
	// 7.813 us upward produced a 7.820 us interval at 100 MHz.
	localparam logic [15:0] T_REFI = cycles_for_ns(CLK_FREQ_HZ[31:0], 32'd7_800);
	localparam logic [15:0] T_REFI_COUNT = count_after_command(T_REFI);
	// One count below the interval. `auto_refresh_due_q` is registered, so it
	// has to be decided from the counter value one cycle before the one it
	// reports on. This changes no interval: the flag still rises on the cycle
	// where `refresh_ctr_q` reaches T_REFI_COUNT. Do not "simplify" this by
	// moving the interval itself -- a T_REFI computed against the wrong clock
	// silently breaks the AS4C32M16SB's 7.8125 us refresh requirement.
	localparam logic [15:0] T_REFI_PRECOUNT =
		(T_REFI_COUNT > 16'd0) ? (T_REFI_COUNT - 16'd1) : 16'd0;
	// tRC covers a one-word read. Every additional programmed burst beat delays
	// auto-precharge by one clock, so extend the closed-bank target accordingly.
	localparam logic [15:0] T_READ_CLOSE = add16_sat(
		T_RC, {13'd0, MODE_BEATS - 3'd1});

	// Refresh is tracked as small debt. Low-priority ports are blocked first
	// as debt rises; p0 is blocked only when refresh becomes more urgent.
	localparam logic [4:0] REFRESH_BLOCK_P2 = 5'd1;
	localparam logic [4:0] REFRESH_BLOCK_P1 = 5'd2;
	localparam logic [4:0] REFRESH_BLOCK_P0 = 5'd4;
	localparam logic [4:0] REFRESH_DEBT_MAX = 5'd15;

	function automatic [4:0] add_refresh_debt;
		input [4:0] debt;
		input [1:0] add_count;
		reg [5:0] sum;
		begin
			sum = {1'b0, debt} + {4'd0, add_count};
			if (sum > {1'b0, REFRESH_DEBT_MAX}) add_refresh_debt = REFRESH_DEBT_MAX;
			else add_refresh_debt = sum[4:0];
		end
	endfunction

	function automatic [4:0] consume_refresh_debt;
		input [4:0] debt;
		begin
			if (debt != 5'd0) consume_refresh_debt = debt - 5'd1;
			else consume_refresh_debt = 5'd0;
		end
	endfunction

	typedef enum logic [4:0] {
		ST_POWER_WAIT,
		ST_NOP_WAIT,
		ST_INIT_TRP,
		ST_INIT_RFC1,
		ST_INIT_RFC2,
		ST_INIT_MRD,
		ST_IDLE,
		ST_RCD,
		ST_READ_LATENCY,
		ST_READ_BEATS,
		ST_READ_DRAIN,
		ST_WRITE_BEATS,
		ST_RAS_WAIT,
		ST_REFRESH_WAIT
	} state_e;

	state_e state_q;
	logic init_done_q;
	logic [15:0] wait_q;
	logic [15:0] active_age_q;
	logic [15:0] close_age_target_q;

	logic [1:0] active_port_q;
	logic [1:0] active_size_q;
	logic active_we_q;
	logic [25:0] active_addr_q;
	logic [63:0] active_din_q;
	logic [7:0] active_byte_en_q;
	logic [2:0] active_beats_q;
	logic [2:0] beat_q;
	logic [63:0] read_data_q;
	logic client_done_q;

	logic pending_valid_q;
	logic [1:0] pending_port_q;
	logic [1:0] pending_size_q;
	logic pending_we_q;
	logic [25:0] pending_addr_q;
	logic [63:0] pending_din_q;
	logic [7:0] pending_byte_en_q;
	logic [2:0] pending_beats_q;

	// One 16-bit word cache per port for 8-bit reads. A hit on either byte
	// of the cached word completes through a one-cycle busy pulse without
	// spending an SDRAM command slot.
	logic [24:0] cache0_tag_q;
	logic [15:0] cache0_data_q;
	logic cache0_valid_q;
	logic [24:0] cache1_tag_q;
	logic [15:0] cache1_data_q;
	logic cache1_valid_q;
	logic [24:0] cache2_tag_q;
	logic [15:0] cache2_data_q;
	logic cache2_valid_q;

	logic [15:0] refresh_ctr_q;
	logic auto_refresh_due_q;
	logic [4:0] refresh_debt_q;
	logic refresh_old_q;

	logic [15:0] dq_out_q;
	logic dq_oe_q;
	wire [15:0] dq_in;
	wire [15:0] dq_aligned_w;

	logic p0_busy_q;
	logic p1_busy_q;
	logic p2_busy_q;
	logic p0_accept_i;
	logic p1_accept_i;
	logic p2_accept_i;
	logic p0_cache_done_q;
	logic p1_cache_done_q;
	logic p2_cache_done_q;

	wire p0_cache_hit = p0_req && !p0_we && (PORT0_SIZE[1:0] == 2'd0) && cache0_valid_q && (cache0_tag_q == p0_addr[25:1]);
	wire p1_cache_hit = p1_req && !p1_we && (PORT1_SIZE[1:0] == 2'd0) && cache1_valid_q && (cache1_tag_q == p1_addr[25:1]);
	wire p2_cache_hit = p2_req && !p2_we && (PORT2_SIZE[1:0] == 2'd0) && cache2_valid_q && (cache2_tag_q == p2_addr[25:1]);
	wire [25:0] p0_aligned_addr = align_addr(PORT0_SIZE[1:0], p0_addr);
	wire [25:0] p1_aligned_addr = align_addr(PORT1_SIZE[1:0], p1_addr);
	wire [25:0] p2_aligned_addr = align_addr(PORT2_SIZE[1:0], p2_addr);

	// `auto_refresh_due_q` registers the interval compare so it does not sit
	// at the head of the longest path in this module. That path is
	//   refresh compare -> refresh debt add -> port blocking -> command select
	//   -> SDRAM_A
	// and it has to cross the die from the controller body to the SDRAM pins,
	// so the routing is fixed and the only budget available is logic depth.
	// Launching from a flop removes the 16-bit compare and its fanout from it.
	wire refresh_edge_due = init_done_q && refresh && !refresh_old_q;
	wire [1:0] refresh_add_count = {1'b0, auto_refresh_due_q} + {1'b0, refresh_edge_due};
	wire [4:0] refresh_debt_with_add = add_refresh_debt(refresh_debt_q, refresh_add_count);
	wire refresh_pending = refresh_debt_with_add != 5'd0;

	wire idle_accept = init_done_q && (state_q == ST_IDLE) && !pending_valid_q;
	wire queue_accept = init_done_q && client_done_q && !pending_valid_q && (state_q != ST_IDLE) && (state_q != ST_REFRESH_WAIT);
	wire accept_window = idle_accept || queue_accept;
	wire refresh_block_p0 = refresh_debt_with_add >= REFRESH_BLOCK_P0;
	wire refresh_block_p1 = refresh_debt_with_add >= REFRESH_BLOCK_P1;
	wire refresh_block_p2 = refresh_debt_with_add >= REFRESH_BLOCK_P2;
	wire refresh_idle_slot = !p0_req && !p1_req && !p2_req;
	wire refresh_blocks_requested_port = (refresh_block_p0 && p0_req) ||
		(refresh_block_p1 && !p0_req && p1_req) ||
		(refresh_block_p2 && !p0_req && !p1_req && p2_req);
	wire refresh_service_now = refresh_pending && (refresh_idle_slot || refresh_blocks_requested_port);

	// Keep the physical clock and DQ capture relationship local to this module.
	// The output DDIO cell produces an exact inverted copy of clk at the pad.
	// The input DDIO cell's low-phase result samples DQ on the next rising SDRAM
	// pin edge, after the preceding beat has had the full tAC interval to arrive.
	// Standard Cyclone V I/O-cell pattern; it adds no fabric falling-edge
	// clock domain.
`ifdef SYNTHESIS
	altddio_out #(
		.extend_oe_disable("OFF"),
		.intended_device_family("Cyclone V"),
		.invert_output("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_out"),
		.oe_reg("UNREGISTERED"),
		.power_up_high("OFF"),
		.width(1)
	) u_sdram_clock (
		.datain_h(1'b0),
		.datain_l(1'b1),
		.outclock(clk),
		.dataout(SDRAM_CLK),
		.aclr(1'b0),
		.aset(1'b0),
		.oe(1'b1),
		.outclocken(1'b1),
		.sclr(1'b0),
		.sset(1'b0)
	);

	altddio_in #(
		.intended_device_family("Cyclone V"),
		.invert_input_clocks("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_in"),
		.power_up_high("OFF"),
		.width(16)
	) u_sdram_dq_capture (
		.datain(dq_in),
		.inclock(clk),
		.inclocken(1'b1),
		.aclr(1'b0),
		.aset(1'b0),
		.sclr(1'b0),
		.sset(1'b0),
		.dataout_h(),
		.dataout_l(dq_aligned_w)
	);
`else
	// Reproduces the physical capture pipeline exactly. Quartus 17's
	// cyclonev_ddio_in atom captures the low word on the falling edge and only
	// transfers it to the `dataout_l` output register on the following rising
	// edge; modelling just the falling edge would make the beat visible to the
	// state machine a full clock earlier than hardware delivers it.
	reg [15:0] dq_capture_sim_q = 16'd0;
	reg [15:0] dq_aligned_sim_q = 16'd0;

	assign SDRAM_CLK = ~clk;
	assign dq_aligned_w = dq_aligned_sim_q;

	// Stage 1: falling state-clock edge, i.e. the rising SDRAM pin-clock edge.
	// The memory model supplies tAC/tOH and pad skew, so this samples the real
	// valid window rather than an idealised one.
	always @(posedge SDRAM_CLK) begin
		dq_capture_sim_q <= dq_in;
	end

	// Stage 2: the DDIO output register that realigns the captured beat to the
	// rising state clock.
	always @(posedge clk) begin
		dq_aligned_sim_q <= dq_capture_sim_q;
	end
`endif

	// Keep the compatibility DQM outputs as exact aliases of A11/A12. This
	// matches the board wiring/reference-core convention and means the same
	// command/address stream can drive the reduced secondary SDRAM interface.
	assign SDRAM_DQML = SDRAM_A[11];
	assign SDRAM_DQMH = SDRAM_A[12];

	// Keep the bidirectional pad handling in one place: data and OE are both
	// registered so Quartus can place the final stage in the SDRAM IOE.
	assign dq_in = SDRAM_DQ;
	assign SDRAM_DQ = dq_oe_q ? dq_out_q : 16'hZZZZ;

	// Public busy is intentionally stricter than the per-port in-flight bit:
	// low means the port can be accepted on the next rising edge. This avoids
	// one-cycle request pulses being lost during refresh, close timing, or a
	// higher-priority request.
	assign p0_busy = p0_busy_q || !p0_accept_i;
	assign p1_busy = p1_busy_q || !p1_accept_i;
	assign p2_busy = p2_busy_q || !p2_accept_i;

	// Internal accept qualification drives the busy-style public interface. It
	// indicates that an idle port can be accepted either into the controller or
	// into the one-entry pending slot after client-visible completion. Urgent
	// refresh debt blocks lower-priority ports first, while cache hits remain
	// free to serve because they do not use the SDRAM bus.
	always_comb begin
		p0_accept_i = 1'b0;
		p1_accept_i = 1'b0;
		p2_accept_i = 1'b0;

		if (accept_window) begin
			p0_accept_i = !p0_busy_q && (!refresh_block_p0 || p0_cache_hit);
			p1_accept_i = !p1_busy_q && !p0_req && (!refresh_block_p1 || p1_cache_hit);
			p2_accept_i = !p2_busy_q && !p0_req && !p1_req && (!refresh_block_p2 || p2_cache_hit);
		end
	end

	// Keep reset asynchronous on this register bank. In particular, Cyclone V
	// SDRAM output and output-enable registers cannot pack into the IOE when they
	// use synchronous clear. Async reset allows those final 100 MHz stages to
	// pack at the pads; do not change this back to synchronous reset without
	// confirming FAST_OUTPUT_REGISTER/FAST_OUTPUT_ENABLE_REGISTER packing.
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			state_q <= ST_POWER_WAIT;
			init_done_q <= 1'b0;
			wait_q <= count_after_command(T_POWER_WAIT);
			active_age_q <= 16'd0;
			close_age_target_q <= 16'd0;
			active_port_q <= 2'd0;
			active_size_q <= 2'd1;
			active_we_q <= 1'b0;
			active_addr_q <= 26'd0;
			active_din_q <= 64'd0;
			active_byte_en_q <= 8'd0;
			active_beats_q <= 3'd1;
			beat_q <= 3'd0;
			read_data_q <= 64'd0;
			client_done_q <= 1'b0;
			pending_valid_q <= 1'b0;
			pending_port_q <= 2'd0;
			pending_size_q <= 2'd1;
			pending_we_q <= 1'b0;
			pending_addr_q <= 26'd0;
			pending_din_q <= 64'd0;
			pending_byte_en_q <= 8'd0;
			pending_beats_q <= 3'd1;
			cache0_valid_q <= 1'b0;
			cache1_valid_q <= 1'b0;
			cache2_valid_q <= 1'b0;
			cache0_tag_q <= 25'd0;
			cache1_tag_q <= 25'd0;
			cache2_tag_q <= 25'd0;
			cache0_data_q <= 16'd0;
			cache1_data_q <= 16'd0;
			cache2_data_q <= 16'd0;
			refresh_ctr_q <= 16'd0;
			auto_refresh_due_q <= 1'b0;
			refresh_debt_q <= 5'd0;
			refresh_old_q <= 1'b0;
			p0_dout <= 64'd0;
			p1_dout <= 64'd0;
			p2_dout <= 64'd0;
			p0_busy_q <= 1'b1;
			p1_busy_q <= 1'b1;
			p2_busy_q <= 1'b1;
			p0_ready <= 1'b1;
			p1_ready <= 1'b1;
			p2_ready <= 1'b1;
			p0_cache_done_q <= 1'b0;
			p1_cache_done_q <= 1'b0;
			p2_cache_done_q <= 1'b0;
			SDRAM_CKE <= 1'b0;
			SDRAM_A <= 13'd0;
			SDRAM_BA <= 2'd0;
			{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_DESELECT;
			dq_out_q <= 16'd0;
			dq_oe_q <= 1'b0;
		end else begin
			refresh_old_q <= refresh;
			if (p0_cache_done_q) begin
				p0_busy_q <= 1'b0;
				p0_ready <= 1'b1;
				p0_cache_done_q <= 1'b0;
			end
			if (p1_cache_done_q) begin
				p1_busy_q <= 1'b0;
				p1_ready <= 1'b1;
				p1_cache_done_q <= 1'b0;
			end
			if (p2_cache_done_q) begin
				p2_busy_q <= 1'b0;
				p2_ready <= 1'b1;
				p2_cache_done_q <= 1'b0;
			end

			{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= init_done_q ? CMD_NOP : CMD_DESELECT;
			dq_oe_q <= 1'b0;
			refresh_debt_q <= refresh_debt_with_add;

			if (init_done_q && AUTO_REFRESH) begin
				// AUTO_REFRESH creates debt on the JEDEC refresh interval.
				// A rising external `refresh` request also adds debt; if both
				// happen in one cycle, both increments are counted before any
				// same-cycle refresh service consumes one debt slot.
				// The counter and its "due" flag advance together. The flag
				// is decided from the value the counter is about to take.
				// The >= keeps a self-healing property: a counter that
				// somehow sat above the interval still raises the flag.
				if (auto_refresh_due_q) begin
					refresh_ctr_q <= 16'd0;
					auto_refresh_due_q <= (T_REFI_COUNT == 16'd0);
				end else begin
					refresh_ctr_q <= refresh_ctr_q + 16'd1;
					auto_refresh_due_q <= (refresh_ctr_q >= T_REFI_PRECOUNT);
				end
			end

			if ((state_q != ST_POWER_WAIT) && (state_q != ST_NOP_WAIT) && (state_q != ST_IDLE)) begin
				if (active_age_q != 16'hFFFF) active_age_q <= active_age_q + 16'd1;
			end

			if (queue_accept) begin
				if (p0_req && p0_accept_i && p0_cache_hit) begin
					p0_dout <= cache_read_data(p0_addr[0], cache0_data_q);
					p0_busy_q <= 1'b1;
					p0_ready <= 1'b0;
					p0_cache_done_q <= 1'b1;
				end else if (p1_req && p1_accept_i && p1_cache_hit) begin
					p1_dout <= cache_read_data(p1_addr[0], cache1_data_q);
					p1_busy_q <= 1'b1;
					p1_ready <= 1'b0;
					p1_cache_done_q <= 1'b1;
				end else if (p2_req && p2_accept_i && p2_cache_hit) begin
					p2_dout <= cache_read_data(p2_addr[0], cache2_data_q);
					p2_busy_q <= 1'b1;
					p2_ready <= 1'b0;
					p2_cache_done_q <= 1'b1;
				end else if (p0_req && p0_accept_i) begin
					p0_busy_q <= 1'b1;
					p0_ready <= 1'b0;
					pending_valid_q <= 1'b1;
					pending_port_q <= 2'd0;
					pending_size_q <= PORT0_SIZE[1:0];
					pending_we_q <= p0_we;
					pending_addr_q <= p0_aligned_addr;
					pending_din_q <= p0_din;
					pending_byte_en_q <= p0_byte_en;
					pending_beats_q <= PORT0_BEATS;
					cache0_valid_q <= p0_we ? 1'b0 : cache0_valid_q;
					cache1_valid_q <= p0_we ? 1'b0 : cache1_valid_q;
					cache2_valid_q <= p0_we ? 1'b0 : cache2_valid_q;
				end else if (p1_req && p1_accept_i) begin
					p1_busy_q <= 1'b1;
					p1_ready <= 1'b0;
					pending_valid_q <= 1'b1;
					pending_port_q <= 2'd1;
					pending_size_q <= PORT1_SIZE[1:0];
					pending_we_q <= p1_we;
					pending_addr_q <= p1_aligned_addr;
					pending_din_q <= p1_din;
					pending_byte_en_q <= p1_byte_en;
					pending_beats_q <= PORT1_BEATS;
					cache0_valid_q <= p1_we ? 1'b0 : cache0_valid_q;
					cache1_valid_q <= p1_we ? 1'b0 : cache1_valid_q;
					cache2_valid_q <= p1_we ? 1'b0 : cache2_valid_q;
				end else if (p2_req && p2_accept_i) begin
					p2_busy_q <= 1'b1;
					p2_ready <= 1'b0;
					pending_valid_q <= 1'b1;
					pending_port_q <= 2'd2;
					pending_size_q <= PORT2_SIZE[1:0];
					pending_we_q <= p2_we;
					pending_addr_q <= p2_aligned_addr;
					pending_din_q <= p2_din;
					pending_byte_en_q <= p2_byte_en;
					pending_beats_q <= PORT2_BEATS;
					cache0_valid_q <= p2_we ? 1'b0 : cache0_valid_q;
					cache1_valid_q <= p2_we ? 1'b0 : cache1_valid_q;
					cache2_valid_q <= p2_we ? 1'b0 : cache2_valid_q;
				end
			end

			unique case (state_q)
				ST_POWER_WAIT: begin
					// JEDEC init: hold CKE low, then CKE high with NOPs,
					// precharge all banks, issue two refreshes, load mode.
					SDRAM_CKE <= 1'b0;
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						SDRAM_CKE <= 1'b1;
						state_q <= ST_NOP_WAIT;
						wait_q <= count_after_command(T_CKE_NOP);
					end
				end

				ST_NOP_WAIT: begin
					SDRAM_CKE <= 1'b1;
					{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_NOP;
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						SDRAM_A <= 13'b0010000000000;
						SDRAM_BA <= 2'd0;
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_PRECHARGE;
						state_q <= ST_INIT_TRP;
						wait_q <= count_after_command(T_RP);
					end
				end

				ST_INIT_TRP: begin
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						state_q <= ST_INIT_RFC1;
						wait_q <= count_after_command(T_RFC);
					end
				end

				ST_INIT_RFC1: begin
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						state_q <= ST_INIT_RFC2;
						wait_q <= count_after_command(T_RFC);
					end
				end

				ST_INIT_RFC2: begin
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						SDRAM_A <= MODE_REG;
						SDRAM_BA <= 2'd0;
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_LOAD_MODE;
						state_q <= ST_INIT_MRD;
						wait_q <= count_after_command(T_MRD);
					end
				end

				ST_INIT_MRD: begin
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						init_done_q <= 1'b1;
						p0_busy_q <= 1'b0;
						p1_busy_q <= 1'b0;
						p2_busy_q <= 1'b0;
						state_q <= ST_IDLE;
					end
				end

				ST_IDLE: begin
					active_age_q <= 16'd0;
					if (pending_valid_q) begin
						active_port_q <= pending_port_q;
						active_size_q <= pending_size_q;
						active_we_q <= pending_we_q;
						active_addr_q <= pending_addr_q;
						active_din_q <= pending_din_q;
						active_byte_en_q <= pending_byte_en_q;
						active_beats_q <= pending_beats_q;
						beat_q <= 3'd0;
						read_data_q <= 64'd0;
						client_done_q <= 1'b0;
						close_age_target_q <= 16'd0;
						pending_valid_q <= 1'b0;
						SDRAM_A <= pending_addr_q[23:11];
						SDRAM_BA <= pending_addr_q[25:24];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
						state_q <= ST_RCD;
						wait_q <= count_after_command(T_RCD);
					end else if (p0_req && p0_accept_i && p0_cache_hit) begin
						// Cache-hit reads use the same busy-falling
						// completion convention as SDRAM-backed reads.
						// If refresh is pending, use this free SDRAM slot.
						p0_dout <= cache_read_data(p0_addr[0], cache0_data_q);
						p0_busy_q <= 1'b1;
						p0_ready <= 1'b0;
						p0_cache_done_q <= 1'b1;
						if (refresh_pending) begin
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
							state_q <= ST_REFRESH_WAIT;
							wait_q <= count_after_command(T_RFC);
							refresh_debt_q <= consume_refresh_debt(refresh_debt_with_add);
						end
					end else if (p1_req && p1_accept_i && p1_cache_hit) begin
						p1_dout <= cache_read_data(p1_addr[0], cache1_data_q);
						p1_busy_q <= 1'b1;
						p1_ready <= 1'b0;
						p1_cache_done_q <= 1'b1;
						if (refresh_pending) begin
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
							state_q <= ST_REFRESH_WAIT;
							wait_q <= count_after_command(T_RFC);
							refresh_debt_q <= consume_refresh_debt(refresh_debt_with_add);
						end
					end else if (p2_req && p2_accept_i && p2_cache_hit) begin
						p2_dout <= cache_read_data(p2_addr[0], cache2_data_q);
						p2_busy_q <= 1'b1;
						p2_ready <= 1'b0;
						p2_cache_done_q <= 1'b1;
						if (refresh_pending) begin
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
							state_q <= ST_REFRESH_WAIT;
							wait_q <= count_after_command(T_RFC);
							refresh_debt_q <= consume_refresh_debt(refresh_debt_with_add);
						end
					end else if (refresh_service_now) begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						state_q <= ST_REFRESH_WAIT;
						wait_q <= count_after_command(T_RFC);
						refresh_debt_q <= consume_refresh_debt(refresh_debt_with_add);
					end else if (p0_req && p0_accept_i) begin
						// Accept a new request. Writes invalidate all
						// 8-bit caches because any byte lane may have changed.
						p0_busy_q <= 1'b1;
						p0_ready <= 1'b0;
						active_port_q <= 2'd0;
						active_size_q <= PORT0_SIZE[1:0];
						active_we_q <= p0_we;
						active_addr_q <= p0_aligned_addr;
						active_din_q <= p0_din;
						active_byte_en_q <= p0_byte_en;
						active_beats_q <= PORT0_BEATS;
						beat_q <= 3'd0;
						read_data_q <= 64'd0;
						client_done_q <= 1'b0;
						close_age_target_q <= 16'd0;
						cache0_valid_q <= p0_we ? 1'b0 : cache0_valid_q;
						cache1_valid_q <= p0_we ? 1'b0 : cache1_valid_q;
						cache2_valid_q <= p0_we ? 1'b0 : cache2_valid_q;
						SDRAM_A <= p0_aligned_addr[23:11];
						SDRAM_BA <= p0_aligned_addr[25:24];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
						state_q <= ST_RCD;
						wait_q <= count_after_command(T_RCD);
					end else if (p1_req && p1_accept_i) begin
						// p1 is accepted only when p0 is not requesting.
						p1_busy_q <= 1'b1;
						p1_ready <= 1'b0;
						active_port_q <= 2'd1;
						active_size_q <= PORT1_SIZE[1:0];
						active_we_q <= p1_we;
						active_addr_q <= p1_aligned_addr;
						active_din_q <= p1_din;
						active_byte_en_q <= p1_byte_en;
						active_beats_q <= PORT1_BEATS;
						beat_q <= 3'd0;
						read_data_q <= 64'd0;
						client_done_q <= 1'b0;
						close_age_target_q <= 16'd0;
						cache0_valid_q <= p1_we ? 1'b0 : cache0_valid_q;
						cache1_valid_q <= p1_we ? 1'b0 : cache1_valid_q;
						cache2_valid_q <= p1_we ? 1'b0 : cache2_valid_q;
						SDRAM_A <= p1_aligned_addr[23:11];
						SDRAM_BA <= p1_aligned_addr[25:24];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
						state_q <= ST_RCD;
						wait_q <= count_after_command(T_RCD);
					end else if (p2_req && p2_accept_i) begin
						// p2 is accepted only when p0 and p1 are idle.
						p2_busy_q <= 1'b1;
						p2_ready <= 1'b0;
						active_port_q <= 2'd2;
						active_size_q <= PORT2_SIZE[1:0];
						active_we_q <= p2_we;
						active_addr_q <= p2_aligned_addr;
						active_din_q <= p2_din;
						active_byte_en_q <= p2_byte_en;
						active_beats_q <= PORT2_BEATS;
						beat_q <= 3'd0;
						read_data_q <= 64'd0;
						client_done_q <= 1'b0;
						close_age_target_q <= 16'd0;
						cache0_valid_q <= p2_we ? 1'b0 : cache0_valid_q;
						cache1_valid_q <= p2_we ? 1'b0 : cache1_valid_q;
						cache2_valid_q <= p2_we ? 1'b0 : cache2_valid_q;
						SDRAM_A <= p2_aligned_addr[23:11];
						SDRAM_BA <= p2_aligned_addr[25:24];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
						state_q <= ST_RCD;
						wait_q <= count_after_command(T_RCD);
					end
				end

				ST_RCD: begin
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						// Column command. A10 requests auto-precharge so
						// the bank closes without spending an extra command
						// slot on an explicit PRECHARGE later.
						SDRAM_BA <= active_addr_q[25:24];
						if (active_we_q) begin
							SDRAM_A <= {write_beat_dqm(active_size_q, active_addr_q[0], active_byte_en_q, 2'd0), active_beats_q == 3'd1, beat_col(active_addr_q[10:1], 2'd0)};
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_WRITE;
							dq_out_q <= write_beat_data(active_size_q, active_din_q, 2'd0);
							dq_oe_q <= 1'b1;
							beat_q <= 3'd1;
							if (active_beats_q == 3'd1) begin
								client_done_q <= 1'b1;
								close_age_target_q <= max16(T_RC, add16_sat(active_age_q, T_DAL));
								unique case (active_port_q)
									2'd0: begin
										p0_busy_q <= 1'b0;
										p0_ready <= 1'b1;
									end
									2'd1: begin
										p1_busy_q <= 1'b0;
										p1_ready <= 1'b1;
									end
									default: begin
										p2_busy_q <= 1'b0;
										p2_ready <= 1'b1;
									end
								endcase
								state_q <= ST_RAS_WAIT;
							end else begin
								state_q <= ST_WRITE_BEATS;
							end
						end else begin
							SDRAM_A <= {2'b00, 1'b1, active_addr_q[10:1]};
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_READ;
							// CAS_READ_CYCLES includes the registered command path,
							// the DDIO capture, and the state edge that consumes it.
							wait_q <= count_after_command(CAS_READ_CYCLES);
							state_q <= ST_READ_LATENCY;
							beat_q <= 3'd0;
						end
					end
				end

				ST_READ_LATENCY: begin
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						read_data_q[15:0] <= dq_aligned_w;
						beat_q <= 3'd1;
						if (active_beats_q == 3'd1) begin
							client_done_q <= 1'b1;
							close_age_target_q <= T_READ_CLOSE;
							unique case (active_port_q)
								2'd0: begin
									p0_dout <= format_read_data(active_size_q, active_addr_q[0], {48'd0, dq_aligned_w});
									p0_busy_q <= 1'b0;
									p0_ready <= 1'b1;
									if (active_size_q == 2'd0) begin
										cache0_valid_q <= 1'b1;
										cache0_tag_q <= active_addr_q[25:1];
										cache0_data_q <= dq_aligned_w;
									end
								end
								2'd1: begin
									p1_dout <= format_read_data(active_size_q, active_addr_q[0], {48'd0, dq_aligned_w});
									p1_busy_q <= 1'b0;
									p1_ready <= 1'b1;
									if (active_size_q == 2'd0) begin
										cache1_valid_q <= 1'b1;
										cache1_tag_q <= active_addr_q[25:1];
										cache1_data_q <= dq_aligned_w;
									end
								end
								default: begin
									p2_dout <= format_read_data(active_size_q, active_addr_q[0], {48'd0, dq_aligned_w});
									p2_busy_q <= 1'b0;
									p2_ready <= 1'b1;
									if (active_size_q == 2'd0) begin
										cache2_valid_q <= 1'b1;
										cache2_tag_q <= active_addr_q[25:1];
										cache2_data_q <= dq_aligned_w;
									end
								end
							endcase
							if (active_beats_q < MODE_BEATS) begin
								state_q <= ST_READ_DRAIN;
								wait_q <= {13'd0,
									MODE_BEATS - active_beats_q - 3'd1};
							end else begin
								state_q <= ST_RAS_WAIT;
							end
						end else begin
							state_q <= ST_READ_BEATS;
						end
					end
				end

				ST_READ_BEATS: begin
					// SDR SDRAM returns one 16-bit word per clock in burst
					// mode. Assemble low-to-high words into the 64-bit bus.
					unique case (beat_q[1:0])
						2'd1: read_data_q[31:16] <= dq_aligned_w;
						2'd2: read_data_q[47:32] <= dq_aligned_w;
						default: read_data_q[63:48] <= dq_aligned_w;
					endcase

					if ((beat_q + 3'd1) >= active_beats_q) begin
						client_done_q <= 1'b1;
						close_age_target_q <= T_READ_CLOSE;
						unique case (active_port_q)
							2'd0: begin
								p0_dout <= format_read_data(active_size_q, active_addr_q[0], set_read_beat(read_data_q, beat_q[1:0], dq_aligned_w));
								p0_busy_q <= 1'b0;
								p0_ready <= 1'b1;
							end
							2'd1: begin
								p1_dout <= format_read_data(active_size_q, active_addr_q[0], set_read_beat(read_data_q, beat_q[1:0], dq_aligned_w));
								p1_busy_q <= 1'b0;
								p1_ready <= 1'b1;
							end
							default: begin
								p2_dout <= format_read_data(active_size_q, active_addr_q[0], set_read_beat(read_data_q, beat_q[1:0], dq_aligned_w));
								p2_busy_q <= 1'b0;
								p2_ready <= 1'b1;
							end
						endcase
						if (active_beats_q < MODE_BEATS) begin
							state_q <= ST_READ_DRAIN;
							wait_q <= {13'd0, MODE_BEATS - active_beats_q - 3'd1};
						end else begin
							state_q <= ST_RAS_WAIT;
						end
					end else begin
						beat_q <= beat_q + 3'd1;
					end
				end

				ST_READ_DRAIN: begin
					// Shorter reads in a wider programmed burst leave
					// harmless tail beats on DQ. Drain those beats so
					// the auto-precharge close timing is counted from
					// the real end of the SDRAM burst.
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						state_q <= ST_RAS_WAIT;
					end
				end

				ST_WRITE_BEATS: begin
					// Writes drive one 16-bit word per clock. For 8-bit
					// writes, A12:A11/DQM masks the byte not selected by
					// addr[0].
					if (beat_q < active_beats_q) begin
						SDRAM_A <= {write_beat_dqm(active_size_q, active_addr_q[0], active_byte_en_q, beat_q[1:0]), (beat_q + 3'd1) >= active_beats_q, beat_col(active_addr_q[10:1], beat_q[1:0])};
						SDRAM_BA <= active_addr_q[25:24];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_WRITE;
						dq_out_q <= write_beat_data(active_size_q, active_din_q, beat_q[1:0]);
						dq_oe_q <= 1'b1;
						if ((beat_q + 3'd1) >= active_beats_q) begin
							client_done_q <= 1'b1;
							close_age_target_q <= max16(T_RC, add16_sat(active_age_q, T_DAL));
							unique case (active_port_q)
								2'd0: begin
									p0_busy_q <= 1'b0;
									p0_ready <= 1'b1;
								end
								2'd1: begin
									p1_busy_q <= 1'b0;
									p1_ready <= 1'b1;
								end
								default: begin
									p2_busy_q <= 1'b0;
									p2_ready <= 1'b1;
								end
							endcase
							state_q <= ST_RAS_WAIT;
						end else begin
							beat_q <= beat_q + 3'd1;
						end
					end else begin
						close_age_target_q <= max16(T_RC, add16_sat(active_age_q, T_DAL));
						state_q <= ST_RAS_WAIT;
					end
				end

				ST_RAS_WAIT: begin
					if (active_age_q >= close_age_target_q) begin
						// READ and the final WRITE requested auto-precharge.
						// The close target is computed from aggregate
						// datasheet timing so we avoid double-rounding
						// tRAS/tWR/tRP into unnecessary extra clocks.
						if (pending_valid_q) begin
							active_port_q <= pending_port_q;
							active_size_q <= pending_size_q;
							active_we_q <= pending_we_q;
							active_addr_q <= pending_addr_q;
							active_din_q <= pending_din_q;
							active_byte_en_q <= pending_byte_en_q;
							active_beats_q <= pending_beats_q;
							beat_q <= 3'd0;
							read_data_q <= 64'd0;
							client_done_q <= 1'b0;
							close_age_target_q <= 16'd0;
							pending_valid_q <= 1'b0;
							active_age_q <= 16'd0;
							SDRAM_A <= pending_addr_q[23:11];
							SDRAM_BA <= pending_addr_q[25:24];
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
							state_q <= ST_RCD;
							wait_q <= count_after_command(T_RCD);
						end else begin
							client_done_q <= 1'b0;
							close_age_target_q <= 16'd0;
							state_q <= ST_IDLE;
						end
					end
				end

				ST_REFRESH_WAIT: begin
					if (wait_q != 16'd0) begin
						wait_q <= wait_q - 16'd1;
					end else begin
						state_q <= ST_IDLE;
					end
				end

				default: begin
					state_q <= ST_POWER_WAIT;
					init_done_q <= 1'b0;
					p0_busy_q <= 1'b1;
					p1_busy_q <= 1'b1;
					p2_busy_q <= 1'b1;
					p0_ready <= 1'b1;
					p1_ready <= 1'b1;
					p2_ready <= 1'b1;
					p0_cache_done_q <= 1'b0;
					p1_cache_done_q <= 1'b0;
					p2_cache_done_q <= 1'b0;
					wait_q <= count_after_command(T_POWER_WAIT);
				end
			endcase
		end
	end

endmodule

`default_nettype wire
