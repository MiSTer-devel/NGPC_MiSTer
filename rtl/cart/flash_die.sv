// Copyright (c) 2026 Jamie Blanks

// One physical NOR flash die on an NGP/NGPC cartridge.
//
// Everything the CPU can observe goes through the cartridge-edge pins: 21
// address lines, an 8-bit data bus split into DQ_in/DQ_out/DQ_oe, and the
// three strobes. Block map from the SNK FlashMem datasheet (p.5-7); the
// AMD/JEDEC command set, ID bytes and DQ7/DQ6/DQ5 status layout follow the
// protocol NGPC software uses -- it polls DQ7 and requires DQ5 to read 0.
//
// The array lives outside this module. mem_* is a tagged, held-request mailbox
// shaped like ngp_cart_sdram's p1 client, so ngp_cart passes it through with
// only address rebasing. A backing store must honour:
//
//   - mem_req is raised with mem_we/mem_addr/mem_wdata/mem_be/mem_lane stable
//     and held until mem_done. mem_tag toggles once per transaction, so a
//     store that caches by tag sees a repeat of the same address as a new
//     access.
//   - reads return the whole 16-bit word containing mem_addr in mem_rdata,
//     qualified by mem_rvalid at or before mem_done. mem_lane names the byte
//     the die wants; a byte-lane-selected read port (SDRAM p1 returns
//     {8'hff, byte}) must re-expand it into that half first.
//   - writes commit the lanes named by mem_be, visible to the very next read.
//
// A program or erase in progress returns an AMD status byte from the whole die
// instead of array data, as a single-plane NOR array does. That busy window is
// load-bearing: the BIOS erases and programs the reserved last block on every
// power-up and reads it back 325 ns after the 0x30 confirm cycle (one 2-state
// 8-bit cart cycle at 6.144 MHz), so without it the readback is stale and the
// work-area flash flags come out wrong. The same hole exists for a byte
// program, whose read-modify-write is ~350 ns against the same 325 ns; a real
// AMD-class part is busy 10-20 us there, so PROG_BUSY costs no fidelity.
//
// Erase is paced rather than throughput-limited: ERASE_WORD_PERIOD clk_sys
// ticks pass between committed words, so the busy window scales with block
// size. 75 ticks at 49.152 MHz reproduces the hardware erase-time family --
// 6.25 ms for 8 KB (hardware 5-15 ms), 25 ms for 32 KB (~20 ms), 50 ms for
// 64 KB (50-200 ms). Pacing also leaves the backing store's program/erase port
// idle ~93% of an erase, holding worst-case cart read latency at ~234 ns
// against a 325.52 ns budget; an unpaced fill misses it at ~346 ns.
//
// Two additions to a real die's port list. rd_ready qualifies a read that has
// to fetch through the SDRAM mailbox and cannot answer in zero time; it
// carries no data and reads 1 whenever the answer is internal (status, ID,
// absent die). DEVICE_ID and SIZE_MASK are runtime inputs (cfg_device_id /
// cfg_size_mask) rather than parameters because the die population is decoded
// from the downloaded image size; they are strapping, stable except across a
// cart load, and so restored by the cart load rather than by a savestate.

module flash_die
#(
	parameter [7:0]   MANUFACTURER_ID   = 8'h98,   // Toshiba
	parameter [7:0]   ID_PROTECT_BYTE   = 8'h02,   // offset +2: sources conflict
	                                               // between 0x02 and 0x00,
	                                               // parameterised on purpose
	parameter [7:0]   ID_TRAILER_BYTE   = 8'h80,   // offset +3
	parameter [21:0]  ERASE_WORD_PERIOD = 22'd75,  // clk_sys ticks per erased word
	parameter [9:0]   SS_BASE           = 10'd96   // internals words SS_BASE+0/+1
)
(
	input  wire        clk,            // clk_sys, 49.152 MHz
	input  wire        ce,             // whole-module enable; low freezes the die
	input  wire        reset,
	// Assert only after a pause-drained sparse overlay restore. The array has
	// already changed below the die, so return command/ID decode to READ before
	// the CPU can observe it again.
	input  wire        force_read_i,

	input  wire        present,        // 0 = no die in this socket position
	input  wire [7:0]  cfg_device_id,  // 0xAB / 0x2C / 0x2F
	input  wire [20:0] cfg_size_mask,  // 0x07FFFF / 0x0FFFFF / 0x1FFFFF

	// --- cartridge edge pins ---------------------------------------------
	input  wire [20:0] A,
	input  wire [7:0]  DQ_in,
	output wire [7:0]  DQ_out,
	output wire        DQ_oe,
	output wire        rd_ready,       // read answer is available this cycle
	input  wire        nCE,
	input  wire        nOE,
	input  wire        nWE,

	// --- backing-store port (the die's own array) -------------------------
	output reg         mem_req,
	output reg         mem_we,
	output reg  [20:0] mem_addr,
	output reg  [15:0] mem_wdata,
	output reg  [1:0]  mem_be,
	output reg         mem_lane,
	output reg         mem_tag,
	input  wire [15:0] mem_rdata,
	input  wire        mem_rvalid,
	input  wire        mem_done,

	// --- status to the connector / save layer -----------------------------
	output wire        busy,
	output wire        dirty_pulse,
	output wire [5:0]  dirty_block,

	// --- savestate tap ----------------------------------------------------
	input  wire [9:0]  ss_bus_adr,
	input  wire [63:0] ss_bus_din,
	input  wire        ss_bus_wren,
	input  wire        ss_bus_rst,
	output wire [63:0] ss_bus_dout,
	input  wire        ss_restore_is_rewind,
	input  wire        pause_req,
	output wire        pause_ready
);

	// Command FSM states.
	localparam [3:0] ST_READ      = 4'd0;   // array data on DQ
	localparam [3:0] ST_UNLOCK1   = 4'd1;   // saw 5555 <- AA
	localparam [3:0] ST_UNLOCK2   = 4'd2;   // saw 2AAA <- 55
	localparam [3:0] ST_CMD       = 4'd3;   // a two-stage command byte is latched
	localparam [3:0] ST_ID        = 4'd4;   // autoselect / ID read
	localparam [3:0] ST_PROG_ARM  = 4'd5;   // saw A0; next write is the target
	localparam [3:0] ST_PROG_BUSY = 4'd6;   // read-modify-write in flight
	localparam [3:0] ST_ERASE     = 4'd7;   // block fill in flight
	localparam [3:0] ST_PROTECT   = 4'd8;   // 9A twice: sequenced, no effect

	// Embedded-algorithm sub-FSM. A step is "needed or in flight"; mem_req
	// distinguishes the two.
	localparam [2:0] OP_IDLE       = 3'd0;
	localparam [2:0] OP_PROG_RD    = 3'd1;
	localparam [2:0] OP_PROG_WR    = 3'd2;
	localparam [2:0] OP_ERASE_PACE = 3'd3;
	localparam [2:0] OP_ERASE_WR   = 3'd4;

	// Owner of the single backing-store port.
	localparam [1:0] OWN_NONE  = 2'd0;
	localparam [1:0] OWN_ARRAY = 2'd1;
	localparam [1:0] OWN_OP    = 2'd2;

	localparam [21:0] PACE_LIMIT = ERASE_WORD_PERIOD - 22'd1;

	reg  [3:0]  state;
	reg  [7:0]  cmd_latch;
	reg  [7:0]  prog_data;
	reg  [7:0]  prog_old;
	reg  [20:0] prog_addr;
	reg  [20:0] erase_base;
	reg  [20:0] fill_addr;
	reg  [21:0] pace_cnt;
	reg  [2:0]  op_state;
	reg  [1:0]  mem_owner;
	reg  [7:0]  rd_byte;
	reg         rd_valid_q;
	reg         toggle_q;
	reg         busy_q;
	reg         dirty_q;
	reg  [5:0]  dirty_block_q;

	// Pin sampling
	// The die is asynchronous logic on real hardware. clk_sys is 49.152 MHz,
	// 8x the 6.144 MHz bus state rate, so sampling the pins sees every strobe
	// edge with margin. A write cycle latches address and data on the rising
	// edge of nWE while nCE is low (WE-controlled write, the AMD convention)
	// -- implemented as "the last values seen while the write strobe was
	// low". A real AMD part latches the address on the FALLING edge instead;
	// the two differ only if the address moves inside a write cycle, which no
	// TLCS-900/H bus cycle does.

	wire [20:0] a_eff     = A & cfg_size_mask;
	wire        wr_active = present && !nCE && !nWE;
	wire        rd_active = present && !nCE && !nOE && nWE;

	reg         wr_active_q;
	reg         rd_active_q;
	reg  [20:0] a_rd_q;
	reg  [20:0] wa_q;
	reg  [7:0]  wd_q;

	always @(posedge clk) begin
		if (reset) begin
			wr_active_q <= 1'b0;
			rd_active_q <= 1'b0;
			a_rd_q      <= 21'd0;
			wa_q        <= 21'd0;
			wd_q        <= 8'd0;
		end else if (ce) begin
			wr_active_q <= wr_active;
			rd_active_q <= rd_active;
			a_rd_q      <= a_eff;
			if (wr_active) begin
				wa_q <= a_eff;
				wd_q <= DQ_in;
			end
		end
	end

	wire wr_pulse = wr_active_q && !wr_active;
	wire rd_start = rd_active && (!rd_active_q || (a_eff != a_rd_q));
	wire rd_end   = rd_active_q && (!rd_active || (a_eff != a_rd_q));

	// Block map: comparators and masks, no division, no modulus
	// FlashMem p.5-6: uniform 64 KB blocks except the top 64 KB, which
	// splits 32 KB / 8 KB / 8 KB / 16 KB. cfg_size_mask[20:16] is the A[20:16]
	// value of that top region (07 for 0xAB, 0F for 0x2C, 1F for 0x2F), and
	// that single constant drives the whole decode.

	// The offset mask of the block containing an address, bits [20:1] only:
	// bit 0 is 1 for every block size, so word addressing never needs it and
	// block membership never depends on it. `ah` is address bits [20:14].
	function automatic [19:0] blk_off_mask(input [6:0] ah, input [4:0] top);
	begin
		if (ah[6:2] != top)  blk_off_mask = 20'h07FFF;   // uniform 64 K
		else if (!ah[1])     blk_off_mask = 20'h03FFF;   // top 32 K
		else if (!ah[0])     blk_off_mask = 20'h00FFF;   // the two 8 K
		else                 blk_off_mask = 20'h01FFF;   // top 16 K
	end
	endfunction

	// Index 0..34 for the dirty bitmap, from address bits [20:13]. One 6-bit
	// adder and three comparators: 31+3 = 34 for a 16 Mbit die, 15+3 = 18 for
	// 8 Mbit, 7+3 = 10 for 4 Mbit, which is FlashMem's table row for row.
	function automatic [5:0] blk_index(input [7:0] ah, input [4:0] top);
		reg [1:0] sub;
	begin
		if (ah[7:3] != top) begin
			blk_index = {1'b0, ah[7:3]};
		end else begin
			if (!ah[2])      sub = 2'd0;
			else if (!ah[1]) sub = {1'b0, ah[0]} + 2'd1;
			else             sub = 2'd3;
			blk_index = {1'b0, top} + {4'd0, sub};
		end
	end
	endfunction

	// What the die presents on DQ

	wire busy_w  = (state == ST_PROG_BUSY) || (state == ST_ERASE);
	wire erase_w = (state == ST_ERASE);

	// scope_off_mask is the erasing block's offset mask, so erase_end_word is
	// that block's last word address.
	wire [19:0] scope_off_mask = blk_off_mask(erase_base[20:14], cfg_size_mask[20:16]);
	wire [19:0] erase_end_word = erase_base[20:1] | scope_off_mask;

	wire status_sel = busy_w;
	wire id_sel     = present && (state == ST_ID);

	// AMD status byte. DQ7 is data# polling, DQ6 the toggle bit, DQ5
	// hardwired 0: NGPC save stubs abort if the "exceeded internal timing
	// limit" flag ever reads 1.
	wire [7:0] status_byte = {erase_w ? 1'b0 : ~prog_data[7], toggle_q, 6'd0};

	// ID read: selected by a_eff[1:0] with every higher address bit ignored,
	// so both access patterns the BIOS uses (offset 0 and the last block's
	// base) are answered without ever mutating the array.
	reg [7:0] id_byte;
	always @* begin
		case (a_eff[1:0])
			2'd0:    id_byte = MANUFACTURER_ID;
			2'd1:    id_byte = cfg_device_id;
			2'd2:    id_byte = ID_PROTECT_BYTE;
			default: id_byte = ID_TRAILER_BYTE;
		endcase
	end

	// A fetch is wanted whenever the current read event still has no array
	// byte. Level-based, so a read event that starts while the die is busy
	// and outlives the busy window still gets its data.
	wire array_valid_w = rd_valid_q && !rd_start;
	wire want_array    = rd_active && !status_sel && !id_sel && !array_valid_w;

	assign DQ_out = !present  ? 8'hFF        :
	                status_sel ? status_byte :
	                id_sel     ? id_byte     : rd_byte;

	assign DQ_oe = rd_active;

	// rd_ready must fall the instant a new read event starts, or the previous
	// event's byte would be handed out as this one's answer -- the same stale
	// readback in miniature. array_valid_w carries that qualification.
	assign rd_ready = !present || status_sel || id_sel || array_valid_w;

	assign busy        = busy_w;
	assign dirty_pulse = dirty_q;
	assign dirty_block = dirty_block_q;

	wire unlock1_w = (wa_q[14:0] == 15'h5555);
	wire unlock2_w = (wa_q[14:0] == 15'h2AAA);

	wire [7:0] prog_new = prog_old & prog_data;

	// Savestate slice (internals SS_BASE+0 and +1)
	// The dirty bitmap lives in ngp_cart, so ngp_cart owns words SS_BASE+2
	// and +3 and this module owns +0 and +1. The erase end address is not
	// stored: it is a pure function of the block base.
	//
	// Transactions drain before pause_ready, so a capture never sees a busy
	// state. A restored busy state is therefore either a foreign or a corrupt
	// slot; it is forced to READ.

	wire ss_hit0 = (ss_bus_adr == SS_BASE);
	wire ss_hit1 = (ss_bus_adr == (SS_BASE + 10'd1));

	wire [63:0] ss_word0 = {21'd0, present, prog_data, prog_addr, toggle_q, cmd_latch, state};
	wire [63:0] ss_word1 = {pace_cnt, fill_addr, erase_base};

	assign ss_bus_dout = ss_hit0 ? ss_word0 :
	                     ss_hit1 ? ss_word1 : 64'd0;

	wire [3:0] ss_state_in   = ss_bus_din[3:0];
	wire       ss_state_busy = (ss_state_in == ST_PROG_BUSY) || (ss_state_in == ST_ERASE);

	// Command FSM, embedded algorithms and the backing-store port
	// One clocked block, so every register has exactly one driver. The
	// savestate restore sits ahead of the ce gate: a paused core may have its
	// enables gated off entirely while the engine broadcasts internals.

	always @(posedge clk) begin
		if (reset || ss_bus_rst) begin
			state         <= ST_READ;
			cmd_latch     <= 8'h00;
			prog_data     <= 8'h00;
			prog_old      <= 8'hFF;
			prog_addr     <= 21'd0;
			erase_base    <= 21'd0;
			fill_addr     <= 21'd0;
			pace_cnt      <= 22'd0;
			op_state      <= OP_IDLE;
			mem_owner     <= OWN_NONE;
			mem_req       <= 1'b0;
			mem_we        <= 1'b0;
			mem_addr      <= 21'd0;
			mem_wdata     <= 16'd0;
			mem_be        <= 2'b00;
			mem_lane      <= 1'b0;
			mem_tag       <= 1'b0;
			rd_byte       <= 8'hFF;
			rd_valid_q    <= 1'b0;
			dirty_q       <= 1'b0;
			// toggle_q and busy_q have their own block below.
			dirty_block_q <= 6'd0;
		end else if (force_read_i && !mem_req) begin
			state      <= ST_READ;
			cmd_latch  <= 8'h00;
			op_state   <= OP_IDLE;
			rd_valid_q <= 1'b0;
		end else if (ss_bus_wren && (ss_hit0 || ss_hit1)) begin
			// A rewind snapshot carries no flash array, so restoring its FSM
			// onto an array that has moved on would be incoherent. "Array is
			// whatever it is, die is idle" is the only self-consistent state.
			if (ss_restore_is_rewind) begin
				state      <= ST_READ;
				cmd_latch  <= 8'h00;
				op_state   <= OP_IDLE;
				rd_valid_q <= 1'b0;
			end else if (ss_hit0) begin
				state      <= ss_state_busy ? ST_READ : ss_state_in;
				cmd_latch  <= ss_bus_din[11:4];
				prog_addr  <= ss_bus_din[33:13];
				prog_data  <= ss_bus_din[41:34];
				op_state   <= OP_IDLE;
				rd_valid_q <= 1'b0;
			end else begin
				erase_base <= ss_bus_din[20:0];
				fill_addr  <= ss_bus_din[41:21];
				pace_cnt   <= ss_bus_din[63:42];
			end
		end else if (ce) begin
			dirty_q <= 1'b0;

			// An empty socket cannot be in a command state. The port still
			// finishes any transaction it owes the store.
			if (!present && !mem_req) begin
				state     <= ST_READ;
				cmd_latch <= 8'h00;
				op_state  <= OP_IDLE;
			end

			// --- backing store: capture, complete, advance ----------------
			if (mem_req && mem_rvalid && !mem_we) begin
				if (mem_owner == OWN_ARRAY) begin
					rd_byte <= mem_lane ? mem_rdata[15:8] : mem_rdata[7:0];
				end else begin
					prog_old <= mem_lane ? mem_rdata[15:8] : mem_rdata[7:0];
				end
			end

			if (mem_req && mem_done) begin
				mem_req   <= 1'b0;
				mem_owner <= OWN_NONE;
				if (mem_owner == OWN_ARRAY) begin
					rd_valid_q <= 1'b1;
				end else begin
					case (op_state)
						OP_PROG_RD: op_state <= OP_PROG_WR;
						OP_PROG_WR: begin
							op_state      <= OP_IDLE;
							state         <= ST_READ;
							dirty_q       <= 1'b1;
							dirty_block_q <= blk_index(prog_addr[20:13], cfg_size_mask[20:16]);
						end
						OP_ERASE_WR: begin
							if (fill_addr[20:1] == erase_end_word) begin
								op_state      <= OP_IDLE;
								state         <= ST_READ;
								dirty_q       <= 1'b1;
								dirty_block_q <= blk_index(erase_base[20:13], cfg_size_mask[20:16]);
							end else begin
								fill_addr <= fill_addr + 21'd2;
								op_state  <= OP_ERASE_PACE;
							end
						end
						default: op_state <= OP_IDLE;
					endcase
				end
			end

			// --- erase pacing --------------------------------------------
			// The pace counter free-runs across the whole erase rather than
			// restarting on each committed word, so exactly one word leaves
			// per ERASE_WORD_PERIOD ticks and the backing store's latency
			// hides inside the period. A store slower than the period just
			// degrades the fill to throughput-limited.
			if (erase_w) begin
				if (pace_cnt >= PACE_LIMIT) begin
					pace_cnt <= 22'd0;
					if (op_state == OP_ERASE_PACE) op_state <= OP_ERASE_WR;
				end else begin
					pace_cnt <= pace_cnt + 22'd1;
				end
			end

			// --- backing store: issue ------------------------------------
			// The CPU's array read wins the port: it is the latency-critical
			// path, and because a busy die answers every read with status it
			// can never collide with an embedded algorithm anyway.
			if (!mem_req && (mem_owner == OWN_NONE)) begin
				if (want_array) begin
					mem_req   <= 1'b1;
					mem_we    <= 1'b0;
					mem_addr  <= {a_eff[20:1], 1'b0};
					mem_wdata <= 16'd0;
					mem_be    <= 2'b00;
					mem_lane  <= a_eff[0];
					mem_tag   <= ~mem_tag;
					mem_owner <= OWN_ARRAY;
				end else if (op_state == OP_PROG_RD) begin
					mem_req   <= 1'b1;
					mem_we    <= 1'b0;
					mem_addr  <= {prog_addr[20:1], 1'b0};
					mem_wdata <= 16'd0;
					mem_be    <= 2'b00;
					mem_lane  <= prog_addr[0];
					mem_tag   <= ~mem_tag;
					mem_owner <= OWN_OP;
				end else if (op_state == OP_PROG_WR) begin
					// A programmed bit can only go 1 -> 0: new = old AND data.
					mem_req   <= 1'b1;
					mem_we    <= 1'b1;
					mem_addr  <= {prog_addr[20:1], 1'b0};
					mem_wdata <= {prog_new, prog_new};
					mem_be    <= prog_addr[0] ? 2'b10 : 2'b01;
					mem_lane  <= prog_addr[0];
					mem_tag   <= ~mem_tag;
					mem_owner <= OWN_OP;
				end else if (op_state == OP_ERASE_WR) begin
					mem_req   <= 1'b1;
					mem_we    <= 1'b1;
					mem_addr  <= {fill_addr[20:1], 1'b0};
					mem_wdata <= 16'hFFFF;
					mem_be    <= 2'b11;
					mem_lane  <= 1'b0;
					mem_tag   <= ~mem_tag;
					mem_owner <= OWN_OP;
				end
			end

			// --- command FSM ---------------------------------------------
			// One completed write cycle per transition. Writes are ignored
			// while an embedded algorithm runs, which is the AMD "algorithm
			// in progress" behaviour.
			if (wr_pulse && !busy_w) begin
				case (state)
					ST_READ: begin
						// A bare F0 reset write is also accepted here and has
						// no further effect: the die is already in read-array
						// state. Some software uses that form; the unlocked
						// form below is the common one.
						cmd_latch <= 8'h00;
						if (unlock1_w && (wd_q == 8'hAA)) begin
							state <= ST_UNLOCK1;
						end
					end

					ST_UNLOCK1: begin
						if (unlock2_w && (wd_q == 8'h55)) state <= ST_UNLOCK2;
						else                              state <= ST_READ;
					end

					ST_UNLOCK2: begin
						if (wd_q == 8'h30) begin
							// Erase confirm is accepted at any address; the
							// address picks the block.
							if (cmd_latch == 8'h80) begin
								state      <= ST_ERASE;
								erase_base <= {wa_q[20:1] &
								               ~blk_off_mask(wa_q[20:14], cfg_size_mask[20:16]), 1'b0};
								fill_addr  <= {wa_q[20:1] &
								               ~blk_off_mask(wa_q[20:14], cfg_size_mask[20:16]), 1'b0};
								pace_cnt   <= 22'd0;
								op_state   <= OP_ERASE_PACE;
								cmd_latch  <= 8'h00;
							end else begin
								state <= ST_READ;
							end
						end else if (unlock1_w) begin
							case (wd_q)
								8'h80: begin
									state     <= ST_CMD;
									cmd_latch <= 8'h80;
								end
								8'h90: state <= ST_ID;
								8'h9A: begin
									// Block protect. Irreversible and
									// development-slot only per the SysCall
									// manual; no retail software issues it,
									// so it is sequenced and does nothing.
									if (cmd_latch == 8'h9A) begin
										state <= ST_PROTECT;
									end else begin
										state     <= ST_CMD;
										cmd_latch <= 8'h9A;
									end
								end
								8'hA0: state <= ST_PROG_ARM;
								8'hF0: state <= ST_READ;
								// Chip erase: accepted and ignored. No retail
								// software issues it, and filling die 0 here
								// would destroy the game image.
								8'h10:   state <= ST_READ;
								default: state <= ST_READ;
							endcase
						end else begin
							state <= ST_READ;
						end
					end

					ST_CMD: begin
						if (unlock1_w && (wd_q == 8'hAA)) begin
							state <= ST_UNLOCK1;
						end else begin
							state     <= ST_READ;
							cmd_latch <= 8'h00;
						end
					end

					ST_ID: begin
						cmd_latch <= 8'h00;
						if (unlock1_w && (wd_q == 8'hAA)) state <= ST_UNLOCK1;
						else                              state <= ST_READ;
					end

					ST_PROG_ARM: begin
						state     <= ST_PROG_BUSY;
						prog_addr <= wa_q;
						prog_data <= wd_q;
						prog_old  <= 8'hFF;
						op_state  <= OP_PROG_RD;
						cmd_latch <= 8'h00;
					end

					ST_PROTECT: state <= ST_READ;

					default:    state <= ST_READ;
				endcase
			end

			// --- read event bookkeeping ----------------------------------
			// Last, so a read event that starts in the same tick a fetch
			// lands discards that byte instead of answering with it.
			if (rd_start) rd_valid_q <= 1'b0;
		end
	end

	// toggle_q flips once per completed read cycle and only while busy, so an
	// AMD toggle-bit loop sees 00, 40, 00, 40 ... during the fill and then a
	// steady array byte the moment it finishes. It is not free-running.
	always @(posedge clk) begin
		if (reset || ss_bus_rst) begin
			toggle_q <= 1'b0;
			busy_q   <= 1'b0;
		end else if (ss_bus_wren && ss_hit0) begin
			toggle_q <= ss_restore_is_rewind ? 1'b0 : ss_bus_din[12];
			busy_q   <= 1'b0;
		end else if (ss_bus_wren && ss_hit1) begin
			busy_q <= 1'b0;
		end else if (ce) begin
			busy_q <= busy_w;
			if (busy_w && !busy_q)   toggle_q <= 1'b0;
			else if (rd_end && busy_w) toggle_q <= ~toggle_q;
		end
	end

	// Drain rule: no transaction in flight, no program commit outstanding,
	// no erase fill outstanding. The erase drain
	// is the long pole -- up to 50 ms for a 64 KB block -- but it is bounded
	// and rare, and draining is far less state than capturing a fill walker
	// mid-flight and resuming it.
	assign pause_ready = pause_req && !busy_w && !mem_req && (op_state == OP_IDLE);

endmodule
