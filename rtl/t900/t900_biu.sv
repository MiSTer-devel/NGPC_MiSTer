// Copyright (c) 2026 Jamie Blanks

// TLCS-900/H bus interface unit.  Owns the external bus, the 4-byte prefetch
// instruction queue and the dynamic bus sizing.  Everything advances on `ce`
// (one "state" per ce).  A zero-wait bus cycle costs two states (T1 address,
// T2 data), and back-to-back cycles run with no dead state between them
// (TMP95C061 bus timing).
//
// What the bus does next is arbitrated at the end of T2 and takes effect
// immediately: the next cycle's T1 is the following state.  That covers all
// three continuations -- the next cycle of a data access, the next prefetch
// fill, and a data request that arrived while a fill held the bus.
//
// Starting a cycle from an IDLE bus is asymmetric between the two clients:
//
//  - a DATA cycle starts in the state its request is presented.  The address
//    unit is already driving the operand address, so that state IS the cycle's
//    T1 and the bus signals are driven combinationally in it (`d_bus_t1`).  An
//    idle-bus data access therefore costs T1 + T2.
//  - a PREFETCH FILL launched from an idle bus still spends its own issue state
//    loading the fetch pointer (BC_IDLE -> BC_T1).  Fills chain out of T2 at
//    full rate, so that state is only ever paid right after a queue flush --
//    which is why a taken control transfer costs a whole bus cycle of refill
//    bubble.  Fills must not start in their presented state: doing so cancels
//    the taken-transfer cost exactly.
//
// The data-side half buys BUS SLACK, not a cheaper instruction.  Instruction
// cost is still the table budget in t900_seq.sv, and that budget still assumes
// the issue state exists (`nominal_states` is deliberately unchanged, and
// `mem_ticks` is seeded to 2 in the new T1 state so every wait-state penalty is
// charged exactly as before).  What changes is that the bus is free for a state
// the instruction is still being charged for, which is where a slow region's
// wait states disappear to on silicon and why LDIR stays 7n+1 there while a
// character-RAM store costs +2.
//
// The one state that is genuinely not a bus state is a queue flush WITH A CYCLE
// IN FLIGHT.  A combinational redirect arriving on a genuinely idle bus is the
// exception: that flush state is the new stream's first fetch T1 (`r_bus_t1`),
// because silicon recovers the state in the queue-full shape.
//
// External bus contract, consumed by the SoC fabric:
//   - plan_addr is the address of the next cycle the BIU intends to issue
//     (combinational): the data port's address when one is pending, else the
//     next prefetch fill address.  While a fill holds the bus it is already the
//     address of the fill that would follow, so a chained fill gets the right
//     region attribute.  The fabric answers bus_width8 for plan_addr
//     combinationally (region attribute from the CS/WAIT controller) and must
//     derive wait states from bus_addr, the address actually on the bus.
//   - bus_req holds addr/be/we/wdata stable from T1 until bus_rdy.
//   - bus_rdy is sampled in T2; low extends the cycle one state at a time.
//   - 8-bit regions transfer on bits [7:0] only with bus_be = 01.  In 16-bit
//     regions an odd single byte rides bits [15:8] with be = 10.
//
// The queue is a 4-byte FIFO filled from spare bus slots (the data port has
// priority), word-wide from even addresses in 16-bit regions, flushed only by
// q_flush on a taken control transfer (CPU900H p.1, "only when branch, CPU
// fetch branch destination code").  A fill starts whenever a slot is free,
// word-wide when two are free and the address allows it, and fills chain out of
// T2, so the sustained fetch rate is one byte per state -- what Toshiba's
// instruction timings assume.  Holding a word-capable fill back until two slots
// are free is worse, not better: the queue is only four deep, so a five-byte
// instruction needs the byte the single free slot can hold and waiting for the
// second slot costs it a state (LD R,#imm32 and similar five-byte forms).
//
// The data port always wins the arbitration it is present for.  It cannot
// preempt a cycle already on the bus, so a fill that started in the state before
// the request still costs it up to two states, which is the same bus the real
// BIU has to wait for.
//
// Back-to-back data accesses cost nothing between them: the state ddone pulses
// in is the state the next access starts in, so two accesses are 2 x (T1 + T2)
// and no more.  LDIR needs exactly that -- it issues two accesses per iteration
// and Toshiba's 7n+1 leaves three states of slack around them, not none.  Three
// rules make it work, and all three are load-bearing:
//  - the sequencer DROPS dreq in the state ddone pulses in unless it has a new
//    access to present.  Holding it there with the finished access's address
//    makes the BIU run that access twice -- a duplicate write, or a read landing
//    on top of the data just returned.
//  - the sequencer gives a newly presented request priority on
//    daddr/dwe/dbytes/dwdata.  Its `mem_pend` still reads 1 in that state, so
//    choosing on it hands over the completed access's address.
//  - the prefetcher does not take the bus while dreq is asserted (want_fill is
//    gated on !dreq): a fill started there would own the bus for the two states
//    the next access wanted.  The fill that chains out of a data cycle's own T2
//    cannot use that gate, since dreq is still high there for the access that is
//    finishing, so the sequencer states it one state early on prefetch_hold
//    instead.  See fill_chain_after_data.
// With those in place start_data needs no !ddone gate; a request seen in the
// ddone state is always a new access.
//
// No store snooping: a write near PC does not invalidate already-fetched bytes.
//
// Dynamic bus sizing is asked PER BUS CYCLE, not per access.  Toshiba's table
// 6.1 splits an operand access into bus cycles by "the current area width and
// the address alignment" (CPU900H p.30-31), and the area is whatever the
// CS/WAIT controller decodes for the address actually on the bus -- that
// controller has no notion of an access.  The SoC has three adjacent pairs of
// different width (see rtl/soc/k2_soc_fabric.sv: 006FFF/007000, 007FFF/008000
// and FEFFFF/FF0000), and a long transfer across any of them must neither lose
// nor invent a byte, so `plan_addr` presents the NEXT chained address while a
// data cycle is in T2 with bytes left, and `chain_next` takes both the width and
// the byte enables from the answer.

module t900_biu
#(
	// Hold the external bus outputs on the opposite phase of `ce`, splitting
	// the CPU-to-fabric chain into two timed halves. See the note beside the
	// hold at the bottom of this file. Requires that `ce` is never high on two
	// consecutive clk_sys cycles, so it defaults to 0 for benches that pulse
	// `ce` every cycle; t900_cpu drives it from SPLIT_RF_READ, which carries
	// the identical precondition.
	parameter SPLIT_BUS_OUT  = 0
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	// Data port (sequencer): the request is held stable from issue until the
	// state ddone pulses in, and in that state it is either down or already
	// describing the next access. See the handshake note above.
	input  wire        dreq,
	input  wire        dwe,
	input  wire [23:0] daddr,
	input  wire [2:0]  dbytes,      // 1, 2 or 4
	input  wire [31:0] dwdata,
	output reg  [31:0] drdata,
	output reg         ddone,

	// Prefetch queue (decoder side)
	input  wire        q_flush,
	input  wire [23:0] q_new_pc,
	input  wire        q_redirect,
	input  wire        q_pop,       // consume one byte this state
	// A data request becomes valid next state: either an address that
	// completes next state (source-class (#16)), or the second of a
	// back-to-back pair, which is presented in the state ddone pulses in.
	input  wire        prefetch_hold,
	input  wire        prefetch_chain_hold,
	output wire [7:0]  q_byte,
	output wire [2:0]  q_count,

	// External bus
	output wire [23:0] plan_addr,
	input  wire        bus_width8,
	output wire        bus_req,
	output wire        bus_we,
	output wire [23:0] bus_addr,
	output wire [1:0]  bus_be,
	output wire [15:0] bus_wdata,
	input  wire [15:0] bus_rdata,
	input  wire        bus_rdy,

	// High in the state an idle bus takes a DATA request, i.e. that cycle's
	// T1. The sequencer's bus-penalty measurement keys on the state the BIU
	// actually takes the request, and it cannot read that off `bus_req` any
	// more: `bus_req` is asserted combinationally in this very state, so
	// `!bus_req` no longer means "idle with a request waiting".
	output wire        d_bus_t1
);

	localparam [1:0] BC_IDLE = 2'd0;
	localparam [1:0] BC_T1   = 2'd1;
	localparam [1:0] BC_T2   = 2'd2;

	// Registered halves of the external bus signals. They hold the cycle from
	// its T1 through T2; in the one state a data cycle starts from an idle bus
	// they are still the previous cycle's, so the outputs below take the live
	// values from the data port instead.
	reg        bus_req_r;
	reg        bus_we_r;
	reg [23:0] bus_addr_r;
	reg [1:0]  bus_be_r;
	reg [15:0] bus_wdata_r;

	reg [1:0]  bc_state;
	reg        bc_for_data;
	reg        bc_two_bytes;
	reg        bc_width8;     // region attribute latched at issue
	reg        redirect_pending;

	// Data transfer bookkeeping. d_addr/d_left describe the cycle currently
	// on the bus; d_pos is the assembly position of its first byte.
	reg        d_active;
	reg [23:0] d_addr;
	reg [2:0]  d_left;
	reg [1:0]  d_pos;
	reg [31:0] d_wdata;

	// Prefetch queue
	reg [7:0]  queue [0:3];
	// Per-queue-byte region width. Read only by simulation instrumentation,
	// hence the lint waiver.
	/* verilator lint_off UNUSEDSIGNAL */
	reg [3:0]  queue_width8;
	/* verilator lint_on UNUSEDSIGNAL */
	reg [1:0]  q_rd;
	reg [2:0]  q_valid;
	reg [23:0] fill_pc;
	reg        fill_epoch;
	reg        bc_epoch;

	assign q_byte   = queue[q_rd];
	assign q_count  = q_valid;

	wire       pop_now = q_pop && (q_valid != 3'd0);
	wire [2:0] q_free  = 3'd4 - q_valid;

	// Where the fill stream goes next. While a fill holds the bus, fill_pc
	// still points at it, so the address after it is the one a chained fill
	// would use -- and the one the region lookup has to answer for.
	wire [2:0]  fill_bytes   = bc_two_bytes ? 3'd2 : 3'd1;
	wire [23:0] fill_pc_next = fill_pc + {21'd0, fill_bytes};
	wire        in_fill      = (bc_state != BC_IDLE) && !bc_for_data;

	// Address the next bus cycle will use, for region-width lookup.
	//
	// `ddone` is deliberately NOT a gate here. The data port drops its request
	// in the state ddone pulses in and gives a newly presented one priority on
	// daddr/dwe/dbytes/dwdata, so a request seen in that state is always a new
	// access and never the one just finished. That is what lets two
	// back-to-back accesses run with no handshake state between them, which is
	// what LDIR's 7n+1 needs. Re-adding the gate does not corrupt anything --
	// it just costs a state per access.
	wire start_data = dreq && !d_active;

	// The address the next chained cycle of the current access will use, and
	// whether there is going to be one. Presenting it on `plan_addr` during T2
	// is what lets the fabric answer `bus_width8` for the address the chain is
	// about to drive rather than for the one it is leaving.
	wire [23:0] d_next_addr = d_addr + (bc_two_bytes ? 24'd2 : 24'd1);
	wire        d_chaining  = d_active && bc_for_data && (bc_state == BC_T2) &&
	                          (bc_two_bytes ? (d_left > 3'd2) : (d_left > 3'd1));

	// A data cycle in its LAST T2 has nothing more to ask for itself, so the
	// address the bus will drive next is the fill pointer. Present it, exactly
	// as an in-flight fill already presents the address of the fill that would
	// follow it, so a fill chaining out of this T2 gets the right region width.
	// The cycle on the bus is unaffected: its own width was latched into
	// `bc_width8` at issue.
	wire        d_final_t2  = bc_for_data && (bc_state == BC_T2) && !d_chaining;

	// A fill completing under a STALE epoch plans the NEW stream's first
	// fetch (fill_pc), never its own successor -- the flush may have come
	// through the registered path with no q_redirect, so this cannot lean
	// on `redirect_pending`.
	wire        stale_fill_t2 = !bc_for_data && (bc_state == BC_T2) &&
	                            (bc_epoch != fill_epoch);

	assign plan_addr = d_chaining ? d_next_addr :
	                   start_data ? daddr :
	                   q_redirect ? q_new_pc :
	                   redirect_pending ? fill_pc :
	                   stale_fill_t2 ? fill_pc :
	                   d_final_t2 ? fill_pc :
	                   d_active   ? d_addr :
	                   in_fill    ? fill_pc_next : fill_pc;

	wire        plan_word_data = !bus_width8 && !plan_addr[0];

	// An idle bus takes a data request in the state it is presented, so THIS
	// state is that cycle's T1 and the bus must be driven in it. The registered
	// halves still hold the previous cycle, so the outputs mux the live values
	// in for this one state and the state machine registers the same values for
	// T2. A prefetch fill deliberately has no equivalent -- see the header.
	// Continuation cycles reach the bus from BC_T2 (chain_next) and the
	// defensive BC_IDLE arm below keeps its registered shape, so this describes
	// a FIRST cycle and nothing else. That is what lets the outputs mux the
	// data port's own values in: `plan_addr` is `daddr` exactly when
	// `start_data` is true.
	// ...and not in a FLUSH state: a data cycle may not begin in the state a
	// COMBINATIONAL redirect fires (q_redirect only -- the registered
	// q_flush_r is held through reset/pause/restore and must not block the
	// vector read).
	assign d_bus_t1 = (bc_state == BC_IDLE) && start_data && !q_redirect;

	wire        d_t1_word = plan_word_data && (dbytes >= 3'd2);

	// The ONE exception to "a fill never starts in its presented state": a
	// COMBINATIONAL redirect (q_redirect -- taken branch or interrupt
	// micro-program transfer) arriving with the bus genuinely idle.  This is the
	// queue-FULL shape: the prefetcher paused with nothing in flight, so there
	// is no discarded cycle whose T2 the stale-fill chain could hand over, and
	// the flush state would otherwise be dead.  Silicon recovers it.
	//
	// Cartridge shapes, where the flush state is NOT a bus state (2.00 states
	// per taken branch), are untouched by construction: a starved cart queue
	// always has a byte fill in flight at the flush, so bc_state is never
	// BC_IDLE there and this term cannot fire.  The registered flush path
	// (q_flush_r: reset, pause release, savestate restore, string interruption)
	// keeps the old shape too, since it asserts q_flush without q_redirect.
	//
	// `plan_addr` answers `q_new_pc` in this state (the q_redirect term), so the
	// fabric's region/width lookup is for the redirect target, and the queue is
	// empty by construction (flushed this very state), so a word fill needs no
	// q_free check -- only the region width and target parity.  Deliberately NOT
	// gated on `prefetch_chain_hold`: that hold is for the DYING stream's fill
	// chain, while the fill issued here is the NEW stream's own first fetch.
	// `prefetch_hold` stays, because its two sources are data-access lookaheads
	// that cannot coexist with a control-transfer flush state.
	//
	// The `!bus_width8` gate is the region boundary: a 16-bit region hands the
	// flush state to the first fetch (sound RAM fetches at work-RAM rate) while
	// the 8-bit cartridge must not.  `bus_width8` answers for `plan_addr` =
	// `q_new_pc` here, i.e. for the redirect target's own region.  Byte cycles
	// cannot overlap the handover; word-capable ones can.
	//
	// The BACKWARD gate: silicon hands the flush state to the new fetch only
	// when the target leaves the discarded stream's window entirely.  A
	// short-forward `jr T,+2` redirect pays the full dead state while a backward
	// back edge recovers it, at the same region, parity and idle-bus flush
	// shape.  At the flush state `fill_pc` still names the DYING stream's next
	// fetch, so "backward" is exactly `q_new_pc < fill_pc`; a short forward
	// target sits inside the window the prefetcher was already streaming and
	// restarts with the ordinary dead-state timing.
	wire        r_bus_t1  = (bc_state == BC_IDLE) && !start_data && !d_active &&
	                        q_redirect &&
	                        !prefetch_hold && !bus_width8 &&
	                        (q_new_pc < fill_pc);
	wire        r_t1_word = plan_word_data;

	// Queue occupancy once the fill on the bus lands and this state's pop
	// is taken, which is what the chaining decision has to look at.
	wire [2:0]  q_after    = q_valid + fill_bytes - {2'd0, pop_now};
	wire [2:0]  free_after = 3'd4 - q_after;

	// Fill policy: any free slot is worth a fill. A word fill needs two, so
	// with one slot free the BIU takes the byte it can get -- the queue is
	// only four deep and a five-byte instruction needs that byte on time.
	//
	// The prefetcher never takes the bus while dreq is asserted. That reads
	// as data priority in BC_IDLE, but the state it actually matters in is
	// the one ddone pulses: the data port presents its next access there, and
	// a fill started in that state would own the bus for the two states that
	// access wanted. When the port has nothing to present it drops dreq and
	// the fill is welcome to the state -- it is genuinely free.
	// The sequencer can identify the first byte of a source-class (#16)
	// address one state before the complete data address exists. Do not launch
	// an idle-bus fill in that one state: the following address-completion
	// state can then issue the operand read as its T1 state, which is the
	// overlap required by Toshiba's six-state LD R,(#16) row. This is a
	// one-bit reservation, not a speculative data access; no incomplete
	// address reaches the external bus.
	wire        want_fill = !q_flush && !dreq &&
	                        !prefetch_hold && (q_free != 3'd0);
	wire        fill_word = plan_word_data && (q_free >= 3'd2);

	// Same decision for the fill that would chain out of T2, against the
	// occupancy this fill is about to produce.
	wire        fill_chain      = !prefetch_hold &&
	                              !prefetch_chain_hold && (free_after != 3'd0);
	wire        fill_word_chain = plan_word_data && (free_after >= 3'd2);

	// And the same decision for a fill chaining out of a DATA cycle's last T2.
	// A data cycle returns no queue bytes, so the occupancy it leaves behind is
	// just this state's pop.
	//
	// This is what stops the bus idling for a state after every access. It is
	// gated on there being somewhere to put the byte, and -- the rule that
	// makes it safe -- on `prefetch_hold`, which the sequencer raises while an
	// access on the bus already has a second one lined up behind it. That
	// second access is presented in the state `ddone` pulses in, so a fill
	// chained out of this T2 would own the two states it wanted and cost it 4
	// states from its issue state instead of 3. `dreq` cannot
	// answer that question here: it is still high in this T2 for the access
	// that is finishing, and the successor's request does not appear until the
	// state after it, so the lookahead has to come from the sequencer.
	//
	// The queue gate is not redundant with it. LDIR is two bytes against a
	// four-byte queue, so after an iteration or two `free_after_data` is zero
	// on its own; a fetch-starved loop out of the 8-bit cartridge always has
	// room, so there the prefetcher takes the state instead of losing it.
	wire [2:0]  q_after_data    = q_valid - {2'd0, pop_now};
	wire [2:0]  free_after_data = 3'd4 - q_after_data;
	wire        fill_chain_after_data = !q_flush &&
	                                    !prefetch_hold && !prefetch_chain_hold &&
	                                    (free_after_data != 3'd0);
	wire        fill_word_after_data  = plan_word_data && (free_after_data >= 3'd2);

	wire [1:0]  q_wr = q_rd + q_valid[1:0];

	// Issue helper values shared by IDLE issue and T2 chaining.
	function automatic [1:0] be_for(input word_cycle, input odd16);
	begin
		be_for = word_cycle ? 2'b11 : (odd16 ? 2'b10 : 2'b01);
	end
	endfunction

	always @(posedge clk) begin
		if (reset) begin
			bc_state     <= BC_IDLE;
			bc_for_data  <= 1'b0;
			bc_two_bytes <= 1'b0;
			bc_width8    <= 1'b0;
			bus_req_r    <= 1'b0;
			bus_we_r     <= 1'b0;
			bus_addr_r   <= 24'd0;
			bus_be_r     <= 2'd0;
			bus_wdata_r  <= 16'd0;
			d_active     <= 1'b0;
			d_addr       <= 24'd0;
			d_left       <= 3'd0;
			d_pos        <= 2'd0;
			d_wdata      <= 32'd0;
			drdata       <= 32'd0;
			ddone        <= 1'b0;
			q_rd         <= 2'd0;
			q_valid      <= 3'd0;
			queue_width8 <= 4'd0;
			fill_pc      <= 24'd0;
			fill_epoch   <= 1'b0;
			bc_epoch     <= 1'b0;
			redirect_pending <= 1'b0;
		end else if (ce) begin
			ddone <= 1'b0;

			if (pop_now) begin
				q_rd    <= q_rd + 2'd1;
				q_valid <= q_valid - 3'd1;
			end

			if (q_flush) begin
				q_rd       <= 2'd0;
				q_valid    <= 3'd0;
				fill_pc    <= q_new_pc;
				fill_epoch <= ~fill_epoch;
				redirect_pending <= q_redirect;

				// A flush state is not a bus state: the destination fetch
				// begins in the state after it, through the ordinary idle-bus
				// path below, so a taken control transfer costs a full refill
				// bubble. Measured on hardware at 2.00 states per taken
				// branch.
				//
				// `want_fill` is gated on `!q_flush`, which is what keeps the
				// flush state itself free of a bus cycle.
			end

			case (bc_state)
				BC_IDLE: begin
					if (start_data && !q_redirect) begin
						// The first cycle of a data access. This state IS its
						// T1 -- the bus signals are already live on the outputs
						// (d_bus_t1) -- so the registered copies only have to
						// hold them through T2, and the next state is T2 and
						// not another T1.
						issue_data(1'b1, BC_T2);
					end else if (d_active) begin
						// Defensive only, and deliberately kept on the ordinary
						// registered shape. A continuation cycle always reaches
						// the bus from BC_T2 through `chain_next`, so an access
						// that is still active cannot be sitting in BC_IDLE.
						// Driving the bus combinationally here would be wrong
						// as well as unreachable: with `start_data` false,
						// `plan_addr` answers for a pending redirect before it
						// answers for `d_addr`.
						issue_data(1'b0, BC_T1);
					end else if (r_bus_t1) begin
						// The idle-bus redirect: this flush state IS the new
						// stream's first fetch T1 (bus driven combinationally
						// through the r_bus_t1 live mux, exactly the d_bus_t1
						// shape), so the next state is T2 and the redirect
						// costs T1+T2 instead of flush+T1+T2.  See the
						// r_bus_t1 comment for the measurement and for why
						// the cart shapes cannot reach this arm.
						//
						// The epoch is the NEW one: the q_flush block above is
						// writing `fill_epoch <= ~fill_epoch` on this same
						// edge, so this fill must carry ~fill_epoch (old) for
						// its bytes to land under the epoch-match test in
						// BC_T2.  `redirect_pending` is cleared here and this
						// assignment wins over the q_flush block's set (same
						// always block, later in source order).
						bus_req_r    <= 1'b1;
						bus_we_r     <= 1'b0;
						bus_addr_r   <= q_new_pc;
						bc_two_bytes <= r_t1_word;
						bus_be_r     <= be_for(r_t1_word, q_new_pc[0] && !bus_width8);
						bus_wdata_r  <= 16'd0;
						bc_width8    <= bus_width8;
						bc_for_data  <= 1'b0;
						bc_epoch     <= ~fill_epoch;
						bc_state     <= BC_T2;
						redirect_pending <= 1'b0;
					end else if (want_fill) begin
						bus_req_r    <= 1'b1;
						bus_we_r     <= 1'b0;
						bus_addr_r   <= fill_pc;
						bc_two_bytes <= fill_word;
						bus_be_r     <= be_for(fill_word, fill_pc[0] && !bus_width8);
						bus_wdata_r  <= 16'd0;
						bc_width8    <= bus_width8;
						bc_for_data  <= 1'b0;
						bc_epoch     <= fill_epoch;
						bc_state     <= BC_T1;
						redirect_pending <= 1'b0;
					end
				end

				BC_T1: begin
					bc_state <= BC_T2;
				end

				BC_T2: begin
					if (bus_rdy) begin
						if (bc_for_data) begin
							if (!bus_we_r) begin
								place_read(bus_rdata, bc_two_bytes,
								           d_addr[0] && !bc_width8, d_pos);
							end

							if (bc_two_bytes ? (d_left <= 3'd2) : (d_left <= 3'd1)) begin
								// Access complete.
								d_active <= 1'b0;
								ddone    <= 1'b1;
								if (q_redirect) begin
									issue_redirect(q_new_pc, ~fill_epoch);
									redirect_pending <= 1'b0;
								end else if (redirect_pending) begin
									issue_redirect(fill_pc, fill_epoch);
									redirect_pending <= 1'b0;
								end else if (fill_chain_after_data) begin
									// Chain a fill straight out of the data
									// cycle's T2, exactly as a fill chains out
									// of a fill. Without this the bus idles for
									// one whole state after every access before
									// the prefetcher may relaunch, and that dead
									// state is charged to instruction fetch.
									//
									// It is the third state of a store's
									// marginal cost: measured on hardware at
									// 2.02 states per byte write.
									//
									// Removing the idle-bus issue state is no
									// substitute: in a fetch-bound loop a
									// store never starts from an idle bus at
									// all -- it chains out of a fill's T2.
									bus_we_r    <= 1'b0;
									bus_wdata_r <= 16'd0;
									bc_for_data <= 1'b0;
									chain_fill(fill_word_after_data);
								end else begin
									bus_req_r <= 1'b0;
									bc_state <= BC_IDLE;
								end
							end else begin
								// Chain the next cycle with no dead state.
								chain_next(bc_two_bytes);
							end
						end else begin
							// A fill retired. Land its bytes unless the stream
							// it belongs to has been flushed out from under it.
							if ((bc_epoch == fill_epoch) && !q_flush) begin
								if (bc_two_bytes) begin
									queue[q_wr]        <= bus_rdata[7:0];
									queue[q_wr + 2'd1] <= bus_rdata[15:8];
									queue_width8[q_wr]        <= bc_width8;
									queue_width8[q_wr + 2'd1] <= bc_width8;
								end else begin
									queue[q_wr] <= (fill_pc[0] && !bc_width8) ?
									               bus_rdata[15:8] : bus_rdata[7:0];
									queue_width8[q_wr] <= bc_width8;
								end
								q_valid <= q_after;
								fill_pc <= fill_pc_next;
							end

							// Arbitrate for the next cycle here, at the end of
							// T2, so the bus never idles with work waiting.
							if (start_data) begin
								// From a fill's T2 the data cycle's T1 is the
								// NEXT state, so this arm keeps the ordinary
								// registered shape.
								issue_data(1'b1, BC_T1);
							end else if (fill_chain && (bc_epoch == fill_epoch) &&
							             !q_flush) begin
								chain_fill(fill_word_chain);
							end else if ((bc_epoch != fill_epoch) && !q_flush &&
							             bc_two_bytes && !prefetch_hold &&
							             !prefetch_chain_hold && (q_free != 3'd0)) begin
								// A DISCARDED WORD fill retired: the flush
								// already happened in an earlier state (this
								// cycle's epoch is stale), the queue is empty,
								// and `fill_pc`/`fill_epoch` have pointed at
								// the new stream since the flush state.  Chain
								// its first fetch out of this T2 instead of
								// spending a state in BC_IDLE.
								//
								// The `bc_two_bytes` gate is the boundary: a
								// discarded WORD cycle can hand its T2 to the
								// new stream, a BYTE cycle cannot.  The
								// byte-fill shapes are bit-exact without the
								// chain and regress the moment it fires there,
								// while every word-filled region wants exactly
								// one state back.  Sound RAM fills as words on
								// silicon too, so the gate is cycle width, not
								// region.
								//
								// `plan_addr` already answers `fill_pc` here
								// (the `redirect_pending` term), so the width
								// decision and `chain_fill`'s address are the
								// new stream's own.
								chain_fill(plan_word_data && (q_free >= 3'd2));
								redirect_pending <= 1'b0;
							end else begin
								bus_req_r <= 1'b0;
								bc_state <= BC_IDLE;
							end
						end
					end
				end

				default: bc_state <= BC_IDLE;
			endcase
		end
	end

	// Start the first fill of a redirected stream without an intervening idle
	// bus state. The caller supplies the queue epoch because a same-edge flush
	// uses the new epoch while a redirect held behind a data access uses the
	// already-registered one.
	task automatic issue_redirect(input [23:0] addr, input epoch);
		reg word_cycle;
	begin
		word_cycle  = !bus_width8 && !addr[0];
		bus_req_r    <= 1'b1;
		bus_we_r     <= 1'b0;
		bus_addr_r   <= addr;
		bc_two_bytes <= word_cycle;
		bus_be_r     <= be_for(word_cycle, addr[0] && !bus_width8);
		bus_wdata_r  <= 16'd0;
		bc_width8    <= bus_width8;
		bc_for_data  <= 1'b0;
		bc_epoch     <= epoch;
		bc_state     <= BC_T1;
	end
	endtask

	// Issue a data cycle: the first one of an access (new_access) or the
	// continuation of one already booked. Runs from BC_IDLE and, when the
	// data port wins the arbitration at the end of a fill, from BC_T2.
	//
	task automatic issue_data(input new_access, input [1:0] next_state);
		reg [2:0]  nbytes;
		reg [15:0] src;
		reg        word;
	begin
		nbytes = new_access ? dbytes : d_left;
		src    = new_access ? dwdata[15:0] : d_wdata[15:0];
		word   = plan_word_data && (nbytes >= 3'd2);

		if (new_access) begin
			d_active <= 1'b1;
			d_addr   <= daddr;
			d_left   <= dbytes;
			d_pos    <= 2'd0;
			d_wdata  <= dwdata;
		end

		bus_req_r    <= 1'b1;
		bus_we_r     <= dwe;
		bus_addr_r   <= plan_addr;
		bc_two_bytes <= word;
		bus_be_r     <= be_for(word, plan_addr[0] && !bus_width8);
		bus_wdata_r  <= word_wdata(src, word);
		bc_width8    <= bus_width8;
		bc_for_data  <= 1'b1;
		bc_state     <= next_state;
	end
	endtask

	// Write-lane formatting: word cycles put the two live bytes on the bus;
	// byte cycles replicate the byte on both lanes and be selects.
	function automatic [15:0] word_wdata(input [15:0] pending, input word_cycle);
	begin
		word_wdata = word_cycle ? pending : {pending[7:0], pending[7:0]};
	end
	endfunction

	// Little-endian read assembly by byte position within the access.
	task automatic place_read(input [15:0] data, input word_cycle,
	                          input odd16, input [1:0] pos);
		reg [7:0] b0;
	begin
		b0 = odd16 ? data[15:8] : data[7:0];
		case (pos)
			2'd0: drdata[7:0]   <= b0;
			2'd1: drdata[15:8]  <= b0;
			2'd2: drdata[23:16] <= b0;
			2'd3: drdata[31:24] <= b0;
		endcase
		if (word_cycle) begin
			case (pos)
				2'd0: drdata[15:8]  <= data[15:8];
				2'd1: drdata[23:16] <= data[15:8];
				2'd2: drdata[31:24] <= data[15:8];
				default: ;
			endcase
		end
	end
	endtask

	// Set up the next chained cycle of the same access (runs in T2).
	task automatic chain_next(input was_word);
		reg [23:0] next_addr;
		reg [2:0]  next_left;
		reg [31:0] next_wdata;
		reg        next_word;
	begin
		next_addr  = d_addr + (was_word ? 24'd2 : 24'd1);
		next_left  = d_left - (was_word ? 3'd2 : 3'd1);
		next_wdata = was_word ? {16'd0, d_wdata[31:16]} : {8'd0, d_wdata[31:8]};
		// Ask the region for the NEW address. `plan_addr` is next_addr here
		// (d_chaining), so bus_width8 answers for it -- see the header. Within
		// one region this is the same answer the old latched width gave, so
		// nothing that stays inside a region changes cost or shape.
		next_word  = plan_word_data && (next_left >= 3'd2);

		d_addr       <= next_addr;
		d_left       <= next_left;
		d_pos        <= d_pos + (was_word ? 2'd2 : 2'd1);
		d_wdata      <= next_wdata;
		bus_addr_r   <= next_addr;
		bc_two_bytes <= next_word;
		bus_be_r     <= be_for(next_word, next_addr[0] && !bus_width8);
		bus_wdata_r  <= word_wdata(next_wdata[15:0], next_word);
		bc_width8    <= bus_width8;
		bc_state     <= BC_T1;
	end
	endtask

	// Start the next prefetch fill straight out of T2 (runs in T2, with
	// bus_req and bus_we already at the values this fill needs). plan_addr
	// is fill_pc_next here, so bus_width8 answers for the new address.
	task automatic chain_fill(input word_cycle);
	begin
		bus_addr_r   <= plan_addr;
		bc_two_bytes <= word_cycle;
		bus_be_r     <= be_for(word_cycle, plan_addr[0] && !bus_width8);
		bus_wdata_r  <= 16'd0;
		bc_width8    <= bus_width8;
		bc_epoch     <= fill_epoch;
		bc_state     <= BC_T1;
	end
	endtask

	// External bus outputs. Every state except the one an idle bus takes a data
	// request in is the registered cycle; in that one state the data port's own
	// values are muxed straight through, because that state is the cycle's T1
	// and the fabric latches its wait budget and its region select from it.
	// The state machine registers the identical values for T2, so nothing on the
	// bus moves between T1 and T2.
	wire        bus_req_live   = (d_bus_t1 || r_bus_t1) ? 1'b1 : bus_req_r;
	wire        bus_we_live    = d_bus_t1 ? dwe :
	                             r_bus_t1 ? 1'b0 : bus_we_r;
	wire [23:0] bus_addr_live  = (d_bus_t1 || r_bus_t1) ? plan_addr : bus_addr_r;
	wire [1:0]  bus_be_live    = d_bus_t1
	                             ? be_for(d_t1_word, plan_addr[0] && !bus_width8)
	                             : r_bus_t1
	                             ? be_for(r_t1_word, plan_addr[0] && !bus_width8)
	                             : bus_be_r;
	wire [15:0] bus_wdata_live = d_bus_t1
	                             ? word_wdata(dwdata[15:0], d_t1_word)
	                             : r_bus_t1 ? 16'd0
	                             : bus_wdata_r;

	// SPLIT_BUS_OUT holds the bus outputs across the machine half period.  The
	// T1 mux above is what makes an idle-bus data access cost T1+T2 rather than
	// issue+T1+T2, but it also stops `bus_addr` being a register, and without a
	// register there the CPU-to-fabric chain is the longest in the machine:
	//
	//   q_valid -> queue/decode -> sequencer -> daddr -> plan_addr -> bus_addr
	//           -> fabric region decode -> ngp_cheat_engine comparators
	//
	// Same cure as SPLIT_RF_READ: a hold register on the opposite phase of the
	// machine enable splits the chain into two timed halves -- the CPU's own
	// cone up to `bus_addr`, and the fabric's decode out of it.
	//
	// It costs the machine nothing.  `ce` is at most one clk_sys cycle in
	// sixteen, and every consumer of these signals -- the fabric's `cyc_active`
	// and wait latch, its T2 strobes, and every memory address port -- samples
	// on a `ce` edge.  The hold loads on every `!ce` cycle, so it is current
	// within one clk_sys cycle of a tick and stable for the fourteen after it.
	//
	// It REQUIRES that `ce` is never high on two consecutive clk_sys cycles,
	// the identical precondition SPLIT_RF_READ carries, which is why t900_cpu
	// drives this parameter from that one.  A bench that pulses `ce` every
	// cycle must run this at 0.
	generate
	if (SPLIT_BUS_OUT != 0) begin : g_bus_out_split
		reg        bus_req_h;
		reg        bus_we_h;
		reg [23:0] bus_addr_h;
		reg [1:0]  bus_be_h;
		reg [15:0] bus_wdata_h;

		// No reset. The registered sources have one, `ce` is low throughout
		// reset so the hold is reloaded on every cycle of it, and it is
		// therefore current before the first tick.
		always @(posedge clk) begin
			if (!ce) begin
				bus_req_h   <= bus_req_live;
				bus_we_h    <= bus_we_live;
				bus_addr_h  <= bus_addr_live;
				bus_be_h    <= bus_be_live;
				bus_wdata_h <= bus_wdata_live;
			end
		end

		assign bus_req   = bus_req_h;
		assign bus_we    = bus_we_h;
		assign bus_addr  = bus_addr_h;
		assign bus_be    = bus_be_h;
		assign bus_wdata = bus_wdata_h;
	end else begin : g_bus_out_direct
		assign bus_req   = bus_req_live;
		assign bus_we    = bus_we_live;
		assign bus_addr  = bus_addr_live;
		assign bus_be    = bus_be_live;
		assign bus_wdata = bus_wdata_live;
	end
	endgenerate

endmodule
