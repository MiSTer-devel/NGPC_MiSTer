// Copyright (c) 2026 Jamie Blanks

// TLCS-900/H CPU core: the sequencer plus everything it drives.
//
//   t900_biu       bus + 4-byte prefetch queue + dynamic bus sizing
//   t900_decode    combinational class decode and table lookup
//   t900_regfile   20 x 32-bit banked register file
//   t900_alu       combinational arithmetic / logic / flags
//   t900_shifter   ce-stepped shift and rotate
//   t900_muldiv    ce-stepped iterative multiply / divide
//   t900_seq       the sequencer
//
// Everything advances on `ce`; one `ce` tick is one Toshiba "state".
//
// The external bus contract is t900_biu's: `plan_addr` is the address the BIU is
// about to issue and the fabric answers `bus_width8` for it combinationally;
// `bus_req` holds address, byte enables and write data stable until `bus_rdy`.
//
// `trace_valid` pulses for exactly one `ce` on the state an instruction retires.
// `trace_pc` is sampled from the same edge and is therefore the post-retirement
// PC -- the address of the next instruction fetch, which is the branch or vector
// target after a taken control transfer.  `trace_sr` / `trace_f` are the
// architectural SR and its flag byte at the same instant.  Interrupt entry and a
// micro-DMA transfer retire the same way and pulse `trace_valid` too.
//
// The architectural register set is read through the savestate tap rather than
// duplicated onto a wide output: `ss_reg_addr` 0x00-0x13 selects a physical
// register file entry and `ss_rdata` answers combinationally.  Physical indices
// are {bank, reg} for banks 0-3 of XWA/XBC/XDE/XHL (0x00-0x0F) then XIX, XIY,
// XIZ, XSP (0x10-0x13); the current bank is `trace_sr[9:8]`.
//
// Savestate tap
//   0x00-0x13  register file, one 32-bit entry each
//   0x20       PC
//   0x21       {F', SR}
//   0x22       {halt, INTNEST}
//   0x30-0x33  micro-DMA DMAS0..3
//   0x34-0x37  micro-DMA DMAD0..3
//   0x38-0x3B  micro-DMA {DMAM, DMAC}0..3
//
// Writes are honoured only while the core is paused.  `pause_req` asks the core
// to stop; `pause_ready` reports that it has stopped at an instruction boundary
// with no bus cycle outstanding.  The prefetch queue is not part of the state:
// it is flushed and rebuilt from the restored PC on resume.
//
// SPLIT_RF_READ splits the core's long combinational cones onto the opposite
// phase of the machine enable.  Unsplit, the worst setup paths are all one chain
// launched from the BIU's prefetch queue read pointer:
//
//   q_rd -> queue byte mux -> t900_decode -> t900_seq's register codes
//        -> t900_regfile read mux -> t900_seq's operand mux -> t900_alu
//        -> t900_seq's write-back mux -> t900_regfile write
//
// With SPLIT_RF_READ = 1 a holding register sits on the register file's two read
// ports, near the middle of that chain, and holds three more things:
//
//   q_byte  the prefetch queue byte, for the paths that never reach the register
//           file -- an immediate assembled out of queue bytes runs
//           q_rd -> queue mux -> decode -> operand placement -> write-back.
//   the decode-table outputs, for the family that launches from the sequencer's
//           own state bit rather than from q_byte:
//           st.S_FETCH1 -> the dec_b0/dec_b1 mux -> t900_decode -> cur_op
//                -> the source-kind mux -> alu_b -> t900_alu -> write-back.
//   the asynchronous control inputs (int_req / dma_req / pause_req), because the
//           register-file READ ADDRESS depends on them through
//           ev_start -> q_pop -> decode_live -> op_avail -> want_sp
//           -> rf_ra_code.  Leaving them live gives an inconsistent snapshot:
//           a read address chosen by the old control inputs and consumed with
//           the new ones.
//
// The hold register loads on every clk_sys cycle in which `ce` is LOW, so this
// is not a pipeline stage and it changes no cycle count.  `ce` is never asserted
// on two consecutive clk_sys cycles -- the machine-state enable is one in
// sixteen and the clock gear only thins it further, to one in 256 at gear 4 --
// so the last load before a `ce` tick lands exactly one clk_sys cycle before it.
// Taking the phase from `ce` itself rather than from a fixed phi1 tick is what
// makes it gear-proof: "the last cycle before the tick" is right at every gear.
//
// Every held cone's inputs move only on `ce`, so the held copy and the live one
// agree at each machine tick.  The decode-table hold is a hold whose input is
// another hold, so its value is correct from the THIRD clk_sys cycle after a
// tick rather than the second, and machine ticks must be at least four clk_sys
// cycles apart; they are sixteen at the fastest gear.  Holding the control
// inputs only costs a request arriving inside that last cycle being taken on the
// next machine tick, which is free because each is a held level rather than a
// pulse: ngp_intc's win_valid/level/vector and dma_req stand until the CPU
// acknowledges, halt_release is a level, and pause_req is a handshake the
// requester waits on.  The pause chain cannot deadlock either -- while the core
// is parked `ce` is low, so the hold tracks pause_req every clk_sys cycle.
//
// The hold keeps no state of its own and takes no savestate tap.  It reloads on
// the first clk_sys cycle in which `ce` is low, which a restore always has
// because the core is paused while its registers are written.  Release stays in
// S_PAUSE while the prefetch queue is flushed and refilled; those ticks read no
// register-file data.  SPLIT_RF_READ defaults to 0 because a bench may drive
// `ce` on every clk_sys edge, leaving no low cycle to split into; the
// synthesised core sets it to 1.
//
// The interrupt acknowledge names a source, not a moment.  The control-input
// snapshot is taken one clk_sys cycle after a machine tick while the acceptance
// decision is made at the next tick, and ngp_intc republishes its priority
// winner on every clk_sys edge, so a moment-based acknowledge can clear a source
// the CPU never accepted.  Toshiba's acceptance step 1 is a single transaction:
// the CPU "reads the interrupt vector from the interrupt controller and clears
// that source's request flip-flop" (TMP95C061 datasheet p.10-11).  So:
//
//   int_id      the id of the source the controller is presenting, snapshotted
//               with int_req and int_level so all three describe one source
//   int_ack_id  that same id, handed back on the acknowledge; the controller
//               clears THIS source, not whoever is winning at that edge
//   int_vector  taken LIVE, not held; the controller answers it for int_ack_id
//               during the acknowledge
//
// Taking the vector live costs nothing on the split cone -- int_vector reaches
// only the sequencer's `ev_vec` latch, never the register-file read address --
// and it preserves the documented default-vector race: if the store that clears
// the winner's request flag lands in the acknowledge cycle, ngp_intc hands back
// 0x28 and the CPU dispatches to 0xFFFF28 (TMP95C061 datasheet p.22 note 4).
// Freezing the controller's presentation until the acknowledge would instead
// suppress legitimate preemption, since silicon samples requests at the end of
// an instruction and the highest-priority source at THAT instant wins
// (CPU900H p.38).

module t900_cpu
#(
	// See t900_seq's header: 0 selects the documented interrupt shadow and
	// interruptible repeats.
	parameter EI_SHADOW_RETI       = 0,
	parameter STRING_INTERRUPTIBLE = 1,
	// 1 = hold the register-file read across the machine half period. See the
	// header note; the synthesised configuration is 1 and requires that `ce`
	// is never high on two consecutive clk_sys cycles.
	parameter SPLIT_RF_READ        = 0
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	// ---- external bus ---------------------------------------------------
	output wire [23:0] plan_addr,
	input  wire        bus_width8,
	// High while the cycle in flight is held by the K2GE composition pass
	// rather than by slow memory.  Only the sequencer uses it, and only to
	// keep a repeat form from paying for arbitration; see `mem_over_charge`.
	input  wire        bus_wait_gfx,
	output wire        bus_req,
	output wire        bus_we,
	output wire [23:0] bus_addr,
	output wire [1:0]  bus_be,
	output wire [15:0] bus_wdata,
	input  wire [15:0] bus_rdata,
	input  wire        bus_rdy,

	// ---- interrupt controller -------------------------------------------
	// The MCU shell decides who wins and presents one request; the core runs
	// the Toshiba entry microprogram and strobes int_ack when it accepts.
	input  wire        int_req,
	input  wire [2:0]  int_level,
	input  wire [7:0]  int_vector,
	// Which source the controller is presenting, and the same identity handed
	// back with the acknowledge. See "THE ACKNOWLEDGE CARRIES AN IDENTITY" in
	// the header: this is what makes the acknowledge name a SOURCE instead of
	// naming a moment in time.
	input  wire [4:0]  int_id,
	output wire        int_ack,
	output wire [4:0]  int_ack_id,

	// A request below the current IFF mask does not run a handler, but it does
	// still bring the CPU out of HALT (TMP95C061 datasheet p.28 Table 3.4(2)). The
	// shell asserts `halt_release` for that case -- any request flip-flop set
	// at level 1-6, or NMI/INTWD, with no IFF test. See t900_seq's port.
	input  wire        halt_release,

	// ---- micro-DMA -------------------------------------------------------
	// dma_req[n] means channel n is armed (DMAnV matched) and its source has
	// a request pending. dma_ack[n] strobes when the transfer starts, so the
	// shell clears that source's request flip-flop; dma_end[n] strobes at
	// terminal count, so the shell raises INTTCn and zeroes DMAnV.
	input  wire [3:0]  dma_req,
	output wire [3:0]  dma_ack,
	output wire [3:0]  dma_end,

	// ---- retirement trace -------------------------------------------------
	output wire        trace_valid,
	output wire [23:0] trace_pc,
	output wire [15:0] trace_sr,
	output wire [7:0]  trace_f,
	output wire        halted,

	// ---- savestate tap ----------------------------------------------------
	input  wire [7:0]  ss_reg_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output wire [31:0] ss_rdata,
	input  wire        restore_hold,
	input  wire        pause_req,
	output wire        pause_ready
);

	// Interconnect

	wire        dreq;
	wire        dwe;
	wire [23:0] daddr;
	wire [2:0]  dbytes;
	wire [31:0] dwdata;
	wire [31:0] drdata;
	wire        ddone;

	wire        q_flush;
	wire [23:0] q_new_pc;
	wire        q_redirect;
	wire        q_pop;
	wire        prefetch_chain_hold;
	wire [7:0]  q_byte_c;      // straight off the prefetch queue's read mux
	wire [7:0]  q_byte;        // what the decoder and sequencer see
	wire [2:0]  q_count;

	wire [7:0]  dec_b0;
	wire [7:0]  dec_b1;

	// Straight off the decode tables, and the sequencer's view of them. They
	// are the same wires unless SPLIT_RF_READ is set -- see the header.
	wire [1:0]  dec_cls_c;
	wire [4:0]  dec_ea_kind_c;
	wire [2:0]  dec_ea_reg_c;
	wire [1:0]  dec_cls_size_c;
	wire        dec_needs_byte1_c;
	wire [7:0]  dec_op_id_c;
	wire [4:0]  dec_op1_kind_c;
	wire [4:0]  dec_op2_kind_c;
	wire [1:0]  dec_op1_size_c;
	wire [1:0]  dec_op2_size_c;
	wire [5:0]  dec_states_c;
	wire [3:0]  dec_group_c;
	wire        dec_undef_c;

	wire [1:0]  dec_cls;
	wire [4:0]  dec_ea_kind;
	wire [2:0]  dec_ea_reg;
	wire [1:0]  dec_cls_size;
	wire        dec_needs_byte1;
	wire [7:0]  dec_op_id;
	wire [4:0]  dec_op1_kind;
	wire [4:0]  dec_op2_kind;
	wire [1:0]  dec_op1_size;
	wire [1:0]  dec_op2_size;
	wire [5:0]  dec_states;
	wire [3:0]  dec_group;
	wire        dec_undef;

	wire [1:0]  rf_rfp;
	wire [7:0]  rf_ra_code;
	wire [1:0]  rf_ra_size;
	wire [31:0] rf_ra_data_c;   // straight off the register file's read mux
	wire [31:0] rf_ra_data;     // what the sequencer sees (see SPLIT_RF_READ)
	wire [7:0]  rf_rb_code;
	wire [1:0]  rf_rb_size;
	wire [31:0] rf_rb_data_c;
	wire [31:0] rf_rb_data;
	wire        rf_wr_en;

	// The sequencer's view of the asynchronous control inputs. Identical to the
	// pins unless SPLIT_RF_READ is set, in which case they are snapshotted with
	// the register-file read -- see the module header's setup-window note.
	wire        seq_int_req;
	wire [2:0]  seq_int_level;
	wire [4:0]  seq_int_id;
	wire        seq_halt_release;
	wire [3:0]  seq_dma_req;
	wire        seq_pause_req;
	wire        seq_pause_ready;
	wire        prefetch_hold;
	// The BIU takes a data request from an idle bus in the state it is
	// presented, so `bus_req` is already high there and the sequencer's
	// bus-penalty measurement cannot key on `!bus_busy` any more.
	wire        d_bus_t1;

	// The sequencer reports `pause_req_held && parked`; the pin reports
	// `pause_req && parked`. They are the same signal when SPLIT_RF_READ is 0.
	// With the split, ANDing the live request back in is what keeps the release
	// edge exact: the held copy needs one more clk_sys cycle to fall, and a
	// pause_ready that outlived its request by a cycle would hold the SoC's
	// enable tree off for that cycle. The rising edge is deliberately left on
	// the held copy, because that is the edge that has to agree with the
	// machine state the sequencer parked in.
	assign pause_ready = restore_hold || (seq_pause_ready && pause_req);

	// The identity half of the acknowledge. It is the SNAPSHOTTED id, so it
	// names exactly the source whose request and level the sequencer used to
	// decide with, and it is valid on the state `int_ack` is high in.
	assign int_ack_id = seq_int_id;
	wire [7:0]  rf_wr_code;
	wire [1:0]  rf_wr_size;
	wire [31:0] rf_wr_data;

	wire [4:0]  alu_op;
	wire [1:0]  alu_size;
	wire [31:0] alu_a;
	wire [31:0] alu_b;
	wire [7:0]  alu_flags_in;
	wire [31:0] alu_result;
	wire [7:0]  alu_flags_out;

	wire        sh_start;
	wire [2:0]  sh_op;
	wire [1:0]  sh_size;
	wire [4:0]  sh_count;
	wire [31:0] sh_data;
	wire [7:0]  sh_flags_in;
	wire        sh_done;
	wire        sh_finish;
	wire [31:0] sh_result;
	wire [7:0]  sh_flags_out;

	wire        md_start;
	wire [1:0]  md_op;
	wire        md_size;
	wire [31:0] md_a;
	wire [31:0] md_b;
	wire        md_done;
	wire        md_finish;
	wire [31:0] md_finish_result;
	wire [31:0] md_result;
	wire        md_v;

	// Savestate routing: the low window is the register file, the rest is
	// sequencer state.
	wire        ss_is_rf = (ss_reg_addr < 8'h14);
	wire [31:0] ss_rf_rdata;
	wire [31:0] ss_seq_rdata;

	assign ss_rdata = ss_is_rf ? ss_rf_rdata : ss_seq_rdata;

	// Bus interface unit

	// SPLIT_BUS_OUT rides on SPLIT_RF_READ deliberately. They are different
	// splits of different chains, but they carry the identical precondition --
	// `ce` must never be high on two consecutive clk_sys cycles -- so a
	// configuration that can take one can take the other, and a bench that
	// cannot take one cannot take the other either.
	t900_biu
	#(
		.SPLIT_BUS_OUT (SPLIT_RF_READ)
	)
	u_biu
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),
		.dreq(dreq),
		.dwe(dwe),
		.daddr(daddr),
		.dbytes(dbytes),
		.dwdata(dwdata),
		.drdata(drdata),
		.ddone(ddone),
		.q_flush(q_flush),
		.q_new_pc(q_new_pc),
		.q_redirect(q_redirect),
		.q_pop(q_pop),
		.prefetch_hold(prefetch_hold),
		.prefetch_chain_hold(prefetch_chain_hold),
		.q_byte(q_byte_c),
		.q_count(q_count),
		.plan_addr(plan_addr),
		.bus_width8(bus_width8),
		.bus_req(bus_req),
		.bus_we(bus_we),
		.bus_addr(bus_addr),
		.bus_be(bus_be),
		.bus_wdata(bus_wdata),
		.bus_rdata(bus_rdata),
		.bus_rdy(bus_rdy),
		.d_bus_t1(d_bus_t1)
	);

	// Decoder

	t900_decode u_decode
	(
		.byte0(dec_b0),
		.byte1(dec_b1),
		.cls(dec_cls_c),
		.ea_kind(dec_ea_kind_c),
		.ea_reg(dec_ea_reg_c),
		.cls_size(dec_cls_size_c),
		.needs_byte1(dec_needs_byte1_c),
		.op_id(dec_op_id_c),
		.op1_kind(dec_op1_kind_c),
		.op2_kind(dec_op2_kind_c),
		.op1_size(dec_op1_size_c),
		.op2_size(dec_op2_size_c),
		.states(dec_states_c),
		.group(dec_group_c),
		.undef(dec_undef_c)
	);

	// Register file

	t900_regfile u_regfile
	(
		.clk(clk),
		.rfp(rf_rfp),
		.ra_code(rf_ra_code),
		.ra_size(rf_ra_size),
		.ra_data(rf_ra_data_c),
		.rb_code(rf_rb_code),
		.rb_size(rf_rb_size),
		.rb_data(rf_rb_data_c),
		.wr_en(rf_wr_en),
		.wr_code(rf_wr_code),
		.wr_size(rf_wr_size),
		.wr_data(rf_wr_data),
		.ss_wren(ss_wren && ss_is_rf && pause_ready),
		.ss_addr(ss_reg_addr[4:0]),
		.ss_wdata(ss_wdata),
		.ss_rdata(ss_rf_rdata)
	);

	// The half-period hold that splits the long path, and the control-input
	// snapshot that keeps it consistent. See the SPLIT_RF_READ note in the
	// module header: `!ce` is the opposite phase of the machine enable, so the
	// load immediately before a `ce` tick is exactly one clk_sys cycle before
	// it at every clock gear.
	//
	// No reset on any of it. The register file has none either, so both
	// configurations propagate the same X until software has written a
	// register; and `ce` is low throughout reset, so the whole snapshot is
	// loaded on every cycle of it and is current before the first tick.
	generate
	if (SPLIT_RF_READ != 0) begin : g_rf_read_split
		reg [31:0] ra_hold;
		reg [31:0] rb_hold;
		reg        int_req_hold;
		reg [2:0]  int_level_hold;
		reg [4:0]  int_id_hold;
		reg        halt_release_hold;
		reg [3:0]  dma_req_hold;
		reg        pause_req_hold;
		reg [7:0]  q_byte_hold;
		reg        ce_d;

		// The decode-table result, held the same way. See "THE SECOND SPLIT"
		// in the module header.
		reg [1:0]  dec_cls_hold;
		reg [4:0]  dec_ea_kind_hold;
		reg [2:0]  dec_ea_reg_hold;
		reg [1:0]  dec_cls_size_hold;
		reg        dec_needs_byte1_hold;
		reg [7:0]  dec_op_id_hold;
		reg [4:0]  dec_op1_kind_hold;
		reg [4:0]  dec_op2_kind_hold;
		reg [1:0]  dec_op1_size_hold;
		reg [1:0]  dec_op2_size_hold;
		reg [5:0]  dec_states_hold;
		reg [3:0]  dec_group_hold;
		reg        dec_undef_hold;

		// The control snapshot is taken ONE CYCLE AFTER the machine tick, not
		// on the same edge as the read data, and the difference matters. Load
		// both on `!ce` and the snapshot is a generation out of step with
		// itself: on the load edge the read data is captured using the control
		// values that are being REPLACED on that same edge, so the sequencer
		// ends up holding a read taken at the previous generation's address.
		//
		// `ce_d` fires once per machine tick, immediately after it, so the
		// control values are frozen for the whole rest of the interval and the
		// data hold's last load -- one cycle before the next tick -- reads the
		// register file at the address those same frozen values chose.
		//
		// Two exceptions keep it out of trouble: during reset there are no
		// ticks, so it follows the pins and is defined the instant reset
		// releases; and while the core is parked there are no ticks either,
		// so it follows the pins again, which is what lets `pause_req` fall
		// and the pause be released. Neither can produce an inconsistent
		// snapshot, because both are states with no `ce` tick in them.
		wire ctl_en = reset | ce_d | seq_pause_ready;

		always @(posedge clk) begin
			ce_d <= ce;

			if (!ce) begin
				ra_hold     <= rf_ra_data_c;
				rb_hold     <= rf_rb_data_c;
				q_byte_hold <= q_byte_c;

				dec_cls_hold         <= dec_cls_c;
				dec_ea_kind_hold     <= dec_ea_kind_c;
				dec_ea_reg_hold      <= dec_ea_reg_c;
				dec_cls_size_hold    <= dec_cls_size_c;
				dec_needs_byte1_hold <= dec_needs_byte1_c;
				dec_op_id_hold       <= dec_op_id_c;
				dec_op1_kind_hold    <= dec_op1_kind_c;
				dec_op2_kind_hold    <= dec_op2_kind_c;
				dec_op1_size_hold    <= dec_op1_size_c;
				dec_op2_size_hold    <= dec_op2_size_c;
				dec_states_hold      <= dec_states_c;
				dec_group_hold       <= dec_group_c;
				dec_undef_hold       <= dec_undef_c;
			end

			if (ctl_en) begin
				int_req_hold      <= int_req;
				int_level_hold    <= int_level;
				int_id_hold       <= int_id;
				halt_release_hold <= halt_release;
				dma_req_hold      <= dma_req;
				pause_req_hold    <= pause_req;
			end
		end

		assign rf_ra_data      = ra_hold;
		assign rf_rb_data      = rb_hold;
		assign q_byte          = q_byte_hold;
		assign dec_cls         = dec_cls_hold;
		assign dec_ea_kind     = dec_ea_kind_hold;
		assign dec_ea_reg      = dec_ea_reg_hold;
		assign dec_cls_size    = dec_cls_size_hold;
		assign dec_needs_byte1 = dec_needs_byte1_hold;
		assign dec_op_id       = dec_op_id_hold;
		assign dec_op1_kind    = dec_op1_kind_hold;
		assign dec_op2_kind    = dec_op2_kind_hold;
		assign dec_op1_size    = dec_op1_size_hold;
		assign dec_op2_size    = dec_op2_size_hold;
		assign dec_states      = dec_states_hold;
		assign dec_group       = dec_group_hold;
		assign dec_undef       = dec_undef_hold;
		assign seq_int_req     = int_req_hold;
		assign seq_int_level   = int_level_hold;
		assign seq_int_id      = int_id_hold;
		assign seq_halt_release = halt_release_hold;
		assign seq_dma_req     = dma_req_hold;
		assign seq_pause_req   = pause_req_hold;
	end else begin : g_rf_read_direct
		assign rf_ra_data      = rf_ra_data_c;
		assign rf_rb_data      = rf_rb_data_c;
		assign q_byte          = q_byte_c;
		assign dec_cls         = dec_cls_c;
		assign dec_ea_kind     = dec_ea_kind_c;
		assign dec_ea_reg      = dec_ea_reg_c;
		assign dec_cls_size    = dec_cls_size_c;
		assign dec_needs_byte1 = dec_needs_byte1_c;
		assign dec_op_id       = dec_op_id_c;
		assign dec_op1_kind    = dec_op1_kind_c;
		assign dec_op2_kind    = dec_op2_kind_c;
		assign dec_op1_size    = dec_op1_size_c;
		assign dec_op2_size    = dec_op2_size_c;
		assign dec_states      = dec_states_c;
		assign dec_group       = dec_group_c;
		assign dec_undef       = dec_undef_c;
		assign seq_int_req     = int_req;
		assign seq_int_level   = int_level;
		assign seq_int_id      = int_id;
		assign seq_halt_release = halt_release;
		assign seq_dma_req     = dma_req;
		assign seq_pause_req   = pause_req;
	end
	endgenerate

	// Function units

	t900_alu u_alu
	(
		.op(alu_op),
		.size(alu_size),
		.a(alu_a),
		.b(alu_b),
		.flags_in(alu_flags_in),
		.result(alu_result),
		.flags_out(alu_flags_out)
	);

	t900_shifter u_shifter
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),
		.start(sh_start),
		.op(sh_op),
		.size(sh_size),
		.count(sh_count),
		.data_in(sh_data),
		.flags_in(sh_flags_in),
		.busy(),
		.done(sh_done),
		.finish(sh_finish),
		.result(sh_result),
		.flags_out(sh_flags_out)
	);

	t900_muldiv u_muldiv
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),
		.start(md_start),
		.op(md_op),
		.size(md_size),
		.a(md_a),
		.b(md_b),
		.busy(),
		.done(md_done),
		.finish(md_finish),
		.finish_result(md_finish_result),
		.result(md_result),
		.v_flag(md_v)
	);

	// Sequencer

	t900_seq
	#(
		.EI_SHADOW_RETI(EI_SHADOW_RETI),
		.STRING_INTERRUPTIBLE(STRING_INTERRUPTIBLE)
	)
	u_seq
	(
		.clk(clk),
		.ce(ce),
		.reset(reset),

		.dreq(dreq),
		.dwe(dwe),
		.daddr(daddr),
		.dbytes(dbytes),
		.dwdata(dwdata),
		.drdata(drdata),
		.ddone(ddone),
		.bus_busy(bus_req),
		.d_bus_t1(d_bus_t1),
		.d_wait_gfx(bus_wait_gfx),

		.q_flush(q_flush),
		.q_new_pc(q_new_pc),
		.q_redirect(q_redirect),
		.q_pop(q_pop),
		.prefetch_hold(prefetch_hold),
		.prefetch_chain_hold(prefetch_chain_hold),
		.q_byte(q_byte),
		.q_count(q_count),

		.dec_b0(dec_b0),
		.dec_b1(dec_b1),
		.dec_cls(dec_cls),
		.dec_ea_kind(dec_ea_kind),
		.dec_ea_reg(dec_ea_reg),
		.dec_cls_size(dec_cls_size),
		.dec_needs_byte1(dec_needs_byte1),
		.dec_op_id(dec_op_id),
		.dec_op1_kind(dec_op1_kind),
		.dec_op2_kind(dec_op2_kind),
		.dec_op1_size(dec_op1_size),
		.dec_op2_size(dec_op2_size),
		.dec_states(dec_states),
		.dec_group(dec_group),
		.dec_undef(dec_undef),

		.rf_rfp(rf_rfp),
		.rf_ra_code(rf_ra_code),
		.rf_ra_size(rf_ra_size),
		.rf_ra_data(rf_ra_data),
		.rf_rb_code(rf_rb_code),
		.rf_rb_size(rf_rb_size),
		.rf_rb_data(rf_rb_data),
		.rf_wr_en(rf_wr_en),
		.rf_wr_code(rf_wr_code),
		.rf_wr_size(rf_wr_size),
		.rf_wr_data(rf_wr_data),

		.alu_op(alu_op),
		.alu_size(alu_size),
		.alu_a(alu_a),
		.alu_b(alu_b),
		.alu_flags_in(alu_flags_in),
		.alu_result(alu_result),
		.alu_flags_out(alu_flags_out),

		.sh_start(sh_start),
		.sh_op(sh_op),
		.sh_size(sh_size),
		.sh_count(sh_count),
		.sh_data(sh_data),
		.sh_flags_in(sh_flags_in),
		.sh_done(sh_done),
		.sh_finish(sh_finish),
		.sh_result(sh_result),
		.sh_flags_out(sh_flags_out),

		.md_start(md_start),
		.md_op(md_op),
		.md_size(md_size),
		.md_a(md_a),
		.md_b(md_b),
		.md_done(md_done),
		.md_finish(md_finish),
		.md_finish_result(md_finish_result),
		.md_result(md_result),
		.md_v(md_v),

		.int_req(seq_int_req),
		.int_level(seq_int_level),
		// LIVE, in both configurations -- see the header. The controller
		// answers for the id being acknowledged, so this is the vector of the
		// ACCEPTED source and not of whoever happens to be winning.
		.int_vector(int_vector),
		.int_ack(int_ack),
		.halt_release(seq_halt_release),

		.dma_req(seq_dma_req),
		.dma_ack(dma_ack),
		.dma_end(dma_end),

		.trace_valid(trace_valid),
		.trace_pc(trace_pc),
		.trace_sr(trace_sr),
		.trace_f(trace_f),
		.halted(halted),

		.ss_reg_addr(ss_reg_addr),
		.ss_wdata(ss_wdata),
		.ss_wren(ss_wren && !ss_is_rf),
		.ss_rdata(ss_seq_rdata),
		.restore_hold(restore_hold),
		.pause_req(seq_pause_req),
		.pause_ready(seq_pause_ready)
	);

endmodule
