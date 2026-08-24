// Copyright (c) 2026 Jamie Blanks

// TLCS-900/H instruction sequencer.  Turns the combinational decode of
// t900_decode into micro-operations on the register file, the ALU, the shifter,
// the multiply/divide unit and the bus interface unit.  Everything advances on
// `ce`; one `ce` tick is one Toshiba "state".
//
//   S_FETCH0  pop the first opcode byte (stall while the queue is empty)
//   S_EA      consume the class effective-address bytes and compute the EA
//   S_FETCH1  pop the class's second opcode byte (the real opcode)
//   S_OPF     pop displacement / immediate / address bytes
//   S_RD      wait for the operand read the instruction needs
//   S_EXEC    multi-state units (shifter, multiply/divide) and micro-programs
//   S_NEXT    issue any deferred store and charge the remaining budget
//
// A state does its work AND commits the instruction in the same tick whenever
// everything it needs is already available: `LD R,r` really is two ticks, the
// second of which pops the opcode byte, reads two registers and writes one.  The
// write-back is therefore a combinational description of the instruction's
// effects (the "execute" section) that any state may strobe with `do_exec`.
//
// A source-class instruction (0x80-0xAF, 0xC0-0xE5) always reads its class
// operand whatever the second opcode byte turns out to be, so that read is
// issued the moment the address exists -- one state before the opcode is even
// known.  That is what keeps `LD R,(XRR)` inside its four-state budget.
//
// Cycle accounting
//   instruction_states = table_states
//                      + addressing-mode adder
//                      + (actual_bus_states - nominal_bus_states)
//
// `used` counts the ticks the current instruction has occupied since its first
// opcode byte was popped; `budget` holds the total it must occupy.  `budget` is
// loaded once the decode table entry is known and grows by the measured bus
// penalty when a data access completes: `mem_nom` is what the same access would
// have cost in 16-bit zero-wait memory (1 issue state plus 2 states per bus
// cycle) and `mem_ticks` is what it really cost.  Nothing retires before
// `used >= budget`.
//
// A store is posted: the sequencer retires once it has issued the write and
// spent its budget, exactly as the real BIU/EU decoupling does.  If that store's
// penalty only lands after retirement it is carried into the next instruction's
// budget through `pen_carry`, so no state is ever lost.
//
// RCODE_ADDER (the C7/D7/E7 full-register-code prefix) is 1 here, matching
// Toshiba (CPU900H p.167 table 10), not the 0 that the generated
// rtl/t900/tables/main.tsv records: the `LD r,#` byte form is `C7 rcode 03 n8`,
// and four bytes cannot be consumed in the three states the table lists unless
// the prefix costs a state.  The generated tables are not edited.
//
// Three multi-state micro-programs sit beside the instruction pipeline.  All of
// them reach the bus through one shared request channel (`mp_*`) and one shared
// register-file write channel (`mpr_*`), so the arbiter still sees a single
// client and the state accounting is the same everywhere.
//
//   S_INT  vector entry and RETI.  Three flavours selected by `ev_kind`:
//          EV_SWI  the SWI n instruction and the INTUNDEF trap
//          EV_INT  a hardware interrupt (adds IFF <- level+1, INTNEST+1,
//                  int_ack and the queue flush)
//          EV_RETI pop SR, pop PC, INTNEST-1
//   S_DMA  one micro-DMA transfer for the highest-priority requesting channel
//   S_STR  the string / repeat group and MULA
//
// The interrupt vector fetch is a 4-byte read even though only 24 bits are used.
// Toshiba's state table (TMP95C061 datasheet p.11) charges +4 states for an
// 8-bit vector area, which is 4 bus cycles rather than the 3 a 24-bit read would
// take; the same table gives +6 for an 8-bit stack area, which is the 6 bytes of
// PC and SR.  Both fall out of the BIU's real bus costs, so 18 is never
// hardcoded.
//
// Implemented: the whole instruction set except the four undefined-by-design
// opcode holes, which trap.  That is load, exchange, stack, ALU (including
// MUL/MULS/DIV/DIVS through t900_muldiv and MULA through the same unit), bit,
// shift, control transfer, the string / repeat group, LINK, UNLK, MINC/MDEC,
// BS1F/BS1B, RLD/RRD, and the system instructions NOP, HALT, EI/DI, LDF, INCF,
// DECF, SWI, RETI and LDC -- plus interrupt entry, the INTUNDEF trap and the
// four micro-DMA channels.
//
// Parameters
//  EI_SHADOW_RETI       0 (default): EI takes effect immediately, and one
//                       instruction of inhibit follows BOTH interrupt entry
//                       (TMP95C061 datasheet p.11, "sampled immediately after
//                       the start instruction of the interrupt processing is
//                       executed"; CPU900H p.6) AND a completed RETI.  Toshiba
//                       describes only the entry side; without the return
//                       inhibit a pending same-level request is accepted at the
//                       first boundary after RETI, so chained services run with
//                       zero instructions of the interrupted program between
//                       them.  1 selects the alternate bundle: inhibit after EI
//                       and RETI, none after entry, EI delayed.
//  STRING_INTERRUPTIBLE 1 (default) samples interrupts between the iterations of
//                       LDIR/LDDR/CPIR/CPDR, rewinding PC to the instruction so
//                       it resumes: "Interrupt requests are sampled every time
//                       1 item of data is loaded" (CPU900H p.97).  0 makes the
//                       repeat atomic.

module t900_seq
#(
	parameter EI_SHADOW_RETI       = 0,
	parameter STRING_INTERRUPTIBLE = 1
)
(
	input  wire        clk,
	input  wire        ce,
	input  wire        reset,

	// t900_biu data port ---------------------------------------------
	output wire        dreq,
	output wire        dwe,
	output wire [23:0] daddr,
	output wire [2:0]  dbytes,
	output wire [31:0] dwdata,
	input  wire [31:0] drdata,
	input  wire        ddone,

	// t900_biu's bus_req: high from T1 until the access completes. A low level
	// means the BIU is idle with nothing on the bus.
	input  wire        bus_busy,
	// The state an idle bus takes a DATA request in, i.e. that cycle's T1.
	// This is where the bus-penalty measurement starts; `!bus_busy` cannot
	// say it any more, because the BIU asserts `bus_req` in this very state.
	input  wire        d_bus_t1,
	// This state is a K2GE arbitration wait, not memory being slow.
	input  wire        d_wait_gfx,

	// t900_biu prefetch queue -----------------------------------------
	output wire        q_flush,
	output wire [23:0] q_new_pc,
	output wire        q_redirect,
	output wire        q_pop,
	output wire        prefetch_hold,
	output wire        prefetch_chain_hold,
	input  wire [7:0]  q_byte,
	input  wire [2:0]  q_count,

	// t900_decode ------------------------------------------------------
	output wire [7:0]  dec_b0,
	output wire [7:0]  dec_b1,
	input  wire [1:0]  dec_cls,
	input  wire [4:0]  dec_ea_kind,
	input  wire [2:0]  dec_ea_reg,
	input  wire [1:0]  dec_cls_size,
	input  wire        dec_needs_byte1,
	input  wire [7:0]  dec_op_id,
	input  wire [4:0]  dec_op1_kind,
	input  wire [4:0]  dec_op2_kind,
	input  wire [1:0]  dec_op1_size,
	input  wire [1:0]  dec_op2_size,
	input  wire [5:0]  dec_states,
	input  wire [3:0]  dec_group,
	input  wire        dec_undef,

	// t900_regfile ------------------------------------------------------
	output wire [1:0]  rf_rfp,
	output wire [7:0]  rf_ra_code,
	output wire [1:0]  rf_ra_size,
	input  wire [31:0] rf_ra_data,
	output wire [7:0]  rf_rb_code,
	output wire [1:0]  rf_rb_size,
	input  wire [31:0] rf_rb_data,
	output wire        rf_wr_en,
	output wire [7:0]  rf_wr_code,
	output wire [1:0]  rf_wr_size,
	output wire [31:0] rf_wr_data,

	// t900_alu (combinational) ------------------------------------------
	output wire [4:0]  alu_op,
	output wire [1:0]  alu_size,
	output wire [31:0] alu_a,
	output wire [31:0] alu_b,
	output wire [7:0]  alu_flags_in,
	input  wire [31:0] alu_result,
	input  wire [7:0]  alu_flags_out,

	// t900_shifter -------------------------------------------------------
	output wire        sh_start,
	output wire [2:0]  sh_op,
	output wire [1:0]  sh_size,
	output wire [4:0]  sh_count,
	output wire [31:0] sh_data,
	output wire [7:0]  sh_flags_in,
	input  wire        sh_done,
	input  wire        sh_finish,
	input  wire [31:0] sh_result,
	input  wire [7:0]  sh_flags_out,

	// t900_muldiv --------------------------------------------------------
	output wire        md_start,
	output wire [1:0]  md_op,
	output wire        md_size,
	output wire [31:0] md_a,
	output wire [31:0] md_b,
	input  wire        md_done,
	input  wire        md_finish,
	input  wire [31:0] md_finish_result,
	input  wire [31:0] md_result,
	input  wire        md_v,

	// interrupt controller -----------------------------------------------
	// The MCU shell owns "who wins": it presents one winning request with its
	// level and vector, and this core runs the Toshiba entry microprogram.
	// `int_ack` strobes for one ce on the state the request is accepted, which
	// is when the shell must clear that source's request flip-flop.
	input  wire        int_req,
	input  wire [2:0]  int_level,
	input  wire [7:0]  int_vector,
	output wire        int_ack,

	// Toshiba distinguishes two outcomes when an interrupt arrives during HALT
	// (TMP95C061 datasheet p.28 Table 3.4(2)). If the level is high enough for the
	// current IFF the CPU leaves HALT and takes the handler -- that is
	// `int_req` above, and `irq_ok` already applies the IFF test. If the level
	// is NOT high enough, the CPU still leaves HALT and returns to RUN mode,
	// but resumes at the instruction after HALT without taking the handler.
	// `halt_release` is that second case: the shell asserts it while any
	// request flip-flop is set at level 1-6, or NMI/INTWD, with no IFF test.
	// Without this input a below-mask request would never wake the CPU at all.
	input  wire        halt_release,

	// micro-DMA -------------------------------------------------------
	// DMAnV lives in the MCU shell (TMP95C061 datasheet p.14, I/O registers 7C-7F),
	// so the shell raises dma_req[n] while channel n is armed AND its source's
	// request flip-flop is set. The core answers:
	//   dma_ack[n]  one ce, on the state the transfer starts: clear the
	//               source request flip-flop
	//   dma_end[n]  one ce, on the state DMAC reaches zero: raise INTTCn and
	//               zero DMAnV, which is what makes the channel one-shot
	input  wire [3:0]  dma_req,
	output wire [3:0]  dma_ack,
	output wire [3:0]  dma_end,

	// retirement trace ----------------------------------------------------
	output reg         trace_valid,
	output wire [23:0] trace_pc,
	output wire [15:0] trace_sr,
	output wire [7:0]  trace_f,
	output wire        halted,

	// savestate tap ---------------------------------------------------
	input  wire [7:0]  ss_reg_addr,
	input  wire [31:0] ss_wdata,
	input  wire        ss_wren,
	output reg  [31:0] ss_rdata,
	input  wire        restore_hold,
	input  wire        pause_req,
	output wire        pause_ready
);

	`include "t900_defs.svh"

	// =====================================================================
	// Constants
	// =====================================================================

	localparam [1:0] CLS_MAIN = 2'd0;
	localparam [1:0] CLS_SRC  = 2'd1;
	localparam [1:0] CLS_DST  = 2'd2;
	localparam [1:0] CLS_REG  = 2'd3;

	// ALU operation codes, copied from t900_alu's localparams.
	localparam [4:0] ALU_ADD  = 5'h00;
	localparam [4:0] ALU_ADC  = 5'h01;
	localparam [4:0] ALU_SUB  = 5'h02;
	localparam [4:0] ALU_SBC  = 5'h03;
	localparam [4:0] ALU_CP   = 5'h04;
	localparam [4:0] ALU_INC  = 5'h05;
	localparam [4:0] ALU_DEC  = 5'h06;
	localparam [4:0] ALU_NEG  = 5'h07;
	localparam [4:0] ALU_AND  = 5'h08;
	localparam [4:0] ALU_OR   = 5'h09;
	localparam [4:0] ALU_XOR  = 5'h0A;
	localparam [4:0] ALU_CPL  = 5'h0B;
	localparam [4:0] ALU_EXTZ = 5'h10;
	localparam [4:0] ALU_EXTS = 5'h11;
	localparam [4:0] ALU_MIRR = 5'h12;
	localparam [4:0] ALU_DAA  = 5'h13;
	localparam [4:0] ALU_PASS = 5'h14;

	localparam [7:0] CODE_SP  = 8'hFC;  // XSP as a full register code
	localparam [7:0] CODE_A   = 8'hE0;  // A: current bank XWA, byte lane 0
	localparam [7:0] CODE_BC  = 8'hE4;  // BC / XBC, current bank
	localparam [7:0] CODE_XDE = 8'hE8;
	localparam [7:0] CODE_XHL = 8'hEC;

	// Addressing-mode state adder for the full-register-code prefix; see the
	// header for why this is 1 and not the table's 0.
	localparam [5:0] RCODE_ADDER = 6'd1;

	// Taken-branch adders, /H column (CPU900H p.167).
	localparam [5:0] ADD_JP_TRUE   = 6'd3;
	localparam [5:0] ADD_CALL_TRUE = 6'd8;
	localparam [5:0] ADD_DJNZ_TRUE = 6'd2;

	localparam [23:0] VECTOR_BASE = 24'hFFFF00;

	// FSM states.
	localparam [3:0] S_RESET  = 4'd0;
	localparam [3:0] S_FETCH0 = 4'd1;
	localparam [3:0] S_EA     = 4'd2;
	localparam [3:0] S_FETCH1 = 4'd3;
	localparam [3:0] S_OPF    = 4'd4;
	localparam [3:0] S_RD     = 4'd5;
	localparam [3:0] S_EXEC   = 4'd6;
	localparam [3:0] S_NEXT   = 4'd7;
	localparam [3:0] S_HALT   = 4'd8;
	localparam [3:0] S_INT    = 4'd9;   // vector entry / RETI
	localparam [3:0] S_DMA    = 4'd10;  // micro-DMA transfer
	localparam [3:0] S_STR    = 4'd11;  // string / repeat group and MULA
	localparam [3:0] S_PAUSE  = 4'd12;  // savestate pause

	// S_INT flavours.
	localparam [1:0] EV_SWI  = 2'd0;    // SWI n and the INTUNDEF trap
	localparam [1:0] EV_INT  = 2'd1;    // hardware interrupt entry
	localparam [1:0] EV_RETI = 2'd2;

	// Interrupt entry, both bus areas 16-bit (TMP95C061 datasheet p.11). The other
	// three rows of that table (22 / 24 / 28 states) come out of the BIU's
	// real bus costs, so only the reference row is a constant here.
	localparam [7:0] INT_STATES = 8'd18;

	// INTUNDEF is SWI 2 (TMP95C061 datasheet p.12 interrupt table), so it is charged
	// what SWI is charged; Appendix B has no row of its own for it.
	localparam [7:0] UNDEF_STATES = 8'd19;
	localparam [7:0] UNDEF_VECTOR = 8'h08;

	// Micro-DMA service cost (TMP95C061 datasheet p.16 == CPU900H p.167).
	localparam [7:0] DMA_STATES_BW  = 8'd8;
	localparam [7:0] DMA_STATES_L   = 8'd12;
	localparam [7:0] DMA_STATES_CNT = 8'd5;

	// Per-iteration cost of the repeat forms: LDIR/LDDR 7n+1, CPIR/CPDR 6n+1
	// (CPU900H p.159). The table entry already carries the "+1" iteration,
	// so each extra iteration adds these.
	localparam [5:0] STR_ITER_LD = 6'd7;
	localparam [5:0] STR_ITER_CP = 6'd6;

	// =====================================================================
	// State
	// =====================================================================

	reg [3:0]  st;

	reg [23:0] pc;              // address of the next instruction byte
	reg [15:0] sr;              // SYSM IFF2:0 MAX RFP2:0 : S Z 0 H 0 V N C
	reg [7:0]  f_alt;           // F'
	reg [15:0] intnest;
	reg        halt_r;

	// One instruction of interrupt shadow; which event arms it is the
	// EI_SHADOW_RETI parameter's business.
	reg        irq_shadow;
	reg        shadow_pend;

	// Micro-DMA control registers: real state, reachable by LDC and by the
	// savestate tap.
	reg [31:0] dmas [0:3];
	reg [31:0] dmad [0:3];
	reg [15:0] dmac [0:3];
	reg [7:0]  dmam [0:3];

	// Micro-program context
	reg [1:0]  ev_kind;
	reg [7:0]  ev_vec;      // vector table offset, relative to 0xFFFF00
	reg [2:0]  ev_lvl;      // accepted interrupt level
	reg [23:0] mp_base;     // saved stack pointer for the RETI pops
	reg [31:0] mp_data;     // data in flight through a transfer
	reg [2:0]  mp_szh;      // S, Z, H latched by a string compare
	reg [15:0] mula_lhs;    // (XDE) word, held while (XHL) is fetched
	reg [1:0]  dma_ch;

	// Latched instruction context
	reg [7:0]  ir0;
	reg [7:0]  ir1;
	reg [1:0]  cls_r;
	reg [4:0]  eak_r;
	reg [2:0]  eareg_r;
	reg [1:0]  clssz_r;
	reg [7:0]  rcode_r;
	reg [7:0]  op_r;
	reg [4:0]  k1_r;
	reg [4:0]  k2_r;
	reg [1:0]  s1_r;
	reg [1:0]  s2_r;
	reg [3:0]  grp_r;
	reg        undef_r;
	reg        dec_done;

	reg [23:0] ea;
	reg [31:0] ea_full;
	reg        ea_done;
	reg [23:0] ea_base;
	reg [31:0] ea_base_full;
	reg [1:0]  ea_step;
	reg [4:0]  ea_form;

	reg [31:0] opb;
	reg [31:0] opv1;
	reg [31:0] opv2;
	reg [2:0]  opf_cnt;

	reg [7:0]  used;
	reg [7:0]  budget;
	reg        budget_valid;
	reg [5:0]  pen_carry;

	reg        insn_done;
	// This microprogram is a micro-DMA transfer. A transfer starts in S_DMA
	// but RETIRES in S_NEXT like everything else, so the state alone cannot
	// tell the interrupt shadow which kind of event just finished.
	reg        dma_evt;
	reg        unit_started;
	reg [2:0]  mstep;
	reg [1:0]  pause_refill;  // release flush/refill phase for the queue
	reg        q_flush_r;
	reg [23:0] q_new_pc_r;
	reg        queue_refill;  // taken-transfer destination refill in progress

	reg        rw2_pend;
	reg [7:0]  rw2_code;
	reg [1:0]  rw2_size;
	reg [31:0] rw2_data;

	reg        wr_pend;
	reg [23:0] wr_addr;
	reg [2:0]  wr_n;
	reg [31:0] wr_data;

	// Data access engine
	reg        mem_pend;
	reg        mem_we_r;
	reg [23:0] mem_a;
	reg [2:0]  mem_n;
	reg [31:0] mem_wd;
	reg [4:0]  mem_ticks;
	reg [4:0]  mem_gfx;      // of those ticks, how many were K2GE arbitration
	reg [4:0]  mem_nom;
	reg        mem_run;      // the access is on the bus, not queued behind one
	reg [31:0] rd_data;
	reg        rd_have;
	reg        rd_started;

	// Combinational bus-access request (driven in the "issue arbitration"
	// section further down).
	wire        iss_req;
	wire        iss_we;
	wire [23:0] iss_addr;
	wire [2:0]  iss_n;
	wire [31:0] iss_wd;

	integer i;

	// The data port is driven combinationally on the state the sequencer
	// decides to access memory, so the BIU issues in that same state and an
	// access really costs one issue state plus its bus cycles. Once accepted
	// the request is held from the registers until ddone, as the BIU requires.
	//
	// Two rules make back-to-back accesses cost no handshake state, and both
	// are load-bearing for the BIU's `start_data = dreq && !d_active`:
	//
	//  - the request DROPS in the state ddone pulses in. That access is
	//    finished; holding the request there with its own address would make
	//    the BIU run it a second time -- a duplicate write, or a read that
	//    lands on top of the data just returned.
	//  - a newly presented request WINS over the registered one. In the ddone
	//    state `mem_pend` still reads 1 (it clears on this edge), so choosing
	//    on `mem_pend` would hand the BIU the address of the access that just
	//    completed instead of the new one.
	assign dreq   = (mem_pend && !ddone) || iss_req;
	assign dwe    = iss_req ? iss_we   : mem_we_r;
	assign daddr  = iss_req ? iss_addr : mem_a;
	assign dbytes = iss_req ? iss_n    : mem_n;
	assign dwdata = iss_req ? iss_wd   : mem_wd;

	assign halted   = halt_r;
	assign trace_sr = sr;
	assign trace_f  = sr[7:0];
	assign trace_pc = pc;
	assign rf_rfp   = sr[9:8];

	// The data port is free when no access is booked, and also in the state
	// ddone pulses in: that access is finished, so the next one may be
	// presented there and start its bus cycle immediately. That is what makes
	// two back-to-back accesses cost nothing between them, which the repeat
	// instructions need -- LDIR issues two per iteration and Toshiba's 7n+1
	// leaves no room for a handshake state on either.
	wire mem_free = !mem_pend || ddone;

	// `mem_we_r` still holds the completing access's direction in the ddone
	// state: `iss_req` only overwrites it on this edge.
	wire rd_ret   = ddone && !mem_we_r;

	// =====================================================================
	// Small helpers
	// =====================================================================

	// 3-bit current-bank register selector to a full register-code byte
	// (CPU900H p.42, p.45). The byte order is W A B C D E H L.
	function automatic [7:0] short_code(input [2:0] r, input [1:0] sz);
	begin
		if (sz == T900_SZ_B)
			short_code = 8'hE0 | {4'd0, r[2:1], 2'd0} | {7'd0, ~r[0]};
		else
			short_code = (r[2] ? 8'hF0 : 8'hE0) | {4'd0, r[1:0], 2'd0};
	end
	endfunction

	function automatic [2:0] size_bytes(input [1:0] sz);
	begin
		case (sz)
			T900_SZ_B: size_bytes = 3'd1;
			T900_SZ_W: size_bytes = 3'd2;
			default:   size_bytes = 3'd4;
		endcase
	end
	endfunction

	// Condition codes (CPU900H p.44). Signed less-than is S xor V.
	/* verilator lint_off UNUSEDSIGNAL */
	function automatic cond_true(input [3:0] cc, input [7:0] fl);
		reg lt;
	begin
		lt = fl[7] ^ fl[2];
		case (cc)
			4'h0:    cond_true = 1'b0;
			4'h1:    cond_true = lt;
			4'h2:    cond_true = lt | fl[6];
			4'h3:    cond_true = fl[6] | fl[0];
			4'h4:    cond_true = fl[2];
			4'h5:    cond_true = fl[7];
			4'h6:    cond_true = fl[6];
			4'h7:    cond_true = fl[0];
			4'h8:    cond_true = 1'b1;
			4'h9:    cond_true = ~lt;
			4'hA:    cond_true = ~(lt | fl[6]);
			4'hB:    cond_true = ~(fl[6] | fl[0]);
			4'hC:    cond_true = ~fl[2];
			4'hD:    cond_true = ~fl[7];
			4'hE:    cond_true = ~fl[6];
			default: cond_true = ~fl[0];
		endcase
	end
	endfunction
	/* verilator lint_on UNUSEDSIGNAL */

	function automatic [23:0] sx8(input [7:0] v);
	begin
		sx8 = {{16{v[7]}}, v};
	end
	endfunction

	function automatic [23:0] sx16(input [15:0] v);
	begin
		sx16 = {{8{v[15]}}, v};
	end
	endfunction

	// Full-register sign extensions. RETD and LDA operate on all 32 bits even
	// though only effective-address bits 23:0 leave the chip on its address bus
	// (CPU900H p.9, p.89, p.166). Keep these separate from the bus-width helpers
	// so memory accesses retain their explicit 24-bit contract.
	function automatic [31:0] sx8_32(input [7:0] v);
	begin
		sx8_32 = {{24{v[7]}}, v};
	end
	endfunction

	function automatic [31:0] sx16_32(input [15:0] v);
	begin
		sx16_32 = {{16{v[15]}}, v};
	end
	endfunction

	// SR's architecturally fixed bits, applied to every wholesale SR write
	// (POP SR, RETI, interrupt-entry restore and the savestate tap).
	// The 900/H has System mode and Maximum mode ONLY: SYSM (15) and MAX (11)
	// always read 1 and Toshiba says "do not set to 0"; RFP2 (10) and F bits 5
	// and 3 always read 0 (CPU900H p.3, p.6, p.7).
	function automatic [15:0] sr_fix(input [15:0] v);
	begin
		sr_fix = (v & 16'hFBD7) | 16'h8800;
	end
	endfunction

	// What the access would have cost in the memory Appendix B assumes: 16-bit,
	// zero wait, and aligned. One issue state plus two states per bus cycle.
	// Alignment is deliberately NOT considered here:
	// an odd word costs an extra bus cycle on real silicon too, and Toshiba
	// says so (CPU900H p.30 table 6.1), so that extra cycle has to show up as
	// a penalty rather than be absorbed.
	function automatic [4:0] nominal_states(input [2:0] nb);
	begin
		case (nb)
			3'd1:    nominal_states = 5'd3;   // 1 cycle
			3'd2:    nominal_states = 5'd3;   // 1 cycle
			3'd3:    nominal_states = 5'd5;   // 2 cycles
			default: nominal_states = 5'd5;   // 2 cycles
		endcase
	end
	endfunction

	// One budget state for every state an access spends on the bus beyond what
	// aligned 16-bit zero-wait memory would have needed. Charging it as it
	// happens (rather than in one lump when the access retires) is what lets a
	// posted store hold its own instruction back when the region is slow.
	wire mem_over = mem_run && !ddone && (mem_ticks >= mem_nom);

	// The same test with the K2GE's arbitration states discounted. Every state
	// the composition pass held the bus raises the bar by one, so it can never
	// produce a charge, while a state slow MEMORY cost still can. Counting
	// rather than gating per state is deliberate: `ddone` is registered, so a
	// charge lands one state after the wait that caused it and a per-state gate
	// would credit only the first of them. The count is always incremented
	// before the charge it pays for, which is the only ordering this needs.
	wire [4:0] mem_arb_bar  = mem_nom + mem_gfx;
	wire       mem_over_arb = mem_run && !ddone && (mem_ticks >= mem_arb_bar);

	// =====================================================================
	// Events serviced between instructions
	// =====================================================================
	//
	// Priority: micro-DMA, then interrupts, then the next instruction. A
	// maskable source is accepted when its level is at least IFF; NMI and WDT
	// arrive at level 7, which is always at least IFF, so they need no special
	// case. Micro-DMA is presented at level 6 whatever the source's own level
	// is, which is exactly "IFF <= 6" (TMP95C061 datasheet p.14).

	wire [2:0] iff_mask = sr[14:12];

	// `irq_want` is the request WITHOUT the shadow. It exists because the
	// repeat instructions need it: see `str_irq`.
	wire irq_want = int_req && (int_level >= iff_mask);
	wire irq_ok = irq_want && !irq_shadow;
	wire dma_ok = (dma_req != 4'd0) && (iff_mask != 3'd7);

	// Channel 0 wins, then 1, 2, 3 (TMP95C061 datasheet p.14).
	wire [1:0] dma_sel = dma_req[0] ? 2'd0 : dma_req[1] ? 2'd1 :
	                     dma_req[2] ? 2'd2 : 2'd3;

	// The channel whose mode registers matter this state: the winner while the
	// request is being accepted, the latched one once the transfer is running.
	wire [1:0] dma_cur  = (st == S_DMA) ? dma_ch : dma_sel;
	wire [4:0] dma_mode = dmam[dma_cur][4:0];
	wire [2:0] dma_kind = dma_mode[4:2];
	wire [1:0] dma_zz   = dma_mode[1:0];

	// ZZ = 0/1/2 for byte/word/long; 3 is reserved and is treated as long.
	wire [2:0] dma_nb = (dma_zz == 2'd0) ? 3'd1 : (dma_zz == 2'd1) ? 3'd2 : 3'd4;

	// 10100 is counter mode: no transfer at all, DMAS just counts up.
	wire dma_counter = (dma_mode == 5'h14);

	wire [7:0] dma_budget = dma_counter    ? DMA_STATES_CNT :
	                        (dma_zz >= 2'd2) ? DMA_STATES_L : DMA_STATES_BW;

	// =====================================================================
	// Decode, live for the tick that latches it
	// =====================================================================

	assign dec_b0 = (st == S_FETCH0) ? q_byte : ir0;
	assign dec_b1 = (st == S_FETCH1) ? q_byte : ir1;

	wire pop_ok = (q_count != 3'd0);

	// Only trust the decoder in a state that is actually consuming a byte: a
	// stalled fetch presents whatever the empty queue happens to hold, and an
	// ungated decode of that garbage would launch a spurious bus read.
	wire decode_live = q_pop &&
	                   (((st == S_FETCH0) && !dec_needs_byte1) || (st == S_FETCH1));

	wire [1:0] cur_cls   = (st == S_FETCH0) ? dec_cls      : cls_r;
	wire [4:0] cur_eak   = (st == S_FETCH0) ? dec_ea_kind  : eak_r;
	wire [2:0] cur_eareg = (st == S_FETCH0) ? dec_ea_reg   : eareg_r;
	wire [1:0] cur_clssz = (st == S_FETCH0) ? dec_cls_size : clssz_r;

	wire [7:0] cur_op   = decode_live ? dec_op_id    : op_r;
	wire [4:0] cur_k1   = decode_live ? dec_op1_kind : k1_r;
	wire [4:0] cur_k2   = decode_live ? dec_op2_kind : k2_r;
	wire [3:0] cur_gp   = decode_live ? dec_group    : grp_r;
	wire       cur_ud   = decode_live ? dec_undef    : undef_r;
	wire       op_avail = decode_live || dec_done;

	wire [1:0] raw_s1 = decode_live ? dec_op1_size : s1_r;
	wire [1:0] raw_s2 = decode_live ? dec_op2_size : s2_r;
	wire [1:0] sz1 = (raw_s1 == T900_SZ_CLASS) ? cur_clssz : raw_s1;
	wire [1:0] sz2 = (raw_s2 == T900_SZ_CLASS) ? cur_clssz : raw_s2;

	wire is_muldiv = (cur_op == T900_OP_MUL) || (cur_op == T900_OP_MULS) ||
	                 (cur_op == T900_OP_DIV) || (cur_op == T900_OP_DIVS);
	wire shift_op  = (cur_op == T900_OP_RLC) || (cur_op == T900_OP_RRC) ||
	                 (cur_op == T900_OP_RL)  || (cur_op == T900_OP_RR)  ||
	                 (cur_op == T900_OP_SLA) || (cur_op == T900_OP_SRA) ||
	                 (cur_op == T900_OP_SLL) || (cur_op == T900_OP_SRL);

	// MUL/DIV write a destination twice the class width ("z2" in the tables).
	wire [1:0] wide_sz = (cur_clssz == T900_SZ_B) ? T900_SZ_W : T900_SZ_L;

	// =====================================================================
	// Register file port muxing
	// =====================================================================
	//
	// Port A carries the effective-address base while the EA is being built
	// and the class's own register operand ("r") afterwards. Port B carries
	// the short register named by the opcode byte ("R"). Stack operations
	// need XSP too, so XSP rides whichever port the class is not using; the
	// accumulator forms borrow port B, which they never use for anything else.

	wire cls_reg_is_op1 = (cur_k1 == T900_K_RCODE_R);

	wire [1:0] cls_reg_sz = (is_muldiv && cls_reg_is_op1) ? wide_sz :
	                        (cls_reg_is_op1 ? sz1 : sz2);

	// A byte-size MUL/MULS/DIV/DIVS names a 16-bit destination PAIR with a
	// 3-bit code, and the pair is the code shifted right by one: 001 = WA,
	// 011 = BC, 101 = DE, 111 = HL, with IX/IY/IZ/SP marked "Specification
	// not possible" (CPU900H p.109, the "RR for the MUL RR,r and MUL RR,(mem)
	// instructions" table). Toshiba only defines the odd codes; the even ones
	// fall on the same pair. The word-size forms use
	// the code directly, unshifted (same page, right-hand table).
	//
	// Both destinations obey it, and they are different wires: `MUL rr,#`
	// names its destination with the class register, while `MUL R,r` and
	// `MUL R,(mem)` name it with the short code out of the second opcode
	// byte. Widening only the first is what left the R-destination forms
	// operating on the neighbouring pair.
	wire muldiv_pair = is_muldiv && (cur_clssz == T900_SZ_B);

	wire [2:0] cls_wide_sel = muldiv_pair ? {1'b0, cur_eareg[2:1]} : cur_eareg;
	wire [2:0] cls_sel = (is_muldiv && cls_reg_is_op1) ? cls_wide_sel : cur_eareg;

	wire [7:0] cls_reg_code =
		(cur_eak == T900_K_RCODE) ? rcode_r : short_code(cls_sel, cls_reg_sz);

	wire       k1_is_R  = (cur_k1 == T900_K_SHORT_R);
	wire [2:0] rsel_raw = (cur_cls == CLS_MAIN) ? dec_b0[2:0] : dec_b1[2:0];
	wire [2:0] rsel3    = (muldiv_pair && k1_is_R) ? {1'b0, rsel_raw[2:1]} : rsel_raw;
	wire [1:0] shortR_sz = (is_muldiv && k1_is_R) ? wide_sz : (k1_is_R ? sz1 : sz2);
	wire [7:0] shortR_code = short_code(rsel3, shortR_sz);

	wire uses_A = op_avail && ((cur_k1 == T900_K_REG_A) || (cur_k2 == T900_K_REG_A));

	wire op_uses_sp = (cur_op == T900_OP_PUSH) || (cur_op == T900_OP_POP)  ||
	                  (cur_op == T900_OP_CALL) || (cur_op == T900_OP_CALR) ||
	                  (cur_op == T900_OP_RET)  || (cur_op == T900_OP_RETD) ||
	                  (cur_op == T900_OP_SWI)  || (cur_op == T900_OP_LINK);
	wire want_sp = op_avail && op_uses_sp;
	wire sp_on_b = (cur_cls == CLS_REG);

	// String group register plan (CPU900H p.96). The first opcode byte names
	// the source pointer with a short register code; the destination is the
	// register one below it, so 0x83 means (XDE+) <- (XHL+) and 0x85 means
	// (XIX+) <- (XIY+). BC is always the counter.
	wire [7:0] str_src_code = short_code(eareg_r, T900_SZ_L);
	wire [7:0] str_dst_code = short_code(eareg_r - 3'd1, T900_SZ_L);
	wire [2:0] str_nb       = size_bytes(clssz_r);

	wire str_is_cp  = (cur_op == T900_OP_CPI)  || (cur_op == T900_OP_CPIR) ||
	                  (cur_op == T900_OP_CPD)  || (cur_op == T900_OP_CPDR);
	wire str_is_dec = (cur_op == T900_OP_LDD)  || (cur_op == T900_OP_LDDR) ||
	                  (cur_op == T900_OP_CPD)  || (cur_op == T900_OP_CPDR);
	wire str_repeat = (cur_op == T900_OP_LDIR) || (cur_op == T900_OP_LDDR) ||
	                  (cur_op == T900_OP_CPIR) || (cur_op == T900_OP_CPDR);

	wire is_mula = (cur_op == T900_OP_MULA);

	// MULA fetches (XDE) one state earlier than S_STR could, by steering port A
	// from the decode of the second opcode byte. Without it the instruction
	// misses its 19-state budget by one: the word multiply alone is 11 ticks.
	wire mula_pre = (st == S_FETCH1) && q_pop && (dec_op_id == T900_OP_MULA);

	// Port A steering for the micro-programs. The micro-programs only ever
	// read 32-bit registers, so their port-A size is the constant T900_SZ_L.
	reg [7:0] mp_ra_code;
	reg       mp_ra_sel;

	always @(*) begin
		mp_ra_sel  = 1'b1;
		mp_ra_code = CODE_SP;
		if (st == S_INT) begin
			mp_ra_code = CODE_SP;             // both pushes and both pops
		end else if (mula_pre) begin
			mp_ra_code = CODE_XDE;
		end else if (st == S_STR) begin
			if (is_mula)
				mp_ra_code = (mstep == 3'd4) ? cls_reg_code : CODE_XHL;
			else
				mp_ra_code = (mstep == 3'd1) ? str_dst_code : str_src_code;
		end else begin
			mp_ra_sel = 1'b0;
		end
	end

	// Port B carries the counter for the string group and the A / WA operand
	// for the compares.
	wire str_cmp_b = (st == S_STR) && str_is_cp && (mstep == 3'd0);
	wire str_bc_b  = (st == S_STR) && !is_mula && (mstep == 3'd2);

	// EA phase: only classes have a class effective address.
	wire ea_phase = ((st == S_FETCH0) && dec_needs_byte1) || (st == S_EA);

	reg [7:0] ea_base_code;
	reg [1:0] ea_base_size;

	always @(*) begin
		ea_base_code = short_code(cur_eareg, T900_SZ_L);
		ea_base_size = T900_SZ_L;
		if (st == S_EA) begin
			case (eak_r)
				T900_K_PRE_DEC, T900_K_POST_INC, T900_K_RCODE: begin
					ea_base_code = q_byte;
					ea_base_size = T900_SZ_L;
				end
				T900_K_EXT: begin
					ea_base_code = q_byte;
					if (ea_step == 2'd2)
						ea_base_size = (ea_form == 5'h07) ? T900_SZ_W : T900_SZ_B;
					else
						ea_base_size = T900_SZ_L;
				end
				default: ;
			endcase
		end
	end

	assign rf_ra_code = mp_ra_sel             ? mp_ra_code   :
	                    ea_phase              ? ea_base_code :
	                    (want_sp && !sp_on_b) ? CODE_SP      : cls_reg_code;
	assign rf_ra_size = mp_ra_sel             ? T900_SZ_L    :
	                    ea_phase              ? ea_base_size :
	                    (want_sp && !sp_on_b) ? T900_SZ_L    : cls_reg_sz;
	assign rf_rb_code = str_cmp_b             ? CODE_A  :
	                    str_bc_b              ? CODE_BC :
	                    (want_sp && sp_on_b)  ? CODE_SP :
	                    uses_A                ? CODE_A  : shortR_code;
	assign rf_rb_size = str_cmp_b             ? clssz_r :
	                    str_bc_b              ? T900_SZ_W :
	                    (want_sp && sp_on_b)  ? T900_SZ_L :
	                    uses_A                ? T900_SZ_B : shortR_sz;

	wire [31:0] sp_val  = sp_on_b ? rf_rb_data : rf_ra_data;
	wire [31:0] cls_reg = rf_ra_data;
	wire [31:0] shortR  = rf_rb_data;
	wire [7:0]  reg_a   = rf_rb_data[7:0];

	// =====================================================================
	// Effective address
	// =====================================================================

	wire [23:0] pc_now = pc + {23'd0, q_pop};

	// Combinational EA evaluation for this tick. `ea_val_now` is the 24-bit bus
	// address; `ea_full_now` retains the architectural 32-bit value consumed by
	// LDA. `ea_ready_now` says both are final.
	reg [23:0] ea_val_now;
	reg [31:0] ea_full_now;
	reg        ea_ready_now;
	reg        ea_last;        // this tick consumes the last EA byte
	reg [1:0]  ea_step_next;
	reg [4:0]  ea_form_next;
	reg        ea_form_set;
	reg        ea_base_latch;

	always @(*) begin
		ea_val_now   = ea;
		ea_full_now  = ea_full;
		ea_ready_now = ea_done;
		ea_last      = 1'b0;
		ea_step_next = ea_step;
		ea_form_next = ea_form;
		ea_form_set  = 1'b0;
		ea_base_latch= 1'b0;

		if ((st == S_FETCH0) && q_pop && dec_needs_byte1) begin
			case (dec_ea_kind)
				T900_K_XRR: begin
					ea_val_now   = rf_ra_data[23:0];
					ea_full_now  = rf_ra_data;
					ea_ready_now = 1'b1;
					ea_last      = 1'b1;
				end
				T900_K_SHORT_R: begin
					ea_val_now   = 24'd0;
					ea_full_now  = 32'd0;
					ea_ready_now = 1'b1;
					ea_last      = 1'b1;
				end
				default: ea_ready_now = 1'b0;
			endcase
		end else if ((st == S_EA) && q_pop) begin
			case (eak_r)
				T900_K_XRR_D8: begin
					ea_val_now   = rf_ra_data[23:0] + sx8(q_byte);
					ea_full_now  = rf_ra_data + sx8_32(q_byte);
					ea_ready_now = 1'b1;
					ea_last      = 1'b1;
				end
				T900_K_N8: begin
					ea_val_now   = {16'd0, q_byte};
					ea_full_now  = {24'd0, q_byte};
					ea_ready_now = 1'b1;
					ea_last      = 1'b1;
				end
				T900_K_N16: begin
					if (ea_step == 2'd0) begin
						ea_val_now   = {16'd0, q_byte};
						ea_full_now  = {24'd0, q_byte};
						ea_step_next = 2'd1;
					end else begin
						ea_val_now   = {8'd0, q_byte, ea[7:0]};
						ea_full_now  = {16'd0, q_byte, ea[7:0]};
						ea_ready_now = 1'b1;
						ea_last      = 1'b1;
					end
				end
				T900_K_N24: begin
					if (ea_step == 2'd0) begin
						ea_val_now   = {16'd0, q_byte};
						ea_full_now  = {24'd0, q_byte};
						ea_step_next = 2'd1;
					end else if (ea_step == 2'd1) begin
						ea_val_now   = {8'd0, q_byte, ea[7:0]};
						ea_full_now  = {16'd0, q_byte, ea[7:0]};
						ea_step_next = 2'd2;
					end else begin
						ea_val_now   = {q_byte, ea[15:0]};
						ea_full_now  = {8'd0, q_byte, ea[15:0]};
						ea_ready_now = 1'b1;
						ea_last      = 1'b1;
					end
				end
				T900_K_PRE_DEC: begin
					// The step is in the operand byte's low two bits, NOT in
					// the operand size (CPU900H code map).
					ea_val_now   = rf_ra_data[23:0] - (24'd1 << q_byte[1:0]);
					ea_full_now  = rf_ra_data - (32'd1 << q_byte[1:0]);
					ea_ready_now = 1'b1;
					ea_last      = 1'b1;
				end
				T900_K_POST_INC: begin
					ea_val_now   = rf_ra_data[23:0];
					ea_full_now  = rf_ra_data;
					ea_ready_now = 1'b1;
					ea_last      = 1'b1;
				end
				T900_K_RCODE: begin
					ea_val_now   = 24'd0;
					ea_full_now  = 32'd0;
					ea_ready_now = 1'b1;
					ea_last      = 1'b1;
				end
				T900_K_EXT: begin
					if (ea_step == 2'd0) begin
						ea_form_set  = 1'b1;
						ea_base_latch= 1'b1;
						if (q_byte[1:0] == 2'b00) begin
							ea_form_next = 5'h00;
							ea_val_now   = rf_ra_data[23:0];
							ea_full_now  = rf_ra_data;
							ea_ready_now = 1'b1;
							ea_last      = 1'b1;
						end else begin
							ea_step_next = 2'd1;
							ea_form_next = (q_byte == 8'h13) ? 5'h13 :
							               (q_byte == 8'h03) ? 5'h03 :
							               (q_byte == 8'h07) ? 5'h07 : 5'h01;
						end
					end else if ((ea_form == 5'h03) || (ea_form == 5'h07)) begin
						// (r32+r8) / (r32+r16): two register-code bytes.
						if (ea_step == 2'd1) begin
							ea_base_latch = 1'b1;
							ea_step_next  = 2'd2;
						end else begin
							ea_val_now = ea_base +
							             ((ea_form == 5'h07) ? sx16(rf_ra_data[15:0])
							                                 : sx8(rf_ra_data[7:0]));
							ea_full_now = ea_base_full +
							              ((ea_form == 5'h07) ? sx16_32(rf_ra_data[15:0])
							                                  : sx8_32(rf_ra_data[7:0]));
							ea_ready_now = 1'b1;
							ea_last      = 1'b1;
						end
					end else begin
						// (r32+d16) and the undocumented 0x13 = (PC+d16)
						// used by LDAR.
						if (ea_step == 2'd1) begin
							ea_val_now   = {16'd0, q_byte};
							ea_full_now  = {24'd0, q_byte};
							ea_step_next = 2'd2;
						end else begin
							ea_val_now = ((ea_form == 5'h13) ? pc_now : ea_base) +
							             sx16({q_byte, ea[7:0]});
							ea_full_now = ((ea_form == 5'h13) ? {8'd0, pc_now} : ea_base_full) +
							              sx16_32({q_byte, ea[7:0]});
							ea_ready_now = 1'b1;
							ea_last      = 1'b1;
						end
					end
				end
				default: begin
					ea_ready_now = 1'b1;
					ea_last      = 1'b1;
				end
			endcase
		end
	end

	// Addressing-mode state adders (CPU900H p.167 table (10)).
	reg [5:0] ea_adder;
	always @(*) begin
		case (eak_r)
			T900_K_XRR:      ea_adder = 6'd0;
			T900_K_XRR_D8:   ea_adder = 6'd1;
			T900_K_N8:       ea_adder = 6'd1;
			T900_K_N16:      ea_adder = 6'd2;
			// MEASURED DEVIATION from the table, destination class only.
			// Toshiba's table (10) gives 3 for a 24-bit absolute, and that is
			// right for a SOURCE operand; a 24-bit absolute DESTINATION costs
			// one state more than the published table (measured on hardware).
			//
			// `cls_r`, not `cur_cls`: provably identical here and one mux
			// shallower.  `ea_adder` is consumed only inside `tbl_total` under
			// `st == S_FETCH1`, and `cur_cls` is defined as
			// `(st == S_FETCH0) ? dec_cls : cls_r` -- so in the only state that
			// reads this, `cur_cls` IS `cls_r`.  Using the registered copy
			// keeps the live decoder mux out of the budget adder chain.
			T900_K_N24:      ea_adder = (cls_r == CLS_DST) ? 6'd4 : 6'd3;
			T900_K_PRE_DEC:  ea_adder = 6'd1;
			T900_K_POST_INC: ea_adder = 6'd1;
			T900_K_SHORT_R:  ea_adder = 6'd0;
			T900_K_RCODE:    ea_adder = RCODE_ADDER;
			T900_K_EXT:      ea_adder = (ea_form == 5'h00) ? 6'd1 : 6'd3;
			default:         ea_adder = 6'd0;
		endcase
	end

	// =====================================================================
	// Operand byte plan
	// =====================================================================

	function automatic [2:0] kind_bytes(input [4:0] k, input [1:0] sz);
	begin
		case (k)
			T900_K_IMM8, T900_K_MEM8, T900_K_D8, T900_K_CR: kind_bytes = 3'd1;
			T900_K_IMM16, T900_K_MEM16, T900_K_D16:         kind_bytes = 3'd2;
			T900_K_IMM24:                                   kind_bytes = 3'd3;
			T900_K_IMM32:                                   kind_bytes = 3'd4;
			T900_K_IMM:                                     kind_bytes = size_bytes(sz);
			default:                                        kind_bytes = 3'd0;
		endcase
	end
	endfunction

	// LDX is `F7 00 n8 00 imm8 00`: five bytes follow the opcode and only the
	// second and fourth carry payload (CPU900H p.164).
	wire is_ldx = (cur_op == T900_OP_LDX);

	wire [2:0] n1 = is_ldx ? 3'd0 : kind_bytes(cur_k1, sz1);
	wire [2:0] n2 = is_ldx ? 3'd0 : kind_bytes(cur_k2, sz2);
	wire [2:0] opf_total = is_ldx ? 3'd5 : (n1 + n2);

	// The SAME three numbers, built from the LATCHED decode instead of the live
	// one.  A timing split, not a behaviour change: `cur_op`, `cur_k1`,
	// `cur_k2`, `sz1` and `sz2` follow the live decoder only while
	// `decode_live` is true, which is only in S_FETCH0 and S_FETCH1, and
	// everything below is used only under `st == S_OPF`, where those five wires
	// are already `op_r`, `k1_r`, `k2_r` and the two latched sizes.
	//
	// It matters because the chain
	//   q_byte -> decode -> cur_op -> n1 -> in_op1/byte_pos -> opb_now
	//     -> o1/o2 -> cr_code -> the control-register read -> rf_wr_data
	// can never carry a real value: its first half needs `st == S_FETCH1` for
	// the decode to be live and its second half needs `st == S_OPF` for
	// `opb_now` to be selected into `o1`/`o2`, and no state is both.  The two
	// `st` tests sit ten levels apart, so the analyser cannot see that.
	// Sourcing the operand-byte plan from the latched decode makes the
	// contradiction structural and the path stops existing.
	//
	// `opf_total` keeps BOTH forms because it has readers on both sides: the
	// S_FETCH0/S_FETCH1 next-state arms and `far_rd` need the live one, the
	// S_OPF readers take the held one.
	wire [1:0] sz1_held = (s1_r == T900_SZ_CLASS) ? clssz_r : s1_r;
	wire [1:0] sz2_held = (s2_r == T900_SZ_CLASS) ? clssz_r : s2_r;
	wire       is_ldx_held = (op_r == T900_OP_LDX);

	wire [2:0] n1_held = is_ldx_held ? 3'd0 : kind_bytes(k1_r, sz1_held);
	wire [2:0] n2_held = is_ldx_held ? 3'd0 : kind_bytes(k2_r, sz2_held);
	wire [2:0] opf_total_held = is_ldx_held ? 3'd5 : (n1_held + n2_held);

	wire in_op1 = is_ldx_held ? 1'b1 : (opf_cnt < n1_held);
	wire [1:0] byte_pos = in_op1 ? opf_cnt[1:0] : (opf_cnt[1:0] - n1_held[1:0]);

	reg [31:0] opb_now;
	always @(*) begin
		opb_now = opb;
		case (byte_pos)
			2'd0: opb_now[7:0]   = q_byte;
			2'd1: opb_now[15:8]  = q_byte;
			2'd2: opb_now[23:16] = q_byte;
			2'd3: opb_now[31:24] = q_byte;
		endcase
	end

	wire opf_live = (st == S_OPF) && q_pop;
	wire [31:0] o1 = (opf_live && in_op1)  ? opb_now : opv1;
	wire [31:0] o2 = (opf_live && !in_op1) ? opb_now : opv2;

	// =====================================================================
	// Memory reads the instruction needs
	// =====================================================================

	wire src_rd  = (cur_cls == CLS_SRC) && ea_ready_now;

	// A source-class (#16) access becomes requestable when its high address
	// byte is consumed. Reserve an otherwise-idle BIU during the preceding
	// low-byte state so that address completion and the read's T1 issue can
	// share the next Toshiba state. Without this narrow lookahead the
	// prefetcher starts a fill in between, and LD R,(#16) measures seven
	// states instead of table 4 plus the two-state addressing adder.
	wire source_prefetch_hold = (st == S_EA) && q_pop &&
	                            (cur_cls == CLS_SRC) &&
	                            (eak_r == T900_K_N16) &&
	                            (ea_step == 2'd0);

	wire dst_rmw = op_avail && (cur_cls == CLS_DST) &&
	               ((cur_op == T900_OP_RES)  || (cur_op == T900_OP_SET)   ||
	                (cur_op == T900_OP_CHG)  || (cur_op == T900_OP_BIT)   ||
	                (cur_op == T900_OP_TSET) || (cur_op == T900_OP_ANDCF) ||
	                (cur_op == T900_OP_ORCF) || (cur_op == T900_OP_XORCF) ||
	                (cur_op == T900_OP_LDCF) || (cur_op == T900_OP_STCF));

	// A conditional RET whose condition is FALSE must leave XSP alone, and it
	// must not read the stack either. A speculative read is not free: a cycle
	// starting from an idle bus costs three states here, and a not-taken
	// RET cc has a four-state budget with nowhere to hide them (dst map
	// F0h+cc, +8 states if taken).
	wire cc1_taken    = cond_true(dec_b1[3:0], sr[7:0]);
	wire ret_cc_false = (cur_op == T900_OP_RET) && (cur_k1 == T900_K_CC) &&
	                    !cc1_taken;

	wire stack_rd = op_avail &&
	                ((cur_op == T900_OP_POP) ||
	                 ((cur_op == T900_OP_RET) && !ret_cc_false) ||
	                 (cur_op == T900_OP_RETD));

	// UNLK reads the saved frame pointer back through the register the
	// instruction names, not through XSP (CPU900H p.151).
	wire unlk_rd = op_avail && (cur_op == T900_OP_UNLK);

	// LD (mem),(nn) fetches the far operand once its address is complete.
	//
	// "Complete" means the state its LAST address byte is popped, not the one
	// after: `o2` already carries that byte (see `opb_now`), and `bytes_done`
	// declares the instruction ready to commit in that very state.
	wire opf_last = (st == S_OPF) && q_pop && ((opf_cnt + 3'd1) == opf_total_held);
	wire far_rd   = op_avail && (cur_k2 == T900_K_MEM16) &&
	                ((opf_cnt == opf_total) || opf_last);

	reg [2:0] stack_n;
	always @(*) begin
		if ((cur_op == T900_OP_RET) || (cur_op == T900_OP_RETD))
			stack_n = 3'd4;
		else if (cur_k1 == T900_K_SR)
			stack_n = 3'd2;
		else if ((cur_k1 == T900_K_FLAGS) || (cur_k1 == T900_K_REG_A) ||
		         (cur_k1 == T900_K_IMM8))
			stack_n = 3'd1;
		else if (cur_k1 == T900_K_IMM16)
			stack_n = 3'd2;
		else
			// A MEMORY operand takes its width from the decoded operand size,
			// NOT from the addressing class. PUSH (mem) lives in the sized
			// source classes and its table size is `z`, so the two agree there
			// -- but POP (mem) lives in the DESTINATION class (0xB0-0xBF),
			// which carries no size of its own, and the word/byte choice is in
			// the opcode: dst 0x04 is POP (mem) byte and dst 0x06 is POP (mem)
			// WORD (CPU900H dst map).
			// Reading the class size made every `POPW (mem)` a BYTE pop: one
			// byte off the stack and XSP advanced by 1 instead of 2. The store
			// was the right width, so the destination even looked plausible --
			// it just got the high byte from nowhere and leaked two bytes of
			// stack per call.
			stack_n = size_bytes(sz1);
	end

	reg        rd_want;
	reg [23:0] rd_addr;
	reg [2:0]  rd_n;

	always @(*) begin
		rd_want = 1'b0;
		rd_addr = ea_val_now;
		rd_n    = size_bytes(cur_clssz);
		if (src_rd) begin
			rd_want = 1'b1;
		end else if (stack_rd) begin
			rd_want = 1'b1;
			rd_addr = sp_val[23:0];
			rd_n    = stack_n;
		end else if (unlk_rd) begin
			rd_want = 1'b1;
			rd_addr = cls_reg[23:0];
			rd_n    = 3'd4;
		end else if (dst_rmw) begin
			rd_want = 1'b1;
			rd_addr = ea;
			rd_n    = 3'd1;
		end else if (far_rd) begin
			rd_want = 1'b1;
			rd_addr = o2[23:0];
			rd_n    = size_bytes(sz2);
		end
	end

	wire in_insn = (st == S_FETCH0) || (st == S_EA)   || (st == S_FETCH1) ||
	               (st == S_OPF)    || (st == S_RD)   || (st == S_EXEC)   ||
	               (st == S_NEXT)   || (st == S_INT)  || (st == S_DMA)    ||
	               (st == S_STR);

	wire rd_ok = !rd_want || rd_have || rd_ret;
	wire [31:0] rd_now = rd_ret ? drdata : rd_data;

	// =====================================================================
	// Operand resolution
	// =====================================================================

	// For the bit, shift, INC/DEC and SCC families operand 1 is a parameter
	// (bit number, count, amount, condition) and operand 2 is the location.
	wire is_minc = (cur_op == T900_OP_MINC1) || (cur_op == T900_OP_MINC2) ||
	               (cur_op == T900_OP_MINC4);
	wire is_mdec = (cur_op == T900_OP_MDEC1) || (cur_op == T900_OP_MDEC2) ||
	               (cur_op == T900_OP_MDEC4);

	// RLD and RRD are filed under the shift group but read the other way
	// round: operand 1 is A, the destination, and operand 2 is the memory
	// byte they rotate through it.
	wire is_rxd = (cur_op == T900_OP_RLD) || (cur_op == T900_OP_RRD);

	wire dst_is_op2 = ((cur_gp == T900_G_SHIFT) && (cur_k2 != T900_K_NONE) && !is_rxd) ||
	                  ((cur_gp == T900_G_BIT)   && (cur_k2 != T900_K_NONE)) ||
	                  (cur_op == T900_OP_INC) || (cur_op == T900_OP_DEC) ||
	                  (cur_op == T900_OP_SCC) || is_minc || is_mdec;

	wire [4:0] dk  = dst_is_op2 ? cur_k2 : cur_k1;
	wire [4:0] skk = dst_is_op2 ? cur_k1 : cur_k2;
	wire [1:0] dsz = dst_is_op2 ? sz2 : sz1;

	reg [31:0] dst_val;
	always @(*) begin
		case (dk)
			T900_K_SHORT_R:   dst_val = shortR;
			T900_K_RCODE_R:   dst_val = cls_reg;
			T900_K_REG_A:     dst_val = {24'd0, reg_a};
			T900_K_SR:        dst_val = {16'd0, sr};
			T900_K_FLAGS:     dst_val = {24'd0, sr[7:0]};
			T900_K_FLAGS_ALT: dst_val = {24'd0, f_alt};
			T900_K_MEM, T900_K_MEM8, T900_K_MEM16: dst_val = rd_now;
			default:          dst_val = 32'd0;
		endcase
	end

	reg [31:0] src_val;
	always @(*) begin
		case (skk)
			T900_K_SHORT_R:   src_val = shortR;
			T900_K_RCODE_R:   src_val = cls_reg;
			T900_K_REG_A:     src_val = {24'd0, reg_a};
			T900_K_SR:        src_val = {16'd0, sr};
			T900_K_FLAGS:     src_val = {24'd0, sr[7:0]};
			T900_K_FLAGS_ALT: src_val = {24'd0, f_alt};
			T900_K_MEM, T900_K_MEM8, T900_K_MEM16: src_val = rd_now;
			T900_K_IMM3:      src_val = {29'd0, dec_b1[2:0]};
			default:          src_val = dst_is_op2 ? o1 : o2;
		endcase
	end

	// =====================================================================
	// Function units
	// =====================================================================

	// INC/DEC take 1..8, with the encoded 0 meaning 8 (CPU900H p.83).
	wire [3:0] inc_amt = (dec_b1[2:0] == 3'd0) ? 4'd8 : {1'b0, dec_b1[2:0]};

	reg [4:0]  alu_op_r;
	reg [31:0] alu_b_r;

	always @(*) begin
		alu_op_r = ALU_PASS;
		alu_b_r  = src_val;
		case (cur_op)
			T900_OP_ADD:  alu_op_r = ALU_ADD;
			T900_OP_ADC:  alu_op_r = ALU_ADC;
			T900_OP_SUB:  alu_op_r = ALU_SUB;
			T900_OP_SBC:  alu_op_r = ALU_SBC;
			T900_OP_CP:   alu_op_r = ALU_CP;
			T900_OP_AND:  alu_op_r = ALU_AND;
			T900_OP_OR:   alu_op_r = ALU_OR;
			T900_OP_XOR:  alu_op_r = ALU_XOR;
			T900_OP_NEG:  alu_op_r = ALU_NEG;
			T900_OP_CPL:  alu_op_r = ALU_CPL;
			T900_OP_EXTZ: alu_op_r = ALU_EXTZ;
			T900_OP_EXTS: alu_op_r = ALU_EXTS;
			T900_OP_MIRR: alu_op_r = ALU_MIRR;
			T900_OP_DAA:  alu_op_r = ALU_DAA;
			T900_OP_INC:  begin alu_op_r = ALU_INC; alu_b_r = {28'd0, inc_amt}; end
			T900_OP_DEC:  begin alu_op_r = ALU_DEC; alu_b_r = {28'd0, inc_amt}; end
			default:      alu_op_r = ALU_PASS;
		endcase
	end

	// The micro-programs borrow the ALU: the string compares run a real CP so
	// S, Z and H come out of the same subtractor every other compare uses
	// (CPU900H p.65 "H = 1 is set if a borrow from bit 3 to bit 4 occurs"),
	// and MULA's accumulate runs a 32-bit ADD for its S, Z and V
	// (CPU900H p.110).
	wire str_alu = (st == S_STR);

	assign alu_op       = !str_alu ? alu_op_r : (is_mula ? ALU_ADD : ALU_CP);
	assign alu_size     = !str_alu ? dsz      : (is_mula ? T900_SZ_L : clssz_r);
	assign alu_a        = !str_alu ? dst_val  : (is_mula ? cls_reg : shortR);
	assign alu_b        = !str_alu ? alu_b_r  : (is_mula ? md_result : rd_now);
	assign alu_flags_in = sr[7:0];

	// BIT always decodes its full four-bit immediate, including for a byte
	// operand (CPU900H p.54). The mutating byte-bit and
	// carry-bit families wrap their index to three bits instead.
	wire [3:0]  bit_no   = ((cur_op == T900_OP_BIT) || (dsz != T900_SZ_B))
	                       ? src_val[3:0] : {1'b0, src_val[2:0]};
	wire [31:0] bit_mask = 32'd1 << bit_no;
	wire        bit_val  = dst_val[{1'b0, bit_no}];

	// BIT writes S with the sign bit of the masked operand: the tested bit
	// when the index is the operand's sign position (7 byte / 15 word), 0
	// otherwise. The datasheet marks S as "undefined value is set", which it
	// distinguishes from "no change"; V is marked the same way and is left
	// preserved.
	wire [3:0]  bit_msb_ix = (dsz == T900_SZ_B) ? 4'd7 : 4'd15;
	wire        bit_sign   = (bit_no == bit_msb_ix) ? bit_val : 1'b0;

	// Shift count: four bits with code 0 meaning 16 (CPU900H p.165 note 1).
	// The (mem) forms always shift once.
	wire [3:0] sh_code = (skk == T900_K_REG_A) ? reg_a[3:0] : src_val[3:0];
	wire [4:0] sh_norm = (sh_code == 4'd0) ? 5'd16 : {1'b0, sh_code};

	reg sh_start_r;
	reg md_start_r;

	assign sh_start    = sh_start_r;
	assign sh_op       = dec_b1[2:0];
	assign sh_size     = dsz;
	assign sh_count    = (cur_cls == CLS_SRC) ? 5'd1 : sh_norm;
	assign sh_data     = dst_val;
	assign sh_flags_in = sr[7:0];

	// MULA multiplies two signed 16-bit memory words, so it drives the unit as
	// a word MULS (CPU900H p.110).
	wire mula_run = (st == S_STR) && is_mula;

	assign md_start = md_start_r;
	assign md_op    = mula_run                 ? 2'd1 :
	                  (cur_op == T900_OP_MUL)  ? 2'd0 :
	                  (cur_op == T900_OP_MULS) ? 2'd1 :
	                  (cur_op == T900_OP_DIV)  ? 2'd2 : 2'd3;
	assign md_size  = mula_run ? 1'b1 : (cur_clssz != T900_SZ_B);
	assign md_a     = mula_run ? {16'd0, mula_lhs}      : dst_val;
	assign md_b     = mula_run ? {16'd0, rd_now[15:0]}  : src_val;

	// =====================================================================
	// Control registers reachable by LDC (CPU900H p.46)
	// =====================================================================

	wire [7:0] cr_code = (cur_k1 == T900_K_CR) ? o1[7:0] : o2[7:0];
	wire [1:0] cr_ch   = cr_code[3:2];

	reg [31:0] cr_rdata;
	always @(*) begin
		// The channel is cr_code[3:2], so the CONSTANT field of a DMAS or
		// DMAD code is its low two bits, not its high two: DMAS0-3 are
		// 00/04/08/0C and DMAD0-3 are 10/14/18/1C (CPU900H p.46;
		// TMP95C061 datasheet p.15). Matching 0000_00?? instead would reach
		// channel 0 only and alias the invalid codes 01/02/03 onto it,
		// leaving six of the eight address registers unreachable.
		casez (cr_code)
			8'h3C:        cr_rdata = {16'd0, intnest};
			8'b0000_??00: cr_rdata = dmas[cr_ch];
			8'b0001_??00: cr_rdata = dmad[cr_ch];
			8'b0010_??00: cr_rdata = {16'd0, dmac[cr_ch]};
			8'b0010_??10: cr_rdata = {24'd0, dmam[cr_ch]};
			default:      cr_rdata = 32'd0;
		endcase
	end

	// =====================================================================
	// Execute: a combinational description of the instruction's effects
	// =====================================================================

	reg        ex_reg_we;
	reg [7:0]  ex_reg_code;
	reg [1:0]  ex_reg_size;
	reg [31:0] ex_reg_data;
	reg        ex_mem_we;
	reg [23:0] ex_mem_addr;
	reg [2:0]  ex_mem_n;
	reg [31:0] ex_mem_data;
	reg        ex_f_we;
	reg [7:0]  ex_f;
	reg        ex_sr_we;
	reg [15:0] ex_sr;
	reg        ex_falt_we;
	reg [7:0]  ex_falt;
	reg        ex_branch;
	reg [23:0] ex_target;
	reg [5:0]  ex_extra;
	reg        ex_halt;
	reg        ex_reg2_we;
	reg [7:0]  ex_reg2_code;
	reg [1:0]  ex_reg2_size;
	reg [31:0] ex_reg2_data;
	reg        ex_multi;

	wire dst_is_mem  = (dk == T900_K_MEM) || (dk == T900_K_MEM8) || (dk == T900_K_MEM16);
	wire [23:0] dst_addr = (dk == T900_K_MEM) ? ea : o1[23:0];
	wire [7:0]  dst_code = (dk == T900_K_SHORT_R) ? shortR_code :
	                       (dk == T900_K_REG_A)   ? CODE_A : cls_reg_code;

	wire [23:0] disp8_t  = pc_now + sx8(o2[7:0]);
	wire [23:0] disp16_t = pc_now + sx16((cur_k1 == T900_K_D16) ? o1[15:0] : o2[15:0]);

	wire cc0_taken = cond_true(dec_b0[3:0], sr[7:0]);
	// cc1_taken is declared up with `ret_cc_false`, which needs it before the
	// stack read is decided.

	// INC/DEC register forms at word or long width change no flags
	// (CPU900H p.83, p.70 notes); the memory forms do.
	wire incdec_noflags = ((cur_op == T900_OP_INC) || (cur_op == T900_OP_DEC)) &&
	                      !dst_is_mem && (dsz != T900_SZ_B);

	// BS1F / BS1B: first set bit of a 16-bit register, counted from the LSB
	// or from the MSB, reported in A. When the register is zero V is set and
	// A is left alone (CPU900H p.57, p.58).
	function automatic [3:0] bs1f_idx(input [15:0] v);
		integer   j;
		reg [3:0] idx;
	begin
		idx = 4'd0;
		for (j = 15; j >= 0; j = j - 1)
			if (v[j[3:0]]) idx = j[3:0];
		bs1f_idx = idx;
	end
	endfunction

	function automatic [3:0] bs1b_idx(input [15:0] v);
		integer   j;
		reg [3:0] idx;
	begin
		idx = 4'd0;
		for (j = 0; j < 16; j = j + 1)
			if (v[j[3:0]]) idx = j[3:0];
		bs1b_idx = idx;
	end
	endfunction

	wire        bs_zero = (src_val[15:0] == 16'd0);
	wire [3:0]  bs_idx  = (cur_op == T900_OP_BS1F) ? bs1f_idx(src_val[15:0])
	                                               : bs1b_idx(src_val[15:0]);

	// RLD / RRD nibble rotate through A (CPU900H p.132, p.135). The worked
	// examples on those pages pin both: A=12 (100H)=34 gives A=13 (100H)=42
	// for RLD and A=14 (100H)=23 for RRD.
	wire [7:0] rld_a   = {reg_a[7:4], src_val[7:4]};
	wire [7:0] rld_mem = {src_val[3:0], reg_a[3:0]};
	wire [7:0] rrd_a   = {reg_a[7:4], src_val[3:0]};
	wire [7:0] rrd_mem = {reg_a[3:0], src_val[7:4]};
	wire [7:0] rxd_a   = (cur_op == T900_OP_RLD) ? rld_a   : rrd_a;
	wire [7:0] rxd_mem = (cur_op == T900_OP_RLD) ? rld_mem : rrd_mem;

	// MINC / MDEC: modulo step inside a power-of-two ring whose mask is the
	// imm16 operand (CPU900H p.104, p.101). No flags change.
	wire [15:0] mod_num  = src_val[15:0];
	wire [15:0] mod_val  = dst_val[15:0];
	// MINC wraps at the TOP of the ring, MDEC at the BOTTOM (CPU900H p.101
	// through p.106, operation lines and worked examples: MDEC1 8,IX takes
	// 1230H to 1237H and 1237H to 1236H).  The mask compare is measured on
	// hardware for misaligned pointers, refuting the doc's exact-modulo
	// reading: MINC2 8,WA=0x1237 -> 0x1231, MINC4 16,WA=0x123E -> 0x1232,
	// MDEC2 8,WA=0x1231 -> 0x1237 (doc modulo predicted 1239/1242/122F).
	wire        minc_hit = ((mod_val & mod_num) == mod_num);
	wire        mdec_hit = ((mod_val & mod_num) == 16'd0);
	wire [15:0] mod_step = ((cur_op == T900_OP_MINC1) || (cur_op == T900_OP_MDEC1))
	                       ? 16'd1 :
	                       ((cur_op == T900_OP_MINC2) || (cur_op == T900_OP_MDEC2))
	                       ? 16'd2 : 16'd4;
	wire [15:0] minc_res = minc_hit ? (mod_val - mod_num) : (mod_val + mod_step);
	wire [15:0] mdec_res = mdec_hit ? (mod_val + mod_num) : (mod_val - mod_step);

	always @(*) begin
		ex_reg_we    = 1'b0;
		ex_reg_code  = dst_code;
		ex_reg_size  = dsz;
		ex_reg_data  = alu_result;
		ex_mem_we    = 1'b0;
		ex_mem_addr  = dst_addr;
		ex_mem_n     = size_bytes(dsz);
		ex_mem_data  = alu_result;
		ex_f_we      = 1'b0;
		ex_f         = alu_flags_out;
		ex_sr_we     = 1'b0;
		ex_sr        = sr;
		ex_falt_we   = 1'b0;
		ex_falt      = f_alt;
		ex_branch    = 1'b0;
		ex_target    = pc_now;
		ex_extra     = 6'd0;
		ex_halt      = 1'b0;
		ex_reg2_we   = 1'b0;
		ex_reg2_code = cls_reg_code;
		ex_reg2_size = dsz;
		ex_reg2_data = 32'd0;
		ex_multi     = 1'b0;

		case (cur_op)
			// load group
			T900_OP_LD: begin
				if (dk == T900_K_SR) begin
					ex_sr_we = 1'b1;
					ex_sr    = src_val[15:0];
				end else if (dst_is_mem) begin
					ex_mem_we   = 1'b1;
					ex_mem_data = src_val;
				end else begin
					ex_reg_we   = 1'b1;
					ex_reg_data = src_val;
				end
			end

			T900_OP_LDA: begin
				ex_reg_we   = 1'b1;
				ex_reg_code = shortR_code;
				ex_reg_size = sz1;
				ex_reg_data = ea_full;
			end

			T900_OP_LDX: begin
				ex_mem_we   = 1'b1;
				ex_mem_addr = {16'd0, opv1[7:0]};
				ex_mem_n    = 3'd1;
				ex_mem_data = opv2;
			end

			T900_OP_PUSH: begin
				ex_reg_we   = 1'b1;
				ex_reg_code = CODE_SP;
				ex_reg_size = T900_SZ_L;
				ex_reg_data = sp_val - {29'd0, stack_n};
				ex_mem_we   = 1'b1;
				ex_mem_addr = sp_val[23:0] - {21'd0, stack_n};
				ex_mem_n    = stack_n;
				ex_mem_data = (cur_k1 == T900_K_SR)      ? {16'd0, sr} :
				              (cur_k1 == T900_K_FLAGS)   ? {24'd0, sr[7:0]} :
				              (cur_k1 == T900_K_REG_A)   ? {24'd0, reg_a} :
				              (cur_k1 == T900_K_IMM8)    ? o1 :
				              (cur_k1 == T900_K_IMM16)   ? o1 :
				              (cur_k1 == T900_K_MEM)     ? rd_now :
				              (cur_k1 == T900_K_SHORT_R) ? shortR : cls_reg;
			end

			T900_OP_POP: begin
				// XSP already advanced when the read was issued.
				if (cur_k1 == T900_K_SR) begin
					ex_sr_we = 1'b1;
					ex_sr    = rd_now[15:0];
				end else if (cur_k1 == T900_K_FLAGS) begin
					ex_f_we = 1'b1;
					ex_f    = rd_now[7:0];
				end else if (cur_k1 == T900_K_MEM) begin
					ex_mem_we   = 1'b1;
					ex_mem_addr = ea;
					ex_mem_n    = size_bytes(sz1);
					ex_mem_data = rd_now;
				end else begin
					ex_reg_we   = 1'b1;
					ex_reg_code = (cur_k1 == T900_K_REG_A)   ? CODE_A :
					              (cur_k1 == T900_K_SHORT_R) ? shortR_code : cls_reg_code;
					ex_reg_size = (cur_k1 == T900_K_REG_A) ? T900_SZ_B : sz1;
					ex_reg_data = rd_now;
				end
			end

			T900_OP_EX: begin
				if (cur_k1 == T900_K_FLAGS) begin
					ex_f_we    = 1'b1;
					ex_f       = f_alt;
					ex_falt_we = 1'b1;
					ex_falt    = sr[7:0];
				end else if (cur_k1 == T900_K_MEM) begin
					ex_mem_we   = 1'b1;
					ex_mem_addr = ea;
					ex_mem_n    = size_bytes(sz1);
					ex_mem_data = shortR;
					ex_reg_we   = 1'b1;
					ex_reg_code = shortR_code;
					ex_reg_size = sz2;
					ex_reg_data = rd_now;
				end else begin
					// EX R,r needs two register writes, so one is deferred.
					ex_reg_we    = 1'b1;
					ex_reg_code  = shortR_code;
					ex_reg_size  = sz1;
					ex_reg_data  = cls_reg;
					ex_reg2_we   = 1'b1;
					ex_reg2_code = cls_reg_code;
					ex_reg2_size = sz2;
					ex_reg2_data = shortR;
				end
			end

			// arithmetic and logic
			T900_OP_ADD, T900_OP_ADC, T900_OP_SUB, T900_OP_SBC,
			T900_OP_AND, T900_OP_OR,  T900_OP_XOR,
			T900_OP_INC, T900_OP_DEC, T900_OP_NEG, T900_OP_CPL,
			T900_OP_EXTZ, T900_OP_EXTS, T900_OP_MIRR, T900_OP_DAA: begin
				if (dst_is_mem) ex_mem_we = 1'b1;
				else            ex_reg_we = 1'b1;
				ex_f_we = !incdec_noflags &&
				          (cur_op != T900_OP_EXTZ) && (cur_op != T900_OP_EXTS) &&
				          (cur_op != T900_OP_MIRR);
			end

			// CP writes flags but not the result (CPU900H p.62).
			T900_OP_CP: ex_f_we = 1'b1;

			// PAA rounds an odd pointer up and changes no flags.
			T900_OP_PAA: begin
				ex_reg_we   = 1'b1;
				ex_reg_data = dst_val + {31'd0, dst_val[0]};
			end

			T900_OP_SCC: begin
				ex_reg_we   = 1'b1;
				ex_reg_data = {31'd0, cc1_taken};
			end

			T900_OP_MUL, T900_OP_MULS, T900_OP_DIV, T900_OP_DIVS: begin
				ex_multi    = 1'b1;
				ex_reg_we   = 1'b1;
				ex_reg_code = k1_is_R ? shortR_code : cls_reg_code;
				ex_reg_size = wide_sz;
				ex_reg_data = md_finish ? md_finish_result : md_result;
				// MUL/MULS change no flags; DIV/DIVS touch V only
				// (CPU900H p.73, p.108).
				ex_f_we     = (cur_op == T900_OP_DIV) || (cur_op == T900_OP_DIVS);
				ex_f        = {sr[7:3], md_v, sr[1:0]};
			end

			// bit group
			T900_OP_RCF: begin ex_f_we = 1'b1; ex_f = sr[7:0] & 8'hEC; end
			T900_OP_SCF: begin ex_f_we = 1'b1; ex_f = (sr[7:0] & 8'hEC) | 8'h01; end
			// CCF and ZCF clear H on silicon; the datasheet calls H
			// undefined there. SCF/RCF already clear H per the datasheet.
			T900_OP_CCF: begin ex_f_we = 1'b1; ex_f = (sr[7:0] & 8'hED) ^ 8'h01; end
			T900_OP_ZCF: begin
				ex_f_we = 1'b1;
				ex_f    = (sr[7:0] & 8'hEC) | {7'd0, ~sr[6]};
			end

			T900_OP_BIT: begin
				// S takes the masked operand's sign bit; see bit_sign above.
				ex_f_we = 1'b1;
				ex_f    = {bit_sign, ~bit_val, 1'b0, 1'b1, 1'b0, sr[2], 1'b0, sr[0]};
			end

			T900_OP_TSET: begin
				// Z takes the complement of the bit, then the bit is set;
				// H forced 1, N forced 0.
				ex_f_we     = 1'b1;
				ex_f        = {sr[7], ~bit_val, 1'b0, 1'b1, 1'b0, sr[2], 1'b0, sr[0]};
				ex_reg_data = dst_val | bit_mask;
				ex_mem_data = dst_val | bit_mask;
				if (dst_is_mem) ex_mem_we = 1'b1;
				else            ex_reg_we = 1'b1;
			end

			T900_OP_RES, T900_OP_SET, T900_OP_CHG: begin
				ex_reg_data = (cur_op == T900_OP_RES) ? (dst_val & ~bit_mask) :
				              (cur_op == T900_OP_SET) ? (dst_val |  bit_mask) :
				                                        (dst_val ^  bit_mask);
				ex_mem_data = ex_reg_data;
				if (dst_is_mem) ex_mem_we = 1'b1;
				else            ex_reg_we = 1'b1;
			end

			T900_OP_ANDCF, T900_OP_ORCF, T900_OP_XORCF, T900_OP_LDCF: begin
				ex_f_we = 1'b1;
				ex_f    = {sr[7:1],
				           (cur_op == T900_OP_ANDCF) ? (sr[0] & bit_val) :
				           (cur_op == T900_OP_ORCF)  ? (sr[0] | bit_val) :
				           (cur_op == T900_OP_XORCF) ? (sr[0] ^ bit_val) : bit_val};
			end

			T900_OP_STCF: begin
				// STCF with an out-of-range bit index on a byte operand
				// leaves the operand unchanged (CPU900H p.145), but still
				// performs the store cycle with the unchanged value. The
				// store is unconditional here and only the DATA reverts.
				ex_reg_data = ((dsz == T900_SZ_B) && src_val[3])
				              ? dst_val
				              : (sr[0] ? (dst_val | bit_mask)
				                       : (dst_val & ~bit_mask));
				ex_mem_data = ex_reg_data;
				if (dst_is_mem) ex_mem_we = 1'b1;
				else            ex_reg_we = 1'b1;
			end

			// shifts
			T900_OP_RLC, T900_OP_RRC, T900_OP_RL, T900_OP_RR,
			T900_OP_SLA, T900_OP_SRA, T900_OP_SLL, T900_OP_SRL: begin
				ex_multi    = 1'b1;
				ex_reg_data = sh_result;
				ex_mem_data = sh_result;
				if (dst_is_mem) ex_mem_we = 1'b1;
				else            ex_reg_we = 1'b1;
				ex_f_we = 1'b1;
				ex_f    = sh_flags_out;
			end

			// control transfer
			T900_OP_JR, T900_OP_JRL: begin
				if (cc0_taken) begin
					ex_branch = 1'b1;
					ex_target = (cur_op == T900_OP_JR) ? disp8_t : disp16_t;
					ex_extra  = ADD_JP_TRUE;
				end
			end

			T900_OP_JP: begin
				if (cur_k1 == T900_K_CC) begin
					if (cc1_taken) begin
						ex_branch = 1'b1;
						ex_target = ea;
						ex_extra  = ADD_JP_TRUE;
					end
				end else begin
					ex_branch = 1'b1;
					ex_target = o1[23:0];
				end
			end

			T900_OP_CALL: begin
				if ((cur_k1 != T900_K_CC) || cc1_taken) begin
					ex_branch   = 1'b1;
					ex_target   = (cur_k1 == T900_K_CC) ? ea : o1[23:0];
					ex_extra    = (cur_k1 == T900_K_CC) ? ADD_CALL_TRUE : 6'd0;
					ex_reg_we   = 1'b1;
					ex_reg_code = CODE_SP;
					ex_reg_size = T900_SZ_L;
					ex_reg_data = sp_val - 32'd4;
					ex_mem_we   = 1'b1;
					ex_mem_addr = sp_val[23:0] - 24'd4;
					ex_mem_n    = 3'd4;
					ex_mem_data = {8'd0, pc_now};
				end
			end

			T900_OP_CALR: begin
				ex_branch   = 1'b1;
				ex_target   = disp16_t;
				ex_reg_we   = 1'b1;
				ex_reg_code = CODE_SP;
				ex_reg_size = T900_SZ_L;
				ex_reg_data = sp_val - 32'd4;
				ex_mem_we   = 1'b1;
				ex_mem_addr = sp_val[23:0] - 24'd4;
				ex_mem_n    = 3'd4;
				ex_mem_data = {8'd0, pc_now};
			end

			T900_OP_RET: begin
				if ((cur_k1 != T900_K_CC) || cc1_taken) begin
					ex_branch = 1'b1;
					ex_target = rd_now[23:0];
					ex_extra  = (cur_k1 == T900_K_CC) ? ADD_CALL_TRUE : 6'd0;
				end
			end

			T900_OP_RETD: begin
				ex_branch   = 1'b1;
				ex_target   = rd_now[23:0];
				ex_reg_we   = 1'b1;
				ex_reg_code = CODE_SP;
				ex_reg_size = T900_SZ_L;
				// 32-bit signed add; see sx16_32.
				ex_reg_data = sp_val + 32'd4 + sx16_32(o1[15:0]);
			end

			T900_OP_DJNZ: begin
				ex_reg_we   = 1'b1;
				ex_reg_code = cls_reg_code;
				ex_reg_size = sz1;
				ex_reg_data = dst_val - 32'd1;
				if (((sz1 == T900_SZ_B) && (dst_val[7:0]  != 8'd1)) ||
				    ((sz1 != T900_SZ_B) && (dst_val[15:0] != 16'd1))) begin
					ex_branch = 1'b1;
					ex_target = disp8_t;
					ex_extra  = ADD_DJNZ_TRUE;
				end
			end

			// system
			T900_OP_HALT: ex_halt = 1'b1;

			// EI num; num = 7 is DI (CPU900H p.164).
			T900_OP_EI: begin
				ex_sr_we = 1'b1;
				ex_sr    = {sr[15], o1[2:0], sr[11:0]};
			end

			T900_OP_LDF: begin
				ex_sr_we = 1'b1;
				ex_sr    = {sr[15:11], 1'b0, o1[1:0], sr[7:0]};
			end

			T900_OP_INCF: begin
				ex_sr_we = 1'b1;
				ex_sr    = {sr[15:11], 1'b0, sr[9:8] + 2'd1, sr[7:0]};
			end

			T900_OP_DECF: begin
				ex_sr_we = 1'b1;
				ex_sr    = {sr[15:11], 1'b0, sr[9:8] - 2'd1, sr[7:0]};
			end

			T900_OP_SWI: ex_multi = 1'b1;   // runs as the S_EXEC micro-program

			T900_OP_LDC: begin
				// LDC r,cr moves into the register file; LDC cr,r is handled
				// by the sequential block's control-register write.
				if (cls_reg_is_op1) begin
					ex_reg_we   = 1'b1;
					ex_reg_code = cls_reg_code;
					ex_reg_size = sz1;
					ex_reg_data = cr_rdata;
				end
			end

			// stack frames
			T900_OP_LINK: begin
				// XSP -= 4; (XSP) <- r; r <- XSP; XSP += d16 (CPU900H p.99).
				// QUIRK: `LINK XIY,N` with N >= 5 is broken on real NGPC
				// silicon. No replacement behavior is documented, so the
				// documented behavior is what is implemented, and code that
				// trips the bug will not match hardware.
				ex_mem_we    = 1'b1;
				ex_mem_addr  = sp_val[23:0] - 24'd4;
				ex_mem_n     = 3'd4;
				ex_mem_data  = cls_reg;
				ex_reg_we    = 1'b1;
				ex_reg_code  = cls_reg_code;
				ex_reg_size  = T900_SZ_L;
				ex_reg_data  = sp_val - 32'd4;
				ex_reg2_we   = 1'b1;
				ex_reg2_code = CODE_SP;
				ex_reg2_size = T900_SZ_L;
				ex_reg2_data = (sp_val - 32'd4) + {{16{o2[15]}}, o2[15:0]};
			end

			T900_OP_UNLK: begin
				// XSP <- r; r <- (XSP); XSP += 4 (CPU900H p.151). XSP is
				// given its final value r+4 on the state the read is issued
				// (see `unlk_sp_we`), which leaves one register write for the
				// commit state and fits the instruction in its 7 states.
				ex_reg_we   = 1'b1;
				ex_reg_code = cls_reg_code;
				ex_reg_size = T900_SZ_L;
				ex_reg_data = rd_now;
			end

			// modulo step and search
			T900_OP_MINC1, T900_OP_MINC2, T900_OP_MINC4: begin
				ex_reg_we   = 1'b1;
				ex_reg_data = {16'd0, minc_res};
			end

			T900_OP_MDEC1, T900_OP_MDEC2, T900_OP_MDEC4: begin
				ex_reg_we   = 1'b1;
				ex_reg_data = {16'd0, mdec_res};
			end

			T900_OP_BS1F, T900_OP_BS1B: begin
				ex_reg_we   = !bs_zero;
				ex_reg_code = CODE_A;
				ex_reg_size = T900_SZ_B;
				ex_reg_data = {28'd0, bs_idx};
				ex_f_we     = 1'b1;
				ex_f        = {sr[7:3], bs_zero, sr[1:0]};
			end

			// nibble rotates
			T900_OP_RLD, T900_OP_RRD: begin
				ex_reg_we   = 1'b1;
				ex_reg_code = CODE_A;
				ex_reg_size = T900_SZ_B;
				ex_reg_data = {24'd0, rxd_a};
				ex_mem_we   = 1'b1;
				ex_mem_addr = ea;
				ex_mem_n    = 3'd1;
				ex_mem_data = {24'd0, rxd_mem};
				// S, Z, H=0, V=parity(A) even, N=0; C is unchanged
				// (CPU900H p.132, p.135).
				ex_f_we     = 1'b1;
				ex_f        = {rxd_a[7], (rxd_a == 8'd0), 1'b0, 1'b0, 1'b0,
				               ~^rxd_a, 1'b0, sr[0]};
			end

			// RETI, MULA and the string group are micro-programs; see S_INT
			// and S_STR.
			default: ;
		endcase
	end

	// =====================================================================
	// Readiness, commit and retirement
	// =====================================================================

	// Every stream byte this instruction needs has been popped.
	wire bytes_done =
		(st == S_FETCH0) ? (q_pop && !dec_needs_byte1 && (opf_total == 3'd0)) :
		(st == S_FETCH1) ? (q_pop && (opf_total == 3'd0)) :
		(st == S_OPF)    ? (q_pop && ((opf_cnt + 3'd1) == opf_total_held)) :
		((st == S_RD) || (st == S_EXEC));

	// RETI, MULA and the string / repeat group are micro-programs: they must
	// never commit through the single-state execute description.
	wire is_micro_op = (cur_gp == T900_G_STRING) || is_mula ||
	                   (cur_op == T900_OP_RETI);

	wire operands_ready = in_insn && op_avail && !cur_ud && !insn_done &&
	                      !is_micro_op && bytes_done && rd_ok;

	// A shift's final four-position slice and its architectural writeback share
	// one Toshiba state. `sh_finish` exposes that registered-work slice before
	// the edge; `sh_done` remains the post-edge pulse used by standalone unit
	// tests. The explicit budget below prevents the A,r form from retiring a
	// state early while allowing the longer #4,r encoding to lose its old
	// one-state overrun.
	wire unit_done  = shift_op ? (sh_done || sh_finish) :
	                  (is_muldiv ? (md_done || md_finish) : 1'b0);
	// The immediate-source word MUL/DIV extra cost is modelled as an
	// operand-arrival stall before the unit starts -- the immediate
	// operand's last byte arrives through the queue -- which charges
	// exec-bound and budget-bound shapes alike while leaving every
	// register-source form untouched.  `muldiv_budget_add` below grows the
	// budget by the same amount, so budget-bound shapes retire at table+N
	// exactly.  The SIGNED immediate forms carry the same class of charge
	// with the magnitudes reversed: MULS rr,# word +2 and DIVS rr,# word +4
	// (measured on hardware).
	wire md_imm_charged = is_muldiv && !mula_run && (cur_clssz != T900_SZ_B) &&
	                      (cur_k2 == T900_K_IMM);

	// A register-source word MUL/DIV is charged for the bus traffic of the
	// instruction IN FRONT OF IT, less the one transaction every instruction
	// performs anyway (measured on hardware: the charge follows the
	// predecessor, not the operation):
	//
	//     stall = ceil(prev_bytes / 2) + prev_data_access - 1
	//
	// The variable is TRANSACTIONS rather than bytes: `ld WA,(XHL)` is two
	// bytes long like the zero-stall group, but it takes a data cycle, and
	// the hardware charges it like the three-byte group.
	reg  [3:0] insn_bytes;    // bytes this instruction took from the queue
	reg        insn_mem;      // this instruction performed a data access
	reg  [2:0] prev_txn;      // bus transactions of the PREVIOUS instruction

	// The instruction's own totals INCLUDING this cycle. `prev_txn` is latched
	// on the retiring cycle, and an instruction whose last operand byte is
	// popped on that very cycle would otherwise be counted one byte short --
	// the register clear below wins over the increment.
	wire [3:0] insn_bytes_now = insn_bytes +
	                            ((q_pop && !insn_bytes[3]) ? 4'd1 : 4'd0);
	wire       insn_mem_now   = insn_mem || iss_rd || iss_wr;

	// ceil(bytes/2) + data-access, computed wide and truncated EXPLICITLY with
	// a saturating cap so no instruction can ask for an unbounded stall.
	wire [3:0] prev_txn_raw = {1'b0, insn_bytes_now[3:1]} +
	                          {3'b0, insn_bytes_now[0]} + {3'b0, insn_mem_now};
	wire [2:0] prev_txn_now = (prev_txn_raw > 4'd4) ? 3'd4 : prev_txn_raw[2:0];
	wire [2:0] md_boundary  = (prev_txn > 3'd1) ? (prev_txn - 3'd1) : 3'd0;

	wire md_q_charged = is_muldiv && !mula_run && (cur_clssz != T900_SZ_B) &&
	                    (cur_k2 != T900_K_IMM);

	reg  [2:0] md_stall;      // imm form counts DOWN, boundary form counts UP
	reg        md_stall_done;
	wire md_q_ready = (md_stall >= md_boundary);

	wire md_want_start = operands_ready && ex_multi && !unit_started &&
	                     (cur_op != T900_OP_SWI);
	wire unit_start = md_want_start &&
	                  (md_imm_charged ? md_stall_done :
	                   md_q_charged   ? md_q_ready    : 1'b1);

	// The immediate-count shift with n=1..3 completes its remainder slice on
	// the start tick. Only that form consumes the combinational completion;
	// A,r keeps its registered-unit boundary and is held to the same table
	// budget below. Thus the new path is opcode-scoped to the case that needs
	// the overlap and is at most the shifter's existing three-position load
	// chain, not its four-position iteration chain.
	wire shift_start_done = shift_op && unit_start && sh_finish &&
	                        (cur_k1 == T900_K_IMM8);

	wire do_exec = operands_ready &&
	               (!ex_multi || (unit_started && unit_done) || shift_start_done);

	// Micro-program readiness

	// Data from the read the current micro-program step is waiting for.
	wire mp_rd_ok = rd_ret || rd_have;

	wire str_run  = (st == S_STR) && !is_mula;

	// BC after this iteration's decrement, and the repeat decision. CPIR/CPDR
	// also stop on a match (CPU900H p.66).
	wire        str_bc_nz  = (shortR[15:0] != 16'd1);
	wire        str_more   = str_repeat && str_bc_nz &&
	                         !(str_is_cp && mp_szh[1]);
	// Toshiba samples interrupts between iterations (CPU900H p.97).
	//
	// THIS TAKES `irq_want`, NOT `irq_ok` -- the shadow is deliberately not
	// applied here, and removing that distinction is a real bug with a name.
	// A repeat ends an iteration as an instruction only when `str_irq` is
	// true (`str_loop` -> `str_fin` -> `retire_now`), and `irq_shadow` is
	// cleared only BY a retirement.  Feed the shadowed request in here and
	// the two deadlock: a RETI that returns into the middle of an LDIR arms
	// the shadow, the shadow suppresses the break-out, and the break-out is
	// the only thing that would clear it -- so the whole remaining repeat
	// runs atomically, however long BC is.  The "one instruction" of shadow
	// silently becomes "one whole string operation".
	//
	// Measured cost when it was wired that way (2026-08-21 .. 2026-08-23):
	// Densha de Go! 2's per-line background-colour gradient lost 17 of its
	// 152 Hint services on every fourth frame -- one repeat ran 714 CPU
	// states, longer than the 515-dot raster line -- and the sky visibly
	// flickered on hardware.  agents/plans/
	// PLAN_densha_sky_flicker_reti_string_shadow_20260823.md has the bisect
	// and the trace.
	//
	// Taking the unshadowed request costs the interrupted program exactly
	// ONE ITERATION before the request is accepted (break out, retire, the
	// retirement clears the shadow, accept at the next boundary).  That is
	// the repeat's analogue of the one instruction the shadow exists to
	// guarantee, and it is what CPU900H p.97 describes: a repeat's interrupt
	// sampling points ARE its iteration boundaries.
	wire        str_irq    = (STRING_INTERRUPTIBLE != 0) && (irq_want || dma_ok);
	wire        str_loop   = str_more && !str_irq;
	wire        str_at_cnt = str_run && (mstep == 3'd2);

	// Each extra iteration costs `str_iter` states, and the table entry already
	// carries the first one, which is what makes 7n+1 and 6n+1 come out.
	//
	// The cadence is enforced one iteration at a time (see `str_hold`) rather
	// than as a total settled at the end; that keeps `budget` inside 8 bits
	// for a repeat of any length.
	wire [5:0]  str_iter = str_is_cp ? STR_ITER_CP : STR_ITER_LD;

	// The states on which a micro-program commits its last architectural
	// effect, so retirement may happen on that very state.
	wire int_fin  = (st == S_INT) && ddone &&
	                ((ev_kind == EV_RETI) ? (mstep == 3'd3) : (mstep == 3'd5));
	wire str_fin  = str_at_cnt && !str_loop;
	wire mula_fin = (st == S_STR) && is_mula && (mstep == 3'd4) && md_done;
	wire mp_fin   = int_fin || str_fin || mula_fin;

	// The budget the current tick must be measured against, including a value
	// being loaded, a taken-branch adder and a bus penalty landing right now.
	wire load_budget = ((st == S_FETCH0) && q_pop && !dec_needs_byte1) ||
	                   ((st == S_FETCH1) && q_pop);

	// An undefined opcode traps as SWI 2 (TMP95C061 datasheet p.12), so it is
	// charged what SWI is charged; Appendix B has no row for INTUNDEF.
	// RETI measures 11 states on silicon against the published 12,
	// independently of its successor; a physical figure overrides the
	// documented one.
	wire [7:0] reti_phys = ((st == S_FETCH0) && (dec_op_id == T900_OP_RETI))
	                       ? 8'd1 : 8'd0;

	wire [7:0] tbl_total = dec_undef ? UNDEF_STATES :
	                       ({2'd0, dec_states} +
	                        ((st == S_FETCH1) ? {2'd0, ea_adder} : 8'd0) +
	                        {2'd0, pen_carry} - reti_phys);

	// The table stores the fixed three-state shift cost. Toshiba adds n/4,
	// truncating (CPU900H p.165). Add it once, on the unit-start tick, from
	// the normalized 1..16 count. Memory shifts live in CLS_SRC, always move
	// once, and carry their complete six-state cost in the table already.
	wire [7:0] shift_budget_add = (unit_start && shift_op && (cur_cls == CLS_REG))
	                              ? {5'd0, sh_norm[4:2]} : 8'd0;

	// The IMMEDIATE-source word MUL is 4 states dearer and the immediate
	// word DIV 2 states dearer on silicon than their Appendix-B rows
	// (measured on hardware), and the REGISTER forms pay NOTHING.  So the
	// charge keys on the source KIND, not the op: only `rr,#` (imm) forms
	// carry it.  The (mem) source forms, MULS/DIVS, MULA and the byte
	// class are unmeasured and stay at table.
	wire [7:0] muldiv_budget_add =
	        (unit_start && md_imm_charged)
	            ? (((cur_op == T900_OP_MUL) ||
	                (cur_op == T900_OP_DIVS)) ? 8'd4 : 8'd2) :
	        (unit_start && md_q_charged) ? {5'd0, md_stall} : 8'd0;

	// A repeat iteration does not grow its frame for the K2GE's drawing-period
	// hold, and it DOES grow it for slow memory.  This term is where the two
	// are separated: an arbitration state is free to a repeat form, a slow
	// MEMORY state is not.  `d_wait_gfx` is the fabric telling us which states
	// were arbitration (see the note beside `bus_wait_gfx` in
	// k2_soc_fabric.sv); everything else in `mem_ticks` -- a CSC cart wait, an
	// 8-bit region's second bus cycle -- is the memory being slow.  Do not
	// collapse this term to either plain `mem_over` or `mem_over && !str_run`.
	//
	// The cadence is still a FLOOR, not a base to add to: the micro-program
	// waits on `ddone` for both accesses, so real bus time is already inside
	// `used` and the iteration lasts max(cadence, actual).
	//
	// Outside a repeat form this is exactly `mem_over` -- an ordinary store is
	// POSTED, its instruction does not wait for it, and the budget is the only
	// thing that can carry the extra bus time. MULA shares S_STR and is
	// excluded from `str_run`, so it is charged like any other instruction.
	wire mem_over_charge = str_run ? mem_over_arb : mem_over;

	wire [7:0] budget_now = (load_budget ? tbl_total : budget) +
	                        (mem_over_charge ? 8'd1 : 8'd0) +
	                        shift_budget_add + muldiv_budget_add +
	                        (do_exec ? {2'd0, ex_extra} : 8'd0);

	// This state is the last one the current budget frame requires.
	wire budget_met = ((used + 8'd1) >= budget_now);

	// Per-iteration cadence floor for the repeat forms.  LDIR/LDDR are 7n+1
	// states and CPIR/CPDR are 6n+1 (CPU900H p.159), and that is a
	// per-ITERATION cadence, not a total the instruction may settle at the end.
	// Toshiba's own figure spends only part of each iteration on the bus; the
	// rest is idle, which is exactly the slack a slow region's wait states
	// disappear into -- which is why a character-RAM write penalty is charged
	// and LDIR still comes out at 7n+1.
	//
	// The decision state, mstep 2, is the iteration boundary. Hold it there
	// until this iteration has occupied its frame, then advance and reload the
	// frame for the next one (see the `str_iter_adv` arm in the budget block).
	// The first iteration keeps the table's own entry, so the totals come out
	// 8 + 7(n-1) and 7 + 6(n-1) with no separate first-iteration case.
	//
	// Only the LOOPING arm is held. The final iteration exits through S_NEXT,
	// where ordinary retirement already applies the same test, so gating it
	// here would be the same wait written twice.
	wire str_hold     = str_at_cnt && str_loop && !budget_met;
	wire str_iter_adv = str_at_cnt && str_loop && budget_met;

	// Micro-program bus channel
	// S_INT, S_DMA and S_STR all reach the bus through this one request, so
	// the arbiter below still sees a single micro-program client.

	reg        mp_want;
	reg        mp_we;
	reg [23:0] mp_addr;
	reg [2:0]  mp_n;
	reg [31:0] mp_wd;

	always @(*) begin
		mp_want = 1'b0;
		mp_we   = 1'b0;
		mp_addr = 24'd0;
		mp_n    = 3'd1;
		mp_wd   = 32'd0;

		if (st == S_INT) begin
			if (ev_kind == EV_RETI) begin
				// POP SR (2), POP PC (4) (CPU900H p.127; TMP95C061 datasheet
				// p.10).
				//
				// The second access is presented IN the state the first one's
				// ddone pulses in -- MULA's (XHL) read below already leans on
				// exactly this BIU property.  Waiting for the next mstep
				// instead leaves the bus free for one state, and the prefetch
				// lookahead takes it: a fall-through FILL lands between the
				// two pops and defers the PC pop by a whole cycle.  The
				// mstep-2/4 re-present arms remain as fallback for a bus that
				// could not take the early presentation.
				case (mstep)
					3'd0: begin
						mp_want = 1'b1;
						mp_addr = rf_ra_data[23:0];
						mp_n    = 3'd2;
					end
					3'd1: begin
						// Presented at LEVEL, not gated on ddone: the BIU
						// accepts only at ddone anyway (mem_free), but the
						// level presentation is what lets data_next_hold
						// cover the SR pop's T2 -- otherwise a prefetch
						// fill of the DOOMED stream chains there (the
						// entry/exit flush discards it) and the service
						// pays the fill's two states.
						mp_want = 1'b1;
						mp_addr = mp_base + 24'd2;
						mp_n    = 3'd4;
					end
					3'd2: begin
						mp_want = 1'b1;
						mp_addr = mp_base + 24'd2;
						mp_n    = 3'd4;
					end
					default: ;
				endcase
			end else begin
				// PUSH PC (4), PUSH SR (2), read the vector (4).  Same
				// ddone-state presentation as the RETI arm; SP moved once at
				// mstep 0 (see `mpr_we`), so the SR push's address is stable
				// long before the PC push completes, and the vector address
				// never depended on the stack at all.
				case (mstep)
					3'd0: begin
						mp_want = 1'b1;
						mp_we   = 1'b1;
						mp_addr = rf_ra_data[23:0] - 24'd4;
						mp_n    = 3'd4;
						mp_wd   = {8'd0, pc};
					end
					3'd1: begin
						// Level presentation; see the RETI arm's comment.
						mp_want = 1'b1;
						mp_we   = 1'b1;
						mp_addr = rf_ra_data[23:0];
						mp_n    = 3'd2;
						mp_wd   = {16'd0, sr};
					end
					3'd2: begin
						mp_want = 1'b1;
						mp_we   = 1'b1;
						mp_addr = rf_ra_data[23:0];
						mp_n    = 3'd2;
						mp_wd   = {16'd0, sr};
					end
					3'd3: begin
						// Level presentation; see the RETI arm's comment.
						mp_want = 1'b1;
						mp_addr = VECTOR_BASE + {16'd0, ev_vec};
						mp_n    = 3'd4;
					end
					3'd4: begin
						mp_want = 1'b1;
						mp_addr = VECTOR_BASE + {16'd0, ev_vec};
						mp_n    = 3'd4;
					end
					default: ;
				endcase
			end
		end else if (st == S_DMA) begin
			case (mstep)
				3'd0: begin
					mp_want = 1'b1;
					mp_addr = dmas[dma_ch][23:0];
					mp_n    = dma_nb;
				end
				3'd2: begin
					mp_want = 1'b1;
					mp_we   = 1'b1;
					mp_addr = dmad[dma_ch][23:0];
					mp_n    = dma_nb;
					mp_wd   = mp_data;
				end
				default: ;
			endcase
		end else if (mula_pre) begin
			mp_want = 1'b1;
			mp_addr = rf_ra_data[23:0];      // (XDE)
			mp_n    = 3'd2;
		end else if (st == S_STR) begin
			if (is_mula) begin
				// The (XHL) read is presented in the state (XDE)'s data
				// arrives, not the state after it. Both because that is what
				// silicon does, and because the state after is one the
				// prefetcher would take: the data port's request is down
				// there, so `want_fill` is free to start a fill that would
				// own the bus for the two states this read wants.
				if ((mstep == 3'd1) && mp_rd_ok) begin
					mp_want = 1'b1;
					mp_addr = rf_ra_data[23:0];  // (XHL)
					mp_n    = 3'd2;
				end
			end else begin
				case (mstep)
					3'd1: begin                  // store to the destination
						mp_want = 1'b1;
						mp_we   = 1'b1;
						mp_addr = rf_ra_data[23:0];
						mp_n    = str_nb;
						mp_wd   = mp_data;
					end
					3'd3: begin                  // next iteration's source read
						mp_want = 1'b1;
						mp_addr = rf_ra_data[23:0];
						mp_n    = str_nb;
					end
					default: ;
				endcase
			end
		end
	end

	// Bus access arbitration
	// One access at a time, in priority order: the reset vector fetch, the
	// running micro-program, the instruction's operand read, then its store.

	wire store_now = (do_exec && ex_mem_we) || wr_pend;

	wire rst_rd_want = (st == S_RESET) && !rd_started;
	wire op_rd_want  = in_insn && rd_want && !rd_started && !rd_have;

	wire iss_rst = mem_free && rst_rd_want;
	wire iss_mp  = mem_free && !rst_rd_want && mp_want;
	wire iss_rd  = mem_free && !rst_rd_want && !mp_want && op_rd_want;
	wire iss_wr  = mem_free && !rst_rd_want && !mp_want && !op_rd_want && store_now;

	assign iss_req = iss_rst || iss_mp || iss_rd || iss_wr;
	assign iss_we  = iss_wr || (iss_mp && mp_we);

	assign iss_addr = iss_rst ? VECTOR_BASE :
	                  iss_mp  ? mp_addr :
	                  iss_rd  ? rd_addr :
	                            (wr_pend ? wr_addr : ex_mem_addr);
	assign iss_n    = iss_rst ? 3'd3 :
	                  iss_mp  ? mp_n :
	                  iss_rd  ? rd_n :
	                            (wr_pend ? wr_n : ex_mem_n);
	assign iss_wd   = iss_mp ? mp_wd : (wr_pend ? wr_data : ex_mem_data);

	// A store already on the bus never holds the instruction up -- it is
	// posted. If the region is slow, `mem_over` grows the budget state by
	// state instead, which defers retirement by exactly the extra bus time.
	wire store_left = store_now && !iss_wr;

	// XSP advances when the stack read is issued, so the destination write and
	// the pointer update never contend for the single write port.
	//
	// A conditional RET whose condition is FALSE must leave XSP alone. Without
	// this, every `RET cc` that falls through leaks four bytes of stack.
	// `stack_rd` above now also suppresses the READ, which is what brings the
	// not-taken cost back to the table's four states; `sp_pop_upd` keeps the
	// pointer guard anyway so neither term alone can leak.
	wire sp_pop_upd = iss_rd && stack_rd && (cur_op != T900_OP_RETD) &&
	                  !ret_cc_false;
	wire unlk_sp_we = iss_rd && unlk_rd;

	// `insn_done` alone is enough once set: if a second register write was
	// deferred (EX R,r) it happens in this very state.
	wire work_ok = (do_exec && !ex_reg2_we) || mp_fin || insn_done;

	wire retire_now = in_insn && work_ok && !store_left && budget_met;

	// Let the final stream byte of a taken transfer finish, but do not chain a
	// byte from the path that is about to be discarded. This is chain-only: an
	// idle BIU may still fetch a missing operand byte, so a short queue cannot
	// deadlock the branch. For JR the final byte is its displacement in
	// S_FETCH1; immediate CALL/JP/CALR forms finish in S_OPF.
	wire branch_stream_last = ((st == S_FETCH1) && (opf_total == 3'd0)) ||
	                          ((st == S_OPF) &&
	                           ((opf_cnt + 3'd1) == opf_total_held));
	wire branch_prefetch_hold = in_insn && op_avail && !insn_done &&
	                            ex_branch && branch_stream_last;

	// Back-to-back data accesses: hold the prefetcher off the handover state.
	// True while the data port holds an access AND the sequencer already has the
	// next request lined up behind it, so that request is presented in the state
	// `ddone` pulses in.  That state is the one the BIU would otherwise hand to
	// a prefetch fill chaining out of the access's final T2
	// (`fill_chain_after_data`), and a fill started there owns the bus for the
	// two states the second access wanted: 4 states from its issue state instead
	// of 3.
	//
	// The BIU cannot see this one state early on its own, which is why the
	// signal lives here.  In that final T2 `dreq` is high for the access that is
	// FINISHING (`dreq = (mem_pend && !ddone) || iss_req`, with `ddone` still
	// low there), so gating the chain on `!dreq` would block it always, and
	// `iss_req` for the successor does not rise until the state after.  This is
	// the same one-state lookahead contract `source_prefetch_hold` already uses,
	// so it folds into the same output rather than opening a parallel channel.
	// Folding is free: `prefetch_hold`'s two other consumers in the BIU are
	// `want_fill`, already gated on `!dreq`, and `fill_chain`, which only runs
	// in a FILL's T2 where `start_data` already outranks it.
	//
	// The mechanism exists because a fill chains out of a data cycle's last T2
	// at all -- a byte write costs 2.02 states, so the dead state after every
	// access is real fetch bandwidth -- while Toshiba's LDIR is 7n+1
	// (CPU900H p.159), which leaves no room for that fill to take the state the
	// second access of the pair needs.  So the chain must stand down exactly
	// when a successor is already determined, and only then: over-asserting
	// loses a fill that should have happened.
	//
	// The terms are the bus arbiter's own request sources, not opcodes:
	//  - `mp_want` covers the micro-program pairs whose successor is presented
	//    in the ddone state.  In `S_STR` that is mstep 1 (the LD forms' store,
	//    unconditional there, so it already reads 1 in the source read's final
	//    T2) and mstep 3 (the next iteration's source read: mstep 2 advances
	//    unconditionally while the store is still in T1, so mstep is already 3
	//    in the store's final T2).
	//  - `wr_pend`, `op_rd_want` and `rst_rd_want` cover anything else already
	//    queued behind the running access -- a posted store, or an operand read
	//    waiting behind one.
	//  - `mula_rd_next` covers the one step `mp_want` cannot show early; see the
	//    note on it below.
	//
	// `S_INT` and `S_DMA` deliberately have no term.  Their odd msteps (1, 3)
	// wait on `ddone` and the next push/read is presented in the state AFTER it,
	// where a chained fill is free rather than costly: its T1 lands in the dead
	// issue state the successor would have spent in `BC_IDLE` anyway.  A
	// read-modify-write instruction presents its store in the state its read
	// returns, and silicon gives that state to the prefetcher, so it is not held
	// either.

	// MULA is the one micro-program step whose `mp_want` cannot be read one
	// state early: its (XHL) read is presented on `mp_rd_ok`, which is the
	// (XDE) read's own `rd_ret`, so `mp_want` is still low in the T2 the BIU
	// decides in. State plus step index states the same fact structurally --
	// entering `S_STR` at mstep 1 with `is_mula` IS "the (XDE) read is on the
	// bus and the (XHL) read follows it".
	wire mula_rd_next = (st == S_STR) && is_mula && (mstep == 3'd1);

	wire data_next_hold = mem_pend && !ddone &&
	                      (rst_rd_want || mp_want || op_rd_want || wr_pend ||
	                       mula_rd_next);

	assign prefetch_hold       = source_prefetch_hold || data_next_hold;
	assign prefetch_chain_hold = branch_prefetch_hold;
	// Event acceptance
	// Micro-DMA and interrupts are serviced between instructions, DMA first
	// (TMP95C061 datasheet p.14-16). `!wr_pend` keeps a store
	// the arbiter has not taken yet from being overtaken by the event's own
	// accesses; retirement already guarantees it, and this makes it explicit.
	// Micro-DMA cannot release HALT (TMP95C061 datasheet p.23), so only the
	// interrupt arm reaches S_HALT.  Transfers are deferred while halted
	// (S_HALT has no DMA arm); measured against the raster-anchored DMA-HALT
	// signature.

	wire ev_bnd    = (st == S_FETCH0) && !pause_req && !wr_pend;

	// Micro-DMA runs at level 6, "highest among MASKABLE" (TMP95C061
	// datasheet p.14); level 7 (NMI, INTWD) is non-maskable and above it
	// (p.9). Letting DMA win unconditionally would starve NMI forever behind
	// a continuously requesting armed channel.
	wire nmi_pend  = irq_ok && (int_level == 3'd7);
	wire dma_start = ev_bnd && dma_ok && !nmi_pend;
	wire int_start = (ev_bnd && (!dma_ok || nmi_pend) && irq_ok) ||
	                 ((st == S_HALT) && !pause_req && irq_ok);
	wire ev_start  = dma_start || int_start;

	wire [7:0] ev_budget = dma_start ? dma_budget : INT_STATES;

	wire [3:0] dma_sel_1h = (dma_sel == 2'd0) ? 4'b0001 :
	                        (dma_sel == 2'd1) ? 4'b0010 :
	                        (dma_sel == 2'd2) ? 4'b0100 : 4'b1000;
	wire [3:0] dma_ch_1h  = (dma_ch  == 2'd0) ? 4'b0001 :
	                        (dma_ch  == 2'd1) ? 4'b0010 :
	                        (dma_ch  == 2'd2) ? 4'b0100 : 4'b1000;

	// DMAC counts down on the transfer's adjust state and on the counter-mode
	// state; reaching zero raises INTTCn and disarms the channel.
	wire dma_tick = (st == S_DMA) &&
	                (((mstep == 3'd2) && iss_mp) || (mstep == 3'd4));
	wire dma_tc   = dma_tick && (dmac[dma_ch] == 16'd1);

	assign int_ack = ce && int_start;
	assign dma_ack = (ce && dma_start) ? dma_sel_1h : 4'd0;
	assign dma_end = (ce && dma_tc)    ? dma_ch_1h  : 4'd0;

	// One instruction of interrupt shadow. Which events arm it is the
	// EI_SHADOW_RETI parameter's whole job (see the parameter note).  The
	// default arms it after interrupt ENTRY and after RETI: without the
	// return-side inhibit, a request that went pending during a service is
	// accepted at the first boundary after RETI and the interrupted program
	// runs zero instructions between chained services.
	wire shadow_arm = (EI_SHADOW_RETI != 0)
	                  ? ((do_exec && (cur_op == T900_OP_EI)) ||
	                     (int_fin && (ev_kind == EV_RETI)))
	                  : ((int_fin && (ev_kind == EV_INT)) ||
	                     (int_fin && (ev_kind == EV_RETI)));

	// String flag update: LDI-family leaves S and Z alone and clears H and N;
	// the compares take S, Z and H from the subtraction and force N
	// (CPU900H p.96, p.65). V is "BC != 0 after execution" for both, and C is
	// never touched.
	wire [7:0] str_f_new = str_is_cp
	    ? {mp_szh[2:1],   1'b0, mp_szh[0],   1'b0, str_bc_nz, 1'b1, sr[0]}
	    : {sr[7:6],       1'b0, 1'b0,        1'b0, str_bc_nz, 1'b0, sr[0]};

	// MULA takes S, Z and V from the 32-bit accumulate and leaves H, N and C
	// alone (CPU900H p.110).
	wire [7:0] mula_f_new = {alu_flags_out[7:6], 1'b0, sr[4], 1'b0,
	                         alu_flags_out[2], sr[1:0]};

	// =====================================================================
	// Register file write port arbitration
	// =====================================================================
	//
	// Only one of these can be active in any single tick by construction:
	// the EA adjust happens during S_EA, the stack pointer update at the tick
	// the stack read is issued, and the execute write-back at the commit tick.

	wire ea_reg_we = (st == S_EA) && q_pop &&
	                 ((eak_r == T900_K_PRE_DEC) || (eak_r == T900_K_POST_INC));
	wire [31:0] ea_reg_data = (eak_r == T900_K_PRE_DEC)
	                          ? (rf_ra_data - (32'd1 << q_byte[1:0]))
	                          : (rf_ra_data + (32'd1 << q_byte[1:0]));

	wire rst_reg_we = (st == S_RESET);

	// The micro-programs' own register writes. At most one is live in any
	// state by construction, and no micro-program state can coincide with an
	// EA adjust, a stack-pop update or an execute write-back.
	reg        mpr_we;
	reg [7:0]  mpr_code;
	reg [1:0]  mpr_size;
	reg [31:0] mpr_data;

	wire [31:0] str_step = {29'd0, str_nb};

	always @(*) begin
		mpr_we   = 1'b0;
		mpr_code = CODE_SP;
		mpr_size = T900_SZ_L;
		mpr_data = 32'd0;

		if (st == S_INT) begin
			// One write does the whole 6-byte stack movement; the two accesses
			// address off the value read on this state or off `mp_base`.
			if (iss_mp && (mstep == 3'd0)) begin
				mpr_we   = 1'b1;
				mpr_data = (ev_kind == EV_RETI) ? (rf_ra_data + 32'd6)
				                                : (rf_ra_data - 32'd6);
			end
		end else if (st == S_STR) begin
			if (is_mula) begin
				if (iss_mp && (mstep == 3'd1)) begin
					mpr_we   = 1'b1;
					mpr_code = CODE_XHL;
					mpr_data = rf_ra_data - 32'd2;   // XHL <- XHL - 2
				end else if ((mstep == 3'd4) && md_done) begin
					mpr_we   = 1'b1;
					mpr_code = cls_reg_code;
					mpr_data = alu_result;
				end
			end else begin
				case (mstep)
					3'd0: if (mp_rd_ok) begin
						mpr_we   = 1'b1;
						mpr_code = str_src_code;
						mpr_data = str_is_dec ? (rf_ra_data - str_step)
						                      : (rf_ra_data + str_step);
					end
					3'd1: if (iss_mp) begin
						mpr_we   = 1'b1;
						mpr_code = str_dst_code;
						mpr_data = str_is_dec ? (rf_ra_data - str_step)
						                      : (rf_ra_data + str_step);
					end
					// The count-down happens once, on the state the iteration
					// actually advances. `str_hold` can stretch mstep 2 across
					// several states to hold the cadence, and BC must not
					// decrement on each of them.
					3'd2: if (!str_hold) begin
						mpr_we   = 1'b1;
						mpr_code = CODE_BC;
						mpr_size = T900_SZ_W;
						mpr_data = {16'd0, shortR[15:0] - 16'd1};
					end
					default: ;
				endcase
			end
		end
	end

	assign rf_wr_en = ce && (rst_reg_we || ea_reg_we || sp_pop_upd || unlk_sp_we ||
	                         mpr_we || rw2_pend || (do_exec && ex_reg_we));

	assign rf_wr_code = rst_reg_we ? CODE_SP :
	                    ea_reg_we  ? q_byte  :
	                    sp_pop_upd ? CODE_SP :
	                    unlk_sp_we ? CODE_SP :
	                    mpr_we     ? mpr_code :
	                    rw2_pend   ? rw2_code : ex_reg_code;

	assign rf_wr_size = (rst_reg_we || ea_reg_we || sp_pop_upd || unlk_sp_we)
	                    ? T900_SZ_L :
	                    mpr_we ? mpr_size : (rw2_pend ? rw2_size : ex_reg_size);

	assign rf_wr_data = rst_reg_we ? 32'h0000_0100 :
	                    ea_reg_we  ? ea_reg_data :
	                    sp_pop_upd ? (sp_val + {29'd0, rd_n}) :
	                    unlk_sp_we ? (cls_reg + 32'd4) :
	                    mpr_we     ? mpr_data :
	                    rw2_pend   ? rw2_data : ex_reg_data;

	// =====================================================================
	// Queue pop control
	// =====================================================================
	// A decoded taken branch exposes its destination while the displacement is
	// consumed. Send that redirect to the BIU on this state, before the branch's
	// three documented taken states begin. Other flush sources are registered
	// because their targets arrive from multi-state bus microprograms.
	wire branch_redirect = do_exec && ex_branch;

	// The same fast path for the interrupt micro-program's final pop: RETI's
	// PC pop and interrupt entry's vector read are DATA cycles whose target
	// arrives on `drdata` in the ddone state, exactly like a RET. The flush
	// must be combinational so the BIU chains the refill out of the pop's own
	// T2; a registered flush lands one state after the data cycle has ended
	// and misses the chain.
	wire mp_redirect = int_fin;
	assign q_flush  = q_flush_r || branch_redirect || mp_redirect;
	assign q_new_pc = branch_redirect ? ex_target :
	                  mp_redirect     ? drdata[23:0] : q_new_pc_r;
	assign q_redirect = branch_redirect || mp_redirect;

	reg pop_want;
	always @(*) begin
		case (st)
			S_FETCH0: pop_want = !pause_req && !ev_start;
			S_EA:     pop_want = 1'b1;
			S_FETCH1: pop_want = 1'b1;
			S_OPF:    pop_want = 1'b1;
			default:  pop_want = 1'b0;
		endcase
	end

	// A taken transfer invalidates all four bytes. Do not let the target opcode
	// consume the first returning byte while the BIU is still filling the other
	// three: the physical queue probes require all four bytes to be resident
	// before instruction consumption resumes.
	wire queue_refill_block = (st == S_FETCH0) && queue_refill &&
	                          (q_count != 3'd4);

	// A flush invalidates the queue at the end of the state it is asserted in,
	// so a pop in that same state would consume a byte from the old stream.
	assign q_pop = pop_want && pop_ok && !q_flush_r && !queue_refill_block;

	// =====================================================================
	// Savestate tap
	// =====================================================================

	// The core stops between instructions with no bus cycle outstanding, then
	// parks in S_PAUSE. `pause_ready` reports only the parked state, so a
	// savestate engine that acts on it can never race a fetch: every write
	// path in this module and in the register file is gated the same way.
	wire at_boundary = (st == S_FETCH0) && !mem_pend && !wr_pend;
	assign pause_ready = restore_hold || (pause_req && (st == S_PAUSE));

	always @(*) begin
		casez (ss_reg_addr)
			8'h20:        ss_rdata = {8'd0, pc};
			8'h21:        ss_rdata = {8'd0, f_alt, sr};
			8'h22:        ss_rdata = {12'd0, irq_shadow, halt_r, 2'd0, intnest};
			8'b0011_00??: ss_rdata = dmas[ss_reg_addr[1:0]];
			8'b0011_01??: ss_rdata = dmad[ss_reg_addr[1:0]];
			8'b0011_10??: ss_rdata = {8'd0, dmam[ss_reg_addr[1:0]], dmac[ss_reg_addr[1:0]]};
			default:      ss_rdata = 32'd0;
		endcase
	end

	// =====================================================================
	// Sequencer
	// =====================================================================

	always @(*) begin
		sh_start_r = unit_start && shift_op;
		md_start_r = (unit_start && is_muldiv) ||
		             (mula_run && (mstep == 3'd3) && mp_rd_ok);
	end

	always @(posedge clk) begin
		if (reset) begin
			st           <= S_RESET;
			pc           <= VECTOR_BASE;
			sr           <= 16'hF800;
			f_alt        <= 8'd0;
			intnest      <= 16'd0;
			halt_r       <= 1'b0;
			irq_shadow   <= 1'b0;
			shadow_pend  <= 1'b0;
			ev_kind      <= EV_SWI;
			ev_vec       <= 8'd0;
			ev_lvl       <= 3'd0;
			mp_base      <= 24'd0;
			mp_data      <= 32'd0;
			mp_szh       <= 3'd0;
			mula_lhs     <= 16'd0;
			dma_ch       <= 2'd0;
			dma_evt      <= 1'b0;
			q_flush_r    <= 1'b1;
			q_new_pc_r   <= VECTOR_BASE;
			ir0          <= 8'd0;
			ir1          <= 8'd0;
			cls_r        <= CLS_MAIN;
			eak_r        <= T900_K_NONE;
			eareg_r      <= 3'd0;
			clssz_r      <= T900_SZ_B;
			rcode_r      <= 8'd0;
			op_r         <= T900_OP_NOP;
			k1_r         <= T900_K_NONE;
			k2_r         <= T900_K_NONE;
			s1_r         <= T900_SZ_B;
			s2_r         <= T900_SZ_B;
			grp_r        <= T900_G_SYSTEM;
			undef_r      <= 1'b0;
			dec_done     <= 1'b0;
			ea           <= 24'd0;
			ea_full      <= 32'd0;
			ea_done      <= 1'b0;
			ea_base      <= 24'd0;
			ea_base_full <= 32'd0;
			ea_step      <= 2'd0;
			ea_form      <= 5'd0;
			opb          <= 32'd0;
			opv1         <= 32'd0;
			opv2         <= 32'd0;
			opf_cnt      <= 3'd0;
			used         <= 8'd0;
			budget       <= 8'd0;
			budget_valid <= 1'b0;
			pen_carry    <= 6'd0;
			insn_done    <= 1'b0;
			unit_started <= 1'b0;
			md_stall     <= 3'd0;
			md_stall_done <= 1'b0;
			insn_bytes   <= 4'd0;
			insn_mem     <= 1'b0;
			prev_txn     <= 3'd1;
			mstep        <= 3'd0;
			pause_refill <= 2'd0;
			queue_refill <= 1'b1;
			rw2_pend     <= 1'b0;
			rw2_code     <= 8'd0;
			rw2_size     <= T900_SZ_B;
			rw2_data     <= 32'd0;
			wr_pend      <= 1'b0;
			wr_addr      <= 24'd0;
			wr_n         <= 3'd1;
			wr_data      <= 32'd0;
			mem_pend     <= 1'b0;
			mem_we_r     <= 1'b0;
			mem_a        <= 24'd0;
			mem_n        <= 3'd1;
			mem_wd       <= 32'd0;
			mem_ticks    <= 5'd0;
			mem_gfx      <= 5'd0;
			mem_run      <= 1'b0;
			mem_nom      <= 5'd0;
			rd_data      <= 32'd0;
			rd_have      <= 1'b0;
			rd_started   <= 1'b0;
			trace_valid  <= 1'b0;
			for (i = 0; i < 4; i = i + 1) begin
				dmas[i] <= 32'd0;
				dmad[i] <= 32'd0;
				dmac[i] <= 16'd0;
				dmam[i] <= 8'd0;
			end
		end else if (restore_hold) begin
			// The destructive restore reset has completed. Stay at the same
			// architectural boundary as a normal pause without fetching the
			// reset vector or consuming any machine state.
			st           <= S_PAUSE;
			trace_valid  <= 1'b0;
			q_flush_r    <= 1'b1;
			q_new_pc_r   <= pc;
			pause_refill <= 2'd0;
			mem_pend     <= 1'b0;
			wr_pend      <= 1'b0;
			rw2_pend     <= 1'b0;
			rd_started   <= 1'b0;
			rd_have      <= 1'b0;
		end else if (ce) begin
			trace_valid <= 1'b0;
			q_flush_r   <= 1'b0;

			// `queue_refill` blocks opcode consumption until the queue has
			// filled. It is armed at RESET only, where the machine genuinely
			// starts with an empty queue and a vector fetch in front of it.
			// A taken transfer's cost comes from the refill bubble in
			// t900_biu.sv, where it is a bus cycle rather than a rule.
			if (q_flush_r)            queue_refill <= 1'b0;
			else if (q_count == 3'd4) queue_refill <= 1'b0;

			// `used` counts only the states an instruction actually occupies:
			// it is held at zero while S_FETCH0 waits for its first byte and
			// while the core is reset, halted or paused. The state that
			// accepts an interrupt or a micro-DMA request is the first state
			// of that event, so it counts too -- which is what makes the
			// 18-state entry sequence measure 18 and not 19.
			if ((st == S_FETCH0) && !q_pop && !ev_start) used <= 8'd0;
			else if (in_insn || ev_start)                used <= used + 8'd1;
			else                                         used <= 8'd0;


			// data access engine ------------------------
			// The bus-penalty measurement must not include time spent queued
			// behind a prefetch: that is arbitration, not a slow memory, and
			// charging it would push every instruction past its table value.
			// Counting starts on the state the BIU actually takes the request,
			// which is any state it spends idle with the request presented.
			//
			// It seeds at 2, not 1, and that is the whole reason
			// `nominal_states` did not have to move when the BIU stopped
			// spending an issue state on an idle-bus data access. The table
			// budget still assumes the issue state exists, so the measurement
			// has to count it too or every slow region would silently lose a
			// wait state's charge. The state the BIU no longer spends becomes
			// bus SLACK inside an unchanged instruction cost, which is exactly
			// what silicon does.
			if (mem_run) begin
				if (!ddone) begin
					mem_ticks <= mem_ticks + 5'd1;
					if (d_wait_gfx) mem_gfx <= mem_gfx + 5'd1;
				end
			end else if ((mem_pend || iss_req) && d_bus_t1) begin
				mem_run   <= 1'b1;
				mem_ticks <= 5'd2;
				mem_gfx   <= 5'd0;
			end

			if (ddone) begin
				mem_pend <= 1'b0;
				mem_run  <= 1'b0;
				if (!mem_we_r) begin
					rd_data <= drdata;
					rd_have <= 1'b1;
				end
			end

			// An access presented in the ddone state restarts the measurement
			// here and nowhere else, so this has to sit after the clear above.
			// The arm above cannot catch it: `mem_run` still reads 1 on that
			// state. Without this the new access would run unmeasured and its
			// bus penalty would never be charged. The ddone state IS that
			// access's T1 now, so it keys on the same signal and seeds the
			// same way.
			if (ddone && iss_req && d_bus_t1) begin
				mem_run   <= 1'b1;
				mem_ticks <= 5'd2;
				mem_gfx   <= 5'd0;
			end

			// budget ------------------------------------
			if (ev_start) begin
				budget       <= ev_budget + {2'd0, pen_carry} +
				                (mem_over_charge ? 8'd1 : 8'd0);
				budget_valid <= 1'b1;
				pen_carry    <= 6'd0;
			end else if (load_budget || budget_valid) begin
				budget       <= budget_now;
				budget_valid <= 1'b1;
				if (load_budget) pen_carry <= 6'd0;
			end else if (mem_over_charge) begin
				pen_carry <= pen_carry + 6'd1;
			end

			// A repeat iteration that has just been allowed to advance opens
			// the next one's frame. This deliberately lands AFTER the arms
			// above so it overrides them, and after the `used` counter, so the
			// boundary state is the last state of the iteration that is ending
			// and the state after it is the first of the next.
			//
			// Charging each iteration its own frame rather than accumulating
			// one total is what makes the cadence a floor the loop cannot run
			// ahead of, and it keeps `budget` inside 8 bits for a repeat of any
			// length. A slow region's `mem_over` still grows the frame it lands
			// in, so a wait state extends its own iteration and nothing else.
			if (str_iter_adv) begin
				used         <= 8'd0;
				budget       <= {2'd0, str_iter};
				budget_valid <= 1'b1;
			end

			// program counter ---------------------------
			if (q_pop) pc <= pc + 24'd1;

			// bus access issue --------------------------
			if (iss_req) begin
				mem_pend  <= 1'b1;
				mem_we_r  <= iss_we;
				mem_a     <= iss_addr;
				mem_n     <= iss_n;
				mem_wd    <= iss_wd;
				mem_nom   <= nominal_states(iss_n);
				if (iss_rst || iss_rd) rd_started <= 1'b1;
				if (iss_wr)            wr_pend    <= 1'b0;
			end

			// multi-state units -------------------------
			if (unit_start) unit_started <= 1'b1;

			// imm-form word MUL/DIV operand-arrival stall (see unit_start)
			if (md_want_start && md_imm_charged && !md_stall_done) begin
				if (md_stall == 3'd0)
					md_stall <= (cur_op == T900_OP_MUL)  ? 3'd3 :
					            (cur_op == T900_OP_DIVS) ? 3'd3 : 3'd1;
				else begin
					md_stall      <= md_stall - 3'd1;
					md_stall_done <= (md_stall == 3'd1);
				end
			end
			// register-source word MUL/DIV boundary charge (see unit_start)
			else if (md_want_start && md_q_charged && !md_q_ready)
				md_stall <= md_stall + 3'd1;

			// bus traffic of the instruction now executing, for the boundary
			// charge the NEXT instruction may be owed
			if (q_pop && !insn_bytes[3]) insn_bytes <= insn_bytes + 4'd1;
			if (iss_rd || iss_wr)        insn_mem   <= 1'b1;

			// EA bookkeeping ----------------------------
			if (ea_ready_now) begin
				ea      <= ea_val_now;
				ea_full <= ea_full_now;
				ea_done <= 1'b1;
			end else if ((st == S_EA) && q_pop) begin
				ea      <= ea_val_now;
				ea_full <= ea_full_now;
			end
			if ((st == S_EA) && q_pop) begin
				ea_step <= ea_step_next;
				if (ea_form_set)   ea_form <= ea_form_next;
				if (ea_base_latch) begin
					ea_base      <= rf_ra_data[23:0];
					ea_base_full <= rf_ra_data;
				end
			end

			// main state machine ------------------------
			case (st)
				S_RESET: begin
					// SR = 0xF800, XSP = 0x000100, INTNEST = 0, queue held
					// invalid, PC <- mem24[0xFFFF00] (CPU900H p.39).
					q_flush_r <= 1'b1;
					sr      <= 16'hF800;
					intnest <= 16'd0;
					if (rd_ret) begin
						pc         <= drdata[23:0];
						q_new_pc_r <= drdata[23:0];
						rd_started <= 1'b0;
						rd_have    <= 1'b0;
						used       <= 8'd0;
						st         <= S_FETCH0;
					end
				end

				S_FETCH0: begin
					if (pause_req && at_boundary) begin
						st           <= S_PAUSE;
						pause_refill <= 2'd0;
					end else if (dma_start) begin
						dma_ch    <= dma_sel;
						dma_evt   <= 1'b1;
						mstep     <= dma_counter ? 3'd4 : 3'd0;
						insn_done <= 1'b0;
						st        <= S_DMA;
					end else if (int_start) begin
						ev_kind   <= EV_INT;
						ev_vec    <= int_vector;
						ev_lvl    <= int_level;
						mstep     <= 3'd0;
						insn_done <= 1'b0;
						st        <= S_INT;
					end else if (q_pop) begin
						ir0       <= q_byte;
						cls_r     <= dec_cls;
						eak_r     <= dec_ea_kind;
						eareg_r   <= dec_ea_reg;
						clssz_r   <= dec_cls_size;
						opf_cnt   <= 3'd0;
						opb       <= 32'd0;
						opv1      <= 32'd0;
						opv2      <= 32'd0;
						ea_step   <= 2'd0;
						insn_done <= 1'b0;

						if (dec_needs_byte1) begin
							dec_done <= 1'b0;
							st       <= ea_ready_now ? S_FETCH1 : S_EA;
						end else begin
							dec_done <= 1'b1;
							op_r     <= dec_op_id;
							k1_r     <= dec_op1_kind;
							k2_r     <= dec_op2_kind;
							s1_r     <= dec_op1_size;
							s2_r     <= dec_op2_size;
							grp_r    <= dec_group;
							undef_r  <= dec_undef;
							ea_done  <= 1'b1;
							if (dec_undef) begin
								// INTUNDEF: an undefined opcode traps as SWI 2
								// (TMP95C061 datasheet p.12).
								ev_kind <= EV_SWI;
								ev_vec  <= UNDEF_VECTOR;
								mstep   <= 3'd0;
								st      <= S_INT;
							end else if (dec_op_id == T900_OP_SWI) begin
								ev_kind <= EV_SWI;
								ev_vec  <= {3'd0, q_byte[2:0], 2'd0};
								mstep   <= 3'd0;
								st      <= S_INT;
							end else if (dec_op_id == T900_OP_RETI) begin
								ev_kind <= EV_RETI;
								mstep   <= 3'd0;
								st      <= S_INT;
							end else if (opf_total != 3'd0) begin
								st <= S_OPF;
							end else if (do_exec) begin
								st <= S_NEXT;
							end else begin
								st <= S_RD;
							end
						end
					end
				end

				S_EA: if (q_pop) begin
					if (eak_r == T900_K_RCODE) rcode_r <= q_byte;
					if (ea_last) st <= S_FETCH1;
				end

				S_FETCH1: if (q_pop) begin
					ir1      <= q_byte;
					dec_done <= 1'b1;
					op_r     <= dec_op_id;
					k1_r     <= dec_op1_kind;
					k2_r     <= dec_op2_kind;
					s1_r     <= dec_op1_size;
					s2_r     <= dec_op2_size;
					grp_r    <= dec_group;
					undef_r  <= dec_undef;

					if (dec_undef) begin
						ev_kind <= EV_SWI;
						ev_vec  <= UNDEF_VECTOR;
						mstep   <= 3'd0;
						st      <= S_INT;
					end else if (dec_group == T900_G_STRING) begin
						// The class source read was issued in S_FETCH0, so the
						// first iteration is already waiting on its data.
						mstep <= 3'd0;
						st    <= S_STR;
					end else if (dec_op_id == T900_OP_MULA) begin
						// `mula_pre` issued the (XDE) read in this very state.
						mstep <= 3'd1;
						st    <= S_STR;
					end else if (opf_total != 3'd0) begin
						st <= S_OPF;
					end else if (do_exec) begin
						st <= S_NEXT;
					end else if (unit_start) begin
						st <= S_EXEC;
					end else begin
						st <= S_RD;
					end
				end

				S_OPF: if (q_pop) begin
					opf_cnt <= opf_cnt + 3'd1;
					if (in_op1 && ((opf_cnt + 3'd1) == n1_held)) begin
						opv1 <= opb_now;
						opb  <= 32'd0;
					end else if ((opf_cnt + 3'd1) == opf_total_held) begin
						opv2 <= opb_now;
						opb  <= 32'd0;
					end else begin
						opb <= opb_now;
					end
					if (is_ldx_held) begin
						// Payload rides bytes 1 and 3; the rest is padding.
						if (opf_cnt == 3'd1) opv1 <= {24'd0, q_byte};
						if (opf_cnt == 3'd3) opv2 <= {24'd0, q_byte};
					end
					if ((opf_cnt + 3'd1) == opf_total_held)
						st <= do_exec ? S_NEXT : (unit_start ? S_EXEC : S_RD);
				end

				S_RD: begin
					if (do_exec)         st <= S_NEXT;
					else if (unit_start) st <= S_EXEC;
				end

				S_EXEC: begin
					if (do_exec) st <= S_NEXT;
				end

				S_NEXT: ;   // retirement is handled below

				S_HALT: begin
					// An interrupt wakes HALT; micro-DMA must not
					// (TMP95C061 datasheet p.23).
					if (pause_req) begin
						st           <= S_PAUSE;
						pause_refill <= 2'd0;
					end else if (int_start) begin
						halt_r    <= 1'b0;
						ev_kind   <= EV_INT;
						ev_vec    <= int_vector;
						ev_lvl    <= int_level;
						mstep     <= 3'd0;
						insn_done <= 1'b0;
						st        <= S_INT;
					end else if (halt_release) begin
						// Below the IFF mask: back to RUN mode at the
						// instruction after HALT, no handler
						// (TMP95C061 datasheet p.28 Table 3.4(2)). PC already
						// points there -- HALT retired before parking here
						// -- so the queue needs no flush.
						halt_r <= 1'b0;
						st     <= S_FETCH0;
					end
				end

				S_PAUSE: begin
					// The prefetch queue is deliberately absent from the state
					// image. Every release therefore rebuilds it from architectural
					// PC, regardless of whether the pause was a read-only save or a
					// restore with writes. Keeping S_PAUSE until all four bytes have
					// refilled gives both paths one canonical boundary without
					// charging refill states to the first instruction.
					if (!pause_req && (pause_refill == 2'd0)) begin
						q_flush_r    <= 1'b1;
						q_new_pc_r   <= pc;
						pause_refill <= 2'd1;
					end else if (!pause_req && (pause_refill == 2'd1)) begin
						// q_flush is registered by this sequencer, so the BIU sees
						// it one ce after the assignment above. Do not mistake the
						// still-full old queue for a completed refill on that edge.
						pause_refill <= 2'd2;
					end else if (!pause_req && (pause_refill == 2'd2) &&
					             (q_count == 3'd4) && !bus_busy) begin
						pause_refill <= 2'd0;
						st           <= halt_r ? S_HALT : S_FETCH0;
					end
				end

				// Vector entry (SWI n, INTUNDEF, hardware interrupt) and
				// RETI (TMP95C061 datasheet p.10-11; CPU900H p.127, p.164).
				S_INT: begin
					if (ev_kind == EV_RETI) begin
						case (mstep)
							3'd0: if (iss_mp) begin
								mp_base <= rf_ra_data[23:0];
								mstep   <= 3'd1;
							end
							3'd1: if (ddone) begin
								// The full SR pop restores RFP, which is how
								// bank switching unwinds (CPU900H p.11).
								// If the PC pop was accepted in this very
								// state (the ddone-state presentation), the
								// re-present state is skipped.
								sr    <= sr_fix(drdata[15:0]);
								mstep <= iss_mp ? 3'd3 : 3'd2;
							end
							3'd2: if (iss_mp) mstep <= 3'd3;
							default: if (ddone) begin
								// The flush is COMBINATIONAL on this state
								// (`mp_redirect` above), so the BIU chains the
								// refill out of this pop's own T2; setting
								// q_flush_r here as well would flush that new
								// stream a state later.
								pc         <= drdata[23:0];
								q_new_pc_r <= drdata[23:0];
								intnest    <= intnest - 16'd1;
								insn_done  <= 1'b1;
								st         <= S_NEXT;
							end
						endcase
					end else begin
						case (mstep)
							3'd0, 3'd2, 3'd4: if (iss_mp) mstep <= mstep + 3'd1;
							// A ddone with the next access accepted in the
							// same state jumps the re-present step entirely.
							3'd1, 3'd3:       if (ddone)  mstep <= mstep + (iss_mp ? 3'd2 : 3'd1);
							default: if (ddone) begin
								// Combinational flush, as the RETI arm above.
								pc         <= drdata[23:0];
								q_new_pc_r <= drdata[23:0];
								if (ev_kind == EV_INT) begin
									// IFF <- level + 1, level 7 stays 7.
									sr[14:12] <= (ev_lvl == 3'd7) ? 3'd7
									                              : (ev_lvl + 3'd1);
									intnest   <= intnest + 16'd1;
								end
								insn_done <= 1'b1;
								st        <= S_NEXT;
							end
						endcase
					end
				end

				// One micro-DMA transfer (TMP95C061 datasheet p.14-16).
				S_DMA: begin
					case (mstep)
						3'd0: if (iss_mp) mstep <= 3'd1;
						3'd1: if (mp_rd_ok) begin
							mp_data <= rd_now;
							mstep   <= 3'd2;
						end
						// The adjust waits for the store to be issued: the
						// store's address IS the un-adjusted DMAD.
						3'd2: if (iss_mp) begin
							case (dma_kind)
								3'd0: dmad[dma_ch] <= dmad[dma_ch] + {29'd0, dma_nb};
								3'd1: dmad[dma_ch] <= dmad[dma_ch] - {29'd0, dma_nb};
								3'd2: dmas[dma_ch] <= dmas[dma_ch] + {29'd0, dma_nb};
								3'd3: dmas[dma_ch] <= dmas[dma_ch] - {29'd0, dma_nb};
								default: ;      // 100ZZ: both addresses fixed
							endcase
							dmac[dma_ch] <= dmac[dma_ch] - 16'd1;
							insn_done    <= 1'b1;
							st           <= S_NEXT;
						end
						default: begin
							// 10100 counter mode: no transfer, DMAS counts up
							// by one whatever ZZ says (TMP95C061 datasheet
							// p.16).
							dmas[dma_ch] <= dmas[dma_ch] + 32'd1;
							dmac[dma_ch] <= dmac[dma_ch] - 16'd1;
							insn_done    <= 1'b1;
							st           <= S_NEXT;
						end
					endcase
				end

				// String / repeat group and MULA.
				S_STR: begin
					if (is_mula) begin
						case (mstep)
							// (XDE) lands and (XHL) is issued on the same
							// state; if the bus is not free yet the data waits
							// in rd_data and this retries.
							3'd1: if (iss_mp) begin
								mula_lhs <= rd_now[15:0];
								rd_have  <= 1'b0;
								mstep    <= 3'd3;
							end
							3'd3: if (mp_rd_ok) mstep <= 3'd4;  // md_start fires
							default: if (md_done) begin
								sr[7:0]   <= mula_f_new & 8'hD7;
								insn_done <= 1'b1;
								st        <= S_NEXT;
							end
						endcase
					end else begin
						case (mstep)
							// Source data in: latch it, latch the compare's
							// flags, and step the source pointer.
							3'd0: if (mp_rd_ok) begin
								mp_data <= rd_now;
								if (str_is_cp) mp_szh <= {alu_flags_out[7:6], alu_flags_out[4]};
								mstep <= str_is_cp ? 3'd2 : 3'd1;
							end
							// Store to the destination and step its pointer.
							3'd1: if (iss_mp) mstep <= 3'd2;
							// Count down, commit flags, decide. Held here by
							// `str_hold` until the iteration has occupied its
							// 7 (or 6) states; nothing in this arm may commit
							// on a held state, and `str_f_new` is stable across
							// the hold because the BC write is held with it.
							3'd2: if (!str_hold) begin
								sr[7:0] <= str_f_new & 8'hD7;
								if (str_loop) begin
									rd_have <= 1'b0;
									mstep   <= 3'd3;
								end else begin
									if (str_more) begin
										// Interrupted between iterations: leave
										// PC on the instruction so it resumes
										// (CPU900H p.97).
										pc         <= pc - 24'd2;
										q_new_pc_r <= pc - 24'd2;
										q_flush_r  <= 1'b1;
									end
									insn_done <= 1'b1;
									st        <= S_NEXT;
								end
							end
							default: if (iss_mp) mstep <= 3'd0;
						endcase
					end
				end

				default: st <= S_FETCH0;
			endcase

			// commit the execute description ------------
			if (do_exec) begin
				insn_done <= 1'b1;
				if (ex_f_we)    sr[7:0] <= ex_f & 8'hD7;
				// SYSM and MAX forced 1, RFP2 and F bits 5 and 3 forced 0.
				if (ex_sr_we)   sr      <= sr_fix(ex_sr);
				if (ex_falt_we) f_alt   <= ex_falt;
				if (ex_halt)    halt_r  <= 1'b1;
				if (ex_reg2_we) begin
					rw2_pend <= 1'b1;
					rw2_code <= ex_reg2_code;
					rw2_size <= ex_reg2_size;
					rw2_data <= ex_reg2_data;
				end
				if (ex_branch) begin
					pc       <= ex_target;
					if (!branch_redirect) begin
						q_new_pc_r <= ex_target;
						q_flush_r  <= 1'b1;
					end
				end
				if ((cur_op == T900_OP_LDC) && !cls_reg_is_op1) begin
					casez (cr_code)
						8'h3C:        intnest     <= src_val[15:0];
						8'b0000_??00: dmas[cr_ch] <= src_val;
						8'b0001_??00: dmad[cr_ch] <= src_val;
						8'b0010_??00: dmac[cr_ch] <= src_val[15:0];
						8'b0010_??10: dmam[cr_ch] <= src_val[7:0];
						default: ;
					endcase
				end
			end

			// The deferred half of EX R,r retires one state later.
			if (rw2_pend) rw2_pend <= 1'b0;

			// deferred store ----------------------------
			// A store the arbiter could not take this state waits here.
			if (do_exec && ex_mem_we && !iss_wr) begin
				wr_pend <= 1'b1;
				wr_addr <= ex_mem_addr;
				wr_n    <= ex_mem_n;
				wr_data <= ex_mem_data;
			end

			// interrupt shadow --------------------------
			// The shadow covers exactly one INSTRUCTION: it is armed when its
			// event completes and dropped when the next instruction retires.
			//
			// A micro-DMA transfer is not an instruction. Toshiba's protection
			// runs until "the start instruction of the interrupt processing is
			// executed" (TMP95C061 datasheet p.11), and micro-DMA is a
			// CPU-microcode transfer serviced BETWEEN instructions
			// (TMP95C061 datasheet p.14-16), so a transfer that slips in
			// ahead of the handler's first instruction has to leave the
			// shadow standing.
			//
			// Whether the shadow should also BLOCK micro-DMA is a different
			// question and no source in the corpus answers it, so it is
			// deliberately not assumed here: the transfer still runs, it just
			// no longer ends the shadow.
			if (retire_now && !dma_evt) begin
				irq_shadow  <= shadow_pend || shadow_arm;
				shadow_pend <= 1'b0;
			end else if (shadow_arm) begin
				shadow_pend <= 1'b1;
			end

			// retirement --------------------------------
			if (retire_now) begin
				trace_valid  <= 1'b1;
				dma_evt      <= 1'b0;
				st           <= (halt_r || (do_exec && ex_halt)) ? S_HALT : S_FETCH0;
				budget_valid <= 1'b0;
				rd_started   <= 1'b0;
				rd_have      <= 1'b0;
				ea_done      <= 1'b0;
				dec_done     <= 1'b0;
				insn_done    <= 1'b0;
				unit_started <= 1'b0;
				md_stall     <= 3'd0;
				md_stall_done <= 1'b0;
				prev_txn     <= prev_txn_now;
				insn_bytes   <= 4'd0;
				insn_mem     <= 1'b0;
				used         <= 8'd0;
			end
		end

		// savestate writes (paused only) ------------
		// NOT gated by `ce`. The savestate engine asserts its write strobe for a
		// single clk_sys cycle (savestates.sv drives BUS_wren for one clock),
		// while `ce` is one cycle in sixteen at gear 0 and one in 256 at gear 4 --
		// so a ce-gated write would be dropped fifteen times in sixteen and a CPU
		// restore would silently land only in part. The strobe is itself the
		// enable, which is the same rule the MCU shell applies to its SFR
		// writes. Placed after the ce branch so a
		// restore wins over anything that branch assigns in the same cycle, and
		// guarded on the active request as well as S_PAUSE: release now remains
		// in S_PAUSE while refilling, but that is no longer an authorized write
		// window once the requester has dropped pause_req.
		if (!reset && ss_wren &&
		    (restore_hold || (pause_req && (st == S_PAUSE)))) begin
			// If a requester reasserts pause and writes after a release refill
			// already started, the next release must restart from the new PC.
			pause_refill <= 2'd0;
			casez (ss_reg_addr)
				8'h20: pc <= ss_wdata[23:0];
				// A restore is held to the same architectural invariant as any
				// other wholesale SR write: an image without the fixed bits is not
				// a state this CPU can be in.
				8'h21: begin sr <= sr_fix(ss_wdata[15:0]); f_alt <= ss_wdata[23:16]; end
				8'h22: begin
					intnest    <= ss_wdata[15:0];
					halt_r     <= ss_wdata[18];
					irq_shadow <= ss_wdata[19];
				end
				8'b0011_00??: dmas[ss_reg_addr[1:0]] <= ss_wdata;
				8'b0011_01??: dmad[ss_reg_addr[1:0]] <= ss_wdata;
				8'b0011_10??: begin
					dmac[ss_reg_addr[1:0]] <= ss_wdata[15:0];
					dmam[ss_reg_addr[1:0]] <= ss_wdata[23:16];
				end
				default: ;
			endcase
		end
	end

	// The extended-form selector only ever uses its low two bits.
	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused = &{1'b0, ea_form_next[4:2]};
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
