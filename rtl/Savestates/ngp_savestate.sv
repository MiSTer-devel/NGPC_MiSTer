// Copyright (c) 2026 Jamie Blanks

`default_nettype none

// NGPC manual-state transaction wrapper.
//
// The generic engine owns only internals and memory types 0..2. Cartridge
// flash is an externally managed sparse Type3 tail: ngp_sparse_state_store
// pre-fills it before a save header is committed and fully validates/stages it
// before a load is allowed to reset or modify the machine. No state contains
// an unconditional cartridge image.
module ngp_savestate
#(
	parameter integer SAVESTATE_ADDR  = 32'h0080_0000,
	parameter integer SAVESTATE_SHIFT = 21
)
(
	input  wire        clk,
	input  wire        reset,

	input  wire        ss_save,
	input  wire        ss_load,
	input  wire [1:0]  ss_slot,
	input  wire        increase_header_count,

	output wire        pause_req,
	input  wire        paused,
	// Unlike a normal pause acknowledgement, this remains true while the
	// parent holds the whole machine in reset for sparse flash application.
	input  wire        restore_safe_stopped_i,
	output wire        core_reset,
	output wire        loading_savestate,
	output wire        busy,
	output reg         load_done,
	output wire        restore_is_rewind,

	input  wire [31:0] identity_raw_crc32_i,
	input  wire [31:0] identity_raw_bytes_i,
	input  wire [31:0] identity_pristine_crc32_i,
	input  wire [31:0] identity_physical_bytes_i,
	input  wire [1:0]  identity_die0_code_i,
	input  wire [1:0]  identity_die1_code_i,
	// ngp_cart briefly clears/reloads its population registers during the
	// restore power-on reset. Sparse apply must wait for the retained image
	// geometry to be strapped again before interpreting physical block maps.
	input  wire        cart_geometry_ready_i,
	input  wire [15:0] identity_catalog_i,
	input  wire [7:0]  identity_subcatalog_i,
	input  wire [95:0] identity_title_i,
	input  wire [34:0] live_ledger0_i,
	input  wire [34:0] live_ledger1_i,
	input  wire [1:0]  die_busy_i,
	input  wire        event0_i,
	input  wire [5:0]  block0_i,
	input  wire        event1_i,
	input  wire [5:0]  block1_i,
	output wire        state_ledger_adopt_o,
	output wire [34:0] state_ledger0_o,
	output wire [34:0] state_ledger1_o,
	output wire        state_force_flash_read_o,

	output wire [9:0]  ss_bus_adr,
	output wire [63:0] ss_bus_din,
	output wire        ss_bus_wren,
	output wire        ss_bus_rst,
	input  wire [63:0] ss_bus_dout,

	output wire [1:0]  ss_mem_type,
	output wire        ss_mem_active,
	output wire [13:0] ss_mem_addr,
	output wire [7:0]  ss_mem_wdata,
	output wire        ss_mem_wren,
	output wire        ss_mem_rden,
	input  wire [7:0]  ss_mem_rdata,

	// Generic state engine, DDR3 channel 1.
	output wire [27:1] ddr_addr,
	output wire [63:0] ddr_din,
	input  wire [63:0] ddr_dout,
	output wire        ddr_req,
	output wire        ddr_rnw,
	output wire [7:0]  ddr_be,
	input  wire        ddr_ready,

	// Sparse Type3 store, shared DDR3 channel 2.
	output wire [27:1] sparse_ddr_addr_o,
	output wire [63:0] sparse_ddr_din_o,
	output wire        sparse_ddr_req_o,
	output wire        sparse_ddr_rnw_o,
	output wire [7:0]  sparse_ddr_be_o,
	input  wire [63:0] sparse_ddr_dout_i,
	input  wire        sparse_ddr_ready_i,

	// Sparse store's canonical live-cart p2 client.
	output wire        p2_req_o,
	output wire        p2_we_o,
	output wire [24:0] p2_addr_o,
	output wire [15:0] p2_wdata_o,
	output wire [1:0]  p2_be_o,
	input  wire        p2_ready_i,
	input  wire        p2_done_i,
	input  wire [15:0] p2_rdata_i
);

	localparam [2:0] WR_IDLE          = 3'd0;
	localparam [2:0] WR_SAVE_PREP     = 3'd1;
	localparam [2:0] WR_SAVE_ENGINE   = 3'd2;
	localparam [2:0] WR_LOAD_PREFLIGHT = 3'd3;
	localparam [2:0] WR_LOAD_ENGINE   = 3'd4;
	localparam [2:0] WR_REWIND_ENGINE = 3'd5;

	localparam [31:0] FIXED_STATE_DWORDS = 32'd8416;

	reg [2:0] wrapper_state_q;
	reg       engine_save_q;
	reg       engine_load_q;
	reg       engine_busy_q;
	reg       engine_restore_seen_q;
	reg       restore_hold_q;
	reg       rewind_mode_q;
	reg       rewind_load_q;
	integer   engine_address_q;

	wire request_savestate_w;
	wire request_loadstate_w;
	wire request_is_rewind_w;
	integer request_address_w;

	wire engine_busy_w;
	wire engine_pause_w;
	wire engine_loading_w;
	wire engine_saving_w;
	wire engine_load_done_w;
	wire engine_restore_begin_w;
	wire engine_reset_delay_unused_w;

	// request_busy is the complete wrapper transaction, not merely the generic
	// engine. This prevents statemanager from issuing another request during
	// sparse pre-capture or load preflight.
	assign busy = (wrapper_state_q != WR_IDLE) || engine_busy_w || store_busy_w;

	statemanager
	#(
		.Softmap_SaveState_ADDR (SAVESTATE_ADDR),
		.Softmap_Rewind_ADDR    (SAVESTATE_ADDR),
		.SAVESTATE_SHIFT        (SAVESTATE_SHIFT)
	)
	u_statemanager
	(
		.clk(clk), .reset(reset),
		.rewind_on(1'b0), .rewind_active(1'b0),
		.savestate_number({30'd0, ss_slot}),
		.save(ss_save), .load(ss_load),
		.sleep_rewind(), .vsync(1'b0),
		.request_savestate(request_savestate_w),
		.request_loadstate(request_loadstate_w),
		.request_is_rewind(request_is_rewind_w),
		.request_address(request_address_w), .request_busy(busy)
	);

	// ---------------------- sparse Type3 transaction -----------------------
	wire store_ready_w, store_accepted_w, store_busy_w;
	wire store_done_w, store_rejected_w, store_rewind_bypass_w;
	wire store_pause_w, store_save_prepared_w, store_load_preflight_w;
	wire store_load_apply_ready_w, store_load_apply_failed_w;
	wire [31:0] store_type3_bytes_w, store_state_size_w;
	wire [34:0] store_frozen0_w, store_frozen1_w;
	wire store_engine_save_done_w = (wrapper_state_q == WR_SAVE_ENGINE) &&
		engine_busy_q && !engine_busy_w;
	wire store_cancel_w = (wrapper_state_q == WR_LOAD_ENGINE) &&
		engine_busy_q && !engine_busy_w && !engine_restore_seen_q &&
		!engine_load_done_w;

	ngp_sparse_state_store u_sparse_store
	(
		.clk(clk), .reset(reset),
		.save_start_i(request_savestate_w && !request_is_rewind_w),
		.load_start_i(request_loadstate_w && !request_is_rewind_w),
		.request_is_rewind_i(1'b0), .cancel_i(store_cancel_w),
		.slot_base_dword_i(request_address_w[25:0]),
		.request_ready_o(store_ready_w), .request_accepted_o(store_accepted_w),
		.busy_o(store_busy_w), .done_o(store_done_w),
		.rejected_o(store_rejected_w), .rewind_bypass_o(store_rewind_bypass_w),
		.identity_raw_crc32_i(identity_raw_crc32_i),
		.identity_raw_bytes_i(identity_raw_bytes_i),
		.identity_pristine_crc32_i(identity_pristine_crc32_i),
		.identity_physical_bytes_i(identity_physical_bytes_i),
		.identity_die0_code_i(identity_die0_code_i),
		.identity_die1_code_i(identity_die1_code_i),
		.geometry_ready_i(cart_geometry_ready_i),
		.identity_catalog_i(identity_catalog_i),
		.identity_subcatalog_i(identity_subcatalog_i),
		.identity_title_i(identity_title_i),
		.live_ledger0_i(live_ledger0_i), .live_ledger1_i(live_ledger1_i),
		.die_busy_i(die_busy_i), .event0_i(event0_i), .block0_i(block0_i),
		.event1_i(event1_i), .block1_i(block1_i),
		.pause_req_o(store_pause_w), .pause_ready_i(restore_safe_stopped_i),
		.save_prepared_o(store_save_prepared_w),
		.engine_save_done_i(store_engine_save_done_w),
		.load_preflight_ready_o(store_load_preflight_w),
		.engine_restore_begin_i(engine_restore_begin_w),
		.engine_load_done_i(engine_load_done_w),
		.load_apply_ready_o(store_load_apply_ready_w),
		.load_apply_failed_o(store_load_apply_failed_w),
		.frozen_type3_bytes_o(store_type3_bytes_w),
		.frozen_state_size_dwords_o(store_state_size_w),
		.frozen_ledger0_o(store_frozen0_w), .frozen_ledger1_o(store_frozen1_w),
		.ledger_adopt_o(state_ledger_adopt_o),
		.force_flash_read_o(state_force_flash_read_o),
		.ledger_target0_o(state_ledger0_o), .ledger_target1_o(state_ledger1_o),
		.p2_req_o(p2_req_o), .p2_we_o(p2_we_o), .p2_addr_o(p2_addr_o),
		.p2_wdata_o(p2_wdata_o), .p2_be_o(p2_be_o),
		.p2_ready_i(p2_ready_i), .p2_done_i(p2_done_i), .p2_rdata_i(p2_rdata_i),
		.ddr_addr_o(sparse_ddr_addr_o), .ddr_din_o(sparse_ddr_din_o),
		.ddr_req_o(sparse_ddr_req_o), .ddr_rnw_o(sparse_ddr_rnw_o),
		.ddr_be_o(sparse_ddr_be_o), .ddr_dout_i(sparse_ddr_dout_i),
		.ddr_ready_i(sparse_ddr_ready_i)
	);

	// ------------------------- generic engine ------------------------------
	wire [2:0] ram_type_w;
	wire [24:0] ram_addr_w;
	wire ram_rden_w, ram_wren_w;
	wire [7:0] ram_wdata_w, ram_rdata_w;
	wire ram_ready_w;
	wire [25:0] engine_ddr_adr_w;
	wire [31:0] engine_state_size_w = rewind_mode_q ? FIXED_STATE_DWORDS :
		store_state_size_w;

	assign ddr_addr = {engine_ddr_adr_w, 1'b0};

	savestates
	#(
		.STATESIZE_PARAM      (8416),
		.SETTLECOUNT_PARAM    (16),
		.INTERNALSCOUNT_PARAM (112),
		.SAVETYPESCOUNT_PARAM (3),
		.SAVETYPE0_SIZE       (12288),
		.SAVETYPE1_SIZE       (4096),
		.SAVETYPE2_SIZE       (16384),
		.SAVETYPE3_SIZE       (0)
	)
	u_savestates
	(
		.clk(clk), .reset_in(reset), .reset_ss(core_reset),
		.reset_delay(engine_reset_delay_unused_w),
		.restore_begin(engine_restore_begin_w), .load_done(engine_load_done_w),
		.restore_prepare_ready_i(rewind_mode_q || store_load_apply_ready_w),
		.restore_prepare_failed_i(!rewind_mode_q && store_load_apply_failed_w),
		.increaseSSHeaderCount(increase_header_count),
		.save(engine_save_q), .load(engine_load_q),
		.state_size_i(engine_state_size_w), .savetype3_size_i(25'd0),
		.is_rewind_i(rewind_mode_q), .savestate_address(engine_address_q),
		.savestate_busy(engine_busy_w), .paused(paused),
		.BUS_Din(ss_bus_din), .BUS_Adr(ss_bus_adr),
		.BUS_wren(ss_bus_wren), .BUS_rst(ss_bus_rst), .BUS_Dout(ss_bus_dout),
		.loading_savestate(engine_loading_w), .saving_savestate(engine_saving_w),
		.sleep_savestate(engine_pause_w),
		.Save_RAMAddr(ram_addr_w), .Save_RAMRdEn(ram_rden_w),
		.Save_RAMWrEn(ram_wren_w), .Save_RAMWriteData(ram_wdata_w),
		.Save_RAMReadData(ram_rdata_w), .Save_RAMReady(ram_ready_w),
		.Save_RAMType(ram_type_w),
		.bus_out_Din(ddr_din), .bus_out_Dout(ddr_dout),
		.bus_out_Adr(engine_ddr_adr_w), .bus_out_rnw(ddr_rnw),
		.bus_out_ena(ddr_req), .bus_out_be(ddr_be), .bus_out_done(ddr_ready)
	);

	ngp_ss_glue u_glue
	(
		.clk(clk), .reset(reset),
		.ss_ram_type(ram_type_w), .ss_ram_addr(ram_addr_w),
		.ss_ram_rden(ram_rden_w), .ss_ram_wren(ram_wren_w),
		.ss_ram_wdata(ram_wdata_w), .ss_ram_rdata(ram_rdata_w),
		.ss_ram_ready(ram_ready_w),
		.mem_type(ss_mem_type), .mem_active(ss_mem_active),
		.mem_addr(ss_mem_addr), .mem_wdata(ss_mem_wdata),
		.mem_wren(ss_mem_wren), .mem_rden(ss_mem_rden),
		.mem_rdata(ss_mem_rdata)
	);

	assign pause_req = engine_pause_w | store_pause_w;
	// This is the board-wide restore-service hold, not merely the generic
	// engine's narrower loading phase. It starts with the destructive reset and
	// remains asserted through sparse ledger adoption, so no machine enable can
	// appear between restored data and its canonical metadata.
	assign loading_savestate = restore_hold_q;
	assign restore_is_rewind = rewind_mode_q;

	always @(posedge clk) begin
		engine_save_q <= 1'b0;
		engine_load_q <= 1'b0;
		load_done <= 1'b0;
		engine_busy_q <= engine_busy_w;
		if (engine_restore_begin_w) begin
			engine_restore_seen_q <= 1'b1;
			restore_hold_q <= 1'b1;
		end

		if (reset) begin
			wrapper_state_q <= WR_IDLE;
			engine_save_q <= 1'b0;
			engine_load_q <= 1'b0;
			engine_busy_q <= 1'b0;
			engine_restore_seen_q <= 1'b0;
			rewind_mode_q <= 1'b0;
			rewind_load_q <= 1'b0;
			restore_hold_q <= 1'b0;
			engine_address_q <= 0;
			load_done <= 1'b0;
		end else begin
			case (wrapper_state_q)
				WR_IDLE: begin
					engine_restore_seen_q <= 1'b0;
					if (request_savestate_w || request_loadstate_w) begin
						engine_address_q <= request_address_w;
						rewind_mode_q <= request_is_rewind_w;
						rewind_load_q <= request_loadstate_w;
						if (request_is_rewind_w) begin
							engine_save_q <= request_savestate_w;
							engine_load_q <= request_loadstate_w;
							wrapper_state_q <= WR_REWIND_ENGINE;
						end else if (request_savestate_w) begin
							wrapper_state_q <= WR_SAVE_PREP;
						end else begin
							wrapper_state_q <= WR_LOAD_PREFLIGHT;
						end
					end
				end

				WR_SAVE_PREP: begin
					if (store_save_prepared_w) begin
						engine_save_q <= 1'b1;
						wrapper_state_q <= WR_SAVE_ENGINE;
					end else if (store_rejected_w) begin
						wrapper_state_q <= WR_IDLE;
					end
				end

				WR_SAVE_ENGINE: begin
					if (store_done_w || store_rejected_w) begin
						rewind_mode_q <= 1'b0;
						wrapper_state_q <= WR_IDLE;
					end
				end

				WR_LOAD_PREFLIGHT: begin
					if (store_load_preflight_w) begin
						engine_load_q <= 1'b1;
						wrapper_state_q <= WR_LOAD_ENGINE;
					end else if (store_rejected_w) begin
						wrapper_state_q <= WR_IDLE;
					end
				end

				WR_LOAD_ENGINE: begin
					if (store_done_w) begin
						load_done <= 1'b1;
						restore_hold_q <= 1'b0;
						rewind_mode_q <= 1'b0;
						wrapper_state_q <= WR_IDLE;
					end else if (store_rejected_w) begin
						restore_hold_q <= 1'b0;
						rewind_mode_q <= 1'b0;
						wrapper_state_q <= WR_IDLE;
					end
				end

				WR_REWIND_ENGINE: begin
					if (engine_busy_q && !engine_busy_w) begin
						restore_hold_q <= 1'b0;
						if (rewind_load_q && engine_load_done_w) load_done <= 1'b1;
						rewind_mode_q <= 1'b0;
						wrapper_state_q <= WR_IDLE;
					end
				end

				default: wrapper_state_q <= WR_IDLE;
			endcase
		end
	end

	// The sparse store's status-only outputs are retained in the lint cone. A
	// rejected request simply returns the wrapper to idle; UI feedback remains
	// the framework's existing state-slot notification path.
	/* verilator lint_off UNUSED */
	wire unused_ok = &{1'b0, store_ready_w, store_accepted_w,
		store_rewind_bypass_w, store_type3_bytes_w, store_frozen0_w,
		store_frozen1_w, engine_loading_w, engine_saving_w,
		engine_reset_delay_unused_w, 1'b0};
	/* verilator lint_on UNUSED */

endmodule

`default_nettype wire
