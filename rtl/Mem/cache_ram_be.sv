// Copyright (c) 2026 Jamie Blanks

// Byte-enable true-dual-port block RAM wrapper, companion to cache_ram.v.
// Quartus uses the Altera primitive; other tools use the portable model.
// Accesses take one clock on both ports. Read-during-write on a written
// address returns the new data on enabled byte lanes and the old data on
// masked lanes in the portable model; the hardware primitive leaves masked
// lanes undefined, so no design may rely on masked-lane readback.

module cache_ram_dp_be
#(
	parameter ADDR_WIDTH = 10,
	parameter DATA_WIDTH = 16,
	/* verilator lint_off UNUSEDPARAM */
	parameter MEM_INIT_FILE = " ",
	parameter DEVICE_FAMILY = "Cyclone V",
	/* verilator lint_on UNUSEDPARAM */
	parameter SIM_INIT_FILE = " "
)
(
	input  wire                      clk_i,
	input  wire [ADDR_WIDTH-1:0]     addr_a_i,
	input  wire                      wren_a_i,
	input  wire [(DATA_WIDTH/8)-1:0] be_a_i,
	input  wire [DATA_WIDTH-1:0]     wdata_a_i,
	output wire [DATA_WIDTH-1:0]     q_a_o,
	input  wire [ADDR_WIDTH-1:0]     addr_b_i,
	input  wire                      wren_b_i,
	input  wire [(DATA_WIDTH/8)-1:0] be_b_i,
	input  wire [DATA_WIDTH-1:0]     wdata_b_i,
	output wire [DATA_WIDTH-1:0]     q_b_o
);
	localparam NUM_WORDS = (1 << ADDR_WIDTH);
	localparam NUM_BYTES = DATA_WIDTH / 8;

	`ifdef ALTERA_RESERVED_QIS
	altsyncram #(
		.address_reg_b                 ("CLOCK0"),
		.byte_size                     (8),
		.byteena_reg_b                 ("CLOCK0"),
		.clock_enable_output_a         ("BYPASS"),
		.clock_enable_output_b         ("BYPASS"),
		.indata_reg_b                  ("CLOCK0"),
		.init_file                     (MEM_INIT_FILE),
		.intended_device_family        (DEVICE_FAMILY),
		.lpm_hint                      ("ENABLE_RUNTIME_MOD=NO"),
		.lpm_type                      ("altsyncram"),
		.numwords_a                    (NUM_WORDS),
		.numwords_b                    (NUM_WORDS),
		.operation_mode                ("BIDIR_DUAL_PORT"),
		.outdata_aclr_a                ("NONE"),
		.outdata_aclr_b                ("NONE"),
		.outdata_reg_a                 ("UNREGISTERED"),
		.outdata_reg_b                 ("UNREGISTERED"),
		.power_up_uninitialized        ("FALSE"),
		.ram_block_type                ("M10K"),
		.read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
		.read_during_write_mode_port_b ("NEW_DATA_NO_NBE_READ"),
		.width_a                       (DATA_WIDTH),
		.width_b                       (DATA_WIDTH),
		.width_byteena_a               (NUM_BYTES),
		.width_byteena_b               (NUM_BYTES),
		.widthad_a                     (ADDR_WIDTH),
		.widthad_b                     (ADDR_WIDTH),
		.wrcontrol_wraddress_reg_b     ("CLOCK0")
	) u_ram (
		.address_a (addr_a_i),
		.address_b (addr_b_i),
		.byteena_a (be_a_i),
		.byteena_b (be_b_i),
		.clock0    (clk_i),
		.data_a    (wdata_a_i),
		.data_b    (wdata_b_i),
		.q_a       (q_a_o),
		.q_b       (q_b_o),
		.wren_a    (wren_a_i),
		.wren_b    (wren_b_i)
	);

`else
	reg [DATA_WIDTH-1:0] q_a_out;
	reg [DATA_WIDTH-1:0] q_b_out;

	(* ramstyle = "M10K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem_q [0:NUM_WORDS-1];

	// Power-on contents -- see rtl/mem/cache_ram.v.
	integer init_i;

	initial begin
		if (SIM_INIT_FILE != " ") begin
			$readmemh(SIM_INIT_FILE, mem_q);
		end else begin
			for (init_i = 0; init_i < NUM_WORDS; init_i = init_i + 1) begin
				mem_q[init_i] = {DATA_WIDTH{1'b0}};
			end
		end
	end

	integer b;

	always @(posedge clk_i) begin
		for (b = 0; b < NUM_BYTES; b = b + 1) begin
			if (wren_a_i && be_a_i[b]) begin
				mem_q[addr_a_i][b*8 +: 8] <= wdata_a_i[b*8 +: 8];
			end
			if (wren_a_i && be_a_i[b]) begin
				q_a_out[b*8 +: 8] <= wdata_a_i[b*8 +: 8];
			end else begin
				q_a_out[b*8 +: 8] <= mem_q[addr_a_i][b*8 +: 8];
			end

			if (wren_b_i && be_b_i[b]) begin
				mem_q[addr_b_i][b*8 +: 8] <= wdata_b_i[b*8 +: 8];
			end
			if (wren_b_i && be_b_i[b]) begin
				q_b_out[b*8 +: 8] <= wdata_b_i[b*8 +: 8];
			end else begin
				q_b_out[b*8 +: 8] <= mem_q[addr_b_i][b*8 +: 8];
			end
		end
	end

	assign q_a_o = q_a_out;
	assign q_b_o = q_b_out;
`endif

endmodule
