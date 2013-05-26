`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/15/2013 06:36:32 PM
// Design Name: 
// Module Name: novena_fpga
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module novena_fpga(
		   output wire       APOPTOSIS,
		   
		   input wire AUD6_TFS,
		   input wire AUD6_TXC,
		   input wire AUD6_TXD,
		   input wire AUD_MCLK,
		   input wire AUD_MIC_CLK,
		   input wire AUD_MIC_DAT,
		   
		   input wire BATT_NRST,
		   input wire BATT_REFLASH_ALRT,
		   
		   input wire CLK2_N,
		   input wire CLK2_P,
		   
		   input wire DDC_SCL,
		   input wire DDC_SDA,
		   
		   output wire ECSPI3_MISO,
		   input wire ECSPI3_MOSI,
		   input wire ECSPI3_RDY,
		   input wire ECSPI3_SCLK,
		   input wire ECSPI3_SS2,
		   
		   input wire EIM_BCLK,
		   input wire [1:0] EIM_CS,
		   inout wire [15:0] EIM_DA,
		   input wire [18:16] EIM_A,
		   input wire EIM_LBA,
		   input wire EIM_OE,
		   input wire EIM_RW,
		   input wire EIM_WAIT,
		   
		   output wire FPGA_LED2,
		   input wire FPGA_LSPI_CLK,
		   input wire FPGA_LSPI_CS,
		   input wire FPGA_LSPI_HOLD,
		   input wire FPGA_LSPI_MISO,
		   input wire FPGA_LSPI_MOSI,
		   input wire FPGA_LSPI_WP,
		   
		   input wire I2C3_SCL,
		   input wire I2C3_SDA,
		   
		   input wire SMB_SCL,
		   input wire SMB_SDA,
		   
		   input wire UART4_CTS,
		   input wire UART4_RTS,
		   input wire UART4_RXD,
		   input wire UART4_TXD,
		   
		   // input wire UIM_CLK,
		   // input wire UIM_DATA,
		   // input wire UIM_PWR,
		   // input wire UIM_PWRON,
		   // input wire UIM_RESET,

//		   input wire F_DX[18:0],

//		   input wire F_LVDS_N[15:0],
//		   input wire F_LVDS_P[15:0],

//		   input wire F_LVDS_NA,
//		   input wire F_LVDS_PA,
//		   input wire F_LVDS_NB,
//		   input wire F_LVDS_PB,
//		   input wire F_LVDS_NC,
//		   input wire F_LVDS_PC,

//		   input wire F_LVDS_CK_N[1:0],
//		   input wire F_LVDS_CK_P[1:0],

		   inout wire [15:0] F_DDR3_D,
		   inout wire F_UDQS_N,
		   inout wire F_UDQS_P,
		   inout wire F_LDQS_N,
		   inout wire F_LDQS_P,
		   output wire F_UDM,
		   output wire F_LDM,
		   
		   output wire [2:0] F_BA,
		   output wire F_CAS_N,
		   output wire [13:0] F_DDR3_A,
		   output wire F_DDR3_CKE,
		   output wire F_DDR3_CK_N,
		   output wire F_DDR3_CK_P,
		   output wire F_DDR3_ODT,
		   output wire F_RAS_N,
		   output wire F_WE_N,
		   inout wire F_DDR3_RZQ,
		   inout wire F_DDR3_ZIO,
		   output wire F_DDR3_RST_N,

		   input wire RESETBMCU
	 );

   wire [15:0] 		      eim_dout;
   wire [15:0] 		      eim_din;
   wire 		      clk;   // free-runs at 50 MHz, unbuffered
   wire 		      clk50; // zero-delay, DLL version of above. Use this.
   wire 		      bclk;  // NOTE: doesn't run until first CPU access to EIM; then free-runs at 133 MHz
   reg [23:0] 		      counter;
   wire 		      eim_d_t;
   
   wire 		      ddr3_dll_locked;
   wire 		      ddr3clk;
   
   wire 		      ddr3_calib_done; // to mcb

   wire 		      ddr3_p2_cmd_en;
   wire [2:0] 		      ddr3_p2_cmd_instr;
   wire [5:0] 		      ddr3_p2_cmd_bl;
   wire [29:0] 		      ddr3_p2_cmd_byte_addr;
   wire 		      ddr3_p2_cmd_empty; // from mcb
   wire 		      ddr3_p2_cmd_full;

   wire 		      ddr3_p2_wr_en; // to mcb
   wire [3:0] 		      ddr3_p2_wr_mask;
   wire [31:0] 		      ddr3_p2_wr_data;
   wire 		      ddr3_p2_wr_full; // from mcb
   wire 		      ddr3_p2_wr_empty;
   wire [6:0] 		      ddr3_p2_wr_count;
   wire 		      ddr3_p2_wr_underrun;
   wire 		      ddr3_p2_wr_error;

   wire 		      ddr3_p3_cmd_en; // to mcb
   wire [2:0] 		      ddr3_p3_cmd_instr;
   wire [5:0] 		      ddr3_p3_cmd_bl;
   wire [29:0] 		      ddr3_p3_cmd_byte_addr;
   wire 		      ddr3_p3_cmd_empty; // from mcb
   wire 		      ddr3_p3_cmd_full;

   wire 		      ddr3_p3_rd_en; // to mcb
   wire [31:0] 		      ddr3_p3_rd_data; // from mcb
   wire 		      ddr3_p3_rd_full;
   wire 		      ddr3_p3_rd_empty;
   wire [6:0] 		      ddr3_p3_rd_count;
   wire 		      ddr3_p3_rd_overflow;
   wire 		      ddr3_p3_rd_error;
   
   
   wire 		      reset;

   ////////////////////////////////////
   ///// MASTER RESET
   ////////////////////////////////////
   
   sync_reset master_res_sync( .glbl_reset(!RESETBMCU), .clk(clk), .reset(reset) );
     
   wire 	      bclk_dll, bclk_div2_dll, bclk_div4_dll, bclk_locked;
   
   ////////////////////////////////////
   ///// BCLK DLL -- generate zero-delay clock plus slower versions for internal use
   ////////////////////////////////////
   
   bclk_dll bclk_dll_mod( .CLK_IN1(bclk), .CLK_OUT1(bclk_dll), .CLK_OUT2(bclk_div2_dll),
			  .CLK_OUT3(bclk_div4_dll), .RESET(reset), .LOCKED(bclk_locked));

   ////////////////////////////////////
   ///// Block RAM section -- an area-efficient piece of memory we can write to and store data from the CPU
   ////////////////////////////////////
   
   wire [15:0]	      bram_dout;
   
   novena_eim novena_eim (
	      .din(eim_din),
	      .dout(bram_dout),
	      .clk(clk),
	      .reset(reset),

	      .bclk(bclk_dll),
	      .cs(EIM_CS),
	      .hi_addr(EIM_A),
	      .lba(EIM_LBA),
	      .oe(EIM_OE),
	      .rw(EIM_RW),
	      .rb_wait(EIM_WAIT)
			  );

   ////////////////////////////////////
   ///// Register set -- area-inefficient, high fan-out/in registers for controlling/monitoring internal signals
   ///// All registers split into write or read only blanks
   ///// 0x40000 - 0x40FFF is reserved for w/o
   ///// 0x41000 - 0x41FFF is reserved for r/o
   /////   -> if you want to check a w/o value, loop it back to an r/o register
   ////////////////////////////////////
   
   reg 		      cs0_r, rw_r;
   reg [15:0] 	      din_r;
   reg [18:0] 	      bus_addr;
   always @(posedge bclk_dll) begin
      cs0_r <= EIM_CS[0];
      rw_r <= EIM_RW;
      din_r <= eim_din;
   end
   
   always @(posedge bclk_dll) begin
      if( !EIM_LBA ) begin // latch address on LBA low
	 bus_addr <= {EIM_A, eim_din};
      end else begin
	 bus_addr <= bus_addr;
      end
   end

   wire [15:0] r40000wo;
   wire [15:0] r40002wo;

   wire [15:0] ro_d;

   //////// write-only registers
   reg_wo reg_wo_40000 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40000),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( r40000wo[15:0] ) );
   
   reg_wo reg_wo_40002 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40002),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( r40002wo[15:0] ) );

   // write control for p2
   // check alignment of verilog when port specs are too short
   reg_wo reg_wo_40020 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40020),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( {ddr3_p2_cmd_bl[5:0], 
				  ddr3_p2_cmd_en, ddr3_p2_cmd_instr[2:0] } ) );

   reg_wo reg_wo_40022 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40022),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( ddr3_p2_cmd_byte_addr[15:0] ) );

   reg_wo reg_wo_40024 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40024),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( ddr3_p2_cmd_byte_addr[29:16] ) );

   reg_wo reg_wo_40026 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40026),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( {ddr3_p2_wr_en, ddr3_p2_wr_mask[3:0]} ) );

   reg_wo reg_wo_40028 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40028),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( ddr3_p2_wr_data[15:0] ) );

   reg_wo reg_wo_4002A ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h4002A),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( ddr3_p2_wr_data[31:16] ) );
   
   // read control for p3
   reg_wo reg_wo_40030 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40030),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( {ddr3_p3_cmd_bl[5:0], 
				  ddr3_p3_cmd_en, ddr3_p3_cmd_instr[2:0] } ) );

   reg_wo reg_wo_40032 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40032),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( ddr3_p3_cmd_byte_addr[15:0] ) );

   reg_wo reg_wo_40034 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40034),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( ddr3_p3_cmd_byte_addr[29:16] ) );


   wire [3:0]        ddr3_p3_dummy;
   reg_wo reg_wo_40036 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h40036),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), 
			 .reg_d( {ddr3_p3_rd_en, ddr3_p3_dummy[3:0]} ) );
			 
   //////// read-only registers
   // loopback readback
   reg_ro reg_ro_41000 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41000),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( r40000wo[15:0] ) );

   reg_ro reg_ro_41002 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41002),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( r40002wo[15:0] ) );

   // general status register
   // 0    : DDR3 DLL lock
   // 1    : DDR3 calibration done
   // 15-2 : reads as 0
   reg_ro reg_ro_41004 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41004),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_calib_done, ddr3_dll_locked} ) );

   /////// ddr p2 write status
   reg_ro reg_ro_41020 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41020),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_p2_wr_count[6:0],
				  2'b00,
				  ddr3_p2_cmd_empty, ddr3_p2_cmd_full,
				  ddr3_p2_wr_full, ddr3_p2_wr_empty, 
				  ddr3_p2_wr_underrun, ddr3_p2_wr_error} ) );

   /////// ddr p3 read status & data
   reg_ro reg_ro_41030 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41030),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_p3_rd_count[6:0],
				  2'b00,
				  ddr3_p3_cmd_empty, ddr3_p3_cmd_full,
				  ddr3_p3_rd_full, ddr3_p3_rd_empty, 
				  ddr3_p3_rd_overflow, ddr3_p3_rd_error} ) );

   reg_ro reg_ro_41032 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41032),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( ddr3_p3_rd_data[15:0] ) );
   
   reg_ro reg_ro_41034 ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41034),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( ddr3_p3_rd_data[31:16] ) );
   
   
   // FPGA minor version code
   reg_ro reg_ro_41FFC ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41FFC),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( 16'h0001 ) );

   // FPGA major version code
   reg_ro reg_ro_41FFE ( .clk(bclk_dll), .bus_a(bus_addr), .my_a(19'h41FFE),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( 16'h0001 ) );

   // mux between block memory and register set based on high bits
   assign eim_dout = (bus_addr[18:16] != 3'b000) ? ro_d : bram_dout;

   //////////////
   /// "heartbeat" counter
   //////////////
   always @(posedge clk) begin
      counter <= counter + 1;
   end

   assign FPGA_LED2 = counter[23];

   //////////////
   // IOBUFs as required by design
   //////////////
   IBUFGDS clkibufgds( .I(CLK2_P), .IB(CLK2_N), .O(clk) );
   IBUFG   clkibufg (.I(EIM_BCLK), .O(bclk) );

   assign eim_d_t = EIM_OE | !EIM_LBA;
   
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim0 (.IO(EIM_DA[ 0]), .I(eim_dout[ 0]), .T(eim_d_t), .O(eim_din[ 0]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim1 (.IO(EIM_DA[ 1]), .I(eim_dout[ 1]), .T(eim_d_t), .O(eim_din[ 1]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim2 (.IO(EIM_DA[ 2]), .I(eim_dout[ 2]), .T(eim_d_t), .O(eim_din[ 2]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim3 (.IO(EIM_DA[ 3]), .I(eim_dout[ 3]), .T(eim_d_t), .O(eim_din[ 3]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim4 (.IO(EIM_DA[ 4]), .I(eim_dout[ 4]), .T(eim_d_t), .O(eim_din[ 4]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim5 (.IO(EIM_DA[ 5]), .I(eim_dout[ 5]), .T(eim_d_t), .O(eim_din[ 5]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim6 (.IO(EIM_DA[ 6]), .I(eim_dout[ 6]), .T(eim_d_t), .O(eim_din[ 6]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim7 (.IO(EIM_DA[ 7]), .I(eim_dout[ 7]), .T(eim_d_t), .O(eim_din[ 7]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim8 (.IO(EIM_DA[ 8]), .I(eim_dout[ 8]), .T(eim_d_t), .O(eim_din[ 8]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim9 (.IO(EIM_DA[ 9]), .I(eim_dout[ 9]), .T(eim_d_t), .O(eim_din[ 9]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim10 (.IO(EIM_DA[10]), .I(eim_dout[10]), .T(eim_d_t), .O(eim_din[10]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim11 (.IO(EIM_DA[11]), .I(eim_dout[11]), .T(eim_d_t), .O(eim_din[11]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim12 (.IO(EIM_DA[12]), .I(eim_dout[12]), .T(eim_d_t), .O(eim_din[12]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim13 (.IO(EIM_DA[13]), .I(eim_dout[13]), .T(eim_d_t), .O(eim_din[13]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim14 (.IO(EIM_DA[14]), .I(eim_dout[14]), .T(eim_d_t), .O(eim_din[14]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim15 (.IO(EIM_DA[15]), .I(eim_dout[15]), .T(eim_d_t), .O(eim_din[15]));

   //////////////
   // DDR3 interface macro
   //////////////

   wire c1_clk0, c1_rst0;
   
   ddr3_clkgen ddr3_clkgen (
			    .clk50in(clk),
			    .clk50(clk50),
			    .clk400(ddr3clk),
			    .RESET(reset),
			    .LOCKED(ddr3_dll_locked)
			    );
   
   ddr3_if # (
	      .C1_P0_MASK_SIZE(4),
	      .C1_P0_DATA_PORT_SIZE(32),
	      .C1_P1_MASK_SIZE(4),
	      .C1_P1_DATA_PORT_SIZE(32),
	      .DEBUG_EN(0),
	      .C1_MEMCLK_PERIOD(2500),
	      .C1_CALIB_SOFT_IP("TRUE"),
	      .C1_SIMULATION("FALSE"),
	      .C1_RST_ACT_LOW(0),
	      .C1_INPUT_CLK_TYPE("DIFFERENTIAL"),
	      .C1_MEM_ADDR_ORDER("ROW_BANK_COLUMN"),
	      .C1_NUM_DQ_PINS(16),
	      .C1_MEM_ADDR_WIDTH(14),
	      .C1_MEM_BANKADDR_WIDTH(3)
	      )
   u_ddr3_if (

	      .c1_sys_clk             (ddr3clk),
	      .c1_sys_rst_i           (reset),
	      
	      .mcb1_dram_dq           (F_DDR3_D[15:0]),  
	      .mcb1_dram_a            (F_DDR3_A[13:0]),  
	      .mcb1_dram_ba           (F_BA[2:0]),
	      .mcb1_dram_ras_n        (F_RAS_N),                        
	      .mcb1_dram_cas_n        (F_CAS_N),                        
	      .mcb1_dram_we_n         (F_WE_N),                          
	      .mcb1_dram_odt          (F_DDR3_ODT),
	      .mcb1_dram_cke          (F_DDR3_CKE),                          
	      .mcb1_dram_ck           (F_DDR3_CK_P),                          
	      .mcb1_dram_ck_n         (F_DDR3_CK_N),       
	      .mcb1_dram_dqs          (F_LDQS_P),                          
	      .mcb1_dram_dqs_n        (F_LDQS_N),
	      .mcb1_dram_udqs         (F_UDQS_P),    // for X16 parts   
	      .mcb1_dram_udqs_n       (F_UDQS_N),  // for X16 parts
	      .mcb1_dram_udm          (F_UDM),     // for X16 parts
	      .mcb1_dram_dm           (F_LDM),
	      .mcb1_dram_reset_n      (F_DDR3_RST_N),
	      .c1_clk0		        (c1_clk0),
	      .c1_rst0		        (c1_rst0),
	
	      .c1_calib_done    (ddr3_calib_done),
	      .mcb1_rzq               (F_DDR3_RZQ),  
	      .mcb1_zio               (F_DDR3_ZIO),
	
	      .c1_p2_cmd_clk                          (bclk_dll),
	      .c1_p2_cmd_en                           (ddr3_p2_cmd_en),
	      .c1_p2_cmd_instr                        (ddr3_p2_cmd_instr),
	      .c1_p2_cmd_bl                           (ddr3_p2_cmd_bl),
	      .c1_p2_cmd_byte_addr                    (ddr3_p2_cmd_byte_addr),
	      .c1_p2_cmd_empty                        (ddr3_p2_cmd_empty),
	      .c1_p2_cmd_full                         (ddr3_p2_cmd_full),
	      .c1_p2_wr_clk                           (bclk_dll),
	      .c1_p2_wr_en                            (ddr3_p2_wr_en),
	      .c1_p2_wr_mask                          (ddr3_p2_wr_mask),
	      .c1_p2_wr_data                          (ddr3_p2_wr_data),
	      .c1_p2_wr_full                          (ddr3_p2_wr_full),
	      .c1_p2_wr_empty                         (ddr3_p2_wr_empty),
	      .c1_p2_wr_count                         (ddr3_p2_wr_count),
	      .c1_p2_wr_underrun                      (ddr3_p2_wr_underrun),
	      .c1_p2_wr_error                         (ddr3_p2_wr_error),
	      .c1_p3_cmd_clk                          (bclk_dll),
	      .c1_p3_cmd_en                           (ddr3_p3_cmd_en),
	      .c1_p3_cmd_instr                        (ddr3_p3_cmd_instr),
	      .c1_p3_cmd_bl                           (ddr3_p3_cmd_bl),
	      .c1_p3_cmd_byte_addr                    (ddr3_p3_cmd_byte_addr),
	      .c1_p3_cmd_empty                        (ddr3_p3_cmd_empty),
	      .c1_p3_cmd_full                         (ddr3_p3_cmd_full),
	      .c1_p3_rd_clk                           (bclk_dll),
	      .c1_p3_rd_en                            (ddr3_p3_rd_en),
	      .c1_p3_rd_data                          (ddr3_p3_rd_data),
	      .c1_p3_rd_full                          (ddr3_p3_rd_full),
	      .c1_p3_rd_empty                         (ddr3_p3_rd_empty),
	      .c1_p3_rd_count                         (ddr3_p3_rd_count),
	      .c1_p3_rd_overflow                      (ddr3_p3_rd_overflow),
	      .c1_p3_rd_error                         (ddr3_p3_rd_error)
	      );
   
   //////////////
   // tie downs (unused signals as of this rev of design)
   //////////////
   assign APOPTOSIS = 1'b0; // make apoptosis inactive, tigh high to force reboot on config
   assign ECSPI3_MISO = 1'b0;
   
endmodule
