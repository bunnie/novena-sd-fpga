//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2013, Andrew "bunnie" Huang
//
// See the NOTICE file distributed with this work for additional 
// information regarding copyright ownership.  The copyright holder 
// licenses this file to you under the Apache License, Version 2.0 
// (the "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// code distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.
//////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

`define USE_ROMULATOR 1
`define USE_NANDLOG 1

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

		   inout wire F_LVDS_N2,  // DAT2
		   inout wire F_DX14,     // DAT1
		   inout wire F_LVDS_P4,  // DAT0, DO
		   inout wire F_LVDS_N4,  // CLK, SCLK
		   inout wire F_LVDS_P1,  // CMD, DI
		   inout wire F_LVDS_N1,  // DAT3, CS

		   inout wire F_DX0,      // IO7
		   inout wire F_DX3,      // IO6
		   inout wire F_DX2,      // IO5
		   inout wire F_DX11,     // IO4
		   inout wire F_LVDS_N11, // IO3
		   inout wire F_DX1,      // IO2
		   inout wire F_LVDS_NC,  // IO1
		   inout wire F_LVDS_PC,  // IO0

		   inout wire F_LVDS_N0, // R/B
		   input wire  F_LVDS_P0,  // CS
		   input wire  F_LVDS_CK_P1, // RE
		   input wire  F_LVDS_P15, // CLE
		   input wire  F_LVDS_NB,  // ALE
		   input wire  F_LVDS_PB,  // WE
		   input wire  F_DX17,     // WP

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

   reg [15:0] 		      eim_dout;
   wire [15:0] 		      eim_din;
   wire 		      clk;   // free-runs at 50 MHz, unbuffered
   wire 		      clk50; // zero-delay, DLL version of above. Use this.
   wire 		      clk100; // doubled-up version of the above. For time base applications.
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
   wire 		      ddr3_p2_wr_pulse;
   wire 		      p2_wr_pulse_gate;

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
   wire 		      ddr3_p3_rd_pulse;
   wire 		      p3_rd_pulse_gate;
   
   
   wire 		      reset;


   wire [15:0] 		      gpioA_din;
   wire [15:0] 		      gpioA_dout;
   wire [15:0] 		      gpioA_dir;
   

   ////////////////////////////////////
   ///// MASTER RESET
   ////////////////////////////////////
   
   sync_reset master_res_sync( .glbl_reset(!RESETBMCU), .clk(clk), .reset(reset) );
     
   wire 	      bclk_dll, bclk_div2_dll, bclk_div4_dll, bclk_locked;
   wire 	      bclk_early;
   
   ////////////////////////////////////
   ///// BCLK DLL -- generate zero-delay clock plus slower versions for internal use
   ////////////////////////////////////
   wire 	      bclk_int_in, bclk_io_in;
   IBUFG   clkibufg (.I(EIM_BCLK), .O(bclk) );
   BUFG    bclk_dll_bufg(.I(bclk), .O(bclk_int_in) );
   
   bclk_dll bclk_dll_mod( .clk133in(bclk_int_in), .clk133(bclk_dll),
			  .RESET(reset), .LOCKED(bclk_locked));

   wire 	      i_reset, i_locked;
   wire 	      o_reset, o_locked;
   wire 	      bclk_i, bclk_o;
   wire 	      i_fbk_out, i_fbk_in;
   wire 	      o_fbk_out, o_fbk_in;
   
   dcm_delay bclk_i_dll( .clk133(bclk_int_in), .clk133out(bclk_i),
			  .CLKFB_IN(i_fbk_in), .CLKFB_OUT(i_fbk_out),
			  .RESET(i_reset), .LOCKED(i_locked));

   dcm_delay bclk_o_dll( .clk133(bclk_int_in), .clk133out(bclk_o),
			  .CLKFB_IN(o_fbk_in), .CLKFB_OUT(o_fbk_out),
			  .RESET(o_reset), .LOCKED(o_locked));
   
   // lock it to the input path
   BUFIO2FB bclk_o_fbk(.I(bclk_o), .O(o_fbk_in));
   // assign o_fbk_in = bclk_o;
//   BUFG bclk_io_fbk(.I(bclk_io), .O(io_fbk_in));
   
   assign i_fbk_in = bclk_i;
   

   ////////////////////////////////////
   ///// nand connections
   ////////////////////////////////////
   wire [7:0] 	      nand_din;
   wire [7:0] 	      nand_dout;
   wire 	      nand_drive_out;
   wire 	      nand_rb;
   wire 	      romulator_on;
   wire 	      nand_re, nand_re_ibufg;
   wire 	      nand_we, nand_we_ibufg;
   wire 	      nand_powered_on;

   wire 	      bypass_rb; // used to override RB for AX211 use case
   
   assign romulator_on = 1'b1;  // for now, jammed on; but later turn off for snooping modes
   
   assign nand_din = {F_DX0, F_DX3, F_DX2, F_DX11, F_LVDS_N11, F_DX1, F_LVDS_NC, F_LVDS_PC};
   assign F_LVDS_PC  = (nand_drive_out & romulator_on & nand_powered_on) ? nand_dout[0] : 1'bZ;
   assign F_LVDS_NC  = (nand_drive_out & romulator_on & nand_powered_on) ? nand_dout[1] : 1'bZ;
   assign F_DX1      = (nand_drive_out & romulator_on & nand_powered_on) ? nand_dout[2] : 1'bZ;
   assign F_LVDS_N11 = (nand_drive_out & romulator_on & nand_powered_on) ? nand_dout[3] : 1'bZ;
   assign F_DX11     = (nand_drive_out & romulator_on & nand_powered_on) ? nand_dout[4] : 1'bZ;
   assign F_DX2      = (nand_drive_out & romulator_on & nand_powered_on) ? nand_dout[5] : 1'bZ;
   assign F_DX3      = (nand_drive_out & romulator_on & nand_powered_on) ? nand_dout[6] : 1'bZ;
   assign F_DX0      = (nand_drive_out & romulator_on & nand_powered_on) ? nand_dout[7] : 1'bZ;

   reg 		      nand_rb_r;
   always @(posedge bclk_dll) begin
      nand_rb_r <= nand_rb; // just to make the timing engine happier, no material effect on design
   end
   assign F_LVDS_N0  = romulator_on & nand_powered_on ? (nand_rb_r | bypass_rb) : 1'bZ;
      
   // nand_re, nand_we are edge signals, so use a BUFG to distribute as clock
   IBUFG nand_we_ibufgp(.I(F_LVDS_PB), .O(nand_we_ibufg) );
   BUFG  nand_we_bufgp(.I(nand_we_ibufg), .O(nand_we) );
   IBUFG nand_re_ibufgp(.I(F_LVDS_CK_P1), .O(nand_re_ibufg) );
   BUFG  nand_re_bufgp(.I(nand_re_ibufg), .O(nand_re) );
   
   wire [7:0] 	      nand_uk_cmd;
   wire 	      nand_uk_updated;
   wire [7:0] 	      nand_known_cmd;
   wire 	      nand_cmd_updated;
   wire [29:0] 	      nand_adr;
   wire 	      nand_adr_updated;
   wire 	      nand_cs;
   assign nand_cs = F_LVDS_P0;

   ////////////////////////////////////
   ///// romulator, DDR3 version
   ////////////////////////////////////
   wire 	      rom_ddr3_reset;
   wire 	      page_addra_over;
   wire 	      outstanding_under;
   
   wire 	      ddr3_wr_clk;
   wire 	      ddr3_wr_cmd_en;
   wire [2:0] 	      ddr3_wr_cmd_instr;
   wire [5:0] 	      ddr3_wr_cmd_bl;
   wire [29:0] 	      ddr3_wr_adr;
   wire 	      ddr3_wr_cmd_full;
   wire 	      ddr3_wr_cmd_empty;
   wire 	      ddr3_wr_dat_en;
   wire [31:0] 	      ddr3_wr_dat;
   wire 	      ddr3_wr_full;
   wire 	      ddr3_wr_empty;
   wire [6:0] 	      ddr3_wr_dat_count;
   wire [3:0] 	      ddr3_wr_mask;
					
   wire 	      ddr3_rd_clk;
   wire 	      ddr3_rd_cmd_en;
   wire [2:0] 	      ddr3_rd_cmd_instr;
   wire [5:0] 	      ddr3_rd_cmd_bl;
   wire [29:0] 	      ddr3_rd_adr;
   wire 	      ddr3_rd_cmd_full;
   wire 	      ddr3_rd_dat_en;
   wire [31:0] 	      ddr3_rd_dat;
   wire 	      ddr3_rd_dat_empty;
   wire [6:0] 	      ddr3_rd_dat_count;
   wire 	      ddr3_rd_dat_full;
   wire 	      ddr3_rd_dat_overflow; // need to monitor this
   wire [2:0] 	      ddr_cstate; // debug output
`ifdef USE_ROMULATOR
   romulator_ddr3 romulator_ddr3(
				 .clk(bclk_dll),  // 133 MHz
		       
				 .nand_we(nand_we),
				 .nand_re(nand_re),
				 .nand_cs(nand_cs),
				 .nand_ale(F_LVDS_NB),
				 .nand_cle(F_LVDS_P15),
				 .nand_rb(nand_rb),
				 .nand_wp(F_DX17),

				 .nand_din(nand_din),
				 .nand_dout(nand_dout),
				 .nand_drive_out(nand_drive_out),

				 .rom_ddr3_reset(rom_ddr3_reset),
		      
				 .ddr3_wr_clk(ddr3_wr_clk),
				 .ddr3_wr_cmd_en(ddr3_wr_cmd_en),
				 .ddr3_wr_cmd_instr(ddr3_wr_cmd_instr[2:0]),
				 .ddr3_wr_cmd_bl(ddr3_wr_cmd_bl[5:0]),
				 .ddr3_wr_adr(ddr3_wr_adr[29:0]),
				 .ddr3_wr_cmd_full(ddr3_wr_cmd_full),
				 .ddr3_wr_cmd_empty(ddr3_wr_cmd_empty),
				 .ddr3_wr_dat_en(ddr3_wr_dat_en),
				 .ddr3_wr_dat(ddr3_wr_dat[31:0]),
				 .ddr3_wr_full(ddr3_wr_full),
				 .ddr3_wr_empty(ddr3_wr_empty),
				 .ddr3_wr_mask(ddr3_wr_mask[3:0]),
		      
				 .ddr3_rd_clk(ddr3_rd_clk),
				 .ddr3_rd_cmd_en(ddr3_rd_cmd_en),
				 .ddr3_rd_cmd_instr(ddr3_rd_cmd_instr[2:0]),
				 .ddr3_rd_cmd_bl(ddr3_rd_cmd_bl[5:0]),
				 .ddr3_rd_adr(ddr3_rd_adr[29:0]),
				 .ddr3_rd_cmd_full(ddr3_rd_cmd_full),
				 .ddr3_rd_dat_en(ddr3_rd_dat_en),
				 .ddr3_rd_dat(ddr3_rd_dat[31:0]),
				 .ddr3_rd_dat_empty(ddr3_rd_dat_empty),
				 .ddr3_rd_dat_count(ddr3_rd_dat_count[6:0]),
				 .ddr3_rd_dat_full(ddr3_rd_dat_full),
				 .ddr3_rd_dat_overflow(ddr3_rd_dat_overflow),

				 .page_addra_over(page_addra_over),
				 .outstanding_under(outstanding_under),

				 .nand_uk_cmd(nand_uk_cmd),
				 .nand_uk_cmd_updated(nand_uk_updated),
				 
				 .nand_known_cmd(nand_known_cmd),
				 .nand_cmd_updated(nand_cmd_updated),

				 .nand_adr(nand_adr),
				 .nand_adr_updated(nand_adr_updated),

				 .ddr_cstate_dbg(ddr_cstate),
		       
				 .reset(reset)
		       );
`endif
   reg 		      page_addra_over_caught;
   reg 		      outstanding_under_caught;
   always @(posedge bclk_dll) begin
      if(rom_ddr3_reset) begin
	 page_addra_over_caught <= 1'b0;
	 outstanding_under_caught <= 1'b0;
      end else begin
	 if( page_addra_over ) begin
	    page_addra_over_caught <= 1'b1;
	 end else begin
	    page_addra_over_caught <= page_addra_over_caught;
	 end

	 if( outstanding_under ) begin
	    outstanding_under_caught <= 1'b1;
	 end else begin
	    outstanding_under_caught <= outstanding_under_caught;
	 end
      end // else: !if(rom_ddr3_reset)
   end // always @ (posedge bclk_dll)


   ////////////////////////////////////
   ///// nand flash logger
   ////////////////////////////////////
   wire [3:0] 	      log_wr_mask;
   wire [31:0] 	      log_wr_data;
   wire 	      log_wr_en;
   wire [6:0] 	      log_wr_count;
   
   wire 	      log_cmd_clk;
   wire [2:0] 	      log_cmd_instr;
   wire 	      log_cmd_en;
   wire [5:0] 	      log_cmd_burstlen;
   wire [29:0] 	      log_cmd_addr;
   wire 	      log_cmd_full;

   wire [2:0] 	      logbuf_cmd_instr;
   wire 	      logbuf_cmd_en;
   wire [5:0] 	      logbuf_cmd_burstlen;
   wire [29:0] 	      logbuf_cmd_addr;
   wire 	      logbuf_cmd_full;
   wire 	      logbuf_empty;
   
   wire [63:0] 	      time_t_clk100;

   wire 	      log_reset;
   wire 	      log_run;
   wire 	      log_cmd_error;
   wire 	      log_data_error;
   wire [26:0]	      log_entries;
   reg [63:0] 	      time_t_bclk;
   wire 	      time_t_update;

   assign logbuf_cmd_full = ddr3_p2_cmd_full;
`ifdef USE_NANDLOG   
   nand_log nand_log(
		     .bclk(bclk_dll),
		     .clk100(clk100),

		     .nand_re(nand_re),
		     .nand_we(nand_we),
		     .nand_cs(nand_cs),
		     .nand_ale(F_LVDS_NB),
		     .nand_cle(F_LVDS_P15),
		     .nand_rb(F_LVDS_N0),
		     .nand_din(nand_din),
		     .nand_uk(10'b0),

		     .log_reset(log_reset),
		     .log_run(log_run),
		     .log_cmd_error(log_cmd_error),
		     .log_data_error(log_data_error),
		     .log_entries(log_entries),

		     .ddr3_wr_mask(log_wr_mask),
		     .ddr3_wr_data(log_wr_data),
		     .ddr3_wr_en(log_wr_en),
		     .ddr3_wr_count(ddr3_p2_wr_count),
		     .ddr3_wr_full(ddr3_p2_wr_full),
		     .ddr3_wr_empty(ddr3_p2_wr_empty),
		     .ddr3_cmd_clk(log_cmd_clk),
		     .ddr3_cmd_instr(log_cmd_instr),
		     .ddr3_cmd_en(log_cmd_en),
		     .ddr3_cmd_burstlen(log_cmd_burstlen),
		     .ddr3_cmd_addr(log_cmd_addr),
		     .ddr3_cmd_full(ddr3_p2_cmd_full),
		     .ddr3_cmd_empty(ddr2_p2_cmd_empty),

		     .time_t_clk100(time_t_clk100),
		     .reset(reset)
		     );
`endif
   assign log_wr_count = ddr3_p2_wr_count;

   // pull time_t into the bclk domain
   always @(posedge bclk_dll) begin
      if( time_t_update ) begin
	 // a bit ugly, because this is a totally asynchronous pull of multiple bits....
	 // but, I think for this application +/- a few hundred ns is okay because we're using
	 // the timestamp just to place the SD command cycles within the stream, which themselves
	 // are being bit-banged and thus take microseconds to issue
	 time_t_bclk <= time_t_clk100;
      end else begin
	 time_t_bclk <= time_t_bclk;
      end
   end // always @ (posedge bclk)

   wire log_cmd_overflow, log_cmd_underflow;
   reg 	log_cmd_overflowed, log_cmd_underflowed;
   wire [4:0] log_cmd_data_count;
   reg [4:0]  log_cmd_peak_data_count;
   reg [6:0]  log_wr_peak_count;
   /////// command FIFO extension -- add some buffering to the command fifo:
   // the default IP is too shallow
   cmd_fifo_exp cmd_fifo_exp(
			     .clk(log_cmd_clk),
			     .srst(log_reset),
			     .din({log_cmd_addr[29:0], log_cmd_burstlen[5:0], log_cmd_instr[2:0]}),
			     .wr_en(log_cmd_en),
			     .rd_en(!logbuf_empty && !logbuf_cmd_full),
			     .dout({logbuf_cmd_addr[29:0], logbuf_cmd_burstlen[5:0], logbuf_cmd_instr[2:0]}),
			     .full(log_cmd_full),
			     .overflow(log_cmd_overflow),
			     .empty(logbuf_empty),
			     .underflow(log_cmd_underflow),
			     .data_count(log_cmd_data_count[4:0])
			     );
   // some bookeeeping to make sure we don't miss an error event
   always @(posedge log_cmd_clk) begin
      if( log_reset ) begin
	 log_cmd_overflowed <= 1'b0;
	 log_cmd_underflowed <= 1'b0;
	 log_cmd_peak_data_count <= 5'b0;
	 log_wr_peak_count <= 7'b0;
      end else begin
	 if( log_cmd_overflow ) begin
	    log_cmd_overflowed <= 1'b1;
	 end else begin
	    log_cmd_overflowed <= log_cmd_overflowed;
	 end

	 if( log_cmd_underflow ) begin
	    log_cmd_underflowed <= 1'b1;
	 end else begin
	    log_cmd_underflowed <= log_cmd_underflowed;
	 end

	 if( log_cmd_data_count > log_cmd_peak_data_count ) begin
	    log_cmd_peak_data_count <= log_cmd_data_count;
	 end else begin
	    log_cmd_peak_data_count <= log_cmd_peak_data_count;
	 end

	 if( log_wr_count > log_wr_peak_count ) begin
	    log_wr_peak_count <= log_wr_count;
	 end else begin
	    log_wr_peak_count <= log_wr_peak_count;
	 end
      end // else: !if( log_reset )
   end // always @ (posedge log_cmd_clk)
      

   ////////////////////////////////////
   ///// address FIFO -- log what addresses are asked of the ROM
   ////////////////////////////////////
   wire nand_adr_updated_pulse;
   reg 	nadr_up_d;
   wire adrfifo_full, adrfifo_over, adrfifo_empty, adrfifo_rst;
   wire adrfifo_rd_pulse;
   wire [13:0] adrfifo_count;
   wire [29:0]  adrfifo_data;
   wire 	log_adr_end;
   reg 		nand_cs_clean;
   reg 		nand_adr_up_clean;
   
   always @(posedge bclk_dll) begin
      // clean up and bring to clock domain before consuming
      nand_cs_clean <= nand_cs;
      nand_adr_up_clean <= nand_adr_updated;
      
      if( log_adr_end ) begin
	 nadr_up_d <= nand_adr_up_clean & !nand_cs_clean;
      end else begin
	 nadr_up_d <= nand_adr_up_clean;
      end
   end
   assign nand_adr_updated_pulse = log_adr_end ? (!nadr_up_d & (nand_adr_up_clean & !nand_cs_clean)) | 
				   (nadr_up_d & !(nand_adr_up_clean & !nand_cs_clean)) :
				   (!nadr_up_d & nand_adr_up_clean);

   reg [29:0] nand_adr_pipe;
   reg 	      nand_adr_updated_pulse_pipe;
   always @(posedge bclk_dll) begin
      nand_adr_pipe <= nand_adr;
      nand_adr_updated_pulse_pipe <= nand_adr_updated_pulse;
   end
   
   nandadr_fifo nandadr_fifo(
		   .rst(adrfifo_rst),
		   .wr_clk(bclk_dll),
		   .rd_clk(bclk_dll),
		   .din(nand_adr_pipe[29:0]),
		   .wr_en(nand_adr_updated_pulse_pipe),
		   .rd_en(adrfifo_rd_pulse),
		   .dout(adrfifo_data[29:0]),
		   .full(adrfifo_full),
		   .overflow(adrfifo_over),
		   .empty(adrfifo_empty),
		   .rd_data_count(adrfifo_count[13:0])
		   );


   ////////////////////////////////////
   ///// known romulator commands FIFO -- log what commands were asked of me
   ////////////////////////////////////
   wire nand_cmd_updated_pulse;
   reg 	ncmd_up_d;
   wire cmdfifo_full, cmdfifo_over, cmdfifo_empty, cmdfifo_rst;
   wire cmdfifo_rd_pulse;
   wire [11:0] cmdfifo_count;
   wire [7:0]  cmdfifo_data;
   
   always @(posedge bclk_dll) begin
      ncmd_up_d <= nand_cmd_updated;
   end
   assign nand_cmd_updated_pulse = !ncmd_up_d && nand_cmd_updated;
   
   uk_fifo cmd_fifo(
		   .rst(cmdfifo_rst),
		   .wr_clk(bclk_dll),
		   .rd_clk(bclk_dll),
		   .din(nand_known_cmd[7:0]),
		   .wr_en(nand_cmd_updated_pulse),
		   .rd_en(cmdfifo_rd_pulse),
		   .dout(cmdfifo_data[7:0]),
		   .full(cmdfifo_full),
		   .overflow(cmdfifo_over),
		   .empty(cmdfifo_empty),
		   .rd_data_count(cmdfifo_count[11:0])
		   );
      

   ////////////////////////////////////
   ///// Unknown romulator commands FIFO -- track errors and what we left on the floor
   ////////////////////////////////////
   wire nand_uk_updated_pulse;
   reg 	nuk_up_d;
   wire ukfifo_full, ukfifo_over, ukfifo_empty, ukfifo_rst;
   wire ukfifo_rd_pulse;
   wire [11:0] ukfifo_count;
   wire [7:0]  ukfifo_data;

   always @(posedge bclk_dll) begin
      nuk_up_d <= nand_uk_updated;
   end
   assign nand_uk_updated_pulse = !nuk_up_d && nand_uk_updated;
   
   uk_fifo uk_fifo(
		   .rst(ukfifo_rst),
		   .wr_clk(bclk_dll),
		   .rd_clk(bclk_dll),
		   .din(nand_uk_cmd[7:0]),
		   .wr_en(nand_uk_updated_pulse),
		   .rd_en(ukfifo_rd_pulse),
		   .dout(ukfifo_data[7:0]),
		   .full(ukfifo_full),
		   .overflow(ukfifo_over),
		   .empty(ukfifo_empty),
		   .rd_data_count(ukfifo_count[11:0])
		   );
      
   ////////////////////////////////////
   ///// GPIO pins
   ////////////////////////////////////
   assign gpioA_din[0] = F_LVDS_P4; // d0, do
   assign gpioA_din[1] = F_DX14;    // d1
   assign gpioA_din[2] = F_LVDS_N2; // d2
   assign gpioA_din[3] = F_LVDS_N1; // d3, cs
   assign gpioA_din[4] = F_LVDS_N4; // clk, sclk
   assign gpioA_din[5] = F_LVDS_P1; // cmd, di

   assign F_LVDS_P4 = gpioA_dir[0] & nand_powered_on ? gpioA_dout[0] : 1'bZ;
   assign F_DX14    = gpioA_dir[1] & nand_powered_on ? gpioA_dout[1] : 1'bZ;
   assign F_LVDS_N2 = gpioA_dir[2] & nand_powered_on ? gpioA_dout[2] : 1'bZ;
   assign F_LVDS_N1 = gpioA_dir[3] & nand_powered_on ? gpioA_dout[3] : 1'bZ;
   assign F_LVDS_N4 = gpioA_dir[4] & nand_powered_on ? gpioA_dout[4] : 1'bZ;
   assign F_LVDS_P1 = gpioA_dir[5] & nand_powered_on ? gpioA_dout[5] : 1'bZ;
   

   ////////////////////////////////////
   ///// Register set -- area-inefficient, high fan-out/in registers for controlling/monitoring internal signals
   ///// All registers split into write or read only blanks
   ///// 0x40000 - 0x40FFF is reserved for w/o
   ///// 0x41000 - 0x41FFF is reserved for r/o
   /////   -> if you want to check a w/o value, loop it back to an r/o register
   ////////////////////////////////////
   
   reg 		      cs0_r, rw_r;
   reg [15:0] 	      din_r;
   reg [18:0] 	      bus_addr_r;
   reg 		      adv_r;

   reg 		      cs0_in, rw_in, adv_in;
   reg [15:0] 	      din_in;
   reg [2:0] 	      a_in;
   
   always @(posedge bclk_i) begin
      cs0_in <= EIM_CS[0];
      rw_in <= EIM_RW;
      din_in <= eim_din;
      adv_in <= !EIM_LBA; // latch address on LBA low
      a_in <= EIM_A[18:16];

      cs0_r <= cs0_in;
      rw_r <= rw_in;
      din_r <= din_in;
      adv_r <= adv_in;
   end
   
   always @(posedge bclk_i) begin 
      if( adv_in ) begin
	 bus_addr_r <= {a_in, din_in};
      end else begin
	 bus_addr_r <= bus_addr_r;
      end
   end

   wire [15:0] r40000wo;
   wire [15:0] r40002wo;

   wire [15:0] ro_d;

   //////// write-only registers
   reg_wo reg_wo_40000 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40000),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( r40000wo[15:0] ) );
   
   reg_wo reg_wo_40002 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40002),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(1'b0), .rbk_d(ro_d), // unreadable
			 .reg_d( r40002wo[15:0] ) );

   ///// GPIO registers
   ///// NOTE: GPIOA is also gated by nand_powered_on in this application!!!!
   reg_wo reg_wo_40010 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40010),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( gpioA_dout[15:0] ) );

   reg_wo reg_wo_40012 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40012),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( gpioA_dir[15:0] ) );
			 
   // write control for p2
   // check alignment of verilog when port specs are too short
   wire [1:0]  dummy_40020;
   reg_wo reg_wo_40020 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40020),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {p2_wr_pulse_gate, dummy_40020[1:0], ddr3_p2_cmd_bl[5:0], 
				  ddr3_p2_cmd_en, ddr3_p2_cmd_instr[2:0] } ) );

   reg_wo reg_wo_40022 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40022),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p2_cmd_byte_addr[15:0] ) );

   reg_wo reg_wo_40024 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40024),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p2_cmd_byte_addr[29:16] ) );

   reg_wo reg_wo_40026 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40026),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {ddr3_p2_wr_en, ddr3_p2_wr_mask[3:0]} ) );

   reg_wo reg_wo_40028 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40028),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p2_wr_data[15:0] ) );

   reg_wo reg_wo_4002A ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4002A),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p2_wr_data[31:16] ) );
   
   reg_r_det reg_det_4102A (.clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4002A),
			 .ena(!cs0_r && !rw_r),
			 .pulse( ddr3_p2_wr_pulse ) );
   
   // read control for p3
   wire        burst_mode;
   wire [3:0]  reg_40030_dummy;
   reg_wo reg_wo_40030 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40030),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {burst_mode, reg_40030_dummy[3:2], p3_rd_pulse_gate, 
				  reg_40030_dummy[1:0], ddr3_p3_cmd_bl[5:0], 
				  ddr3_p3_cmd_en, ddr3_p3_cmd_instr[2:0] } ) );

   reg_wo reg_wo_40032 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40032),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p3_cmd_byte_addr[15:0] ) );

   reg_wo reg_wo_40034 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40034),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p3_cmd_byte_addr[29:16] ) );


   wire [3:0]        ddr3_p3_dummy;
   reg_wo reg_wo_40036 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40036),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {ddr3_p3_rd_en, ddr3_p3_dummy[3:0]} ) );

   // write control for romulator
   // adrfifo_rst  resets the address fifo
   // bypass_rb    enables r/b signal if set, disables if clear
   // cmdfifo_rst  resets the command fifo
   // ukfifo_rst   resets the unknown commands fifo
   // log_adr_end  enables address termination logging when set
   reg_wo reg_wo_40100 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40100),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {rom_ddr3_reset, adrfifo_rst, 
				  bypass_rb, cmdfifo_rst, ukfifo_rst, log_adr_end} ) );


   // nand_powered_on => when 1, outputs are engaged; if 0, all outputs are off
   reg_wo reg_wo_40102 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40102),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {nand_powered_on} ) );

   ///// write control for logger
   reg_wo reg_wo_40200 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40200),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {log_run, log_reset} ) );
   
   //////// read-only registers
   // loopback readback
   reg_ro reg_ro_41000 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41000),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( r40000wo[15:0] ) );

   reg_ro reg_ro_41002 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41002),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( r40002wo[15:0] ) );

   // general status register
   // 0    : DDR3 DLL lock
   // 1    : DDR3 calibration done
   // 15-2 : reads as 0
   reg_ro reg_ro_41004 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41004),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_calib_done, ddr3_dll_locked} ) );

   //// GPIO registers
   reg_ro reg_ro_41010 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41010),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( gpioA_din[15:0] ) );

   /////// ddr p2 write status
   reg_ro reg_ro_41020 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41020),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_p2_wr_count[6:0],
				  2'b00,
				  ddr3_p2_cmd_empty, ddr3_p2_cmd_full,
				  ddr3_p2_wr_full, ddr3_p2_wr_empty, 
				  ddr3_p2_wr_underrun, ddr3_p2_wr_error} ) );

   /////// ddr p3 read status & data
   reg_ro reg_ro_41030 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41030),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_p3_rd_count[6:0],
				  2'b00,
				  ddr3_p3_cmd_empty, ddr3_p3_cmd_full,
				  ddr3_p3_rd_full, ddr3_p3_rd_empty, 
				  ddr3_p3_rd_overflow, ddr3_p3_rd_error} ) );

   reg_ro reg_ro_41032 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41032),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( ddr3_p3_rd_data[15:0] ) );
   
   reg_ro reg_ro_41034 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41034),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( ddr3_p3_rd_data[31:16] ) );

   reg_r_det reg_det_41034 (.clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41034),
			 .ena(!cs0_r && rw_r),
			 .pulse( ddr3_p3_rd_pulse ) );
   
   // read status & data for romulator
   reg_ro reg_ro_41100 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41100),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {8'b0, ukfifo_data[7:0]} ) );

   reg_r_det reg_det_41100 (.clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41100),
			 .ena(!cs0_r && rw_r),
			 .pulse( ukfifo_rd_pulse ) );
				
   reg_ro reg_ro_41102 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41102),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {1'b0, ukfifo_full, ukfifo_over, ukfifo_empty, ukfifo_count[11:0]} ) );

   reg_ro reg_ro_41104 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41104),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {8'b0, cmdfifo_data[7:0]} ) );

   reg_r_det reg_det_41104 (.clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41104),
			 .ena(!cs0_r && rw_r),
			 .pulse( cmdfifo_rd_pulse ) );
				
   reg_ro reg_ro_41106 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41106),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {1'b0, cmdfifo_full, cmdfifo_over, cmdfifo_empty, cmdfifo_count[11:0]} ) );

   reg_ro reg_ro_41108 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41108),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {adrfifo_full, adrfifo_empty, adrfifo_count[13:0]} ) );

   reg_ro reg_ro_4110A ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4110A),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {adrfifo_data[15:0]} ) );

   reg_ro reg_ro_4110C ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4110C),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {2'b0,adrfifo_data[29:16]} ) );

   reg_r_det reg_det_4110C (.clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4110C),
			 .ena(!cs0_r && rw_r),
			 .pulse( adrfifo_rd_pulse ) );

   reg_ro reg_ro_4110E ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4110E), //romulator extra status
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {page_addra_over_caught, outstanding_under_caught} ) );

   reg_ro reg_ro_41110 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41110), //romulator debug
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_rd_cmd_full, ddr3_rd_dat_en,  // bind names here for
				  ddr3_rd_dat_full, ddr3_rd_dat_empty,  // easier debug
				  ddr3_rd_dat_count[6:0]} ) );

   reg_ro reg_ro_41112 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41112), //romulator debug
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr_cstate[2:0], 1'b0,
				  ddr3_wr_cmd_full, ddr3_wr_dat_en, 
				  ddr3_wr_full, ddr3_wr_empty, 
				  ddr3_wr_dat_count[6:0]} ) );


   
   // read status & data for nand logger interface
   reg_ro reg_ro_41200 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41200),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {log_wr_peak_count[6:0], log_cmd_peak_data_count[4:0], 
				  log_cmd_overflowed, log_cmd_underflowed, 
				  log_cmd_error, log_data_error} ) );

   reg_ro reg_ro_41202 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41202),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( log_entries[15:0] ) );
			 
   reg_ro reg_ro_41204 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41204),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( log_entries[26:16] ) );

   reg_ro reg_ro_41206 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41206),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( time_t_bclk[15:0] ) );

   reg_ro reg_ro_41220 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41220), // ooo: supplemental debug
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {log_cmd_full, log_wr_full} ) );

   ///// ASSUME: LSB is read first!!!
   reg_r_det_early reg_det_41206 (.clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41206),
			 .ena(!cs0_r && rw_r),
			 .pulse( time_t_update ) );

   reg_ro reg_ro_41208 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41208),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( time_t_bclk[31:16] ) );
   
   reg_ro reg_ro_4120A ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4120A),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( time_t_bclk[47:32] ) );

   reg_ro reg_ro_4120C ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4120C),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( time_t_bclk[63:48] ) );


   ///////////////////////
   ///////////////////////
   // CS1 bank registers: minimum size here is 64-bit, tuned for synchronous burst access only
   ///////////////////////

   wire [63:0] 	     rC04_0000wo;
   wire [63:0] 	     rC04_0008wo;
   wire [15:0] 	     ro_d_b;
   
   ///////// write registers
   // loopback test
   reg_wo_4burst reg_wo_4b_C04_0000( .clk(bclk_dll), .bus_ad(eim_din), .my_a(19'h4_0000), 
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]), 
				     .reg_d( rC04_0000wo[63:0] ), .rbk_d(ro_d_b) );

   reg_wo_4burst reg_wo_4b_C04_0008( .clk(bclk_dll), .bus_ad(eim_din), .my_a(19'h4_0008),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0008wo[63:0] ), .rbk_d(ro_d_b) );

   wire [63:0] 	     burst_ctl;
   wire 	     burst_stb;
      reg_wo_4burst reg_wo_4b_C04_0100( .clk(bclk_dll), .bus_ad(eim_din), .my_a(19'h4_0100),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( burst_ctl[63:0] ), .rbk_d(ro_d_b), .strobe(burst_stb) );
   

   ///////// read registers
   // loopback test
   reg_ro_4burst reg_ro_4b_C04_1000( .clk(bclk_dll), .bus_ad(eim_din), .my_a(19'h4_1000),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0000wo[63:0] ), .rbk_d(ro_d_b) );

   reg_ro_4burst reg_ro_4b_C04_1008( .clk(bclk_dll), .bus_ad(eim_din), .my_a(19'h4_1008),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0008wo[63:0] ), .rbk_d(ro_d_b) );

   wire [63:0] 	     burst_status;
   reg_ro_4burst reg_ro_4b_C04_1108( .clk(bclk_dll), .bus_ad(eim_din), .my_a(19'h4_1108),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( burst_status ), .rbk_d(ro_d_b) );
   
   wire [63:0] 	     burst_data;
   wire 	     burst_data_stb;
   reg_ro_4burst reg_ro_4b_C04_1100( .clk(bclk_dll), .bus_ad(eim_din), .my_a(19'h4_1100),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( burst_data ), .rbk_d(ro_d_b), .strobe(burst_data_stb) );
   
   
   // FPGA minor version code
   reg_ro reg_ro_41FFC ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41FFC),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( 16'h001A ) );

   // FPGA major version code
   reg_ro reg_ro_41FFE ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41FFE),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( 16'h0001 ) );

   ////////// VERSION LOG (major version 0001) /////////////
   //////
   // Minor version 001A, Jun 28 2013
   //    Fix bugs in burst DDR3 interface. Add read count to burst interface to help debug
   //    compiler issues where 32-bit accesses are generated instead of 64-bit accesses.
   //////
   // Minor version 0019, Jun 28 2013
   //    Fixed timing constraints, closure pretty much guaranteed @ 133 MHz. Only constraint
   //    that may typically fail is clk-q for data output, but only by 0.2ns.
   //    Added burst DDR3 read interface (first rev).
   //    Converted issue log to be most-recent at top, so you don't have to scroll all the way
   //    down to read the most recent message.
   //////
   // minor version 0001, May 26 2013
   //    Initial test release. Has 32kiB of mapped 16-bit RAM at 0x0800.0000 - 0x0800.7FFE
   //    write-only registers are from 0x0804.0000 - 0x0804.0FFE, 16-bit only
   //    read-only registers are from 0x0804.1000 - 0x0804.1FFE, 16-bit only
   //    Register set features include loop-back tests, as well as DDR3 interface tests
   //    DDR3 memory interface is integrated but untested.
   //////
   // Minor version 0002, May 27 2013
   //    add gpioA bank for SD card application. SD interface only wired up for now.
   //////
   // Minor version 0003, May 27 2013
   //    fix various bugs to DDR3 memory interface; add level-to-pulse converters on enables
   //////
   // Minor version 0004, May 29 2013
   //    add romulator (initial version) to design
   //////
   // Minor version 0005, May 29 2013
   //    add nand_powered_on (reg 040102, bit 0)
   //    add known_command feedback
   //////
   // Minor version 0006, May 30 2013
   //    romulator seems to be mostly working now. this version maps all address accesses
   //    above 0x80000 to RAM addresses 0x8000 in a looping fashion. It also assumes
   //    the presence of ECC data inside the RAM data.
   //    Also added the option to log when addresses terminate (so adr_fifo entries are in start-end pairs)
   //////
   // Minor version 0007, May 30 2013
   //    increased FIFO depth of address log to 16384 from 4096, changed bitfields of status register
   //    accordingly
   //////
   // Minor version 0008, Jun  1 2013
   //    added auto-increment pulse & pulse gate bits to DDR3 interface
   //    modified DDR3 core to have two additional 64-bit read/write ports for LA function
   //////
   // Minor version 0009, Jun  9 2013
   //    convert romulator to DDR3 version, for greater ROM depth. Just reads supported for now.
   //////
   // Minor version 000A, Jun 10 2013
   //    fix bugs in DDR3 romulator code
   //////
   // Minor version 000B, Jun 10 2013
   //    moar boogs
   //////
   // Minor version 000C, Jun 10 2013
   //    boogs! boogs! boogs!
   //    and with this version, we have a working DDR3 romulator servicing reads only
   //////
   // Minor version 000D, Jun 11 2013
   //    add in NAND logger. It runs based upon log_run being set, and it will ignore
   //    DDR3 writes from the CPU interface when log_run is set
   //    NAND logging starts 0x0F00_0000, ends at 0x0FFF_FFFF (high 16 MiB)
   //////
   // Minor version 000E, Jun 11 2013
   //    add page program support into romulator. Initial version, bugs expected!
   //////
   // Minor version 000F, Jun 12 2013
   //    boog fixes, mostly found through sim work. First attempt at real implementation.
   //////
   // Minor version 0010, Jun 12 2013
   //    derp. boogs at top level. added debug signals to check wr fifo logic irl.
   //////
   // Minor version 0011, Jun 13 2013
   //    Seems like I have to implement partial page programming. We from burst-of-16 for
   //    write efficiency to burst-of-one for convenience. Writes take a long time anyways on
   //    real NAND so I'm not bent out of shape that it takes a lot longer to complete. Still
   //    faster than NAND in the end.
   //////
   // Minor version 0012, Jun 13 2013
   //    Basic functions working; can write a single page via SD interface and read it back.
   //    Fixing logger integration bugs.
   //////
   // Minor version 0013, Jun 15 2013
   //    Fix bug with row_adr_w not being used correctly, causing page write addresses to change
   //    part-way through the write process. Suspect that nand_rb is being ignored or not properly
   //    generated.
   //////
   // Minor version 0014, Jun 19 2013
   //    Add support for page erase operations.
   //////
   // Minor version 0015, Jun 19 2013
   //    Fix operation size for erase operations to be 256kB pages
   //////
   // Minor version 0016, Jun 21 2013
   //    Fix problem with DDR3 reset during reset log (tripping reset causes garbage to write into memory)
   //////
   // Minor version 0017, Jun 21 2013
   //    Add CS1 interface with burst capability
   //////
   // Minor version 0018, Jun 21 2013
   //    Add timing constraints to improve performance of MAP/PAR
   //////
   
   // mux between block memory and register set based on high bits
   //   assign eim_dout = (bus_addr[18:16] != 3'b000) ? ro_d : bram_dout;
   // pipeline to improve timing
   reg [15:0]		     ro_d_r;
   reg [15:0] 		     ro_d_b_r;
   reg [1:0] 		     eim_rdcs;
   reg [15:0] 		     eim_dout_pipe;
   reg [15:0] 		     eim_dout_pipe2;
   
   always @(posedge bclk_dll) begin
      ro_d_r <= ro_d;
      ro_d_b_r <= ro_d_b;
      eim_rdcs[1:0] <= EIM_CS[1:0];
      eim_dout_pipe <= (eim_rdcs[1:0] == 2'b10) ? ro_d_r : ro_d_b_r;
   end

   always @(posedge bclk_o) begin
      eim_dout_pipe2 <= eim_dout_pipe; // retime near the source to allow max time for wire delay
      eim_dout <= eim_dout_pipe2; // no time to do anything but transit between these domains
   end;
   

   //////////////
   /// "heartbeat" counter
   //////////////
   always @(posedge clk50) begin
      counter <= counter + 1;
   end

   assign FPGA_LED2 = counter[23];

   //////////////
   // IOBUFs as required by design
   //////////////
   IBUFGDS clkibufgds( .I(CLK2_P), .IB(CLK2_N), .O(clk) );

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
			    .clk100(clk100),
			    .RESET(reset),
			    .LOCKED(ddr3_dll_locked)
			    );

   wire p2_cmd_en_pulse, p2_wr_en_pulse, p3_cmd_en_pulse, p3_rd_en_pulse;
   rising_edge p2cmdp2e( .clk(bclk_dll), .level(ddr3_p2_cmd_en), .pulse(p2_cmd_en_pulse) );
   rising_edge p2wrp2e( .clk(bclk_dll), .level(ddr3_p2_wr_en), .pulse(p2_wr_en_pulse) );
   rising_edge p3cmdp2e( .clk(bclk_dll), .level(ddr3_p3_cmd_en), .pulse(p3_cmd_en_pulse) );
   rising_edge p2rdp2e( .clk(bclk_dll), .level(ddr3_p3_rd_en), .pulse(p3_rd_en_pulse) );

   // add mux to switch between CPU-write to DDR3 memory, and logger writing to DDR3
   wire p2_cmd_en;
   wire [2:0] p2_cmd_instr;
   wire [5:0] p2_cmd_bl;
   wire [29:0] p2_cmd_byte_addr;
   wire       p2_wr_en;
   wire [3:0] p2_wr_mask;
   wire [31:0] p2_wr_data;

   reg 	       log_cmd_en_delay;
   
   always @(posedge bclk_dll) begin
      log_cmd_en_delay <= !logbuf_empty && !logbuf_cmd_full;
   end
   
   assign p2_cmd_en = log_run ? log_cmd_en_delay : p2_cmd_en_pulse;
   assign p2_cmd_instr[2:0] = log_run ? logbuf_cmd_instr[2:0] : ddr3_p2_cmd_instr[2:0];
   assign p2_cmd_bl = log_run ? logbuf_cmd_burstlen : ddr3_p2_cmd_bl;
   assign p2_cmd_byte_addr = log_run ? logbuf_cmd_addr : ddr3_p2_cmd_byte_addr;
   assign p2_wr_en = log_run ? log_wr_en : (p2_wr_en_pulse || (ddr3_p2_wr_pulse & p2_wr_pulse_gate));
   assign p2_wr_mask = log_run ? log_wr_mask : ddr3_p2_wr_mask;
   assign p2_wr_data = log_run ? log_wr_data : ddr3_p2_wr_data;

   wire p3_cmd_en;
   wire p3_burst_cmd_en;
   wire [2:0] p3_cmd_instr;
   wire [2:0] p3_burst_cmd_instr;
   wire [5:0] p3_cmd_bl;
   wire [5:0] p3_burst_cmd_bl;
   wire [29:0] p3_cmd_byte_addr;
   wire [29:0] p3_burst_addr;
   wire        p3_rd_en;
   wire        p3_burst_rd_en;

   assign p3_cmd_en = burst_mode ? p3_burst_cmd_en : p3_cmd_en_pulse;
   assign p3_cmd_instr = burst_mode ? p3_burst_cmd_instr : ddr3_p3_cmd_instr;
   assign p3_cmd_bl = burst_mode ? p3_burst_cmd_bl : ddr3_p3_cmd_bl;
   assign p3_cmd_byte_addr = burst_mode ? p3_burst_addr : ddr3_p3_cmd_byte_addr;
   assign p3_rd_en = burst_mode ? p3_burst_rd_en : (p3_rd_en_pulse || (ddr3_p3_rd_pulse & p3_rd_pulse_gate));

   wire ddr3_reset_local;
   ddr3_eim_cs1 cs1_adapter(.clk(bclk_dll),
			    .ctl(burst_ctl),
			    .ctl_stb(burst_stb),
			    .burst_rd(burst_data[63:0]),
			    .rd_stb(burst_data_stb),
			    .status(burst_status[63:0]),

			    .ddr3_rd_cmd(p3_burst_cmd_instr),
			    .ddr3_rd_bl(p3_burst_cmd_bl),
			    .ddr3_rd_adr(p3_burst_addr),
			    .ddr3_rd_cmd_en(p3_burst_cmd_en),
			    .ddr3_rd_cmd_empty(ddr3_p3_cmd_empty),
			    .ddr3_rd_cmd_full(ddr3_p3_cmd_full),

			    .ddr3_rd_data(ddr3_p3_rd_data[31:0]),
			    .ddr3_rd_count(ddr3_p3_rd_count),
			    .ddr3_rd_empty(ddr3_p3_rd_empty),
			    .ddr3_rd_full(ddr3_p3_rd_full),
			    .ddr3_rd_en(p3_burst_rd_en),
			    
			    .reset(ddr3_reset_local)
			    );
   
   sync_reset ddr3_res_sync( .glbl_reset(log_reset), .clk(ddr3clk), .reset(ddr3_reset_local) );

   ddr3_if_4port # (
	      .C1_P0_MASK_SIZE(4),
	      .C1_P0_DATA_PORT_SIZE(32),
	      .C1_P1_MASK_SIZE(4),
	      .C1_P1_DATA_PORT_SIZE(32),
	      .DEBUG_EN(0),
	      .C1_MEMCLK_PERIOD(2500),
	      .C1_CALIB_SOFT_IP("TRUE"),
	      .C1_SIMULATION("FALSE"),
	      .C1_RST_ACT_LOW(0),
	      .C1_INPUT_CLK_TYPE("SINGLE_ENDED"),
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

	      // port 2 shared between logger and CPU interface (based on log_run signal)
	      .c1_p2_cmd_clk                          (bclk_dll),
	      .c1_p2_cmd_en                           (p2_cmd_en),
	      .c1_p2_cmd_instr                        (p2_cmd_instr),
	      .c1_p2_cmd_bl                           (p2_cmd_bl),
	      .c1_p2_cmd_byte_addr                    (p2_cmd_byte_addr),
	      .c1_p2_cmd_empty                        (ddr3_p2_cmd_empty),
	      .c1_p2_cmd_full                         (ddr3_p2_cmd_full),
	      .c1_p2_wr_clk                           (bclk_dll),
	      .c1_p2_wr_en                            (p2_wr_en),
	      .c1_p2_wr_mask                          (p2_wr_mask),
	      .c1_p2_wr_data                          (p2_wr_data),
	      .c1_p2_wr_full                          (ddr3_p2_wr_full),
	      .c1_p2_wr_empty                         (ddr3_p2_wr_empty),
	      .c1_p2_wr_count                         (ddr3_p2_wr_count),
	      .c1_p2_wr_underrun                      (ddr3_p2_wr_underrun),
	      .c1_p2_wr_error                         (ddr3_p2_wr_error),

	      // port 3 shared between cs0 and cs1 burst read interfaces
	      .c1_p3_cmd_clk                          (bclk_dll),
	      .c1_p3_cmd_en                           (p3_cmd_en),
	      .c1_p3_cmd_instr                        (p3_cmd_instr),
	      .c1_p3_cmd_bl                           (p3_cmd_bl),
	      .c1_p3_cmd_byte_addr                    (p3_cmd_byte_addr),
	      .c1_p3_cmd_empty                        (ddr3_p3_cmd_empty),
	      .c1_p3_cmd_full                         (ddr3_p3_cmd_full),
	      .c1_p3_rd_clk                           (bclk_dll),
	      .c1_p3_rd_en                            (p3_rd_en),
	      .c1_p3_rd_data                          (ddr3_p3_rd_data),
	      .c1_p3_rd_full                          (ddr3_p3_rd_full),
	      .c1_p3_rd_empty                         (ddr3_p3_rd_empty),
	      .c1_p3_rd_count                         (ddr3_p3_rd_count),
	      .c1_p3_rd_overflow                      (ddr3_p3_rd_overflow),
	      .c1_p3_rd_error                         (ddr3_p3_rd_error),


	      /////////// romulator read & write ports
	      .c1_p4_cmd_clk                          (ddr3_wr_clk),
	      .c1_p4_cmd_en                           (ddr3_wr_cmd_en),
	      .c1_p4_cmd_instr                        (ddr3_wr_cmd_instr[2:0]),
	      .c1_p4_cmd_bl                           (ddr3_wr_cmd_bl[5:0]),
	      .c1_p4_cmd_byte_addr                    (ddr3_wr_adr[29:0]),
	      .c1_p4_cmd_empty                        (ddr3_wr_cmd_empty),
	      .c1_p4_cmd_full                         (ddr3_wr_cmd_full),
	      .c1_p4_wr_clk                           (ddr3_wr_clk),
	      .c1_p4_wr_en                            (ddr3_wr_dat_en),
	      .c1_p4_wr_mask                          (ddr3_wr_mask[3:0]),
	      .c1_p4_wr_data                          (ddr3_wr_dat[31:0]),
	      .c1_p4_wr_full                          (ddr3_wr_full),
	      .c1_p4_wr_empty                         (ddr3_wr_empty),
//	      .c1_p4_wr_underrun                      
//	      .c1_p4_wr_error                         
	      .c1_p4_wr_count                         (ddr3_wr_dat_count[6:0]),

	      .c1_p5_cmd_clk                          (ddr3_rd_clk),
	      .c1_p5_cmd_en                           (ddr3_rd_cmd_en),
	      .c1_p5_cmd_instr                        (ddr3_rd_cmd_instr[2:0]),
	      .c1_p5_cmd_bl                           (ddr3_rd_cmd_bl[5:0]),
	      .c1_p5_cmd_byte_addr                    (ddr3_rd_adr[29:0]),
//	      .c1_p5_cmd_empty                        (c1_p5_cmd_empty),
	      .c1_p5_cmd_full                         (ddr3_rd_cmd_full),
	      .c1_p5_rd_clk                           (ddr3_rd_clk),
	      .c1_p5_rd_en                            (ddr3_rd_dat_en),
	      .c1_p5_rd_data                          (ddr3_rd_dat[31:0]),
	      .c1_p5_rd_full                          (ddr3_rd_dat_full),
	      .c1_p5_rd_empty                         (ddr3_rd_dat_empty),
	      .c1_p5_rd_count                         (ddr3_rd_dat_count[6:0]),
	      .c1_p5_rd_overflow                      (ddr3_rd_dat_overflow)
//	      .c1_p5_rd_error                         (c1_p5_rd_error)
	      );
   
   //////////////
   // tie downs (unused signals as of this rev of design)
   //////////////
   assign APOPTOSIS = 1'b0; // make apoptosis inactive, tigh high to force reboot on config
   assign ECSPI3_MISO = 1'b0;
   
endmodule
