module reg_wo(
	      input wire clk,
	      input wire [18:0] bus_a,
	      input wire [18:0] my_a,
	      input wire [15:0] bus_d,
	      input wire we,
	      output wire [15:0] reg_d
	      );

   reg [15:0] 			 state;

   always @(posedge clk) begin
      if( (bus_a[18:1] == my_a[18:1]) && we ) begin
	 state <= bus_d;
      end else begin
	 state <= state;
      end
   end

   assign reg_d = state;

endmodule // reg_wo
