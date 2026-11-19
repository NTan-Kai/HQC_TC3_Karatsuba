module dual_port_ram
#(
  parameter AWIDTH = 12, 			// address width
  parameter DWIDTH = 7 				// data width
)
(
	///*** Port A***///
	input clka, 					// clock
	input ena, 						// port A read enable
	input wea,	 					// port A write enable
	input [AWIDTH-1:0] addra, 		// port A address
	input [DWIDTH-1:0] dina, 		// port A data
	output reg [DWIDTH-1:0] douta, 	// port A data output
	
	///*** Port B***///
	input clkb, 					// clock
	input enb, 						// port B read enable
	input web,	 					// port B write enable
	input [AWIDTH-1:0] addrb, 		// port B address
	input [DWIDTH-1:0] dinb, 		// port B data
	output reg [DWIDTH-1:0] doutb 	// port B data output
);

	// Let synthesis choose BRAM vs LUTRAM per depth/width to relieve BRAM exhaustion.
	(* ram_style = "auto" *) reg [DWIDTH-1:0] mem [2**AWIDTH-1:0];

	always @(posedge clka) begin
		// /*** Port A***///
		if (ena) begin
			if(wea) begin
				mem[addra] <= dina;
			end
			douta <= mem[addra];
		end
		
	end
	
	always @(posedge clkb) begin		
		// /*** Port B***///
		if (enb) begin
			if(web) begin
				mem[addrb] <= dinb;
			end
			doutb <= mem[addrb];
		end
	end
endmodule