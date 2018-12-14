module C43(
	input CK,
	input [3:0] D,
	input nL, EN, CI, nCL,
	output reg [3:0] Q = 4'd0,
	output CO
);

	assign CL = ~nCL;
	
	always @(posedge CK, posedge CL)
	begin
		if (CL)
		begin
			Q <= 4'd0;		// Clear
		end
		else
		begin
			if (!nL)
				Q <= D;			// Load
			else if (EN & CI)
				Q <= Q + 1'b1;	// Count
			else
				Q <= Q;
		end
	end
	
	assign CO = &{Q[3:0], CI};

endmodule
