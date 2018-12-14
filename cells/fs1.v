`timescale 1ns/1ns

module FS1(
	input CK,
	input SD,
	output reg [3:0] Q
);

	always @(negedge CK)
		Q <= #1 {Q[2:0], ~SD};

endmodule
