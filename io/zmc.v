// NeoGeo logic definition
// Copyright (C) 2018 Sean Gonsalves
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

module zmc(
	input SDRD0,
	input [1:0] SDA_L,
	input [15:8] SDA_U,
	output [21:11] MA
);

	// Initialize ?
	//      Z80         ROM          Region	Reg
	// 1E = F000~F7FF = F000~F7FF		1111		(0)
	// 0E = E000~EFFF = E000~EFFF		1110		(1)
	// 06 = C000~DFFF = C000~DFFF		110x		(2)
	// 02 = 8000~BFFF = 8000~BFFF		10xx		(3)
	
	//                      IIII Iiii
	//                      MMMM M
	// 0001 1110 -> 00 0xxx 1111 0--- ---- ----
	// 0000 1110 -> 00 xxxx 1110 ---- ---- ----
	// 0000 0110 -> 0x xxxx 110- ---- ---- ----
	// 0000 0010 -> xx xxxx 10-- ---- ---- ----
	// MA:          xx xxxx xxxx x--- ---- ----

	wire [7:0] BANKSEL = SDA_U[15:8];		// SDA15 used for something else ?
	
	reg [7:0] RANGE_0;					// 2048 ? Can only access max. 512kB ROM
	reg [7:0] RANGE_1;					// 1024 ? Can only access max. 1MB ROM
	reg [7:0] RANGE_2;					// 512 ?  Can only access max. 2MB ROM
	reg [7:0] RANGE_3;
	
	assign MA = SDA_U[15] ?				// In mapped region ?
						~SDA_U[14] ?
							{RANGE_3, SDA_U[13:11]} :					// 8000~BFFF
							~SDA_U[13] ?
								{1'b0, RANGE_2, SDA_U[12:11]} :		// C000~DFFF
								~SDA_U[12] ?
									{2'b00, RANGE_1, SDA_U[11]} :		// E000~EFFF
									{3'b000, RANGE_0} :					// F000~F7FF
					{6'b000000, SDA_U[15:11]};							// Pass through

	always @(posedge SDRD0)
	begin
		case (SDA_L)
			0: RANGE_0 <= BANKSEL;
			1: RANGE_1 <= BANKSEL;
			2: RANGE_2 <= BANKSEL;
			3: RANGE_3 <= BANKSEL;
		endcase
	end
	
endmodule
