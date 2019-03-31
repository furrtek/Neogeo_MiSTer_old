//============================================================================
//  SNK NeoGeo for MiSTer
//
//  Copyright (C) 2019 Sean 'Furrtek' Gonsalves
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module cd_dma(
	input [1:0] SYSTEM_MODE,
	input [23:1] M68K_ADDR,
	input [15:0] M68K_DATA,
	input nLDS, nUDS,
	input RW, nAS
);

	reg [31:0] SOURCE;
	reg [31:0] DEST;
	reg [31:0] VALUE;
	reg [31:0] COUNT;
	reg [15:0] MICROCODE [9];
	reg RUN;
	
	wire ENABLED = SYSTEM_MODE[1];

	always @(posedge RW)
	begin
		if (M68K_ADDR[23:8] == 16'hFF00)
		begin
			casez ({M68K_ADDR[7:1], nUDS, nLDS})
				9'b0110000_10: RUN <= M68K_DATA[6];				// FF0061
				9'b0110010_00: SOURCE[31:16]	<= M68K_DATA;	// FF0064~FF0065
				9'b0110011_00: SOURCE[15:0] <= M68K_DATA;		// FF0066~FF0067
				9'b0110100_00: DEST[31:16] <= M68K_DATA;		// FF0068~FF0069
				9'b0110101_00: DEST[15:0]	<= M68K_DATA;		// FF006A~FF006B
				9'b0110110_00: VALUE[31:16] <= M68K_DATA;		// FF006C~FF006D
				9'b0110111_00: VALUE[15:0] <= M68K_DATA;		// FF006E~FF006F
				9'b0111000_00: COUNT[31:16] <= M68K_DATA;		// FF0070~FF0071
				9'b0111001_00: COUNT[15:0] <= M68K_DATA;		// FF0072~FF0073
				9'b0111111_00: MICROCODE[M68K_ADDR[4:1]] <= M68K_DATA;	// FF007E+
				default;
			endcase
		end
	end

endmodule
