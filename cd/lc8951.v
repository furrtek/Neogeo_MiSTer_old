//============================================================================
//  SNK NeoGeo for MiSTer
//
//  Copyright (C) 2018 Sean 'Furrtek' Gonsalves
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

module lc8951(
	input nRESET,
	input CLK_12M,
	input nWR, nRD,
	input RS,	// Register select, 0=Access address register, 1=Access pointed register
	input [3:0] DIN,
	output reg [3:0] DOUT
);

	reg [3:0] AR;	// Address Register
	reg [7:0] REGS_WRITE [16];
	reg [7:0] REGS_READ [16];
	
	reg nWR_PREV, nRD_PREV;
	
	always @(negedge CLK_12M or negedge nRESET)
	begin
		if (!nRESET)
		begin
			AR <= 4'd0;
			nWR_PREV <= 1;
			nRD_PREV <= 1;
		end
		else
		begin
			if (nWR_PREV & ~nWR)
			begin
				// Write
				if (~RS)
					AR <= DIN[3:0];
				else
				begin
					REGS_WRITE[AR] <= DIN[3:0];
					
					// Auto-inc
					if (|{AR})
						AR <= AR + 1'b1;
				end
			end
			else if (nRD_PREV & ~nRD)
			begin
				// Read
				if (~RS)
					DOUT <= {4'b0000, AR};
				else
				begin
					DOUT <= REGS_READ[AR];
				
					// Auto-inc
					if (|{AR})
						AR <= AR + 1'b1;
				end
			end
		end
	end
	
endmodule
