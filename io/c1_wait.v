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

module c1_wait(
	input CLK_68KCLK, nAS,
	input nROM_ZONE, nPORT_ZONE, nCARD_ZONE, nSROM_ZONE,
	input nROMWAIT, nPWAIT0, nPWAIT1, PDTACK,
	output nDTACK
);

	reg [2:0] WAIT_CNT;
	
	//assign nPDTACK = ~(nPORT_ZONE | PDTACK);		// Really a NOR ? May stall CPU if PDTACK = GND

	// Notes:
	// Normally, nDTACK = nAS for nROM_ZONE
	
	assign nDTACK = nAS | ~WAIT_MUX;					// Is it nVALID instead of nAS ?
	
	wire WAIT_MUX = (!nROM_ZONE) ? (WAIT_CNT < 4) :
			(!nPORT_ZONE) ? (WAIT_CNT < 4) :
			(!nCARD_ZONE) ? (WAIT_CNT < 3) :
			(!nSROM_ZONE) ? (WAIT_CNT < 4) :
			1'b1;
	
	//assign nCLK_68KCLK = ~nCLK_68KCLK;
	
	always @(posedge CLK_68KCLK)	// negedge
	begin
		if (!nAS)
		begin
			if (WAIT_CNT)
				WAIT_CNT <= WAIT_CNT - 1'b1;
		end
		else
		begin
			WAIT_CNT <= 5;
		end
	end
	
endmodule
