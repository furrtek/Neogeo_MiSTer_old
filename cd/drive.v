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

module cd_drive(
	input nRESET,
	input CLK_12M,
	input HOCK,
	output reg CDCK,
	input [3:0] CDD_DIN,
	output reg [3:0] CDD_DOUT,
	output reg CD_nIRQ,
	output reg [15:0] sd_req_type
);

	// TOC sub-commands:
	// 0: Get position
	//		Unimplemented
	// 1: Get position relative
	//		Unimplemented
	// 2: Get track number
	//		Unimplemented
	// 3: Get CD length: sd_req_type = 16'hD100
	//		0: M
	//		1: S
	//		2: F
	// 4: Get first/last: sd_req_type = 16'hD000
	//		0: First track # BCD
	//		1: Last track # BCD
	// 5: Get track start: sd_req_type = 16'hD2nn
	//		0: M
	//		1: S
	//		2: F
	// 6: Get track type
	//		Unimplemented

	reg [5:0] CLK_DIV;
	reg [11:0] IRQ_TIMER;
	reg [3:0] DOUT_COUNTER;	// 0~10
	reg [3:0] DIN_COUNTER;	// 0~10
	
	reg [3:0] STATUS_DATA [9];
	reg [3:0] COMMAND_DATA [10];
	
	reg HOCK_PREV;
	reg [1:0] COMM_STATE;		// 0~2
	
	reg [3:0] CHECKSUM_IN;
	
	wire [3:0] CHECKSUM_OUT = ~(4'd5 + STATUS_DATA[0] + STATUS_DATA[1] + STATUS_DATA[2] +  STATUS_DATA[3] + 
										STATUS_DATA[4] +  STATUS_DATA[5] +  STATUS_DATA[6] +  STATUS_DATA[7] + 
										STATUS_DATA[8]);
	
	always @(posedge CLK_12M or negedge nRESET)
	begin
		if (!nRESET)
		begin
			CLK_DIV <= 6'd0;
			IRQ_TIMER <= 12'd0;
			DOUT_COUNTER <= 4'd10;
			DIN_COUNTER <= 4'd10;
			HOCK_PREV <= 0;
			COMM_STATE <= 2'd0;
			CD_nIRQ <= 1;
			sd_req_type <= 16'h0000;
			
			STATUS_DATA[0] <= 4'd0;	// Stopped
			STATUS_DATA[1] <= 4'd0;
			STATUS_DATA[2] <= 4'd0;
			STATUS_DATA[3] <= 4'd0;
			STATUS_DATA[4] <= 4'd0;
			STATUS_DATA[5] <= 4'd0;
			STATUS_DATA[6] <= 4'd0;
			STATUS_DATA[7] <= 4'd0;
			STATUS_DATA[8] <= 4'd0;
		end
		else
		begin
		
			// This simulates the Sony CDD MCU, so it should be quite slow
			// Here it "runs" at 12M/48=250kHz
			
			if (CLK_DIV == 6'd48-1)
			begin
				CLK_DIV <= 6'd0;
				
				HOCK_PREV <= HOCK;
				
				// Fire IRQ at 64Hz
				if (IRQ_TIMER == 12'd3906-1)
				begin
					IRQ_TIMER <= 12'd0;
					CD_nIRQ <= 1'b0;
					COMM_STATE <= 2'd0;
					DOUT_COUNTER <= 4'd0;
					DIN_COUNTER <= 4'd0;
				end
				else
				begin
					// To allow CD_nIRQ retry
					if (IRQ_TIMER == 12'd1953-1)
						CD_nIRQ <= 1'b1;
						
					IRQ_TIMER <= IRQ_TIMER + 1'b1;
				end
				
				if (~HOCK & ~CD_nIRQ)
					CD_nIRQ <= 1'b1;		// Comm. started ok
				
				if (CD_nIRQ)
				begin
					if (DOUT_COUNTER != 4'd10)
					begin
						// CDD to HOST
						
						if (COMM_STATE == 2'd0)
						begin
							// Put data on bus
							CDD_DOUT <= (DOUT_COUNTER == 4'd9) ? CHECKSUM_OUT : STATUS_DATA[DOUT_COUNTER];
							CDCK <= 1'b0;
							COMM_STATE <= 2'd1;
						end
						else if (COMM_STATE == 2'd1)
						begin
							// Wait for HOCK high
							if (~HOCK_PREV & HOCK)
							begin
								CDCK <= 1'b1;
								COMM_STATE <= 2'd2;
								// Escape from CDD -> HOST mode at last
								if (DOUT_COUNTER == 4'd9)
								begin
									DOUT_COUNTER <= 4'd10;
									COMM_STATE <= 2'd1;
									CHECKSUM_IN <= 4'd5;
								end
							end
						end
						else if (COMM_STATE == 2'd2)
						begin
							// Wait for HOCK low
							if (HOCK_PREV & ~HOCK)
							begin
								DOUT_COUNTER <= DOUT_COUNTER + 1'b1;
								COMM_STATE <= 2'd0;
							end
						end
						
					end
					else if (DIN_COUNTER < 4'd10)
					begin
						// HOST to CDD
						
						if (COMM_STATE == 2'd0)
						begin
							// Wait for HOCK rising edge
							if (~HOCK_PREV & HOCK)
							begin
								COMMAND_DATA[DIN_COUNTER] <= CDD_DIN;
								CHECKSUM_IN <= CHECKSUM_IN + CDD_DIN;
								CDCK <= 1'b1;
								DIN_COUNTER <= DIN_COUNTER + 1'b1;
								COMM_STATE <= 2'd1;
							end
						end
						else if (COMM_STATE == 2'd1)
						begin
							// Wait for HOCK falling edge
							if (HOCK_PREV & ~HOCK)
							begin
								CDCK <= 1'b0;
								COMM_STATE <= 2'd0;
							end
						end
						
					end
					else if (DIN_COUNTER == 4'd10)
					begin
						DIN_COUNTER <= 4'd11;
						
						// Comm frame just ended
						if (CHECKSUM_IN == 4'd15)
						begin
							// Checksum ok, process command
							if (COMMAND_DATA[0] == 4'd1)
							begin
								// STOP command
								STATUS_DATA[0] <= 4'd0;	// Stopped
							end
							else if (COMMAND_DATA[0] == 4'd2)
							begin
								// TOC command
								STATUS_DATA[1] <= COMMAND_DATA[3];
								
								if (COMMAND_DATA[3] == 4'd0)
								begin
									// Get absolute position
								end
								else if (COMMAND_DATA[3] == 4'd1)
								begin
									// Get relative position
								end
								else if (COMMAND_DATA[3] == 4'd2)
								begin
									// Get track number
								end
								else if (COMMAND_DATA[3] == 4'd3)
								begin
									// Get CD length
									STATUS_DATA[0] <= 4'd9;	// Reading TOC
									STATUS_DATA[2] <= 4'd5;	// 59:00:00 length DEBUG
									STATUS_DATA[3] <= 4'd9;
									STATUS_DATA[4] <= 4'd0;
									STATUS_DATA[5] <= 4'd0;
									STATUS_DATA[6] <= 4'd0;
									STATUS_DATA[7] <= 4'd0;
									STATUS_DATA[8] <= 4'd0;
								end
								else if (COMMAND_DATA[3] == 4'd4)
								begin
									// Get first and last tracks
									STATUS_DATA[0] <= 4'd9;	// Reading TOC
									STATUS_DATA[2] <= 4'd0;
									STATUS_DATA[3] <= 4'd1;
									STATUS_DATA[4] <= 4'd1;	// 15 tracks DEBUG
									STATUS_DATA[5] <= 4'd5;
									STATUS_DATA[6] <= 4'd0;
									STATUS_DATA[7] <= 4'd0;
									STATUS_DATA[8] <= 4'd0;
								end
								else if (COMMAND_DATA[3] == 4'd5)
								begin
									// Get track start and type
									STATUS_DATA[0] <= 4'd9;	// Reading TOC
									STATUS_DATA[2] <= COMMAND_DATA[4];	// xx:02:00
									STATUS_DATA[3] <= COMMAND_DATA[5];
									STATUS_DATA[4] <= 4'd0;
									STATUS_DATA[5] <= 4'd2;
									STATUS_DATA[6] <= 4'd0;	// OR 8 for data track
									STATUS_DATA[7] <= 4'd0;
									STATUS_DATA[8] <= COMMAND_DATA[5];
								end
								else if (COMMAND_DATA[3] == 4'd6)
								begin
									// Get last error
								end
							end
						end
							
					end
					
				end
			end
			else
				CLK_DIV <= CLK_DIV + 1'b1;
		end
	end
	
endmodule
