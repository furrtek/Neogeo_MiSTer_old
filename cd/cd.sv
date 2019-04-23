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

module cd_sys(
	input nRESET,
	input CLK_68KCLK,
	input [23:1] M68K_ADDR,
	inout [15:0] M68K_DATA,
	input A22Z, A23Z,
	input nLDS, nUDS,
	input M68K_RW, nAS, nDTACK,
	input [1:0] SYSTEM_TYPE,
	input [1:0] CD_REGION,
	output reg CD_VIDEO_EN,
	output reg CD_FIX_EN,
	output reg CD_SPR_EN,
	
	output reg CD_nRESET_Z80,
	
	output CD_TR_WR_SPR,
	output CD_TR_WR_PCM,
	output CD_TR_WR_Z80,
	output CD_TR_WR_FIX,
	output reg [2:0] CD_TR_AREA,
	output reg [1:0] CD_BANK_SPR,
	output reg CD_BANK_PCM,
	output reg [15:0] CD_TR_WR_DATA,
	output reg [19:1] CD_TR_WR_ADDR,
	
	input IACK,
	output reg CD_IRQ,
	
	input clk_sys,
	output [15:0] sd_req_type,
	output sd_rd,
	input sd_ack,
	input [15:0] sd_buff_dout,	// Data from HPS
	input sd_buff_wr,
	output [31:0] sd_lba,
	
	input CD_LID,	// DEBUG
	
	// For DMA
	output reg nBR,
	input nBG,
	output reg nBGACK
);

	reg CD_USE_SPR, CD_USE_PCM, CD_USE_Z80, CD_USE_FIX;
	reg CD_UPLOAD_EN;
	reg CD_nRESET_DRIVE;
	reg [15:0] REG_FF0002;
	reg [2:0] CD_IRQ_FLAGS;
	
	reg [31:0] DMA_SOURCE;
	reg [31:0] DMA_DEST;
	reg [31:0] DMA_VALUE;
	reg [31:0] DMA_COUNT;
	reg [31:0] DMA_COUNT_RUN;		// TESTING
	reg [15:0] DMA_MICROCODE [9];
	reg DMA_RUN;
	
	reg CDD_nIRQ_PREV, CDC_nIRQ_PREV;
	reg nLDS_PREV, nUDS_PREV;
	
	reg [3:0] CDD_DIN;
	wire [3:0] CDD_DOUT;
	
	wire [7:0] MSF_M;
	wire [7:0] MSF_S;
	wire [7:0] MSF_F;
	
	cd_drive DRIVE(
		nRESET & CD_nRESET_DRIVE,
		CLK_68KCLK,
		HOCK, CDCK,
		CDD_DIN, CDD_DOUT,
		CDD_nIRQ,
		
		clk_sys,
		DRIVE_sd_req_type,
		DRIVE_sd_rd,
		sd_ack,
		sd_buff_dout,
		sd_buff_wr,
		
		MSF_M, MSF_S, MSF_F,
		NEXT_SECTOR_REQ,	// To increment MSF
		DRIVE_READING
	);
	
	wire [7:0] LC8951_DOUT;
	
	lc8951 LC8951(
		nRESET,
		CLK_68KCLK,
		~LC8951_WR, ~LC8951_RD, M68K_ADDR[1],
		M68K_DATA[7:0],
		LC8951_DOUT,
		
		MSF_M, MSF_S, MSF_F,
		
		SECTOR_READY,
		DMA_DONE,
		CDC_nIRQ,
		NEXT_SECTOR_REQ
	);
	
	reg SECTOR_REQ;
	wire [15:0] DRIVE_sd_req_type;
	
	assign sd_req_type = SECTOR_REQ ? 16'h4801 : DRIVE_sd_req_type;
	assign sd_rd = SECTOR_REQ ? DMA_sd_rd : DRIVE_sd_rd;
	
	reg SECTOR_READY;
	reg FORCE_WR;
	reg [1:0] LOADING_STATE;
	reg [10:0] CACHE_ADDR;	// 0~2047
	
	wire [7:0] CACHE_DIN = CACHE_ADDR[0] ? sd_buff_dout[7:0] : sd_buff_dout[15:8];
	wire CACHE_WR = sd_buff_wr | FORCE_WR;
	wire [7:0] CACHE_DOUT;	// TODO
	
	cache CACHE(
		CACHE_ADDR,
		clk_sys,
		CACHE_DIN,
		CACHE_WR,
		CACHE_DOUT
	);
	
	reg DMA_START_PREV, DMA_START;
	reg [1:0] DMA_MODE;
	reg [2:0] DMA_STATE;
	reg DMA_DONE;
	reg DMA_sd_rd;
	
	reg DRIVE_READING_PREV;
	reg NEXT_SECTOR_REQ_PREV;
	
	always @(posedge clk_sys or negedge nRESET)
	begin
		if (!nRESET)
		begin
			FORCE_WR <= 0;
			SECTOR_READY <= 0;
			LOADING_STATE <= 2'd0;	// Idle, waiting for request
			
			DMA_STATE <= 3'd0;
			DMA_sd_rd <= 0;
			nBR <= 1;
			nBGACK <= 1;
			
			DRIVE_READING_PREV <= 0;
			DMA_DONE <= 0;
		end
		else
		begin
			DRIVE_READING_PREV <= DRIVE_READING;
			NEXT_SECTOR_REQ_PREV <= NEXT_SECTOR_REQ;
			
			// Kickstart sector loading
			if (~DRIVE_READING_PREV & DRIVE_READING)
				SECTOR_REQ <= 1;
			
			// Continue sector loading
			if (~NEXT_SECTOR_REQ_PREV & NEXT_SECTOR_REQ & DRIVE_READING)
				SECTOR_REQ <= 1;
		
			if (LOADING_STATE == 2'd0)
			begin
				if (SECTOR_REQ)
				begin
					// Request sector from HPS
					SECTOR_READY <= 0;
					sd_lba <= {8'h00, MSF_M, MSF_S, MSF_F};
					DMA_sd_rd <= 1;
					CACHE_ADDR <= 11'h000;
					LOADING_STATE <= 2'd1;
				end
			end
			else if (LOADING_STATE == 2'd1)
			begin
				if (sd_ack)
				begin
					DMA_sd_rd <= 0;
					LOADING_STATE <= 2'd2;
				end
			end
			else if (LOADING_STATE == 2'd2)
			begin
				if (sd_buff_wr)
				begin
					// Wrote first byte of pair
					CACHE_ADDR <= CACHE_ADDR + 1'b1;
					LOADING_STATE <= 2'd3;
					FORCE_WR <= 1;
				end
			
				if (~sd_ack)
				begin
					// Sector load done
					SECTOR_READY <= 1;
					LOADING_STATE <= 2'd0;
					SECTOR_REQ <= 0;
				end
			end
			else if (LOADING_STATE == 2'd3)
			begin
				// Wrote second byte of pair
				FORCE_WR <= 0;
				CACHE_ADDR <= CACHE_ADDR + 1'b1;
				LOADING_STATE <= 2'd2;
			end
			
			DMA_START_PREV <= DMA_START;
			
			if (~DMA_START_PREV & DMA_START)
			begin
				DMA_STATE <= 3'd1;
				DMA_COUNT_RUN <= DMA_COUNT;
				DMA_DONE <= 0;
			end
			
			// DMA logic
			if (DMA_STATE == 3'd1)
			begin
				// Do 68k bus request
				nBR <= 0;
				DMA_STATE <= 3'd2;
			end
			else if (DMA_STATE == 3'd2)
			begin
				// Wait for nBG low
				if (~nBG)
				begin
					nBR <= 1;	// Is this too early ? Wait for nBGACK low ?
					DMA_STATE <= 3'd3;
				end
			end
			else if (DMA_STATE == 3'd3)
			begin
				// Wait for nAS and nDTACK low
				if (~nAS & ~nDTACK)
				begin
					nBGACK <= 0;
					DMA_STATE <= 3'd4;
				end
			end
			else if (DMA_STATE == 3'd4)
			begin
				// Working...
				if (DMA_MODE == 2'd0)
				begin
					if (!DMA_COUNT_RUN)
					begin
						DMA_STATE <= 3'd0;	// Go back to idle
						nBGACK <= 1;			// Release bus
						DMA_DONE <= 1;			// Inform CDC that the transfer is finished
					end
					else
						DMA_COUNT_RUN <= DMA_COUNT_RUN - 1'b1;	// Word count, not bytes !
				end
			end
			
		end
	end
	
	wire READING = ~nAS & M68K_RW & (M68K_ADDR[23:12] == 12'hFF0) & SYSTEM_TYPE[1];
	wire WRITING = ~nAS & ~M68K_RW & (M68K_ADDR[23:12] == 12'hFF0) & SYSTEM_TYPE[1];
	
	wire LC8951_RD = (READING & (M68K_ADDR[11:2] == 10'b0001_000000));	// FF0101, FF0103
	wire LC8951_WR = (WRITING & (M68K_ADDR[11:2] == 10'b0001_000000));	// FF0101, FF0103
	
	// nAS used ?
	wire TR_ZONE_WR = CD_UPLOAD_EN & (M68K_ADDR[23:20] == 4'hE) & ~M68K_RW;
	
	// Allow writes only if the "allow write" flag of the corresponding region is set
	assign CD_TR_WR_SPR = TR_ZONE_WR & (CD_TR_AREA == 3'd0) & CD_USE_SPR;
	assign CD_TR_WR_PCM = TR_ZONE_WR & (CD_TR_AREA == 3'd1) & CD_USE_PCM;
	assign CD_TR_WR_Z80 = TR_ZONE_WR & (CD_TR_AREA == 3'd4) & CD_USE_Z80;
	assign CD_TR_WR_FIX = TR_ZONE_WR & (CD_TR_AREA == 3'd5) & CD_USE_FIX;
	
	reg [1:0] CDD_nIRQ_SR;

	// TODO: Should this be clocked by clk_sys ?
	always @(posedge CLK_68KCLK or negedge nRESET)
	begin
		if (!nRESET)
		begin
			CD_USE_SPR <= 0;
			CD_USE_PCM <= 0;
			CD_USE_Z80 <= 0;
			CD_USE_FIX <= 0;
			CD_SPR_EN <= 1;	// ?
			CD_FIX_EN <= 1;	// ?
			CD_VIDEO_EN <= 1;	// ?
			CD_UPLOAD_EN <= 0;
			CD_nRESET_Z80 <= 0;	// ?
			REG_FF0002 <= 16'h0;	// ?
			nLDS_PREV <= 1;
			nUDS_PREV <= 1;
			CD_IRQ <= 1;
			CD_IRQ_FLAGS <= 3'b111;
			DMA_START <= 0;
			
			CDD_nIRQ_SR <= 2'b11;
		end
		else
		begin
			nLDS_PREV <= nLDS;
			nUDS_PREV <= nUDS;
			CDD_nIRQ_SR <= {CDD_nIRQ_SR[0], CDD_nIRQ};
			CDC_nIRQ_PREV <= CDC_nIRQ;
			
			// Falling edge of CDD_nIRQ
			//if (CDD_nIRQ_PREV & ~CDD_nIRQ)
			if (CDD_nIRQ_SR == 2'b10)
			begin
				// Trigger CD comm. interrupt
				if (REG_FF0002[6] | REG_FF0002[4])
					CD_IRQ_FLAGS[1] <= 0;
				
				// REG_FF0002:
				// C = CD comm. interrupt
				// S = Sector ready interrupt ?
				// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
				//                 S     S     C     C
			end
			
			// Falling edge of CDC_nIRQ
			if (CDC_nIRQ_PREV & ~CDC_nIRQ)
			begin
				// Trigger CDC interrupt (decoder or transfer end)
				//if (REG_FF0002[10] | REG_FF0002[8])
					CD_IRQ_FLAGS[2] <= 0;
			end
			
			CD_IRQ <= ~&{CD_IRQ_FLAGS};
			
			// Trigger
			if (DMA_START)
				DMA_START <= 0;
			
			if (SYSTEM_TYPE[1] & ((nLDS_PREV & ~nLDS) | (nUDS_PREV & ~nUDS)))	// & PREV_nAS & ~nAS
			begin
				// Writes
				if ((M68K_ADDR[23:9] == 15'b11111111_0000000) & ~M68K_RW)
				begin
					casez ({M68K_ADDR[8:1], nUDS, nLDS})
						// FF00
						10'b0_0000001_00: REG_FF0002 <= M68K_DATA;			// FF0002
						10'b0_0000111_?0: CD_IRQ_FLAGS <= CD_IRQ_FLAGS | M68K_DATA[5:3];	// FF000E
						
						10'b0_0110000_10:			// FF0061
						begin
							if (M68K_DATA[6])
							begin
								// DMA start
								if (DMA_MICROCODE[0] == 16'hFF89)	// TOP-SP1 @ 0C119F4
								begin
									DMA_START <= 1;
									DMA_MODE <= 2'd0;		// LC8951 cache to extended RAM
								end
							end
						end
						10'b0_0110010_00: DMA_SOURCE[31:16]	<= M68K_DATA;	// FF0064~FF0065
						10'b0_0110011_00: DMA_SOURCE[15:0] <= M68K_DATA;	// FF0066~FF0067
						10'b0_0110100_00: DMA_DEST[31:16] <= M68K_DATA;		// FF0068~FF0069
						10'b0_0110101_00: DMA_DEST[15:0]	<= M68K_DATA;		// FF006A~FF006B
						10'b0_0110110_00: DMA_VALUE[31:16] <= M68K_DATA;	// FF006C~FF006D
						10'b0_0110111_00: DMA_VALUE[15:0] <= M68K_DATA;		// FF006E~FF006F
						10'b0_0111000_00: DMA_COUNT[31:16] <= M68K_DATA;	// FF0070~FF0071
						10'b0_0111001_00: DMA_COUNT[15:0] <= M68K_DATA;		// FF0072~FF0073
						10'b0_0111111_00: DMA_MICROCODE[M68K_ADDR[4:1]] <= M68K_DATA;	// FF007E+
						
						// FF01
						10'b1_0000010_?0: CD_TR_AREA <= M68K_DATA[2:0];		// FF0105 Upload area select
						10'b1_0001000_?0: CD_SPR_EN <= ~M68K_DATA[0];	// FF0111 REG_DISBLSPR
						10'b1_0001010_?0: CD_FIX_EN <= ~M68K_DATA[0];	// FF0115 REG_DISBLFIX
						10'b1_0001100_?0: CD_VIDEO_EN <= M68K_DATA[0];	// FF0119 REG_ENVIDEO
						10'b1_0010000_?0: CD_USE_SPR <= 1;	// FF0121 REG_UPMAPSPR
						10'b1_0010001_?0: CD_USE_PCM <= 1;	// FF0123 REG_UPMAPPCM
						10'b1_0010011_?0: CD_USE_Z80 <= 1;	// FF0127 REG_UPMAPZ80
						10'b1_0010100_?0: CD_USE_FIX <= 1;	// FF0129 REG_UPMAPFIX
						10'b1_0100000_?0: CD_USE_SPR <= 0;	// FF0141 REG_UPUNMAPSPR
						10'b1_0100001_?0: CD_USE_PCM <= 0;	// FF0143 REG_UPUNMAPSPR
						10'b1_0100011_?0: CD_USE_Z80 <= 0;	// FF0147 REG_UPUNMAPSPR
						10'b1_0100100_?0: CD_USE_FIX <= 0;	// FF0149 REG_UPUNMAPSPR
						10'b1_0110001_10: CDD_DIN <= M68K_DATA[3:0];		// FF0163 REG_CDDOUTPUT
						10'b1_0110010_10: HOCK <= M68K_DATA[0];			// FF0165 REG_CDDCTRL
						10'b1_0110111_?0: CD_UPLOAD_EN <= M68K_DATA[0];		// FF016F
						10'b1_1000000_?0: CD_nRESET_DRIVE <= M68K_DATA[0];	// FF0181
						10'b1_1000001_?0: CD_nRESET_Z80 <= M68K_DATA[0];	// FF0183
						10'b1_1010000_?0: CD_BANK_SPR <= M68K_DATA[1:0];	// FF01A1 REG_SPRBANK
						10'b1_1010001_?0: CD_BANK_PCM <= M68K_DATA[0];	// FF01A3 REG_PCMBANK
						default:;
					endcase
				end
				else if ((M68K_ADDR[23:20] == 4'hE) & ~M68K_RW)
				begin
					// Upload zone
					// Is this buffered or is the write directy forwarded to memory ?
					CD_TR_WR_DATA <= M68K_DATA;
					CD_TR_WR_ADDR <= M68K_ADDR[19:1];
				end
			end
		end
	end
	
	// 1111:JP, 1110:US, 1101: EU
	wire [3:0] CD_JUMPERS = {2'b11, CD_REGION};
	
	wire [7:0] CD_IRQ_VECTOR = ~CD_IRQ_FLAGS[0] ? 8'd23 :
										~CD_IRQ_FLAGS[1] ? 8'd22 :
										~CD_IRQ_FLAGS[2] ? 8'd21 :
										8'd24;	// Spurious interrupt, should not happen
	
	assign M68K_DATA = (~nAS & M68K_RW & IACK & (M68K_ADDR[3:1] == 3'd4)) ? {8'h00, CD_IRQ_VECTOR} :		// Vectored interrupt handler
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h11C) ? {3'b110, CD_LID, CD_JUMPERS, 8'h00} :	// Top mech
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h160) ? {11'b00000000_000, CDCK, CDD_DOUT} :	// REG_CDDINPUT
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h188) ? 16'h0000 :	// CDDA L
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h18A) ? 16'h0000 :	// CDDA R
							(LC8951_RD) ? {8'h00, LC8951_DOUT} :
							16'bzzzzzzzz_zzzzzzzz;
endmodule
