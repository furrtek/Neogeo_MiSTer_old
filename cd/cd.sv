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
	input M68K_RW, nAS,
	input [1:0] SYSTEM_TYPE,
	input [1:0] CD_REGION,
	//output cd_download,
	//output [24:0] cd_addr,
	output reg CD_VIDEO_EN,
	output reg CD_FIX_EN,
	output reg CD_SPR_EN,
	
	output reg CD_nRESET_Z80,
	
	output reg CD_TR_WR_SPR,
	output reg CD_TR_WR_PCM,
	output reg CD_TR_WR_Z80,
	output reg CD_TR_WR_FIX,
	output reg [2:0] CD_TR_AREA,
	output reg [1:0] CD_BANK_SPR,
	output reg CD_BANK_PCM,
	output reg [15:0] CD_TR_WR_DATA,
	output reg [19:1] CD_TR_WR_ADDR,
	
	input IACK,
	output reg IPL2,
	
	output [15:0] sd_req_type,
	
	input CD_LID	// DEBUG
	
	// For DMA: BR, BG, BGACK
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
	reg [15:0] DMA_MICROCODE [9];
	reg DMA_RUN;
	
	reg CD_nIRQ_PREV;
	reg nLDS_PREV, nUDS_PREV;
	
	wire [3:0] CDD_DIN;
	wire [3:0] CDD_DOUT;
	
	cd_drive DRIVE(
		nRESET & CD_nRESET_DRIVE,
		CLK_68KCLK,
		HOCK, CDCK,
		CDD_DIN, CDD_DOUT,
		CD_nIRQ,
		sd_req_type
	);

	wire [7:0] LC8951_DOUT;
	
	lc8951 LC8951(
		nRESET,
		CLK_68KCLK,
		~LC8951_WR, ~LC8951_RD, M68K_ADDR[1],
		M68K_DATA[7:0],
		LC8951_DOUT
	);
	
	wire READING = ~nAS & M68K_RW & (M68K_ADDR[23:12] == 12'hFF0) & SYSTEM_TYPE[1];
	wire WRITING = ~nAS & ~M68K_RW & (M68K_ADDR[23:12] == 12'hFF0) & SYSTEM_TYPE[1];
	
	wire LC8951_RD = (READING & (M68K_ADDR[11:2] == 10'b0001_000000));	// FF0101, FF0103
	wire LC8951_WR = (WRITING & (M68K_ADDR[11:2] == 10'b0001_000000));	// FF0101, FF0103

	// We should be detecting the falling edge of nAS and check the state of M68K_RW
	always @(posedge CLK_68KCLK or negedge nRESET)
	begin
		if (!nRESET)
		begin
			CD_USE_SPR <= 0;
			CD_USE_PCM <= 0;
			CD_USE_Z80 <= 0;
			CD_USE_FIX <= 0;
			CD_TR_WR_SPR <= 0;
			CD_TR_WR_PCM <= 0;
			CD_TR_WR_Z80 <= 0;
			CD_TR_WR_FIX <= 0;
			CD_SPR_EN <= 1;	// ?
			CD_FIX_EN <= 1;	// ?
			CD_VIDEO_EN <= 1;	// ?
			CD_UPLOAD_EN <= 0;
			CD_nRESET_Z80 <= 0;	// ?
			REG_FF0002 <= 16'h0;	// ?
			nLDS_PREV <= 1;
			nUDS_PREV <= 1;
			IPL2 <= 1;
			CD_IRQ_FLAGS <= 3'b111;
		end
		else
		begin
			nLDS_PREV <= nLDS;
			nUDS_PREV <= nUDS;
			CD_nIRQ_PREV <= CD_nIRQ;
			
			if (CD_TR_WR_SPR) CD_TR_WR_SPR <= 0;
			if (CD_TR_WR_PCM) CD_TR_WR_PCM <= 0;
			if (CD_TR_WR_Z80) CD_TR_WR_Z80 <= 0;
			if (CD_TR_WR_FIX) CD_TR_WR_FIX <= 0;
			
			// Falling edge of CD_nIRQ
			if (CD_nIRQ_PREV & ~CD_nIRQ)
			begin
				// CD comm. interrupt enable
				if (REG_FF0002[6] | REG_FF0002[4])
					CD_IRQ_FLAGS[1] <= 0;
			end
			
			IPL2 <= &{CD_IRQ_FLAGS};
			
			if (SYSTEM_TYPE[1] & ((nLDS_PREV & ~nLDS) | (nUDS_PREV & ~nUDS)))	// & PREV_nAS & ~nAS
			begin
				// Writes
				if ((M68K_ADDR[23:9] == 15'b11111111_0000000) & ~M68K_RW)
				begin
					casez ({M68K_ADDR[8:1], nUDS, nLDS})
						// FF00
						10'b0_0000001_00: REG_FF0002 <= M68K_DATA;			// FF0002
						10'b0_0000111_?0: CD_IRQ_FLAGS <= CD_IRQ_FLAGS | M68K_DATA[5:3];	// FF000E
						
						10'b0_0110000_10: DMA_RUN <= M68K_DATA[6];			// FF0061
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
					
					// Allow writes only if the "allow write" flag of the corresponding region is set
					if (CD_UPLOAD_EN)
					begin
						if ((CD_TR_AREA == 3'd0) & CD_USE_SPR)
							CD_TR_WR_SPR <= 1;
						else if ((CD_TR_AREA == 3'd1) & CD_USE_PCM)
							CD_TR_WR_PCM <= 1;
						else if ((CD_TR_AREA == 3'd4) & CD_USE_Z80)
							CD_TR_WR_Z80 <= 1;
						else if ((CD_TR_AREA == 3'd5) & CD_USE_FIX)
							CD_TR_WR_FIX <= 1;
					end
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
	
	assign M68K_DATA = (IACK & (M68K_ADDR[3:1] == 3'd4)) ? {8'h00, CD_IRQ_VECTOR} :		// Vectored interrupt handler
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h11C) ? {3'b110, CD_LID, CD_JUMPERS, 8'h00} :	// Top mech
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h161) ? {11'b00000000_000, CDCK, CDD_DOUT} :	// REG_CDDINPUT
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h188) ? 16'h0000 :	// CDDA L
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h18A) ? 16'h0000 :	// CDDA R
							(LC8951_RD) ? {8'h00, LC8951_DOUT} :
							16'bzzzzzzzz_zzzzzzzz;
endmodule
