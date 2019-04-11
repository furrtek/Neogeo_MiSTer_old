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
	
	output CD_nRESET_Z80,
	
	output reg CD_TR_WR_SPR,
	output reg CD_TR_WR_PCM,
	output reg CD_TR_WR_Z80,
	output reg CD_TR_WR_FIX,
	output reg [2:0] CD_TR_AREA,
	output reg [1:0] CD_BANK_SPR,
	output reg CD_BANK_PCM,
	output reg [15:0] CD_TR_WR_DATA,
	output reg [19:1] CD_TR_WR_ADDR
	
	// BR, BG, BGACK
);

	reg CD_USE_SPR, CD_USE_PCM, CD_USE_Z80, CD_USE_FIX;
	reg CD_UPLOAD_EN;
	reg CD_nRESET_DRIVE;
	
	reg [31:0] DMA_SOURCE;
	reg [31:0] DMA_DEST;
	reg [31:0] DMA_VALUE;
	reg [31:0] DMA_COUNT;
	reg [15:0] DMA_MICROCODE [9];
	reg DMA_RUN;
	
	reg PREV_nAS;
	
	wire [3:0] CDD_DIN;
	wire [3:0] CDD_DOUT;
	
	cd_drive DRIVE(
		nRESET & CD_nRESET_DRIVE,
		CLK_68KCLK,
		HOCK, CDCK,
		CDD_DIN, CDD_DOUT,
		CD_nIRQ
	);

	wire [3:0] LC8953_DOUT;
	
	lc8953 LC8953(
		nRESET,
		CLK_68KCLK,
		~LC8953_WR, ~LC8953_RD, M68K_ADDR[1],
		M68K_DATA[7:0],
		LC8953_DOUT
	);
	
	wire READING = ~nAS & M68K_RW & (M68K_ADDR[23:12] == 12'hFF0) & SYSTEM_TYPE[1];
	wire WRITING = ~nAS & ~M68K_RW & (M68K_ADDR[23:12] == 12'hFF0) & SYSTEM_TYPE[1];
	
	wire LC8953_RD = (READING & (M68K_ADDR[11:2] == 10'b0001_000000));	// FF0101, FF0103
	wire LC8953_WR = (WRITING & (M68K_ADDR[11:2] == 10'b0001_000000));	// FF0101, FF0103

	// We should be detecting the falling edge of nAS and check the state of M68K_RW
	always @(negedge CLK_68KCLK or negedge nRESET)
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
			PREV_nAS <= 1;
		end
		else
		begin
			PREV_nAS <= nAS;
			
			if (CD_TR_WR_SPR) CD_TR_WR_SPR <= 0;
			if (CD_TR_WR_PCM) CD_TR_WR_PCM <= 0;
			if (CD_TR_WR_Z80) CD_TR_WR_Z80 <= 0;
			if (CD_TR_WR_FIX) CD_TR_WR_FIX <= 0;
			
			if (SYSTEM_TYPE[1] & PREV_nAS & ~nAS)
			begin
				// Writes
				if ((M68K_ADDR[23:9] == 15'b11111111_0000000) & ~M68K_RW)
				begin
					casez ({M68K_ADDR[8:1], nUDS, nLDS})
						// FF00
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
	
	assign M68K_DATA = (READING & {M68K_ADDR[11:1], 1'b0} == 12'h11C) ? {4'b1100, CD_JUMPERS, 8'h00} :	// Top mech, lid closed
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h165) ? {11'b00000000_000, CDCK, CDD_DOUT} :	// REG_CDDINPUT
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h188) ? 16'h0000 :	// CDDA L
							(READING & {M68K_ADDR[11:1], 1'b0} == 12'h18A) ? 16'h0000 :	// CDDA R
							(LC8953_RD) ? {8'h00, LC8953_DOUT} :
							16'bzzzzzzzz_zzzzzzzz;
	
endmodule
