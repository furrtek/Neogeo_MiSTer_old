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

// TODO: Check if ZMC is working correctly
// TODO: See if Fatal Fury 2 is still working passed stage 1

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [44:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)
	input         TAPE_IN,

	// SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,

	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE
);

assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign AUDIO_S   = 1;		// Signed
assign AUDIO_MIX = status[5:4];
assign AUDIO_L = snd_left;
assign AUDIO_R = snd_right;

assign LED_USER  = ioctl_download;
assign LED_DISK  = 0;
assign LED_POWER = 0;

assign VIDEO_ARX = 8'd10;	// 320/32
assign VIDEO_ARY = 8'd7;	// 224/32

assign VGA_DE = ~CHBL & nBNKB;

// status bit definition:
// 31       23       15       7
// --AA--SS -------- L--CGGDD DEEMVTTR
// R:	status[0]		Reset, used by the HPS, keep it there
// T:	status[2:1]		System type, 0=Console, 1=Arcade, 2=CD, 3=CDZ
// V: status[3]		Video mode
// M: status[4]		Memory card presence
// E:	status[6:5]		Stereo mix
// D:	status[9:7]		DIP switches
// G:	status[11:10]	Neo CD region
// C:	status[12]		Save memory card & backup RAM
// L:	status[12]		CD lid state (DEBUG)
// S:	status[25:24]	Special chip type, 0=None, 1=PRO-CT0, 2=Link MCU
// A: status[29:28]	Sprite tile # remap hack, 0=no remap, 1=kof95, 2=whp, 3=kizuna

`include "build_id.v"

// Common to all systems
localparam CONF_STR1 = {
	"NEOGEO;;",
	"-;"
};

localparam CONF_STR2_CART = {
	"S,EP1P1,Load romset;"	// F
};
localparam CONF_STR2_CD = {
	"0,ISOBIN,Load ISO CD Image;"	// S
};

// Common to all systems
localparam CONF_STR3 = {
	"-;",
	"O12,System type,Console,Arcade,CD,CDZ;",
	"O3,Video mode,NTSC,PAL;",
	"RC,Save memory card;"
};

localparam CONF_STR4_CART = {
	"4,Memory card,Plugged,Unplugged;",	// O
};
localparam CONF_STR4_CD = {
	"AB,Region,US,EU,JP,AS;",	// O
};
localparam CONF_STR4B_CD = {
	"F,CD lid,Opened,Closed;",	// O
};

localparam CONF_STR5_MVS = {
	"7,DIP:Settings,OFF,ON;"	// O
};
localparam CONF_STR6_MVS = {
	"8,DIP:Freeplay,OFF,ON;"	// O
};
localparam CONF_STR7_MVS = {
	"9,DIP:Freeze,OFF,ON;",		// O
};

// Common to all systems
localparam CONF_STR8 = {
	"-;",
	"O56,Stereo mix,none,25%,50%,100%;",
	"R0,Reset & apply;",
	"J1,A,B,C,D,Start,Select,Coin,ABC;",	// ABC is a special key to press A+B+C at once, useful for
	"V,v",`BUILD_DATE								// keyboards that don't allow more than 2 keypresses at once
};


////////////////////   CLOCKS   ///////////////////

wire locked;
wire clk_sys;

// 50MHz in, 5*24=120MHz out
// CAS latency = 3 (20.8ns)
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(SDRAM_CLK),	// Phase shifted
	.locked(locked)
);

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL = CLK_6MB;
wire nRST = ~(ioctl_download | status[0]);

// The watchdog should output nRESET but it makes video sync stop for a moment, so the
// MiSTer OSD jumps around. Provide an indication for devs that a watchdog reset happened ?
//wire nRESET = nRST;
reg nRESET;
reg nRST_PREV_24M;
reg [15:0] TRASH_ADDR;

always @(posedge CLK_24M)
begin
	nRST_PREV_24M <= nRST;
	// nRST rising edge: start RAM trashing
	if (~nRST_PREV_24M & nRST)
	begin
		TRASHING <= 1;
		TRASH_ADDR <= 16'h0000;
	end
	// nRST falling edge: put system in reset
	if (nRST_PREV_24M & ~nRST)
		nRESET <= 0;
	
	if (TRASHING)
	begin
		if (TRASH_ADDR == 16'hFFFF)
		begin
			// Trashing done: release system reset
			TRASHING <= 0;
			nRESET <= 1;
		end
		else
			TRASH_ADDR <= TRASH_ADDR + 1'b1;
	end
end

wire CLK_24M;
reg [2:0] counter_p = 0;	// 0~4
reg diva, divb;
reg nRST_PREV;
reg [1:0] SYSTEM_TYPE;

always @(posedge clk_sys)
begin
	nRST_PREV <= nRST;
	
	if ({nRST_PREV, nRST} == 2'b01)
	begin
		// Rising edge of nRST
		counter_p <= 3'd0;
		diva <= 1'd0;
	end
	else
	begin
		if (counter_p == 3'd4)
			counter_p <= 3'd0;
		else
			counter_p <= counter_p + 3'd1;
		
		if (counter_p == 3'd0)
			diva <= ~diva;
	end
end

always @(negedge clk_sys)
begin	
	if ({nRST_PREV, nRST} == 2'b01)
	begin
		// Rising edge of nRST
		SYSTEM_TYPE <= status[2:1];	// Latch the system type on reset
		divb <= 1'd0;
	end
	else
	begin
		if (counter_p == 3'd3)
			divb <= ~divb;
	end
end

assign CLK_24M = diva ^ divb;	// Glitch-less divide-by-5

//////////////////   HPS I/O   ///////////////////

// VD 0: CD bin file
// VD 1: CD cue file, let MiSTer binary take care of it, do not touch !
// VD 2: Save file
wire [2:0] img_mounted;
wire [2:0] sd_rd;
wire [2:0] sd_wr;

wire sd_ack, sd_buff_wr, img_readonly;

wire [7:0] sd_buff_addr;	// Address inside 256-word sector
wire [15:0] sd_buff_dout;
wire [15:0] sd_buff_din;
wire [15:0] sd_req_type;
wire [63:0] img_size;
reg ioctl_download_prev;
reg memcard_load = 0;
reg memcard_ena = 0;
reg memcard_loading = 0;
reg memcard_state = 0;
reg memcard_save_prev = 0, sd_ack_prev;
reg [31:0] sd_lba;
wire [31:0] CD_sd_lba;

wire [15:0] joystick_0;	// ----HNLS DCBAUDLR
wire [15:0] joystick_1;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire [31:0] status;

wire [64:0] rtc;

wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire [15:0] ioctl_dout;
wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        ioctl_wait;

wire SYSTEM_MVS = (SYSTEM_TYPE == 2'd1);
wire SYSTEM_CDx = SYSTEM_TYPE[1];

// Conditional modification of the CONF strings chaining according to chosen system type
wire [7:0] char_cart_file = SYSTEM_CDx ? "+" : "F";
wire [7:0] char_cd_img = SYSTEM_CDx ? "S" : "+";
wire [7:0] char_cart_option = SYSTEM_CDx ? "+" : "O";
wire [7:0] char_mvs_option = SYSTEM_MVS ? "O" : "+";
wire [7:0] char_cd_option = SYSTEM_CDx ? "O" : "+";

//                   Con Arc CD CDz
// SYSTEM_TYPE       00  01  10 11
// char_cart_file    F   F   +  +
// char_cd_img       +   +   S  S
// char_cart_option  O   O   +  +
// char_mvs_option   +   O   +  +
// char_cd_option    +   +   O  O

hps_io #(
	.STRLEN(
		($size(CONF_STR1)>>3) +
		($size(CONF_STR2_CART)>>3) +
		($size(CONF_STR2_CD)>>3) +
		($size(CONF_STR3)>>3) +
		($size(CONF_STR4_CART)>>3) +
		($size(CONF_STR4_CD)>>3) +
		($size(CONF_STR4B_CD)>>3) +
		($size(CONF_STR5_MVS)>>3) +
		($size(CONF_STR6_MVS)>>3) +
		($size(CONF_STR7_MVS)>>3) +
		($size(CONF_STR8)>>3) + 8), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str({
		CONF_STR1,
		char_cart_file, CONF_STR2_CART,
		char_cd_img, CONF_STR2_CD,
		CONF_STR3,
		char_cart_option, CONF_STR4_CART,
		char_cd_option, CONF_STR4_CD,
		char_cd_option, CONF_STR4B_CD,
		char_mvs_option, CONF_STR5_MVS,
		char_mvs_option, CONF_STR6_MVS,
		char_mvs_option, CONF_STR7_MVS,
		CONF_STR8
	}),

	//.ps2_mouse(ps2_mouse),	// Could be used for The Irritating Maze ?
	
	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.buttons(buttons),			// DE10 buttons ?
	.status(status),				// status read (32 bits)
	.RTC(rtc),
	
	// Loading signals
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),
	
	.sd_lba(sd_req_type ? CD_sd_lba : sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	.sd_req_type(sd_req_type),
	
	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size)
);
	
//////////////////   Her Majesty   ///////////////////

	wire [15:0] snd_right;
	wire [15:0] snd_left;
	wire sdram_ready, ready_fourth;
	reg  [24:0] sdram_addr;
	
	wire nRESETP, nSYSTEM, CARD_WE, SHADOW, nVEC, nREGEN, nSRAMWEN, PALBNK;
	
	// Clocks
	wire CLK_12M, CLK_68KCLK, CLK_68KCLKB, CLK_8M, CLK_6MB, CLK_4M, CLK_4MB, CLK_1HB;
	
	// 68k stuff
	wire [15:0] M68K_DATA;
	wire [23:1] M68K_ADDR;
	wire A22Z, A23Z;
	wire M68K_RW, nAS, nLDS, nUDS, nDTACK, nHALT;
	wire [15:0] M68K_DATA_BYTE_MASK;
	wire [15:0] FX68K_DATAIN;
	wire [15:0] FX68K_DATAOUT;
	wire IPL0, IPL1;
	reg [1:0] P_BANK;
	
	// RTC stuff
	wire RTC_DOUT, RTC_DIN, RTC_CLK, RTC_STROBE, RTC_TP;
	
	// OEs and WEs
	wire nSROMOEL, nSROMOEU, nSROMOE;
	wire nROMOEL, nROMOEU;
	wire nPORTOEL, nPORTOEU, nPORTWEL, nPORTWEU, nPORTADRS;
	wire nSRAMOEL, nSRAMOEU, nSRAMWEL, nSRAMWEU;
	wire nWRL, nWRU, nWWL, nWWU;
	wire nLSPOE, nLSPWE;
	wire nPAL, nPAL_WE;
	wire nBITW0, nBITW1, nBITWD0, nDIPRD0, nDIPRD1;
	
	// RAM outputs
	wire [7:0] WRAML_OUT;
	wire [7:0] WRAMU_OUT;
	wire [7:0] SRAML_OUT;
	wire [7:0] SRAMU_OUT;
	
	// Memory card stuff
	wire [23:0] CDA;
	wire [2:0] BNK;
	wire [7:0] CDD;
	wire nCD1, nCD2;
	wire nCRDO, nCRDW, nCRDC;
	wire nCARDWEN, CARDWENB;
	
	// Z80 stuff
	wire [7:0] SDD;
	wire [15:0] SDA;
	wire nSDRD, nSDWR, nMREQ, nIORQ;
	wire nZ80INT, nZ80NMI, nSDW, nSDZ80R, nSDZ80W, nSDZ80CLR;
	wire nSDROM, nSDMRD, nSDMWR, SDRD0, SDRD1, nZRAMCS;
	wire n2610CS, n2610RD, n2610WR;
	
	// Graphics stuff
	wire [23:0] PBUS;
	wire [7:0] LO_ROM_DATA;
	wire nPBUS_OUT_EN;
	
	wire [19:0] C_LATCH;
	reg [3:0] C_LATCH_EXT;
	reg [63:0] CR_DOUBLE;
	wire [24:0] CROM_ADDR;
	
	wire [15:0] S_LATCH;
	wire [7:0] FIXD;
	
	wire CWE, BWE, BOE;
	
	wire [14:0] SLOW_VRAM_ADDR;
	reg [15:0] SLOW_VRAM_DATA_IN;
	wire [15:0] SLOW_VRAM_DATA_OUT;
	
	wire [10:0] FAST_VRAM_ADDR;
	wire [15:0] FAST_VRAM_DATA_IN;
	wire [15:0] FAST_VRAM_DATA_OUT;
	
	wire [11:0] PAL_RAM_ADDR;
	wire [15:0] PAL_RAM_DATA;
	reg [15:0] PAL_RAM_REG;
	
	wire PCK1, PCK2, EVEN1, EVEN2, LOAD, H;
	wire DOTA, DOTB;
	wire CA4, S1H1, S2H1;
	wire CHBL, nBNKB, VCS;
	wire CHG, LD1, LD2, SS1, SS2;
	wire [3:0] GAD;
	wire [3:0] GBD;
	wire [3:0] WE;
	wire [3:0] CK;
	
	// SDRAM multiplexing stuff
	reg [1:0] SDRAM_M68K_SIG_SR;
	reg [1:0] SDRAM_CROM_SIG_SR;
	reg [1:0] SDRAM_SROM_SIG_SR;
	//reg [1:0] CD_WR_SDRAM_SIG_SR;
	reg [15:0] SROM_DATA;
	//reg [31:0] SROM_DATA;
	reg [15:0] PROM_DATA;
	//reg [63:0] PROM_DATA_QUAD;
	reg M68K_RD_REQ, SROM_RD_REQ, CROM_RD_REQ;
	reg M68K_RD_RUN, SROM_RD_RUN, CROM_RD_RUN;
	reg SDRAM_RD_PULSE, SDRAM_WR_PULSE;
	reg [1:0] SDRAM_READY_SR;
	//reg [1:0] SDRAM_READY_SECOND_SR;
	reg [1:0] SDRAM_READY_FOURTH_SR;
	//reg [24:0] M68K_RD_ADDR_CACHED;
	//wire [24:0] M68K_RD_ADDR_SDRAM;
	
	reg CD_WR_REQ;
	reg CD_WR_RUN;

	parameter INDEX_SPROM = 0;
	parameter INDEX_LOROM = 1;
	parameter INDEX_SFIXROM = 2;
	parameter INDEX_P1ROM_A = 4;
	parameter INDEX_P1ROM_B = 5;
	parameter INDEX_P2ROM = 6;
	parameter INDEX_S1ROM = 8;
	parameter INDEX_M1ROM = 9;
	parameter INDEX_CROMS = 64;
	
	wire memcard_save = status[12];
	wire [1:0] cart_chip = status[25:24];
	wire video_mode = status[3];
	
	// Memory card and backup ram image save/load
	always @(posedge clk_sys)
	begin
		ioctl_download_prev <= ioctl_download;
		
		// Disable memcard operations on download start
		if(~ioctl_download_prev & ioctl_download)
			memcard_ena <= 0;

		// Enable memcard operations while not downloading, 'disk' #2 is mounted, has a != 0 size, and is writable
		if(~ioctl_download && img_mounted[2] && img_size && ~img_readonly)
			memcard_ena <= 1;

		// Start memcard image load on download end
		if (ioctl_download_prev & ~ioctl_download)
			memcard_load <= 1;
		else if (memcard_state)
			memcard_load <= 0;

		memcard_save_prev <= memcard_save;
		sd_ack_prev <= sd_ack;
		
		if (~sd_ack_prev & sd_ack)
			{sd_rd[2], sd_wr[2]} <= 2'b00;
		else
		begin
			if (!memcard_state)
			begin
				// No current memcard operation
				if (memcard_ena & (memcard_load | (~memcard_save_prev & memcard_save)))
				begin
					// Start operation
					memcard_state <= 1;
					memcard_loading <= memcard_load;
					sd_lba <= 32'd0;
					sd_rd[2] <= memcard_load;
					sd_wr[2] <= ~memcard_load;
				end
			end
			else
			begin
				if (sd_ack_prev & ~sd_ack)
				begin
					// On ack falling edge, either increase lba and continue, or stop operation
					if ((SYSTEM_CDx & (sd_lba[7:0] >= 8'd15)) |	// 8kB / 256 words per sector / 2 bytes per word
						(~SYSTEM_CDx & (sd_lba[7:0] >= 8'd131)))	// (64kB + 2kB) / 256 words per sector / 2 bytes per word
					begin
						memcard_loading <= 0;	// Useless ?
						memcard_state <= 0;
					end
					else
					begin
						sd_lba <= sd_lba + 1'd1;
						sd_rd[2] <= memcard_loading;
						sd_wr[2] <= ~memcard_loading;
					end
				end
			end
		end
	end
	
	wire [1:0] CD_REGION;
	
	always @(status[11:10])
	begin
		// CD Region code remap:
		// status[11:10] remap
		// 00            10 US
		// 01            01 EU
		// 10            11 JP
		// 11            00 ASIA
		case(status[11:10])
			 2'b00: CD_REGION = 2'b10;
			 2'b01: CD_REGION = 2'b01;
			 2'b10: CD_REGION = 2'b11;
			 2'b11: CD_REGION = 2'b00;
		endcase
	end
	
	wire [2:0] CD_TR_AREA;	// Transfer area code
	wire [15:0] CD_TR_WR_DATA;
	wire [19:1] CD_TR_WR_ADDR;
	reg [24:0] CD_REMAP_TR_ADDR;
	wire [1:0] CD_BANK_SPR;
	
	cd_sys cdsystem(
		nRESET,
		CLK_68KCLK,
		M68K_ADDR,
		M68K_DATA,
		A22Z, A23Z,
		nLDS, nUDS,
		M68K_RW, nAS, nDTACK,
		SYSTEM_TYPE,
		CD_REGION,
		CD_VIDEO_EN,
		CD_FIX_EN,
		CD_SPR_EN,
		
		CD_nRESET_Z80,
		
		CD_TR_WR_SPR,
		CD_TR_WR_PCM,
		CD_TR_WR_Z80,
		CD_TR_WR_FIX,
		CD_TR_AREA,
		CD_BANK_SPR,
		CD_BANK_PCM,
		CD_TR_WR_DATA,
		CD_TR_WR_ADDR,
		
		IACK,
		CD_IRQ,
		
		clk_sys,
		sd_req_type,
		sd_rd[0],
		sd_ack,
		sd_buff_dout,
		sd_buff_wr,
		CD_sd_lba,
		
		status[15],	// CD lid state (DEBUG)
		
		nBR, nBG, nBGACK
	);
	
	// The P1 zone is writable on the Neo CD
	// Is there a write enable register for it ?
	wire CD_WR_EXT = ~|{A23Z, A22Z, M68K_ADDR[21]} & ~M68K_RW & ~nAS;	// Writes to $000000~$1FFFFF
	
	wire CD_WR_SDRAM_SIG = SYSTEM_CDx & |{CD_TR_WR_SPR, CD_TR_WR_FIX, CD_WR_EXT};
	
	// SDRAM stuff
	wire nROMOE = nROMOEL & nROMOEU;
	wire nPORTOE = nPORTOEL & nPORTOEU;
	// CD system work ram is in SDRAM
	wire CD_EXT_RD = SYSTEM_CDx & (~nWRL | ~nWRU);
	wire SDRAM_M68K_SIG = ~&{nSROMOE, nROMOE, nPORTOE} | CD_EXT_RD;
	
	reg SDRAM_WR_BYTE_MODE;
	reg nLDS_PREV, nUDS_PREV;
	
	//wire M68K_CACHE_MISS = (M68K_RD_ADDR_CACHED[24:3] != M68K_RD_ADDR_SDRAM[24:3]);
	
	always @(posedge clk_sys)
	begin
		if (!nRESET)
		begin
			CROM_RD_REQ <= 0;
			SROM_RD_REQ <= 0;
			M68K_RD_REQ <= 0;
			CD_WR_REQ <= 0;
			CROM_RD_RUN <= 0;
			SROM_RD_RUN <= 0;
			M68K_RD_RUN <= 0;
			CD_WR_RUN <= 0;
			SDRAM_WR_BYTE_MODE <= 0;
			nLDS_PREV <= 1;
			nUDS_PREV <= 1;
			//M68K_RD_ADDR_CACHED[3:0] <= 4'b1111;	// Force cache miss on startup
		end
		else
		begin
			
			// Detect falling edge of nLDS or nUDS while CD_WR_SDRAM_SIG is high
			nLDS_PREV <= nLDS;
			nUDS_PREV <= nUDS;
			//CD_WR_SDRAM_SIG_SR <= {CD_WR_SDRAM_SIG_SR[0], CD_WR_SDRAM_SIG};
			//if ((CD_WR_SDRAM_SIG_SR == 2'b01) & nRESET)
			if (((nLDS_PREV & ~nLDS) | (nUDS_PREV & ~nUDS)) & CD_WR_SDRAM_SIG & nRESET)
			begin
				// Convert and latch address
				casez({CD_WR_EXT, CD_TR_AREA})
					4'b1_???: CD_REMAP_TR_ADDR <= {4'b0_001, M68K_ADDR[20:1], ~nLDS};	// EXT zone SDRAM
					4'b0_000: CD_REMAP_TR_ADDR <= {3'b0_10, CD_BANK_SPR, M68K_ADDR[19:7], M68K_ADDR[5:2], ~M68K_ADDR[6], ~M68K_ADDR[1], 1'b0};	// Sprites SDRAM
					//4'b0_001: CD_REMAP_TR_ADDR <= {4'b0_000, CD_BANK_PCM, CD_TR_WR_ADDR, 1'b0};	// ADPCM DDRAM
					4'b0_101: CD_REMAP_TR_ADDR <= {8'b0_0000_100, M68K_ADDR[17:6], M68K_ADDR[3:1], ~M68K_ADDR[5], ~M68K_ADDR[4]};	// Fix SDRAM
					//4'b0_100: CD_REMAP_TR_ADDR <= {8'b0_0000_000, CD_TR_WR_ADDR[16:1], 1'b0};		// Z80 BRAM
					default: CD_REMAP_TR_ADDR <= 25'h0000000;
				endcase
				
				/*
				Normal fix data bytes:
				10 18 00 08
				11 19 01 09
				12 1A 02 0A
				13 1B 03 0B
				14 1C 04 0C
				15 1D 05 0D
				16 1E 06 0E
				17 1F 07 0F
				
				SDRAM organization words:
				10-18 00-08 11-19 01-09 12-1A 02-0A 13-1B 03-0B...
				
				The Neo CD will write fix data as bytes so two byte writes would have to be grouped in
				one SDRAM word write, but that won't work with the byte address remapping. It would require
				a 32 byte buffer which would be burst written and it would be a mess...
				Instead of that, just use LDQM/UDQM to do byte writes with the remapped address directly.
				
				addr_out = ?, addr_in[...:0], ~addr_in[4], addr_in[3]
				00 -> 02
				01 -> 06
				02 -> 0A
				...
				07 -> 1E
				
				08 -> 03
				09 -> 07
				...
				
				10 -> 00
				11 -> 04
				...
				
				18 -> 01
				19 -> 05
				*/
				
				// Warning: this is fed RIGHT NOW to the sdram controller. If the write cycle starts now, it's fine,
				// but if the write cycle is reported, the byte mode could affect the currently running cycle !
				// Is this dangerous ? are CD uploads spaced enough to not cause any problems ?
				SDRAM_WR_BYTE_MODE <= ((CD_TR_AREA == 3'd5) & ~CD_WR_EXT) | (CD_WR_EXT & (nLDS ^ nUDS));	// Fix or extended RAM data
				
				if (~|{M68K_RD_REQ, M68K_RD_RUN, SROM_RD_RUN, CROM_RD_RUN} & sdram_ready)
				begin
					// Start write cycle right now
					CD_WR_RUN <= 1;
					SDRAM_WR_PULSE <= 1;
				end
				else
					CD_WR_REQ <= 1;	// Set request flag for later
			end
			
			// Detect rising edge of SDRAM_M68K_SIG
			SDRAM_M68K_SIG_SR <= {SDRAM_M68K_SIG_SR[0], SDRAM_M68K_SIG};
			if ((SDRAM_M68K_SIG_SR == 2'b01) & nRESET)	// & M68K_CACHE_MISS)
			begin
				if (~|{SROM_RD_REQ, SROM_RD_RUN, CROM_RD_RUN, CROM_RD_REQ} & sdram_ready)
				begin
					// Start M68K read cycle right now
					M68K_RD_RUN <= 1;
					SDRAM_RD_PULSE <= 1;
				end
				else
					M68K_RD_REQ <= 1;	// Set request flag for later
			end
			
			// Detect rising edge of PCK1B
			// CA4's polarity changes depending on the tile's h-flip attribute
			// Normal: CA4 high, then low
			// Flipped: CA4 low, then high
			SDRAM_CROM_SIG_SR <= {SDRAM_CROM_SIG_SR[0], ~PCK1};
			if ((SDRAM_CROM_SIG_SR == 2'b01) & nRESET)
			begin
				if (~|{M68K_RD_REQ, SROM_RD_REQ, M68K_RD_RUN, SROM_RD_RUN} & sdram_ready)
				begin
					// Start C ROM read cycle right now
					CROM_RD_RUN <= SPR_EN;
					SDRAM_RD_PULSE <= SPR_EN;
				end
				else
					CROM_RD_REQ <= 1;	// Set request flag for later
			end
			
			// Detect rising edge of PCK2B
			// See dev_notes.txt about why there's only one read for FIX graphics
			// regardless of the S2H1 signal
			SDRAM_SROM_SIG_SR <= {SDRAM_SROM_SIG_SR[0], ~PCK2};
			if ((SDRAM_SROM_SIG_SR == 2'b01) & nRESET)
			begin
				if (~|{M68K_RD_REQ, M68K_RD_RUN, CROM_RD_RUN, CROM_RD_REQ} & sdram_ready)
				begin
					// Start S ROM read cycle right now
					SROM_RD_RUN <= FIX_EN;
					SDRAM_RD_PULSE <= FIX_EN;
				end
				else
					SROM_RD_REQ <= 1;	// Set request flag for later
			end
			
			if (SDRAM_WR_PULSE)
				SDRAM_WR_PULSE <= 0;
			
			if (SDRAM_RD_PULSE)
			begin
				SDRAM_RD_PULSE <= 0;
				// Save start address of cached data 1 clock after M68K_RD_RUN is set
				// to let sdram_addr stabilize (comb logic)
				//if (M68K_RD_RUN)
				//	M68K_RD_ADDR_CACHED <= M68K_RD_ADDR_SDRAM;
			end
			
			if (sdram_ready & ~SDRAM_RD_PULSE & ~SDRAM_WR_PULSE)
			begin
				// Start requested access, if needed
				if (CD_WR_REQ && !CROM_RD_RUN && !M68K_RD_RUN && !SROM_RD_RUN)
				begin
					CD_WR_REQ <= 0;
					CD_WR_RUN <= 1;
					SDRAM_WR_PULSE <= 1;
				end
				else if (CROM_RD_REQ && !CD_WR_RUN && !M68K_RD_RUN && !SROM_RD_RUN)
				begin
					CROM_RD_REQ <= 0;
					CROM_RD_RUN <= SPR_EN;
					SDRAM_RD_PULSE <= SPR_EN;
				end
				else if (SROM_RD_REQ && !CD_WR_RUN && !M68K_RD_RUN && !CROM_RD_RUN)
				begin
					SROM_RD_REQ <= 0;
					SROM_RD_RUN <= FIX_EN;
					SDRAM_RD_PULSE <= FIX_EN;
				end
				else if (M68K_RD_REQ && !CD_WR_RUN && !SROM_RD_RUN && !CROM_RD_RUN)
				begin
					M68K_RD_REQ <= 0;
					M68K_RD_RUN <= 1;
					SDRAM_RD_PULSE <= 1;
				end
			end
			
			// Terminate running access, if needed
			// Having two non-nested IF statements with the & in the condition
			// prevents synthesis from chaining too many muxes and causing
			// timing analysis to fail
			SDRAM_READY_SR <= {SDRAM_READY_SR[0], sdram_ready};
			if ((SDRAM_READY_SR == 2'b01) & CD_WR_RUN)
			begin
				CD_WR_RUN <= 0;
			end
			if ((SDRAM_READY_SR == 2'b01) & SROM_RD_RUN)
			begin
				SROM_DATA <= sdram_dout[63:48];
				SROM_RD_RUN <= 0;
			end
			if ((SDRAM_READY_SR == 2'b01) & M68K_RD_RUN)
			begin
				PROM_DATA <= sdram_dout[63:48];
				M68K_RD_RUN <= 0;
			end
			
			SDRAM_READY_FOURTH_SR <= {SDRAM_READY_FOURTH_SR[0], ready_fourth};
			if (SDRAM_READY_FOURTH_SR == 2'b01)
			begin
				/*if (M68K_RD_RUN)
				begin
					PROM_DATA_QUAD <= sdram_dout;
					M68K_RD_RUN <= 0;
				end*/
				if (CROM_RD_RUN)
				begin
					CR_DOUBLE <= sdram_dout;
					CROM_RD_RUN <= 0;
				end
			end
		end
	end
	
	wire [63:0] sdram_dout;
	wire [15:0] sdram_din = ioctl_download ? ioctl_dout :
										CD_WR_RUN ? M68K_DATA :
										16'h0000;
	wire sdram_rd = ioctl_download ? 1'b0 : SDRAM_RD_PULSE;
	// ioctl_download is used to load the system ROM on CD systems, we need it !
	wire sdram_we = SYSTEM_CDx ? (ioctl_download & (ioctl_index == INDEX_SPROM)) ? ioctl_wr : SDRAM_WR_PULSE :
							(ioctl_download & (ioctl_index != INDEX_LOROM) & (ioctl_index != INDEX_M1ROM)) ? ioctl_wr : 1'b0;
	
	gap_hack GAPHACK(
		{C_LATCH_EXT, C_LATCH[19:4]},
		C_LATCH[3:0],
		status[29:28],
		CROM_ADDR
	);
	
	wire [22:0] P2ROM_ADDR = {P_BANK + 3'd3, M68K_ADDR[19:1], 1'b0};
	
	wire [24:0] ioctl_addr_offset =
		(ioctl_index == INDEX_SPROM) ?	{8'b0_0000_0, ioctl_addr[18:0]} :	// System ROM
		(ioctl_index == INDEX_S1ROM) ?	{6'b0_0000_1, ioctl_addr[18:0]} :	// S1
		(ioctl_index == INDEX_SFIXROM) ? {8'b0_0001_000, ioctl_addr[16:0]} :	// SFIX
		(ioctl_index == INDEX_P1ROM_A) ? {5'b0_0010, ioctl_addr[19:0]} :		// P1 first half or full
		(ioctl_index == INDEX_P1ROM_B) ? {6'b0_0010_1, ioctl_addr[18:0]} :	// P1 second half
		(ioctl_index == INDEX_P2ROM) ? ioctl_addr + 25'h0300000 :				// P2+
		(ioctl_index >= INDEX_CROMS) ? {ioctl_addr[23:0], 1'b0} + {ioctl_index[5:1], 18'h00000, ioctl_index[0], 1'b0} + 25'h0800000 : // C*
		25'h0000000;
	
	// sdram_addr is 25 bits, LSB is = 0 in word mode
	always_comb begin 
		casez ({ioctl_download, CD_WR_RUN, CROM_RD_RUN, SROM_RD_RUN, ~nROMOE & M68K_RD_RUN, (CD_EXT_RD | ~nPORTOE) & M68K_RD_RUN, ~nSROMOE & M68K_RD_RUN})
			// HPS loading pass-through
			7'b1zzzzzz: sdram_addr = ioctl_addr_offset;
			// CD transfer
			7'b01zzzzz: sdram_addr = CD_REMAP_TR_ADDR;
			// C ROMs Bytes $0800000~$1FFFFFF
			7'b001zzzz: sdram_addr = CROM_ADDR;
			// S ROM $0080000~$009FFFF or SFIX ROM (cart) $0100000~$011FFFF
			7'b0001zzz: sdram_addr = (nSYSTEM_G | SYSTEM_CDx) ?
												{8'b0_0000_100, S_LATCH[15:4], S_LATCH[2:0], ~S_LATCH[3], 1'b0} :
												{8'b0_0001_000, S_LATCH[15:4], S_LATCH[2:0], ~S_LATCH[3], 1'b0};
			// P1 ROM $0200000~$02FFFFF
			7'b00001zz: sdram_addr = {5'b0_0010, M68K_ADDR[19:1], 1'b0};
			// Extended RAM (CD)	$0300000~$03FFFFF
			// P2 ROM (cart)		$0300000~$07FFFFF bankswitched
			7'b000001z: sdram_addr = {2'b0_0, P2ROM_ADDR};
			// System ROM (CD)	$0000000~$007FFFF
			// System ROM (cart)	$0000000~$001FFFF
			7'b0000001: sdram_addr = SYSTEM_CDx ? {6'b0_0000_0, M68K_ADDR[18:1], 1'b0} :
															{8'b0_0000_000, M68K_ADDR[16:1], 1'b0};
			// Default
			7'b0000000: sdram_addr = 25'h0000000;
		endcase
	end
	
	sdram ram
	(
		.*,					// Connect all nets with the same names (SDRAM_* pins)
		.init(~locked),	// Init SDRAM as soon as the PLL is locked
		.clk(clk_sys),
		.dout(sdram_dout),
		.din(sdram_din),
		.addr(sdram_addr),
		.wtbt((ioctl_download | ~SDRAM_WR_BYTE_MODE) ? 2'b11 : 2'b00),		// Always used in 16-bit mode except for CD fix data write
		.we(sdram_we),
		.rd(sdram_rd),
		.ready_first(sdram_ready),
		.ready_fourth(ready_fourth)
	);
	
	neo_d0 D0(CLK_24M, nRESET, nRESETP, CLK_12M, CLK_68KCLK, CLK_68KCLKB, CLK_6MB, CLK_1HB, M68K_ADDR[4],
				nBITWD0, M68K_DATA[5:0], SDA[15:11], SDA[4:2], nSDRD, nSDWR, nMREQ, nIORQ, nZ80NMI, nSDW, nSDZ80R,
				nSDZ80W, nSDZ80CLR, nSDROM, nSDMRD, nSDMWR, SDRD0, SDRD1, n2610CS, n2610RD, n2610WR, nZRAMCS,
				BNK);
	
	// Re-priority-encode the interrupt lines with the CD_IRQ one (IPL* are active-low)
	// Also swap IPL0 and IPL1 for CD systems
	//                      Cartridge     		CD
	// CD_IRQ IPL1 IPL0		IPL2 IPL1 IPL0		IPL2 IPL1 IPL0
	//    0     1    1		  1    1    1  	  1    1    1	No IRQ
	//    0     1    0        1    1    0		  1    0    1	Vblank
	//    0     0    1        1    0    1		  1    1    0  Timer
	//    0     0    0        1    0    0		  1    0    0	Cold boot
	//    1     x    x        1    1    1  	  0    1    1	CD vectored IRQ
	wire IPL0_OUT = SYSTEM_CDx ? CD_IRQ | IPL1 : IPL0;
	wire IPL1_OUT = SYSTEM_CDx ? CD_IRQ | IPL0 : IPL1;
	wire IPL2_OUT = ~(SYSTEM_CDx & CD_IRQ);
	
	// Because of the SDRAM latency, nDTACK is handled differently for ROM zones
	// If the address is in a ROM zone, nDTACK_ADJ is used instead of the normal nDTACK output by NEO-C1
	//wire nDTACK_ADJ = SDRAM_M68K_SIG ? nDTACK | M68K_CACHE_MISS : nDTACK;
	cpu_68k	M68KCPU(
		CLK_24M, nRESET,
		IPL2_OUT, IPL1_OUT, IPL0_OUT,
		nDTACK, M68K_ADDR, FX68K_DATAIN, FX68K_DATAOUT, nLDS, nUDS, nAS, M68K_RW,
		FC2, FC1, FC0,
		nBG, nBR, nBGACK
		);
	
	wire IACK = &{FC2, FC1, FC0};
	
	// FX68K doesn't like byte masking with Z's, replace with 0's:
	assign M68K_DATA_BYTE_MASK = (~|{nLDS, nUDS}) ? M68K_DATA :
											(~nLDS) ? {8'h00, M68K_DATA[7:0]} :
											(~nUDS) ? {M68K_DATA[15:8], 8'h00} :
											16'h0000;

	assign M68K_DATA = M68K_RW ? 16'bzzzzzzzz_zzzzzzzz : FX68K_DATAOUT;
	assign FX68K_DATAIN = M68K_RW ? M68K_DATA_BYTE_MASK : 16'h0000;
	
	// Bankswitching for the PORT zone, do all games use a 1MB window ?
	// P_BANK stays at 0 for CD systems
	always @(posedge nPORTWEL or negedge nRESET)
	begin
		if (!nRESET)
			P_BANK <= 2'd0;
		else
			if (!SYSTEM_CDx) P_BANK <= M68K_DATA[1:0];
	end
	
	// This is used to split burst-read fix gfx data in half at the right time
	/*reg [2:0] PCK1_SR;
	reg S1_A3_REG;
	
	always @(posedge clk_sys)
	begin
		PCK1_SR <= {PCK1_SR[1:0], PCK1};
		if (PCK1_SR == 3'b011)
			S1_A3_REG <= S_LATCH[3];
	end*/
	
	//wire [15:0] FIX_DOUBLE = SROM_DATA;	//S1_A3_REG ? SROM_DATA[15:0] : SROM_DATA[31:16];
	//assign FIXD = S2H1 ? FIX_DOUBLE[15:8] : FIX_DOUBLE[7:0];
	
	assign FIXD = S2H1 ? SROM_DATA[15:8] : SROM_DATA[7:0];
	
	// Disable ROM in PORT zone if the game uses a special chip
	assign M68K_DATA = (nROMOE & nSROMOE & |{nPORTOE, cart_chip}) ? 16'bzzzzzzzzzzzzzzzz : PROM_DATA;
	
	//assign M68K_DATA = (nROMOE & nSROMOE & nPORTOE) ? 16'bzzzzzzzzzzzzzzzz : PROM_DATA_QUAD[{~M68K_ADDR[2:1], 4'b0000} +:16];
	
	// 68k work RAM
	wire [14:0] WRAM_ADDR = TRASHING ? TRASH_ADDR[14:0] : M68K_ADDR[15:1];
	wire [15:0] WRAM_DIN = TRASHING ? TRASH_ADDR[15:0] : M68K_DATA;
	m68k_ram WRAML(WRAM_ADDR, CLK_24M, WRAM_DIN[7:0], ~nWWL | TRASHING, WRAML_OUT);
	m68k_ram WRAMU(WRAM_ADDR, CLK_24M, WRAM_DIN[15:8], ~nWWU | TRASHING, WRAMU_OUT);
	// On CD systems, disable the blockRAM WRAM, use the SDRAM instead
	assign M68K_DATA[7:0] = (nWRL | SYSTEM_CDx) ? 8'bzzzzzzzz : WRAML_OUT;
	assign M68K_DATA[15:8] = (nWRU | SYSTEM_CDx) ? 8'bzzzzzzzz : WRAMU_OUT;
	// ~CD_EXT_RD & 
	assign M68K_DATA[7:0] = (nWRL | ~SYSTEM_CDx) ? 8'bzzzzzzzz : PROM_DATA[7:0];
	assign M68K_DATA[15:8] = (nWRU | ~SYSTEM_CDx) ? 8'bzzzzzzzz : PROM_DATA[15:8];
	

	// Backup RAM
	wire nBWL = nSRAMWEL | nSRAMWEN_G;
	wire nBWU = nSRAMWEU | nSRAMWEN_G;
	
	wire [15:0] sd_buff_din_sram;
	wire [14:0] sram_addr = {sd_lba[6:0], sd_buff_addr};
	// Enable writes for save file's 000000~00FFFF
	wire sram_wr = SYSTEM_MVS & ~sd_lba[7] & sd_buff_wr & sd_ack;
	
	dpram #(.ADDRWIDTH(15)) SRAML(
		.clock_a(CLK_24M),
		.address_a(M68K_ADDR[15:1]),
		.wren_a(~nBWL),
		.data_a(M68K_DATA[7:0]),
		.q_a(SRAML_OUT),
		
		.clock_b(clk_sys),
		.address_b(sram_addr),
		.wren_b(sram_wr),
		.data_b(sd_buff_dout[7:0]),
		.q_b(sd_buff_din_sram[7:0])
	);
	
	dpram #(.ADDRWIDTH(15)) SRAMU(
		.clock_a(CLK_24M),
		.address_a(M68K_ADDR[15:1]),
		.wren_a(~nBWU),
		.data_a(M68K_DATA[15:8]),
		.q_a(SRAMU_OUT),
		
		.clock_b(clk_sys),
		.address_b(sram_addr),
		.wren_b(sram_wr),
		.data_b(sd_buff_dout[15:8]),
		.q_b(sd_buff_din_sram[15:8])
	);
	
	// Backup RAM is only for MVS
	assign M68K_DATA[7:0] = (nSRAMOEL | ~SYSTEM_MVS) ? 8'bzzzzzzzz : SRAML_OUT;
	assign M68K_DATA[15:8] = (nSRAMOEU | ~SYSTEM_MVS) ? 8'bzzzzzzzz : SRAMU_OUT;
	
	
	
	// Memory card
	// Always plugged in CD systems
	assign {nCD1, nCD2} = {2{status[4] & ~SYSTEM_CDx}};
	assign CARD_WE = (SYSTEM_CDx | (~nCARDWEN & CARDWENB)) & ~nCRDW;
	
	wire [7:0] CDD_L;
	wire [7:0] CDD_U;
	wire [15:0] sd_buff_din_memcard;
	wire [11:0] memcard_addr = {sd_lba[3:0], sd_buff_addr};
	wire memcard_wr = SYSTEM_CDx ? (sd_req_type == 16'h0000) & sd_buff_wr & sd_ack :		// Enable writes for save file's 000000+
							sd_lba[7] & sd_buff_wr & sd_ack;			// Enable writes for save file's 010000+
	
	// Split the 8kB 8-bit memory card into two 1kB dual-port RAMs so the HPS
	// can access it in 16-bits. The lower address bit (CDA[0]) selects which
	// one is accessed from the NeoGeo side. The HPS always accesess in words.
	// In cart mode, there's one 2kB memory card for each game.
	// In CD mode, there's one 8kB memory card for all games.
	dpram #(.ADDRWIDTH(12)) MEMCARDL(
		.clock_a(CLK_24M),
		.address_a(SYSTEM_CDx ? CDA[12:1] : {2'b00, CDA[10:1]}),
		.wren_a(CARD_WE & CDA[0]),
		.data_a(M68K_DATA[7:0]),
		.q_a(CDD_L),
		
		.clock_b(clk_sys),
		.address_b(memcard_addr),
		.wren_b(memcard_wr),
		.data_b(sd_buff_dout[7:0]),
		.q_b(sd_buff_din_memcard[7:0])
	);
	
	dpram #(.ADDRWIDTH(12)) MEMCARDU(
		.clock_a(CLK_24M),
		.address_a(SYSTEM_CDx ? CDA[12:1] : {2'b00, CDA[10:1]}),
		.wren_a(CARD_WE & ~CDA[0]),
		.data_a(M68K_DATA[7:0]),
		.q_a(CDD_U),
		
		.clock_b(clk_sys),
		.address_b(memcard_addr),
		.wren_b(memcard_wr),
		.data_b(sd_buff_dout[15:8]),
		.q_b(sd_buff_din_memcard[15:8])
	);
	
	assign CDD = CDA[0] ? CDD_L : CDD_U;
	
	// Feed save file writer with backup RAM data or memory card data
	assign sd_buff_din = SYSTEM_CDx ? sd_buff_din_memcard :
								sd_lba[7] ? sd_buff_din_memcard : sd_buff_din_sram;
	
	

	// Cartridge stuff
	zmc ZMC(SDRD0, SDA[1:0], SDA[15:8], MA);

	// PRO-CT0 used as security chip
	wire [3:0] GAD_SEC;
	wire [3:0] GBD_SEC;
	zmc2_dot ZMC2DOT(nPORTWEL, M68K_ADDR[2], M68K_ADDR[1], M68K_ADDR[3],
					{
					M68K_ADDR[19], M68K_ADDR[15], M68K_ADDR[18], M68K_ADDR[14],
					M68K_ADDR[17], M68K_ADDR[13], M68K_ADDR[16], M68K_ADDR[12],
					M68K_ADDR[11], M68K_ADDR[7], M68K_ADDR[10], M68K_ADDR[6],
					M68K_ADDR[9], M68K_ADDR[5], M68K_ADDR[8], M68K_ADDR[4],
					M68K_DATA[15], M68K_DATA[11], M68K_DATA[14], M68K_DATA[10],
					M68K_DATA[13], M68K_DATA[9], M68K_DATA[12], M68K_DATA[8],
					M68K_DATA[7], M68K_DATA[3], M68K_DATA[6], M68K_DATA[2],
					M68K_DATA[5], M68K_DATA[1], M68K_DATA[4], M68K_DATA[0]
					}, GAD_SEC, GBD_SEC);
	assign M68K_DATA[7:0] = ((cart_chip == 2'd1) & ~nPORTOEL) ?
									{GBD_SEC[1], GBD_SEC[0], GBD_SEC[3], GBD_SEC[2],
									GAD_SEC[1], GAD_SEC[0], GAD_SEC[3], GAD_SEC[2]} : 8'bzzzzzzzz;

	neo_273	NEO273(PBUS[19:0], ~PCK1, ~PCK2, C_LATCH, S_LATCH);
	
	// 4 MSBs not handled by NEO-273
	always @(negedge PCK1)
		C_LATCH_EXT <= PBUS[23:20];
	
	// Fake COM MCU
	wire [15:0] COM_DOUT;
	com COM(nRESET, CLK_24M, nPORTOEL, nPORTOEU, nPORTWEL, COM_DOUT);
	assign M68K_DATA = (cart_chip == 2'd2) ? COM_DOUT : 16'bzzzzzzzz_zzzzzzzz;
	
	
	
	syslatch SL(M68K_ADDR[4:1], nBITW1, nRESET,
					SHADOW, nVEC, nCARDWEN, CARDWENB, nREGEN, nSYSTEM, nSRAMWEN, PALBNK,
					CLK_68KCLK);
	wire nSRAMWEN_G = SYSTEM_MVS ? nSRAMWEN : 1'b1;	// nSRAMWEN is only for MVS
	wire nSYSTEM_G = SYSTEM_MVS ? nSYSTEM : 1'b1;	// nSYSTEM is only for MVS
	
	neo_e0 E0(M68K_ADDR[23:1], BNK, nSROMOEU, nSROMOEL, nSROMOE, nVEC, A23Z, A22Z, CDA);
	
	wire [7:0] DIPSW = {~status[9:8], 5'b11111, ~status[7]};
	neo_f0 F0(nRESET, nDIPRD0, nDIPRD1, nBITW0, nBITWD0, DIPSW, ~joystick_0[10], ~joystick_1[10], M68K_ADDR[7:4],
				M68K_DATA[7:0], ~nSYSTEM_G, , , , , , , RTC_DOUT, RTC_TP, RTC_DIN, RTC_CLK, RTC_STROBE, , SYSTEM_MVS);
	
	uPD4990 RTC(nRESET, CLK_12M, rtc, 1'b1, 1'b1, RTC_CLK, RTC_DIN, RTC_STROBE, RTC_TP, RTC_DOUT);
	
	neo_g0 G0(M68K_DATA, nCRDC, nPAL, M68K_RW, {8'hFF, CDD}, PAL_RAM_DATA, nPAL_WE);
	
	wire [2:0] joystick_0_hack = joystick_0[11] ? 3'b111 : joystick_0[6:4];
	
	neo_c1 C1(M68K_ADDR[21:17], M68K_DATA[15:8], A22Z, A23Z, nLDS, nUDS, M68K_RW, nAS, nROMOEL, nROMOEU,
				nPORTOEL, nPORTOEU, nPORTWEL, nPORTWEU, nPORTADRS, nWRL, nWRU, nWWL, nWWU, nSROMOEL, nSROMOEU, 
				nSRAMOEL, nSRAMOEU, nSRAMWEL, nSRAMWEU, nLSPOE, nLSPWE, nCRDO, nCRDW, nCRDC, nSDW,
				~{joystick_0[9:7], joystick_0_hack, joystick_0[0], joystick_0[1], joystick_0[2], joystick_0[3]},
				~{joystick_1[9:4], joystick_1[0], joystick_1[1], joystick_1[2], joystick_1[3]},
				nCD1, nCD2, 1'b0,			// Memory card is never write-protected
				1'b1, 1'b1, 1'b1, 1'b1,	// nROMWAIT, nPWAIT0, nPWAIT1, PDTACK,
				SDD, nSDZ80R, nSDZ80W, nSDZ80CLR, CLK_68KCLK,
				nDTACK, nBITW0, nBITW1, nDIPRD0, nDIPRD1, nPAL,
				SYSTEM_TYPE);

	// This is used to split burst-read sprite gfx data in half at the right time
	reg [2:0] LOAD_SR;
	reg CA4_REG;
	
	always @(posedge clk_sys)
	begin
		LOAD_SR <= {LOAD_SR[1:0], LOAD};
		if (LOAD_SR == 3'b011)
			CA4_REG <= CA4;
	end
	
	// CR_DOUBLE: [8px left] [8px right]
	//         BP  A B C D    A B C D
	wire [31:0] CR = CA4_REG ? CR_DOUBLE[63:32] : CR_DOUBLE[31:0];
	
	neo_zmc2 ZMC2(CLK_12M, EVEN1, LOAD, H, CR, GAD, GBD, DOTA, DOTB);
	
	// VCS is normally used as the LO ROM's nOE but the NeoGeo relies on the fact that the LO ROM
	// will still have its output active for a short moment (~50ns) after nOE goes high
	// nPBUS_OUT_EN is used internally by LSPC2 but it's broken out here to use the additional
	// half mclk cycle it provides compared to VCS. This makes sure LO_ROM_DATA is valid when latched.
	wire lo_loading = ioctl_download & (ioctl_index == INDEX_LOROM);
	wire [15:0] LO_ADDR = lo_loading ? ioctl_addr[16:1] : PBUS[15:0];
	wire lo_we = lo_loading ? ioctl_wr : 1'b0;
	lo_rom LO(LO_ADDR, ~clk_sys, ioctl_dout[7:0], lo_we, LO_ROM_DATA);
	assign PBUS[23:16] = nPBUS_OUT_EN ? LO_ROM_DATA : 8'bzzzzzzzz;
	
	fast_vram UFV(
		FAST_VRAM_ADDR,
		clk_sys,	//~CLK_24M,		// Is just CLK ok ?
		FAST_VRAM_DATA_OUT,
		~CWE,
		FAST_VRAM_DATA_IN);
	
	slow_vram USV(
		SLOW_VRAM_ADDR,
		clk_sys,	//~CLK_24M,		// Is just CLK ok ?
		SLOW_VRAM_DATA_OUT,
		~BWE,
		SLOW_VRAM_DATA_IN);
	
	wire [18:11] MA;
	wire [7:0] M1_ROM_DATA;
	wire [7:0] Z80_RAM_DATA;
	
	wire m1_loading = ioctl_download & (ioctl_index == INDEX_M1ROM);
	wire [16:0] M1_ADDR = m1_loading ? ioctl_addr[17:1] : {MA[16:11], SDA[10:0]};
	wire m1_we = m1_loading ? ioctl_wr : 1'b0;
	z80_rom M1(M1_ADDR, ~clk_sys, ioctl_dout[7:0], m1_we, M1_ROM_DATA);
	
	z80_ram Z80RAM(SDA[10:0], CLK_4M, SDD, ~(nZRAMCS | nSDMWR), Z80_RAM_DATA);		
	
	assign SDD = (~SDRD0 | ~SDRD1) ? 8'b00000000 :	// Fix to prevent TV80 from going nuts because the data bus is open on port reads for NEO-ZMC
						(~nSDROM & ~nSDMRD) ? M1_ROM_DATA :
						(~nZRAMCS & ~nSDMRD) ? Z80_RAM_DATA :
						(~n2610CS & ~n2610RD) ? YM2610_DOUT :
						8'bzzzzzzzz;
	
	wire Z80_nRESET = SYSTEM_CDx ? nRESET & CD_nRESET_Z80 : nRESET;
	cpu_z80 Z80CPU(CLK_4M, nRESET, SDD, SDA, nIORQ, nMREQ, nSDRD, nSDWR, nZ80INT, nZ80NMI);
	
	wire [19:0] ADPCMA_ADDR;
	wire [3:0] ADPCMA_BANK;
	wire [7:0] ADPCMA_DATA = 8'h00;
	wire [23:0] ADPCMB_ADDR;
	wire [7:0] ADPCMB_DATA = SYSTEM_CDx ? 8'h00 : 8'h00;	// CD has no ADPCM-B
	
	wire [7:0] YM2610_DOUT;
	jt10 YM2610(~nRESET, CLK_8M, 1'b1, SDD, SDA[1:0], n2610CS, n2610WR, YM2610_DOUT, nZ80INT,	// SDA[1:0]
					ADPCMA_ADDR, ADPCMA_BANK, nSDROE, ADPCMA_DATA, ADPCMB_ADDR, nSDPOE,
						, , snd_right, snd_left, );
	
	// For Neo CD only
	wire DOTA_GATED = SPR_EN & DOTA;
	wire DOTB_GATED = SPR_EN & DOTB;
	
	lspc2_a2	LSPC(CLK_24M, nRESET,
					PBUS[15:0],
					PBUS[23:16],
					M68K_ADDR[3:1],
					M68K_DATA,
					nLSPOE, nLSPWE,
					DOTA_GATED, DOTB_GATED,
					CA4, S2H1, S1H1,
					LOAD,
					H, EVEN1, EVEN2,
					IPL0, IPL1,
					CHG, LD1, LD2,
					PCK1, PCK2,
					WE, CK, SS1, SS2,
					nRESETP,
					VGA_HS, VGA_VS,
					CHBL, nBNKB,
					VCS,
					CLK_8M, CLK_4M,
					SLOW_VRAM_ADDR, SLOW_VRAM_DATA_IN, SLOW_VRAM_DATA_OUT, BOE, BWE,
					FAST_VRAM_ADDR, FAST_VRAM_DATA_IN, FAST_VRAM_DATA_OUT, CWE,
					nPBUS_OUT_EN,
					video_mode
					);
	
	// These are always set on cartridg systems
	wire VIDEO_EN = SYSTEM_CDx ? CD_VIDEO_EN : 1'b1;
	wire FIX_EN = SYSTEM_CDx ? CD_FIX_EN : 1'b1;
	wire SPR_EN = SYSTEM_CDx ? CD_SPR_EN : 1'b1;
	
	neo_b1	B1(CLK_24M, CLK_6MB, CLK_1HB,
					PBUS,
					FIXD,
					PCK1, PCK2,
					CHBL, nBNKB,
					GAD, GBD,
					WE, CK,
					CHG, LD1, LD2, SS1, SS2, S1H1, A23Z, A22Z,
					PAL_RAM_ADDR,
					nLDS, M68K_RW, nAS, M68K_ADDR[21:17], M68K_ADDR[12:1],
					 , , ,	// nHALT, nRESET, nRST
					FIX_EN);
	
	pal_ram PALRAM({PALBNK, PAL_RAM_ADDR}, CLK_24M, M68K_DATA, ~nPAL_WE, PAL_RAM_DATA);	// Was CLK_12M
	
	// DAC latches
	always @(posedge CLK_6MB, negedge nBNKB)
	begin
		if (!nBNKB)
			PAL_RAM_REG <= 16'h0000;
		else
			// For Neo CD only:
			PAL_RAM_REG <= VIDEO_EN ? PAL_RAM_DATA : 16'h0000;
	end
	
	// Final video output 6 bits -> 8 bits
	assign VGA_R = {PAL_RAM_REG[11:8], PAL_RAM_REG[14], PAL_RAM_REG[15], 2'b00};
	assign VGA_G = {PAL_RAM_REG[7:4], PAL_RAM_REG[13], PAL_RAM_REG[15], 2'b00};
	assign VGA_B = {PAL_RAM_REG[3:0], PAL_RAM_REG[12], PAL_RAM_REG[15], 2'b00};
	
endmodule
