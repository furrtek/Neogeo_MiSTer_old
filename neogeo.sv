//============================================================================
//  SNK NeoGeo for MiSTer
//  Copyright (C) 2018 Sean 'Furrtek' Gonsalves
//
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
	output        VGA_DE,    // = ~(VBlank | HBlank)

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
assign AUDIO_MIX = status[3:2];
assign AUDIO_L = 16'h0000;	// No audio for now
assign AUDIO_R = 16'h0000;	// No audio for now

assign LED_USER  = ioctl_download;
assign LED_DISK  = 0;
assign LED_POWER = 0;

assign VIDEO_ARX = 8'd10;	// 320/32
assign VIDEO_ARY = 8'd7;	// 224/32

assign VGA_DE = ~(CHBL | ~nBNKB);

`include "build_id.v"
localparam CONF_STR1 = {
	"NEOGEO;;",
	"-;",
	"F,BIN,Load Cart;",
	"-;",
	"O1,Test Option 1,On,Off;",
	"-;",
	"O23,Stereo mix,none,25%,50%,100%;",
	"R0,Reset & apply;",
	"J1,Fire 1,Fire 2;",
	"V,v",`BUILD_DATE
};


////////////////////   CLOCKS   ///////////////////

wire locked;
wire clk_sys;

// 50MHz in, 6*24=144MHz out
// CAS latency = 3 (22.5ns)
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(SDRAM_CLK),	// 180° phase shift
	.locked(locked)
);

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL = CLK_6MB;

reg CLK_24M;
reg [2:0] counter = 0;

// 144MHz:
// 012345012345
// ___'''___'''
// 96MHz:
// 01230123
// __''__''

always @(negedge clk_sys)
begin
	if (counter == 3'd1)	// Was 2
		CLK_24M <= 1'b1;
	
	if (counter == 3'd3)	// Was 5
	begin
		CLK_24M <= 1'b0;
		counter <= 3'd0;
	end
	else
		counter <= counter + 1'd1;
end


//////////////////   HPS I/O   ///////////////////
//wire [24:0] ps2_mouse;

wire [15:0] joystick_0;
wire [15:0] joystick_1;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire [31:0] status;

wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire [15:0] ioctl_dout;
wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        ioctl_wait;

reg  [31:0] status_out;

hps_io #(.STRLEN(($size(CONF_STR1)>>3)), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str(CONF_STR1),

	//.ps2_mouse(ps2_mouse),	// Could be used for The Irritating Maze ?

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.buttons(buttons),			// What buttons ?
	//.forced_scandoubler(forced_scandoubler),
	.status(status),				// status read (32 bits)
	//.status_set(speed_set|arch_set|snap_hwset),	// status load on rising edge
	//.status_in({status[31:25], speed_set ? speed_req : 3'b000, status[21:13], arch_set ? arch : snap_hwset ? snap_hw : status[12:8], status[7:0]}),	// status write

	// Loading signals
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait)
);

	// INFOS:
	// 68k RAM is in Block RAM
	// Slow VRAM is in Block RAM
	// Fast VRAM is in Block RAM
	// Palette RAM is in Block RAM
	// Line buffers are in Block RAM
	// P ROM data is in SDRAM
	// C ROM data is in SDRAM
	// S ROM data is in SDRAM
	// LO ROM data is in Block RAM
	
	reg  [24:0] sdram_addr;

	wire [23:0] PBUS;
	wire [7:0] LO_ROM_DATA;
	
	wire [19:0] C_LATCH;
	wire [20:0] SPR_ROM_ADDR;
	reg [31:0] CR;
	
	wire [15:0] S_LATCH;
	wire [16:0] FIX_ROM_ADDR;
	reg [7:0] FIXD;
	
	wire [14:0] SLOW_VRAM_ADDR;
	reg [15:0] SLOW_VRAM_DATA_IN;
	wire [15:0] SLOW_VRAM_DATA_OUT;
	
	wire [10:0] FAST_VRAM_ADDR;
	wire [15:0] FAST_VRAM_DATA_IN;
	wire [15:0] FAST_VRAM_DATA_OUT;
	
	wire [11:0] PAL_RAM_ADDR;
	wire [15:0] PAL_RAM_DATA;
	reg [15:0] PAL_RAM_REG;
	
	wire [3:0] GAD;
	wire [3:0] GBD;
	wire [3:0] WE;
	wire [3:0] CK;
	
	wire [15:0] M68K_DATA;
	wire [23:1] M68K_ADDR;
	wire M68K_RW, nAS, nLDS, nUDS;
	wire [15:0] M68K_DATA_BYTE_MASK;
	wire [15:0] TG68K_DATAIN;
	wire [15:0] TG68K_DATAOUT;
	
	wire [7:0] SDD;

	wire READY;
	
	wire [7:0] WRAML_OUT;
	wire [7:0] WRAMU_OUT;
	
	assign SDD = 8'bzzzzzzzz;
	
	reg [1:0] SR_SDRAM_M68K_REQ;
	reg [1:0] SR_SDRAM_CROM_REQ;
	reg [1:0] SR_SDRAM_SROM_REQ;
	reg [1:0] SR_SDRAM_SROM_REQ_B;
	//reg [15:0] SROM_DATA;
	reg [15:0] PROM_DATA;
	reg M68K_RD_REQ, CROM_RD_REQ, SROM_RD_REQ;
	reg CROM_RD_STEP;
	reg M68K_LATCH_FLAG, M68K_LATCH_FLAG_PREV;
	reg SROM_LATCH_FLAG, SROM_LATCH_FLAG_PREV;
	reg SDRAM_RD_PULSE;
	
	fast_vram UFV(
		FAST_VRAM_ADDR,
		~CLK_24M,		// Is just CLK ok ?
		FAST_VRAM_DATA_OUT,
		~CWE,
		FAST_VRAM_DATA_IN);
		
	slow_vram USV(
		SLOW_VRAM_ADDR,
		~CLK_24M,		// Is just CLK ok ?
		SLOW_VRAM_DATA_OUT,
		~BWE,
		SLOW_VRAM_DATA_IN);

	// Only clocks from NEO-D0 for now
	clocks	U3(CLK_24M, nRESETP, CLK_12M, CLK_68KCLK, CLK_68KCLKB, CLK_6MB, CLK_1MB);
	
	cpu_68k	M68KCPU(CLK_68KCLK, nRESET, IPL1, IPL0, nDTACK, M68K_ADDR,
		TG68K_DATAIN, TG68K_DATAOUT, nLDS, nUDS, nAS, M68K_RW);
	
	// TG68K doesn't like byte masking with Z's, replace with 0's:
	assign M68K_DATA_BYTE_MASK = (~|{nLDS, nUDS}) ? M68K_DATA :
											(~nLDS) ? {8'h00, M68K_DATA[7:0]} :
											(~nUDS) ? {M68K_DATA[15:8], 8'h00} :
											16'h0000;

	assign M68K_DATA = M68K_RW ? 16'bzzzzzzzzzzzzzzzz : TG68K_DATAOUT;
	assign TG68K_DATAIN = M68K_RW ? M68K_DATA_BYTE_MASK : 16'h0000;

	assign nRESET = ~ioctl_download;
	
	assign SDRAM_M68K_REQ = ~&{nSROMOE, nROMOE};
	//assign READ_REQ = CROM_RD_REQ | SROM_RD_REQ | M68K_RD_REQ;
	
	assign nROMOE = nROMOEL & nROMOEU;
	
	always @(posedge clk_sys)
	begin
		if (!nRESET)
		begin
			CROM_RD_REQ <= 0;
			SROM_RD_REQ <= 0;
			M68K_RD_REQ <= 0;
			M68K_LATCH_FLAG <= 0;
		end
		else
		begin
			// Detect rising edge of SDRAM_M68K_REQ
			SR_SDRAM_M68K_REQ <= {SR_SDRAM_M68K_REQ[0], SDRAM_M68K_REQ};
			if ((SR_SDRAM_M68K_REQ == 2'b01) & nRESET)
				M68K_RD_REQ <= 1'b1;
			
			// Detect rising edge of PCK1B
			/*SR_SDRAM_CROM_REQ <= {SR_SDRAM_CROM_REQ[0], ~PCK1};
			if ((SR_SDRAM_CROM_REQ == 2'b01) & nRESET)
			begin
				CROM_RD_REQ <= 1'b1;
				CROM_RD_STEP <= 1'b0;
			end*/
			
			// Detect rising edge of PCK2B or S2H1
			SR_SDRAM_SROM_REQ <= {SR_SDRAM_SROM_REQ[0], ~PCK2};
			if ((SR_SDRAM_SROM_REQ == 2'b01) & nRESET)
				SROM_RD_REQ <= 1'b1;
			SR_SDRAM_SROM_REQ_B <= {SR_SDRAM_SROM_REQ_B[0], S2H1};
			if ((SR_SDRAM_SROM_REQ_B == 2'b01) & nRESET)
				SROM_RD_REQ <= 1'b1;
			
			if (SDRAM_RD_PULSE)
				SDRAM_RD_PULSE <= 0;
			
			if (sdram_ready)
			begin
				
				/*if (!CROM_RD_STEP)
				begin
					CROM_RD_STEP <= 1;	// Repeat request for second 16bit word
					//CROM_RD_REQ <= 1;
				end*/
				
				// Prioritize C ROM read
				/*if (CROM_RD_REQ)
				begin
					if (!CROM_RD_STEP)
						CR[31:16] <= sdram_dout;
					else
						CR[15:0] <= sdram_dout;
					CROM_RD_REQ <= 0;
					SDRAM_RD_PULSE <= 1;
				end
				else*/
				if (SROM_RD_REQ)
				begin
					SROM_LATCH_FLAG <= 1;
					SROM_RD_REQ <= 0;		// Request is being fulfilled
					SDRAM_RD_PULSE <= 1;	// Tell SDRAM controller to do a read
				end
				else if (M68K_RD_REQ)
				begin
					M68K_LATCH_FLAG <= 1;
					M68K_RD_REQ <= 0;
					SDRAM_RD_PULSE <= 1;
				end
				
				// Ugly :(
				if (M68K_LATCH_FLAG)
					M68K_LATCH_FLAG <= 0;
				// Falling edge detector
				M68K_LATCH_FLAG_PREV <= M68K_LATCH_FLAG;
				if (M68K_LATCH_FLAG_PREV & ~M68K_LATCH_FLAG)
					PROM_DATA <= {sdram_dout[7:0], sdram_dout[15:8]};
					
				// Ugly :(
				if (SROM_LATCH_FLAG)
					SROM_LATCH_FLAG <= 0;
				// Falling edge detector
				SROM_LATCH_FLAG_PREV <= SROM_LATCH_FLAG;
				if (SROM_LATCH_FLAG_PREV & ~SROM_LATCH_FLAG)
					FIXD <= FIX_ROM_ADDR[0] ? sdram_dout[15:8] : sdram_dout[7:0];
			end
		end
	end
	
	/*
	// SDRAM address mux
	assign DRAM_CTRL_ADDR = nRESET ?
		CROM_RD_REQ ? {2'b01, SPR_ROM_ADDR[18:0], CROM_RD_STEP} :		// $200000~$3FFFFF
		SROM_RD_REQ ? {6'b001000, S_LATCH[15:0]} :	// FIX_ROM_ADDR $100000~$10FFFF
		~nROMOE ? {3'b000, M68K_ADDR[19:1]} :		// $000000~$0FFFFF
		~nSROMOE ? {3'b111, M68K_ADDR[19:1]} :		// $700000~$7FFFFF
		22'h0 : // Could remove this and ~nSROMOE condition ?
		INIT_SDRAM_ADDR;
	*/
	//assign CR = 32'b00000000_00000000_00000000_10101010;
	
	wire [15:0] sdram_dout;
	wire [15:0] sdram_din = ioctl_download ? ioctl_dout : 16'h0000;
	wire sdram_rd = ~ioctl_download & SDRAM_RD_PULSE;
	wire sdram_we = ioctl_download ? ioctl_wr : 1'b0;
	
	// TODO: make sdram_addr a register ?
	// sdram_addr is 25 bits, LSB is ignored in word mode
	always_comb begin
		casez ({ioctl_download, CROM_RD_REQ, (SROM_LATCH_FLAG_PREV | SROM_LATCH_FLAG) & ~M68K_RD_REQ, ~nROMOE, ~nSROMOE})
			5'b1zzzz: sdram_addr = ioctl_addr;	// Loading mode, let hps_io use the SDRAM
			// The following mapping currently depends on the test cartridge structure !
			// See pbobble/structure.txt
			//5'b01zzz: sdram_addr = {1'b0, {SPR_ROM_ADDR[19:0], CROM_RD_STEP, 1'b0} + 24'h100000};	// C ROMs Bytes $100000~$6FFFFF
			// Testing
			5'b1zzzz: sdram_addr = 25'h0000000;
			5'b01zzz: sdram_addr = 25'h0000000;
			//5'b001zz: sdram_addr = 25'h0000000;
			5'b001zz: sdram_addr = {8'b00000100, FIX_ROM_ADDR[16:1], 1'b0};	// Fix ROM Bytes $080000~$09FFFF
			5'b0001z: sdram_addr = {5'b00000, M68K_ADDR[19:1], 1'b0};			// P ROM $000000~$07FFFF
			5'b00001: sdram_addr = {7'b0011100, M68K_ADDR[17:1], 1'b0};			// System ROM $700000~$71FFFF
			5'b00000: sdram_addr = 25'h0000000;
		endcase
	end
	
	// This is used in 16-bit mode
	sdram ram
	(
		.*,					// Connect all nets with the same names (SDRAM_* pins)
		.init(~locked),	// Init SDRAM as soon as the PLL is locked
		.clk(clk_sys),
		.dout(sdram_dout),
		.din(sdram_din),
		.addr(sdram_addr),
		.wtbt(2'b11),		// SDRAM is always used in 16-bit words
		.we(sdram_we),
		.rd(sdram_rd),
		.ready(sdram_ready)
	);

	//assign FIXD = S2H1 ? SROM_DATA[7:0] : SROM_DATA[15:8];
	assign M68K_DATA = (nROMOE & nSROMOE) ? 16'bzzzzzzzzzzzzzzzz : {PROM_DATA[7:0], PROM_DATA[15:8]};
	
	m68k_ram U8(M68K_ADDR[15:1], CLK_24M, M68K_DATA[7:0], ~nWWL, WRAML_OUT);
	m68k_ram U9(M68K_ADDR[15:1], CLK_24M, M68K_DATA[15:8], ~nWWU, WRAMU_OUT);
	assign M68K_DATA[7:0] = nWRL ? 8'bzzzzzzzz : WRAML_OUT;
	assign M68K_DATA[15:8] = nWRU ? 8'bzzzzzzzz : WRAMU_OUT;
	
	//assign nBITWD0 = |{nBITW0, M68K_ADDR[6:5]};
	
	syslatch SL(M68K_ADDR[4:1], nBITW1, nRESET, SHADOW, nVEC, nCARDWEN, CARDWENB, nREGEN, nSYSTEM, nSRAMWEN, PALBNK);

	neo_e0 E0(M68K_ADDR[23:1], 3'b000, nSROMOEU, nSROMOEL, nSROMOE, nVEC, A23Z, A22Z, );
	
	neo_c1	C1(M68K_ADDR[21:17], M68K_DATA[15:8], A22Z, A23Z, nLDS, nUDS, M68K_RW, nAS, nROMOEL, nROMOEU,
				nPORTOEL, nPORTOEU, nPORTWEL, nPORTWEU, nPORTADRS, nWRL, nWRU, nWWL, nWWU, nSROMOEL, nSROMOEU, 
				nSRAMOEL, nSRAMOEU, nSRAMWEL, nSRAMWEU, nLSPOE, nLSPWE, nCRDO, nCRDW, nCRDC, nSDW,
				~joystick_0[9:0], ~joystick_1[9:0],
				1'b1, 1'b1, 1'b1,	//nCD1, nCD2, nWP,
				1'b1, 1'b1, 1'b1, 1'b1,	// nROMWAIT, nPWAIT0, nPWAIT1, PDTACK,
				SDD, 1'b1, 1'b1, 1'b1, CLK_68KCLK,
				nDTACK, nBITW0, nBITW1, nDIPRD0, nDIPRD1, nPAL, 1'b1);	// SYSTEM_MODE
	
	neo_273	U4(PBUS[19:0], ~PCK1, ~PCK2, C_LATCH, S_LATCH);

	neo_zmc2 ZMC2(CLK_12M, EVEN1, LOAD, H, 32'h00000000, GAD, GBD, DOTA, DOTB);	// CR DEBUG
	
	// VCS is normally used as nOE but the NeoGeo's design relies on the fact that the LO ROM
	// will have its output still active for a short moment (~50ns) after its nOE goes high
	// nPBUS_OUT_EN is used internally by LSPC2 but it's broken out here to use the additional
	// half mclk cycle it provides compared to VCS
	lo_rom LO(PBUS[15:0], CLK_24M, LO_ROM_DATA);
	assign PBUS[23:16] = nPBUS_OUT_EN ? LO_ROM_DATA : 8'bzzzzzzzz;
	
	lspc2_a2	U5(CLK_24M, nRESET,
					PBUS[15:0],
					PBUS[23:16],
					M68K_ADDR[3:1],
					M68K_DATA,
					nLSPOE, nLSPWE,
					DOTA, DOTB,
					CA4,
					S2H1,	S1H1,
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
					LSPC_8M,	LSPC_4M,
					SLOW_VRAM_ADDR, SLOW_VRAM_DATA_IN, SLOW_VRAM_DATA_OUT, BOE, BWE,
					FAST_VRAM_ADDR, FAST_VRAM_DATA_IN, FAST_VRAM_DATA_OUT, CWE,
					nPBUS_OUT_EN
					);
	
	neo_b1	U6(CLK_24M, CLK_6MB, CLK_1MB,
					PBUS,
					FIXD,
					PCK1, PCK2,
					CHBL, nBNKB,
					GAD, GBD,
					WE[0], WE[1], WE[2], WE[3],
					CK[0], CK[1], CK[2], CK[3],
					CHG, LD1, LD2, SS1, SS2, S1H1, A23Z, A22Z,
					PAL_RAM_ADDR, nLDS, M68K_RW, nAS, M68K_ADDR[21:17], M68K_ADDR[12:1],
					, , 1'b1);

	// Cartridge PCB
	assign SPR_ROM_ADDR = {C_LATCH[19:4], CA4, C_LATCH[3:0]};
	assign FIX_ROM_ADDR = {S_LATCH[15:3], S2H1, S_LATCH[2:0]};

	pal_ram U7(PAL_RAM_ADDR, CLK_12M, M68K_DATA, (~nPAL & ~M68K_RW), PAL_RAM_DATA);
	
	// DAC latches
	always @(posedge CLK_6MB, negedge nBNKB)
	begin
		if (!nBNKB)
			PAL_RAM_REG <= 16'h0000;
		else
			PAL_RAM_REG <= PAL_RAM_DATA;
	end

	// Final video output
	assign VGA_R = {PAL_RAM_REG[11:8], PAL_RAM_REG[14], PAL_RAM_REG[15], 2'b00};
	assign VGA_G = {PAL_RAM_REG[7:4], PAL_RAM_REG[13], PAL_RAM_REG[15], 2'b00};
	assign VGA_B = {PAL_RAM_REG[3:0], PAL_RAM_REG[12], PAL_RAM_REG[15], 2'b00};
	
	// VGA DAC tester
	/*reg [23:0] TEMP;
	reg [1:0] TEST_COLOR;
	always @(posedge CLK_24M)
	begin
		TEMP <= TEMP + 1'b1;
		if (TEMP >= 24'd12000000)
		begin
			TEMP <= 24'd0;
			TEST_COLOR <= TEST_COLOR + 1'b1;
		end
	end
	
	assign VGA_R = (TEST_COLOR == 2'd0) ? 8'hFF : 8'h00;
	assign VGA_G = (TEST_COLOR == 2'd1) ? 8'hFF : 8'h00;
	assign VGA_B = (TEST_COLOR == 2'd2) ? 8'hFF : 8'h00;*/

endmodule
