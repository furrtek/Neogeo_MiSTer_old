// New init module: now also loads to SDRAM
// Flash is 90ns so max read frequency is 11MHz, so must divide CLK (24MHz) by 3 (4 used, simpler)

module init(
	input CLK,
	input nRST,
	
	output nRESET,
	
	output [21:0] FL_ADDR,
	input [7:0] FL_DQ,
	
	output reg INIT_WR_REQ,
	output [21:0] INIT_SDRAM_ADDR,
	output [15:0] INIT_SDRAM_DATA,
	
/*	input [16:0] FIX_ROM_ADDR,
	output [7:0] FIXD,*/
	
	input [10:0] FVRAM_ADDR,
	output [15:0] FAST_VRAM_DATA_IN,
	input [15:0] FAST_VRAM_DATA_OUT,
	input CWE,
	
/*	input [20:0] SPR_ROM_ADDR,
	output [31:0] CR,*/
	
	input [15:0] LO_ADDR
	
/*	input [9:0] P_ROM_ADDR,
	output [15:0] PROM_DATA_OUT*/
);

	// Flash contains concatenated data:
	// $000000~$01FFFF: neo-ep0.bin system ROM ($20000 bytes, copied to SDRAM)
	// $020000~$02FFFF: LO ROM ($10000 bytes, stays in flash)
	// $030000~$0AFFFF: 202-p1.p1 ROM ($80000 bytes, copied to SDRAM)
	// $0B0000~$0CFFFF: 202-s1.s1 ROM ($20000 bytes, copied to SDRAM)
	// $0D0000~$1CFFFF: 202-c1.c1 ROM ($100000 bytes, copied to SDRAM)
	// $1D0000~$2CFFFF: 202-c2.c2 ROM ($100000 bytes, copied to SDRAM)

	// TODO: Pre-interleave C ROM data in the SDRAM so that burst reads can be used to get 32 bits quickly ?

	// Init (INIT_STATE) goes like this:
	// 0: Copy neo-ep0.bin to SDRAM @ $700000 (bytes)
	// 1: Copy 202-p1.p1 to SDRAM @ $000000 (bytes)
	// 2: Copy 202-s1.s1 to SDRAM @ $100000 (bytes)
	// 3: Copy 202-c1.c1 & 202-c2.c2 to SDRAM @ $200000 (bytes)
	// 4: Run, flash=LO ROM

	reg [2:0] INIT_STATE;		// FSM state, see above
	reg [2:0] INIT_SEQ;			// For access sequencing
	reg [15:0] WORD_BUFFER;		// To buffer flash 8bit -> 16bit
	reg [21:0] INIT_SRC_ADDR;	// In bytes
	reg [21:0] INIT_DEST_ADDR;	// In words
	reg [2:0] CLK_DIV;
	wire INIT_MODE;
	wire [21:0] C_ROM_SRC_ADDR;
	
	assign INIT_SDRAM_ADDR = INIT_DEST_ADDR;
	assign INIT_SDRAM_DATA = WORD_BUFFER;

	assign INIT_MODE = (INIT_STATE != 3'd4) ? 1'b1 : 1'b0;
	assign nRESET = ~INIT_MODE;	// NeoGeo /RESET

	FASTVRAM UFV(
		FVRAM_ADDR,
		~CLK,		// Is just CLK ok ?
		FAST_VRAM_DATA_OUT,
		~CWE,
		FAST_VRAM_DATA_IN);

	always @(posedge CLK)
		CLK_DIV <= CLK_DIV + 1'b1;

	assign C_ROM_SRC_ADDR = INIT_SRC_ADDR[1] ? {1'b0, INIT_SRC_ADDR[21:2], INIT_SRC_ADDR[0]} + 22'h100000 :
		{1'b0, INIT_SRC_ADDR[21:2], INIT_SRC_ADDR[0]};
	
	// SDRAM layout for C ROMs (bytes):
	// [C1 0] [C1 1] [C2 0] [C2 1] [C1 2] [C1 3] [C2 2] [C2 3] ...
		
	// Init/run memory switches
	assign FL_ADDR = INIT_MODE ?
			(INIT_STATE == 3'd2) ? {INIT_SRC_ADDR[21:4], INIT_SRC_ADDR[0], INIT_SRC_ADDR[3:1]} :	// Reorganize SROM
			(INIT_STATE == 3'd3) ? C_ROM_SRC_ADDR :
			INIT_SRC_ADDR : {5'b00010, LO_ADDR};

	always @(posedge CLK_DIV[1])
	begin
		if (!nRST)
		begin
			INIT_SRC_ADDR <= 22'h000000;		// From flash
			INIT_DEST_ADDR <= 22'h380000;		// To SDRAM ($700000 in bytes)
			INIT_STATE <= 3'd0;
			INIT_SEQ <= 3'd0;
			INIT_WR_REQ <= 1'b0;
		end
		else
		begin
			if (INIT_STATE == 3'd0)
			begin
				if (INIT_SEQ == 3'd1)			// 0 is noop, for delay
				begin
					WORD_BUFFER[15:8] <= FL_DQ;	// Load MSB
					INIT_SRC_ADDR <= INIT_SRC_ADDR + 1'b1;
					INIT_SEQ <= INIT_SEQ + 1'b1;
				end
				else if (INIT_SEQ == 3'd3)			// 2 is noop, for delay
				begin
					WORD_BUFFER[7:0] <= FL_DQ;		// Load LSB
					INIT_SRC_ADDR <= INIT_SRC_ADDR + 1'b1;
					INIT_SEQ <= INIT_SEQ + 1'b1;
					INIT_WR_REQ <= 1'b1;
				end
				else if (INIT_SEQ == 3'd4)
				begin
					INIT_SEQ <= 3'd0;
					INIT_WR_REQ <= 1'b0;
					if (INIT_DEST_ADDR == 22'h38FFFF)
					begin
						INIT_SRC_ADDR <= 22'h030000;
						INIT_DEST_ADDR <= 22'h000000;
						INIT_STATE <= 3'd1;		// Copy done, next state
					end
					else
					begin
						INIT_DEST_ADDR <= INIT_DEST_ADDR + 1'b1;
					end
				end
				else
					INIT_SEQ <= INIT_SEQ + 1'b1;
			end
			else if (INIT_STATE == 3'd1)
			begin
				if (INIT_SEQ == 3'd1)			// 0 is noop, for delay
				begin
					WORD_BUFFER[15:8] <= FL_DQ;	// Load MSB
					INIT_SRC_ADDR <= INIT_SRC_ADDR + 1'b1;
					INIT_SEQ <= INIT_SEQ + 1'b1;
				end
				else if (INIT_SEQ == 3'd3)		// 2 is noop, for delay
				begin
					WORD_BUFFER[7:0] <= FL_DQ;		// Load LSB
					INIT_SRC_ADDR <= INIT_SRC_ADDR + 1'b1;
					INIT_SEQ <= INIT_SEQ + 1'b1;
					INIT_WR_REQ <= 1'b1;
				end
				else if (INIT_SEQ == 3'd4)
				begin
					INIT_SEQ <= 3'd0;
					INIT_WR_REQ <= 1'b0;
					if (INIT_DEST_ADDR == 22'h03FFFF)
					begin
						INIT_SRC_ADDR <= 22'h0B0000;
						INIT_DEST_ADDR <= 22'h080000;
						INIT_STATE <= 3'd2;		// Copy done, next state
					end
					else
					begin
						INIT_DEST_ADDR <= INIT_DEST_ADDR + 1'b1;
					end
				end
				else
					INIT_SEQ <= INIT_SEQ + 1'b1;
			end
			else if (INIT_STATE == 3'd2)
			begin
				if (INIT_SEQ == 3'd1)			// 0 is noop, for delay
				begin
					WORD_BUFFER[15:8] <= FL_DQ;	// Load MSB
					INIT_SRC_ADDR <= INIT_SRC_ADDR + 1'b1;
					INIT_SEQ <= INIT_SEQ + 1'b1;
					INIT_WR_REQ <= 1'b1;
				end
				else if (INIT_SEQ == 3'd3)			// 2 is noop, for delay
				begin
					WORD_BUFFER[7:0] <= FL_DQ;		// Load LSB
					INIT_SRC_ADDR <= INIT_SRC_ADDR + 1'b1;
					INIT_SEQ <= INIT_SEQ + 1'b1;
				end
				else if (INIT_SEQ == 3'd4)
				begin
					INIT_SEQ <= 3'd0;
					INIT_WR_REQ <= 1'b0;
					if (INIT_DEST_ADDR == 22'h08FFFF)
					begin
						INIT_SRC_ADDR <= 22'h0D0000;
						INIT_DEST_ADDR <= 22'h100000;
						INIT_STATE <= 3'd3;		// Copy done, next state
					end
					else
					begin
						INIT_DEST_ADDR <= INIT_DEST_ADDR + 1'b1;
					end
				end
				else
					INIT_SEQ <= INIT_SEQ + 1'b1;
			end
			else if (INIT_STATE == 3'd3)
			begin
				if (INIT_SEQ == 3'd1)			// 0 is noop, for delay
				begin
					WORD_BUFFER[15:8] <= FL_DQ;	// Load MSB
					INIT_SRC_ADDR <= INIT_SRC_ADDR + 1'b1;
					INIT_SEQ <= INIT_SEQ + 1'b1;
				end
				else if (INIT_SEQ == 3'd3)			// 2 is noop, for delay
				begin
					WORD_BUFFER[7:0] <= FL_DQ;		// Load LSB
					INIT_SRC_ADDR <= INIT_SRC_ADDR + 1'b1;
					INIT_SEQ <= INIT_SEQ + 1'b1;
					INIT_WR_REQ <= 1'b1;
				end
				else if (INIT_SEQ == 3'd4)
				begin
					INIT_SEQ <= 3'd0;
					INIT_WR_REQ <= 1'b0;
					if (INIT_DEST_ADDR == 22'h1FFFFF)
					begin
						INIT_STATE <= 3'd4;		// Copy done, run !
					end
					else
					begin
						INIT_DEST_ADDR <= INIT_DEST_ADDR + 1'b1;
					end
				end
				else
					INIT_SEQ <= INIT_SEQ + 1'b1;
			end
		end
	end

endmodule
