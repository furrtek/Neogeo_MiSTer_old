The code running on the ARM CPU (HPS) loads the requested core as
an .rbf file via fpga_load_rbf in fpga_io.cpp in Main_MiSTer.

In the main menu, the FPGA core acts as a special graphics & sound card for the
linux system which is able to display a text interface.

In each core, the stuff in /sys/ provides the video scaling, video
and audio output, the OSD, and the HPS interface logic.
The hps_io module "breaks out" a bunch of signals for the core to
utilize.

The core itself must take care of giving access to the SDRAM controller
for the HPS during ROM loading, and then switch to run mode to use the data.

The ROM files are loaded by user_io_file_tx in user_io.cpp
There's core-specific code in there (to skip the SNES ROM header for example)
so adding format convertion code for NeoGeo graphics here should be ok.

To connect to the DE10 via ethernet: run D:\OpenDHCPServer\runstandalone.bat
NOTE: It may already be running as a service !
Make sure the LAN connection is enabled in the Windows control panel
It will assign 192.168.1.200 to the DE10 by matching its MAC address
It's then possible to use FTP or SSH with root:1 as login

To debug linux-side stuff (Main_MiSTer):
Putty root:1@192.168.1.200
killall MiSTer so that the binary can be replaced
Ctrl+shift+B in VS to build
Upload new binary via FTP
Run /media/fat/MiSTer

The generated .rbf is output_files\neogeo-lite.rbf

neogeo-lite allows for faster compilation time by omitting the HDMI upscaler,
it outputs 6-6-6 VGA video for the IO board on the GPIO_1 header as follows:
VGA_R[0]: GPIO_1_D35
VGA_R[1]: GPIO_1_D33
VGA_R[2]: GPIO_1_D31
VGA_R[3]: GPIO_1_D29
VGA_R[4]: GPIO_1_D27
VGA_R[5]: GPIO_1_D25

VGA_G[0]: GPIO_1_D34
VGA_G[1]: GPIO_1_D32
VGA_G[2]: GPIO_1_D30
VGA_G[3]: GPIO_1_D28
VGA_G[4]: GPIO_1_D26
VGA_G[5]: GPIO_1_D24

VGA_B[0]: GPIO_1_D19
VGA_B[1]: GPIO_1_D21
VGA_B[2]: GPIO_1_D23
VGA_B[3]: GPIO_1_D22
VGA_B[4]: GPIO_1_D20
VGA_B[5]: GPIO_1_D18

VGA_HS: GPIO_1_D17
VGA_VS: GPIO_1_D16

Bottom header, pin 1 is upper right
 _39________________________________________________________01
|                                                             |
| G0 G1 G2 G3 G4 +3 G5 B3 B4 B5 VS -- -- -- -- +5 -- -- -- -- |
|                                                             |
| R0 R1 R2 R3 R4 gn R5 B2 B1 B0 HS -- -- -- -- gn -- -- -- -- |
|_____________________________________________________________|
  40                                                        02

A pin has to be shorted to ground to enable the VGA output. This is used to
detect the IO extension board.


SDRAM multiplexing:
The main SDRAM holds the 68k program, sprites and fix graphics.
It currently runs at 96MHz, 68k must run with 1 wait cycle :(

The system may request:
-68k data as soon as nSROMOE, nROMOE or nPORTOE goes low
-Fix graphics as soon as PCK2B rises
  Since the SDRAM provides 16-bit words and there's only an 8-bit data bus for the
  fix (2 pixels), one SDRAM read gives 4 pixels.
  The fix data is re-organized during loading so that pixel pairs are kept adjacent
  in the SDRAM. This allows to do only one read for 4 pixels and use S2H1 to select
  which pair must be fed to NEO-B1 at the right time.
-Sprite graphics as soon as PCK1B rises
  The sprite graphics data bus is 32-bit so for one 8-pixel line, two words must be
  read.
  The data is reorganized so that the bitplane data can be read in 4-word bursts and
  used as-is. The PCK1B edge is used to trigger the reads, and CA4 is used to select
  which 8-pixel group must be fed to NEO-ZMC2 at the right time.

If there's only one active request, just start the corresponding read cycle.
If there are multiple active requests use the following priorities:
-Sprite graphics
-Fix graphics
-68k data

If there's a read cycle already going on, DO NOT interrupt it. Keep sdram_addr valid.

FIXD is latched on rising edge of CLK_1MB: 36 sys_clk to valid data (250ns)
CR is latched on rising edge of CLK_12M+LOAD: 36 sys_clk to valid data (250ns)

Howto:
Detect edges of PCK1B, PCK2B, and nROMOE, etc... with two chained 1-bit registers
On the correct edges:
  If the *_REQ registers are all low, start the appropriate read cycle(s)
  Otherwise, set the appropriate *_REQ register

Fix graphics are loaded in a way that takes advantage of the 16-bit wide SDRAM
data bus. Since fix pixels are stored in columns but always read in lines,
instead of storing:
column 2 (lines 0~7), column 3 (lines 0~7), column 0 (lines 0~7), column 1 (lines 0~7)
Load like:
line 0 (columns 0~3), line 1 (columns 0~3)...

Sprite gfx bytes are loaded in SDRAM like this:
C2 C2 C1 C1 C2 C2 C1 C1...
So bitplanes look like this:
0  1  2  3  0  1  2  3
Burst reads for sprites gfx were not working because row and columns were inverted in
the SDRAM controller :/
To load a complete 16-pixel line, 4*16 = 64 bits = 4 16-bit SDRAM words must be loaded

ioctl_index is used to tell the core where to store the data being loaded. The
currently used values are:
0: System ROM (BIOS)
1: LO ROM
2~3: -
4: P1 first half (or full)
5: P1 second half
6: P2
7: -
8: S1
9: M1
10~31: -
32+: C ROMs, lower 5 bits are a bitfield used like this:
 xx1BBBBS
 B: 1MB bank
 S: word shift (used to interleave odd/even ROMs)
 In the end, SDRAM address = 1 BBBB0000 00000000 000000S0 + ioctl_addr


// Todo: "FPGAize" LSPC and NEO-B1: Group logic and clocked always() blocks
// Todo: Check if FD2 and FD4 are really clocked on negedge
// Todo: Find a way to prioritize 68k ROM reads to get rid of the wait states, use burst reads for prefetch and
//       run SDRAM back at 144MHz ?
//       SDRAM is now working at 144MHz again, burst reads for M68K were tested but caused random freezes ?
// Todo: See if it's possible to read a whole 8-pixel fix line with a n=2 burst read instead of 2x 4pixels
//       It's possible but it's better to do plenty of short reads than longer reads less often
//       Has less chances of blocking SDRAM access when the M68K reads
