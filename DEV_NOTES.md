These notes were written for myself/my setup but many points could be useful for others.

# MiSTer basics
The code running on the HPS (ARM CPU) loads the requested core as an .rbf file via fpga_load_rbf in fpga_io.cpp in Main_MiSTer.

In each core, the files in /sys/ provide the common functionalities: video scaling, video and audio output, the OSD, and the HPS interface logic. The hps_io module "breaks out" a bunch of signals for the core to utilize.

The core must be held in reset and provide access to the SDRAM controller and BRAM for the HPS during ROM loading. It can then switch to run mode and start using the data itself.

To connect to the DE10 via ethernet: run `D:\OpenDHCPServer\runstandalone.bat`
**NOTE**: It may already be running as a service ! Make sure the LAN connection is enabled in the Windows network control panel. By matching its MAC address, the DE10 will be assigned IP 192.168.1.200. It's then possible to use FTP or SSH with root:1 as login.

To debug linux-side stuff (Main_MiSTer):
Putty `root:1@192.168.1.200`, `killall MiSTer` so that the binary file can be replaced, Ctrl+shift+B in VS to build, upload new binary via FTP, sync, and run `/media/fat/MiSTer`.

neogeo-lite allows for faster compilation time by omitting the HDMI scaler, it outputs 6-6-6 VGA video and stereo audio on the GPIO_1 header as follows:
```
Bottom header, pin 1 is upper right
 _39________________________________________________________01
|                                                             |
| G0 G1 G2 G3 G4 +3 G5 B3 B4 B5 VS -- -- -- +5 RA -- -- -- -- |
|                                                             |
| R0 R1 R2 R3 R4 gn R5 B2 B1 B0 HS -- -- -- gn gn -- -- -- LA |
|_____________________________________________________________|
  40                                                        02
```
Pin 10 has to be connected to ground to enable the VGA output. This is used to detect the IO extension board.

# NeoGeo core specifics

The generated .rbf is `output_files\neogeo-lite.rbf`

The ROM files are normally loaded by user_io_file_tx from user_io.cpp. For this core, neogeo_romset_tx from `/support/neogeo/loader.cpp` is used instead. 

## Save files for cartridge games

Data | Start | End
---- | ---- | ----
Backup RAM | 000000 | 00FFFF
Memory card | 010000 | 0107FF

## Save file for CD systems

Data | Start | End
---- | ---- | ----
Memory card | 000000 | 001FFF

## SDRAM multiplexing
The SDRAM extension board stores the 68k program, sprites and fix graphics.
It currently runs at 120MHz (5*24M) but may be pushed up to 144MHz (6*24M) if needed.

### 68k data
Read as soon as nSROMOE (system ROM read), nROMOE (cart ROM read) or nPORTOE (cart ROM extension read) goes low.
Right now, the 68k always runs with 1 wait cycle :(

An attempt was made to fetch 68k data 4 words at a time with burst reads, but it didn't seem to be very reliable. Try again ?

### Fix graphics
Read as soon as PCK2B rises.
Since the SDRAM provides 16-bit words and there's only an 8-bit data bus for the fix (2 pixels), one SDRAM read gives 4 pixels.
The fix data is re-organized during loading so that pixel pairs are kept adjacent in the SDRAM. This allows to do only one read for 4 pixels and use S2H1 to select which pair must be fed to NEO-B1 at the right time.

An attempt was made to fetch fix data 2 words at a time with burst reads to have the whole 8 pixel line at once. It worked but it seems better to have multiple shorter reads for more efficient SDRAM access interleave.

FIXD is latched on rising edge of CLK_1MB.

### Sprite graphics
Read as soon as PCK1B rises.
The sprite graphics data bus is 32-bit, so for one 8-pixel line, two words must be read.
The sprite data is re-organized during so that the bitplane data can be read in 4-word bursts and used as-is. The PCK1B edge is used to trigger the reads, and CA4 is used to select which 8-pixel group must be fed to NEO-ZMC2 at the right time.

CR is latched on rising edge of CLK_12M when LOAD is high.

## Loading

Fix graphics are loaded in a way that takes advantage of the 16-bit wide SDRAM data bus.

Since fix pixels are stored in pairs in columns but always read in lines, instead of storing:
`column 2 (lines 0~7), column 3 (lines 0~7), column 0 (lines 0~7), column 1 (lines 0~7)`

Load like:
`line 0 (columns 0~3), line 1 (columns 0~3)...`

Sprite graphics bytes are loaded like this:
C2 C2 C1 C1 C2 C2 C1 C1...

So bitplanes for a single line look like this:
0  1  2  3  0  1  2  3...

Complete 16-pixel line: 4*16 = 64 bits = four 16-bit SDRAM words

ioctl_index is used to tell the core where to store the data being loaded. The currently used values are:
* 0: System ROM (BIOS)
* 1: LO ROM
* 2: SFIX ROM
* 4: P1 ROM first half (or full)
* 5: P1 ROM second half
* 6: P2 ROM
* 8: S1 ROM
* 9: M1 ROM
* 64+: C ROMs, the lower 6 bits are used as a bitfield like this:
 x1BBBBBS
 B: 512kB bank number
 S: word shift (used to interleave odd/even ROMs)
 So SDRAM address = 0x0800000 + 0b1_BBBBB000_00000000_000000S0 + ioctl_addr

## BRAM zones

Memory | Size
------ | ----
68k RAM | 64kB
Z80 RAM | 2kB
Slow VRAM | 64kB
Fast VRAM | 4kB
Palette RAM | 16kB
Line buffers | 9kB
LO ROM | 64kB
M1 ROM | 128kB
Memory card | 2kB
Backup RAM | 32kB

## SDRAM cart map

Memory | Size | Start | End
------ | ---- | ----- | ---
System ROM | 128kB | 0000000 | 001FFFF
Free | 384kB | 0020000 | 007FFFF
S ROM | 512kB | 0080000 | 00FFFFF
SFIX ROM | 128kB | 0100000 | 011FFFF
Free | 896kB | 0120000 | 01FFFFF
P ROMs | 6MB | 0200000 | 07FFFFF
C ROM | 24MB | 0800000 | 1FFFFFF

Games using NEO-CMC are able to bankswitch S ROMs larger than 128kB

## SDRAM CD map

Memory | Size | Start | End
------ | ---- | ----- | ---
System ROM | 512kB | 0000000 | 007FFFF
S ROM | 128kB | 0080000 | 009FFFF
Free | 1408kB | 00A0000 | 01FFFFF
P ROM | 1MB | 0200000 | 02FFFFF
Extended RAM | 1MB | 0300000 | 03FFFFF
Free | 4MB | 0400000 | 07FFFFF
C ROM | 4MB | 0800000 | 0BFFFFF
Free | 20MB | 0C00000 | 01FFFFF

# TODO

* Check if FD2 and FD4 are really clocked on negedge
