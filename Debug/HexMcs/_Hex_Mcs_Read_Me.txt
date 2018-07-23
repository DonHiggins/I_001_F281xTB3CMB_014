_Hex_Mcs_Read_Me.txt


Running to_hex_Mcs.bat creates a readable (Intel MCS format -- .mcs) ROM image of all
the loadable code and data sections from the project.

The mcs-format file, .mcs, was useful for viewing the object image, when I was
comparing it with the .boot (binary) image in the HexBoot file directory.  I had to 
use a hex editor to view the .boot file, and I was checking to make sure it contained
all the data plus proper headers to represent the code segments from the COFF .out file.

The mcs-format file is also what we use when we send a program to the CAN bootloader.

The command file, to_hex_Mcs.cmd,  uses a ROMS directive which identifies the 
target memory range starting at 0x003E4000 for a length of10000 words (corresponding
to Flash sections FLASHC, D, E, and FLASHF.

Current plan is to use FLASHA & B for bootloader, FLASHC, D, E, and F for the main
program.

