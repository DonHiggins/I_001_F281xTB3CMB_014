// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     ExtRam.C
//
//   Testing The 256K x 16 RAM chip on the Ext Bus at address 0x10,0000 - 0x13,FFFF
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "Rs232Out.H"
#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"
#include "TaskMgr.h"
#include "ExtRam.H"

// - - - - Here's What Works to Access the External RAM Chip - - - - -
//
//  In the TB3CMB rev of the control board we have a 256K x 16-bit RAM
//  chip on the external addr & data bus, addressable starting at 0x10,0000 - 0x13,FFFF.
//  Following are methods to access the external RAM memmory from DSP code.
//
// (1) we can use #defines to identify addr of RAM chip
#define EXTRAM_START_ADDR  0x100000L
#define EXTRAM_LEN 0x040000L
//  and use those values to load a pointer like so
//
//       volatile Uint16 *extRamStart = (Uint16 *)EXTRAM_START_ADDR;
//
// (2) in the linker .cmd file I used a START directive  like so:
//
//   .extram             : RUN /*LOAD*/ = EXTRAM,
//                         START(_ExtRamStart), /* give us a symbolic address */
//                         SIZE (_ExtRamSize),  /* give us symbolic val for # words allocated */
//                         PAGE = 1
//
//   to generate a symbol "_ExtRamStart", with the value of the start of the .extram
//   section (External Ram Chip address space).  Then in my DSP code I can declare
//   "ExtRamStart" as an extern, and use it to initialize a pointer like so:
//
		extern Uint16 ExtRamStart;
		extern Uint16 ExtRamSize;
	    union EXTRAM16_32 extRamSize;
//		extRamSize.all = (Uint32)&ExtRamSize;
//		Uint16 ramSize = extRamSize.words.lsw;
//
//  The above code works and leaves a 32 bit value in the extRamSize union
//  equal to the # of words of static memory allocated into section .extram
//
// (3) can use pragma DATA_SECTION to create static variables that will be
//  located within the .extram section, (External Ram Chip address space).
//  Note that with this method (3) the variables will be allocated starting at
//  the lowest address in .extram, eg: 0x10,0000 and if we do this, then we have
//  to account for it when using methods (1) or (2) above, so as not to write over
//  memory already allocated to static variables.
//
#pragma DATA_SECTION(xramData1, ".extram");
Uint16 xramData1;
#pragma DATA_SECTION(xramData2, ".extram");
Uint16 xramData2;
#pragma DATA_SECTION(xramData3, ".extram");
Uint16 xramData3;
//
// (4) Never got the LOCATION pragma to work
//  LOCATION Pragma didn't work -- something about "requires EABI"
//       #pragma LOCATION(xramData2, 0x100001);
//       Uint16 xramData2;
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void xram_testExternalRam(Uint16 dataWord){
// Called from Command interpreter c2020:0000cr
// For starters, we are testing our use of complier directives and cmd file
// definitions to properly identify address of the External RAM

	volatile Uint16 *extptr0;
	volatile Uint16 *extptr1;
//	volatile Uint16 *extptr2;
	volatile Uint16 *extptr3;
	Uint16 i;
	Uint16 err;
	Uint16 extRamVal;
    Uint16 ramSize;

    if (dataWord == 1){
    	xram_ExtRamRWTest();
    }

	// load 32-bit pointers with values of symbols from linker to point into Ext RAM
	extptr0 = (Uint16 *)EXTRAM_START_ADDR;  //0x10,0000 -- #define
	extptr1 = &xramData1;  //0x10,0000 - ram assigned via DATA_SECTION pragma

	// Write to DATA_SECTION ext Ram addr, read back from #define EXTRAM_START_ADDR
	err = 0;
	for (i=0;i<16;i++){
		*(extptr1++) = i;
		if (*extptr0 != i) {
			err++;
		}
		extptr0++;
	}

	// load 32-bit pointers with values of symbols from linker to point into Ext RAM

	extptr0 = (Uint16 *)EXTRAM_START_ADDR;  //0x10,0000 -- #define
	extptr3 = &ExtRamStart;  //0x10,0000 - symbolic address from .cmd file

	// Write to DATA_SECTION ext Ram addr, read back from #define EXTRAM_START_ADDR
	err = 0;
	for (i=0;i<16;i++){
		*(extptr3++) = i;
		if (*extptr0 != i) {
			err++;
		}
		extptr0++;
	}

	// Linker generated symbol "ExtRamSize" is a 32-bit quantity
	// telling how many 16-bit words have been allocated in section .extram
	// via aproach (3) above, using pragma DATA_SECTION.
	// Here we use a union, EXTRAM16_32 extRamSize, to extract the ls 16 bits
	// of the value of "ExtRamSize". Keep in mind that the RAM chip used in TB3CMB
	// 256k x 16-bits, and technically the value of "ExtRamSize" can overflow 16 bits.
	extRamSize.all = (Uint32)&ExtRamSize;
	ramSize = extRamSize.words.lsw;

	extRamVal = 0;
	extptr3 = &ExtRamStart;  //0x10,0000 - symbolic address from .cmd file
	for (i=0;i<ramSize;i++){
		extRamVal += *(extptr3++);
	}

}
void xram_ExtRamRWTest(){
// Called from Command interpreter c2020:0001cr
// run a test writing to and reading back from every word in external RAM chip.
// report results on diagnostic RS232.
	char msgOut[64];
    char *ptr;
    bool err;

    err = xram_RWPassThruWholeChip(0xA5F0);

	if (err == false) {
		ptr = strU_strcpy(msgOut,"  Ext RAM test OK, no errors.\n\r");
	} else {
		ptr = strU_strcpy(msgOut,"  Ext RAM test reports error.\n\r");
	}

	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

}

bool xram_RWPassThruWholeChip(Uint16 seed){
// called: err = xram_RWPassThruWholeChip(seed);
// Fill all words of Ext RAM chip with sequential 16-bit values starting at seed.
// Read back values sequentially, compare with value written,
// report success or failure.

	Uint32 i32;
	Uint16 *extRam;
    union EXTRAM16_32 extRamValue;

    // Write values
	extRam = (Uint16 *)EXTRAM_START_ADDR;  //0x10,0000 -- #define
	extRamValue.words.lsw = seed;
	extRamValue.words.msw = 0;
	for (i32=0L;i32<EXTRAM_LEN;i32++) {
		*(extRam++) = extRamValue.words.lsw;
		extRamValue.all++;
	}

	// Read back and compare
	extRam = (Uint16 *)EXTRAM_START_ADDR;  //0x10,0000 -- #define
	extRamValue.words.lsw = seed;
	extRamValue.words.msw = 0;
	for (i32=0L;i32<EXTRAM_LEN;i32++) {
		if(*(extRam++) == extRamValue.words.lsw) {
		   extRamValue.all++;
		} else {
			return true; // error
		}
	}

	return false; // no error
}
