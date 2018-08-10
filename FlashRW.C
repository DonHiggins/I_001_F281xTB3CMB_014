// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     FlashRW.C
//
//   Read, Write, or Erase portions of the FLASH memory
//       Accept commands and data from CAN communications.
//       Return data read from FLASH over CAN.
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
//#include "CanComm.h"
#include "Rs232Out.H"
//#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"
#include "CanOpen.h"
#include "FlashRW.h"
#include "Spi.H"
#include "TaskMgr.h"
#include "McsParse.H"
#include "GpioUtil.H"
#include "Timer0.H"
#include "CPLD.H"
#include "Log.H"
#include "LED.H"

#define BITSTREAM_BLOCKSIZE 1024

#define MAX_FPGAS_TO_LOAD 2
// Don't bump this from 2 to 3 until after we get Flash3 & Fpga3 working
// Otherwise it will error out every time we power up.
// #define MAX_FPGAS_TO_LOAD 3

extern struct MULTI_PACKET_BUF multi_packet_buf;

Uint16 frwWhichFlashChip; //1=FLASH_1, 2=FLASH_2, 3=FLASH_3
enum LOAD_FPGAS_AT_STARTUP_STATUS frwLoadFpgasAtStartupStatus;
bool frwLoadMultipleFgpas;
union CANOPEN16_32 frwFlashAddr;
#define maxWordsInFlashRWBuff 64
Uint16 flashRWBuff[maxWordsInFlashRWBuff];
Uint16 flashRWBuffFillIndex;
Uint16 fastRWBuff[64];
bool fastRWBuffInUse;
Uint16 fastRWBuffCharCount;
bool mcsFileRecvInProgress = false;
Uint16 mcsFileRecvStatus;
Uint16 mcsFileRecvError;
Uint16 mcsFileRecvAddrExtension;
Uint16 mcsFileRecvParseStatus;
bool frwMcsNotFastDownload;
Uint16 frw_bulkEraseToken;
enum MISC_FLASH_TASK_OPERATION miscFlashTaskOperation;
Uint16 miscFlashTaskState;
union CANOPEN16_32 mcsFileSendByteCount;
Uint16 mcsFileSendByteAddr;
Uint16 bitstream_blockcount;
enum LOAD_FPGA_FROM_FLASH_STATE frwFlashTaskState; // state variable
union CANOPEN16_32 bitStreamOffset;
union CANOPEN16_32 bitStreamLength;

union CANOPEN16_32 frwDiagFlashAddr;


// define bits in flash status register
#define    WIP            0x01
#define    WEL            0x02
#define    BPX            0x0C
#define    WPEN           0x80

void frw_RemoveHwWriteProtect(){
// tell CPLD to raise the hardware ~WriteProtect line
	Uint16 *extData; // pointer for external bus address to access CPLD

	if (frwWhichFlashChip == 1){    //1=FLASH_1, 2=FLASH_2, 3=FLASH_3
		extData = CPLD_XINTF_ADDR(TBIOM_WP_FLASH_1);
	} else if (frwWhichFlashChip == 2){
		extData = CPLD_XINTF_ADDR(TBIOM_WP_FLASH_2);
	} else {
		extData = CPLD_XINTF_ADDR(TBPM_WP_FLASH_3);
	}
	*extData = 0; // write, de-assert ~WP, data value not important
}

void frw_AssertHwWriteProtect(){
// tell CPLD to lower the hardware ~WriteProtect lines for all 3 FLASH chips
	Uint16 *extData; // pointer for external bus address to access CPLD
	volatile Uint16 dummyDdataIn;

	extData = CPLD_XINTF_ADDR(TBIOM_WP_FLASH_1);
	dummyDdataIn = *extData; // read, assert ~WP, not interested in value read in
	extData = CPLD_XINTF_ADDR(TBIOM_WP_FLASH_2);
	dummyDdataIn = *extData; // read, assert ~WP, not interested in value read in
	extData = CPLD_XINTF_ADDR(TBPM_WP_FLASH_3);
	dummyDdataIn = *extData; // read, assert ~WP, not interested in value read in
}


// So, what sort of commands do we want to implement . . .

// 1. Read Status Register
//      do this stand-alone,
//      and also do it in conjunction w/ program & erase actions
//      for starters, lets assume we can do this quick enough to avoid
//      breaking it into tasks.  Assume we can respond to a CAN command to
//      read the status register, and return the result over CAN.
//      If it's not that fast, we can break it down later.

// 2. Page Programming -- up to 256 data bytes within a page boundary 0xNNNxx
//      3-byte addr identifies first byte to write

// 3. Sector Erase -- 3-byte addr identifies sector 0xN0000
//      N is 0 to 8

// 4. Read Data Bytes -- 3-byte addr identifies first byte to read
//      reads contiguous bytes, as many as we want.

// 5. Write Enable
//      probably need to do this in conjunction w/ program & erase actions


// Initialize a few variables before launching frw_SpiFlashTask()
void frw_SpiFlashInit(){
	frwFlashTaskState = LOAD_FPGA_AT_STARTUP;
	frwLoadMultipleFgpas = true;
	frwLoadFpgasAtStartupStatus = F_LOAD_NOT_STARTED;
}

// Select between 2 FLASH chips, store selection in globals
// All subsequent flash operations are direcrted against selected flash chip
void frw_SetWhichFlash(Uint16 whichFlash){
   frwWhichFlashChip = whichFlash; // 1=FLASH_1, 2=FLASH_2, 3=FLASH_3, others illegal
}
Uint16 frw_GetWhichFlash(void){
	return frwWhichFlashChip;
}

enum LOAD_FPGAS_AT_STARTUP_STATUS frw_GetLoadFpgasAtStartupStatus(void){
	return frwLoadFpgasAtStartupStatus;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  G P I O   P I N S
//     Used to Load FPGAs from FLASH
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// 3 routines, to
//   1) Initialize direction and HI/LOW state frw_initFpgaLoadPins(void)
//   2) Dynamically reconfigure direction and HI/LOW state
//   3) Read HI/LOW state of input
//
// Selection of GPIO pins changed from TB3IOMA to TB3IOMB
// and some signals come from a CPLD output on TB3IOM,
// rather than from a DSP GPIO pin.
//
// FPGA1     TB3CMB     TB3CMA
//   -----   -------    ------
//   prog    GPIOF8     GPIOF9
//   done    GPIOA0     GPIOF11
//   init    GPIOB12    GPIOB14
//   reset   (cpld)     (cpld)
//
// FPGA2     TB3CMB     TB3CMA
//   -----   -------    ------
//   prog    GPIOF9     (cpld)
//   done    GPIOA1     (cpld)
//   init    GPIOB13    (cpld)
//   reset   (cpld)     (cpld)
//
// FPGA3     TB3CMB
//   -----   -------
//   prog    GPIOF10
//   done    GPIOA2
//   init    GPIOB14
//   reset   (cpld)

void frw_initFpgaLoadPins(void){
    // Setup the PROG pins to the FPGAs as output, and set it HI
	// Used in loading FPGA Program from FLASH
#ifdef TB3CMA_GPIO
    EALLOW;
    GpioMuxRegs.GPFDIR.bit.GPIOF9 = 1;   // GPIOF9, PROG output for FPGA1, TB3CMA
    EDIS;
    GpioDataRegs.GPFSET.all |= 0x0200;   // set PROG, GPIOF9, HI
#endif
#ifdef TB3CMB_GPIO
    EALLOW;
    GpioMuxRegs.GPFDIR.bit.GPIOF8 = 1;   // GPIOF8, PROG output for FPGA1, TB3CMB
    GpioMuxRegs.GPFDIR.bit.GPIOF9 = 1;   // GPIOF9, PROG output for FPGA2, TB3CMB
    GpioMuxRegs.GPFDIR.bit.GPIOF10 = 1;   // GPIOF10, PROG output for FPGA3, TB3CMB
    EDIS;
    GpioDataRegs.GPFSET.all |= 0x0700;   // set PROG, GPIOF8 & 9 & 10, HI
#endif
}

// Manipulate control lines from DSP & CPLD to load program into FPGA.
// This routine is called from the background task which implements a state machine
// to control the process of loading code into the FPGAs.  This routine
// performs a simple, I/O line manipulation as requested by the caller
// based on (1) value of (enum F_LOAD_CTRL_OPS)ctrlLineOp,
// and based on (2) which of the 3 FLASH/FPGA pairs we are working with.
// Operations that read and return the HI/LOW value of a control line
// are implemented in the next routine down, frw_ReadFpgaCtrlLines()
void frw_SetFpgaCtrlLines(enum F_LOAD_CTRL_OPS ctrlLineOp){

//	enum F_LOAD_CTRL_OPS {
//		F_LOAD_RESET_LOW	= 0,  drop the RESET line LOW to the FPGA
//		F_LOAD_PROG_OUT_HI	= 1,  config the PROG line as an output with a HI out value
//		F_LOAD_PROG_LOW		= 2,  set the PROG line LOW
//		F_LOAD_READ_INIT	= 3,  read the HI/LOW value of the INIT input
//		F_LOAD_PROG_IN		= 4,  config the PROG line as an input
//		F_LOAD_READ_DONE	= 5,  read the HI/LOW value of the DONE input
//		F_LOAD_RESET_HI		= 6   set the RESET line HI to the FPGA

	Uint16 whichFlash;		// This tells which FPGA & Flash we are working with: 1, 2, or 3
	volatile Uint16 *extData; // pointer for external bus address to access CPLD
    volatile Uint16 data_in;

    whichFlash = frw_GetWhichFlash(); // Flash # 1 == FPGA # 1, etc.

#ifdef TB3CMA_GPIO
    switch(ctrlLineOp){
    case F_LOAD_RESET_LOW:
    	if (whichFlash == 1) {
    	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA1_RESET);
    	} else {
     	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA2_RESET);
    	}
    	// *** Eventually this may need to work with PWB Flash/Fpga also ****
    	data_in = *extData; // data not important, read sets ~RESET LOW,
        break;
    case F_LOAD_PROG_OUT_HI:
    	if (whichFlash == 1) {
    		GpioU_Fpga1ProgHI();
    	} else {
    		extData = CPLD_XINTF_ADDR(TBIOM_F_PRGM_2);
    		*extData = 1; // data not important, write sets F_PRGM_2 HI
    	}
        break;
    case F_LOAD_PROG_LOW:
    	if (whichFlash == 1) {
        	GpioDataRegs.GPFCLEAR.all |= 0x0200;  // set PROG, GPIOF9, LOW
    	} else {
    		extData = CPLD_XINTF_ADDR(TBIOM_F_PRGM_2);
    		data_in = *extData; // data not important, read sets F_PRGM_2 LOW
    	}
        break;
    case F_LOAD_PROG_IN:
	    if (whichFlash == 1) {
	    	EALLOW;
	    	GpioMuxRegs.GPFDIR.bit.GPIOF9 = 0;   // GPIOF9, PROG input
	    	EDIS;
	    } else {
            // WHAT TO DO FOR FPGA2 ??? Tristate it
	    }
        break;
    case F_LOAD_RESET_HI:
    	if (whichFlash == 1) {
    	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA1_RESET);
    	} else {
     	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA2_RESET);
    	}
    	*extData = 1; // data not important, write sets ~RESET HI,
        break;
    default:
    	break;
    }
#endif

#ifdef TB3CMB_GPIO
    switch(ctrlLineOp){
    case F_LOAD_RESET_LOW: // drop the RESET line LOW to the FPGA
    	if (whichFlash == 1) {
    	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA1_RESET);
    	} else if (whichFlash == 2) {
     	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA2_RESET);
    	} else {
      	   extData = CPLD_XINTF_ADDR(TBPM_FPGA3_RESET);
    	}
    	data_in = *extData; // data not important, read sets ~RESET LOW,
        break;

    case F_LOAD_PROG_OUT_HI: // config the PROG line as an output with a HI out value
    	if (whichFlash == 1) {
    	    EALLOW;
    	    GpioMuxRegs.GPFDIR.bit.GPIOF8 = 1;   // GPIOF8, PROG output for FPGA1, TB3CMB
    	    EDIS;
    	    GpioDataRegs.GPFSET.all |= 0x0100;   // set PROG, GPIOF8, HI
    	} else if (whichFlash == 2) {
    	    EALLOW;
    	    GpioMuxRegs.GPFDIR.bit.GPIOF9 = 1;   // GPIOF9, PROG output for FPGA2, TB3CMB
    	    EDIS;
    	    GpioDataRegs.GPFSET.all |= 0x0200;   // set PROG, GPIOF9, HI
    	} else {
    	    EALLOW;
    	    GpioMuxRegs.GPFDIR.bit.GPIOF10 = 1;   // GPIOF10, PROG output for FPGA3, TB3CMB
    	    EDIS;
    	    GpioDataRegs.GPFSET.all |= 0x0400;   // set PROG, GPIOF10, HI
    	}
        break;

    case F_LOAD_PROG_LOW: // set the PROG line LOW
    	if (whichFlash == 1) {
        	GpioDataRegs.GPFCLEAR.all |= 0x0100;  // set PROG, GPIOF8, LOW
    	} else if (whichFlash == 2) {
        	GpioDataRegs.GPFCLEAR.all |= 0x0200;  // set PROG, GPIOF9, LOW
    	} else {
        	GpioDataRegs.GPFCLEAR.all |= 0x0400;  // set PROG, GPIOF10, LOW
    	}
        break;

    // looking for case F_LOAD_READ_INIT ? see frw_ReadFpgaCtrlLines() below

    case F_LOAD_PROG_IN: // config the PROG line as an input
	    if (whichFlash == 1) {
	    	EALLOW;
	    	GpioMuxRegs.GPFDIR.bit.GPIOF8 = 0;   // GPIOF8, PROG input for FPGA1, TB3CMB
	    	EDIS;
    	} else if (whichFlash == 2) {
	    	EALLOW;
	    	GpioMuxRegs.GPFDIR.bit.GPIOF9 = 0;   // GPIOF9, PROG input for FPGA2, TB3CMB
	    	EDIS;
	    } else {
	    	EALLOW;
	    	GpioMuxRegs.GPFDIR.bit.GPIOF10 = 0;   // GPIOF10, PROG input for FPGA3, TB3CMB
	    	EDIS;
	    }
        break;

	// looking for case F_LOAD_READ_DONE ? see frw_ReadFpgaCtrlLines() below

    case F_LOAD_RESET_HI: // set the RESET line HI to the FPGA
    	if (whichFlash == 1) {
    	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA1_RESET);
    	} else if (whichFlash == 2) {
     	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA2_RESET);
    	} else {
      	   extData = CPLD_XINTF_ADDR(TBPM_FPGA3_RESET);
    	}
    	*extData = 1; // data not important, write sets ~RESET HI,
        break;
    default:
    	break;
    }
#endif
}

// This routine simply reads the HI/LOW value of a control line as requested
// by the caller. The action it performs is based on
// (1) value of (enum F_LOAD_CTRL_OPS)ctrlLineOp,
// and based on (2) which of the 3 FLASH/FPGA pairs we are working with.
// Operations that configure and manipulate these control lines
// are implemented in the next previous routine above, frw_SetFpgaCtrlLines().

Uint16 frw_ReadFpgaCtrlLines(enum F_LOAD_CTRL_OPS ctrlLineOp){
    Uint16 whichFlash;
	// volatile Uint16 *extData; // pointer for ernal bus address to access CPLD
	volatile Uint16 data_in;

    whichFlash = frw_GetWhichFlash(); // Flash # 1 == FPGA # 1, etc.

#ifdef TB3CMA_GPIO
    switch(ctrlLineOp){
    case F_LOAD_READ_INIT: // read the HI/LOW value of the INIT input
    	if (whichFlash == 1) {
  	      //data_in = (GpioDataRegs.GPBDAT.all & 0x4000); // read INIT pin
  	      data_in = (GpioDataRegs.GPBDAT.all >> 14) & 0x1;
    	} else {
    	  extData = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, FPGA1_READ_F_INIT_DONE_2);
     	  data_in = *extData & 0x01;
    	}
    	// *** Eventually this may need to work with PWB Flash/Fpga also ****
        return data_in;
    	// --break;--
    case F_LOAD_READ_DONE: // read the HI/LOW value of the DONE input
    	if (whichFlash == 1) {
        	//data_in = (GpioDataRegs.GPFDAT.all & 0x0800); // read DONE pin, GPIOF11
        	data_in = (GpioDataRegs.GPFDAT.all >> 11) & 0x1;
    	} else {
      	  extData = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, FPGA1_READ_F_INIT_DONE_2);
       	  data_in = (*extData >> 1) & 0x01;
    	}
        return data_in;
        // --break;--
    default:
    	break;
#endif

#ifdef TB3CMB_GPIO
    switch(ctrlLineOp){
    case F_LOAD_READ_INIT: // read the HI/LOW value of the INIT input
    	if (whichFlash == 1) {
    		data_in = (GpioDataRegs.GPBDAT.all >> 12) & 0x1; //GPIOB12 INIT FPGA1 TB3CMB
    	} else if (whichFlash == 2) {
    		data_in = (GpioDataRegs.GPBDAT.all >> 13) & 0x1; // GPIOB13 INIT FPGA2 TB3CMB
    	} else {
    		data_in = (GpioDataRegs.GPBDAT.all >> 14) & 0x1; // GPIOB14 INIT FPGA3 TB3CMB
    	}
        return data_in;
    	// --break;--

    case F_LOAD_READ_DONE: // read the HI/LOW value of the DONE input
    	if (whichFlash == 1) {
        	data_in = (GpioDataRegs.GPADAT.all >> 0) & 0x1; // GPIOA0 DONE FPGA1 TB3CMB
    	} else if (whichFlash == 2) {
        	data_in = (GpioDataRegs.GPADAT.all >> 1) & 0x1; // GPIOA1 DONE FPGA2 TB3CMB
    	} else {
        	data_in = (GpioDataRegs.GPADAT.all >> 2) & 0x1; // GPIOA2 DONE FPGA3 TB3CMB
    	}
        return data_in;
        // --break;--
    default:
    	break;
#endif
    }
    return 0;
}

// * * *   T E M P O R A R Y   * * * to figure why we need a delay before loading FPGA
//void frw_rs232_Diag_SpiFlashTask(Uint16* words_from_flash_6, Uint16 calc_hash_value){
//	char msgOut[64];
//    char *ptr;
//    Uint16 i;
//
//   	ptr = msgOut;
//    ptr = strU_strcpy(msgOut,"frw_SpiFlashTask - ");
//    ptr = strU_strcpy(ptr,"\n\r");
//	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
//
//   	ptr = msgOut;
//    ptr = strU_strcpy(msgOut,"words_from_flash_6: ");
//    for (i=0;i<6;i++){
//    	// display Words as hex
//    	ptr = strU_strcpy(ptr,"0x");
//    	ptr = hexUtil_binTo4HexAsciiChars(ptr,*(words_from_flash_6++));
//    	ptr = strU_strcpy(ptr," ");
//    	// display Chars as Ascii
//	}
//	ptr = strU_strcpy(ptr,"\n\r");
//	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
//
//	ptr = strU_strcpy(msgOut,"calc_hash_value: 0x");
//	// display Words as hex
//	ptr = hexUtil_binTo4HexAsciiChars(ptr, calc_hash_value);
//	ptr = strU_strcpy(ptr," frw_GetWhichFlash: 0x");
//	ptr = hexUtil_binTo4HexAsciiChars(ptr, frw_GetWhichFlash());
//	// display Chars as Ascii
//	ptr = strU_strcpy(ptr,"\n\r");
//	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
//
//}
// * * *   E N D   T E M P O R A R Y    * * * * *


void frw_SpiFlashTask(){
	// This task is launched at power up (with frwFlashTaskState = LOAD_FPGA_AT_STARTUP.
	// Alternately, it may be launched on CAN command with frwFlashTaskState = LOAD_FPGA_ON_CAN_REQUEST.
	// It's job is to program the FPGAs from Flash
	// It maintains a state variable, frwFlashTaskState,
	// and re-launches itself until all steps of its job are complete
	// Q: Which FPGA are we loading?
	// A: the FPGA assoc. w/ the FLASH identified by frw_GetWhichFlash( ), ie: frwWhichFlashChip
    Uint16 wordsFromFlash[8];
    Uint16 j;
    Uint32 i;
    Uint16 countWordsToRead;
	volatile Uint16 *extData; // pointer for ernal bus address to access CPLD
	volatile Uint16 data_in;
	volatile Uint16 checkSumFromHeader; // "volatile" avoids warning until we
                                        // extend code to use this info

	switch(frwFlashTaskState){
    case LOAD_FPGA_NO_OP: // test turned off
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// This is a final exit from task activity
    	//  by executing a return here, we do not re-launch this task.
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	return;
        // break; // commented out because it yields a compiler warning as "unreachable"

    case LOAD_FPGA_AT_STARTUP:
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// This is a starting point, when the task is first launched from main.c initialization
    	// Drop through to to be handled by the LOAD_FPGA_ON_CAN_REQUEST case
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	frwLoadFpgasAtStartupStatus = F_LOAD_IN_PROGRESS;
    case LOAD_FPGA_ON_CAN_REQUEST:// command CPLD to set all enable lines to "disable" for devices on the SPI bus
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// This is a starting point, when the task is requested via a CAN message
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	spi_disableAllSpiDevices();
    	LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_AT_STARTUP,frwWhichFlashChip,0);
    	frwFlashTaskState = LOAD_FPGA_READ_OFFSET_LENGTH;
    	break;

    case LOAD_FPGA_READ_OFFSET_LENGTH: // Read 10 bytes at FLASH address 0
                                       // 4 bytes offset, 4 bytes bit-stream length (in bytes), 2 bytes hash
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// Before clocking data into FPGA,
    	//    read header info from the first 0x100-byte page of FLASH
    	//    find offset and byte-count for configuration data in FLASH
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// First 0x100 byte page in FLASH is header information,
    	LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_READ_OFFSET_LENGTH,frwWhichFlashChip,0);
    	frwFlashAddr.all = 0L;
    	spi_ReadFlash(&(frwFlashAddr.words.lsw) , (12>>1), wordsFromFlash);
    	bitStreamOffset.words.msw = wordsFromFlash[0];
    	bitStreamOffset.words.lsw = wordsFromFlash[1];
    	bitStreamLength.words.msw = wordsFromFlash[2];
    	bitStreamLength.words.lsw = wordsFromFlash[3];

    	// Here we read in the CheckSum from the FLASH header block this should
    	// Equal the simple sum (mod 2^16) of all 16-bit words making up the FPGA data stream
    	// Potentially we could read the entire data stream and calculate this value
    	// as a check before loading the data stream into the FPGA.
    	// As of 11/12/2014 we haven't implemented the read-and-calculate code.
    	checkSumFromHeader = wordsFromFlash[5]; // 16-bit checkSum of FPGA Configuration Stream

    	// Check the hash
    	// exclusive-or of first 8 bytes is found in byte 9
    	// byte 10 is 0xA5
    	data_in = 0;
    	for (j=0;j<4;j++) {
    	  data_in ^= ((wordsFromFlash[j]>>8) & 0xFF);
  	      data_in ^= (wordsFromFlash[j] & 0xFF);
    	}

// * * *   T E M P O R A R Y   * * * to figure why we need delay before loading FPGA
//    	frw_rs232_Diag_SpiFlashTask(wordsFromFlash, data_in);
// * * *   E N D   T E M P O R A R Y    * * * * *


    	if (((data_in ^ ((wordsFromFlash[4]>>8)& 0xFF)) == 0) && ((wordsFromFlash[4] & 0xFF) == 0xA5)) {
    	   // passed the hash check
            spi_ClockFlashToFpgaStart(&(bitStreamOffset.words.lsw)); // Set read-address into FLASH
        	LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_READ_OFFSET_LENGTH,frwWhichFlashChip,data_in);
        	frwFlashTaskState = LOAD_FPGA_PROG_AND_INIT; // n step is to manipulate PROG and INIT_B lines
         	break;
    	} else {
    		// failed the hash check
    		led_dspLedErrMsg(LED_ERROR_BAD_FLASH_BLOCK_0000); // blink the LEDs
        	LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_READ_OFFSET_LENGTH,frwWhichFlashChip,data_in);
        	frwFlashTaskState = LOAD_FPGA_ERROR_CLEANUP; // clean up before exit
    	}
     	break;

    case LOAD_FPGA_PROG_AND_INIT:// manipulate FPGA's PROG and INIT_B Lines
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// Give HI->LOW transition the Fpga's PROG line to trigger device initialization.
    	// Monitor FPGA's INIT_B line to verify it is ready for configuration  data.
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// to initialize FPGA and prepare it to be configured.

    	LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_READ_OFFSET_LENGTH,frwWhichFlashChip,0);

    	// Before programming FPGA1, set the PROG line for FPGA2 LOW, otherwise
    	// FPGA2 will think it is being programmed, and come out of configuration
    	// mode at the same time as FPGA1.
    	if (frw_GetWhichFlash() == 1) {     // Flash # 1 == FPGA # 1, etc.
    		extData = CPLD_XINTF_ADDR(TBIOM_F_PRGM_2);
    		data_in = *extData; // data not important, read sets F_PRGM_2 LOW
    	}

    	// First, make sure the ~RESET line to the FPGA is LOW
    	frw_SetFpgaCtrlLines(F_LOAD_RESET_LOW);

    	// Set PROG HI
    	frw_SetFpgaCtrlLines(F_LOAD_PROG_OUT_HI);

    	// Hold PROG HI for at least 500 nSec
        //  measured: 10 loops, app 5.0 uSec
    	for(i=10L;  i>0L;   i--) { asm("   NOP; "); }

    	// Set PROG LOW, giving the HI-to-LOW transition to trigger the FPGA reload cycle
    	frw_SetFpgaCtrlLines(F_LOAD_PROG_LOW);

    	// Look for FPGA to pull INIT LOW, within a time window
    	for(i=0L; i < 2000000L; i++)
    	      {
    	      asm("   NOP; ");
    	      if (i <  100L)         continue;
    	      data_in = frw_ReadFpgaCtrlLines(F_LOAD_READ_INIT);
    	      if (data_in == 0)  break;
    	      }
   	    if (data_in != 0) {
   	    	// Failure, INIT pin did not go LOW
   	    	led_dspLedErrMsg(LED_ERROR_BAD_INIT_B_NOT_LOW); // blink the LEDs
  	    	frwFlashTaskState = LOAD_FPGA_ERROR_CLEANUP;
  	        LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_READ_OFFSET_LENGTH,frwWhichFlashChip,0);
            break; // exit the case statement
   	    }
        // FPGA did pull INIT LOW,
   	    // now raise RROG HI, look for INIT to go HI
   	    frw_SetFpgaCtrlLines(F_LOAD_PROG_OUT_HI);
   	    for(i=3000000L;  i>0;  i--) {
   	    	// measure: 1000000 loops -> 210 uSec
   	        data_in = frw_ReadFpgaCtrlLines(F_LOAD_READ_INIT);
   	        if (data_in!=0)  break; // break out of the for loop
   	        }
      	if (data_in==0) {
    	    // Failure, INIT pin did not go HI
      		led_dspLedErrMsg(LED_ERROR_BAD_INIT_B_NOT_HI); // blink the LEDs
   	    	frwFlashTaskState = LOAD_FPGA_ERROR_CLEANUP;
  	        LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_READ_OFFSET_LENGTH,frwWhichFlashChip,0);
            break; // exit the case statement
      	}
    	// FPGA is ready to receive configuration data (next)
      	// advance to next state in this state machine
	    LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_READ_OFFSET_LENGTH,frwWhichFlashChip,0);
	    bitstream_blockcount = 0;
     	frwFlashTaskState = LOAD_FPGA_CLOCK_DATA_IN;
     	break;

    case LOAD_FPGA_CLOCK_DATA_IN:// As we read data out of the FLASH it is clocked into the FPGA
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// Read from FLASH address bitStreamOffset, one block at a time.
    	//   We set that read address back in the LOAD_FPGA_READ_OFFSET_LENGTH step.
    	// Total # of bytes of config data is bitStreamLength.
    	// we convert from bytes to words for SPI clocking routine
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	if (bitStreamLength.all < BITSTREAM_BLOCKSIZE) {
    		countWordsToRead = (bitStreamLength.words.lsw + 1)>>1;
        	bitStreamLength.all = 0L;
    	}else {
    		countWordsToRead = BITSTREAM_BLOCKSIZE>>1;
        	bitStreamLength.all -= BITSTREAM_BLOCKSIZE;
    	}

    	if ((bitstream_blockcount & 0x003F) == 0) {
    	    LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_CLOCK_DATA_IN,frwWhichFlashChip,bitstream_blockcount);
    	}
        spi_ClockFlashToFpga(countWordsToRead); // clock bits out from read-address set earlier
        if(bitStreamLength.all != 0L){
        	frwFlashTaskState = LOAD_FPGA_CLOCK_DATA_IN; // not done, clock more data in
        } else {
           // Done loading the FPGA
    	   // Return configuration of the only DSP output line we were using, PROG, to be an input
           frw_SetFpgaCtrlLines(F_LOAD_PROG_IN);
    	   spi_disableAllSpiDevices();
    	   frwFlashTaskState = LOAD_FPGA_COMPLETE_CONFIGURATION; // look for DONE to go HI
        }
    	if ((bitstream_blockcount & 0x003F) == 0) {
    	    LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_CLOCK_DATA_IN,frwWhichFlashChip,bitstream_blockcount);
    	}
        bitstream_blockcount++;
       break;


    case LOAD_FPGA_COMPLETE_CONFIGURATION: // Clocked in all data, look for DONE to go HI
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// INIT_B line goes low in the event of a CRC error.
    	// DONE line goes HI upon successful configuration.
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	    LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_COMPLETE_CONFIGURATION,frwWhichFlashChip,0);
        data_in = frw_ReadFpgaCtrlLines(F_LOAD_READ_INIT);
    	if (data_in==0) {
    		// INIT_B pulled LOW, signalling CRC error in FPGA configuratiion
    		led_dspLedErrMsg(LED_ERROR_FPGA_CRC); // blink the LEDs
    		frwFlashTaskState = LOAD_FPGA_ERROR_CLEANUP;
    		break;
    	}
        data_in = frw_ReadFpgaCtrlLines(F_LOAD_READ_DONE); // read DONE pin
    	if (data_in==0) {
    		// DONE did not go HI, FPGA not properly configured
    		led_dspLedErrMsg(LED_ERROR_DONE_LINE_LOW); // blink the LEDs
    		frwFlashTaskState = LOAD_FPGA_ERROR_CLEANUP;
        	break;
    	}
    	// by default, DONE is Hi, and we have successfully configured the FPGA
    	// De-assert the ~RESET line to the FPGA -- set it HI
    	frw_SetFpgaCtrlLines(F_LOAD_RESET_HI);

    	//clean up data lines, return PROG output to be an input when not in use
    	frw_SetFpgaCtrlLines(F_LOAD_PROG_IN);
    	spi_disableAllSpiDevices();

    	// Successful programing that FPGA, see if we have more FPGAs to Program
	    LOG_FLOAD_ADDTOLOG3(LOG_EVENT_FPGA_LOAD,LOAD_FPGA_COMPLETE_CONFIGURATION,frwWhichFlashChip,0);
    	if (frwLoadMultipleFgpas) {
    		// see if we have more FPGA's to Load
    		j = frw_GetWhichFlash();
    		if (j < MAX_FPGAS_TO_LOAD) {
    			frw_SetWhichFlash(j+1);
    	    	frwFlashTaskState = LOAD_FPGA_AT_STARTUP; // Start Next FPGA
    	    	break; // this "break" applies to the switch-case statement, not to the "if"
    		} else {
    	    	frwLoadFpgasAtStartupStatus = F_LOAD_COMPLETE;
    		}
    	}

    	led_setDspLedPattern(LED_PATTERN_SLOW_HEARTBEAT); // blink TB3CM LEDs

    	// Write to FPGA's to synchronize LED heartbeats
    	// (slow_heartbeat is default FPGA LED activity when they first come up)
    	extData = CPLD_F1_XA(FPGA1_WRITE_RESET_COUNT_CLK);
    	*extData = 1; // data not important, write resets FPGA's internal counter,
    	extData = CPLD_F2_XA(FPGA2_WRITE_RESET_COUNT_CLK);
    	*extData = 1; // data not important, write resets FPGA's internal counter,
    	extData = CPLD_F3_XA(FPGA3_WRITE_RESET_COUNT_CLK);
    	*extData = 1; // data not important, write resets FPGA's internal counter,

    	frwFlashTaskState = LOAD_FPGA_NO_OP; // we are done
    	break;

    case LOAD_FPGA_ERROR_CLEANUP:// Tried to start FPGA program Load, and failed
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// An earlier step detected a failure.
        // Clean up I/O line use and release the SPI bus.
    	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    	// Return configuration of the only DSP output line we were using, PROG, to be an input
    	frw_SetFpgaCtrlLines(F_LOAD_PROG_IN);
    	spi_disableAllSpiDevices();
    	frwFlashTaskState = LOAD_FPGA_NO_OP; // for now, just disable task
    	frwLoadFpgasAtStartupStatus = F_LOAD_ERROR;
    	break;

    default:
    	break;
	}

	// Set this task to run again, to take the next step
    taskMgr_setTask(TASKNUM_SpiFlashTask); // Access FLASH via SPI, program FPGA's
}

void frw_MiscFlashTasks(){
	// aside from what is done in frw_SpiFlashTask( ) above, eg programming FPGAs,
	// This task handles anything Flash-related that runs in the background, and
	// not handled immediately as we receive and reply to CAN commands.
	// 1. We launch a Flash programming operation, wait until it is complete,
	//    then set a status variable to allow MCS_File_Recv to continue.
   Uint16 status;
   Uint16 countWordsToWrite;

	switch(miscFlashTaskOperation){
    case MISC_FLASH_TASK_WRITE_FLASH: //--------------------------------------
    	// We are in the midst of an MCS File Receive.
    	// We have data in the flashRWBuff[] and we want to program it into
    	// the flash, and advance the offset address pointers
    	// and we want to wait until the Flash is done before
    	// alerting the MCS File Receive to continue.
    		switch(miscFlashTaskState){
    		case 0:
    	    	// Set Write-Enable ON for Flash Chip
    			frw_RemoveHwWriteProtect(); // raise the ~Write_Protect line
    			spi_SetFlashWriteEnable();  // set internal write enable bit
    			miscFlashTaskState++;
    			break;
    		case 1:
    			// read Flash status, make sure it is ok to write
    			//   check for (~BPX | WEL | ~WIP) --  ~BlockProtected, WriteEnabled, ~WriteInProgress
    			status = spi_ReadSpiFlashStatus();
    			if ((status & (BPX | WEL | WIP)) != WEL) {
    				mcsFileRecvStatus = MCS_FILE_RECV_BKGND_PROBLEM;
    				return;  // exit without re-running task
    			}
    			miscFlashTaskState++;
    			break;
    		case 2:
    			//OK here we write.
    			// We assume that whoever launched this, saw to it
    			// that the address & count are such as to not cross
    			// a page boundary (and cause undesired wrapping)
    			countWordsToWrite = flashRWBuffFillIndex;
    			spi_WriteFlash(&frwFlashAddr.words.lsw, countWordsToWrite, flashRWBuff);
    			miscFlashTaskState++;
    			break;
    		case 3:
    			//Now we read status until write is complete.
    			// read Flash status, make sure it is ok to write
    			//   check for (~BPX | WEL | ~WIP) --  ~BlockProtected, WriteEnabled, ~WriteInProgress
    			status = spi_ReadSpiFlashStatus();
    			if ((status & WIP) == 0) {
    				// Advance the address pointer (bytes) by 2x #words written to Flash
    				frwFlashAddr.all += (flashRWBuffFillIndex<<1);
    				// zero out buffer fill index
    				flashRWBuffFillIndex = 0;
    				// background operation thru, go back to receiving MCS records
    				if (mcsFileRecvStatus == MCS_FILE_RECV_EOF_BKGND) {
    					// wrote final buffer of data to flash, file recv is done
    					mcsFileRecvStatus = MCS_FILE_RECV_IDLE;
    				} else if (frwMcsNotFastDownload == true) {
    					// go back to receiving more
    					mcsFileRecvStatus = MCS_FILE_READY_TO_RECEIVE;
    				} else {
    					// go back to receiving more
    					mcsFileRecvStatus = FAST_FILE_READY_TO_RECEIVE;
    				}
    				frw_AssertHwWriteProtect(); // lower ~WriteProtect line to FLASH Chip
        			miscFlashTaskState++;
    			}
    			// continue, re-running the task until write is through.
    			break;
    		case 4:
    			//Are we doing the double-buffered, overlapped "fast" algorithm?
    			//(as opposed to the MCS algorithm.)
    			//Do we have another buffer of data queued up to write?
    			//If so move it from fastRWBuff[]into the flashRWBuff[] for the writing.
    			//Let the rest of the world know fastRWBuff[] is available to hold more data.

    		    if ((frwMcsNotFastDownload == false)
    		   	&& (fastRWBuffInUse == true)){
    		    	Uint16 *fastRWB = &fastRWBuff[0];
    		    	Uint16 *flashRWB = &flashRWBuff[0];
    		    	Uint16 i;
    		    	for(i=0;i<64;i++){
    		    		*flashRWB++ = *fastRWB++;
    		       }
        		   flashRWBuffFillIndex = (fastRWBuffCharCount + 1) >> 1;
        		   fastRWBuffCharCount = 0;
    		       fastRWBuffInUse = false; //free up that buffer for more transmissions

    		       miscFlashTaskState = 0; // so we restart task to write to flash, next time through
    		    } else {
				   miscFlashTaskOperation = MISC_FLASH_TASK_NO_OP;
				   return;  // exit without re-running task, we're done
    		    }
				break;
    		default:
				return;  // exit without re-running task
    			//break;
    		}

    	break;

    default:                           //--------------------------------------
    	return; // exit without re-running task
    	//break;
    }


	// set our task to run again, if we ever finish a task, we
	// return before we get here
	taskMgr_setTask(TASKNUM_MiscFlashTasks);
}

//===========================================================================
// Fielding CAN Commands
//
//===========================================================================

enum CANOPEN_STATUS frw_readFlashStatusReg(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// Read Status Register of Flash Memory chip and return it as response to a CAN request.
	Uint16 status;

	status = spi_ReadSpiFlashStatus();
	diagRs232readFlashStatusReg(status,1);

	*(data+2) = status; //MboxC
	*(data+3) = 0;      //MboxD

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_readFlashRDID(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// Read RIDI Register of Flash Memory chip and return it as response to a CAN request.
	Uint16 memoryType;
	Uint16 memoryCapacity;

	spi_ReadSpiFlashRDID(&memoryType, &memoryCapacity);
	diagRs232readFlashRDID(memoryType, memoryCapacity);

	*(data+2) = memoryCapacity; //MboxC
	*(data+3) = memoryType;      //MboxD

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_releasePowerdownRES(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// Flash memory command causes chip to come out of deep powerdown mode and
	// report it's electronic signature which is 0x12 for the M25P40
	Uint16 status;

	status = spi_ReleasePowerdownRES();
	diagRs232readFlashStatusReg(status,2);

	*(data+2) = status; //MboxC
	*(data+3) = 0;      //MboxD

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_startMcsFileRecv(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants to start downloading an MCS file for us to receive and save in flash

	// This is our opportunity to clean up from any previous mcs file recv operation
	// (not sure what that is, but this is where we put it when we figure it out)
	flashRWBuffFillIndex = 0;
	frwFlashAddr.words.msw = 0xFFFF; // illegal ms address word, requires PC to send new offset address
	mcsFileRecvAddrExtension = 0; // assume MCS file data starts at 0x0000xxxx

	mcsFileRecvStatus = MCS_FILE_READY_TO_RECEIVE;
	mcsFileRecvError =  MCS_ERR_NO_ERROR;
	mcsFileRecvParseStatus = MCS_PARSE_NO_ERR;

	// We now support 2 bitfile download algorithms
	// 0 -> MCS download, Ascii, non-overlapped
	// 1 -> "fast" algorithm, binary data, overlap download with flash burn
	if (*(data+2) == 0) {
	   frwMcsNotFastDownload = true;
	} else {
       frwMcsNotFastDownload = false;
   	   mcsFileRecvStatus = FAST_FILE_READY_TO_RECEIVE;
   	   fastRWBuffInUse = false;
   	   miscFlashTaskOperation = MISC_FLASH_TASK_NO_OP;
	}

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_mcsFileRecvStatus(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants to start downloading an MCS file for us to receive and save in flash
	// Here we return 2 data words, w/ 4 separate pieces of info:
    //    LS word lsb is # bytes PC can send to our buffer
	//    LS word msb is status from Flash
	//    MS word lsb is MCS file recv operation error identifier
	//    MS word msb is status of MCS file recv operation (see enum MCS_FILE_RECV_STATUS)
	// Note: we also save frwStatus, that we have arrived at here and counsult it at later.

	Uint16 flashStatus;
	Uint16 availableBufferWords;

	// LS byte of status is Flash Status Register
	flashStatus = spi_ReadSpiFlashStatus();

	// flash status should not have write in progress
	// we toggle between MCS_ERR_NO_ERROR and MCS_ERR_WRITE_IN_PROG
	// but don't alter other status.
	if ((flashStatus & WIP) != 0){
		if (mcsFileRecvError == MCS_ERR_NO_ERROR) {
			mcsFileRecvError = MCS_ERR_WRITE_IN_PROG;
		}
	} else {
		// not Write In Progress
		if (mcsFileRecvError == MCS_ERR_WRITE_IN_PROG) {
			mcsFileRecvError = MCS_ERR_NO_ERROR;
		}
	}

	// we should have File Recv in progress
	if ((mcsFileRecvStatus != MCS_FILE_READY_TO_RECEIVE)
	 && (mcsFileRecvStatus != FAST_FILE_READY_TO_RECEIVE)){
		mcsFileRecvError = MCS_ERR_RECV_NOT_IN_PROG;
	}
	// we should have legal address offset
	if (frwFlashAddr.words.msw > 0x7F){
		mcsFileRecvError = MCS_ERR_BAD_ADDRESS;
	}

	//PC will send us more data if we indicate buffer availability to hold it.
	//For MCS algorithm, it's availability in fastRWBuff[].
	//For "fast" algorithm, it's availability in fastRWBuff[], which is an all or nothing: 0/128
    if (frwMcsNotFastDownload == true) {
	   availableBufferWords = (maxWordsInFlashRWBuff - flashRWBuffFillIndex) << 1;
    } else {
    	if (fastRWBuffInUse){
    		availableBufferWords = 0;
    	} else {
    		availableBufferWords = 128;
    	}
    }
	*(data+2) = availableBufferWords
			    | (flashStatus<<8);                            //MboxC
	*(data+3) = ((mcsFileRecvError)
	            | (mcsFileRecvStatus<<8));                     //MboxD

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_bulkEraseFlashSend(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants us to perform bulk erase of Flash.
	// To avoid accidental erasures, this exchange sends 16-bit token to the PC
	// If the PC sends the token back to us in an erasure request, then we erase it.

	// As a token we use a quasi random #, the lsw of 32-bit T0 timer counter
	frw_bulkEraseToken = CpuTimer0Regs.TIM.half.LSW;
	*(data+2) = frw_bulkEraseToken; //MboxC
	*(data+3) = 0;      //MboxD
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_bulkEraseFlashRecv(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants us to perform bulk erase of Flash.
	// To avoid accidental erasures, we give a random 16-bit token to the PC
	// If the PC sends the token back to us in an erasure request, then we erase the Flash.
    Uint16 status;

    //
    if (*(data+2) != frw_bulkEraseToken){
    	return CANOPEN_BAD_FLASH_ERASE_TOKEN;
    }

   	// Set Write-Enable ON for Flash Chip,
    frw_RemoveHwWriteProtect(); // raise the ~Write_Protect line
   	spi_SetFlashWriteEnable();  // set internal write-enable bit inside chip
   	status = spi_ReadSpiFlashStatus();
   	//diagRs232readFlashStatusReg(status,1);  // display Status on diagnostic output
    if (status != 0x02){ // Write-Enabled and not Write-in-Progress
    	return CANOPEN_BAD_FLASH_STATUS;
    }

   	// Bulk Erase Flash Chip
   	spi_BulkEraseFlash();
  	//status = spi_ReadSpiFlashStatus();
   	//diagRs232readFlashStatusReg(status,1); // display Status on diagnostic output

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_mcsFileRecvData(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC is downloading an MCS file for us to receive and save in flash
	// This routine is called after PC completes a multi-segment transfer
	// leaving an MCS format record in the CAN multi-segment buffer
    // Here we check for possible errors, then add the MCS data
	// into FRW's buffer for writing to the Flash.
	// If the data record has filled up our buffer, we initiate the
	// write to Flash.

    // what to check for before taking action?
	//   file receive should be in progress
	//   mcsFileRecvStatus should not identify any errors
	//   available buffer space <= byte count from received MCS record
	//   checksum OK on received MCS record

	struct MULTI_PACKET_BUF *mpb;
	Uint16 countCharIn;
	char *mcsDataIn;
	Uint16 mcsByteCount;
	Uint16 mcsRecType;
	Uint16 mcsAddr;
	Uint16 numWordsFromMcs;
    Uint16 wordsFromMcs[16];
    Uint16 i;


	if ((mcsFileRecvStatus != MCS_FILE_READY_TO_RECEIVE)
     || (mcsFileRecvError != MCS_ERR_NO_ERROR)){
		// then we have had an error getting here and shouldn't go forward
    	return CANOPEN_MCS_FILE_RECV_001_ERR; // error reported in mcsFileRecvStatus
	}

	// received multi-segment data given to us in a MULTI_PACKET_BUF structure
	mpb = (struct MULTI_PACKET_BUF *)(data-2);
    countCharIn = mpb->count_of_bytes_in_buf;
    mcsDataIn = mpb->buff; // this points to the first character of MCS data

    //Parse the MCS Data
    mcsFileRecvParseStatus = mcsParseReceivedData(mcsDataIn,countCharIn,
    		                          &mcsByteCount,&mcsRecType,&mcsAddr,
    		                          &numWordsFromMcs,wordsFromMcs);

    if (mcsFileRecvParseStatus == MCS_PARSE_RECEIVED_SOME_DATA){
       // now make sure we have room in our buffer for the data, and then
       // we move the data into the buffer
    	if ((maxWordsInFlashRWBuff - flashRWBuffFillIndex) >= numWordsFromMcs) {
    		for (i=0;i<numWordsFromMcs;i++){
    			flashRWBuff[flashRWBuffFillIndex++]= wordsFromMcs[i];
    		}
        	if (flashRWBuffFillIndex == maxWordsInFlashRWBuff) {
        		// it is time to write our buffer to the Flash memory
        		// we invoke a background task to do this for us
        		miscFlashTaskOperation = MISC_FLASH_TASK_WRITE_FLASH;
        		miscFlashTaskState = 0;
        		taskMgr_setTask(TASKNUM_MiscFlashTasks);

        	}
    	    return CANOPEN_NO_ERR;
    	} else {
    		// don't have space for it in the flashRWBuff
    		mcsFileRecvStatus = MCS_FILE_STOPPED_FOR_ERROR;
    		mcsFileRecvError = MCS_ERR_RECV_BUFF_OVERRUN;
    		return CANOPEN_MCS_FILE_RECV_001_ERR;
    	}
    } else if (mcsFileRecvParseStatus == MCS_PARSE_RECEIVED_ADDR_EXTEN){
    	// received an address extension record
    	mcsFileRecvAddrExtension = wordsFromMcs[0];
    } else if (mcsFileRecvParseStatus == MCS_PARSE_RECEIVED_EOF){
    	// received the EOF record, need to write remaining buffer to flash

    	if (flashRWBuffFillIndex != 0) {
    		// it is time to write any data in our buffer to the Flash memory
    		// we invoke a background task to do this for us
    		mcsFileRecvStatus = MCS_FILE_RECV_EOF_BKGND;
    		miscFlashTaskOperation = MISC_FLASH_TASK_WRITE_FLASH;
    		miscFlashTaskState = 0;
    		taskMgr_setTask(TASKNUM_MiscFlashTasks);
    	} else {
    		mcsFileRecvStatus = MCS_FILE_RECV_IDLE;  // operation complete
    	}

    } else {
    	// we encountered an error, something we weren't prepared to parse
    	mcsFileRecvError = MCS_ERR_MCS_PARSE_ERR;
		mcsFileRecvStatus = MCS_FILE_STOPPED_FOR_ERROR;
		return CANOPEN_MCS_FILE_RECV_001_ERR;
    }


    return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_mcsFileSendData(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
    // PC wants us to send one MCS-format record with data from address frwFlashAddr.
	// Max 16 bytes per record
	// Assume file starts on even 0x00000010 address boundary in flash
	// In advance, pc sends total data byte count.
	// final data rec my be less than 16 bytes.
	// if PC fetches a data rec after we have exceeded total data byte count,
	//  then we return EOF record.

	//struct MULTI_PACKET_BUF *mpb;

    Uint16 wordsFromFlash[8];
    Uint16 i;
    Uint16 dataBytesInThisRecord;
    char mcsRec[45];
    char *c;
    Uint16 checksumAccumulator;
    Uint16 countCopied;

    // - - - - Called at the start of a multi-packet SEND operation - - - -

    // Enforce the convention that MCS data files are read from addresses
    // starting on even 0x00000010 address
    frwFlashAddr.words.lsw &= 0xFFF0;

    // If PC is asking to receive data past the amount that it originally
    // asked for, then send an EOF record
    if (mcsFileSendByteCount.all == 0) {
    	// ":00000001FF"  0x3A30 0x3030 0x3030 0x3030 0x3146, 0x46
    	c = strU_strcpy(mcsRec,":00000001FF");
    } else {

    	// Figure out how many data bytes to send (1 to 16)
    	if (mcsFileSendByteCount.all > 15){
    		dataBytesInThisRecord = 16;
    	} else {
    		dataBytesInThisRecord = mcsFileSendByteCount.words.lsw;
    	}

    	// Read the data from Flash and advance our flash address pointer
    	spi_ReadFlash(&(frwFlashAddr.words.lsw) , ((dataBytesInThisRecord+1)>>1),wordsFromFlash);
    	frwFlashAddr.all += dataBytesInThisRecord;

    	// assemble our MCS record
    	mcsRec[0] = ':';
    	hexUtil_binTo2HexAsciiChars((mcsRec+1),dataBytesInThisRecord);
    	hexUtil_binTo4HexAsciiChars((mcsRec+3),mcsFileSendByteAddr);
    	mcsRec[7] = '0';
    	mcsRec[8] = '0';
    	checksumAccumulator = dataBytesInThisRecord;
    	checksumAccumulator += (mcsFileSendByteAddr & 0xFF);
    	checksumAccumulator += ((mcsFileSendByteAddr>>8) & 0xFF);
    	c = (mcsRec+9);
    	for (i=0;i<dataBytesInThisRecord;i++){
    		if ((i & 1) == 0) {
    			// even byte
        		c = hexUtil_binTo2HexAsciiChars(c,wordsFromFlash[i>>1]>>8);
        		checksumAccumulator += ((wordsFromFlash[i>>1]>>8) & 0xFF);
    		} else {
    			// odd byte
        		c = hexUtil_binTo2HexAsciiChars(c,wordsFromFlash[i>>1]);
        		checksumAccumulator += (wordsFromFlash[i>>1] & 0xFF);
    		}
       	}
    	checksumAccumulator = 0x100 - checksumAccumulator;
    	c = hexUtil_binTo2HexAsciiChars(c,checksumAccumulator);
    }

    countCopied = copyDataToMultiPacketBuf(mcsRec, (Uint16)(c -  mcsRec));

    // Advance mcsFileSendByteAddr for next time,
    // decrement remaining byte count left to send
    mcsFileSendByteAddr += dataBytesInThisRecord;
    mcsFileSendByteCount.all -= dataBytesInThisRecord;

    if (countCopied == 0) {
    	return CANOPEN_MCS_SEND_001_ERR; // we can't copy to MultiPacketBuf, maybe it is is in use
    }

    return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_fastFileRecvData(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC is downloading data for us to receive and save in flash
	// This routine is called after PC completes a multi-segment transfer
	// leaving a 128 byte block of data in the CAN multi-segment buffer.
    // Here we store the received data in the fastRWBuff buffer, part of
	// a double-buffering scheme, allowing overlapped transmission and burning.
	// If the flashRWBuff is available, we copy it into that buffer,
	// from which we can write to the Flash, and we initiate the
	// write to Flash.

    // what to check for before taking action?
	//   file receive should be in progress
	//   mcsFileRecvStatus should not identify any errors
	//   available buffer space <= byte count from received MCS record
	//   checksum OK on received MCS record

	struct MULTI_PACKET_BUF *mpb;
	Uint16 countCharIn;
	char *fastDataIn;
    Uint16 i;
    Uint16 *fastBuf;


	if (mcsFileRecvStatus != FAST_FILE_READY_TO_RECEIVE){
		// then we have had an error getting here and shouldn't go forward
    	return CANOPEN_MCS_FILE_RECV_001_ERR; // error reported in mcsFileRecvStatus
	}

	// received multi-segment data given to us in a MULTI_PACKET_BUF structure
	mpb = (struct MULTI_PACKET_BUF *)(data-2);
    countCharIn = mpb->count_of_bytes_in_buf;
    fastDataIn = mpb->buff; // this points to the first character of MCS data

    //copy data from MULTI_PACKET_BUF into fastRWBuff
    //packing as we go from chars to 16-bit words.
    if (countCharIn > 0){
       fastBuf = &fastRWBuff[0];
 	   for (i=0;i<countCharIn;i++) {
 		   // pack 2 bytes into each word
 		   if ((i & 1) == 0) {
 			   *fastBuf = ((*fastDataIn++)<<8) & 0xFF00;
 		   } else {
 			   *(fastBuf++) |= (*fastDataIn++);
 		   }
 	   }
    }
    fastRWBuffCharCount = countCharIn;

    // Suspend transmissions, until we again free up fastRWBuff
    fastRWBuffInUse = true;

    //If MiscFlashTasks is idle, start it up
    if (miscFlashTaskOperation == MISC_FLASH_TASK_NO_OP){
    	miscFlashTaskOperation = MISC_FLASH_TASK_WRITE_FLASH;
    	miscFlashTaskState = 4;
    	taskMgr_setTask(TASKNUM_MiscFlashTasks);
    }

    return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_startLoadFpgaFromFlash(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
    // PC asks us to load FPGA program from FLASH memory
	// We kick off a background task to do that.
	// We set up any necessary state variables before kicking it off.

	frwLoadMultipleFgpas = false; // load only 1 FPGA, as selected by frwWhichFlashChip
	frwFlashTaskState = LOAD_FPGA_ON_CAN_REQUEST;
	taskMgr_setTask(TASKNUM_SpiFlashTask); // Access FLASH via SPI, program FPGA's

    return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_turnOnFlashWriteProtect(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
    // PC asks us to write protect the FLASH chips by raising ~WriteProtect lines

	frw_AssertHwWriteProtect();
    return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_readWhichFlashChip(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
    // PC asks us which of our 2 (IO Board) FLASH chips is selected as target
	// for subsequent activity -- 0=FLASH_1, 1=FLASH_2

	*(data+2) = frw_GetWhichFlash(); //MboxC
	*(data+3) = 0;                   //MboxD

    return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_writeWhichFlashChip(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
    // PC tells us which of our 2 (IO Board) FLASH chips (+ 1 Pwr Board) is selected as target
	// for subsequent activity -- 1=FLASH_1, 2=FLASH_2, 3=FLASH_3

	frw_SetWhichFlash(*(data+2));    // MboxC

    return CANOPEN_NO_ERR;
}


//===========================================================================
// Misc Diagnostic routines
//
//===========================================================================

void diagRs232readFlashStatusReg(Uint16 status,Uint16 functionCode){
// Log to RS232/USB for diagnostic purposes only
// Simple text display
	char msgOut[64];
    char *ptr;

    ptr = strU_strcpy(msgOut,"SPI Flash Diagnostics - ");

    if (functionCode == 1) {
    	ptr = strU_strcpy(ptr," status  0x");
    } else if(functionCode == 2) {
    	ptr = strU_strcpy(ptr," RDID    0x");
    } else {
    	ptr = strU_strcpy(ptr," unknown 0x");
    }

    ptr = hexUtil_binTo4HexAsciiChars(ptr,status);
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}

void diagRs232readFlashRDID(Uint16 status,Uint16 status_2){
// Log to RS232/USB for diagnostic purposes only
// Simple text display
	char msgOut[64];
    char *ptr;

    ptr = strU_strcpy(msgOut,"SPI Flash - RDID  0x");

    ptr = hexUtil_binTo4HexAsciiChars(ptr,status);
    ptr = strU_strcpy(ptr," ");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,status_2);
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}

void diagRs232FlashRWBuff(Uint16 countWords){
// Log to RS232/USB for diagnostic purposes only
// Display # Words from flashRWBuff
// Called only from frw_test4003( ), resulting from commint command C4003:ddddCr
	char msgOut[64];
    char *ptr;
    Uint16 i;
    Uint16* flashRWBuffPtr = flashRWBuff;

    ptr = strU_strcpy(msgOut,"SPI Flash RW Buff - ");
    ptr = strU_strcpy(ptr,"\n\r");
	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

    while (countWords > 0) {
        // display words 8 at a time
    	ptr = msgOut;
        for (i=0;i<8;i++){
        	if (countWords > 0){
        		// display Words as hex
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,*(flashRWBuffPtr++));
        		ptr = strU_strcpy(ptr," ");
                // display Chars as Ascii
        		countWords--;
        	}
        }
        ptr = strU_strcpy(ptr,"\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    }

}

//===========================================================================
// Routines run from Command Interpreter in Comint.c
//
//===========================================================================
void frw_test2005(Uint16 dataWord){
// Various specific actions on CPLD / FPGA
//    0 = Turn off CPLD_LED_0, (on in TB3IOMC)
//    1 = Turn on CPLD_LED_0,  (off in TB3IOMC)
//    2 = Turn off CPLD_LED_1, (on in TB3IOMC)
//    3 = Turn on CPLD_LED_1,  (off in TB3IOMC)
//    4 = Turn off F_PROG_2,
//    5 = Turn on F_PROG_2,
//    6 = Turn ~FPGA1_RESET LOW,
//    7 = Turn ~FPGA1_RESET HI,
//    8 = Turn ~FPGA2_RESET LOW,
//    9 = Turn ~FPGA2_RESET HI,
//    A = Turn off FPGA2_LED_0,
//    B = Turn on FPGA2_LED_0,
//    C = Turn on FPGA1_LED_0, & 1 => 00
//    D = Turn on FPGA1_LED_0, & 1 => 01
//    E = Turn on FPGA1_LED_0, & 1 => 10
//    F = Turn on FPGA1_LED_0, & 1 => 11
//   10 = Turn on testpoint_0,
//   11 = Turn off testpoint_0,
//   12 = Turn on testpoint_1,
//   13 = Turn off testpoint_1,
//   14 = Turn on testpoint_2,
//   15 = Turn off testpoint_2,
//   16 = Turn on testpoint_3,
//   17 = Turn off testpoint_3,

	volatile Uint16 *extData; // pointer for ernal bus address to access CPLD
	volatile Uint16 data_in;

	switch(dataWord){
    case 0x0000:
    	// 0 = Turn off CPLD_LED_0, (on in TB3IOMC)
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_0);
    	data_in = *extData;
    	break;
    case 0x0001:
    	// 1 = Turn on CPLD_LED_0, (off in TB3IOMC)
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_0);
    	*extData = 0; // data value not important
    	break;
    case 0x0002:
    	// 2 = Turn off CPLD_LED_1, (on in TB3IOMC)
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1);
    	data_in = *extData;
    	break;
    case 0x0003:
    	// 3 = Turn off CPLD_LED_1, (off in TB3IOMC)
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1);
    	*extData = 0; // data value not important
    	break;
    case 0x0004:
    	// 4 = Turn off F_PROG_2
    	extData = CPLD_XINTF_ADDR(TBIOM_F_PRGM_2);
    	data_in = *extData;
    	break;
    case 0x0005:
    	// 5 = Turn off F_PROG_2
    	extData = CPLD_XINTF_ADDR(TBIOM_F_PRGM_2);
    	*extData = 0; // data value not important
    	break;
    case 0x0006:
    	// 6 = Turn ~FPGA1_RESET LOW
    	extData = CPLD_XINTF_ADDR(TBIOM_FPGA1_RESET);
    	data_in = *extData;
    	break;
    case 0x0007:
    	// 7 = Turn ~FPGA1_RESET HI
    	extData = CPLD_XINTF_ADDR(TBIOM_FPGA1_RESET);
    	*extData = 0; // data value not important
    	break;
    case 0x0008:
    	// 8 = Turn ~FPGA2_RESET LOW
    	extData = CPLD_XINTF_ADDR(TBIOM_FPGA2_RESET);
    	data_in = *extData;
    	break;
    case 0x0009:
    	// 9 = Turn ~FPGA2_RESET HI
    	extData = CPLD_XINTF_ADDR(TBIOM_FPGA2_RESET);
    	*extData = 0; // data value not important
    	break;
    case 0x000A:
    	// A = Turn off FPGA2_LED_0
    	extData = CPLD_F2_XA(FPGA1_WRITE_LED_FUNCTION); // same offset both FPGA 1 & 2
    	*extData = 0; // parameter [1:0]LED_DIRECTLY_FROM_DSP = 2'b00;
    	extData = CPLD_F2_XA(FPGA1_WRITE_LED_DIRECTLY); // same offset both FPGA 1 & 2
    	*extData = 0; // OFF
    	break;
    case 0x000B:
    	// B = Turn on FPGA2_LED_0
    	extData = CPLD_F2_XA(FPGA1_WRITE_LED_FUNCTION); // same offset both FPGA 1 & 2
    	*extData = 0; // parameter [1:0]LED_DIRECTLY_FROM_DSP = 2'b00;
    	extData = CPLD_F2_XA(FPGA1_WRITE_LED_DIRECTLY); // same offset both FPGA 1 & 2
    	*extData = 1; // ON
    	break;
    case 0x000C:
    	// C = Turn on FPGA1_LED_0, & 1 => 00
    	extData = CPLD_F1_XA(FPGA1_WRITE_LED_FUNCTION); // same offset both FPGA 1 & 2
    	*extData = 0; // parameter [1:0]LED_DIRECTLY_FROM_DSP = 2'b00;
    	extData = CPLD_F1_XA(FPGA1_WRITE_LED_DIRECTLY); // same offset both FPGA 1 & 2
    	*extData = 0;
    	break;
    case 0x000D:
    	// D = Turn on FPGA1_LED_0, & 1 => 01
    	extData = CPLD_F1_XA(FPGA1_WRITE_LED_FUNCTION); // same offset both FPGA 1 & 2
    	*extData = 0; // parameter [1:0]LED_DIRECTLY_FROM_DSP = 2'b00;
    	extData = CPLD_F1_XA(FPGA1_WRITE_LED_DIRECTLY); // same offset both FPGA 1 & 2
    	*extData = 1;
    	break;
    case 0x000E:
    	// E = Turn on FPGA1_LED_0, & 1 => 10
    	extData = CPLD_F1_XA(FPGA1_WRITE_LED_FUNCTION); // same offset both FPGA 1 & 2
    	*extData = 0; // parameter [1:0]LED_DIRECTLY_FROM_DSP = 2'b00;
    	extData = CPLD_F1_XA(FPGA1_WRITE_LED_DIRECTLY); // same offset both FPGA 1 & 2
    	*extData = 2;
    	break;
    case 0x000F:
    	// F = Turn on FPGA1_LED_0, & 1 => 11
    	extData = CPLD_F1_XA(FPGA1_WRITE_LED_FUNCTION); // same offset both FPGA 1 & 2
    	*extData = 0; // parameter [1:0]LED_DIRECTLY_FROM_DSP = 2'b00;
    	extData = CPLD_F1_XA(FPGA1_WRITE_LED_DIRECTLY); // same offset both FPGA 1 & 2
    	*extData = 3;
    	break;
    case 0x0010:
    	// 0x10 = Set Testpoint_0 HI
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_0);
    	*extData = 0; // data value not important
    	break;
    case 0x0011:
    	// 0x11 = Set Testpoint_0 LOW
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_0);
    	data_in = *extData;
    	break;
    case 0x0012:
    	// 0x10 = Set Testpoint_1 HI
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_1);
    	*extData = 0; // data value not important
    	break;
    case 0x0013:
    	// 0x11 = Set Testpoint_1 LOW
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_1);
    	data_in = *extData;
    	break;
    case 0x0014:
    	// 0x10 = Set Testpoint_2 HI
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_2);
    	*extData = 0; // data value not important
    	break;
    case 0x0015:
    	// 0x11 = Set Testpoint_2 LOW
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_2);
    	data_in = *extData;
    	break;
    case 0x0016:
    	// 0x10 = Set Testpoint_3 HI
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_3);
    	*extData = 0; // data value not important
    	break;
    case 0x0017:
    	// 0x11 = Set Testpoint_3 LOW
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_3);
    	data_in = *extData;
    	break;

    default:
    	break;
    }

}

void frw_test4000(Uint16 dataWord){

	Uint16 status;
	Uint16 status_2;
	Uint16 i;

	switch(dataWord){
    case 0x0000:
    	// Disable All SPI Devices on SPI Bus
    	spi_disableAllSpiDevices();
    	break;
    case 0x0001:
    	// Select Flash chip on SPI bus
    	diag_SelectSpiFlash();
    	break;
    case 0x0002:
    	// Read Status of Flash Chip, display on diagnostic output
    	status = spi_ReadSpiFlashStatus();
    	diagRs232readFlashStatusReg(status,1);
    	break;
    case 0x0003:
    	// Read Flash Chip RDID, display on diagnostic output
    	spi_ReadSpiFlashRDID(&status, &status_2);
    	diagRs232readFlashRDID(status,status_2);
    	break;
    case 0x0004:
    	// Set Write-Enable ON for Flash Chip, display Status on diagnostic output
    	// Also raise the ~Write_Protect line to whichever Flash chip is currently selected
    	frw_RemoveHwWriteProtect(); // raise the ~Write_Protect line
    	spi_SetFlashWriteEnable();  // set chip internal write enable bit
    	status = spi_ReadSpiFlashStatus();
    	diagRs232readFlashStatusReg(status,1);
    	break;
    case 0x0005:
    	// Bulk Erase Flash Chip, display Status on diagnostic output
    	spi_BulkEraseFlash();
    	status = spi_ReadSpiFlashStatus();
    	diagRs232readFlashStatusReg(status,1);
    	break;
    case 0x0006:
    	// Init frwFlashAddr16 and frwWRBuf for testing;
    	frwFlashAddr.words.lsw = 0x0002;
    	frwFlashAddr.words.msw = 0x0000;
    	flashRWBuff[0] = 0x0101;
    	for (i=0;i<maxWordsInFlashRWBuff;i++){
    		flashRWBuff[i] = flashRWBuff[i-1] + 0x0101;
    	}
    	break;
    case 0x0007:
    	// Tell CPLD to assert ~WriteProtect lines to both FLASH chips.
    	frw_AssertHwWriteProtect();
    	break;

    default:
    	break;
    }
}

void frw_test4001(Uint16 dataWord){
//Set HI  word of Address for Flash Read / write
	frwFlashAddr.words.msw = dataWord; // parameter telling where to read or write in FLASH
	                              // {LS word, MS word}
}
void frw_test4002(Uint16 dataWord){
//Set LOW word of Address for Flash Read / write
	frwFlashAddr.words.lsw = dataWord; // parameter telling where to read or write in FLASH
	                              // {LS word, MS word}
}

void frw_test4003(Uint16 bytesToRead){
//Read # bytes from Flash, display on RS232
	Uint16 countWordsToRead;
	Uint16 status;

	//Read Status, check for ( ~WIP) --  ~WriteInProgress
	status = spi_ReadSpiFlashStatus();
	diagRs232readFlashStatusReg(status,1);
	if ((status & WIP) != 0) {
		return;
	}

	//convert bytes to words, don't overrun our buffer
	countWordsToRead = ((bytesToRead + 1) >> 1) & 0x7FFF;
	if (countWordsToRead > maxWordsInFlashRWBuff) {
		countWordsToRead = maxWordsInFlashRWBuff;
	}
	spi_ReadFlash(&frwFlashAddr.words.lsw, countWordsToRead, flashRWBuff);
	diagRs232FlashRWBuff(countWordsToRead);

//NOTE: we don't yet have anything in here to deal with byte-allignment
//      and I don't know whether or not we need something.
//      eg.  we allow a write of word-alligned data from Uint16[]flashRWBuff
//      even when frwFlashAddr16 is an odd # (eg not word-alligned).
//		Think about it.
}

void frw_test4004(Uint16 bytesToWrite){
//Write # bytes to Flash
	Uint16 countWordsToWrite;
	Uint16 status;

	//Read Status, check for (~BPX | WEL | ~WIP) --  ~BlockProtected, WriteEnabled, ~WriteInProgress
	status = spi_ReadSpiFlashStatus();
	diagRs232readFlashStatusReg(status,1);
	if ((status & (BPX | WEL | WIP)) != WEL) {
		return;
	}

	//convert bytes to words, don't overrun our buffer
	countWordsToWrite = ((bytesToWrite + 1) >> 1) & 0x7FFF;
	if (countWordsToWrite > maxWordsInFlashRWBuff) {
		countWordsToWrite = maxWordsInFlashRWBuff;
	}

	// writes are limited to one 256-Byte Page (128 words), else they wrap
	if (((countWordsToWrite >> 1) + (frwFlashAddr.words.lsw & 0x00FF)) > 0x00FF){
		countWordsToWrite = (0x100 - frwFlashAddr.words.lsw) >> 1;
	}
	spi_WriteFlash(&frwFlashAddr.words.lsw, countWordsToWrite, flashRWBuff);

}
void frw_test4005(Uint16 seedValue){
//Initialize the flashRWBuff for testing
	Uint16 i;
	for (i=0;i<maxWordsInFlashRWBuff;i++){
		flashRWBuff[i] = seedValue;
		seedValue += 0x101;
	}
}
void frw_test4006(Uint16 whichFlashChip){
//Store value in Globals, telling which of 3 FLASH chips is target of subsequent operations
// -- Select which FLASH: 1=FLASH_1, 2=FLASH_2, 3=FLASH_2
	frw_SetWhichFlash(whichFlashChip);
}
//===========================================================================
// Diagnostic routine task
//
//===========================================================================

void frw_diagFlashTasks(){
	// PC wants us to read from Flash memory at a given address,
	// and display via RS232.
	// Note, the flash address is in bytes, but we read words (2 bytes)
	// at a time from the Flash.

	Uint16 countWordsToRead;
	Uint16 wordsFromFlash[8];
	char msgOut[64];
    char *ptr;
    Uint16 i;
    Uint16 addrInPage;

	countWordsToRead = ((frwDiagFlashAddr.words.lsw & 0x000F) + 1)>>1;
	if (countWordsToRead == 0){
		countWordsToRead = 8;
	}
	spi_ReadFlash(&(frwDiagFlashAddr.words.lsw), countWordsToRead, wordsFromFlash);

	ptr = strU_strcpy(msgOut,"@ 0x");
	ptr = hexUtil_binTo4HexAsciiChars(ptr,frwDiagFlashAddr.words.msw);
	ptr = hexUtil_binTo4HexAsciiChars(ptr,frwDiagFlashAddr.words.lsw);
    ptr = strU_strcpy(ptr,":");
    for (i=0;i<countWordsToRead;i++){
        ptr = strU_strcpy(ptr," ");
    	ptr = hexUtil_binTo4HexAsciiChars(ptr,wordsFromFlash[i]);
    }
    ptr = strU_strcpy(ptr,"\n\r");
    if (r232Out_outChars(msgOut, (Uint16)(ptr - msgOut))){
    	// then success writing out rs232
    	// Advance our read address and see if we have reached the end of the page
    	frwDiagFlashAddr.words.lsw &= 0xFFF0;
		frwDiagFlashAddr.all += 0x0010;
		addrInPage = frwDiagFlashAddr.words.lsw & 0x00FF;
		if (addrInPage == 0){
			// then we got to the end of the page and we are done
			return; // exit w/out re-starting the task
		}
    }
    // either RS232 Out Buffer full, we have to try again later
    // or we have more to display, next time around

	taskMgr_setTask(TASKNUM_DiagFlashTasks);
}
//===========================================================================
// Diagnostic routines from CAN
//
//===========================================================================

enum CANOPEN_STATUS frw_diagDisplFlashPage(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants us to display a page of Flash memory at a given address.
	char msgOut[64];
    char *ptr;

	ptr = strU_strcpy(msgOut,"\n\rfrw_diagDisplFlashPage( )\n\r");
	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

	frwDiagFlashAddr.words.lsw = *(data+2); // MboxC
	frwDiagFlashAddr.words.msw = *(data+3); // MboxD
	taskMgr_setTask(TASKNUM_DiagFlashTasks);


	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS frw_runTest2005(const struct CAN_COMMAND* can_command, Uint16* data){
	// CAN command to run frw_test2005() which was originally run from comint.
	// *(data+2)) is MboxC -- 16-bit value selects test from frw_test2005()
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	frw_test2005(*(data+2)); // MboxC -- 16-bit value selects test from frw_test2005()

	return CANOPEN_NO_ERR;
}


