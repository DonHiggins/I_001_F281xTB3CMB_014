// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Xintf.C
//
//   Initialization function for the external interface (XINTF).
//
// Adapted from DSP281x_Xintf.c in
//    C:\TI_SPRCO97_C281x_Header_n_Peripherals_Ex\tidcs\c28\DSP281x\v120\DSP281x_common\source
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//          Example initialization function for the external interface (XINTF).
//          This example configures the XINTF to its default state.  For an
//          example of how this function can be modified to configure the XINTF
//          for use with the F2812 eZdsp, refer to the examples/run_from_xintf
//          project.
//
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "GpioUtil.H"
#include "CPLD.H"
#include "Rs232Out.H"
#include "HexUtil.H"
#include "StrUtil.H"
#include "Timer0.H"
#include "LED.H"

//---------------------------------------------------------------------------
// InitXINTF:
//---------------------------------------------------------------------------
// This function initializes the External Interface to the default reset state.
//
// Do not modify the timings of the XINTF while running from the XINTF.  Doing
// so can yield unpredictable results


void xintf_InitXintf(void)
{

#if DSP28_F2812

    // This shows how to write to the XINTF registers.  The
    // values used here are the default state after reset.
    // Different hardware will require a different configuration.

    // For an example of an XINTF configuration used with the
    // F2812 eZdsp, refer to the examples/run_from_xintf project.

    // Any changes to XINTF timing should only be made by code
    // running outside of the XINTF.

    // All Zones---------------------------------
    // Timing for all zones based on XTIMCLK = 1/2 SYSCLKOUT
    XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
    // No write buffering
    XintfRegs.XINTCNF2.bit.WRBUFF = 0;
    // XCLKOUT is enabled
    XintfRegs.XINTCNF2.bit.CLKOFF = 0;
    // XCLKOUT = XTIMCLK/2
    XintfRegs.XINTCNF2.bit.CLKMODE = 1;


    // Zone 0------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING0.bit.XWRLEAD = 3;
    XintfRegs.XTIMING0.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING0.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING0.bit.XRDLEAD = 3;
    XintfRegs.XTIMING0.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING0.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING0.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING0.bit.USEREADY = 1;
    XintfRegs.XTIMING0.bit.READYMODE = 1;  // sample asynchronous

    // Size must be 1,1 - other values are reserved
    XintfRegs.XTIMING0.bit.XSIZE = 3;

    // Zone 1------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING1.bit.XWRLEAD = 3;
    XintfRegs.XTIMING1.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING1.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING1.bit.XRDLEAD = 3;
    XintfRegs.XTIMING1.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING1.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING1.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING1.bit.USEREADY = 1;
    XintfRegs.XTIMING1.bit.READYMODE = 1;  // sample asynchronous

    // Size must be 1,1 - other values are reserved
    XintfRegs.XTIMING1.bit.XSIZE = 3;

    // Zone 2------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING2.bit.XWRLEAD = 3;
    XintfRegs.XTIMING2.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING2.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING2.bit.XRDLEAD = 3;
    XintfRegs.XTIMING2.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING2.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    //XintfRegs.XTIMING2.bit.X2TIMING = 1; //Default doubles timing
    XintfRegs.XTIMING2.bit.X2TIMING = 0;   //for now, don't double timing

    // Zone will sample XREADY signal
    // XintfRegs.XTIMING2.bit.USEREADY = 1; //Default
    XintfRegs.XTIMING2.bit.USEREADY = 0;    // TB3CMA doesn't use XREADY -- DH
    XintfRegs.XTIMING2.bit.READYMODE = 1;   // sample asynchronous

    // Size must be 1,1 - other values are reserved
    XintfRegs.XTIMING2.bit.XSIZE = 3;


    // Zone 6------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING6.bit.XWRLEAD = 3;
    XintfRegs.XTIMING6.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING6.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING6.bit.XRDLEAD = 3;
    XintfRegs.XTIMING6.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING6.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING6.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING6.bit.USEREADY = 1;
    XintfRegs.XTIMING6.bit.READYMODE = 1;  // sample asynchronous

    // Size must be 1,1 - other values are reserved
    XintfRegs.XTIMING6.bit.XSIZE = 3;


    // Zone 7------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING7.bit.XWRLEAD = 3;
    XintfRegs.XTIMING7.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING7.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING7.bit.XRDLEAD = 3;
    XintfRegs.XTIMING7.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING7.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING7.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING7.bit.USEREADY = 1;
    XintfRegs.XTIMING7.bit.READYMODE = 1;  // sample asynchronous

    // Size must be 1,1 - other values are reserved
    XintfRegs.XTIMING7.bit.XSIZE = 3;

    // Bank switching
    // Assume Zone 7 is slow, so add additional BCYC cycles
    // when ever switching from Zone 7 to another Zone.
    // This will help avoid bus contention.
    XintfRegs.XBANK.bit.BANK = 7;
    XintfRegs.XBANK.bit.BCYC = 7;

   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.

   asm(" RPT #7 || NOP");

    #endif
}

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Xintf non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
Uint16 xintf_testData2001;
Uint16 xintf_testData2002;
Uint16 xintf_testData2003;
Uint16 xintf_testSelection2004;
Uint16 xintf_testUnderTimer0_count;
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for Xintf non-public variables
void xintf_setTestData2001(Uint16 dataWord){
	xintf_testData2001 = dataWord;
}
void xintf_setTestData2002(Uint16 dataWord){
	xintf_testData2002 = dataWord;
}
void xintf_setTestData2003(Uint16 dataWord){
	xintf_testData2003 = dataWord;
}
void xintf_setTestSelection2004(Uint16 dataWord){
	xintf_testSelection2004 = dataWord;
	if (dataWord != 0){
		// avoid contention with another routine using the TB3IOM CPLD LEDs
		led_setLedCpldIoPattern(LED_CPLD_PATTERN_DISABLED);
	}
}
Uint16 xintf_getTestSelection2004(void){
	return xintf_testSelection2004;
}
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

void xintf_initTest(void){
	xintf_testSelection2004 = 0x000A; // turn LEDs OFF, then turn testing off
	xintf_testData2001 =0x0008;  // addr Hi
	xintf_testData2002 =0x1234;  // addr low
	xintf_testData2003 = 0x4321; // write 0x4321 to addr 0x08,1234

	xintf_testUnderTimer0_count = 0;
}


void xintf_testUnderTimer0(void){
	// Called periodically from Timer0 task
	// Perform something to exercise the external bus based
	// on state value in xintf_testSelection2004

	volatile Uint16 *extData;
	volatile Uint16 *extData_2;
	Uint32 tempData32;
	Uint16 tempData16;
	Uint16 dataRead;

	xintf_testUnderTimer0_count++;

	switch(xintf_testSelection2004){
    case 0x0000: // test turned off
    	return;
        // break; // commented out because it yields a compiler warning as "unreachable"

    case 0x0001: // write a specific data value @ a specific address value
    	tempData32 = xintf_testData2001;
    	tempData32 = (tempData32 << 16) + xintf_testData2002;
    	extData = (Uint16 *)tempData32;
    	*extData = xintf_testData2003;
        break;

    case 0x0002: // Test all 19 bits of the external Address Bus
    	         // write a specific data value @ a specific address value
    	         // then read from same address with lower 19 bits inverted
    	         // Trigger the scope on ~I/O_WE or I/O_R_W w/ analog probes
    	         // Use digital probes to look at address lines I/O_AB0 to 19.
    	         // Should see each address line assume correct value at start
    	         // of the write, then toggle at the start of Read.
    	tempData32 = xintf_testData2001;
    	tempData32 = (tempData32 << 16) + xintf_testData2002;
    	extData = (Uint16 *)tempData32;
    	*extData = xintf_testData2003;

    	tempData32 = (xintf_testData2001 ^ 0x0007);
    	tempData32 = (tempData32 << 16) + (xintf_testData2002 ^ 0xFFFF);
    	extData = (Uint16 *)tempData32;
    	dataRead = *extData;
        break;

    case 0x0003: // Test the 16-bit external Data Bus
    	         // Write data to an external address
    	         // then invert the data and write it again
                 // Should see each data line assume correct value at start
                 // of first write, then toggle at the start of 2nd write.
    	tempData32 = xintf_testData2001;
    	tempData32 = (tempData32 << 16) + xintf_testData2002;
    	extData = (Uint16 *)tempData32;
    	*extData = xintf_testData2003;

    	tempData16 = xintf_testData2003 ^ 0xFFFF;
    	*extData = tempData16;
        break;

    case 0x0004: // Turn on / off CPLD_LED_1 (latched J/K flip flop output)
    	         // Read or write data to address 0x08,09xx
    	         // for CPLD_LED, doesn't matter what data we read or write
    	         // just the fact that we are reading or writing
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1);
    	extData_2 = (Uint16 *)0x080000;
        if (xintf_testUnderTimer0_count & 1){
        	*extData_2 = xintf_testData2003; // access out of target area
        	*extData = xintf_testData2003;
        	*extData = xintf_testData2003; // see if 3 x helps (or works)
        	*extData = xintf_testData2003;
        	*extData_2 = xintf_testData2003; // access out of target area
        } else {
        	tempData16 = *extData_2; // access out of target area
        	tempData16 = *extData;
        	tempData16 = *extData;
        	tempData16 = *extData;
        	tempData16 = *extData_2; // access out of target area
        }
        break;

    case 0x0005: // Turn on / off CPLD_LED_1 (latched J/K flip flop output)
    	         // (Used to read or write data to alternate address 0x00,2Axx)
                 // Discarded the dual-bus, dual addr-map with TB3CMA  & TB3IOMA
    			 // so now case 0x0005, reads or writes the same address as case 0x0004
    	         // for CPLD_LED, doesn't matter what data we read or write
    	         // just the fact that we are reading or writing
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1);
        if (xintf_testUnderTimer0_count & 1){
        	*extData = xintf_testData2003;
        } else {
        	tempData16 = *extData;
        }
        break;

    case 0x0006: // Write to all 4 test points (non-latched async multiplexor outputs)
    	         // Write to addresses 0x080C
    	         // 0x08,0C00 , 0x08,0D00, 0x08,0E00, 0x08,0F00
    	         // just the fact that we are reading or writing
        extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_0);
    	*extData = xintf_testData2003;

    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_1);
    	*extData = xintf_testData2003;

    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_2);
    	*extData = xintf_testData2003;

    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_TESTPOINT_3);
    	*extData = xintf_testData2003;

        break;

    case 0x0007: // read data value from a specific address value
    	tempData32 = xintf_testData2001;
    	tempData32 = (tempData32 << 16) + xintf_testData2002;
    	extData = (Uint16 *)tempData32;
    	dataRead = *extData;
        break;

    case 0x0008: // Display count in CPLD LED's
    	// LED's 0, & 1

    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_0); // CPLD_LED_0
        if (xintf_testUnderTimer0_count & 1){
        	*extData = xintf_testData2003; //data not important, write turns LED ON
        } else {
        	tempData16 = *extData;         // data not important, read turns LED OFF
        }

    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1); // CPLD_LED_1
        if (xintf_testUnderTimer0_count & 2){
        	*extData = xintf_testData2003; //data not important, write turns LED ON
        } else {
        	tempData16 = *extData;         // data not important, read turns LED OFF
        }

    	//extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_2); // CPLD_LED_2
        //if (xintf_testUnderTimer0_count & 4){
        //	*extData = xintf_testData2003; //data not important, write turns LED ON
        //} else {
        //	tempData16 = *extData;         // data not important, read turns LED OFF
        //}

    	//extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_3); // CPLD_LED_3
        //if (xintf_testUnderTimer0_count & 8){
        //	*extData = xintf_testData2003; //data not important, write turns LED ON
        //} else {
        //	tempData16 = *extData;         // data not important, read turns LED OFF
        //}
        break;

    case 0x0009: // Each T0Timer Interrupt toggle 2 LEDs on / off in sync
    	// CPLD LED's 0, & 1
    	extData = (Uint16 *)0x080A00; // CPLD_LED_0
        if (xintf_testUnderTimer0_count & 1){
        	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_0); // CPLD_LED_0
        	*extData = xintf_testData2003; //data not important, write turns LED ON
        	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1); // CPLD_LED_1
        	*extData = xintf_testData2003; //data not important, write turns LED ON
        	//extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_2); // CPLD_LED_2
        	//*extData = xintf_testData2003; //data not important, write turns LED ON
        	//extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_3); // CPLD_LED_3
        	//*extData = xintf_testData2003; //data not important, write turns LED ON
        } else {
        	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_0); // CPLD_LED_0
        	tempData16 = *extData;         // data not important, read turns LED OFF
        	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1); // CPLD_LED_1
        	tempData16 = *extData;         // data not important, read turns LED OFF
        	//extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_2); // CPLD_LED_2
        	//tempData16 = *extData;         // data not important, read turns LED OFF
        	//extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_3); // CPLD_LED_3
        	//tempData16 = *extData;         // data not important, read turns LED OFF
        }
        break;

    case 0x000A: // Turn 2 CPLD LED's OFF
    	// LED's 0, & 1

    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_0); // CPLD_LED_0
        tempData16 = *extData;         // data not important, read turns LED OFF
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1); // CPLD_LED_1
        tempData16 = *extData;         // data not important, read turns LED OFF
    	//extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_2); // CPLD_LED_2
        //tempData16 = *extData;         // data not important, read turns LED OFF
    	//extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_3); // CPLD_LED_3
        //tempData16 = *extData;         // data not important, read turns LED OFF

        xintf_testSelection2004 = 0; // do once then turn off
        break;

    case 0x000B: // Turn on / off CPLD_LED_0 (latched J/K flip flop output)
    	         // Read or write data to address TBIOM_CPLD_LED_0
    	         // for CPLD_LED, doesn't matter what data we read or write
    	         // just the fact that we are reading or writing
    	extData = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_0);
    	extData_2 = (Uint16 *)0x080000;
        if (xintf_testUnderTimer0_count & 1){
        	*extData_2 = xintf_testData2003; // access out of target area
        	*extData = xintf_testData2003;
        	*extData = xintf_testData2003; // see if 3 x helps (or works)
        	*extData = xintf_testData2003;
        	*extData_2 = xintf_testData2003; // access out of target area
        } else {
        	tempData16 = *extData_2; // access out of target area
        	tempData16 = *extData;
        	tempData16 = *extData;
        	tempData16 = *extData;
        	tempData16 = *extData_2; // access out of target area
        }
        break;

    case 0x000C: // Turn on / off TB3IOMA's CPLD_LED_0 and 1
    	         // Here we Read/Write to hardcoded address for rev_A TB3IOMB
    	         // This is used specifically for comparing CPLD for RevA and B
    	extData = (Uint16 *)0x080A00;    // CPLD LED 0 for TB3IOMA
    	extData_2 = (Uint16 *)0x080000;
        if (xintf_testUnderTimer0_count & 1){
        	*extData_2 = xintf_testData2003; // access out of target area
        	*extData = xintf_testData2003;
        	*extData = xintf_testData2003; // see if 3 x helps (or works)
        	*extData = xintf_testData2003;
        	*extData_2 = xintf_testData2003; // access out of target area

        	extData = (Uint16 *)0x0C0900;    // CPLD LED 1 for TB3IOMA
        	*extData_2 = xintf_testData2003; // access out of target area
        	*extData = xintf_testData2003;
        	*extData = xintf_testData2003; // see if 3 x helps (or works)
        	*extData = xintf_testData2003;
        	*extData_2 = xintf_testData2003; // access out of target area
        } else {
        	tempData16 = *extData_2; // access out of target area
        	tempData16 = *extData;
        	tempData16 = *extData;
        	tempData16 = *extData;
        	tempData16 = *extData_2; // access out of target area

        	extData = (Uint16 *)0x0C0900;    // CPLD LED 1 for TB3IOMA
        	tempData16 = *extData_2; // access out of target area
        	tempData16 = *extData;
        	tempData16 = *extData;
        	tempData16 = *extData;
        	tempData16 = *extData_2; // access out of target area
        }
        break;


    default:
    	break;
    }

	tempData16 = dataRead; // this statement does NOTHING except
	                       // to prevent getting a warning on
	                       // dataRead set but never used -- DH

}

void xintf_test2010(Uint16 dataword){
	// Write Data Once to previously specified external address
	volatile Uint16 *extData;
	Uint32 tempData32;

	tempData32 = xintf_testData2001;
	tempData32 = (tempData32 << 16) + xintf_testData2002;
	extData = (Uint16 *)tempData32;
	*extData = dataword;
}

void xintf_test2011(void){
	// Read data Data Once from previously specified external address
	// and log result to RS232
	volatile Uint16 *extData;
	Uint16 dataRead;
	Uint32 tempData32;
	char msgOut[64];
    char *ptr;

	tempData32 = xintf_testData2001;
	tempData32 = (tempData32 << 16) + xintf_testData2002;
	extData = (Uint16 *)tempData32;
	dataRead = *extData;

    ptr = strU_strcpy(msgOut,"Read from address 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,xintf_testData2001);
    ptr = strU_strcpy(ptr,",");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,xintf_testData2002);
    ptr = strU_strcpy(ptr," : 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,dataRead);
    ptr = strU_strcpy(ptr,"\n\r");
	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

}
