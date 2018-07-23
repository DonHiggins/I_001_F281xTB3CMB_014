// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     LED.C
//
// Turn on/off LED's for DSP (on TB3CMB)
// and for FPGAs and CPLD on TB3IOMC
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "stdbool.h"            // needed for bool data types

#include "Timer0.H"
#include "TaskMgr.h"
#include "GpioUtil.h"
#include "Xintf.h"
#include "FpgaTest.h"
#include "CPLD.H"
#include "LED.H"

Uint16 led_synchronizedHeartbeatCounter; // doesn't need initialization
void led_synchronizedSlowHeartbeat(void){
	// called from timer0 every 0.5 sec
	// advance a counter used by DSP, CPLD, and FPGA LED's for synchronized heartbeat
	// Counts from 0 to F, bit 0x8 turns LED on/off
	led_synchronizedHeartbeatCounter++;
	led_synchronizedHeartbeatCounter &= 0x000F;
}


//==========================================================================
//
//                C P L D   L E D S
//
//==========================================================================
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// LED non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
enum LED_CPLD_PATTERN led_cpldIoPattern;	// For CPLD on TB3IOM
											// if < 0x10 then write ls 4 bits to LEDs
                                    		// if == 0x1003 then display SLOW_HEARTBEAT
enum LED_CPLD_PATTERN led_cpldPmPattern;	// For CPLD on TB3PM
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessors for LED non-public variables
void led_setLedCpldIoPattern(enum LED_CPLD_PATTERN pattern){
	led_cpldIoPattern = pattern;
	if (pattern != LED_CPLD_PATTERN_DISABLED) {
		xintf_setTestSelection2004(0); // Turn off XINTF tests involving CPLD LEDs
	}
}

void led_setLedCpldPmPattern(enum LED_CPLD_PATTERN pattern){
	led_cpldPmPattern = pattern;
	if (pattern != LED_CPLD_PATTERN_DISABLED) {
		xintf_setTestSelection2004(0); // Turn off XINTF tests involving CPLD LEDs
	}
}

enum LED_CPLD_PATTERN led_getCpldIoLedPattern(void){
	return led_cpldIoPattern;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Initialize a couple led/cpld-io variables
//  called from timer0_task_init( )
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void led_cpldLedIoInit(void){
	// LED's
	led_setLedCpldIoPattern(LED_CPLD_PATTERN_SLOW_HEARTBEAT); // display slow heartbeatin TB3IOM CPLD LED's
	led_setLedCpldPmPattern(LED_CPLD_PATTERN_SLOW_HEARTBEAT); // display slow heartbeatin TB3PM CPLD LED's
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Manage CPLD LEDs
//  called from timer0_task, launched each every 0.5 sec
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void led_manageCpldIoLEDsUnderTimer0(void){
	volatile Uint16 *extData;
	volatile Uint16 dummyReadData;
	Uint16 i;
	enum LED_CPLD_PATTERN led_cpldPattern;
    Uint16* cpldLed0;
    Uint16* cpldLed1;

	// handle TB3IOM CPLD (0), then TB3PM CPLD (1)
	for (i=0;i<2;i++){
		// First pick out addresses and values specific to TB3IOM or TB3PM
		if (i==0) {
			led_cpldPattern = led_cpldIoPattern;
		    cpldLed0 = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_0);
			cpldLed1 = CPLD_XINTF_ADDR(TBIOM_CPLD_LED_1);
		} else {
			led_cpldPattern = led_cpldPmPattern;
		    cpldLed0 = CPLD_XINTF_ADDR(TBPM_CPLD_LED_0);
			cpldLed1 = CPLD_XINTF_ADDR(TBPM_CPLD_LED_1);
		}

		// CPLD LED's (see LED.h)
		//	enum LED_PATTERN {
		//		LED_CPLD_PATTERN_DIRECT               =  0x000F,
		//		LED_CPLD_PATTERN_SLOW_HEARTBEAT       =  0x1003

		// Now go ahead and parse the LED_CPLD_PATTERN enum value and apply the
		// proper signals to the specified CPLD (TB3IOM or TB3PM)
		if(led_cpldPattern <= LED_CPLD_PATTERN_DIRECT){
			// ----- Display value of ledPattern in LEDs -----
			extData = cpldLed0; // copy address
			if (led_cpldPattern & 0x0001) {
				*extData = 1; // data not important, on/off based on read/write
			} else {
				dummyReadData = *extData;
			}
			extData = cpldLed1; // copy address
			if (led_cpldPattern & 0x0002) {
				*extData = 1; // data not important, on/off based on read/write
			} else {
				dummyReadData = *extData;
			}

		} else if(led_cpldIoPattern == LED_PATTERN_SLOW_HEARTBEAT){
			// ----- turn one LED slowly on and off -----
			extData = cpldLed0; // copy address
			if ((led_synchronizedHeartbeatCounter) & 0x0008) {
				dummyReadData = *extData;
			} else {
				*extData = 1; // data not important, on/off based on read/write
			}
			extData = cpldLed1; // copy address
			*extData = 1; // data not important, on/off based on read/write

		} else if(led_cpldPattern == LED_CPLD_PATTERN_DISABLED){
			// ----- do nothing with CPLD LEDs, someone else is controlling them -----

		} else {
			// ----- CPLD LEDs Off -----
			extData = cpldLed0; // copy address
			*extData = 1; // data not important, on/off based on read/write
			extData = cpldLed1; // copy address
			*extData = 1; // data not important, on/off based on read/write
		}
	} //end For loop

}


//==========================================================================
//
//                D S P   L E D S
//
//==========================================================================

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// LED non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
enum LED_PATTERN led_dspLedPattern;	// if < 0x10 then write ls 4 bits to LEDs
                                    // if == 0x1000 then display incrementing count (led_dspLedCount)
                                    // else toggle LEDs
enum LED_ERROR_NUMBER led_dspLedErrCount;
Uint16 led_dspLedCount;

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessors for LED non-public variables
void led_setDspLedPattern(enum LED_PATTERN pattern){
	led_dspLedPattern = pattern;
}

enum LED_PATTERN led_getDspLedPattern(void){
	return led_dspLedPattern;
}

void led_dspLedErrMsg(enum LED_ERROR_NUMBER count){
// blink LED on/off a number of times, pause, and repeat
	led_dspLedPattern = LED_PATTERN_BLINK_ERROR;
	led_dspLedCount = 0;
	led_dspLedErrCount = count;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Toggle all 4 DSP LEDs: B0, B1, B2, B3
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void led_toggle4DspLeds(void){
	 GpioDataRegs.GPBTOGGLE.all = 0x000F; // toggle only LS 4 bits of GPIOB
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Initialize ON/OFF for all 4 DSP LEDs: B0, B1, B2, B3
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void led_init4DspLeds(void){
   // GPIO3B0:3 -- 2 on 2 off
   // all other off
   GpioDataRegs.GPBCLEAR.all  |= 0x0001;
   GpioDataRegs.GPBSET.all    |= 0x0002;
   GpioDataRegs.GPBCLEAR.all  |= 0x0004;
   GpioDataRegs.GPBSET.all    |= 0x0008;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Manage DSP LEDs: B0, B1, B2, B3
//  called from timer0_task, launched each Timer0 interrupt, nominally every 0.5 sec
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void led_manageDspLEDsUnderTimer0(void){

	// LED's (see Timer0.h)
	//	enum LED_PATTERN {
	//		LED_PATTERN_DIRECT               =  0x000F,
	//		LED_PATTERN_INCREMENT            =  0x1001,
	//		LED_PATTERN_BLINK_ERROR          =  0x1002,
	//		LED_PATTERN_SLOW_HEARTBEAT       =  0x1003

	if(led_dspLedPattern <= LED_PATTERN_DIRECT){
	    // ----- Display value of ledPattern in LEDs -----
		GpioDataRegs.GPBSET.all    |= 0x000F; // clear all 4 in TB3CMB
		GpioDataRegs.GPBCLEAR.all  |=                // display patern in TB3CMB
				(((led_dspLedPattern << 3) & 0x0008)  // reverse order of bits here
				|((led_dspLedPattern << 1) & 0x0004)
				|((led_dspLedPattern >> 1) & 0x0002)
				|((led_dspLedPattern << 3) & 0x0001));

	} else if(led_dspLedPattern == LED_PATTERN_BLINK_ERROR){
	    // ---- error code: blink a set number of times, pause, repeat -----
	    if (led_dspLedCount < (led_dspLedErrCount<<1)) {
	       if ((led_dspLedCount & 1) == 0) {
	    	  // LED OFF, on even values
			  GpioDataRegs.GPBSET.all  |= 0x000F; // clear all 4 off (adjusted for TB3CMB)
	       } else {
			  GpioDataRegs.GPBCLEAR.all    |= 0x000F; // set all 4 on (adjusted for TB3CMB)
	       }
	    } else if (led_dspLedCount < (led_dspLedErrCount<<1)+ 4) {
		   GpioDataRegs.GPBSET.all  |= 0x000F; // clear all 4 off (adjusted for TB3CMB)
	    } else {
   		   led_dspLedCount = 0;
	    }
	    led_dspLedCount++;

	} else if(led_dspLedPattern == LED_PATTERN_INCREMENT){
	    // ----- increment led_dspLedCount and display in LED's -----
		GpioDataRegs.GPBSET.all    |= 0x000F; // clear all 4 in TB3CMB
		GpioDataRegs.GPBCLEAR.all  |=                // display patern in TB3CMB
				(((led_dspLedCount << 3) & 0x0008)  // reverse order of bits here
				|((led_dspLedCount << 1) & 0x0004)
				|((led_dspLedCount >> 1) & 0x0002)
				|((led_dspLedCount >> 3) & 0x0001));
	    led_dspLedCount++;

	} else if(led_dspLedPattern == LED_PATTERN_SLOW_HEARTBEAT){
	    // ----- increment led_dspLedCount and turn one LED slowly on and off -----
	    GpioDataRegs.GPBSET.all    |= 0x000F; // turn all 4 off (in TB3CMB)
	    // apply on/off bit.1 to setting (or not) rightmost LED (#4)
	    GpioDataRegs.GPBCLEAR.all  |= ((led_synchronizedHeartbeatCounter) & 0x0008);

	} else {
	    // ----- Toggle LEDs GPIO -----
		led_toggle4DspLeds();
   }
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Initializatize a couple variables
//  called from timer0_task_init( ) above
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void led_DspLedInit(void){
	// LED's
	led_dspLedPattern = LED_PATTERN_INCREMENT; // increment led_dspLedCount and display in LED's
	led_dspLedCount = 0x0000;
}

//==========================================================================
//
//                F P G A   L E D S
//
//==========================================================================
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// LED non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
enum LED_FPGA_PATTERN led_fpga1Pattern;
enum LED_FPGA_PATTERN led_fpga2Pattern;
enum LED_FPGA_PATTERN led_fpga3Pattern;

Uint16 led_fpgaCount;
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessors for LED non-public variables
void led_setLedFpga1Pattern(enum LED_FPGA_PATTERN pattern){
	led_fpga1Pattern = pattern;
	if (pattern != LED_CPLD_PATTERN_DISABLED) {
		//xintf_setTestSelection2004(0); // Turn off XINTF tests involving CPLD LEDs
	}
}

void led_setLedFpga2Pattern(enum LED_FPGA_PATTERN pattern){
	led_fpga2Pattern = pattern;
	if (pattern != LED_CPLD_PATTERN_DISABLED) {
		//xintf_setTestSelection2004(0); // Turn off XINTF tests involving CPLD LEDs
	}
}

void led_setLedFpga3Pattern(enum LED_FPGA_PATTERN pattern){
	led_fpga3Pattern = pattern;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Initializatize a couple variables
//  called from timer0_task_init( ) above
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void led_FpgaLedInit(void){
	// LED's
	led_fpga1Pattern = LED_FPGA_PATTERN_DSP_SLOW_HEARTBEAT;
	led_fpga2Pattern = LED_FPGA_PATTERN_DSP_SLOW_HEARTBEAT;
	led_fpga3Pattern = LED_FPGA_PATTERN_DSP_SLOW_HEARTBEAT;
}

void led_manageFpgaLedsUnderTimer0(void){
	// called each 0.5 sec from timer0 task
	Uint16 fpga_num;
	Uint16 *fpgaLedFunction;
	Uint16 *fpgaLedWriteDirectly;
	enum LED_FPGA_PATTERN led_fpgaPattern;


	for (fpga_num=1;fpga_num<4;fpga_num++){
		// Do everything 3 times: FPGAs 1 & 2 on I/O module, 3 on Pwr module
		switch(fpga_num){
		// First, we set our local pointers to the proper addresses for whichever FPGA
		// we are working on.
	    case 1: //
	    	fpgaLedFunction = CPLD_F1_XA(FPGA1_WRITE_LED_FUNCTION);
	    	fpgaLedWriteDirectly = CPLD_F1_XA(FPGA1_WRITE_LED_DIRECTLY);
	    	led_fpgaPattern = led_fpga1Pattern;
	        break;

	    case 2: //
	    	fpgaLedFunction = CPLD_F2_XA(FPGA2_WRITE_LED_FUNCTION);
	    	fpgaLedWriteDirectly = CPLD_F2_XA(FPGA2_WRITE_LED_DIRECTLY);
	    	led_fpgaPattern = led_fpga2Pattern;
	        break;

	    case 3: //
	    	// not yet implemented for FPGA3 on TB3PM power module
	    	fpgaLedFunction = CPLD_F3_XA(FPGA3_WRITE_LED_FUNCTION);
	    	fpgaLedWriteDirectly = CPLD_F3_XA(FPGA3_WRITE_LED_DIRECTLY);
	    	led_fpgaPattern = led_fpga3Pattern;
	        break;

	     default:

	    	break;
	    } // end Switch/Case


		// Now perform actions based on which "pattern" is selected for the FPGA.
		// Remember this is called every 0.5 sec from a timer0 task
		if(led_fpgaPattern <= LED_FPGA_PATTERN_DIRECT){
			*fpgaLedFunction = 0;	// function is LED_DIRECTLY_FROM_DSP
			*fpgaLedWriteDirectly = (Uint16)led_fpgaPattern;
		} else if(led_fpgaPattern == LED_FPGA_PATTERN_COUNT_CLOCK){
			*fpgaLedFunction = 1;	// LED_FROM_COUNT_CLK               = 2'b01;
		} else if(led_fpgaPattern == LED_FPGA_FROM_COUNT_WR_STORERD_VAL_1){
			*fpgaLedFunction = 2;	// LED_FROM_COUNT_WR_STORERD_VAL_1  = 2'b10;
		} else if(led_fpgaPattern == LED_FPGA_PATTERN_INTERNAL_SLOW_HEARTBEAT){
			*fpgaLedFunction = 3;	// LED_SLOLW_HEARTBEAT              = 2'b11;
		} else if(led_fpgaPattern == LED_FPGA_PATTERN_DSP_SLOW_HEARTBEAT){
		    // ----- turn one LED slowly on and off -----
			*fpgaLedFunction = 0;	// function is LED_DIRECTLY_FROM_DSP
			if ((led_synchronizedHeartbeatCounter) & 0x0008) {
				*fpgaLedWriteDirectly = 0x0E;	// LED on -- TB3IOMC 0 is on, 1 is off
			} else {							//   hardware may be different in TB3IOMD
				*fpgaLedWriteDirectly = 0x0F;	// LED off
			}
		} else if(led_fpgaPattern == LED_FPGA_PATTERN_DISABLED){
			//This "pattern" tells us explicitly to do nothing.
			//If you want the leds off, you may want to command LED_FPGA_PATTERN_DIRECT
			// with a direct value of 0 before doing this.
		}

	} // end For
}
