// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Comint.C
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
// NOTE: "DSP281x_Device.h" defines "Uint16" data type

#include "Comint.H"
#include "Timer0.h"
#include "Xintf.h"
#include "GpioUtil.h"
#include "FpgaTest.h"

#include "Rs232Out.H"
#include "Rs232.H"
#include "StrUtil.H"
#include "HexUtil.H"

#include "TaskMgr.h"
#include "FlashRW.h"
#include "stdbool.h"
#include "EVTimer4.H"
#include "I2CEE.H"
#include "TimeStamp.H"
#include "ADC.H"
#include "CPLD.H"
#include "ExtRam.H"
#include "CanComm.H"
#include "LED.H"
#include "Main.H"

char msg_notACommand[] = {"Not a command\n\r"};
char msg_crLf[] = {"\n\r"};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Command Interpreter -- rs232 decoded an incoming command
//   Here we act on it.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void comint_comint(Uint16 command,Uint16 dataWord,bool dataPresent){

    // NEED TO GENERALIZE MECHANISM OF DECODING COMMANDS AND KICKING OFF TASKS

	char msgOut[64];
    char *ptr;
    Uint16 data_in;

	switch(command){
    case 0x1000: // select a pattern to blink the LEDs
    	if (dataPresent){
    		led_setDspLedPattern((enum LED_PATTERN)dataWord); // see timer0_task()
    	}
        break;

    case 0x1002: // select a pattern to blink the LEDs
    	if (dataPresent){
    		led_dspLedErrMsg((enum LED_ERROR_NUMBER)dataWord); // see timer0_task() -- blink a fixed # times, pause, repeat
    	}
        break;

    case 0x1003: // Display the build date-timestamp on RS232
    	ts_displayBuildDateTimestamp();
        break;


    case 0x1005: // Display baud rate, parity, chars & stop bits for rs232
        rs232_reportBaudParityEtc();
        break;

    case 0x1006: // Report to RS232 are CPLD addresses configured for TB3IOMA or B
    	comint_DisplayCpldAddrConfig();
        break;

    case 0x1007: // Dispaly count of times thru main loop, waiting for Power Good
    	main_DisplayPgLoopCount();
        break;

    case 0x1011: // Read ADC input channel dddd and report voltage on RS232
    	if (dataPresent){
    		adc_readOneAdcChannelToRs232(dataWord);
    	}
        break;

    case 0x1012: // Read ADC input channel dddd and report voltage on RS232
    	if (dataPresent){
    		adc_addOneAdcChannelToList(dataWord);
    	}
        break;

    case 0x1013: // Repeat previously configured ADC test
    	adc_repeatPrevAdcTest();
    	break;

    case 0x1014: //
    	adc_readAll16Channels();
        break;

    case 0x1020: // Test Task Mgr scheduling algorithms
    	if (dataPresent){
    		taskMgr_testSchedAlgorithms(dataWord);
    	}
        break;

    case 0x1030: // Read GPIOA bits 2 & 0, and report to RS232
    	// Note: TB3CMA Protos were blue-wired connecting
    	//  GPIOA0 == PG_1V2_I/O, and
    	//  GPIOA2 == PG_1V9_2
    	// in other words, I was connecting "power good" signals
    	// from on-board power supplies -- one on TB3CM and one
    	// accross the inter-module connector on TB3IOM.
    	// At the time this was strictly a test of whether the signals
    	// properly drove the DSP GPIO ports.
    	// As of TB3CMB, those GPIO pins are repurposed, to read
    	// F_DONE_x signals from FPGA's.
    	data_in = (GpioDataRegs.GPADAT.all & 0x0005);
    	ptr = strU_strcpy(msgOut,"GPIOA bits 2,0 0x");
    	ptr = hexUtil_binTo4HexAsciiChars(ptr,data_in);
    	ptr = strU_strcpy(ptr,"\n\r");
    	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
        break;

    case 0x1031: // Read Power Good signals from IO and PM
    	data_in = main_readPowerGoodSignals();
    	ptr = strU_strcpy(msgOut,"PowerGood Signals 0x");
    	ptr = hexUtil_binTo4HexAsciiChars(ptr,data_in);
    	ptr = strU_strcpy(ptr,"\n\r");
    	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
        break;

    case 0x1040: // Read CAN Address Dip Switches, and report to RS232
    	data_in = canC_readCanAddrDipSwitches();
    	ptr = strU_strcpy(msgOut,"CAN Addr from dip switches 0x");
    	ptr = hexUtil_binTo4HexAsciiChars(ptr,data_in);
    	ptr = strU_strcpy(ptr,"\n\r");
    	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
        break;

    case 0x2000: // initialize Xintf, external bus
    	// As of 4/8/2014 I'm adding a call to xintf_InitXintf() to initialization in main()
    	xintf_InitXintf();
        break;

    // 0x2001-4 store 4 words of data to configure action of
    // the Xintf test of external bus run by Timer0 interrupt routine
    case 0x2001: // Xintf, test configuration data
    	if (dataPresent){
    		xintf_setTestData2001(dataWord);
    	}
        break;
    case 0x2002: // Xintf, test configuration data
    	if (dataPresent){
    		xintf_setTestData2002(dataWord);
    	}
        break;
    case 0x2003: // Xintf, test configuration data
    	if (dataPresent){
    		xintf_setTestData2003(dataWord);
    	}
        break;

    case 0x2004: // Xintf, test configuration data
    	if (dataPresent){
    		xintf_setTestSelection2004(dataWord);
    	}
        break;

    case 0x2005: // Misc specific actions on CPLD & FPGA
    	if (dataPresent){
    		frw_test2005(dataWord);
    	}
        break;
    case 0x2010: // Write data to previously determined external address
    	if (dataPresent){
    		xintf_test2010(dataWord);
    	}
        break;

    case 0x2011: // Read from previously determined external address, and report on RS232
    	xintf_test2011();
        break;

    case 0x2012: // Turn off CPLD and FPGA LED actions based on T0 Timer
        			//  This removes that activity from the parallel bus, so you
        			// can scope other bus activity.
    	led_setLedCpldIoPattern(LED_CPLD_PATTERN_DISABLED);
    	led_setLedFpga1Pattern(LED_FPGA_PATTERN_DISABLED);
    	led_setLedFpga2Pattern(LED_FPGA_PATTERN_DISABLED);
        break;

    case 0x2020: // Test External RAM chip on TB3CMB
    	if (dataPresent){
    		xram_testExternalRam(dataWord);
    	}
        break;

   case 0x2100: // GPIO, test IO
    	if (dataPresent){
    		GpioU_ioTest2100(dataWord);
    	}
        break;

   case 0x2101: // GPIO, test IO
    	if (dataPresent){
    		GpioU_ioTest2101(dataWord);
    	}
        break;

   case 0x2102: // GPIO, test IO
    	if (dataPresent){
    		GpioU_ioTest2102(dataWord);
    	}
        break;

    case 0x3003: // FpgaT, test configuration data
    	if (dataPresent){
    		fpgaT_setTestData3003(dataWord);
    	}
        break;

    case 0x3004: // fpgaT, test configuration data
    	if (dataPresent){
    		fpgaT_setTestSelection3004(dataWord);
    	}
        break;

    case 0x3005: // fpgaT, test configuration data
    	if (dataPresent){
    		fpgaT_test3005(dataWord);
    	}
        break;

    case 0x3010: // fpga, set Timer0 Pattern for FPGA1 LEDs
    	if (dataPresent){
    		led_setLedFpga1Pattern((enum LED_FPGA_PATTERN)dataWord);
    	}
        break;

    case 0x3011: // fpga, LED FUNCTION to FPGA1
    	if (dataPresent){
    		*CPLD_F1_XA(FPGA1_WRITE_LED_FUNCTION) = dataWord;
    	}
        break;

    case 0x3012: // fpga, DIRECT LED DATA to FPGA1
    	if (dataPresent){
    		*CPLD_F1_XA(FPGA1_WRITE_LED_DIRECTLY) = dataWord;
    	}
        break;

    case 0x3020: // fpga, set Timer0 Pattern for FPGA2 LEDs
    	if (dataPresent){
    		led_setLedFpga2Pattern((enum LED_FPGA_PATTERN)dataWord);
    	}
        break;

    case 0x3021: // fpga, LED FUNCTION to FPGA2
    	if (dataPresent){
    		*CPLD_F2_XA(FPGA2_WRITE_LED_FUNCTION) = dataWord;
    	}
        break;

    case 0x3022: // fpga, DIRECT LED DATA to FPGA2
    	if (dataPresent){
    		*CPLD_F2_XA(FPGA2_WRITE_LED_DIRECTLY) = dataWord;
    	}
        break;

    case 0x4000: // SPI / Flash test
    	if (dataPresent){
    		frw_test4000(dataWord);
    	}
        break;

    case 0x4001: // SPI / Flash set HI word of Addr
    	if (dataPresent){
    		frw_test4001(dataWord);
    	}
        break;

    case 0x4002: // SPI / Flash set LOW word of Addr
    	if (dataPresent){
    		frw_test4002(dataWord);
    	}
        break;

    case 0x4003: // SPI / Flash Read # bytes from Flash
    	if (dataPresent){
    		frw_test4003(dataWord);
    	}
        break;
    case 0x4004: // SPI / Flash Write # bytes from Flash
    	if (dataPresent){
    		frw_test4004(dataWord);
    	}
        break;
    case 0x4005: // SPI / Flash initialize frwRWBuff for testing
    	if (dataPresent){
    		frw_test4005(dataWord);
    	}
        break;
    case 0x4006: // SPI / Select one of 2 flash chips for subsequent operations
    	if (dataPresent){
    		frw_test4006(dataWord);
    	}
        break;

    case 0x4101: // EVTimer4 turn it on, dummy task, use timer prescale 128
    	if (dataPresent){
    		evtimer4_turn_on (dataWord, 128);
    	}
        break;
    case 0x4102: // EVTimer4 turn it off
    	if (dataPresent){
    		evtimer4_turn_off();
    	}
        break;
    case 0x4103: // EVTimer4 turn it on, dummy task, use timer prescale 1
    	if (dataPresent){
    		evtimer4_turn_on (dataWord, 1);
    	}
        break;
    case 0x4200: // I2C EEProm Read 8 bytes from address = dataWord
    	if (dataPresent){
    		i2cee_test4200(dataWord);
    	}
        break;
    case 0x4201: // I2C EEProm Write bytes at address = (dataWord >> 8),
    	//use (dataWord & 0xFF) as a seed for data values
    	if (dataPresent){
    		i2cee_test4201(dataWord);
    	}
        break;
    case 0x4202: // Set / Reset I2C_EE_WP line from CPLD
    	         // dddd == 0 -> reset to 0, dddd !=0 -> set to 1
    	// In 24LC01B I2C EEProm,
    	//	Write protect is in force (ON) when the WP pin is HIGH (1)
    	//  The chip is write enabled - write protect is OFF - when the WP pin is LOW (0)
    	if (dataPresent){
    		if (dataWord == 0){
    			i2cee_writeProtect(I2CEE_WP_OFF);
    		} else {
    			i2cee_writeProtect(I2CEE_WP_ON);
    		}
    	}
        break;
    case 0x4203: // Dump entire I2C EEProm to RS232
    	if (dataPresent){
    		i2cee_test4203(dataWord);
    	}
        break;
    case 0x4204: // Write 1 byte in I2cEEProm
    	         // dataword = 0xWXYZ
    	         // write data 0xYZ at address 0xWX
    	if (dataPresent){
    		i2cee_test4204(dataWord);
    	}
        break;
    case 0x4205: // Read 1 byte in I2cEEProm
    	         // dataword = 0xWXYZ
    	         // read data at address 0xWX
    	if (dataPresent){
    		i2cee_test4205(dataWord);
    	}
        break;
    case 0x4206: // Read 2 consecutive bytes in I2cEEProm
    	         // dataword = 0xWXYZ
    	         // read data starting at address 0xWX
    	if (dataPresent){
    		i2cee_test4206(dataWord);
    	}
        break;
    case 0x4207: // Designate one of 3 I2cEEProms for future r/w operations
    			//  dataword = 0 -- none selected
				//  dataword = 1 -- TB3CM
				//  dataword = = 2 -- TB3IOM
				//  dataword = = 3 -- TB3PM
    	if (dataPresent){
    		i2cee_selectEEProm(dataWord);
    	}
        break;

     default:
     	/* success = */ r232Out_outCharsNT(msg_notACommand);
    	break;
    }
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Command Interpreter Extension -- rs232 decoded an incoming command
//   Here we take an abbreviated form of the command, flesh it out,
//   and call comint_comint( ) to act on it.  These are the "Speed-Dial" commands
//   starting with "s"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
enum LED_CPLD_PATTERN ledCpldPattern;
void comint_comintSpeedDial(Uint16 command){
	char msgOut[32];
	char *ptr;

	switch(command){
    case 0x0000: // OoaD out to RS232
    	taskMgr_setTaskRoundRobin(TASKNUM_BgTask_ooad,0); // transmit ooad out of RS232 port
        break;
    case 0x0001: // Set DSP lines to FPGA to allow JTAG Programming
		ptr = strU_strcpy(msgOut," -Set FPGA ready for JTAG-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	comint_comint(0x2100,0x0020,true);
        break;
    case 0x0002: // turn on DSP external Bus
		ptr = strU_strcpy(msgOut," -Set DSP external Bus ON-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	comint_comint(0x2000,0,false);
        break;
    case 0x0003: // Toggle DSP CPLD LED mode
		ptr = strU_strcpy(msgOut," -DSP LED display-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
		if (led_getDspLedPattern() == LED_PATTERN_INCREMENT){      // incrementing count pattern
			led_setDspLedPattern((enum LED_PATTERN)0);			   // all 4 off
		} else if (led_getDspLedPattern() == (enum LED_PATTERN)0){
			led_setDspLedPattern(LED_PATTERN_SLOW_HEARTBEAT);
		} else {
			led_setDspLedPattern(LED_PATTERN_INCREMENT);
		}

        break;
    case 0x0004: // Toggle CPLD LED display
		ptr = strU_strcpy(msgOut," -CPLD LED display-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
		//comint_comint(0x2004,0x0008,true);
		//if (xintf_getTestSelection2004() == 0x0008){  // incrementing count pattern
		//	xintf_setTestSelection2004(0x000A);		  // all 4 off
		//} else {
		//	xintf_setTestSelection2004(0x0008);
		//}
		ledCpldPattern = led_getCpldIoLedPattern();
		if (ledCpldPattern < 4){
			ledCpldPattern++; // set LED's to stationary value 0,1,2,3
			led_setLedCpldIoPattern(ledCpldPattern);
		} else if (ledCpldPattern == 4){
			led_setLedCpldIoPattern(LED_CPLD_PATTERN_SLOW_HEARTBEAT);
		} else {
			led_setLedCpldIoPattern(LED_CPLD_PATTERN_DIRECT_0); // set LED's to stationary value 0
		}
        break;
    case 0x0005: // FPGA LED display
		ptr = strU_strcpy(msgOut," -FPGA LED display-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
		if (fpgaT_getledStatus() == FPGAT_LED_DISP_OFF) {
    	    comint_comint(0x3005,0x1106,true); // FPGA LED displays count clock FPGA_1
    	    comint_comint(0x3005,0x1108,true); // FPGA LED displays count clock FPGA_2
		} else {
    	    comint_comint(0x3005,0x1105,true); // FPGA LEDs off FPGA_1
    	    comint_comint(0x3005,0x1107,true); // FPGA LEDs off FPGA_2
		}
        break;
    case 0x0006: // START: FPGA read/write via both busses
		ptr = strU_strcpy(msgOut," -FPGA r/w test ON-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	comint_comint(0x3004,0x5000,true);
        break;
    case 0x0007: // REPORT ON: FPGA read / write via both busses
		ptr = strU_strcpy(msgOut," -FPGA r/w test REPORT-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	comint_comint(0x3005,0x5001,true);
        break;
    case 0x0008: // STOP: FPGA read/write via both busses
		ptr = strU_strcpy(msgOut," -FPGA r/w test STOP-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	comint_comint(0x3004,0x0000,true);
        break;
    case 0x0009: // Can DSP talk to FPGA ?
		ptr = strU_strcpy(msgOut," -Is FPGA Talking?-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
		comint_comint(0x3005,0x3110,true);
        break;
    case 0x0010: // Watchdog Reset
		ptr = strU_strcpy(msgOut,"\n\r -Watchdog Reset-\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
		taskMgr_setTaskRoundRobin(TASKNUM_wDogReset,0); // Cause Watchdog Reset
        break;

    default:
    	/* success = */ r232Out_outCharsNT(msg_notACommand);
    	break;
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Display list of speed-dial commands on RS232
// This is a task, fired off by task manager
// If it can't fit it's entire message in the RS232 Out Buff,
// then it sets itself again on the task list and exits.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Uint16 dsdl_index = 0;
void comint_DisplaySpeedDialList(void){
	char msgOut[64];
    char *ptr;
    bool success;

	switch(dsdl_index){
    case 0:
    	ptr = strU_strcpy(msgOut,"\n\r"); // add line feed at top.
	    ptr = strU_strcpy(ptr,"S0    -- OoaD to RS232");
        break;
    case 1:
//	    ptr = strU_strcpy(msgOut,"S1    -- Set DSP lines to FPGA to allow JTAG Programming");
        break;
    case 2:
//	    ptr = strU_strcpy(msgOut,"S2    -- Turn on DSP external Bus");
        break;
    case 3:
	    ptr = strU_strcpy(msgOut,"S3    -- Toggle DSP LED mode");
        break;
    case 4:
	    ptr = strU_strcpy(msgOut,"S4    -- Toggle CPLD LEDs");
        break;
    case 5:
	    ptr = strU_strcpy(msgOut,"S5    -- FPGA LEDs");
        break;
    case 6:
//	    ptr = strU_strcpy(msgOut,"S6    -- START: FPGA read/write via both busses");
        break;
    case 7:
//	    ptr = strU_strcpy(msgOut,"S7    -- REPORT ON: FPGA read/write via both busses");
        break;
    case 8:
//	    ptr = strU_strcpy(msgOut,"S8    -- STOP: FPGA read/write via both busses");
        break;
    case 9:
//	    ptr = strU_strcpy(msgOut,"S9    -- Is FPGA Talking?");
        break;
    case 10:
	    ptr = strU_strcpy(msgOut,"S10   -- Watchdog Reset");
        break;
    case 11:
	    ptr = strU_strcpy(msgOut,"  ");
        break;
   default:
    	dsdl_index = 0; // set us to restart at the top for next time
    	return; // exit without restarting task
    	// break;
    }

    ptr = strU_strcpy(ptr,"\n\r");
    success = r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    if (success){
    	dsdl_index++; // do next line next time
    }
	// run this task again to continue with the next step
    taskMgr_setTaskRoundRobin(TASKNUM_DisplayComintSpeedDialList,1);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Display which way we have CPLD addresses configured,
// For TB3IOMA or B.  See CPLD.H
// If it can't fit it's entire message in the RS232 Out Buff,
// then it sets itself again on the task list and exits.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void comint_DisplayCpldAddrConfig(void){
	char msgOut[64];
    char *ptr;

    ptr = strU_strcpy(msgOut,"  CPLD Addr. Configured for ");
#if TB3IOMA
    ptr = strU_strcpy(ptr,"TB3IOMA");
#else
    ptr = strU_strcpy(ptr,"TB3IOMB");
#endif
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Another Command Interpreter Extension --
//   Here we take a numeric command accompanied by a character-text parameter
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
char msg1[] = {"This is msg1\n\r"};
void comint_comintTextCommand(Uint16 command, char* textData, Uint16 length){

	switch(command){
    case 0x1000: // Write a fixed text string to the RS232 output
     	   /* success = */ r232Out_outCharsNT(msg1);
         break;

    case 0x1004: // Set RS232 speed, etc. --  T1004:9600-N-8-1
         rs232_changeBaudParityEtc(textData, length);
         break;

    default:
    	/* success = */ r232Out_outCharsNT(msg_notACommand);

    	break;
   }
}

void comint_comintHelp(void){
// user typed "HELP" or something like it
// for now, launch task to display Speed Dial menu
	/* success = */ r232Out_outCharsNT(msg_crLf); // leading line feed
	dsdl_index = 0;
	taskMgr_setTaskRoundRobin(TASKNUM_DisplayComintSpeedDialList,0);
}

