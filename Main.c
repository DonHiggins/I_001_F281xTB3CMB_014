// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Main.C
//
//	Innitialize DSP Hardware and Interrupts
//	Innitialize user code
//	Loop, invoking task manager to run tasks in the background, including polling CAN
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//


#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "RS232.H"
#include "TaskMgr.h"
#include "GpioUtil.H"
#include "Timer0.H"
#include "Xintf.h"
#include "CanComm.H"
#include "FlashRW.H"
#include "EVTimer4.H"
#include "ADC.H"
#include "F1Int.H"
#include "F2Int.H"
#include "AnlgIn.H"
#include "I2CEE.h"
#include "Main.H"
#include "SPI.H"
#include "StrUtil.H"
#include "HexUtil.H"
#include "Rs232Out.H"
#include "Log.H"
#include "DigIO.H"
#include "DigIO2.H"
#include "Resolver.H"
#include "SSEnc.h"
#include "LED.h"
#include "DigIO.h"
#include "LimitChk.h"


#define WDKEY        (volatile Uint16*)0x00007025   /* Watchdog key register */
#define WDCR         (volatile Uint16*)0x00007029   /* Watchdog control register */

extern Uint16 co_reset;
extern Uint16 co_jump_to_bootloader;

Uint16 idle_loop_count;
Uint16 pg_loopCount; // # of loops waiting to see Power_Good
                     // made this static so a task can report it to RS232
Uint16 main_startupTaskState;

void main(void) {
	void (*bootloaderProgStartVector)(void);
	Uint16 *bootloaderProgVerificationKey;
	Uint16 key1;
	Uint16 key2;

// Step 0.  Copy Ramfuncts code section into RAM
	Uint16* rfSrcPtr;
	Uint16* rfDestPtr;
	rfSrcPtr = &RamfuncsLoadStart;
	rfDestPtr = &RamfuncsRunStart;
//	Uint16 status;

	while (rfSrcPtr < &RamfuncsLoadEnd)
	{
		*(rfDestPtr++) = *(rfSrcPtr++);
	}

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP281x_SysCtrl.c file.
   InitSysCtrl();
   LOG_INIT; // init ram values for diagnostic event logging,
             // but only if it is instantiated in Log.H


// Step 2. Initalize GPIO:
   GpioU_defaultInit();			// Default initialization -- all inputs
   GpioU_initGpiosAllZero();	// set all GPIO's output value to 0
   GpioU_rs232Init();			// Use 2 pins as SciA Tx & Rx
   GpioU_sci2Init();			// Use 2 pins as SciB Tx & Rx
   GpioU_timer0Init();			// Use 4 pins as outputs for LEDs
   GpioU_SpiInit();				// 3 lines used for SPI, 1 Chip Select
   GpioU_i2cee_Init();			// 2 lines used for I2C to EEprom on TB3IOM

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP281x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP281x_DefaultIsr.c.
// This function is found in DSP281x_PieVect.c.
   InitPieVectTable();

// Interrupts used in TB3CMB are re-mapped to
// ISR functions found within modules in the source code.
   rs232_store_int_vectors_in_PIE();
   timer0_store_int_vectors_in_PIE();
   evtimer4_store_int_vectors_in_PIE();
   f2i_store_int_vectors_in_PIE(); // XInt13 from FPGA #2 for SS Enc
   f1i_store_int_vectors_in_PIE(); // XInt1 from FPGA #1

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP281x_InitPeripherals.c
// InitPeripherals(); // Not required
   rs232_setBaudRateDefault();     // SCI-A 9600 baud
   rs232_scia_fifo_init();         // Init SCI-A
   timer0_initConfig_n_Start();    // Initialize Timer peripheral registers
                                   // Configure frequency for Timer0 and Start it.
   evtimer4_initConfig_n_Start();  // Initialize Timer peripheral registers
   led_init4DspLeds();             // set 4 LED outputs On/Off
   InitSpi(); // initialize SPI serial device (connects to Flash on TB3IOM & TB3PM)
   adc_init(); // init A to D converters
   f2i_initialize_interrupt(); // XInt13 from FPGA #2 for SS Enc
   f1i_initialize_interrupt(); // XInt1 from FPGA #1

   xintf_InitXintf(); // External Bus Interface initialization.

//
// Step 5. User specific code initializations
   digio_initDacComparatorValuesClassic(); // table initialization for Digital In Comparators
   frw_initFpgaLoadPins(); // Set PROG line to FPGAs HI
   rs232_BgTaskInit();
   taskMgr_init();
   i2cee_Init(); // select NONE of 3 i2c eeproms
   res_init(); // resolver simulator
   ssEnc_init(); // sinusoidal encoder simulator
   timer0_task_init(); // zero out a state variable used in background task
   led_cpldLedIoInit(); // Set default TB3IOM CPLD LEDs to slow_heartbeat
   led_FpgaLedInit();   // Set default TB3IOM FPGA LEDs to slow_heartbeat
   rs232_Init();  // Initialize Tx & Rx variables and routines

   frw_SetWhichFlash(1); // 1=FLASH_1, 2=FLASH_2, 3=FLASH_3
   frw_SpiFlashInit(); //Initialize a few variables before we launch TASKNUM_SpiFlashTask

   // Temporary: For testing CPLD on TB3IOMB -- don't try programming FPGA from FLASH,
   //   instead display a 7-flash code in DSP LEDs
   // timer0_LedErrMsg(LED_ERROR_7); // blink the LEDs


   canC_initComm();  // further setup in the CanComm.c

   f2i_SSEnc_init(); // Sinusoidal Encoder / FPGA2

   ain_offsetCalcInit(); // initialization for routines to calibrate analog in

   pg_loopCount = 0;	// count the number of times we loop waiting to see power good
   main_startupTaskState = 0;
   taskMgr_setTaskRoundRobin(TASKNUM_main_startupTask,0); // orderly startup of other tasks
   co_reset = 0;				// set non-zero when CAN command requests DSP Reset
   co_jump_to_bootloader = 0;	// set non-zero when CAN command requests vector to bootloader
   limchkInit();       // initialize Limit Check RAM

// Step 6. Enable interrupts
   rs232_enable_PIE_int();
   timer0_init_03();
   evtimer4_enable_int();
   f2i_enable_interrupt();  // Int13 from FPGA #2 for SS Enc
// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
// CPU_enableGlobalInts(myCpu);
   ERTM;   // Enable Global realtime interrupt DBGM
// CPU_enableDebugInt(myCpu);

   // Step 7. IDLE loop. Just sit and loop forever :
   idle_loop_count = 0;
// MAIN LOOP
   for(;;){
	   idle_loop_count++; // just a convenient place for breakpoint, not functional

	   taskMgr_runBkgndTasks();

	   canC_pollRecv(); // see if we have received anything on CAN

	   if (co_reset != 0) {
	   // Upon receiving CAN 0x2036.3, we set do_reset to a non-zero
	   //   value then we decrement it each time through the main loop and
	   //   RESET the processor after it goes to 0.  Hopefully this
	   //   gives us enough time to reply to the PC before we reset.
		  co_reset--;
		  if (co_reset == 0) {
 		     EALLOW;
	         *WDKEY = 666;		// Bad Watchdog key value causes a RESET,
			 EDIS;				//   and vectors to codestart
	      }
	 }

	 if (co_jump_to_bootloader != 0) {
	 // Upon receiving CAN 0x2036.1, we set do_jump_to_bootloader to a non-zero
	 //   value then we decrement it each time through the main loop and
	 //   jump to the bootloader program via a vector after it goes to 0.  Hopefully this
	 //   gives us enough time to reply to the PC before we jump.
		  co_jump_to_bootloader--;
		  if (co_jump_to_bootloader == 0) {
			  // check key fields at 0x003E4002,3 to see if a botloader program
			  // is installed in Flash
			  bootloaderProgVerificationKey = (Uint16*)0x003F4002;
			  key1 = *(bootloaderProgVerificationKey++);
			  key2 = *bootloaderProgVerificationKey;
			  if ((key1 == 0x2001) && (key2 == 0x1776)) {
				  bootloaderProgStartVector = (void(*)(void))0x003f4000; // vector to launch boot program
				  bootloaderProgStartVector();
			  }
	      }
	 }



   }

}

Uint16 main_readPowerGoodSignals(void){
// PG_IO_1, PG_IO_2, PG_PM_1, PG_PM_2 = GPIOB8,9,10,11
//
// As of 7/29/2016 we only have IO module so after >> 8
//   PG_IO_1 is ls bit 0x01 -- 1.2V power to FPGA's
//   PG_IO_2 is bit 0x02    -- 3.3V_core power to FPGAs & CPLD
// 0 in those bits indicates Power is Good.
	Uint16 pgSignals;
	pgSignals = (GpioDataRegs.GPBDAT.all >> 8) & 0xF;
	return pgSignals;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Diagnostic RS232 display reports pg_loopCount, # of loops main() waited to see Power_Good
// see Power_Good. Invoked via diagnostic command interpreter.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void main_DisplayPgLoopCount(void){
	char msgOut[64];
    char *ptr;

    ptr = strU_strcpy(msgOut,"\n\r  pg_loopCount - 0x");
	ptr = hexUtil_binTo4HexAsciiChars(ptr,pg_loopCount);
    ptr = strU_strcpy(ptr,"\n\r\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// The idea here is to take any complex startup activity and provide a framework
// to do it in an orderly fashion, one thing at a time, making sure something
// is complete before starting the next item.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void main_startupTask(void){

	switch(main_startupTaskState){
    case 0: // Check power_good signals from TB3IOM
    	// Loop if necessary waiting for power good
    	// When I installed a diagnostic task to display how many times we looped waiting
    	// on Power_Good, it displayed 0x0000. So the following loop probably isn't necessary,
    	// except that it will detect a power supply failure on TB3IOMC.
    	// Note that pg_loopCount is visible via the diagnostic command interpreter.
    	if((main_readPowerGoodSignals() & 0x3) == 0){
    		// 0 --> power is good
    		main_startupTaskState++; // advance to next item in startup task
    	} else {
    		pg_loopCount++;			// count how many loops before we see power good
        	if (pg_loopCount >= 1000) {
        		// Didn't see power good
        		led_dspLedErrMsg(LED_ERROR_NO_POWER_GOOD_IO); // blink the LEDs
        		main_startupTaskState++; // advance to next item in startup task
        	}
    	}
    	break;
    case 1: // Load FPGA's from Flash, stay in next state until that is complete

    		//Set a task to load FPGA's.
    		//If I set the task to run with no delay, it fails, except when it is running in
    		//Code Composer's Debug mode.  Heuristically I arrived at setting a 1 sec delay
    		//and that appears to solve the problem.  I suspect the problem is related to
    		//use of the TaskMgr and whether or not certain tasks are set to allow round-robin
    		//scheduling.  But I haven't yet figured which.

    		//taskMgr_setTask(TASKNUM_SpiFlashTask); // Access FLASH via SPI, program FPGA's
    		taskMgr_setTaskRoundRobin(TASKNUM_SpiFlashTask, 10); // Access FLASH via SPI, program FPGA's
		                                                         // 10 tenths = delay 1.0 Sec
    		main_startupTaskState++; // advance to next item in startup task
		break;
	case 2: // Stay in this state until we have completed loading FPGAs
		if (frw_GetLoadFpgasAtStartupStatus() == F_LOAD_COMPLETE){
    		main_startupTaskState++; // advance to next item in startup task
		}
    	break;

    case 3: // transmit start up msg out of RS232 port
    	//taskMgr_setTaskRoundRobin(TASKNUM_BgTask_ts3StartUp,0); // transmit start up msg out of RS232 port
    	rs232_DiagPortalMsg();  // write "diagnostic Portal" msg to RS232
    	digio_EncInit(); // Initialize digital I/O Encoder output (writes to FPGA)
    	digio2_Init(); // Initialize Digital Input Machine mapping to Digital Input Pins
    	digio2_Init_Enc_Index_Freq_Div(); // Initialize DIM Enc_Index_Freq_Divisor
    	digio_HallOutInit(); // Init for digital output Hall machine (writes to FPGA)
    	digio2_initDigOutSignalAssignment(); // Init Single ended digital output signal assignment / function
    	digio2_initDiffOutSignalAssignment(); // Init Differential digital output signal assignment / function
    	digio2_initDigOutMode(); // Init Single ended digital output mode
    	digio2_initDigOutRails(); // Init Single ended digital output Rail Voltages
    	digio2_initDigOutLevel(); // Init Single ended digital outputs to LOW state.
    	digio2_initDiffOutLevel(); // Init Differential digital outputs to LOW state.
    	digio2_initDiffOutEnable(); // Init Differential digital outputs to disabled
    	main_startupTaskState++; // advance to next item in startup task
	   break;

    case 4: // start task to configure AD7175 a-to-d for read-continuous mode
    	ain_setSwitchDefaultsForB1();  // set switches to configure B1 "standard" / nothing fancy
    	ain_ad7175_setup_task_init();
		taskMgr_setTaskRoundRobin(TASKNUM_ain_ad7175_setup_task,0);
		main_startupTaskState++; // advance to next item in startup task
		break;

    case 5: // wait here for completion of task to configure AD7175 a-to-d for read-continuous mode
    	if (ain_get_ad7175SetupTaskState() == ANLGIN_AD7175_SETUP_DONE) {
    		main_startupTaskState++; // advance to next item in startup task
    	}
		break;

    case 6: // start task to display speed dial menu
    	taskMgr_setTaskRoundRobin(TASKNUM_DisplayComintSpeedDialList,0);
		main_startupTaskState++; // advance to next item in startup task
		break;

    case 7: // start task to calibrate Analog Inputs
    	ain_ad7175_setup_task_init();
		taskMgr_setTaskRoundRobin(TASKNUM_ain_offsetCalcTask,0);
		main_startupTaskState++; // advance to next item in startup task
		break;

    case 8: // wait here for completion of task to calibrate Analog Inputs
    	if (ain_getAnlginCalState() == ANLGIN_CS_CALIBRATION_DONE) {
    		main_startupTaskState++; // advance to next item in startup task
    	}
		break;

    case 9: //
	   return; // exit without re-launching this task
	 //  break;  // commented out "break" because it generated compiler warning

     default:
    	break;
    }

	taskMgr_setTaskRoundRobin(TASKNUM_main_startupTask,0); // keep re-launching this task
}

//===========================================================================
// No more.
//===========================================================================

