// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Timer0.C
//
// Timer0 is used for slower, background activities
// EVTimer4 is used for fast, brief activities like bit-banging I2C serial data
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "DSP281x_CpuTimers.h"
#include "stdbool.h"            // needed for bool data types

#include "Timer0.H"
#include "TaskMgr.h"
#include "GpioUtil.h"
#include "Xintf.h"
#include "FpgaTest.h"
#include "CPLD.H"
#include "Resolver.H"
#include "SSEnc.H"
#include "SCI2.H"
#include "LED.H"
#include "LimitChk.H"


void InitCpuTimers(void);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        P E R I O D   F O R   T I M E R   0 (relocated to .H file)
//
//#define TIMER_0_PERIOD_IN_USEC 200
//#define T_0_COUNTS_TO_1_MILISEC 5
//#define T_0_COUNTS_TO_1_TENTH_SEC 500
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

struct CPUTIMER_VARS CpuTimer0;
// When using DSP BIOS & other RTOS, comment out CPU Timer 2 code.
struct CPUTIMER_VARS CpuTimer1;
struct CPUTIMER_VARS CpuTimer2;


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//  T I M E R   0   S E T U P   H A R D W A R E   A N D   I N T E R R U P T S
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
interrupt void timer0_isr(void){
// Timer0 Interrupt Service Routine
   CpuTimer0.InterruptCount++;

   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

   taskMgr_setTaskRoundRobin(TASKNUM_timer0_task,0);	// run a background task
   	   	   	   	   	   	   	   	   	   	   	   	 		// period is: TIMER_0_PERIOD_IN_USEC
}

void timer0_store_int_vectors_in_PIE(void){
// Timer0 Initialization Routines
// Update entry in PIE interrupt vector table to point to ISR in this file
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.TINT0 = &timer0_isr;
	EDIS;    // This is needed to disable write to EALLOW protected registers
}

void timer0_initConfig_n_Start(void){
// This function lifted from from DSP281x_CpuTimers.c
// Call other functions to set timer period, and to start it running
	InitCpuTimers();   // For this example, only initialize the Cpu Timers

	// Configure CPU-Timer 0 to interrupt every second:
	//   The CPU "Freq" is entered as "MHz", the Timer0 period in "uSeconds"
	// -------------------------------------------
	//   TB3CMB: Timer0 ==> 0.1 Sec              -
	// -------------------------------------------
//	ConfigCpuTimer(&CpuTimer0, 150, 100000); // 150MHz CPU Freq, 0.1 second Period (in uSeconds)
	ConfigCpuTimer(&CpuTimer0, 150, TIMER_0_PERIOD_IN_USEC);    // 150MHz CPU Freq, Period (in uSeconds)
	StartCpuTimer0(); // #define for a bit-set in DSP281x_CpuTimers.h
}

void timer0_init_03(void){
// Called from initialization code in main.c
// Enable CPU INT1 which is connected to CPU-Timer 0:
  IER |= M_INT1;

   // Enable TINT0 in the PIE: Group 1 interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
}

void InitCpuTimers(void){
// InitCpuTimers: --- from DSP281x_CpuTimers.c

	// CPU Timer 0
    // Initialize address pointers to respective timer registers:
    CpuTimer0.RegsAddr = &CpuTimer0Regs;
    // Initialize timer period to maximum:
    CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
    CpuTimer0Regs.TPR.all  = 0;
    CpuTimer0Regs.TPRH.all = 0;
    // Make sure timer is stopped:
    CpuTimer0Regs.TCR.bit.TSS = 1;
    // Reload all counter register with period value:
    CpuTimer0Regs.TCR.bit.TRB = 1;
    // Reset interrupt counters:
    CpuTimer0.InterruptCount = 0;


// CpuTimer2 is reserved for DSP BIOS & other RTOS
// Do not use this timer if you ever plan on integrating
// DSP-BIOS or another realtime OS.

    // Initialize address pointers to respective timer registers:
    CpuTimer1.RegsAddr = &CpuTimer1Regs;
    CpuTimer2.RegsAddr = &CpuTimer2Regs;
    // Initialize timer period to maximum:
    CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;
    CpuTimer2Regs.PRD.all  = 0xFFFFFFFF;
    // Make sure timers are stopped:
    CpuTimer1Regs.TCR.bit.TSS = 1;
    CpuTimer2Regs.TCR.bit.TSS = 1;
    // Reload all counter register with period value:
    CpuTimer1Regs.TCR.bit.TRB = 1;
    CpuTimer2Regs.TCR.bit.TRB = 1;
    // Reset interrupt counters:
    CpuTimer1.InterruptCount = 0;
    CpuTimer2.InterruptCount = 0;

}

void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period){
// ConfigCpuTimer: --- from DSP281x_CpuTimers.c
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// This function initializes the selected timer to the period specified
// by the "Freq" and "Period" parameters. The "Freq" is entered as "MHz"
// and the period in "uSeconds". The timer is held in the stopped state
// after configuration.

    Uint32  temp;

    // Initialize timer period:
    Timer->CPUFreqInMHz = Freq;
    Timer->PeriodInUSec = Period;
    temp = (long) (Freq * Period);
    Timer->RegsAddr->PRD.all = temp;

    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    Timer->RegsAddr->TPR.all  = 0;
    Timer->RegsAddr->TPRH.all  = 0;

    // Initialize timer control register:
    Timer->RegsAddr->TCR.bit.TSS = 1;      // 1 = Stop timer, 0 = Start/Restart Timer
    Timer->RegsAddr->TCR.bit.TRB = 1;      // 1 = reload timer
    Timer->RegsAddr->TCR.bit.SOFT = 1;
    Timer->RegsAddr->TCR.bit.FREE = 1;     // Timer Free Run
    Timer->RegsAddr->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt

    // Reset interrupt counter:
    Timer->InterruptCount = 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   B A C K G R O U N D   T A S K S   L A U N C H E D   V I A   T A S K   M G R
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Uint16 timer0_count_Tenths;
Uint16 timer0_count_T0_periods_to_1_Tenth;
Uint16 timer0_count_T0_periods_to_1_MiliSec;
Uint32 timer0_SystemMiliSecCount; // count of milisec since power-up
void timer0_task(void){
	//  timer0_task: background task runs via TaskMgr.
	//  Launched each Timer0 interrupt, see TIMER_0_PERIOD_IN_USEC
	//  Handles "constant velocity" functions for resolver and SSEnc outputs
	//  Counts up to 1/10th sec and launches TASKNUM_timer0_tenthOfSecTask

	res_ConstVelocityTimerRoutine(); // act if resolver const velocity != 0
	ssEnc_ConstVelocityTimerRoutine(); // act if SSEnc const velocity != 0
	sci2_rx_tx(); // Monitor TB3IOM RS232/RS485 port.

	if (++timer0_count_T0_periods_to_1_MiliSec >= T_0_COUNTS_TO_1_MILISEC) {
		timer0_count_T0_periods_to_1_MiliSec = 0;
		taskMgr_setTaskRoundRobin(TASKNUM_timer0_miliSecTask, 0);
	}

	if (++timer0_count_T0_periods_to_1_Tenth >= T_0_COUNTS_TO_1_TENTH_SEC) {
		timer0_count_T0_periods_to_1_Tenth = 0;
		taskMgr_setTaskRoundRobin(TASKNUM_timer0_tenthOfSecTask, 0);
	}

	// run limit check measurements and comparisons every 200
	// (Classic test station used to do it every 250 uSec)
	limChkBackgroundMeasurements();

}

void timer0_tenthOfSecTask(void){
// Runs every 0.1 sec

	taskMgr_ageTaskDelays();

	if (++timer0_count_Tenths < 5) {
		return;
	}
	timer0_count_Tenths = 0;

	// - - - - - - - - - - - - - - - - - - - -
	// Run following operations every 0.5 sec

	led_synchronizedSlowHeartbeat();
	led_manageDspLEDsUnderTimer0();			// DSP LEDs
	led_manageCpldIoLEDsUnderTimer0();		// CPLD LEDs
	led_manageFpgaLedsUnderTimer0();		// FPGA LEDs

	// Xintf external bus test
	xintf_testUnderTimer0();
	// fpgaT Testing the FPGA
	fpgaT_testUnderTimer0();
}

void timer0_miliSecTask(void){
// Runs 1 milisec
// Update a milisecTimer value

	timer0_SystemMiliSecCount++;

	// every milisec we call the limit check state machine code
	limChkStateMachine();
}

void timer0_task_init(void){
	// Called from Main.c: Step 5. User specific code initializations
	// Initialize some variables before the first time we run the timer0 tasks

	// LED's
	led_DspLedInit();
	led_FpgaLedInit();

	// Xintf external bus test
	xintf_initTest();
	fpgaT_initTest();

	// counters
	timer0_count_Tenths = 0;
	timer0_count_T0_periods_to_1_Tenth = 0;
	timer0_count_T0_periods_to_1_MiliSec = 0;
	timer0_SystemMiliSecCount = 0L;
}

Uint32 timer0_fetchSystemMiliSecCount(void){
	// safe way for Limit Check to access timer0_SystemMiliSecCount
	return timer0_SystemMiliSecCount;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//       L O G . C   U T I L I T Y
//
//  Return timestamp info used in log routines
//    Note: we disable timer0 interrupt stop the timer before reading
//    counter values to RAM, in the hope that the two counters are
//    then synchronized -- in other words, 1 counter doesn't increment
//    or roll over while we are reading the other.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Uint32 timer0_synchronized_count_value;
Uint32 timer0_synchronized_reg_value;

Uint32 timer0_interrupt_count_value(){

	StopCpuTimer0();
	PieCtrlRegs.PIEIER1.bit.INTx7 = 0;	// Disable TINT0 in the PIE: Group 1 interrupt 7
	timer0_synchronized_reg_value = CpuTimer0Regs.TIM.all;
	timer0_synchronized_count_value = CpuTimer0.InterruptCount;
	StartCpuTimer0();
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;	// Enable TINT0 in the PIE: Group 1 interrupt 7
	return timer0_synchronized_count_value;
}
Uint32 timer0_count_reg_value(){
	return timer0_synchronized_reg_value;

}

