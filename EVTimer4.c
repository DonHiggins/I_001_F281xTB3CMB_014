// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     EVTimer4.C
//
// Timer0 is used for slower, background activities
// EVTimer4 is used for fast, brief activities like bit-banging I2C serial data
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "DSP281x_CpuTimers.h"
#include "stdbool.h"            // needed for bool data types

#include "EVTimer4.H"
#include "TaskMgr.h"
#include "GpioUtil.h"
#include "FpgaTest.h"

enum EVTIMRE4_STATE evtimer4_state = EVT4_ST_IDLE;
bool evtimer4_ext_req_to_turn_off = false;

void evtimer4_turn_off (void){
	evtimer4_ext_req_to_turn_off = true;
}

void evtimer4_turn_on (Uint16 period, Uint16 prescale){
	// prescale can be 1 to 128
	// Interrupt period = (1/75Mhz) x prescale * (period + 1)
	// Interrupt freq   = 1/((1/75Mhz) x prescale * (period + 1))
	// For prescale = 128, 1.7066 uSec per unit of "period"
	// For prescale = 064, 0.8533 uSec per unit of "period"
	// For prescale = 032, 0.4266 uSec per unit of "period"
	// For prescale = 016, 0.2133 uSec per unit of "period"
	// For prescale = 008, 0.1066 uSec per unit of "period"
	// For prescale = 004, 0.0533 uSec per unit of "period"
	// For prescale = 002, 0.0266 uSec per unit of "period"
	// For prescale = 001, 0.0133 uSec per unit of "period"

	Uint16 prescale_bits;

	evtimer4_state = EVT4_ST_DUMMY1;

	if (prescale > 127) {
		prescale_bits = 0x700;
	} else if (prescale > 63) {
		prescale_bits = 0x600;
	} else if (prescale > 31) {
		prescale_bits = 0x500;
	} else if (prescale > 15) {
		prescale_bits = 0x400;
	} else if (prescale > 7) {
		prescale_bits = 0x300;
	} else if (prescale > 3) {
		prescale_bits = 0x200;
	} else if (prescale > 1) {
		prescale_bits = 0x100;
	} else  {
		prescale_bits = 0x000;
	}
    // Initialize EVB Timer 4:
    // Setup Timer 4 Registers (EV B)
    EvbRegs.GPTCONB.all = 0;

    EvbRegs.T4PR = period;       // Timer 4 Period Register
    EvbRegs.T4CMPR = 0x0000;     // Compare Reg -- count down from period to 0x0000

    // Enable Period interrupt bits for GP timer 4
    // Count up, x128, internal clk, enable compare, use own period
    EvbRegs.EVBIMRB.bit.T4PINT = 1;
    EvbRegs.EVBIFRB.bit.T4PINT = 1;

    // Clear the counter for GP timer 4
    EvbRegs.T4CNT = 0x0000;
    EvbRegs.T4CON.all = 0x1042 | prescale_bits;
       // continuous up count mode
       // Input clock prescaler: prescale_bits (x = HSPCLK)
       // enable timer operations using own TENABLE bit
       // Clock source: Internal (i.e., HSPCLK)
       // Timer compare register reload condition: When counter is 0
       // Enable timer compare operation
       // Use own period register (not pigybacking off another Event Module)
}

// Interrupts that are used in this function are re-mapped to
// ISR functions found within this file.
void evtimer4_store_int_vectors_in_PIE(void){

	   EALLOW;  // This is needed to write to EALLOW protected registers
	   PieVectTable.T4PINT = &evtimer4_isr;
	   EDIS;   // This is needed to disable write to EALLOW protected registers
}

void evtimer4_initConfig_n_Start(void){
	// I think we can do without this, and I should take it out
}

void evtimer4_initConfig_n_Start_disabled(void){
    // Initialize EVB Timer 4:
    // Setup Timer 4 Registers (EV B)
    EvbRegs.GPTCONB.all = 0;

    // frequency for our timer4 will be
    //   75MHz HSPCLK
    //   /128 prescale configured in T4CON
    //   /T4PR period register (4096 in example)
    //   = 142.34 Hz (in example)
    // Period for our timer4 will be
    //   13.333 nSec (1/75MHz)
    //   x 128 prescale
    //   xT4PR period register (4096 in example)
    //   = 6.99 mSec (in example)

    // Set the Period for the GP timer 4 to 0x0200;
    EvbRegs.T4PR = 0x1000;       // Timer 4 Period Register
    EvbRegs.T4CMPR = 0x0000;     // Compare Reg

    // Enable Period interrupt bits for GP timer 4
    // Count up, x128, internal clk, enable compare, use own period
    EvbRegs.EVBIMRB.bit.T4PINT = 1;
    EvbRegs.EVBIFRB.bit.T4PINT = 1;

    // Clear the counter for GP timer 4
    EvbRegs.T4CNT = 0x0000;
    EvbRegs.T4CON.all = 0x1742;
       // continuous up count mode
       // Input clock prescaler: x/128 (x = HSPCLK)
       // enable timer operations using own TENABLE bit
       // Clock source: Internal (i.e., HSPCLK)
       // Timer compare register reload condition: When counter is 0
       // Enable timer compare operation
       // Use own period register (not pigybacking off another Event Module)

}


void evtimer4_enable_int(void){
    // Enable PIE group 5 interrupt 1 for T4PINT
    PieCtrlRegs.PIEIER5.all = M_INT1;

    // Enable CPU INT2 for T1PINT, INT3 for T2PINT, INT4 for T3PINT
    // and INT5 for T4PINT:
    IER |= M_INT5;
}

#define TIMER_DISABLE 0xFFBF
interrupt void evtimer4_isr(void)
{

   // Note: To be safe, use a mask value to write to the entire
   // EVBIFRB register.  Writing to one bit will cause a read-modify-write
   // operation that may have the result of writing 1's to clear
   // bits other then those intended.
   EvbRegs.EVBIFRB.all = BIT0;
     // writes 1 to LS bit, resets T4 Period Interrupt Flag

   // Acknowledge interrupt to receive more interrupts from PIE group 5
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;


   	switch(evtimer4_state){
    case EVT4_ST_IDLE: // Shouldn't get here
    case EVT4_ST_TURN_OFF: // finished previous task, turn interrupt off
        	EvbRegs.T4CON.all &= TIMER_DISABLE; // Disable the timer
        	evtimer4_state = EVT4_ST_IDLE;
        	evtimer4_ext_req_to_turn_off = false;
        break;

    case EVT4_ST_DUMMY1: // Dummy Task 1, Toggle LED's until we are turned off
    	   //Need something here to toggle an I/O so we can see the effect of the interrupt
    	   GpioDataRegs.GPBTOGGLE.all = 0x000F; // toggle all 4 LS bit of GPIOB
    	   break;

    default:
   	break;
   }

   // external code, like comint, for ex, sets evtimer4_ext_req_to_turn_off
   // when we se it, we turn off, and reset the flag
   if (evtimer4_ext_req_to_turn_off == true) {
	   evtimer4_state = EVT4_ST_TURN_OFF;
   }

}
