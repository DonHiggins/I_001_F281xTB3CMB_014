// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     F1Int.H
//         support for FPGA1 to interrupt DSP
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File

#include "Rs232Out.H"
#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"

#include "TaskMgr.h"
#include "GpioUtil.H"
#include "F1Int.H"

enum F1I_STATE f1i_state;
Uint16 f1i_BgTask_count;

void f1i_enable_interrupt (void){
    // XINT1 is muxed through PIE into CPU interrupt INT1.4, hence we
	// need to enable a PIE group for it.
    // Enable PIE group 1 interrupt INT1.4 for XINT1
	// (see p 6-18 in spru078b TMS321x281x DSP System Control and Interrupts Reference Guide)
	PieCtrlRegs.PIEIER1.all = M_INT4;

    // Enable CPU INT1
    IER |= M_INT1;
}

// Interrupts that are used in this function are re-mapped to
// ISR functions found within this file.
void f1i_store_int_vectors_in_PIE(void){

	   EALLOW;  // This is needed to write to EALLOW protected registers
	   PieVectTable.XINT1 = &f1i_isr;
	   EDIS;   // This is needed to disable write to EALLOW protected registers

}

void f1i_initialize_interrupt (void){

	// XIntruptRegs.XINT1CR.bit.SELECT = 1;		// SELECT applies to XINT13 pin, not XINT1
	XIntruptRegs.XINT1CR.bit.ENABLE = 1;		// 1-> enable XINT0
	XIntruptRegs.XINT1CR.bit.POLARITY = 1;	// interrupt generated on rising edge

	GpioU_f2iInit(); 						// Setup the GPIO to use IO pin E2 As XNMI_XINT13 interrupt
	f1i_state = F1I_ST_IDLE;

	f1i_BgTask_count = 0;
	f1i_enable_interrupt(); // Above: Enable PIE group 1 interrupt INT1.4 for XINT1
}

interrupt void f1i_isr(void){

	    // Acknowledge interrupt to receive more interrupts from PIE group 1
	    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

	    //Queue a task to run in bacground -- following task just outputs to RS232
		//for testing, this gives us some indication of interrupt activation w/out
		//having to stop the code in debug.
		taskMgr_setTask(TASKNUM_F1Int); // queue this task


	   	switch(f1i_state){
	    case F1I_ST_IDLE: // Shouldn't get here
	        break;

	    case F1I_ST_DUMMY1: // Dummy Task 1, Toggle LED's until we are turned off
	    	   //Need something here to toggle an I/O so we can see the effect of the interrupt
//	    	   GpioDataRegs.GPBTOGGLE.all = 0x000F; // toggle all 4 LS bit of GPIOB
	    	   break;

	    default:
	   	break;
	   }

	   // Re-Enable CPU INT1
	   IER |= M_INT1;
}

// ==========================================================================
//    T A S K S for running features in background
// ==========================================================================

void f1i_BgTask(void){
// Got here after receiving XINT1 interrupt,
// Interrupt handler fired off this background task.
// Interrupt signals from FPGA1 (TB3IOMC)
// As yet (4/26/2016) haven't assigned functionality to this.
// For now, just print out a diagnostic message to the RS232.
	char msgOut[64];
    char *ptr;

    // Just for diagnostics, echo the info to the RS232 output
    f1i_BgTask_count +=1;
    ptr = strU_strcpy(msgOut,"f1i_BgTask: XINT1 interrupt");
    ptr = strU_strcpy(ptr,"\n\r");
    /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}
