// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     SSEnc.c
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "RS232.h"
#include "Rs232Out.h"
#include "Timer0.h"
#include "TaskMgr.h"
#include "stdbool.h"            // needed for bool data types
#include "CPLD.H"
#include "CanOpen.H"
#include "DigIO.H"
#include "TaskMgr.h"
#include "Resolver.H"

Uint16 ssEnc_velocity;                  // 0-> no constant-velocity rotation
                                        // <other> -> increment ssSnc_hi_precis_shaft_angle by ssSnc_velocity each 200uSec

Uint16 ssEnc_HiPrecisShaftAngle;        // 360/65536 degrees

void ssEnc_init(void){
	ssEnc_velocity = 0;
	ssEnc_HiPrecisShaftAngle = 0;
}

//===========================================================================
// Routines run to service CAN commands
//
//===========================================================================

enum CANOPEN_STATUS ssEnc_recvShaftAngleIincreasedPrecision(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host, store in memory at dest = (Uint16*)can_command->datapointer
	// Launch a background task to . . .
	// call appropriate routine to convert shaft angle data to sin/cos DAC outputs.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	// MboxD ignored

	taskMgr_setTaskRoundRobin(TASKNUM_ssEnc_ShaftAngleIincreasedPrecision, 0);

	return CANOPEN_NO_ERR;
}


//===========================================================================
// Constant Velocity Feature
// Routines kicked off by Timer0.
//===========================================================================

void ssEnc_ShaftAngleOutTask(void){
// Runs as a background task
// Take shaft-angle, convert to sin/cos, write output to DACs for Resolver Simulator

	Uint16 sin;
	Uint16 cos;

	res_calcSinCosFromHiPrecisShaftAngle(ssEnc_HiPrecisShaftAngle,&sin,&cos);

    digio_writeDacOutputValue(FPGA1_WRITE_DAC_SSE_COS, cos); // Cos
    digio_writeDacOutputValue(FPGA1_WRITE_DAC_SSE_SIN, sin); // Sin
}

void ssEnc_ConstVelocityTimerRoutine(void){
// Called from a hardware timer routine to periodically . . .
// if ssEnc_velocity != 0, then . . .
// increment ssEnc_HiPrecisShaftAngle and launch a background task
// to take shaft-angle, convert to sin/cos, write output to DACs for SSEnc Simulator.

	if (ssEnc_velocity != 0){
		ssEnc_HiPrecisShaftAngle += ssEnc_velocity; // no problem when HiPrecisShaftAngle wraps
		                                        // from 0xFFFF to 0x0000 or vis versa, it is
		                                        // analogous to shaft angle wrapping from 364 to 0 deg.
		taskMgr_setTaskRoundRobin(TASKNUM_ssEnc_ShaftAngleIincreasedPrecision, 0);
	}

}

