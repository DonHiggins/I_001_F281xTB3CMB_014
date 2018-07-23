// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     DigIO2.C
//
//   Digital Input and Digital Output features
//
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"
#include "CanOpen.h"
#include "CPLD.H"
#include "DigIO2.H"
#include "TaskMgr.h"


Uint16 digio2_EncInMap;
Uint16 digio2_HallInMap;
Uint16 digio2_PwmInMap;

void digio2_Init(void){
	// Initialize RAM variables and also the FPGA Machine.
	// Called from main.c, but not until after FPGA is loaded.
	digio2_EncInMap = 0x0000;
	digio2_HallInMap = 0x0BAD;
	digio2_PwmInMap = 0x0000;

   *CPLD_F2_XA(FPGA2_WRITE_DIGINMACHINE_ENC_MAP) = 0x0000;
   *CPLD_F2_XA(FPGA2_WRITE_DIGINMACHINE_PWM_MAP) = 0x0000;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N   C O M M A N D S   T O   C O N F I G U R E   D I G I T A L   I N   M A C H I N E S
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//	PWM / ENC DUTY CYCLE CONFUSION
//
//	Reading PWM Duty Cycle or "On Time", the Classic test station reports the duration of the OFF portion of the PWM wave.
// 	For compatibility, the TS3 test station also reports the duration of the OFF portion (rather than the ON portion) of
//	the measured PWM wave.
//  This is non-intuitive and can be a source of confusion, though all of our legacy PC client software works with it this way.
//   Here's what you need to keep in mind:  In commands that set the "Duty Cycle" for digital machine output, you must
//	specify the duration of the ON time, whereas when you read the measured "Duty Cycle" from the digital machine input,
//	it gives you the duration of the OFF time of the PWM waveform.


enum CANOPEN_STATUS digio2_recvEncInMap(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received 16-bit data from host, telling which digital input circuits to assign
// as inputs to the digital-input-machines to measure encoder freq, duty cycle, etc.
// Store the value so we can retrieve it if the host requests it,
// And convert it to native TS3 scaling so we can write it to the DAC via the FPGA
// *data is MboxA of received Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
   Uint16 *dest;

   // Store 16-bit input value so it can be returned later
   dest = (Uint16*)can_command->datapointer;
   *dest = *(data+2); // MboxC

   // and write it to the FPGA
   *CPLD_F2_XA(FPGA2_WRITE_DIGINMACHINE_ENC_MAP) = *(data+2); // MboxC

   return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio2_recvHallInMap(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received 16-bit data from host, telling which digital input circuits to assign
// as inputs to the digital-input-machines to measure Hall freq, duty cycle, etc.
// Store the value so we can retrieve it if the host requests it,
// And convert it to native TS3 scaling so we can write it to the DAC via the FPGA
// *data is MboxA of received Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
   Uint16 *dest;

   // Store 16-bit input value so it can be returned later
   dest = (Uint16*)can_command->datapointer;
   *dest = *(data+2); // MboxC

   // and write it to the FPGA . . . but we have no requirement for hall inputs
   // so we just keep it in memory and return it on request in the interest
   // of compatibility with classic test station commands.
   //*CPLD_F2_XA(FPGA2_WRITE_DIGINMACHINE_PWM_MAP) = *(data+2); // MboxC

   return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio2_recvPwmInMap(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received 16-bit data from host, telling which digital input circuits to assign
// as inputs to the digital-input-machines to measure PWM freq, duty cycle, etc.
// Store the value so we can retrieve it if the host requests it,
// And convert it to native TS3 scaling so we can write it to the DAC via the FPGA
// *data is MboxA of received Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
   Uint16 *dest;

   // Store 16-bit input value so it can be returned later
   dest = (Uint16*)can_command->datapointer;
   *dest = *(data+2); // MboxC

   *CPLD_F2_XA(FPGA2_WRITE_DIGINMACHINE_PWM_MAP) = *(data+2); // MboxC

   return CANOPEN_NO_ERR;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N   C O M M A N D S   T O   R E A D   P W M   M E A S U R E M E N T S
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS digio2_sendPwmIn1Period(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received a request from the PC to send PWM period (aka frequency) for
// PWM1 digital input machine.
// Fetch 32 bits from FPGA and return it
// *data is MboxA of transmit Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	union CANOPEN16_32 pwm_period;

	if ((digio2_PwmInMap & 0x1F) == 0) {
	   // if no digital input is configured for the PWM Input machine then return 0
       pwm_period.all = 0L;
	} else {
	   // When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
	   // guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
	   pwm_period.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_PERIOD_MS_16);
	   pwm_period.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_PERIOD_LS_16);
	}

	*(data+2) = pwm_period.words.lsw;    //MboxC
	*(data+3) = pwm_period.words.msw;   //MboxD
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio2_sendPwmIn1OnTime(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received a request from the PC to send PWM on-time (used to calc duty cycle) for
	// PWM1 digital input machine.
	// Fetch 32 bits from FPGA and return it
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
		union CANOPEN16_32 pwm_onTime;

		if ((digio2_PwmInMap & 0x1F) == 0) {
		   // if no digital input is configured for the PWM Input machine then return 0
		   pwm_onTime.all = 0L;
		} else {
		   // When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
		   // guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
		   pwm_onTime.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_ON_TIME_MS_16);
		   pwm_onTime.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_ON_TIME_LS_16);
		}

		*(data+2) = pwm_onTime.words.lsw;    //MboxC
		*(data+3) = pwm_onTime.words.msw;   //MboxD
		return CANOPEN_NO_ERR;
}
enum CANOPEN_STATUS digio2_sendPwmIn2Period(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received a request from the PC to send PWM period (aka frequency) for
	// PWM2 digital input machine.
	// Fetch 32 bits from FPGA and return it
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
		union CANOPEN16_32 pwm_period;

		if ((digio2_PwmInMap & 0x3E0) == 0) {
		   // if no digital input is configured for the PWM Input machine then return 0
	       pwm_period.all = 0L;
		} else {
		   // When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
           // guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
		   pwm_period.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_PERIOD_MS_16);
		   pwm_period.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_PERIOD_LS_16);
		}

		*(data+2) = pwm_period.words.lsw;    //MboxC
		*(data+3) = pwm_period.words.msw;   //MboxD
		return CANOPEN_NO_ERR;
}
enum CANOPEN_STATUS digio2_sendPwmIn2OnTime(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received a request from the PC to send PWM on-time (used to calc duty cycle) for
	// PWM2 digital input machine.
	// Fetch 32 bits from FPGA and return it
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
		union CANOPEN16_32 pwm_onTime;

		if ((digio2_PwmInMap & 0x3E0) == 0) {
		   // if no digital input is configured for the PWM Input machine then return 0
			pwm_onTime.all = 0L;
		} else {
			// When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
			// guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
			pwm_onTime.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_ON_TIME_MS_16);
			pwm_onTime.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_ON_TIME_LS_16);
		}

		*(data+2) = pwm_onTime.words.lsw;    //MboxC
		*(data+3) = pwm_onTime.words.msw;   //MboxD
		return CANOPEN_NO_ERR;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N   C O M M A N D S   T O   R E A D   E N C O D E R   M E A S U R E M E N T S
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS digio2_sendEncAInPeriod(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received a request from the PC to send ENCA period (aka frequency) for
	// ENC digital input machine.
	// Fetch 32 bits from FPGA and return it
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
		union CANOPEN16_32 enca_period;

		if ((digio2_EncInMap & 0x1F) == 0) {
		   // if no digital input is configured for the Enca Input then return 0
	       enca_period.all = 0L;
		} else {
		   // When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
           // guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
		   enca_period.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_PERIOD_MS_16);
		   enca_period.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_PERIOD_LS_16);
		}

		*(data+2) = enca_period.words.lsw;    //MboxC
		*(data+3) = enca_period.words.msw;    //MboxD
		return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio2_sendEncAInOnTime(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received a request from the PC to send ENCA on-time (used to calc duty cycle) for
	// ENCA digital input machine.
	// Fetch 32 bits from FPGA and return it
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
		union CANOPEN16_32 enca_onTime;

		if ((digio2_EncInMap & 0x1F) == 0) {
		   // if no digital input is configured for the ENCA Input machine then return 0
		   enca_onTime.all = 0L;
		} else {
		   // When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
		   // guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
		   enca_onTime.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCA_ON_TIME_MS_16);
		   enca_onTime.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCA_ON_TIME_LS_16);
		}

		*(data+2) = enca_onTime.words.lsw;    //MboxC
		*(data+3) = enca_onTime.words.msw;    //MboxD
		return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio2_sendEncIInPeriod(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received a request from the PC to send ENCI period (aka frequency) for
	// ENCI digital input machine.
	// Fetch 32 bits from FPGA and return it
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
		union CANOPEN16_32 enci_period;

		if ((digio2_EncInMap & 0x7C) == 0) {
		   // if no digital input is configured for the Enci Input then return 0
	       enci_period.all = 0L;
		} else {
		   // When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
           // guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
		   enci_period.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCI_PERIOD_MS_16);
		   enci_period.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCI_PERIOD_LS_16);
		}

		*(data+2) = enci_period.words.lsw;    //MboxC
		*(data+3) = enci_period.words.msw;    //MboxD
		return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio2_sendEncIInOnTime(const struct CAN_COMMAND *can_command,Uint16 *data) {
	return CANOPEN_NO_ERR;
}
enum CANOPEN_STATUS digio2_sendEncInDir(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received a request from the PC to send ENC direction from
	// ENC digital input machine.
	// Fetch 1 bit data from FPGA and return it
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
		Uint16  enc_dir;

		if ((digio2_EncInMap & 0x3FF) == 0) {
		   // if no digital input is configured for the EncA and B then return 0
			enc_dir = 0;
		} else {
		   // When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
           // guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
			enc_dir = (*CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_DIR)) & 0x0001;
		}

		*(data+2) = enc_dir;	//MboxC
		*(data+3) = 0;			//MboxD
		return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio2_sendEncCounts(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received a request from the PC to send ENC counts for
	// ENC digital input machine.
	// Fetch 32 bits from FPGA and return it
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
		union CANOPEN16_32 enc_counts;

		if ((digio2_EncInMap & 0x7C) == 0) {
		   // if no digital input is configured for the Enci Input then return 0
			enc_counts.all = 0L;
		} else {
		   // When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
           // guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
			enc_counts.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_COUNTS_MS_16);
			enc_counts.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_COUNTS_LS_16);
		}

		*(data+2) = enc_counts.words.lsw;    //MboxC
		*(data+3) = enc_counts.words.msw;    //MboxD
		return CANOPEN_NO_ERR;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N   C O M M A N D S   T O   R E A D   P W M   M E A S U R E M E N T S
//   C O M P A T I B L E   W I T H   C L A S S I C   T E S T   S T A T I O N
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS digio2_sendPwmReadingClassic(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received a request from the PC to send PWM period (aka frequency)
//  or OnTime (aka duty cycle) for PWM_1 or PWM_2 digital input machine.
// CAN Subindex determines which of those 4 values we find and return.
// Fetch 32 bits from FPGA and convert it to a classic-compatible 16-bit value and return that.
// *data is MboxA of transmit Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	union CANOPEN16_32 pwm_data;
	Uint16 subindex;
	int m1;
	int m2;
	int result;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;


	// Now we use a switch/case statement to read 32-bit data from proper FPGA address
	switch(subindex){
    case 0x01: // PWM_1 Period
    	pwm_data.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_PERIOD_MS_16);
    	pwm_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_PERIOD_LS_16);
        break;
    case 0x02: // PWM_1 On Time
    	pwm_data.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_ON_TIME_MS_16);
    	pwm_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_ON_TIME_LS_16);
        break;
    case 0x03: // PWM_2 Period
    	pwm_data.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_PERIOD_MS_16);
    	pwm_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_PERIOD_LS_16);
        break;
    case 0x04: // PWM_2 On Time
    	pwm_data.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_ON_TIME_MS_16);
    	pwm_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_ON_TIME_LS_16);
        break;

     default:

    	break;
    }

    // - - - - - - - convert TS3 native 32-bit PWM measurement to Classic NTSTSYS compatible  - - - - - - -
	// <Classic_counts> = <TS3_counts> *(20MHz/75MHz)
	//                  = <TS3_counts> * 0.2666...
	//                  = (<TS3_counts>/4) * 0x1111 *(1/(2^14))

    // REF on 2812 Multiplication:
    // "C/C++ Code Access to the Upper 16 Bits of 16-Bit Multiply"
    // p139, spru514e_TMS320C28x Compiler and Linker Users Guide.pdf

	// Bracket FPGA Data between 4 and 245,756 -- data outside that range doesn't translate to classic test station
	if (pwm_data.all < 4L) {
		pwm_data.all = 4L;
	} else if (pwm_data.all > 245756L){
		pwm_data.all = 245756L;
	}
	pwm_data.all >>= 2; // so result fits in 16-bits

	m1 = pwm_data.words.lsw;
    m2 = (int)0x4444;
    result = ((long)m1 * (long)m2) >> 14; // event log clocked similar operation at 6.4 uS (-5.86 for event_log) -> 0.54 uS


	*(data+2) = result;    //MboxC
	*(data+3) = 0;   //MboxD
	return CANOPEN_NO_ERR;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N   C O M M A N D S   T O   S E N D / R E A D   E N C   M E A S U R E M E N T S
//   C O M P A T I B L E   W I T H   C L A S S I C   T E S T   S T A T I O N
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS digio2_sendEncReadingClassic(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received a request from the PC to send Enc period (aka frequency)
//  or OnTime (aka duty cycle) or Index Pewriod, or Direction for a digital input machine.
// CAN Subindex determines which of those 4 values we find and return.
// Fetch 32 bits from FPGA and convert it to a classic-compatible 16-bit value and return that.
// *data is MboxA of transmit Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	union CANOPEN16_32 enc_data;
	Uint16 subindex;
	int m1;
	int m2;
	int result;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;


	// Now we use a switch/case statement to read 32-bit data from proper FPGA address
	switch(subindex){
    case 0x01: // Encoder Frequency / Period
    	enc_data.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_PERIOD_MS_16);
    	enc_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_PERIOD_LS_16);
        break;
    case 0x02: // Encoder Index Frequency / Period
    	enc_data.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCI_PERIOD_MS_16);
    	enc_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCI_PERIOD_LS_16);
        break;
    case 0x03: // Encoder Direction
    	enc_data.words.msw = 0;
    	enc_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_DIR);
        break;
    case 0x04: // Encoder A On time
    	enc_data.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCA_ON_TIME_MS_16);
    	enc_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCA_ON_TIME_LS_16);
        break;
    case 0x05: // Encoder Counts
    	// even though we only need the LS 16 bits, we have to read the MS 16 bits first
    	// because of our caching algorithm.  If you just read the LS 16 bits, you get
    	// whatever value was cached the last time someone read the MS-16 bits.
    	enc_data.words.msw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_COUNTS_MS_16);
    	enc_data.words.lsw = *CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_COUNTS_LS_16);
        break;

     default:

    	break;
    }

	if (subindex == 3) {
		// don't apply conversion math to direction, which is a 1 or 0
		*(data+2) = enc_data.words.lsw;    //MboxC
		*(data+3) = 0;   //MboxD
		return CANOPEN_NO_ERR;
	}
	if (subindex == 5) {
		// don't apply conversion math to encoder counts
		*(data+2) = enc_data.words.lsw;    //MboxC
		*(data+3) = 0;   //MboxD
		return CANOPEN_NO_ERR;
	}

    // - - - - - - - convert TS3 native 32-bit PWM measurement to Classic NTSTSYS compatible  - - - - - - -
	// <Classic_counts> = <TS3_counts> *(20MHz/75MHz)
	//                  = <TS3_counts> * 0.2666...
	//                  = (<TS3_counts>/4) * 0x1111 *(1/(2^14))

    // REF on 2812 Multiplication:
    // "C/C++ Code Access to the Upper 16 Bits of 16-Bit Multiply"
    // p139, spru514e_TMS320C28x Compiler and Linker Users Guide.pdf

	// Bracket FPGA Data between 4 and 245,756 -- data outside that range doesn't translate to classic test station
	if (enc_data.all < 4L) {
		enc_data.all = 4L;
	} else if (enc_data.all > 245756L){
		enc_data.all = 245756L;
	}
	enc_data.all >>= 2; // so result fits in 16-bits

	m1 = enc_data.words.lsw;
    m2 = (int)0x4444;
    result = ((long)m1 * (long)m2) >> 14; // event log clocked similar operation at 6.4 uS (-5.86 for event_log) -> 0.54 uS

	*(data+2) = result;    //MboxC
	*(data+3) = 0;   //MboxD
	return CANOPEN_NO_ERR;
}

Uint16 digio2_EncIndexDivisor;
void digio2_Init_Enc_Index_Freq_Div(void){
	digio2_EncIndexDivisor = 1;
	*CPLD_F2_XA(FPGA2_WRITE_DIGINMACHINE_ENC_INDEX_FREQ_DIV) = 1;
}

enum CANOPEN_STATUS digio2_recvEncIndexDivisor(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received a request from the PC giving us a 16-bit divisor value used in timing encoder index periods.
// Write this to an FPGA and store it in a static location in case PC wants to read it.
// *data is MboxA of transmit Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;

	// Take the 16-bits received in MboxC and store it
	// in one of our static locations at the dest address we get from the can_command table entry.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC -- save LS16 to static variable

	*CPLD_F2_XA(FPGA2_WRITE_DIGINMACHINE_ENC_INDEX_FREQ_DIV) = *(data+2); // MboxC

	return CANOPEN_NO_ERR;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N   C O M M A N D S   T O   R E A D   H A L L   M E A S U R E M E N T S
//   C O M P A T I B L E   W I T H   C L A S S I C   T E S T   S T A T I O N
//   except there is no requirement for measuring hall input parameters in TS3
//   so we return static "pretty" values, so as not to cause problems with legacy client software.
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS digio2_sendHallReadingClassic(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received a request from the PC to send Hall period (aka frequency)
//  or OnTime (aka duty cycle) or Phase, or Direction for a digital input machine.
//  As there is no requirement for measuring hall input parameters in TS3
//  we return static "pretty" values, so as not to cause problems with legacy client software.
// CAN Subindex determines which of those 4 values we find and return.
// Fetch 32 bits from FPGA and convert it to a classic-compatible 16-bit value and return that.
// *data is MboxA of transmit Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 subindex;
	Uint16 return_value;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;


	// Now we use a switch/case statement to return values based on CAN subindex
	switch(subindex){
    case 0x01: // Hall Frequency / Period
    	return_value = 0x01BB; // 45kHz (assuming a 20MHz clock
        break;
    case 0x02: 	// Hall Phase
    	   		// 0 => 60 degrees
    			// 1 => 120 Degrees
    	return_value = 0;
        break;
    case 0x03:	//Hall Direction
    			// 1 => A  Leads B
    			// 0 => B  Leads A
    	return_value = 0;
        break;
    case 0x04: // Hall A On time
    	return_value = 0x00E6;  // 50% duty cycle, given 0x1BB value for period / frequency
        break;

     default:

    	break;
    }

	*(data+2) = return_value;    //MboxC
    *(data+3) = 0;   //MboxD
    return CANOPEN_NO_ERR;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N   C O M M A N D S   T O   S E T   A N D   R E A D   I / O   P A R A M E T E R S
//   T H A T   W E R E   W R I T E - O N L Y  I N   C L A S S I C   T E S T   S T A T I O N
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Uint16 digio2_DigOutSignalAssignment[6]; // Level, PWM, ENC, etc signal assigned to Single ended Digital Outputs

enum CANOPEN_STATUS digio2_recvDigOutSignalAssignment(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, assigning a signal (Level, PWM, ENC, etc) to each
	// of 4 single ended digital outputs in a "bank".
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to the appropriate address in one of the FPGA's.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 subindex;
	Uint16 *dest;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	// Store the received data value in a static location where it can be retrieved
	// by the PC at a later time, to verify what value was written.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC

	// Now we use a switch/case statement to write reveived data value to FPGA address based on CAN subindex
	switch(subindex){
    case 0x01:
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_A_FUNCT) = *(data+2); // MboxC
        break;
    case 0x02:
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_B_FUNCT) = *(data+2); // MboxC
        break;
    case 0x03:	//Hall Direction
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_C_FUNCT) = *(data+2); // MboxC
        break;
    case 0x04: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_D_FUNCT) = *(data+2); // MboxC
        break;
    case 0x05: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_E_FUNCT) = *(data+2); // MboxC
        break;
    case 0x06: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_F_FUNCT) = *(data+2); // MboxC
        break;

     default:

    	break;
    }

	return CANOPEN_NO_ERR;
}

void digio2_initDigOutSignalAssignment(void){
	// Initialize Signal Assignment (Level, PWM, Enc, etc. for all Single Ended Digital Outputs.
	// Write value to FPGA, also store in static array, to return to PC.
	Uint16 i;

	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_A_FUNCT) = 0; // Level 1/0
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_B_FUNCT) = 0; // Level 1/0
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_C_FUNCT) = 0; // Level 1/0
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_D_FUNCT) = 0; // Level 1/0
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_E_FUNCT) = 0; // Level 1/0
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_F_FUNCT) = 0; // Level 1/0

	for (i=0;i<6;i++){
		digio2_DigOutSignalAssignment[i] = 0;
	}
}

Uint16 digio2_DigOutMode[6]; // Push Pull, Open Collector, etc mode assigned to Single ended Digital Outputs

enum CANOPEN_STATUS digio2_recvDigOutMode(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, assigning a mode (Push Pull, Open Collector, etc) to each
	// of 4 single ended digital outputs in a "bank".
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to the appropriate address in one of the FPGA's.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 subindex;
	Uint16 *dest;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	// Store the received data value in a static location where it can be retrieved
	// by the PC at a later time, to verify what value was written.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC

	// Now we use a switch/case statement to write reveived data value to FPGA address based on CAN subindex
	switch(subindex){
    case 0x01:
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_A_MODE) = *(data+2); // MboxC
        break;
    case 0x02:
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_B_MODE) = *(data+2); // MboxC
        break;
    case 0x03:	//Hall Direction
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_C_MODE) = *(data+2); // MboxC
        break;
    case 0x04: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_D_MODE) = *(data+2); // MboxC
        break;
    case 0x05: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_E_MODE) = *(data+2); // MboxC
        break;
    case 0x06: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_F_MODE) = *(data+2); // MboxC
        break;

     default:

    	break;
    }

	return CANOPEN_NO_ERR;
}

void digio2_initDigOutMode(void){
	// Initialize Signal Assignment (Level, PWM, Enc, etc. for all Single Ended Digital Outputs.
	// Write value to FPGA, also store in static array, to return to PC.
	Uint16 i;

	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_A_MODE) = 0; // push pull
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_B_MODE) = 0; // push pull
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_C_MODE) = 0; // push pull
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_D_MODE) = 0; // push pull
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_E_MODE) = 0; // push pull
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_F_MODE) = 0; // push pull

	for (i=0;i<6;i++){
		digio2_DigOutMode[i] = 0;
	}
}

Uint16 digio2_DiffOutSignalAssignment[2]; // Level, PWM, ENC, etc signal assigned to Differential Digital Outputs

enum CANOPEN_STATUS digio2_recvDiffOutSignalAssignment(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, assigning a signal (Level, PWM, ENC, etc) to each
	// of 4 differential digital outputs in a "bank".
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to the appropriate address in one of the FPGA's.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 subindex;
	Uint16 *dest;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	// Store the received data value in a static location where it can be retrieved
	// by the PC at a later time, to verify what value was written.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC

	// Now we use a switch/case statement to write reveived data value to FPGA address based on CAN subindex
	switch(subindex){
    case 0x01:
    	*CPLD_F1_XA(FPGA1_WRITE_DIFFOUT_1234_FUNCT) = *(data+2); // MboxC
        break;
    case 0x02:
    	*CPLD_F1_XA(FPGA1_WRITE_DIFFOUT_5678_FUNCT) = *(data+2); // MboxC
        break;

     default:

    	break;
    }

	return CANOPEN_NO_ERR;
}

void digio2_initDiffOutSignalAssignment(void){
	// Initialize Signal Assignment (Level, PWM, Enc, etc. for all Single Ended Digital Outputs.
	// Write value to FPGA, also store in static array, to return to PC.
	Uint16 i;

	*CPLD_F1_XA(FPGA1_WRITE_DIFFOUT_1234_FUNCT) = 0; // Level 1/0
	*CPLD_F1_XA(FPGA1_WRITE_DIFFOUT_5678_FUNCT) = 0; // Level 1/0

	for (i=0;i<2;i++){
		digio2_DiffOutSignalAssignment[i] = 0;
	}
}

Uint16 digio2_DigOutRails[6]; // In each word, two 2-bit values encode pos & neg rail voltage
                              // for one of 6 banks of single ended digital outputs

enum CANOPEN_STATUS digio2_recvDigOutRails(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, assigning pos and neg voltage rails to all 4
	// single ended digital outputs in a single "bank".
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to the appropriate address in one of the FPGA's.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 subindex;
	Uint16 *dest;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	// Store the received data value in a static location where it can be retrieved
	// by the PC at a later time, to verify what value was written.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC

	// Now we use a switch/case statement to write reveived data value to FPGA address based on CAN subindex
	switch(subindex){
    case 0x01:
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_A_RAILS) = *(data+2); // MboxC
        break;
    case 0x02:
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_B_RAILS) = *(data+2); // MboxC
        break;
    case 0x03:	//Hall Direction
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_C_RAILS) = *(data+2); // MboxC
        break;
    case 0x04: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_D_RAILS) = *(data+2); // MboxC
        break;
    case 0x05: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_E_RAILS) = *(data+2); // MboxC
        break;
    case 0x06: // Hall A On time
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_F_RAILS) = *(data+2); // MboxC
        break;

     default:

    	break;
    }

	return CANOPEN_NO_ERR;
}

void digio2_initDigOutRails(void){
	// Initialize Signal Assignment (Level, PWM, Enc, etc. for all Single Ended Digital Outputs.
	// Write value to FPGA, also store in static array, to return to PC.
	Uint16 i;

	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_A_RAILS) = 0x000F; // PosRail=None, NegRail=None
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_B_RAILS) = 0x000F; // PosRail=None, NegRail=None
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_C_RAILS) = 0x000F; // PosRail=None, NegRail=None
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_D_RAILS) = 0x000F; // PosRail=None, NegRail=None
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_E_RAILS) = 0x000F; // PosRail=None, NegRail=None
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_F_RAILS) = 0x000F; // PosRail=None, NegRail=None

	for (i=0;i<6;i++){
		digio2_DigOutRails[i] = 0x000F; // PosRail=None, NegRail=None
	}
}

Uint16 digio2_DigOutLevel[3]; // In each word, 8 bits, 1/0, Hi/Low for each of 8 differential digital outputs
                              // for each of 8 single ended digital outputs (2 banks)

enum CANOPEN_STATUS digio2_recvDigOutLevel(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, assigning Hi/Low Level
	// to 8 Single ended digital outputs = 2 banks.
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to the appropriate address in one of the FPGA's.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 subindex;
	Uint16 *dest;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	// Store the received data value in a static location where it can be retrieved
	// by the PC at a later time, to verify what value was written.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC

	// Now we use a switch/case statement to write reveived data value to FPGA address based on CAN subindex
	switch(subindex){
    case 0x01:
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_AB_STATE) = *(data+2); // MboxC
        break;
    case 0x02:
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_CD_STATE) = *(data+2); // MboxC
        break;
    case 0x03:	//Hall Direction
    	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_EF_STATE) = *(data+2); // MboxC
        break;

     default:

    	break;
    }

	return CANOPEN_NO_ERR;
}

void digio2_initDigOutLevel(void){
	// Initialize Signal Level (Hi / Low) for all Single Ended Digital Outputs.
	// Write value to FPGA, also store in static array, to return to PC.
	Uint16 i;

	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_AB_STATE) = 0x0000; // Low
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_CD_STATE) = 0x0000; // Low
	*CPLD_F1_XA(FPGA1_WRITE_DIGOUT_BANK_EF_STATE) = 0x0000; // Low

	for (i=0;i<3;i++){
		digio2_DigOutLevel[i] = 0x0000; // Low
	}
}

Uint16 digio2_DiffOutLevel; // 8 bits, 1/0, Hi/Low for each of 8 differential digital outputs

enum CANOPEN_STATUS digio2_recvDiffOutLevel(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, assigning Hi/Low Level
	// to 8 Differential digital outputs = 2 banks.
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to the appropriate address in one of the FPGA's.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;

	// Store the received data value in a static location where it can be retrieved
	// by the PC at a later time, to verify what value was written.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC

	// Now we use a switch/case statement to write reveived data value to FPGA address based on CAN subindex
	*CPLD_F1_XA(FPGA1_WRITE_DIFFOUT_STATE) = *(data+2); // MboxC

	return CANOPEN_NO_ERR;
}

void digio2_initDiffOutLevel(void){
	// Initialize Signal Level (Hi / Low) for all Differential Digital Outputs.
	// Write value to FPGA, also store in static array, to return to PC.

	*CPLD_F1_XA(FPGA1_WRITE_DIFFOUT_STATE) = 0x0000; // Low
	digio2_DiffOutLevel = 0x0000; // Low
}

Uint16 digio2_DiffOutEnable; // 2 bits, one for each back of 4 Differential Digital Outputs
                             // indicating whether the outputs are enabled or not.

enum CANOPEN_STATUS digio2_recvDiffOutEnable(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, indicating whether or not each of the 2
	// banks of 4 Differential digital outputs is enabled or not.
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to the appropriate address in one of the FPGA's.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;

	// Store the received data value in a static location where it can be retrieved
	// by the PC at a later time, to verify what value was written.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC

	*CPLD_F1_XA(FPGA1_WRITE_DIFFOUT_ENABLE) = *(data+2); // MboxC

	return CANOPEN_NO_ERR;
}

void digio2_initDiffOutEnable(void){
	// Initialize Signal Level (Hi / Low) for all Differential Digital Outputs.
	// Write value to FPGA, also store in static array, to return to PC.

	*CPLD_F1_XA(FPGA1_WRITE_DIFFOUT_ENABLE) = 0x0000; // Low
	digio2_DiffOutEnable = 0x0000; // Disabled
}

