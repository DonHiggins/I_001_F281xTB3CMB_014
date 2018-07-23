// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     DigIO.C
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
#include "DigIO.H"
#include "TaskMgr.h"

// store values PC has sent in CAN commands
Uint16 digio_DacComparatorValuesClassic[4];
Uint16 digio_DacAnlgOutValuesClassic[8];
Uint16 digio_nativeDacVal[16]; // values received from PC, written to DAC's

void digio_initDacComparatorValuesClassic(){
// When FPGA starts up the machine controlling the DAC's, all
// DAC output voltages are commanded to native dac_value 0x800.
// Equivalent to dac_value scaled for Classic NTSTSYS 0xD58.
// We load the digio_DacComparatorValuesClassic[4] table with
// this value and we report it th the PC if It asks what voltage
// is set in the comparators.
	Uint16 i;

	for (i=0;i<4;i++){
		digio_DacComparatorValuesClassic[i] = 0xD58;
	}

}

enum CANOPEN_STATUS digio_recvComparitorClassic(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, telling DAC output value setting for Digital Inputput
	// comparator Threshold Voltage.
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to native TS3 scaling so we can write it to the DAC via the FPGA
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	Uint16 i;
	int dac_val_int;
	int m2;
	int result;

	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	dac_val_int = (int)(*(data+2)); // MboxC

    // - - - - - - - convert input NTSTSYS scaled DAC value to native TS3 scaling - - - - - - -
	// <TS3_counts> = (<ntstsys_counts> - 2048) *(245.2 / 163.84)
	// <TS3_counts> = (<ntstsys_counts> - 0x0800) *(0x5FC8 / 0x4000)

    // REF on 2812 Multiplication:
    // "C/C++ Code Access to the Upper 16 Bits of 16-Bit Multiply"
    // p139, spru514e_TMS320C28x Compiler and Linker Users Guide.pdf
    dac_val_int = dac_val_int - 0x0800;
    m2 = (int)0x5FC8;
    result = ((long)dac_val_int * (long)m2) >> 14; // event log clocked this at 6.4 uS (-5.86 for event_log) -> 0.54 uS

    // write native-TS3-scaled DAC value to FPGA
    i = dest - digio_DacComparatorValuesClassic; // index into digio_DacComparatorValuesClassic[4]
		                                         // eg. which of 4 Comparator Dac's: 0,1,2,3 ?
	*(CPLD_F1_XA(FPGA1_WRITE_DAC_DIG_IN_A) + i) = (Uint16)result; // write to DAC via FPGA1

	// Save native DAC value in static RAM in case PC wants to retrieve it later
	digio_nativeDacVal[8+i] = result;

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio_send16DigitalInputs(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received a request from the PC to send 1/0 state of all 16 digital inputs
// Fetch it from FPGA and invert it to make it compatible with "Classic" NTSTSYS
// *data is MboxA of transmit Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *src;
	Uint16 digitalInputs;
	src = (Uint16*)can_command->datapointer; // CPLD_F2_XA(FPGA2_READ_DIG_IN)
	digitalInputs = *src; // fetch from FPGA
	*(data+2) = digitalInputs^0xFFFF; // Invert value into MboxC
	*(data+3) = 0;   //MboxD
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio_receiveNativeDacVal(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, native mode, to write to one of our DAC's.
	// Store the value so we can retrieve it if the host requests it,
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;
	Uint16 subindex;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2);	// MboxC -- write 16-bit data from CAN command
						// into element of digio_nativeDacVal[16]

	// Now we use a switch/case statement to write DAC value to proper FPGA address
	switch(subindex){
    case 0x01: // received digio_Enc1OutFreq32
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_1) = *dest;
        break;
    case 0x02: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_2) = *dest;
        break;
    case 0x03: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_3) = *dest;
        break;
    case 0x04: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_4) = *dest;
        break;
    case 0x05: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_5) = *dest;
        break;
    case 0x06: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_6) = *dest;
        break;
    case 0x07: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_7) = *dest;
        break;
    case 0x08: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_8) = *dest;
        break;
    case 0x09: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_DIG_IN_A) = *dest;
        break;
    case 0x0A: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_DIG_IN_B) = *dest;
        break;
    case 0x0B: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_DIG_IN_C) = *dest;
        break;
    case 0x0C: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_DIG_IN_D) = *dest;
        break;
    case 0x0D: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_SSE_COS) = *dest;
        break;
    case 0x0E: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_SSE_SIN) = *dest;
        break;
    case 0x0F: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_RES_COS) = *dest;
        break;
    case 0x10: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_DAC_RES_SIN) = *dest;
    	break;

     default:

    	break;
    }
	return CANOPEN_NO_ERR;
}


enum CANOPEN_STATUS digio_recvAnlgOutClassic(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, telling Analog Output level (one of 8 Anlg Out DAC's)
	// Store the value so we can retrieve it if the host requests it,
	// And convert it to native TS3 scaling so we can write it to the DAC via the FPGA
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	Uint16 i;
	Uint16 result;

	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC

	// Classic Analog Out Command should be 0x0000 to 0x0FFF
	// Check for proper range
	if (*dest & 0x8000) {
		*dest = 0x000;             // assume PC calculation underflowed
	} else if (*dest > 0xFFF) {
		*dest =  0x0FFF;       // assume PC overflowed
	}

    // - - - - - - - convert input NTSTSYS DAC value to native TS3 scaling - - - - - - -
	// following algorithm translates classic req 0x800 (0V) to native DAC value 0x800 (0V)
	// "You ask for 0 you get 0"
	// and allows classic range 0x000 -> 0xFFF to cover full native DAC range 0x000 -> 0xFFF.
	// non-linearities turn out to be 6mV, occurring at voltage extremes +/-12.5v
	// CAVIET: it turns out 0x800 (native) does not uniformly produce 0 V across all 8 Analog Output
	// channels, but it is a good compromise as "close to 0v."
	// <TS3_counts> = ((0x1000 - <ntstsys_counts>) // multiplication mot required
	                                               // 0x0001 -> 0x0000
	                                               // 0x1000 -> 0x0FFF
    result = (0x1000 - *dest);
    if (result == 0x0001) {
    	result = 0x0000;
    } else if (result == 0x1000) {
    	result = 0x0FFF;
    }

    // write native-TS3-scaled DAC value to FPGA
    i = dest - digio_DacAnlgOutValuesClassic;    // index into digio_DacAnlgOutValuesClassic[8]
		                                         // eg. which of 8 Anlg Out Dac's: 0,1,2,3,4,5,6,7 ?
	*(CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_1) + i) = result; // write to DAC via FPGA1

	// Save native DAC value in static RAM in case PC wants to retrieve it later
	digio_nativeDacVal[i] = result;

	return CANOPEN_NO_ERR;
}

void digio_writeDacOutputValue(Uint16 dac_fpga_address, Uint16 dac_output_value){
// Called from other modules to write an output value (0x000 - 0xFFF) to
// one of our 16 DAC's (dac_fpga_address: mnenomic values in CPLD.H, eg: "FPGA1_WRITE_DAC_RES_COS")

	*CPLD_F1_XA(dac_fpga_address)  = dac_output_value; // write to DAC via FPGA1


}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   P W M   D I G I T A L   O U T P U T -- N A T I V E   M O D E
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Uint32 digio_PwmOutputFreq32;
Uint32 digio_PwmOutputDutyCycl32;
Uint16 digio_PwmOutputFreqClassic16;
Uint16 digio_PwmOutputDutyCyclClassic16;
Uint16 digio_PwmOutputFreq16TaskState;

enum CANOPEN_STATUS digio_recvPwmOutputFreq32(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 32-bit data from host, representing frequency for the Dig Out PWM
	// signal, encoded as # of 75MHz clock pulses in 1 period of PWM output.
	// Store the value so we can retrieve it if the host requests it,
	// And write it to the Dig Out PWM machine in FPGA1
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;

	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	*CPLD_F1_XA(FPGA1_WRITE_PWM_FREQ_LS16) = *dest++;
	*dest = *(data+3);  //MboxD
	*CPLD_F1_XA(FPGA1_WRITE_PWM_FREQ_MS16) = *dest;

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio_recvPwmOutputDutyCyc32(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 32-bit data from host, representing duty cycle for the Dig Out PWM
	// signal, encoded as # of 75MHz clock pulses in the low phase of 1 period of PWM output.
	// Store the value so we can retrieve it if the host requests it,
	// And write it to the Dig Out PWM machine in FPGA1
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;

	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	*CPLD_F1_XA(FPGA1_WRITE_PWM_DTY_CYCL_LS16) = *dest++;
	*dest = *(data+3);  //MboxD
	*CPLD_F1_XA(FPGA1_WRITE_PWM_DTY_CYCL_MS16) = *dest;


	return CANOPEN_NO_ERR;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   P W M   D I G I T A L   O U T P U T  --
//      C O M P A T I B L E   W/  C L A S S I C   T E S T   S T A T I O N
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS digio_recvPwmOutputFreqClassic16(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit data from host, representing frequency for the Dig Out PWM
	// signal, encoded as # of 20MHz clock pulses in 1 period of PWM output.
	// Store the value so we can retrieve it if the host requests it,
	// And write it to the Dig Out PWM machine in FPGA1
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;

	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC, store value from PC so PC can read it back later

	// Set background task to write freq & duty cycle to FPGA
	digio_PwmOutputFreq16TaskState = 0;
	taskMgr_setTaskRoundRobin(TASKNUM_DigIO_Pwm_Freq_Out16, 0);

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS digio_recvPwmOutputDutyCycClassic16(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 32-bit data from host, representing duty cycle for the Dig Out PWM
	// signal, encoded as # of 75MHz clock pulses in the low phase of 1 period of PWM output.
	// Store the value so we can retrieve it if the host requests it,
	// And write it to the Dig Out PWM machine in FPGA1
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;

	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC, store value from PC so PC can read it back later

	// Set background task to write freq & duty cycle to FPGA
	digio_PwmOutputFreq16TaskState = 0;
	taskMgr_setTaskRoundRobin(TASKNUM_DigIO_Pwm_Freq_Out16, 0);

	return CANOPEN_NO_ERR;
}
void digio_PwmOutputFreq16Task(void) {

	Uint16 m1, m2;
	union CANOPEN16_32 result;

	// For compatibility with classic test station, multiply this value by
	// 3.75 since new clock master clock freq is 75MHz = 3.75 x Classic test station
	// clock rate of 20MHz.
	// Note: 3.75 = 15 >> 2;

	// We do this in 2 passes. to spread the out the 32-bit multiplication activity in time

	if (digio_PwmOutputFreq16TaskState == 0) {
	   // 1st pass, set frequency (period)
	   m1 = digio_PwmOutputFreqClassic16; // set up Uint16 values for multiplication
	   m2 = 15;

	   if (m1 < 0x3FFF) {
		   m1 += 2;     // Heuristically, found this makes highest freqs more
		                // compatible with Classic test ststion. (Low Freq doesn't matter.)
	   }
	   result.all = ((unsigned long) m1 * (unsigned long) m2) >> 2;

	   *CPLD_F1_XA(FPGA1_WRITE_PWM_FREQ_LS16) = result.words.lsw;
	   *CPLD_F1_XA(FPGA1_WRITE_PWM_FREQ_MS16) = (result.words.msw) & 0x3FFF;
	   digio_PwmOutputFreq32 = result.all; // save native 32-bit value

	   // Set background task to run again to write duty cycle to FPGA
	   digio_PwmOutputFreq16TaskState = 1;
	   taskMgr_setTaskRoundRobin(TASKNUM_DigIO_Pwm_Freq_Out16, 0);
	   return; // exit, setting task to run again
    }

	// Now do the same for Duty Cycle
    if (digio_PwmOutputFreq16TaskState == 1) {
 	   // 2nd pass, set Duty Cycle

       // set up Uint16 values for multiplication
	   m1 = (digio_PwmOutputFreqClassic16           // subtract duty cycle from period
			   - digio_PwmOutputDutyCyclClassic16)  // in Classic TS, Duty cycle % is
	           - 1;                                 // % of time output is High.
	   m2 = 15;

	   if (digio_PwmOutputFreqClassic16 < 0x3FFF) {
		   m1 += 1;     // Heuristically, found this makes highest freqs more
		                // compatible with Classic test station. (Low Freq doesn't matter.)
	   }

	   result.all = ((unsigned long) m1 * (unsigned long) m2) >> 2;

	   *CPLD_F1_XA(FPGA1_WRITE_PWM_DTY_CYCL_LS16) = result.words.lsw;
	   *CPLD_F1_XA(FPGA1_WRITE_PWM_DTY_CYCL_MS16) = (result.words.msw) & 0x3FFF;
	   digio_PwmOutputDutyCycl32 = result.all;  // save native 32-bit value

	   digio_PwmOutputFreq16TaskState = 0;
   }  // exit, without running again

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   E N C O D E R   D I G I T A L   O U T P U T  --  N A T I V E   M O D E
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Uint32 digio_Enc1OutFreq32;
Uint32 digio_Enc2OutFreq32;
Uint32 digio_Enc1StopAfterN;
Uint32 digio_Enc2StopAfterN;
Uint32 digio_Enc1OutIndex32;
Uint32 digio_Enc2OutIndex32;
Uint16 digio_Enc1OutDir;
Uint16 digio_Enc2OutDir;
Uint16 digio_Enc1ManualStop;
Uint16 digio_Enc2ManualStop;

void digio_EncInit(void){
	// Initialize RAM variables and also the FPGA Machine.
	// Called from main.c, but not until after FPGA is loaded.
	digio_Enc1OutFreq32 = 0x000003A8; // 10 kHz
	digio_Enc1StopAfterN = 0;	// disabled
	digio_Enc1OutIndex32 = 4;	// 1 index pulse every 4 encoder cycles (every 16 counts)
	digio_Enc1OutDir = 1;		// 1 = A leads B
	digio_Enc1ManualStop = 0;	// 0 = not stopped, free run

	digio_Enc2OutFreq32 = 0x000003A8; // 10 kHz
	digio_Enc2StopAfterN = 0;  // disabled
	digio_Enc2OutIndex32 = 4;  // 1 index pulse every 4 encoder cycles (every 16 counts)
	digio_Enc2OutDir = 1;      // 1 = A leads B
	digio_Enc2ManualStop = 0;	// 0 = not stopped, free run

	// Encoder Output #1
	*CPLD_F1_XA(FPGA1_WRITE_ENC1_MANUAL_STOP) = 1; // to be safe, stop it before writing params

	*CPLD_F1_XA(FPGA1_WRITE_ENC1_FREQ_LS16) = (Uint16)(digio_Enc1OutFreq32 & 0x0000FFFF);
    *CPLD_F1_XA(FPGA1_WRITE_ENC1_FREQ_MS16) = (Uint16)((digio_Enc1OutFreq32 >> 16) & 0x0000FFFF);

	*CPLD_F1_XA(FPGA1_WRITE_ENC1_INDEX_COUNT_LS16) = (Uint16)(digio_Enc1OutIndex32 & 0x0000FFFF);
	*CPLD_F1_XA(FPGA1_WRITE_ENC1_INDEX_COUNT_MS16) = (Uint16)((digio_Enc1OutIndex32 >> 16) & 0x0000FFFF);

	*CPLD_F1_XA(FPGA1_WRITE_ENC1_DIR) = digio_Enc1OutDir;

	*CPLD_F1_XA(FPGA1_WRITE_ENC1_STOP_AFTER_LS16) = (Uint16)(digio_Enc1StopAfterN & 0x0000FFFF);
	*CPLD_F1_XA(FPGA1_WRITE_ENC1_STOP_AFTER_MS16) = (Uint16)((digio_Enc1StopAfterN >> 16) & 0x0000FFFF);

	*CPLD_F1_XA(FPGA1_WRITE_ENC1_MANUAL_STOP) = digio_Enc1ManualStop;


	// Encoder Output #2
	*CPLD_F1_XA(FPGA1_WRITE_ENC2_MANUAL_STOP) = 1; // to be safe, stop it before writing params

	*CPLD_F1_XA(FPGA1_WRITE_ENC2_FREQ_LS16) = (Uint16)(digio_Enc2OutFreq32 & 0x0000FFFF);
	*CPLD_F1_XA(FPGA1_WRITE_ENC2_FREQ_MS16) = (Uint16)((digio_Enc2OutFreq32 >> 16) & 0x0000FFFF);

	*CPLD_F1_XA(FPGA1_WRITE_ENC2_INDEX_COUNT_LS16) = (Uint16)(digio_Enc2OutIndex32 & 0x0000FFFF);
	*CPLD_F1_XA(FPGA1_WRITE_ENC2_INDEX_COUNT_MS16) = (Uint16)((digio_Enc2OutIndex32 >> 16) & 0x0000FFFF);

	*CPLD_F1_XA(FPGA1_WRITE_ENC2_DIR) = digio_Enc2OutDir;

	*CPLD_F1_XA(FPGA1_WRITE_ENC2_STOP_AFTER_LS16) = (Uint16)(digio_Enc2StopAfterN & 0x0000FFFF);
	*CPLD_F1_XA(FPGA1_WRITE_ENC2_STOP_AFTER_MS16) = (Uint16)((digio_Enc2StopAfterN >> 16) & 0x0000FFFF);

	*CPLD_F1_XA(FPGA1_WRITE_ENC2_MANUAL_STOP) = digio_Enc2ManualStop;

}

enum CANOPEN_STATUS digio_recvEncOutParam(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host: a parameter value configuring Simulated Encoder Output.
	// I'm using the same routine to handle all of the different encoder parameters
	// for this CAN Index, so first thing to do is to extract the CAN Subindex
	// and use that to determine which encoder parameter I just received.
	// Store the value so we can retrieve it if the host requests it,
	// And write it to the Enc Out machine in FPGA1
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;
	Uint16 subindex;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	// For all subindex values, we take the 16-bits received in MboxC and store it
	// in one of our static locations at the dest address we get from the can_command table entry.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC -- save LS16 to static variable

	//DSP fix to a minor FPGA ideosyncracy / User problem
	//If user changes Freq, Index, or StopAfterN count while StopAfterN !=0 and ManualStop = 0,
	//the FPGA encoder generates a couple extra pulses, that may or may not be problematic.
	//Work around is to use ManualStop = 1 to stop the encoder before changing other parameters.
	//Here we try to do that for the user, in case he can't do it for himself.
	if ((subindex == 1) || (subindex == 2) ||(subindex == 7)) {
		*CPLD_F1_XA(FPGA1_WRITE_ENC1_MANUAL_STOP) = 1;
	}
	if ((subindex == 4) || (subindex == 5) ||(subindex == 8)) {
		*CPLD_F1_XA(FPGA1_WRITE_ENC2_MANUAL_STOP) = 1;
	}

	// Now we perform different things depending on the CAN subindex value
	switch(subindex){
    case 0x01: // received digio_Enc1OutFreq32
    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_FREQ_LS16) = *dest++;
    	*dest = *(data+3);  //MboxD -- save MS16 to static variable
    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_FREQ_MS16) = *dest;
        break;

    case 0x02: // received digio_Enc1OutIndex16
    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_INDEX_COUNT_LS16) = *dest++;
    	*dest = *(data+3);  //MboxD -- save MS16 to static variable
    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_INDEX_COUNT_MS16) = *dest;
        break;

    case 0x03: // received digio_Enc1OutDir
    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_DIR) = *dest;
        break;

    case 0x04: // received digio_Enc2OutFreq32
    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_FREQ_LS16) = *dest++;
    	*dest = *(data+3);  //MboxD -- save MS16 to static variable
    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_FREQ_MS16) = *dest;
        break;

    case 0x05: // received digio_Enc2OutIndex16
    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_INDEX_COUNT_LS16) = *dest++;
    	*dest = *(data+3);  //MboxD -- save MS16 to static variable
    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_INDEX_COUNT_MS16) = *dest;
        break;

    case 0x06: // received digio_Enc2OutDir
    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_DIR) = *dest;
        break;

    case 0x07: // received digio_Enc1StopAfterN
    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_STOP_AFTER_LS16) = *dest++;
    	*dest = *(data+3);  //MboxD -- save MS16 to static variable
    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_STOP_AFTER_MS16) = *dest;
        break;

    case 0x08: // received digio_Enc2StopAfterN
    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_STOP_AFTER_LS16) = *dest++;
    	*dest = *(data+3);  //MboxD -- save MS16 to static variable
    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_STOP_AFTER_MS16) = *dest;
        break;

    case 0x09: // received digio_Enc1ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_MANUAL_STOP) = *dest;
        break;

    case 0x0A: // received digio_Enc2ManualStop
    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_MANUAL_STOP) = *dest;
        break;

     default:

    	break;
    }

	// Always re-write the ManualStop param.  Doing this in case we
	// set ManualStop to 1 above before changing Freq, Index, or StopAfterN parameters.
	*CPLD_F1_XA(FPGA1_WRITE_ENC1_MANUAL_STOP) = digio_Enc1ManualStop;
	*CPLD_F1_XA(FPGA1_WRITE_ENC2_MANUAL_STOP) = digio_Enc2ManualStop;


	return CANOPEN_NO_ERR;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   E N C O D E R   D I G I T A L   O U T P U T  --
//      C O M P A T I B L E   W/  C L A S S I C   T E S T   S T A T I O N
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	Uint16 digio_Enc1ClassicFreq;
	Uint16 digio_Enc1ClassicIndex;
	Uint16 digio_Enc1ClassicDir;
	Uint16 digio_Enc2ClassicFreq;
	Uint16 digio_Enc2ClassicIndex;
	Uint16 digio_Enc2ClassicDir;

	enum CANOPEN_STATUS digio_recvClassicEncOutParam(const struct CAN_COMMAND *can_command,Uint16 *data) {
		// We received data from host: a parameter value configuring Simulated Encoder Output.
		// This command uses 16-bit parameters compatible with the classic test station,
		// so we must perform conversions to convert to TS3 32-bit architecture and new 75MHz clock speed.
		// I'm using the same routine to handle all of the different encoder parameters
		// for this CAN Index, actually two CAN Indices, so first thing to do is to extract the CAN Subindex
		// and CAN Index and use that to determine which encoder parameter I just received.
		// Store the value so we can retrieve it if the host requests it,
		// And write it to the Enc Out machine in FPGA1
		// *data is MboxA of received Message
		// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

		Uint16 *dest;
		Uint16 subindex;
		Uint16 index;
		Uint16 m1, m2;
		union CANOPEN16_32 result;

		// Subindex is MS 8 bits of MboxB
		subindex = ((*(data+1)) >> 8) & 0x00FF;
		// Index is ((LS 8 bits of MboxB) << 8) Or ((MS 8 bits of MboxA) >> 8)
		index = (((*(data+1)) << 8) & 0xFF00) | (((*(data)) >> 8) & 0x00FF);

		// For all subindex values, we take the 16-bits received in MboxC and store it
		// in one of our static locations at the dest address we get from the can_command table entry.
		dest = (Uint16*)can_command->datapointer;
		*dest = *(data+2); // MboxC -- save LS16 to static variable

		if (index == 0x2008) {
			// Enc1

			// Now perform different actions depending on the CAN subindex value
			switch(subindex){
		    case 0x01: // Frequency
				// Lets try to convert from a Classic Test Station 16-bit frequency request to
				// a 32-bit TS3 "native" frequency parameter
				// 		classic_Freq_num = (20,000,000/(desired_Hz*8))-1				// Classic
				//		desired_Hz = 75,000,000 / (8 * (TS3_stored_enc_freq + 1))		// TS3
				//		TS3_stored_enc_freq  =((classic_Freq_num + 1 ) * (15/4)) - 1	// Conversion

		    	// 1st pass, set frequency (period) -- HAVE TO ADJUST FOR ENC1 / ENC2
		    	m1 = digio_Enc1ClassicFreq + 1; // set up Uint16 values for multiplication
		    	m2 = 15;

		    	//if (m1 < 0x3FFF) {
		    	// m1 += 2;		// Heuristically, found this makes highest freqs more
		    	// 				//compatible with Classic test ststion. (Low Freq doesn't matter.)
		    	//}
		    	result.all = ((unsigned long) m1 * (unsigned long) m2) >> 2;

		    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_FREQ_LS16) = result.words.lsw;
		    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_FREQ_MS16) = (result.words.msw) & 0x3FFF;
                // store native equivalent of result, can be read vis 32-bit, native interface commands
		    	digio_Enc1OutFreq32 = result.all & 0x3FFFFFFF;

		        break;

		    case 0x02: // Index

		    	// Classic test station, request (N) EncA counts between Index pulses, get (N+1)
		    	// TS3 test station, request (N) EncA counts between Index pulses, get (N+1)
		    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_INDEX_COUNT_LS16) = digio_Enc1ClassicIndex;
		    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_INDEX_COUNT_MS16) = 0;
		    	digio_Enc1OutIndex32 = (Uint32)digio_Enc1ClassicIndex;
		        break;

		    case 0x03: // Direction
		    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_DIR) = digio_Enc1ClassicDir;
		    	digio_Enc1OutDir = digio_Enc1ClassicDir;
		        break;
		     default:

		    	break;
		    }

			// So if they are writing Classic, commands, the assumption is it shouldn't be stopped.
			*CPLD_F1_XA(FPGA1_WRITE_ENC1_MANUAL_STOP) = 0;
			digio_Enc1ManualStop = 0;
	    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_STOP_AFTER_LS16) = 0;
	    	*CPLD_F1_XA(FPGA1_WRITE_ENC1_STOP_AFTER_MS16) = 0;
			digio_Enc1StopAfterN = 0;	// disabled


		} else {
			// Enc2

			// Now perform different actions depending on the CAN subindex value
			switch(subindex){
		    case 0x01: // Frequency
				// Lets try to convert from a Classic Test Station 16-bit frequency request to
				// a 32-bit TS3 "native" frequency parameter
				// 		classic_Freq_num = (20,000,000/(desired_Hz*8))-1				// Classic
				//		desired_Hz = 75,000,000 / (8 * (TS3_stored_enc_freq + 1))		// TS3
				//		TS3_stored_enc_freq  =((classic_Freq_num + 1 ) * (15/4)) - 1	// Conversion

		    	// 1st pass, set frequency (period) -- HAVE TO ADJUST FOR ENC1 / ENC2
		    	m1 = digio_Enc2ClassicFreq + 1; // set up Uint16 values for multiplication
		    	m2 = 15;


		    	result.all = ((unsigned long) m1 * (unsigned long) m2) >> 2;

		    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_FREQ_LS16) = result.words.lsw;
		    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_FREQ_MS16) = (result.words.msw) & 0x3FFF;
                // store native equivalent of result, can be read vis 32-bit, native interface commands
		    	digio_Enc2OutFreq32 = result.all & 0x3FFFFFFF;

		        break;

		    case 0x02: // Index

		    	// Classic test station, request (N) EncA counts between Index pulses, get (N+1)
		    	// TS3 test station, request (N) EncA counts between Index pulses, get (N+1)
		    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_INDEX_COUNT_LS16) = digio_Enc2ClassicIndex;
		    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_INDEX_COUNT_MS16) = 0;
		    	digio_Enc2OutIndex32 = (Uint32)digio_Enc2ClassicIndex;
		        break;

		    case 0x03: // Direction
		    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_DIR) = digio_Enc2ClassicDir;
		    	digio_Enc2OutDir = digio_Enc2ClassicDir;
		        break;
		     default:

		    	break;
		    }

			// So if they are writing Classic, commands, the assumption is it shouldn't be stopped.
			*CPLD_F1_XA(FPGA1_WRITE_ENC2_MANUAL_STOP) = 0;
			digio_Enc2ManualStop = 0;
	    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_STOP_AFTER_LS16) = 0;
	    	*CPLD_F1_XA(FPGA1_WRITE_ENC2_STOP_AFTER_MS16) = 0;
			digio_Enc2StopAfterN = 0;	// disabled
		}

		return CANOPEN_NO_ERR;
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	//
	//  H A L L   D I G I T A L   O U T P U T  --  N A T I V E   M O D E
	//        A N D   C O M P A T I B L E   W /  C L A S S I C   T E S T   S T A T I O N
	//
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	Uint32 digio_HallOutputFreq32;			// hall_out_freq = (75MHz) / (12 * (digio_HallOutputFreq32 + 1))
	Uint16 digio_HallOutputFreqClassic16;
	Uint16 digio_HallOutputPhaseClassic16;  // Phase: 0 = 60 degree phasing, 1 = 120 degree phasing
	Uint16 digio_HallOutputDirClassic16;    // Direction: 1 == A leads B leads C, 0 == C leads B leads A


	void digio_HallOutInit(void){
		// Initialize RAM variables and also the FPGA Machine.
		// Called from main.c, but not until after FPGA is loaded.
		digio_HallOutputFreq32 = 0x000003A8; // 10 kHz
		digio_HallOutputFreqClassic16 = 1;	 // ???
		digio_HallOutputPhaseClassic16 = 0;	 // Phase: 0 = 60 degree
		digio_HallOutputDirClassic16 = 0;	 // 0 == C leads B leads A

		// Need to write these to the FPGA
		//*CPLD_F1_XA(FPGA1_WRITE_ENC1_MANUAL_STOP) = 1; // to be safe, stop it before writing params

	}

	enum CANOPEN_STATUS digio_recvHallOutputParamClassic16(const struct CAN_COMMAND *can_command,Uint16 *data) {
		// We received data from host: a parameter value configuring Simulated Hall Output.
		// This command uses 16-bit parameters compatible with the classic test station,
		// so we may have to perform conversions to convert to TS3 32-bit architecture and new 75MHz clock speed.
		// I'm using the same routine to handle all of the different hall parameters
		// for this CAN Index, so first thing to do is to extract the CAN Subindex
		// and use that to determine which Hall parameter I just received.
		// Store the value so we can retrieve it if the host requests it,
		// And write it to the Hall Out machine in FPGA1
		// *data is MboxA of received Message
		// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

		Uint16 *dest;
		Uint16 subindex;
		Uint16 m1, m2;
		union CANOPEN16_32 result;

		// Subindex is MS 8 bits of MboxB
		subindex = ((*(data+1)) >> 8) & 0x00FF;

		// For all subindex values, we take the 16-bits received in MboxC and store it
		// in one of our static locations at the dest address we get from the can_command table entry.
		dest = (Uint16*)can_command->datapointer;
		*dest = *(data+2); // MboxC -- save LS16 to static variable

		// Now perform different actions depending on the CAN subindex value
		switch(subindex){
	    case 0x01: // Frequency
			// Lets try to convert from a Classic Test Station 16-bit frequency request to
			// a 32-bit TS3 "native" frequency parameter
			// 		classic_Freq_num = (20,000,000/(desired_Hz*12))-1				// Classic
	    	//		desired_Hz = 75,000,000 / (12 * (TS3_stored_hall_freq + 1))		// TS3
			//		TS3_stored_enc_freq  =((classic_Freq_num + 1 ) * (15/4)) - 1	// Conversion

	    	// 1st pass, set frequency (period) -- HAVE TO ADJUST FOR ENC1 / ENC2
	    	m1 = digio_HallOutputFreqClassic16 + 1; // set up Uint16 values for multiplication
	    	m2 = 15;

	    	//if (m1 < 0x3FFF) {
	    	// m1 += 2;		// Heuristically, found this makes highest freqs more
	    	// 				//compatible with Classic test ststion. (Low Freq doesn't matter.)
	    	//}
	    	result.all = ((unsigned long) m1 * (unsigned long) m2) >> 2;

	    	*CPLD_F1_XA(FPGA1_WRITE_HALL_FREQ_LS16) = result.words.lsw;
	    	*CPLD_F1_XA(FPGA1_WRITE_HALL_FREQ_MS16) = (result.words.msw) & 0x3FFF;
            // store native equivalent of result, can be read vis 32-bit, native interface commands
	    	digio_HallOutputFreq32 = result.all & 0x3FFFFFFF;

	        break;

	    case 0x02: // Phasing
	    	// Same for Classic as for Native interface
	    	// Just write it to FPGA1
	    	*CPLD_F1_XA(FPGA1_WRITE_HALL_PHASE) = digio_HallOutputPhaseClassic16;
	        break;

	    case 0x03: // Direction
	    	// Same for Classic as for Native interface
	    	// Just write it to FPGA1
	    	*CPLD_F1_XA(FPGA1_WRITE_HALL_DIR) = digio_HallOutputDirClassic16;
	        break;

	     default:

	    	break;
	    }

		return CANOPEN_NO_ERR;
	}

	enum CANOPEN_STATUS digio_recvHallOutputParam32(const struct CAN_COMMAND *can_command,Uint16 *data) {
		// We received 32-bit data from host, representing frequency for the Dig Out Hall
		// signal, encoded as:
		//     hall_output_freq = (75MHz) / (12 * (digio_HallOutputFreq32 + 1))
		// Note that other Hall parameters (Dir and Phase) are handled in the classic 16-bit interface.
		// Store the value so we can retrieve it if the host requests it,
		// And write it to the Dig Out PWM machine in FPGA1
		// *data is MboxA of received Message
		// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

		Uint16 *dest;

		dest = (Uint16*)can_command->datapointer;
		*dest = *(data+2); // MboxC
		*CPLD_F1_XA(FPGA1_WRITE_HALL_FREQ_LS16) = *dest++;
		*dest = *(data+3);  //MboxD
		*CPLD_F1_XA(FPGA1_WRITE_HALL_FREQ_MS16) = *dest;

		return CANOPEN_NO_ERR;
	}

