// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     AnlgIn.C
//
//   analog input
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "CanOpen.h"
#include "TaskMgr.h"
#include "CPLD.H"
#include "AnlgIn.H"
#include "Log.H"

// - - - following static Ram used for Analog Input Calibration task - - -
enum ANLGIN_CAL_STATE anlgin_cal_state;
Uint16 ain_offsetCount;
Uint16 ain_ioPinSwitches; // store original setting, restore it when thru
Uint16 ain_loopbackMux;   // store original setting, restore it when thru
Uint32 ain_values[8];
Uint16 ain_offsets[8];
Uint16 saved_ioPinSwitches; // read I/O pin switch settings before changing them

void ain_offsetCalcInit(void){
	// Set up state variabbles prior to running Analog Input Calibration
	anlgin_cal_state = ANLGIN_CS_DISCONNECT_IO;
}

void ain_setSwitchDefaultsForB1(void){
	// In TB3IOMC we have a set of digital switches that specifically affect
	// Analog In channel B1
	*CPLD_F2_XA(FPGA2_WRITE_ANLG_IN_B1_SWITCH) = 0x0001; // AN_IN_B1_STD_SW
}


enum ANLGIN_CAL_STATE ain_getAnlginCalState(void){
	return anlgin_cal_state;
}
void ain_offsetCalcTask(void){
	// This procedure is run under the task manager to perform a sequence of steps
	// to calibrate the 8 analog inputs.  It sets internal switches so that 8 Analog
	// outputs are looped back to 8 Analog Inputs, and all are disconnected from I/O
	// connector pins.  We command 0V from the Analog Outputs and read the Analog Inputs,
	// collecting baseline 0V readings that are henceforth subtracted from Analog Input
	// readings to yield calibrated Analog Input values.
	// TIMING: we have a 0.1 Sec delay between commanding 0V out of the DACs (Analog
	// outputs) and reading the Analog Inputs from the A to D converter.  We take 4
	// readings for each Analog Input, with 0.1 Sec delay between readings, and average
	// them to use as out baseline 0V calibration value.
	// In setting up for the calibration, this task also sets the Filter Freq to
	// 1kHz for the LP Filters on ANLG_IN_5 to 8.  And it puts the A-to-D converter for
	// ANLG_IN_1 to 4 in AUTO_CAPTURE mode.
	// ON COMPLETION the task leaves I/O Connector Pins disabled.
	volatile Uint16 * write_ioPinSwitches;
	volatile Uint16 * write_dacAnlgOut;
	volatile Uint16 * write_loopbackMux;
	volatile Uint16 * write_anlgInCaptureMode;
	volatile Uint16 * read_ainValue;
	volatile Uint16 * write_anlgInFilterClk;
	Uint16 i;
	Uint16 delayInTenthsOfSec;
	union CANOPEN16_32 temp32;

	delayInTenthsOfSec = 0;

	switch(anlgin_cal_state){
    case ANLGIN_CS_DISCONNECT_IO: // Disconnect I/O Pins

    	// A bit if initialization first
    	ain_ioPinSwitches = *CPLD_F2_XA(FPGA2_READ_IO_PIN_SWITCHES);
    	ain_loopbackMux = *CPLD_F2_XA(FPGA2_READ_LOOPBACK_MUX);
    	ain_offsetCount = 0;
    	for(i=0;i<8;i++){
    		ain_values[i] = 0x00000000;
    		ain_offsets[i] = 0;
    	}
    	write_ioPinSwitches = CPLD_F2_XA(FPGA2_WRITE_IO_PIN_SWITCHES);
    	*write_ioPinSwitches = ain_ioPinSwitches & 0xFF3F; // turn off anlg in & anlg out
    	anlgin_cal_state = ANLGIN_CS_SET_ANLG_OUT;
    	LOG_AINCAL_ADDTOLOG(LOG_EVENT_ANLG_IN_CAL,0x0001);
        break;

    case ANLGIN_CS_SET_ANLG_OUT: // Set values for Analog Out
    	// For calibration purposes, we will set them all to 0V = 0x800
    	write_dacAnlgOut = CPLD_F1_XA(FPGA1_WRITE_DAC_ANLG_OUT_1);
    	* (write_dacAnlgOut++) = 0x800; //	ANLG_OUT_1, 0.00 Volts, and incr. ptr. after write
    	* (write_dacAnlgOut++) = 0x800; //
    	* (write_dacAnlgOut++) = 0x800; //
    	* (write_dacAnlgOut++) = 0x800; //
    	* (write_dacAnlgOut++) = 0x800; //
    	* (write_dacAnlgOut++) = 0x800; //
    	* (write_dacAnlgOut++) = 0x800; //
    	* (write_dacAnlgOut++) = 0x800; // ANLG_OUT_8

    	// Set Analog in for auto capture mode
    	write_anlgInCaptureMode = CPLD_F2_XA(FPGA2_WRITE_ADC_CAPTURE_MODE);
    	* write_anlgInCaptureMode = 1; // 1 => AUTO MODE, continuously running capture
    	write_anlgInFilterClk = CPLD_F2_XA(FPGA2_WRITE_ANLG_IN_FLTR_CLK);
    	* write_anlgInFilterClk = 0x00BA; // 1000Hz LPF
    	anlgin_cal_state = ANLGIN_CS_SET_LOOPBACK;
   	    LOG_AINCAL_ADDTOLOG(LOG_EVENT_ANLG_IN_CAL,0x0002);
        break;

    case ANLGIN_CS_SET_LOOPBACK: // Connect Anlg Out to Anlg In via loopback Mux
    	write_loopbackMux = CPLD_F2_XA(FPGA2_WRITE_LOOPBACK_MUX);
    	*write_loopbackMux = 0x0009; // ANLG_MUX_EN=1,ANLG_MUX_A0=1;ANLG_MUX_A1=0;ANLG_MUX_A2=0

    	delayInTenthsOfSec = 1;  // re-launch the task after 0.1Sec delay
    	anlgin_cal_state = ANLGIN_CS_READ_AI_VALUES;
    	ain_offsetCount = 0;
    	LOG_AINCAL_ADDTOLOG(LOG_EVENT_ANLG_IN_CAL,0x0003);
    	LOG_AINCAL_ADDTOLOG(LOG_EVENT_ANLG_IN_CAL,0x0004);
        break;

    case ANLGIN_CS_READ_AI_VALUES: // Read Analog In Values
    	read_ainValue = CPLD_F2_XA(FPGA2_READ_ADC_A1);
    	for(i=0;i<8;i++){
    		ain_values[i] += *(read_ainValue++); // increment pointer after each read
    	}
    	if (ain_offsetCount++ < 3) {
        	anlgin_cal_state = ANLGIN_CS_READ_AI_VALUES;
        	delayInTenthsOfSec = 1;  // re-launch the task after 0.1Sec delay
    	} else {
        	anlgin_cal_state = ANLGIN_CS_CALC_OFFSETS;
        	delayInTenthsOfSec = 0;  // re-launch the task
    	}
    	LOG_AINCAL_ADDTOLOG3(LOG_EVENT_ANLG_IN_CAL,0x0005,ain_offsetCount,0);
        break;

    case ANLGIN_CS_CALC_OFFSETS: // OK we are done but for the final calculation
    	for(i=0;i<8;i++){
    		temp32.all = ain_values[i]>>2; // we summed 4 samples, now divide by 4

    		// For all 8 analog inputs we store the average deviation
    		// from ideal 0 Volts (0x8000)
    		ain_offsets[i] = temp32.words.lsw - 0x8000;

    		// put I/O pins connections back to as we found them
    		// Leave loopback mux off
        	write_loopbackMux = CPLD_F2_XA(FPGA2_WRITE_LOOPBACK_MUX);
        	*write_loopbackMux = 0x0000; // ANLG_MUX_EN=0,ANLG_MUX_A0=0;ANLG_MUX_A1=0;ANLG_MUX_A2=0
        	write_ioPinSwitches = CPLD_F2_XA(FPGA2_WRITE_IO_PIN_SWITCHES);
        	*write_ioPinSwitches = ain_ioPinSwitches & 0xFF3F; // turn off anlg in & anlg out
    	}
    	anlgin_cal_state = ANLGIN_CS_RESTORE_SWITCHES; // signals "DONE" to whoever started us
    	LOG_AINCAL_ADDTOLOG(LOG_EVENT_ANLG_IN_CAL,0x0006);
    	delayInTenthsOfSec = 1;  // re-launch the task after 0.1Sec delay
        break;

    case ANLGIN_CS_RESTORE_SWITCHES: // give previoux mux setting time to settle
		// previously turned off DACs & Loopback mux's, and waited 0.1 sec
    	// now we restore mux & IO/Pin switches to way they were originally
    	write_loopbackMux = CPLD_F2_XA(FPGA2_WRITE_LOOPBACK_MUX);
    	*write_loopbackMux = ain_loopbackMux; // saved this value when we dstarted calibration
    	write_ioPinSwitches = CPLD_F2_XA(FPGA2_WRITE_IO_PIN_SWITCHES);
    	*write_ioPinSwitches = ain_ioPinSwitches; // saved this value when we dstarted calibration
    	LOG_AINCAL_ADDTOLOG(LOG_EVENT_ANLG_IN_CAL,0x0007);
    	anlgin_cal_state = ANLGIN_CS_CALIBRATION_DONE; // signals "DONE" to whoever started us
     	return; // w/out re-launching task
     	// break; //unreachable break

    default:
     	return; // w/out re-launching task
     	// break; //unreachable break
    }

	taskMgr_setTaskRoundRobin(TASKNUM_ain_offsetCalcTask, delayInTenthsOfSec); // re-launch this task
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       C A N   O P E N   R E S P O N S E S
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
enum CANOPEN_STATUS ain_startOffsetCalc(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
    // This function kicks off a background task to calibrate analog inputs.
	ain_offsetCalcInit();
	taskMgr_setTaskRoundRobin(TASKNUM_ain_offsetCalcTask, 0);
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS ain_calibratedValue(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function reads an analog input value from the FPGA, and applies an offset
	// calibration to it before returning the value via CAN
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to the ain_offsets array, specifically to one of the
	// elements [0] thru [7], effectively identifying which Anlg In port to report on.
	volatile Uint16 * read_ainValue;
	Uint16 ain_value;
	Uint16 channel;
	Uint16 *src;
	Uint16 offset;

	// figure which of our 8 Analog Inputs is requested
	src = (Uint16*)can_command->datapointer;
	channel = (Uint16)(src - ain_offsets);
	// fetch a fresh measured value from the FPGA
	read_ainValue = CPLD_F2_XA(FPGA2_READ_ADC_A1  + channel);
	ain_value = *(read_ainValue);

	// subtract the calibration offset from the raw value, and do it without
	// wrapping around the bounds of our 16-bit values
	offset = ain_offsets[channel];
    if ((offset & 0x8000) && (offset < ain_value)) {
    	ain_value = 0xFFFF;
    } else if ((!(offset & 0x8000)) && (offset > ain_value)) {
    	ain_value = 0x0000;
    } else {
    	ain_value -= offset;
    }

	*(data+2) = ain_value; //MboxC
	*(data+3) = 0;   //MboxD

	return CANOPEN_NO_ERR;
}

Uint16 savedAin;   // saved (raw ain) value from last call to ..ClassicValue
Uint16 savedAinCal; // saved (ain - calibration) value from last call to ..ClassicValue

enum CANOPEN_STATUS ain_calibratedClassicValue(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function reads an analog input value from the FPGA, and applies an offset
	// calibration to it, and scales it to be compatible with the
	// "classic" Test Station and with legacy PC software, before returning the value via CAN
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to the ain_offsets array, specifically to one of the
	// elements [0] thru [7], effectively identifying which Anlg In port to report on.
	volatile Uint16 * read_ainValue;
	Uint16 ain_value;
	Uint16 channel;
	Uint16 *src;
	Uint16 offset;

	int ain_val_int;
	int m2;
	int result;

	LOG_ACLAS_ADDTOLOG(LOG_EVENT_ANLG_IN_CLASSIC,1);  // event log clocked this procedure at 9.49 uS
	                                                  // (-5.86 for event_log) -> 3.63 uS net for whole procedure.

	// figure which of our 8 Analog Inputs is requested
	src = (Uint16*)can_command->datapointer;
	channel = (Uint16)(src - ain_offsets);

	// fetch a fresh measured value from the FPGA
	read_ainValue = CPLD_F2_XA(FPGA2_READ_ADC_A1  + channel);
	ain_value = *(read_ainValue);
	savedAin = ain_value; // save value to be fetched later for diagnostic purposes

	// subtract the calibration offset from the raw value, and do it without
	// wrapping around the bounds of our 16-bit values
	offset = ain_offsets[channel];
    if ((offset & 0x8000) && (offset < ain_value)) {
    	ain_value = 0xFFFF;
    } else if ((!(offset & 0x8000)) && (offset > ain_value)) {
    	ain_value = 0x0000;
    } else {
    	ain_value -= offset;
    }

	savedAinCal = ain_value; // save value to be fetched later for diagnostic purposes

	// Now apply scaling to be compatible w/ classic Test Station and legacy PC software.
	// We use 2 different A-2-Ds for 1,2,3,4 vs 5,6,7,8 with different scaling.
	// In TB3IOMC Anlg_In_8 had yet a  3rd scaling applied, because it used a different
	// input amplifier.  In TB3IOMD, Anlg_In_8 has the same input amplifier, and scales
	// the same as 7,6, & 5.

    // <ain_value_classic> = (<ain_calibrated_native> - 0x8000) * (0x3565/0x10000)
    //                     = (<ain_calibrated_native> - 0x8000) * 0.20857
    // REF on 2812 Multiplication:
    // "C/C++ Code Access to the Upper 16 Bits of 16-Bit Multiply"
    // p139, spru514e_TMS320C28x Compiler and Linker Users Guide.pdf
	if (channel < 4) { // channel 0,1,2,3, AKA Anlg_In_1,2,3,4, AKA Anlg_in_A1,A2,A3,A4

		ain_val_int = ain_value - 0x8000;
		m2 = (int)0x3565;
		result = ((long)ain_val_int * (long)m2) >> 16; // event log clocked this at 6.4 uS (-5.86 for event_log) -> 0.54 uS

	} else {

		// <ain_value_classic> = (0x8000 - <ain_calibrated_native>) * (0x3BAB/0x10000)
		//                     = (0x8000 - <ain_calibrated_native>) * 0.233079
		ain_val_int = 0x8000 - ain_value;
		m2 = (int)0x3BAB;
		result = ((long)ain_val_int * (long)m2) >> 16;
	}

	*(data+2) = result; //MboxC
	*(data+3) = 0;   //MboxD
	LOG_ACLAS_ADDTOLOG(LOG_EVENT_ANLG_IN_CLASSIC,2);

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS ain_calibratedClassicValRaw(const struct CAN_COMMAND* can_command, Uint16* data){
	// Fetch the calibrated value (ain_value +/- offset) used in the previous call / CAN
	// request to ain_calibratedClassicValue(), above.  This is useful only for diagnostic
	// purposes -- verifying that the voltage reported using scaling for the Classic Test
	// Station is the same as the voltage using the nominal scaling for TS3 / TB3IOMC.
	*(data+2) = savedAinCal; //MboxC
	*(data+3) = savedAin;   //MboxD

	return CANOPEN_NO_ERR;
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       A D 7 1 7 5   S E T U P   /   C O N F I G U R A T I O N
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum ANLGIN_AD7175_SETUP ain_ad7175SetupTaskState;
Uint16 ain_ad7175SetupWait;
enum ANLGIN_AD7175_ERR ain_ad7175SetupError;
Uint16 ain_ad7175SetupRegisterIndex;

// To configure the AD7175 A-to-D converter on startup, we have to write
// configuration values to many internal registers within the AD7175.
// The following table contains 2 16-bit values for each such register
// requiring setup.  The first value is a "register address," identifying
// the register within the AD7175.  The 2nd value is the configuration
// value we write to the register.  A full explanation of the registers
// and configuration values can be found in the AD7175-2 Data Sheet.  I
// have attempted to provide some cursory notes within the comments.
struct ANLGIN_AD7175_REGISTER_CONFIG ain_ad7175RegisterConfigValues[] = {
		{0x0020,0x0F00}, // Setup_Config_Reg_0
		{0x0021,0x0F00}, // Setup_Config_Reg_1
		{0x0022,0x0F00}, // Setup_Config_Reg_2
//		{0x0023,0x1F00}, // Setup_Config_Reg_3 in TB3IOMC, ch 3 is differential
		{0x0023,0x0F00}, // Setup_Config_Reg_3 in TB3IOMD, ch 3 is single ended

		//Setup Configuration Register 0,1,2,3
		//15:13     000      N/A
		//12         0       Unipolar (1->Bipolar)
		//11         1       Enable Buffering Ref +
		//10         1       Enable Buffering Ref -
		//9          1       Enable Buffering Ain_In +
		//8          1       Enable Buffering Ain_In -
		//7:6        00      N/A
		//5:4        00      External Ref
		//3:0        0000    N/A
		//See Spec page 57

		{0x0010,0x8016}, // Channel_Reg_0
		{0x0011,0x9036}, // Channel_Reg_1
		{0x0012,0xA056}, // Channel_Reg_2
//		{0x0013,0xB064}, // Channel_Reg_3 in TB3IOMC is differential ref Ain4
		{0x0013,0xB076}, // Channel_Reg_3 in TB3IOMD is single ended ref REF-

		//Channel Register 0,1,2,3
		//15         1       Channel is Enabled (0->Disabled)
		//14         0       N/A
		//13:12      00     00->SetupConfig_Reg_0
		//                        01->SetupConfig_Reg_1
		//                        10->SetupConfig_Reg_2
		//                        11->SetupConfig_Reg_3
		//11:10      00     N/A
		//9:5      00000    pos input pin 0000->Ain0
		//                          00000-> Ain0
		//                          00001-> Ain1
		//                          00010-> Ain2
		//                          00011-> Ain3
		//4:0     10110     neg input pin is REF-
		//                          00100-> Ain4 for B4 -diff in
		//See Spec page 55

		{0x0002,0x0040}, // Interface_Mode_Reg

		//Interface Mode Register, 0x02
		//15:13     000     N/A
		//12         0      (ALT_SYNC ??)
		//11         0      (IOSTRENGTH ??)
		//10:9      00      N/A
		//8          0      (DOUT_RESET ??)
		//7          0      Continuous read mode of data reg
		//                     can only be used in continuous
		//                     conversion mode.
		//6          1      DATA_STAT
		//                      1-> append 8-bit status
		//                      to 24-bit data.
		//                      0-> disabled
		//5           0     (REG_CHECK ??)
		//4           0     N/A
		//3:2        00     CRC_EN
		//                     00-> disabled
		//1           0     N/A
		//0           0     WL16
		//                     0-> 24 bit data
		//                     1-> 16 bit data
		//See Spec page 52

		{0x0001,0x0000} // ADC_Mode_Reg (0->read continuous conversion)

		//ADC Mode Register, 0x01
		//15         0      REF_EN
		//                      1-> enable internal 2.5V ref to
		//                      REFOUT pin
		//                      0-> disabled
		//14         0      (HIDE_DELAY ??)
		//13         0      (SING_CYC ??)
		//12:11      00     N/A
		//10:8       000    (DELAY ??)
		//7           0      N/A
		//6:4        001    000-> Contin. Conversion Mode
		//                      001-> Single Conversion Mode
		//                      010-> Standby
		//                      011-> Power-down
		//                      100-> Internal Offset Cal
		//                      101-> (not defined)
		//                      110-> System Offset Cal
		//                      111-> System Gain Cal
		//3:2         00    clock selection
		//                      00-> internal oscilator
		//1:0         00    N/A
		//See Spec page 51
};
#define MAX_AD7175_REGISTER_CONFIG 9

void ain_ad7175_setup_task_init(){
	ain_ad7175SetupTaskState = ANLGIN_AD7175_SETUP_START;
	ain_ad7175SetupError = ANLGIN_AD7175_NO_ERROR;
	ain_ad7175SetupWait = 0;
	ain_ad7175SetupRegisterIndex = 0;
}

enum ANLGIN_AD7175_SETUP ain_get_ad7175SetupTaskState(void) {
	return ain_ad7175SetupTaskState;
}

void ain_ad7175_setup_task(){
	// This procedure is run under the task manager to perform a sequence of steps
	// configuring the AD7175 A-to-D converter for general use reading Analog Input
	// Voltages.  It operates as a state machine and performs different steps
	// depending on the value of the state variable, ain_ad7175SetupTaskState.

	volatile Uint16 *ad7175FpgaStatus; // pointer for external address to access FPGA
	volatile Uint16 *ad7175FpgaActionCmd; // pointer for external address to access FPGA
	volatile Uint16 *ad7175WriteDataMsw; // pointer for external address to access FPGA
	volatile Uint16 *ad7175WriteDataLsw; // pointer for external address to access FPGA
	volatile Uint16 *ad7175ActionCmd; // pointer for external address to access FPGA

	ad7175FpgaStatus = CPLD_F2_XA(FPGA2_READ_AD7175_STATUS);
	ad7175FpgaActionCmd = CPLD_F2_XA(FPGA2_WRITE_AD7175_ACTION_CMD);
	ad7175WriteDataMsw = CPLD_F2_XA(FPGA2_WRITE_AD7175_DATA_MS_16);
	ad7175WriteDataLsw = CPLD_F2_XA(FPGA2_WRITE_AD7175_DATA_LS_16);
	ad7175ActionCmd = CPLD_F2_XA(FPGA2_WRITE_AD7175_ACTION_CMD);

	switch(ain_ad7175SetupTaskState){

    case ANLGIN_AD7175_SETUP_START:
    	// If FPGA is in read-continuous-conversion mode, then reset FPGA AD7175 machine
    	if ((*ad7175FpgaStatus & 0x0002) != 0) {
    		// "2" means FPGA is in read-continuous-conversion mode
       		*ad7175FpgaActionCmd = 0x0800; //  action command 8 to reset out of read-continuous-conversion mode
    		ain_ad7175SetupWait = 0;
    		ain_ad7175SetupTaskState = ANLGIN_AD7175_SETUP_WAIT_001;
    	}
		ain_ad7175SetupWait = 0;
		ain_ad7175SetupTaskState = ANLGIN_AD7175_SETUP_WAIT_001;
    	break;

    case ANLGIN_AD7175_SETUP_WAIT_001:
    	//  Wait for FPGA to return 0 status
    	if (*ad7175FpgaStatus != 0) {
    		ain_ad7175SetupWait++;
    		if (ain_ad7175SetupWait > 5) {
    			// problem: we looped here 5 times without seeing 0 status
    			ain_ad7175SetupError = ANLGIN_AD7175_ERR_NO_ZERO_STATUS;
    		    ain_ad7175SetupTaskState = ANLGIN_AD7175_SETUP_DONE;
    		    return; // exit without re-launching this task
    		}
    	} else {
        	if (ain_ad7175SetupRegisterIndex <= MAX_AD7175_REGISTER_CONFIG){
    		    ain_ad7175SetupTaskState = ANLGIN_AD7175_SETUP_CONFIG_REG;
        	} else {
		    	ain_ad7175SetupTaskState = ANLGIN_AD7175_SETUP_FPGA_FOR_CONTINUOUS_READ;
        	}
    	}
    	break;

    case ANLGIN_AD7175_SETUP_CONFIG_REG:
    	// Write a value to one of the AD7175's configuration registers
    	// Run thru the ain_ad7175RegisterConfigValues[] table above, each
    	// line in the table gives a register address and the proper config value.
    	// Fetch next values from table, write to AD7175
    	*ad7175WriteDataMsw = 0;
    	*ad7175WriteDataLsw =
    	    ain_ad7175RegisterConfigValues[ain_ad7175SetupRegisterIndex].configValue;
    	*ad7175ActionCmd = 0x0500 // Action 5 is a 16-bit write
    		| ain_ad7175RegisterConfigValues[ain_ad7175SetupRegisterIndex].registerAddr;
    	ain_ad7175SetupRegisterIndex++;
    	ain_ad7175SetupWait = 0;
    	ain_ad7175SetupTaskState = ANLGIN_AD7175_SETUP_WAIT_001;
		break;

    case ANLGIN_AD7175_SETUP_FPGA_FOR_CONTINUOUS_READ: //
   		*ad7175FpgaActionCmd = 0x0700; //  action command 7 to start read-continuous-conversion mode
		ain_ad7175SetupWait = 0;
		ain_ad7175SetupTaskState = ANLGIN_AD7175_SETUP_DONE;
    	return; // with out re-launching this task
//		break;  // commented out "break" to avoid compiler warning

    case ANLGIN_AD7175_SETUP_DONE: //
    	return; // with out re-launching this task
//		break;  // commented out "break" to avoid compiler warning

     default:
     	return; // with out re-launching this task
 //		break;  // commented out "break" to avoid compiler warning
    }

	taskMgr_setTaskRoundRobin(TASKNUM_ain_ad7175_setup_task,0);	 // run this ask again
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       C A N   O P E N   A D 7 1 7 5
//   S I N G L E   R E G I S T E R   R / W   O P E R A T I O N S
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
union CANOPEN16_32 ain_ad7175_single_write_data;

enum CANOPEN_STATUS ain_ad7175_request(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// Here PC sends us a 4-bits action code + 8-bits communication register value
	//  and we send it to AD7175 FPGA code to launch a request to the AD7175.
	//    4-bit action code
	//       (0=8-bit read, 1=16-bit read, 2=24-bit read, 3=32-bit read0
	//       (4=8-bit write, 5=16-bit write, 6=24-bit write)
	//       (7=start continuous conversion read mode, 8= stop c.c.r. mode)
	//    8-bit "communications register" value, to send to ad7175
	//			see spec sheet for ad7175, identifies which ad7175 internal
	//          register to write to, etc.
	// for "write" requests we assume PC has already left 8, 16, or 24-bit data
	//    right justified in ain_ad7175_single_write_data.

	volatile Uint16 *extData; // pointer for external address to access FPGA
	Uint16 action_code;

	// locate 4-bit action code in MboxC
	action_code = *(data+2);
	action_code = action_code >> 8;
	action_code = action_code & 0x000F;

	// for a "write" operation, send data to FPGA
	if ((action_code >= 4) && (action_code <= 6)) {
    	extData = CPLD_F2_XA(FPGA2_WRITE_AD7175_DATA_MS_16);
    	*extData = ain_ad7175_single_write_data.words.msw;
    	extData = CPLD_F2_XA(FPGA2_WRITE_AD7175_DATA_LS_16);
    	*extData = ain_ad7175_single_write_data.words.lsw;
    }
	// send action_code & communications register value
	extData = CPLD_F2_XA(FPGA2_WRITE_AD7175_ACTION_CMD);
    *extData = *(data+2); //MboxC, 4-bits action + 8-bits communications reg

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS ain_ad7175_fetch_status(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// Here PC asks us to return status from FPGA for ad7175

	volatile Uint16 *extData; // pointer for ernal address to access FPGA

	extData = CPLD_F2_XA(FPGA2_READ_AD7175_STATUS);

	*(data+2) = *extData; //MboxC
	*(data+3) = 0;   //MboxD

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS ain_ad7175_fetch_data_read(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// Here PC asks us to return data read from an ad7175 register.
	// the read operation was launched previously via ain_ad7175_request()

	volatile Uint16 *extData; // pointer for ernal address to access FPGA

	extData = CPLD_F2_XA(FPGA2_READ_AD7175_DATA_LS_16);
	*(data+2) = *extData; //MboxC
	extData = CPLD_F2_XA(FPGA2_READ_AD7175_DATA_MS_16);
	*(data+3) = *extData;   //MboxD

	return CANOPEN_NO_ERR;
}
