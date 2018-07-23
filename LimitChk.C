// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   LimitChk.C
//
//   Continuously monitor selected inputs
//   Compare readings against specified limits -- inner and outer, hi, & low
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "CanOpen.h"
#include "LimitChk.h"
#include "CPLD.H"
#include "AnlgIn.H"
#include "Timer0.H"
#include "Log.H"



//struct HI_LOW_IN_OUT_UINT16_LIMITS
//	{
//		Uint16 Hi_Outer;
//		Uint16 Hi_Inner;
//		Uint16 Low_Inner;
//		Uint16 Low_Outer;
//	};

// following 3 items support Analog_In_Comparison PC client
// for testing Anlg_In native/classic conversion
// not part of production product.
struct HI_LOW_IN_OUT_UINT16_LIMITS limChkAnlgInLimits;
struct HI_LOW_IN_OUT_UINT16_LIMITS limChkAnlgInLimitsClassic;
Uint16 limChkAnlgInChannel; // 1-8 which of the 8 Anlog_Inputs are we using

// Set this to 10 if we fail a HVPS_I test,
// Decrement it every mSec, and open the HVPS Relay when we hit 0
Uint16 limChkOpenHvPsRelayTimer;

// One entry (struct)  for each of 8 limit-check-channels,
// eg: for each of 8 limit check tests running simultaneously
struct LIMIT_CHECK_PARAMETERS limitCheckParams[NUM_LIM_CHK_CHANNELS];

// One entry (struct) for each IO port that can be tested by Limit Check
const struct LIMIT_CHECK_INPUTS limitCheckInputs[] = {
		//  enum IO_TYPE         function ptr       offset           read from FPGA address
		//-------------------- -----------------  --------------   ------------------------------------------
/* 00 Anlg_In_A1  */	{IO_TYPE_ANLG_IN_1234,	&limChkAnlgIn1234,	&ain_offsets[0],	(Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + 0) }, // 0
/* 01 Anlg_In_A2  */	{IO_TYPE_ANLG_IN_1234,	&limChkAnlgIn1234,	&ain_offsets[1],	(Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + 1) },
/* 02 Anlg_In_A3  */	{IO_TYPE_ANLG_IN_1234,	&limChkAnlgIn1234,	&ain_offsets[2],	(Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + 2) },
/* 03 Anlg_In_A4  */	{IO_TYPE_ANLG_IN_1234,	&limChkAnlgIn1234,	&ain_offsets[3],	(Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + 3) },
/* 04 Anlg_In_A5  */	{IO_TYPE_ANLG_IN_5678,	&limChkAnlgIn5678,	&ain_offsets[4],	(Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + 4) },
/* 05 Anlg_In_A6  */	{IO_TYPE_ANLG_IN_5678,	&limChkAnlgIn5678,	&ain_offsets[5],	(Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + 5) },  // 5
/* 06 Anlg_In_A7  */	{IO_TYPE_ANLG_IN_5678,	&limChkAnlgIn5678,	&ain_offsets[6],	(Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + 6) },
/* 07 Anlg_In_A8  */	{IO_TYPE_ANLG_IN_5678,	&limChkAnlgIn5678,	&ain_offsets[7],	(Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + 7) },
/* 08 Digtl_In_16 */	{IO_TYPE_DIG_IN_16,		&limChkDigIn16,		NULL,				(Uint16*)CPLD_F2_XA(FPGA2_READ_DIG_IN)},
/* 09 Diff_In_8   */	{IO_TYPE_DIFF_IN_8,		&limChkDiffIn8,		NULL,				(Uint16*)CPLD_F2_XA(FPGA2_READ_DIFF_IN)},

/* 10 Enc A Freq  */	{IO_TYPE_FREQ,	&limChkFreqTest, NULL,	(Uint16*)CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_PERIOD_MS_16)},		// 10
/* 11 Enc I Freq  */	{IO_TYPE_FREQ,	&limChkFreqTest, NULL,	(Uint16*)CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCI_PERIOD_MS_16)},	// 11
/* 12 Enc Dir     */	{IO_TYPE_LEVEL, &limChkLevelTest, NULL,	(Uint16*)CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENC_DIR)},				// 12
/* 13 EncA On Time*/	{IO_TYPE_FREQ,	&limChkFreqTest, NULL,	(Uint16*)CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_ENCA_ON_TIME_MS_16)},	// 13

// Classic Test Station has a Hall signal input machine, but TS3 does not (RC says it's not used).
// I include Hall facilities here to insure that commands from Classic PC software for Hall signal
// input machine don't crash anything.
/* 14 Hall Freq   */	{IO_TYPE_UNDEFINED, &limChkUndefinedTest, 0, 0},		// 14
/* 15 Hall Phase  */	{IO_TYPE_UNDEFINED, &limChkUndefinedTest, 0, 0},		// 15
/* 16 Hall Dir    */	{IO_TYPE_UNDEFINED, &limChkUndefinedTest, 0, 0},		// 16
/* 17 Hall On Time*/	{IO_TYPE_UNDEFINED, &limChkUndefinedTest, 0, 0},		// 17

/* 18 PWM1 Freq   */	{IO_TYPE_FREQ, &limChkFreqTest, NULL, (Uint16*)CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_PERIOD_MS_16)},	// 18
/* 19 PWM1 On Time*/	{IO_TYPE_FREQ, &limChkFreqTest, NULL, (Uint16*)CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM1_ON_TIME_MS_16)},	// 19
/* 20 PWM2 Freq   */	{IO_TYPE_FREQ, &limChkFreqTest, NULL, (Uint16*)CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_PERIOD_MS_16)},	// 20
/* 21 PWM2 On Time*/	{IO_TYPE_FREQ, &limChkFreqTest, NULL, (Uint16*)CPLD_F2_XA(FPGA2_READ_DIGINMACHINE_PWM2_ON_TIME_MS_16)},	// 21


//	     /* 16 */     {PHASE_A_CURRENT_DATA,   &testPhaseIParameter,     &offsets.phase_a_curr_hi,   1},
//	     /* 17 */     {PHASE_B_CURRENT_DATA,   &testPhaseIParameter,     &offsets.phase_b_curr_hi,   1},
//	     /* 18 */     {PHASE_C_CURRENT_DATA,   &testPhaseIParameter,     &offsets.phase_c_curr_hi,   1},
//	     /* 19 */     {(PORT)(&psCurrentLpfAvg), &testPsCurrent,         &offsets.psi,   1},     // LPF PS Current. This offset NOT used; real offset applied in call to convert_raw_psi()
//	     /* 1a */     {PS_VOLTAGE_DATA,        &testPsOrPhaseVoltage,    &offsets.psv,   1},
//	     /* 1b */     {PHASE_A_VOLTAGE_DATA,   &testPsOrPhaseVoltage,    &offsets.phase_a_vol,   1},
//	     /* 1c */     {PHASE_B_VOLTAGE_DATA,   &testPsOrPhaseVoltage,    &offsets.phase_b_vol,   1},
//	     /* 1d */     {PHASE_C_VOLTAGE_DATA,   &testPsOrPhaseVoltage,    &offsets.phase_c_vol,   1},
//	     /* 1e */     {PHASE_A_FREQUENCY,      &testFrequency,           &noOffset,   0},
//	     /* 1f */     {PHASE_B_FREQUENCY,      &testFrequency,           &noOffset,   0},
//	     /* 20 */     {PHASE_C_FREQUENCY,      &testFrequency,           &noOffset,   0},
//	     /* 21 */     {PHASE_A_DUTY_CYCLE,     &testFrequency,           &noOffset,   0},
//	     /* 22 */     {PHASE_B_DUTY_CYCLE,     &testFrequency,           &noOffset,   0},
//	     /* 23 */     {PHASE_C_DUTY_CYCLE,     &testFrequency,           &noOffset,   0},
//	     /* 24 */     {DGTL_SEL1_FREQ_READ,    &performSCTest,           &noOffset,   0},     // ????
//	     /* 25 */     {PS24V_CURRENT_A_TO_D,   &test24VoltPsCurrent,     &offsets.ps24v_current,   1},
//	     /* 26 */     {&phase_I_HC_Results.phaseI_A, &testPhaseIParameter, &offsets.phase_a_curr_hc, 1},
//	     /* 27 */     {&phase_I_HC_Results.phaseI_B, &testPhaseIParameter, &offsets.phase_b_curr_hc, 1},
//	     /* 28 */     {&phase_I_HC_Results.phaseI_C, &testPhaseIParameter, &offsets.phase_c_curr_hc, 1}


/* 22 PH_A_I      */	{IO_TYPE_UNDEFINED, 0, 0, 0},  // 22
/* 23 PH_B_I      */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 24 PH_C_I      */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 25 PS_I        */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 26 PS_V        */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 27 PH_A_V      */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 28 PH_B_V      */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 29 PH_C_V      */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 30 PH_A Freq   */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 31 PH_B Freq   */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 32 PH_C Freq   */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 33 PH_A DutyC  */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 34 PH_B DutyC  */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 35 PH_C DutyC  */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 36 SC Freq Test*/	{IO_TYPE_UNDEFINED, 0, 0, 0},
//	     /* 0x24 */     {DGTL_SEL1_FREQ_READ,    &performSCTest,           &noOffset,   0},     // ????
/* 37 PS24V_I     */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 38 PH_A_I_HC   */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 39 PH_B_I_HC   */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 40 PH_C_I_HC   */	{IO_TYPE_UNDEFINED, 0, 0, 0},

// --- above ^ are compatible w Classic Test Station
// --- below v new for TS3

/* 41 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 42 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 43 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 44 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 45 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 46 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 47 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 48 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 49 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0},
/* 50 TBD         */	{IO_TYPE_UNDEFINED, 0, 0, 0}
};


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       I N I T
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void limchkInit(void){
// Called from main() at powerup
   Uint16 i;

   for (i=0; i<NUM_LIM_CHK_CHANNELS; i++) {

	   limchkInitOneChannel(i); // (below)
   }
}


void limchkInitOneChannel(Uint16 limitCheckChannel){

	   limitCheckParams[limitCheckChannel].enableTest 			= LIMCHK_TEST_DISABLED;
	   limitCheckParams[limitCheckChannel].reqEnable 			= LIMCHK_NO_PENDING_REQ;
	   limitCheckParams[limitCheckChannel].reqSync 				= LIMCHK_NO_PENDING_SYNC;

	   limitCheckParams[limitCheckChannel].limitCheckState 		= LCSM_PERFORM_TEST_END;
	   limitCheckParams[limitCheckChannel].limitCheckInput 		= (enum LIMIT_CHECK_INPUT_INDEX)0;

	   limitCheckParams[limitCheckChannel].timeStart 			= 0;
	   limitCheckParams[limitCheckChannel].timeEnd 				= 0;
	   limitCheckParams[limitCheckChannel].allowedFails 		= 0;
	   limitCheckParams[limitCheckChannel].detectedFails 		= 0;
	   limitCheckParams[limitCheckChannel].noHalt 				= 0;
	   limitCheckParams[limitCheckChannel].ioType 				= IO_TYPE_UNDEFINED;
	   limitCheckParams[limitCheckChannel].limits.Hi_Outer 		= 0x8000;
	   limitCheckParams[limitCheckChannel].limits.Hi_Inner 		= 0x8000;
	   limitCheckParams[limitCheckChannel].limits.Low_Inner 	= 0x8000;
	   limitCheckParams[limitCheckChannel].limits.Low_Outer 	= 0x8000;
	   limitCheckParams[limitCheckChannel].measValue			= 0x8000L;

	   // minValue, maxValue, avgValue, sumValue, sumCount
	   // may get different initializations based on ioType
	   limChkBackgroundMinMaxAvgInit(&limitCheckParams[limitCheckChannel]);

	   limitCheckParams[limitCheckChannel].testStatus 			= NOT_TESTING;
	   limitCheckParams[limitCheckChannel].comparisonResult 	= LCC_WITHIN_INNER_LIMITS;

}

void limchkPreTestInit(Uint16 limitCheckChannel){
// Called from State Machine to initialize some limitCheckParams[] values
// as we are starting a test

	limitCheckParams[limitCheckChannel].detectedFails 		= 0;
	limitCheckParams[limitCheckChannel].measValue	 		= 0L;
	limitCheckParams[limitCheckChannel].testStatus 			= NOT_TESTING;
	limitCheckParams[limitCheckChannel].comparisonResult 	= LCC_WITHIN_INNER_LIMITS;

	// minValue, maxValue, avgValue, sumValue, sumCount
	// may get different initializations based on ioType
	limChkBackgroundMinMaxAvgInit(&limitCheckParams[limitCheckChannel]);
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       C O N V E R T   N A T I V E / C L A S S I C
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Uint32 limchkConvertClassicToNative(Uint16 classicValue, enum IO_TYPE ioType) {
//called: limitLowNative = limchkConvertClassicToNative(limitLowClassic,ioType);
//Different conversions apply for different types of I/O

	int ain_classic_int;
	int m2;
	int result;
	Uint32 x_dbl;
	Uint32 y_dbl;

	if (ioType == IO_TYPE_ANLG_IN_1234) {
       // REF on 2812 Multiplication:
       // "C/C++ Code Access to the Upper 16 Bits of 16-Bit Multiply"
       // p139, spru514e_TMS320C28x Compiler and Linker Users Guide.pdf

   		// <ain_limit_value_native> = (<ain_limit_value_classic> / (0x3565/0x10000)) + 0x8000
   		//                          = (<ain_limit_value_classic> * (0x10000 / 0x3565) + 0x8000
   		//                          = (<ain_limit_value_classic> * (    4.79449     ) + 0x8000
   		//                          = (<ain_limit_value_classic> * (0x4CB6 / 0x1000) + 0x8000

   		ain_classic_int = (int)(classicValue);
   		m2 = (int)0x4CB6;
   		result = ((long)ain_classic_int * (long)m2) >> 12; // event log clocked this at 6.4 uS (-5.86 for event_log) -> 0.54 uS
   		return (Uint32)(result + 0x8000);

   } else if (ioType == IO_TYPE_ANLG_IN_5678) {
		// <ain_limit_value_native> = 0x8000 - (<ain_limit_value_classic> / (0x3BAB/0x10000))
		//                          = 0x8000 - (<ain_limit_value_classic> * (0x10000/0x3BAB))
		//                          = 0x8000 - (<ain_limit_value_classic> * (  4.290 ))
		//                          = 0x8000 - (<ain_limit_value_classic> * (0x44A5/0x1000)) <- By Formula
		ain_classic_int = (int)(classicValue);
		m2 = (int)0x44A5; // use formula value, not hueristic
		result = ((long)ain_classic_int * (long)m2) >> 12; // event log clocked this at 6.4 uS (-5.86 for event_log) -> 0.54 uS
		return (Uint32)(0x8000 - (Uint16)result);

	} else if (ioType == IO_TYPE_DIG_IN_16) {
		// Internal representation of digital I/O is bit-reversed from Classic Test Station
		return (Uint32)(classicValue ^ 0xFFFF);

	} else if (ioType == IO_TYPE_DIFF_IN_8) {
		return (Uint32)classicValue;

	} else if (ioType == IO_TYPE_FREQ) {
		//TS3_PwmFreqCount = ((75 / 20) * (Classic_ PwmFreqCount + 1)) – 1
		//    = ((75 / 20) * (Classic_ PwmFreqCount)) + 2.75
		// note: 75/20 = 3.75
		x_dbl = (Uint32)classicValue + 1L;
		y_dbl = x_dbl;		// x1
		y_dbl += x_dbl;		// x2
		y_dbl += x_dbl;		// x3
		x_dbl = (x_dbl >> 2) & 0x3FFFL;
		y_dbl += x_dbl;		// x3.25
		y_dbl += x_dbl;		// x3.5
		y_dbl += x_dbl;		// x3.75
		y_dbl -= 1L;
		return y_dbl;

   } else if (ioType == IO_TYPE_LEVEL) {
	   return (Uint32)classicValue;

   } else {
	   // NEED TO FLESH THIS OUT FOR OTHER VALUES OF ioType.
   }

	return 0;
}

Uint16 limchkConvertNativeToClassic(Uint32 nativeValue, enum IO_TYPE ioType) {
//called: classicValue = limchkConvertNativeToClassic(nativeValue,ioType);
//Different conversions apply for different types of I/O
//Timing measurement in Log.C: 3.25uSec to call this
// Note that Native Value is a 32-bit, whereas classicValue is a 16-bit

	int ain_native_int;
	int m2;
	int result;
	union CANOPEN16_32 nativeVal;

	nativeVal.all = nativeValue;

	if (ioType == IO_TYPE_ANLG_IN_1234) {
	   // <ain_value_classic> = (<ain_calibrated_native> - 0x8000) * (0x3565/0x10000)
	   //                     = (<ain_calibrated_native> - 0x8000) * 0.20857
	   // REF on 2812 Multiplication:
	   // "C/C++ Code Access to the Upper 16 Bits of 16-Bit Multiply"
	   // p139, spru514e_TMS320C28x Compiler and Linker Users Guide.pdf

	   ain_native_int = nativeVal.words.lsw - 0x8000;
	   m2 = (int)0x3565;
	   result = ((long)ain_native_int * (long)m2) >> 16; // event log clocked this at 6.4 uS (-5.86 for event_log) -> 0.54 uS
	   return ((Uint16)result);

	} else if (ioType == IO_TYPE_ANLG_IN_5678) {

	   // <ain_value_classic> = (0x8000 - <ain_calibrated_native>) * (0x1E0C/0x8000)
	   //                     = (0x8000 - <ain_calibrated_native>) * 0.23474
	   ain_native_int = 0x8000 - nativeVal.words.lsw;
	   m2 = (int)0x1E0C;
	   result = ((long)ain_native_int * (long)m2) >> 15;
	   return ((Uint16)result);

	} else if (ioType == IO_TYPE_DIG_IN_16) {
		// Internal representation of digital Input is bit-reversed from Classic Test Station
		return nativeVal.words.lsw ^ 0xFFFF;

	} else if (ioType == IO_TYPE_DIFF_IN_8) {
		return nativeVal.words.lsw;

	} else if (ioType == IO_TYPE_FREQ) {
		//Classic_PwmFreqCount = ((20 / 75) * (TS3_ PwmFreqCount + 1)) – 1
		//    = ((20 / 75) * (TS3_ PwmFreqCount)) - 0.73
		// note: 20/75 = 0.2667 = 0x2222 / 0x8000 = 8738 / 32768
		m2 = (int)0x2222;
		nativeVal.all += 1L;
		result = ((long)nativeVal.all * (long)m2) >> 15;
		//return ((Uint16)result - 1); // heuristically, when we subtract one, we have creep as we
		                               // write and re-read limit values from the client
		return ((Uint16)result);

	} else if (ioType == IO_TYPE_LEVEL) {
		return nativeVal.words.lsw;

	} else {
		   // NEED TO FLESH THIS OUT FOR OTHER VALUED OF ioType.
	}
	return 0;
}



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       L I M I T _ C H E C K _C O M P A R I S O N
//       run via function pointer in const struct LIMIT_CHECK_INPUTS limitCheckInputs[]
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum LIMIT_CHECK_COMPARISON limChkAnlgIn1234(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* ain_offset,
		                                     Uint16* readAnlgInFpga, Uint32* measValue){
	// This compares present Analog In voltage against 4 pre-configured limits.
	// If measured value is outside outer limits, we declare an immediate failure
	// If measured value is outside inner limits, we increment a detectedFails count,
	// and declare a failure only if detectedFails count exceeds allowedFails parameter.
	// Returns a LIMIT_CHECK_COMPARISON enum value describing result of the comparison.
	// For speed, we have separate comparison routines for Anlg_In_1,2,3,4 vs 5,6,7,8

	volatile Uint16 ain_value;
	Uint16 offset;

	//Fetch ADC input value via FPGA
	ain_value = *(readAnlgInFpga);
	// subtract the calibration offset from the raw value, and do it without
	// wrapping around the bounds of our 16-bit values
	offset = *ain_offset;
    if ((offset & 0x8000) && (offset < ain_value)) {
    	ain_value = 0xFFFF;
    } else if ((!(offset & 0x8000)) && (offset > ain_value)) {
    	ain_value = 0x0000;
    } else {
    	ain_value -= offset;
    }

    *measValue = (Uint32)ain_value; // store present measured value in limitCheckParams struct

    //Compare ADC value to limits
	// for channels 0,1,2,3, voltage runs low to hi, for ADC values low to hi
 	if (ain_value > InLimits->Hi_Outer) {
 		return LCC_ABOVE_OUTER_HI_LIMIT;
 	} else if (ain_value > InLimits->Hi_Inner) {
 		return LCC_ABOVE_INNER_HI_LIMIT; //MboxC
 	} else if (ain_value < InLimits->Low_Outer) {
 		return LCC_BELOW_OUTER_LOW_LIMIT; //MboxC
 	} else if (ain_value < InLimits->Low_Inner) {
 		return LCC_BELOW_INNER_LOW_LIMIT; //MboxC
 	}
	return LCC_WITHIN_INNER_LIMITS; //MboxC
}

enum LIMIT_CHECK_COMPARISON limChkAnlgIn5678(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* ain_offset,
		                                     Uint16* readAnlgInFpga, Uint32* measValue){
	// This compares present Analog In voltage against 4 pre-configured limits.
	// If measured value is outside outer limits, we declare an immediate failure
	// If measured value is outside inner limits, we increment a detectedFails count,
	// and declare a failure only if detectedFails count exceeds allowedFails parameter.
	// Returns a LIMIT_CHECK_COMPARISON enum value describing result of the comparison.
	// For speed, we have separate comparison routines for Anlg_In_1,2,3,4 vs 5,6,7,8

	volatile Uint16 ain_value;
	Uint16 offset;

	//Fetch ADC input value via FPGA
	ain_value = *(readAnlgInFpga);
	// subtract the calibration offset from the raw value, and do it without
	// wrapping around the bounds of our 16-bit values
	offset = *ain_offset;
    if ((offset & 0x8000) && (offset < ain_value)) {
    	ain_value = 0xFFFF;
    } else if ((!(offset & 0x8000)) && (offset > ain_value)) {
    	ain_value = 0x0000;
    } else {
    	ain_value -= offset;
    }

    *measValue = (Uint32)ain_value; // store present measured value in limitCheckParams struct

    //Compare ADC value to limits
    // for channels 5,6,7,8 voltage runs hi to low, for ADC values low to hi
    if (ain_value < InLimits->Hi_Outer) {
       return LCC_ABOVE_OUTER_HI_LIMIT;
    } else if (ain_value < InLimits->Hi_Inner) {
       return LCC_ABOVE_INNER_HI_LIMIT;
    } else if (ain_value > InLimits->Low_Outer) {
       return LCC_BELOW_OUTER_LOW_LIMIT;
    } else if (ain_value > InLimits->Low_Inner) {
       return LCC_BELOW_INNER_LOW_LIMIT;
    }

    return LCC_WITHIN_INNER_LIMITS;
}

enum LIMIT_CHECK_COMPARISON limChkDigIn16(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
		                                     Uint16* readInFpga, Uint32* measValue){
// This reads 16 (non differential) digital input values in a 16-bit word and performs a comparison

// FYI, in TS3, native measValue for DigitalInputs is inverted compared to classic test station
// for ex, if digital input voltage is < threshold voltage, TS3 reports a 0,
//    whereas classic test station reports 1
// To handle that, TS3 Limit Check code performs conversions on measValue and Limits,
//    sent and received in CAN messages related to Digital Inputs.
// This does not apply to differential digital inputs, as TS3 native values
//    are the same polarity as for classic test station differential digital inputs.

// Limits & comparison:
// if (measValue & Low_Inner (limit) == High_Inner (limit))
//   then PASS, otherwise Immediate Fail
// But the above comparison is expressed in terms of classic compatible data format.
// To do the same comparison with our inverted data, we must do
// if (~measValue & ~Low_Inner (limit) == ~High_Inner (limit))
//    remember (~A & ~B) === ~(A | B), so we can reformulate that as
// if (~(measValue | Low_Inner (limit))) == ~High_Inner (limit))
//    or, since both sides are inverted, we can also reformulate that as
// if (measValue | Low_Inner (limit)) == High_Inner (limit))

	volatile Uint16 digInValue16;

	//Fetch Digital Input values via FPGA
	digInValue16 = (*readInFpga);

	//Store them in limit check params measured value, which holds native format value
    *measValue = (Uint32)digInValue16; // store present measured value in limitCheckParams struct

	if ((digInValue16 | InLimits->Low_Inner) != InLimits->Hi_Inner) {
		return LCC_ABOVE_OUTER_HI_LIMIT;
	}

    return LCC_WITHIN_INNER_LIMITS; //MboxC
}

enum LIMIT_CHECK_COMPARISON limChkDiffIn8(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
		                                     Uint16* readInFpga, Uint32* measValue){
// This reads 16 differential digital input values in a 16-bit word and performs a comparison
// Limits & comparison:
// if (measValue & Low_Inner (limit) == High_Inner (limit))
//   then PASS, otherwise Immediate Fail
// Note: whereas the (non differential) Digital Inputs for TS3 report native bit values that are inverted
// compared to the classic test station, no such inversion occurs with the differential inputs.

	volatile Uint16 diffInValue8;

	//Fetch Digital Input values via FPGA
	diffInValue8 = ((*readInFpga) & 0xFF);

	//Store them in limit check params measured value
    *measValue = (Uint32)diffInValue8; // store present measured value in limitCheckParams struct

	if ((diffInValue8 & InLimits->Low_Inner) != InLimits->Hi_Inner) {
		return LCC_ABOVE_OUTER_HI_LIMIT;
	}

    return LCC_WITHIN_INNER_LIMITS; //MboxC
}


enum LIMIT_CHECK_COMPARISON limChkFreqTest(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
        										Uint16* readInFpga, Uint32* measValue){
// This performs a comparison of "frequency" values.
// Technically our measurements are of the period of a waveform, the inverse of the frequency.
// Here we read 2 consecutive 16-bit values from the FPGA, getting the MS then the LS word of the period measurement.
// When we read the MS_16 first, the FPGA caches the corresponding value for the LS_16
// guaranteeing that when we read the LS_16 value later, it is from the same 32-bit value as the MS_16.
// Since both Frequency and Duty Cycle are reported as time durations -- # of clock counts --
// we use this same limChkFreqTest() for both Freq and Duty Cycle
//
//	PWM / ENC DUTY CYCLE CONFUSION
//
//	Reading PWM Duty Cycle or "On Time", the Classic test station reports the duration of the OFF portion of the PWM wave.
// 	For compatibility, the TS3 test station also reports the duration of the OFF portion (rather than the ON portion) of
//	the measured PWM wave.
//  This is non-intuitive and can be a source of confusion, though all of our legacy PC client software works with it this way.
//   Here's what you need to keep in mind:  In commands that set the "Duty Cycle" for digital machine output, you must
//	specify the duration of the ON time, whereas when you read the measured "Duty Cycle" from the digital machine input,
//	it gives you the duration of the OFF time of the PWM waveform.

	union LC16_32 limChkPeriod;

	limChkPeriod.words.msw = *readInFpga;		//PERIOD_MS_16);
	limChkPeriod.words.lsw = *(readInFpga+1);	//PERIOD_LS_16);

	// Store Measured Value
	*measValue = limChkPeriod.all;

	// Compare MeasuredValue against Inner Limits
	// if we are outside of those, then declare LIMITS_FAILED
	// otherwise TEST_PASSED.
	// If the test fails, we return OUTER_LIMIT failure enum values, because those
	// tell the caller to declare a hard  failure, rather than continuing to
	// count up soft  failures.
	if (*measValue >= InLimits->Hi_Inner) {
		return LCC_ABOVE_OUTER_HI_LIMIT;
	} else if (*measValue <= InLimits->Low_Inner) {
		return LCC_BELOW_OUTER_LOW_LIMIT;
	}

    return LCC_WITHIN_INNER_LIMITS; //MboxC
}

enum LIMIT_CHECK_COMPARISON limChkLevelTest(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
        										Uint16* readInFpga, Uint32* measValue){
// This performs a comparison of "level" values, like direction for Encoders or Halls, or Hall Phase.
// These are all single bit values, ls bit or word, comparison should be pretty easy.
// We read the 16-bit measValue
// Bitwise And (&) it with the Inner_Low_Limit
// Pass if that equals (==) Inner_Hi_Limit
	volatile Uint16 levelInValue;

	//Fetch Level Input values via FPGA
	levelInValue = ((*readInFpga) & 0xFF);

	//Store them in limit check params measured value
    *measValue = (Uint32)levelInValue; // store present measured value in limitCheckParams struct

	if ((levelInValue & InLimits->Low_Inner) != InLimits->Hi_Inner) {
		return LCC_ABOVE_OUTER_HI_LIMIT;
	}

    return LCC_WITHIN_INNER_LIMITS; //MboxC
}

enum LIMIT_CHECK_COMPARISON limChkUndefinedTest(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
        										Uint16* readInFpga, Uint32* measValue){
// This is a "dummy" comparison-test procedure, assigned to entries in the
// LIMIT_CHECK_INPUTS table for I/O Ports (aka inputs, or "Which Input") that we don't test.

	return LCC_ABOVE_OUTER_HI_LIMIT; // always return a failure
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       C A N   O P E N   R E S P O N S E S   F O R   L I M I T _ C H E C K   P R O P E R
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS limchkSendLimits(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns two 16-bit test limit values, as a 32-bit response.
	// This returns either the Inner (Hi & Low) or Outer (Hi & Low) limits based on CAN Subindex.
	// This returns limit values from 1 of 8 Limit Check Channels, based on CAN Index
	// Limit data is sent/received in classic compatible units, and may require translation
	// into or  out of TS3 native units.  Furthermore, the translation may vary depending
	// on the type of I/O, for example different translations for Analog_In vs HV_PS_I.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 subindex;
	Uint16 canIndex;
	Uint16 limitCheckChannel;
	Uint16 limitLowClassic;
	Uint16 limitHiClassic;
	Uint32 limitLowNative;
	Uint32 limitHiNative;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;
	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1;

	// Some error checking
	if ((limitCheckChannel > 7) || (subindex > 4) || (subindex < 3)){
		   return CANOPEN_LIMCHK_002_ERR;
    }

    // Fetch 2 16-bit limits from limitCheckParams (array of structs) to local variables
	if ((subindex == 3)){
		// Inner Limits -- Hi limit is packed as MS Word
		limitHiNative = limitCheckParams[limitCheckChannel].limits.Hi_Inner;
		limitLowNative = limitCheckParams[limitCheckChannel].limits.Low_Inner;
	} else {
		// Outer Limits -- Hi limit is packed as MS Word
		limitHiNative = limitCheckParams[limitCheckChannel].limits.Hi_Outer;
		limitLowNative = limitCheckParams[limitCheckChannel].limits.Low_Outer;
	}

	// translate TS3 Native values stored in internal tables to Classic compatible values for transmission
	// Potentially, we have to do different translations based on the type of I/O
	limitHiClassic = limchkConvertNativeToClassic(limitHiNative,limitCheckParams[limitCheckChannel].ioType);
	limitLowClassic = limchkConvertNativeToClassic(limitLowNative,limitCheckParams[limitCheckChannel].ioType);

	// Inner Limits -- Hi limit is packed as MS Word
	*(data+3) = limitHiClassic; // Mbox D
	*(data+2) = limitLowClassic; // Mbox C

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkRecvLimits(const struct CAN_COMMAND* can_command, Uint16* data){
	// This function receives two 16-bit test limit values, as  32-bit data.
	// This saves them as either the Inner (Hi & Low) or Outer (Hi & Low) limits based on CAN Subindex.
	// This saves these limit values for one of the 8 Limit Check Channels, based on CAN Index
	// Limit data is sent/received in classic compatible units, and may require translation
	// into or  out of TS3 native units.  Furthermore, the translation may vary depending
	// on the type of I/O, for example different translations for Analog_In vs HV_PS_I.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 subindex;
	Uint16 canIndex;
	Uint16 limitCheckChannel;
	Uint16 limitLowClassic;
	Uint16 limitHiClassic;
	Uint32 limitLowNative;
	Uint32 limitHiNative;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;
	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1;

	// Some error checking
	if ((limitCheckChannel > 7) || (subindex > 4) || (subindex < 3)){
		   return CANOPEN_LIMCHK_002_ERR;
    }

	// fetch Classic limits from received CAN message
	limitHiClassic = *(data+3); // Mbox D
	limitLowClassic = *(data+2); // Mbox C

	// translate Classic compatible values used for transmission to TS3 Native values stored in internal tables
	// Potentially, we have to do different translations based on the type of I/O
	limitHiNative = limchkConvertClassicToNative(limitHiClassic,limitCheckParams[limitCheckChannel].ioType);
	limitLowNative = limchkConvertClassicToNative(limitLowClassic,limitCheckParams[limitCheckChannel].ioType);

	// store translated native limit values in internal table
	if ((subindex == 3)){
		// Inner Limits -- Hi limit is packed as MS Word
		limitCheckParams[limitCheckChannel].limits.Hi_Inner = limitHiNative;
		limitCheckParams[limitCheckChannel].limits.Low_Inner = limitLowNative;
	} else {
		// Outer Limits -- Hi limit is packed as MS Word
		limitCheckParams[limitCheckChannel].limits.Hi_Outer = limitHiNative;
		limitCheckParams[limitCheckChannel].limits.Low_Outer = limitLowNative;
	}


	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkSendWhichInput(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns two 16-bit values, as  32-bit data.
	// Values are retrieved from limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// LS word is an index into an array of structs, identifying which I/O input is used in this test.
	// MS word is "no_halt," 0-> stop test on failure, 1->continue running test after failure.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7){
		   return CANOPEN_LIMCHK_002_ERR;
    }

	*(data+2) = limitCheckParams[limitCheckChannel].limitCheckInput; // Mbox C
	*(data+3) = limitCheckParams[limitCheckChannel].noHalt; // Mbox D

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkRecvWhichInput(const struct CAN_COMMAND* can_command, Uint16* data){
	// This function receives two 16-bit values, as  32-bit data.
	// LS word is an index into an array of structs, identifying which I/O input is used in this test.
	// MS word is "no_halt," 0-> stop test on failure, 1->continue running test after failure.
	// Values are saved to limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;
	enum IO_TYPE ioType;
	enum LIMIT_CHECK_INPUT_INDEX limitCheckInput;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7) {
		   return CANOPEN_LIMCHK_002_ERR;
    }

	limitCheckInput = (enum LIMIT_CHECK_INPUT_INDEX)(*(data+2)); // Mbox C
	limitCheckParams[limitCheckChannel].limitCheckInput = limitCheckInput;
	limitCheckParams[limitCheckChannel].noHalt = *(data+3); // Mbox D

	// Store ioType value in limitCheckParams, based on limitCheckInput
	// The ioType value is used to select an appropriate translation algorithm
	// converting limits between Classic and Native format.
	ioType = limitCheckInputs[limitCheckInput].ioType;
	limitCheckParams[limitCheckChannel].ioType = ioType;

	return CANOPEN_NO_ERR;
}


enum CANOPEN_STATUS limchkSendMeasValueClassic(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns one 16-bit value.
	// Values are retrieved from limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// LS word is he measStatus value, compatible with Classic Test Station.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;
	Uint32 measValueNative;
	Uint16 measValueClassic;
	enum IO_TYPE ioTypeApplied;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7){
		   return CANOPEN_LIMCHK_002_ERR;
    }

	// fetch values of interest from the limitCheckParams[] array of structs
	ioTypeApplied = limitCheckParams[limitCheckChannel].ioType;
	measValueNative = limitCheckParams[limitCheckChannel].measValue;

	// use conversion routines to get classic compatible values from native values
	measValueClassic = limchkConvertNativeToClassic(measValueNative, ioTypeApplied);

	*(data+2) = measValueClassic; // Mbox C
	*(data+3) = 0; // Mbox D

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkSendTestStatus(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns two 16-bit values, as  32-bit data.
	// Values are retrieved from limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// LS word is he testStatus value, compatible with Classic Test Station.
	// MS word is measValue -- most recent measurement taken for the purpose of the test.
	// measValue is converted from Native to Classic.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;
	Uint32 measValueNative;
	Uint16 measValueClassic;
	enum IO_TYPE ioTypeApplied;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7){
		   return CANOPEN_LIMCHK_002_ERR;
    }

	// fetch values of interest from the limitCheckParams[] array of structs
	ioTypeApplied = limitCheckParams[limitCheckChannel].ioType;
	measValueNative = limitCheckParams[limitCheckChannel].measValue;

	// use conversion routines to get classic compatible values from native values
	measValueClassic = limchkConvertNativeToClassic(measValueNative, ioTypeApplied);

	*(data+2) = limitCheckParams[limitCheckChannel].testStatus; // Mbox C
	*(data+3) = measValueClassic; // Mbox D

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkSendTimeStartEnd(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns two 16-bit values, as  32-bit data.
	// Values are retrieved from limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// LS word is Time_Start -- option to delay a number of test cycles before acting on comparisons.
	// MS word is Time_End -- alternatives to end after a finit number of tests or continue indefinitely.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7){
		   return CANOPEN_LIMCHK_002_ERR;
    }

	*(data+2) = limitCheckParams[limitCheckChannel].timeStart; // Mbox C
	*(data+3) = limitCheckParams[limitCheckChannel].timeEnd; // Mbox D

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkRecvTimeStartEnd(const struct CAN_COMMAND* can_command, Uint16* data){
	// This function receives two 16-bit values, as  32-bit data.
	// LS word is Time_Start -- option to delay a number of test cycles before acting on comparisons.
	// MS word is Time_End -- alternatives to end after a finit number of tests or continue indefinitely.
	// Values are saved to limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7) {
		   return CANOPEN_LIMCHK_002_ERR;
    }

	limitCheckParams[limitCheckChannel].timeStart = *(data+2); // Mbox C
	limitCheckParams[limitCheckChannel].timeEnd = *(data+3); // Mbox D

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkSendFails(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns two 16-bit values, as  32-bit data.
	// Values are retrieved from limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// LS word is allowedFails -- allow this many failures of inner limits before declaring a failed test.
	// MS word is detectedFails -- how many failures of inner limits have occurred since starting the test.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7){
		   return CANOPEN_LIMCHK_002_ERR;
    }

	*(data+2) = limitCheckParams[limitCheckChannel].allowedFails; // Mbox C
	*(data+3) = limitCheckParams[limitCheckChannel].detectedFails; // Mbox D

	return CANOPEN_NO_ERR;
}


enum CANOPEN_STATUS limchkSendMinMaxValue(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns two 16-bit values, as  32-bit data.
	// Values are retrieved from limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// LS word is minValue -- minimum of the measValue measured during the test.
	// MS word is maxValue  -- maximum of the measValue measured during the test.
	// Convert min/max values from native to classic-compatible before returning them.
	// For some ioTypes, such as IO_TYPE_ANLG_IN_5678, max and min max be reversed . . .
	// meaning: since there is a (-1) multiplier in the conversion the minimum native value
	// is actually the maximum voltage.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;
	Uint16 minClassic;
	Uint16 maxClassic;
	Uint32 minNative;
	Uint32 maxNative;
	enum IO_TYPE ioTypeApplied;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7){
		   return CANOPEN_LIMCHK_002_ERR;
    }

	// fetch values of interest from the limitCheckParams[] array of structs
	ioTypeApplied = limitCheckParams[limitCheckChannel].ioType;
	minNative = limitCheckParams[limitCheckChannel].minValue;
	maxNative = limitCheckParams[limitCheckChannel].maxValue;

	// use conversion routines to get classic compatible values from native values
	minClassic = limchkConvertNativeToClassic(minNative, ioTypeApplied);
	maxClassic = limchkConvertNativeToClassic(maxNative, ioTypeApplied);

	if (ioTypeApplied == IO_TYPE_ANLG_IN_5678) {
		//since there is a (-1) multiplier in the conversion, the minimum native value
		// is actually the maximum voltage
		*(data+2) = maxClassic; // Mbox C = Min to Client
		*(data+3) = minClassic; // Mbox D = Max to Client
	} else {
		*(data+2) = minClassic; // Mbox C = Min to Client
		*(data+3) = maxClassic; // Mbox D = Max to Client
	}

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkSendAvgValue(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns one 16-bit value
	// Values are retrieved from limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// LS word is avgValue -- average of measValue measured during the test.
	// *** NOT YET DECIDED HOW TO CALCULATE AVERAGE *******
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;
	Uint16 avgClassic;

	LOG_LIMCHK_ADDTOLOG(LOG_EVENT_LIMCHK_SEND_AVG,0x0001);
	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7){
		   return CANOPEN_LIMCHK_002_ERR;
    }

	// Computes Average
	// 2.25 uSec to call this
	limChkBackgroundComputeAvg(&limitCheckParams[limitCheckChannel]);

	LOG_LIMCHK_ADDTOLOG(LOG_EVENT_LIMCHK_SEND_AVG,0x0002);

	// 3.25 uSec to call this
	avgClassic = limchkConvertNativeToClassic(limitCheckParams[limitCheckChannel].avgValue,
						limitCheckParams[limitCheckChannel].ioType);


	*(data+2) = avgClassic; // Mbox C
	*(data+3) = 0; // Mbox D
	LOG_LIMCHK_ADDTOLOG(LOG_EVENT_LIMCHK_SEND_AVG,0x0003);

	return CANOPEN_NO_ERR;
}


enum CANOPEN_STATUS limchkRecvEnable(const struct CAN_COMMAND* can_command, Uint16* data){
	// This function receives one 16-bit value.
	// LS word is  enum LC_TEST_ENABLE enableTest.
	// A change in enableTest may call for a state change in the limit check state-machine.
	// so any possible changes are handled through a routine that is aware of the state machine
	// and acts accordingly.
	// Values are saved to limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7) {
		   return CANOPEN_LIMCHK_002_ERR;
    }

	// Save a REQUEST to set the ENABLE / DISABLE state
	// Later the State Machine Code makes the actual change to enum LC_TEST_ENABLE enableTest.
	if ((enum LC_TEST_ENABLE)(*(data+2)) == LIMCHK_TEST_ENABLED) { // (*(data+2)) -- Mbox C
		limitCheckParams[limitCheckChannel].reqEnable = LIMCHK_REQ_TEST_ENABLE;
	} else if ((enum LC_TEST_ENABLE)(*(data+2)) == LIMCHK_TEST_DISABLED) {
		limitCheckParams[limitCheckChannel].reqEnable = LIMCHK_REQ_TEST_DISABLE;
	}
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkRecvSync(const struct CAN_COMMAND* can_command, Uint16* data){
	// No data received, just a command to "Synchronize" certain tests.
	// We just save the request in the limitCheckParams[] and let the state machine code handle it.
	// Basically "Synchronize" has to do with setting up multiple tests with possibly
	// different start/end times, then synchronizing them so they are all running on the same clock.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 i;

	for (i=0; i< 8; i++) {
		// Store the LIMCHK_REQ_SYNC in two cases:
		// 1) if limitCheckState == LCSM_WAIT_FOR_SYNC, meaning we are already waiting for Sync
		// 2) if reqEnable == LIMCHK_REQ_TEST_ENABLE we got a request t enable the test,
		//    but the state machine hasn't yet acted on it.  EG the Sync request came
		//    within 1mSec after the enable request
		if ((limitCheckParams[i].limitCheckState == LCSM_WAIT_FOR_SYNC)
		  || (limitCheckParams[i].reqEnable == LIMCHK_REQ_TEST_ENABLE)){
			limitCheckParams[i].reqSync = LIMCHK_REQ_SYNC;
		}
	}

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS limchkResetOneChannel(const struct CAN_COMMAND* can_command, Uint16* data){
	// No data received, just a command to "Reset" one of the tests.
	// We call a routine to fill limitCheckParams[limitCheckChannel] with default values.
	// Values are saved to limitCheckParams[] for one of the 8 Limit Check Channels, based on CAN Index
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 canIndex;
	Uint16 limitCheckChannel;

	canIndex = (((*data) >> 8) & 0x00FF) + 0x2000; // ls byte of *data is ls byte of canIndex
	                                               // ms byte of canIndex i 0x20
	limitCheckChannel = canIndex - CAN_INDEX_OF_LIM_CHK_CHANNEL_1; // index into limitCheckParams[]

	// Some error checking
	if (limitCheckChannel > 7) {
		   return CANOPEN_LIMCHK_002_ERR;
    }

	// Disable the test and set the state to LCSM_PERFORM_TEST_END
	limChkStateMachineResetOneChannel(limitCheckChannel);

	return CANOPEN_NO_ERR;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       S T U B B S   T O   B E   C O M P L E T E D   W H E N   P O W E R   M O D U L E   I S   D O N E
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void limChkOpenHvPsRelay(void) {

	// Add code here to open the HV PS Relay

	// ***************** Add code here ******************


}

void limChkHvPsShutdown(void) {

	// Add code here to command the HVPS voltage to 0.

	// ***************** Add code here ******************



	// Set limChkOpenHvPsRelayTimer to 10 if we fail a HVPS_I test,
	// Decrement it every mSec, and open the HVPS Relay when we hit 0
	// Gives PS PID time to ramp voltage down before we open the relay
	limChkOpenHvPsRelayTimer = 10;

}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       L I M I T   C H E C K   S T A T E   M A C H I N E
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//enum LIMIT_CHECK_STATE_MACHINE {
//	LCSM_PERFORM_TEST_END		=	0,
//	LCSM_PERFORM_TEST			=	1,
//	LCSM_PERFORM_TEST_WAIT		=	2,
//	LCSM_PERFORM_TEST_UNTIL		=	3,
//	LCSM_WAIT_FOR_SYNC			=	4
//};

void limChkStateMachine(void){
// called here every milisec from a to timer background task

	Uint16 i;

	// For each of our 8 channels, manage state machine transitions
	for (i=0;i<8;i++) {
		limChkStateMachineChannel(i);
	}

	// Set limChkOpenHvPsRelayTimer to 10 if we fail a HVPS_I test,
	// Decrement it every mSec, and open the HVPS Relay when we hit 0
	// Gives PS PID time to ramp voltage down before we open the relay
	if (limChkOpenHvPsRelayTimer > 0){
		limChkOpenHvPsRelayTimer++;
		if (limChkOpenHvPsRelayTimer == 0){
			limChkOpenHvPsRelay();	// open the HVPS Relay
		}
	}

}

void limChkStateMachineChannel(Uint16 limitCheckChannel){

	enum LIMIT_CHECK_STATE_MACHINE limitCheckState;
	struct LIMIT_CHECK_PARAMETERS* lcParams;
	Uint32 mSecTimerNow;
	Uint32 mSecTimerStartTime;
	Uint32 mSecTimerEndTime;

	lcParams = &limitCheckParams[limitCheckChannel];

	// Request to ENABLE a test
	if ((lcParams->reqEnable == LIMCHK_REQ_TEST_ENABLE)	// asynchronous request to enable the test
	&& (lcParams->enableTest != LIMCHK_TEST_ENABLED)) {
		lcParams->reqEnable = LIMCHK_NO_PENDING_REQ;	// turn off the request, we are handling it
		// initialize some limitCheckParams[] values before starting test
		limchkPreTestInit (limitCheckChannel);

		// may want to do different min/max initializations for different ioTypes
		lcParams->maxValue = 0;
		lcParams->minValue = 0xFFFF;

		lcParams->enableTest = LIMCHK_TEST_ENABLED;

		// save the present mSec timer timestamp when this becomes enabled
		lcParams->mSecEnableTimeStamp = timer0_fetchSystemMiliSecCount();

		// if timeStart == 0 --> PERFORM_TEST
		if (lcParams->timeStart == 0) {
			lcParams->limitCheckState = LCSM_PERFORM_TEST;
			lcParams->testStatus = TESTING;
			lcParams->reqSync = LIMCHK_NO_PENDING_SYNC;
		} else {
			lcParams->limitCheckState = LCSM_WAIT_FOR_SYNC;
		}
	}

	// Request to DISABLE a test
	if ((lcParams->reqEnable == LIMCHK_REQ_TEST_DISABLE)	// asynchronous request to enable the test
	&& (lcParams->enableTest != LIMCHK_TEST_DISABLED)) {
		lcParams->reqEnable = LIMCHK_NO_PENDING_REQ;	// turn off the request, we are handling it
		lcParams->enableTest = LIMCHK_TEST_DISABLED;
		lcParams->limitCheckState = LCSM_PERFORM_TEST_END;
		if (lcParams->testStatus == TESTING) {
			// if test status indicates a failure, don't overwrite it
			lcParams->testStatus = NOT_TESTING;
		}
}

	limitCheckState = lcParams->limitCheckState;


	switch(limitCheckState){

	//enum LIMIT_CHECK_STATE_MACHINE {
		//	LCSM_PERFORM_TEST_END		=	0,
		//	LCSM_PERFORM_TEST			=	1,
		//	LCSM_PERFORM_TEST_WAIT		=	2,
		//	LCSM_PERFORM_TEST_UNTIL		=	3,
		//	LCSM_WAIT_FOR_SYNC			=	4
		//};

    case LCSM_WAIT_FOR_SYNC: //
    	// Got here because PC set testEnable to LIMCHK_TEST_ENABLED . . .
    	// and PC param timeStart > 0
    	// Waiting for SYNC message from PC before moving forward
    	if (lcParams->reqSync == LIMCHK_REQ_SYNC)	{ // asynchronous request to "Synchronise"
    		lcParams->reqSync = LIMCHK_NO_PENDING_SYNC;	// turn off the request, we are handling it
    		lcParams->limitCheckState = LCSM_PERFORM_TEST_WAIT;
    		// save the present mSec timer timestamp at Sync time
    		lcParams->mSecEnableTimeStamp = timer0_fetchSystemMiliSecCount();
    	}
        break;

    case LCSM_PERFORM_TEST: //
    	// Got here because PC set testEnable to LIMCHK_TEST_ENABLED . . .
    	// and PC param timeStart == 0.
    	// Or we got here from  state LCSM_PERFORM_TEST_WAIT . . .
    	// and PC param timeStart > timeEnd.
    	// Stay here until test fails of PC stops it.
        break;

    case LCSM_PERFORM_TEST_WAIT: //
    	// Got to this state -- LCSM_PERFORM_TEST_WAIT -- only after receiving a SYNC message
    	// Wait here until the current system mSec timer is >=
    	//      mSecEnableTimeStamp -- mSec timestamp when SYNC message was handled
    	//   +  timeStart           -- param set from PC, telling how many mSec to wait before starting
    	// After waiting, proceed to either
    	//      LCSM_PERFORM_TEST -- just start testing if PC params are : (timeStart > timeEnd)
    	//  or  LCSM_PERFORM_TEST_UNTIL --  if PC params are : (timeStart <= timeEnd)
    	mSecTimerStartTime = lcParams->mSecEnableTimeStamp + (Uint32)(lcParams->timeStart);
    	mSecTimerNow = timer0_fetchSystemMiliSecCount();
    	if (mSecTimerStartTime <= mSecTimerNow){
    	   	if (lcParams->timeStart > lcParams->timeEnd) {
    	   		lcParams->limitCheckState = LCSM_PERFORM_TEST;
    			lcParams->testStatus = TESTING;

    	   	} else {
    	   		lcParams->limitCheckState = LCSM_PERFORM_TEST_UNTIL;
    			lcParams->testStatus = TESTING;
    	   	}
    	}
        break;

    case LCSM_PERFORM_TEST_UNTIL: //
    	// got here because PC sent a SYNC message to kick off limit checking . . .
    	// and timeStart <=  timeEnd (params sent from PC) . . .
    	// specifying we test for a period of time then stop.
    	// Here, if we have exceeded the requested duration of our test, we stop
    	mSecTimerEndTime = lcParams->mSecEnableTimeStamp + (Uint32)(lcParams->timeEnd);
    	mSecTimerNow = timer0_fetchSystemMiliSecCount();
    	if (mSecTimerEndTime <= mSecTimerNow){
	   		lcParams->limitCheckState = LCSM_PERFORM_TEST_END;
			if (lcParams->testStatus == TESTING) {
				// if test status indicates a failure, don't overwrite it
				lcParams->testStatus = NOT_TESTING;
			}
    	}

        break;

    case LCSM_PERFORM_TEST_END: //
        break;
     default:
    	break;
    }
}

void limChkStateMachineOnTestFail(Uint16 limitCheckChannel){
// called from background measurement task when a test fails

	struct LIMIT_CHECK_PARAMETERS* lcParams;

	lcParams = &limitCheckParams[limitCheckChannel];

	// Update status and state machine when test fails
	lcParams->enableTest = LIMCHK_TEST_DISABLED;
	lcParams->limitCheckState = LCSM_PERFORM_TEST_END;

}

void limChkStateMachineResetOneChannel(Uint16 limitCheckChannel){
// called from Can Command to reset (Disable) one Channel

	struct LIMIT_CHECK_PARAMETERS* lcParams;

	lcParams = &limitCheckParams[limitCheckChannel];

	// Update status and state machine when test fails
	lcParams->enableTest = LIMCHK_TEST_DISABLED;
	lcParams->limitCheckState = LCSM_PERFORM_TEST_END;
	lcParams->testStatus = NOT_TESTING;

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       L I M I T   C H E C K   B A C K G R O U N D   T A S K
//       called from timer0_task() every 200uSec
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


void limChkBackgroundMeasurements(void){
// for each limit check channel 0-7
// see if measurements/comparisons are called for
// record measured value
// figure comparisonResult
	Uint16 i;
	struct LIMIT_CHECK_PARAMETERS* lcParams;
	const struct LIMIT_CHECK_INPUTS* lcInputs;
	enum LIMIT_CHECK_COMPARISON comparisonResult;
	Uint16 failures;

	failures = 0;

	for (i=0;i<8;i++){
		lcParams = &limitCheckParams[i];
		// do measurement and comparisons only in following 2 states
		if ((lcParams->limitCheckState == LCSM_PERFORM_TEST)
		|| (lcParams->limitCheckState == LCSM_PERFORM_TEST_UNTIL)){
			lcInputs = &limitCheckInputs[lcParams->limitCheckInput];
			// call routine via function pointer to compare present measurement vs limits
			// this call leaves measured value in measValue
			// (adding comment w/ limit check proc names, so searching gets here where it is called.)
			// limChkAnlgIn1234(), limChkAnlgIn5678(), limChkDigIn16(), limChkDiffIn8()
			// limChkFreqTest(), limChkUndefinedTest(),
			comparisonResult = lcInputs->limitCheckProcess(&(lcParams->limits),
					           	   lcInputs->offset,lcInputs->readFpga, &(lcParams->measValue));
			lcParams->comparisonResult = comparisonResult;

			// if comparison is outside outer Hi/Low limits, we have a hard failure
			if ((comparisonResult == LCC_ABOVE_OUTER_HI_LIMIT)
			|| (comparisonResult == LCC_BELOW_OUTER_LOW_LIMIT)) {
				limChkStateMachineOnTestFail(i); // update status and state machine
				lcParams->testStatus = LIMITS_FAILED;

			} else if ((comparisonResult == LCC_ABOVE_INNER_HI_LIMIT)
			|| (comparisonResult == LCC_BELOW_INNER_LOW_LIMIT)) {
				// if comparison is outside inner Hi/Low limits, and we have exceeded
				// count of allowed Failures, and the noHalts flag is not set
				// then we have a hard failure
				lcParams->detectedFails++;
				if ((lcParams->detectedFails > lcParams->allowedFails)
				&& (lcParams->noHalt == 0)){
				limChkStateMachineOnTestFail(i); // update status and state machine
				lcParams->testStatus = SPIKES_FAILED;
				}
			}

			// **** HAVE TO REACT TO SOME TEST FAILURES SUCH AS
			// **** OPENING HVPS RELAY, 24V RELAY, Shut off HVPS

			if ((lcParams->testStatus == SPIKES_FAILED)
			|| (lcParams->testStatus == LIMITS_FAILED)) {
				failures++;
				if (lcParams->limitCheckInput == LCI_PS_I) {
					// We experienced a failure on test of HV PS I
					// Command the HV PS to 0 volts, then open the HV PS Relay
					limChkHvPsShutdown();
				} else if (lcParams->limitCheckInput == LCI_24VPS_I) {	// 24V I Failure

					// ***************** Add code here ******************
					// to open the 24V relay
				}
			}


			// Taking into account the ioType of the input we are measuring
			// do housekeeping to calculation of Min, Max, and Average values.
			// At this point lcParams points to a specific Limit Check test.
			limChkBackgroundMinMaxAvg(lcParams);

		}
	}

	if (failures > 0) {
		//On any failure:
		//turn off all tests, setting enableTest <== 0
		//and set the State to something appropriate
		for (i=0;i<8;i++){
			limChkStateMachineOnTestFail(i);
		}
	}
}

void limChkBackgroundMinMaxAvg(struct LIMIT_CHECK_PARAMETERS* lcParams){
// Taking into account the ioType of the input we are measuring
// do housekeeping to calculation of Min, Max, and Average values.
// At this point lcParams points to a specific Limit Check test.

	switch(lcParams->ioType){

    case IO_TYPE_ANLG_IN_1234:	// Analog In
    case IO_TYPE_ANLG_IN_5678:	// Analog In
    case IO_TYPE_FREQ:			// Digital In Machine Frequency Measurement

    	if(lcParams->measValue > lcParams->maxValue) {
			lcParams->maxValue = lcParams->measValue;
		}
		if(lcParams->measValue < lcParams->minValue) {
			lcParams->minValue = lcParams->measValue;
		}

    	// sum & count for subsequent average calculation.
		(lcParams->sumCount)++;
    	lcParams->sumValue += lcParams->measValue;
    	// clever: if we are overrunning our 32 bit sumValue,
    	// divide both sum and count by 2
    	if ((lcParams->sumValue & 0x80000000L)){
    		lcParams->sumValue = ((lcParams->sumValue)>>1) & 0x7FFFFFFFL;
    		lcParams->sumCount = ((lcParams->sumCount)>>1) & 0x7FFFFFFFL;
    	}
		break;

    case IO_TYPE_DIG_IN_16: //
    case IO_TYPE_DIFF_IN_8: //

    	lcParams->minValue = 0L;
    	lcParams->maxValue = 0L;
    	lcParams->sumCount = 0L;
    	lcParams->sumValue = 0L;

		break;
		// ***************** Add code here ******************
		// to handle other ioTypes

    default:

   	break;
    }

}

void limChkBackgroundMinMaxAvgInit(struct LIMIT_CHECK_PARAMETERS* lcParams){
// Taking into account the ioType of the input we are measuring
// Set initial values for Min, Max, and Average values.
// At this point lcParams points to a specific Limit Check test.


	switch(lcParams->ioType){

    case IO_TYPE_UNDEFINED:		//
    case IO_TYPE_ANLG_IN_1234:	//
    case IO_TYPE_ANLG_IN_5678: 	//
    	lcParams->minValue 			= 0x8000L;
    	lcParams->maxValue 			= 0x8000L;
    	lcParams->avgValue 			= 0x8000L;
    	lcParams->sumValue				= 0L;
    	lcParams->sumCount				= 0L;
		break;

    case IO_TYPE_FREQ: 	// Digital In Machine Frequency Measurement (actually period)
    	lcParams->minValue 			= 0x7FFFFFFFL;
    	lcParams->maxValue 			= 0x0L;
    	lcParams->avgValue 			= 0x0L;
    	lcParams->sumValue				= 0L;
    	lcParams->sumCount				= 0L;
		break;

		// ***************** Add code here ******************
		// to handle other ioTypes

    default:

   	break;
    }

}


void limChkBackgroundComputeAvg(struct LIMIT_CHECK_PARAMETERS* lcParams){
// Taking into account the ioType of the input we are measuring
// Set initial values for Min, Max, and Average values.
// At this point lcParams points to a specific Limit Check test.
	int result;
	double result_dbl;

	switch(lcParams->ioType){

    case IO_TYPE_UNDEFINED:		//
    	lcParams->avgValue 			= 0x8000L;
		break;

    case IO_TYPE_ANLG_IN_1234:	// Analog In
    case IO_TYPE_ANLG_IN_5678:	// Analog In
    	// logically, the average = sumValue / sumCount
    	// Have not yet found guidance on division in TI documentation,
    	// except that maybe there is a run-time-library routine that
    	// may be used to implement division.
    	// For starters I'll just code in a division operation and see if it flies.
    	// Conclusion:
    	// Looks like it works pretty well
    	// measured the division operation below at 2.25uSec
    	if((long)lcParams->sumCount != 0L){
    		LOG_LIMCHK_ADDTOLOG(LOG_EVENT_LIMCHK_CALC,0x0010);
    		result = ((long)lcParams->sumValue) / ((long)lcParams->sumCount);
    		lcParams->avgValue = (Uint32)(result & 0xFFFF);
    		LOG_LIMCHK_ADDTOLOG(LOG_EVENT_LIMCHK_CALC,0x0011);
    	}
		break;

    case IO_TYPE_FREQ:			// Digital In Machine Frequency Measurement
    	if((long)lcParams->sumCount != 0L){
    		result_dbl = ((long)lcParams->sumValue) / ((long)lcParams->sumCount);
    		lcParams->avgValue = (Uint32)(result_dbl);
    	}
		break;


		// ***************** Add code here ******************
		// to handle other ioTypes

    default:

   	break;
    }

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       F O L L O W I N G   U S E D   F O R   A N A L O G _ I N _ C O M P A R I S O N
//       Not part of production code.  Supports PC client used to test conversions
//       for Analog_In (native / classic)
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS limChkAnlgInClassic(const struct CAN_COMMAND *can_command,Uint16 *data) {
//	We received a value from the PC to use as one of the 4 limits for limit-checking
//	Analog_Input values.  CAN Subinddex identifies which of 4 limits it is.
//	Limit value is scaled as for Classic test station Analog_Inputs
//	So some translation is needed to convert to native scale.
// *data is MboxA of received Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex


	Uint16 *dest;
	Uint16 subindex;
	Uint32 *classicLimit;
	Uint32 *nativeLimit;

	int ain_limit_value_classic_int;
	int m2;
	int result;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	// For all subindex values, we take the 16-bits received in MboxC and store it
	// in one of our static locations at the dest address we get from the can_command table entry.
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC -- save LS16 to static variable

	switch(subindex){
	// Set pointers into the classic and native limit structs based on subindex
    case 0x01: // subindex = 1 -> Hi_Outer Limit
    	classicLimit = &limChkAnlgInLimitsClassic.Hi_Outer;
    	nativeLimit = &limChkAnlgInLimits.Hi_Outer;
        break;

    case 0x02: // subindex = 2 -> Hi_Inner Limit
    	classicLimit = &limChkAnlgInLimitsClassic.Hi_Inner;
    	nativeLimit = &limChkAnlgInLimits.Hi_Inner;
        break;

    case 0x03: // subindex = 3 -> Low_Inner Limit
    	classicLimit = &limChkAnlgInLimitsClassic.Low_Inner;
    	nativeLimit = &limChkAnlgInLimits.Low_Inner;
        break;

    case 0x04: // subindex = 4 -> Low_Outer Limit
    	classicLimit = &limChkAnlgInLimitsClassic.Low_Outer;
    	nativeLimit = &limChkAnlgInLimits.Low_Outer;
        break;

    default:

   	break;
    }

    if ((subindex > 0) && (subindex < 5)){
    	// Store the limit value from the CAN command into our classic-struct
    	*classicLimit = *(data+2); // MboxC -- save LS16 value from the CAN command

    	// Convert the classic value to a native value
    	// We have different conversions for Anlg_IN_1,2,3,4 vs 5,6,7,8

        // REF on 2812 Multiplication:
        // "C/C++ Code Access to the Upper 16 Bits of 16-Bit Multiply"
        // p139, spru514e_TMS320C28x Compiler and Linker Users Guide.pdf

    	if ((limChkAnlgInChannel > 0) && (limChkAnlgInChannel < 5)) {

    		// <ain_limit_value_native> = (<ain_limit_value_classic> / (0x3565/0x10000)) + 0x8000
    		//                          = (<ain_limit_value_classic> * (0x10000 / 0x3565) + 0x8000
    		//                          = (<ain_limit_value_classic> * (    4.79449     ) + 0x8000
    		//                          = (<ain_limit_value_classic> * (0x4CB6 / 0x1000) + 0x8000

    		ain_limit_value_classic_int = (int)(*classicLimit);
    		m2 = (int)0x4CB6;
    		result = ((long)ain_limit_value_classic_int * (long)m2) >> 12; // event log clocked this at 6.4 uS (-5.86 for event_log) -> 0.54 uS
    		*nativeLimit = (Uint16)result + 0x8000;


    	} else if ((limChkAnlgInChannel > 4) && (limChkAnlgInChannel < 8)) {

    		// <ain_limit_value_native> = 0x8000 - (<ain_limit_value_classic> / (0x3BAB/0x10000))
    		//                          = 0x8000 - (<ain_limit_value_classic> * (0x10000/0x3BAB))
    		//                          = 0x8000 - (<ain_limit_value_classic> * (  4.290 ))
    		//                          = 0x8000 - (<ain_limit_value_classic> * (0x44A5/0x1000)) <- By Formula

    		//                          = 0x8000 - (<ain_limit_value_classic> * (  4.259)) <-- Heuristic correction
    		//                          = 0x8000 - (<ain_limit_value_classic> * (0x4424/0x1000))
    		ain_limit_value_classic_int = (int)(*classicLimit);
    		m2 = (int)0x44A5; // by formula, not by heuristic correction
    		result = ((long)ain_limit_value_classic_int * (long)m2) >> 12; // event log clocked this at 6.4 uS (-5.86 for event_log) -> 0.54 uS
    		*nativeLimit = 0x8000 - (Uint16)result;
    	}

    }
    return CANOPEN_NO_ERR;
}


enum CANOPEN_STATUS limChkAnlgInComparison(const struct CAN_COMMAND* can_command, Uint16* data){
    // This function returns an integer (enum value) characterizing the result of
	// limit-check comparisons, for an analog input channel.
	// Initially, this is written to test how we configure limits: it reads
	// one analog input voltage #1-8, specified in (Uint16 limChkAnlgInChannel)
	// and it compares against 4 limits in (struct HI_LOW_IN_OUT_UINT16_LIMITS limChkAnlgInLimits)
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// *can_command.datapointer points to ????

	Uint16 channel;
	enum LIMIT_CHECK_COMPARISON result;
	Uint32 measValue;
	// local measValue, is used only as a place for limChkAnlgIn1234( ) and limChkAnlgIn5678( )
	// to store a result that we don't use here.

	//valid values for analog input channel are 0 to 7
	//static value limChkAnlgInChannel is 1 to 8
	channel = limChkAnlgInChannel - 1;
	if (channel > 7){
	   return CANOPEN_LIMCHK_001_ERR;
	}

	if (channel < 4) {
       result = limChkAnlgIn1234(&limChkAnlgInLimits, &ain_offsets[channel],
    		   (Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + channel), &measValue);
	} else {
       result = limChkAnlgIn5678(&limChkAnlgInLimits, &ain_offsets[channel],
    		   (Uint16*)CPLD_F2_XA(FPGA2_READ_ADC_A1  + channel), &measValue);
	}

    *(data+2) = (Uint16)result;
	*(data+3) = 0;   //MboxD

	return CANOPEN_NO_ERR;
}


