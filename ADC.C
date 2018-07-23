// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     ADC.C
//
//   Analog to Digital Converter operations
//		This exercises the DSP's internal A-to-D features.
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  P R O B L E M  /  W O R K   A R O U N D
//
//  Problem: Works mostly BUT . . . if I use C1011 to reset the ADC and start with a
//  single input channel (for ex 6 or 7 which have non-zero voltages), and then I use
//  C1012 to add input channels (for ex, continuing to use 6 or 7, with non-zero
//  voltages.  Eventually I get a reading where the most recently added channel comes
//  back with a low reading.  Continuing to read with a C1013 may come back with the same
//  bogus last reading, or it may come back with the correct one.  Once it gets the
//  correct value, then repeating C1013 always repeats the correct value.
//
//  Work Around:  Not sure.  Maybe I just  have to read the new data several times
//  and discard it.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "ADC.H"
#include "Rs232Out.H"
#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"
#include "TaskMgr.h"


#define ADC_usDELAY_8mS  8000L
#define ADC_usDELAY_20uS 20L
#define ADC_usDELAY_1uS 1L
#define ADC_MODCLK 0x3   // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)         = 25MHz
#define ADC_CKPS   0x1   // ADC module clock = HSPCLK/2*ADC_CKPS   = 25MHz/(1*2) = 12.5MHz
#define ADC_SHCLK  0xf   // S/H width in ADC module periods                      = 16 ADC clocks
#define AVG        1000  // Average sample limit
#define ZOFFSET    0x00  // Average Zero offset
#define BUF_SIZE   16    // Sample buffer size

// Global variable for this example

Uint16 adc_ChSelSeq1; // Map ADC input channels to Adc results
Uint16 adc_ChSelSeq2; // See ADC Input Channel Select Sequencing Control Registers (ADCCHSELSEQ1-4)
Uint16 adc_ChSelSeq3;
Uint16 adc_ChSelSeq4;
Uint16 adc_maxConv;   // # if conversions, see ADCMAXCONV

Uint16 adc_DisplayAdcResultsIndex; // used in adc_DisplayAdcResults task

void adc_init(void)
// initializations from
//   C:\TI_SPRCO97_C281x_Header_n_Peripherals_Ex\tidcs\c28\DSP281x\v120\
//     \DSP281x_examples\adc_seqmode_test\Example_281xAdcSeqModeTest.c
{
	extern void DSP28x_usDelay(Uint32 Count);
	// Please note that for the delay function used below to
    // operate correctly the CPU_CLOCK_SPEED define statement in the
    // DSP28_Examples.h file must contain the correct CPU clock period in
    // nanoseconds.

    // (0) To powerup the ADC, the ADCENCLK bit should be set first to enable
    //     clocks, and this is already done in call from main( ) to  InitSysCtrl( ).
	//     But we do it again here anyway. "SysCtrlRegs.PCLKCR.bit.ADCENCLK=1;"
    // (1) Set the prescaler for the ADC Clock

	EALLOW;
	SysCtrlRegs.PCLKCR.bit.ADCENCLK=1;
	SysCtrlRegs.HISPCP.all = ADC_MODCLK;	// HSPCLK = SYSCLKOUT/ADC_MODCLK
	EDIS;

    // (2) Reset the ADC machine and wait briefly, as there is a latency
    //     before the reset occurs.

	AdcRegs.ADCTRL1.bit.RESET = 1;  // **RESET** ADC Hardware
	DELAY_US(ADC_usDELAY_1uS);      // Delay before accessing any other ADC registers

	// (3) if external reference is desired, enable this mode using bit 8 in the
	//     ADCCTRL3 Register. This mode must be enabled before band gap is
	//     powered. This avoids internal reference signals (ADCREFP and
	//     ADCREFM) driving external reference sources if present on the board.

	AdcRegs.ADCTRL3.bit.ADCEXTREF = 0x1;    // Configure ADCREFP(2V) and ADCREFM(1V) pins
	                                        // as inputs for external voltages
	                                        // This mode must be enabled before band gap is
	                                        // powered. This avoids internal reference signals (ADCREFP and
	                                        // ADCREFM) driving external reference sources present on the board.

	// (4) power up the bandgap and reference circuitry.

	AdcRegs.ADCTRL3.bit.ADCBGRFDN = 0x3;	// Power up bandgap/reference circuitry
	DELAY_US(ADC_usDELAY_8mS);              // Delay before powering up rest of ADC

    // (5) After a 7ms delay (we use 8mS) the rest of the ADC can be powered up.
	//     After ADC powerup, another 20us delay is required before performing the first
    //     ADC conversion.

	AdcRegs.ADCTRL3.bit.ADCPWDN = 1;		// Power up rest of ADC
	DELAY_US(ADC_usDELAY_20uS);             // Delay after powering up ADC

    // (6) Additional configuration, see user's guide
	AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;  // acquisition window size
	AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS; // core clock divider -> 12.5MHz
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        // 1  Cascaded mode
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0;        // Stop after 1 run

	// This default configuration for assigning adc inputs to ADCRESULTx registers
	// configures conversion of 1 ADC input channel, ADCINA0, w/ result in  ADCRESULT0
	adc_ChSelSeq1 = 0;
	adc_ChSelSeq2 = 0;
	adc_ChSelSeq3 = 0;
	adc_ChSelSeq4 = 0;
	adc_maxConv = 0;  // n+1=4 # of conversions in this run
	adc_configureChannels();
}

void adc_configureChannels(void)
{
	// Assign ADC input channels to locations in the AdcRegs.ADCRESULTx registers
	// Before reading analog inputs, the caller sets up values in adc_ChSelSeq1-4
	// and adc_maxConv, and then calls this routine, to write them into the ADC registers.

	AdcRegs.ADCCHSELSEQ1.all = adc_ChSelSeq1;
	AdcRegs.ADCCHSELSEQ2.all = adc_ChSelSeq2;
	AdcRegs.ADCCHSELSEQ3.all = adc_ChSelSeq3;
	AdcRegs.ADCCHSELSEQ4.all = adc_ChSelSeq4;
	AdcRegs.ADCMAXCONV.all = adc_maxConv;              // n+1=4 # of conversions in this run
}

void adc_runConversion(void)
{
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; // try clearing the interrupt (DONE) flag before starting
	// Start of conversion SOC SEQ1
	AdcRegs.ADCTRL2.all = 0x2000;

    while (AdcRegs.ADCST.bit.INT_SEQ1== 0) {} // Wait for interrupt flag to indicate conversion DONE
    // Software wait = (HISPCP*2) * (ADCCLKPS*2) * (CPS+1) cycles
    //               = (3*2)      * (1*2)        * (0+1)   = 12 cycles
    asm(" RPT #11 || NOP");

    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
}

//===========================================================================
// Demo Functions, activated via RS232 Command Interpreter (COMINT),
//
//===========================================================================

void adc_convertToMilliVolts(Uint16 adcValue,Uint16* mVResult)
{
	//Take a value read from an AdcRegs.ADCRESULTx register, (>>4)
	//Output a 16-bit unsigned value equal to the mV reading at the ADC channelx input
	//
	//To interpret the value from the AdcRegs.ADCRESULTx register,
	//remember that it uses 12 bits to represent it's input voltage range: 0 - 3V,
	//hence Volts = (AdcRegs.ADCRESULTx>>4)* (3 / ((2^12)-1)
	//[note: we approximate and use just 2^12 in the denominator, instead of (2^12)-1]
	//
	//Numerically it comes out to
	//Value in AdcRegs.ADCRESULTx register x 0.0007324 = volts [or x 0.7324 = mV]
	//
	//Now if we do it in millivolts, we introduce a factor of 1000 (mv/V)
	// 0.7324
	// = (3/4096) x 1000
	// = 357/512
	// = 0x177 / 0x200
	// = (0x100 + 0x70 + 0x07) / 0x200
	// we can multiply our AdcRegs.ADCRESULTx x 0.7324 through 3 shift/add operations.

	Uint16 i,j;

	//To calculate (adcValue x 0.7324) =
	// (adcValue * 7) >> 9
	// + ((adcValue >> 4) * 7) >> 1
	// + adcValue >> 1

	i = (((adcValue<<3)-adcValue)>>9);
	j =	(((adcValue>>1)-(adcValue>>4))>>1);
	*mVResult = (adcValue >> 1)+ i + j;
}


void adc_readOneAdcChannelToRs232(Uint16 adcChannelNum)
{
	// Read analog voltage from 1 ADC channel
	// and report result to RS232
	// adcChannelNum = 0-7 => ADCINA0-7, 8-15 => ADCINB0-7)
	// bool success;
	char msgOut[64];
	char *ptr;
	Uint16 channelNum;
	Uint16 adcResult;
	Uint16 mV;

	adc_init();                           // reset and initialize the ADC

	channelNum = adcChannelNum & 0x000F;  // adcChannelNum = 0-7 => ADCINA0-7, 8-15 => ADCINB0-7)

	adc_maxConv = 0;                      // n+1=1 # of conversions in this run, just 1
	adc_ChSelSeq1 = channelNum;           // we use adc_ChSelSeq1-4 RAM to shaddow hardware registers
	adc_configureChannels();              // write channel configuration from RAM into registers

	adc_runConversion();                     // Run the conversion
	adcResult = ((AdcRegs.ADCRESULT0>>4) );  // fetch the result

    adc_convertToMilliVolts(adcResult,&mV);  // convert ADC value to mV

    // Write result to RS232 buffer
	ptr = strU_strcpy(msgOut,"Read 1 Anlg Input Ch# 0x");
    ptr = hexUtil_binTo2HexAsciiChars (ptr,channelNum);
    ptr = strU_strcpy(ptr,", milliVolts = ");
    ptr = hexUtil_binToDecAsciiCharsZeroSuppress(ptr,mV);
    ptr = strU_strcpy(ptr,"\n\r");
    /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

    ptr = strU_strcpy(msgOut,"   AdcValue = 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,adcResult);
    ptr = strU_strcpy(ptr,", mVinHex = 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,mV);
    ptr = strU_strcpy(ptr,"\n\r");
    /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}

void adc_addOneAdcChannelToList(Uint16 adcChannelNum){
	// Here we demonstrate reading multiple adc channels in a single "run".
	// And we do it without resertting and reconfiguring the whole ADC peripheral.
	// This is in contrast to adc_readOneAdcChannelToRs232( ) where we do all the resetting
	// and reconfiguring easc time and read only a single channel.
	//
	// To start we have globals adc_ChSelSeq1,2,3,4, and adc_maxConv telling which adc
	// channels to read, and the minimum  number of channels to read is 1 (adc_maxConv = 0),
	// and the 1 channel to read is coded in the ls 4 bits of adc_ChSelSeq1.
	//
	// Se we increment adc_maxConv by 1, and put the adc channel number in the next
	// available nibble in adc_ChSelSeq1,2,3,4. Note, doing this to add 1 arbitrary selected
	// new channel is brobably more work  (see select case stmnts below) than just starting
	// with a list of desired channels and configuring the ADC to fetch them.

	char msgOut[64];
	char *ptr;
	Uint16 mask;
	Uint16 maskedAndShifted;

	// Can't have more than 16 channels to read
	if (adc_maxConv >=0x0F)
	{
		ptr = strU_strcpy(msgOut,"Max # Channels\n\r");
	    /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

	} else {
		// add one more adc input channel to adc_maxConv & adc_ChSelSeq1,2,3,4
		adc_maxConv++;

		switch(adc_maxConv & 0x03){
	    case 0x00:
	    	mask = 0xFFF0;
	    	maskedAndShifted = adcChannelNum & 0x000F;
	        break;
	    case 0x01: // select a pattern to blink the LEDs
	    	mask = 0xFF0F;
	    	maskedAndShifted = (adcChannelNum<<4) & 0x00F0;
	        break;
	    case 0x02: // select a pattern to blink the LEDs
	    	mask = 0xF0FF;
	    	maskedAndShifted = (adcChannelNum<<8) & 0x0F00;
	        break;
	    default:
	  //case 0x03:
	    	mask = 0x0FFF;
	    	maskedAndShifted = (adcChannelNum<<12) & 0xF000;
	    	break;
	    }
		switch(adc_maxConv & 0x0C){
	    case 0x00:
	    	adc_ChSelSeq1 = (adc_ChSelSeq1 & mask) | maskedAndShifted;
	        break;
	    case 0x04:
	    	adc_ChSelSeq2 = (adc_ChSelSeq2 & mask) | maskedAndShifted;
	        break;
	    case 0x08:
	    	adc_ChSelSeq3 = (adc_ChSelSeq3 & mask) | maskedAndShifted;
	        break;
	    default:
	  //case 0x0C:
	    	adc_ChSelSeq4 = (adc_ChSelSeq4 & mask) | maskedAndShifted;
	    	break;
	    }
	}

	adc_configureChannels();              // write channel configuration from RAM into registers

	adc_runConversion();                  // Run the conversion

	// Now lets report the results, set a background task to do that
	adc_DisplayAdcResultsIndex = 0;
	taskMgr_setTask(TASKNUM_DisplayAdcResults);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Display results from the last ADC results
// This is a task, fired off by task manager
// If it can't fit it's entire message in the RS232 Out Buff,
// then it sets itself again on the task list and exits.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void adc_DisplayAdcResults(void){
	char msgOut[64];
    char *ptr;
    bool success;
	Uint16 i;
	Uint16 mV;
	Uint16 *ptrResult;
	Uint16 channelNum;
	Uint16 adcResult;

	i = adc_DisplayAdcResultsIndex;
	if (i > adc_maxConv){
		return; // exit without relaunching task
	}
	ptrResult = (Uint16*)(&AdcRegs.ADCRESULT0);

	adcResult = (*(ptrResult + i))>>4;  // fetch the result
    adc_convertToMilliVolts(adcResult,&mV);  // convert ADC value to mV

    // extract the adc channel number based on i, the loop count
	switch(i & 0x0C){
    case 0x00:
    	channelNum = adc_ChSelSeq1;
        break;
    case 0x04: //
    	channelNum = adc_ChSelSeq2;
        break;
    case 0x08: //
    	channelNum = adc_ChSelSeq3;
        break;
    default:
  //case 0x0C:
    	channelNum = adc_ChSelSeq4;
    	break;
    }
	switch(i & 0x03){
	case 0x00:
    	channelNum = channelNum;
        break;
    case 0x01:
    	channelNum = channelNum>>4;
        break;
    case 0x02:
    	channelNum = channelNum>>8;
        break;
    default:
  //case 0x03:
    	channelNum = channelNum>>12;
    	break;
    }
	channelNum &= 0x000F;

	// Write result to RS232 buffer
	ptr = strU_strcpy(msgOut,"Read mult Anlg Input Ch# 0x");
    ptr = hexUtil_binTo2HexAsciiChars (ptr,channelNum);
    ptr = strU_strcpy(ptr,", milliVolts = ");
    ptr = hexUtil_binToDecAsciiCharsZeroSuppress(ptr,mV);
    ptr = strU_strcpy(ptr,"\n\r");
    success = r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

    if (success){
    	adc_DisplayAdcResultsIndex++; // do next line next time
    }
	// run this task again to continue with the next step
	taskMgr_setTaskRoundRobin(TASKNUM_DisplayAdcResults, 0);
}

void adc_repeatPrevAdcTest(void)
{
	// Previous commands initialized the ADC hardware and left configuration
	// values in adc_maxConv and adc_ChSelSeq1,2,3,4.
	// Re-run the adc capture & conversion, and display the results.

	adc_configureChannels();              // write channel configuration from RAM into registers

	adc_runConversion();                  // Run the conversion

	// Now lets report the results, set a background task to do that
	adc_DisplayAdcResultsIndex = 0;
	taskMgr_setTask(TASKNUM_DisplayAdcResults);
}

void adc_readAll16Channels(void)
{
	adc_init();                           // reset and initialize the ADC
	adc_maxConv = 0x0F;                   // n+1=1 # of conversions in this run, just 1
	adc_ChSelSeq1 = 0x3210;               // we use adc_ChSelSeq1-4 RAM to shaddow hardware registers
	adc_ChSelSeq2 = 0x7654;               // we use adc_ChSelSeq1-4 RAM to shaddow hardware registers
	adc_ChSelSeq3 = 0xBA98;               // we use adc_ChSelSeq1-4 RAM to shaddow hardware registers
	adc_ChSelSeq4 = 0xFEDC;               // we use adc_ChSelSeq1-4 RAM to shaddow hardware registers
	adc_configureChannels();              // write channel configuration from RAM into registers

	adc_repeatPrevAdcTest();              // Run the conversion, start task to report results
}
