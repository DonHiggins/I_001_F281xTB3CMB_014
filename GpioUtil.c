// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     GpioUtil.C
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File

#include "stdbool.h"            // needed for bool data types
#include "CPLD.H"
#include "GpioUtil.H"

// - - - - I M P O R T A N T - R E A D   T H I S - - - - - - - - - - - - - -
//
// Rather than having a complete and finished plan for GPIO usage in this
// project, I envision GPIO use will evolve substantially over the course of
// this project.  What I'm going to do is to have a default initialization
// routine, to be called first, which makes all of the GPIOs inputs. Then
// I'll provide individual, function specific init routines to reconfigure
// specific necessary GPIOs for specific functions.
//
// Examples of GPIO initialization may be found in the DSP281x_Gpio.c file,
// eg: InitGpio().
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// EALLOW / EDIS PROTECTION -- see
// spru078b: TMS321x281x DSP System Control and Interrupts Reference Guide
// section 4
//
// These registers use EALLOW / EDIS protection
//  GPxMUX
//  GPxDIR
//  GPxQUAL
//
// These registers DON'T use EALLOW / EDIS
//  GPxDAT
//  GPxSET
//  GPxCLEAR
//  GPxTOGGLE
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



void GpioU_defaultInit(void){
    // For starters setup the GPIO as all inputs.

    Uint16 var1;
    Uint16 var2_all_inputs;
    Uint16 var3;

    EALLOW;

    var1= 0x0000;		            // sets GPIO Muxs as I/Os
	GpioMuxRegs.GPAMUX.all=var1;
    GpioMuxRegs.GPBMUX.all=var1;
    GpioMuxRegs.GPDMUX.all=var1;
    GpioMuxRegs.GPFMUX.all=var1;
    GpioMuxRegs.GPEMUX.all=var1;
    GpioMuxRegs.GPGMUX.all=var1;

    var2_all_inputs = 0x0000;                // sets GPIO DIR as all inputs
    GpioMuxRegs.GPADIR.all=var2_all_inputs;  // If GPxDIR.bit = 0, then the pin is configured as an input
    GpioMuxRegs.GPBDIR.all=var2_all_inputs;  // If GPxDIR.bit = 1, then the pin is configured as an output
    GpioMuxRegs.GPDDIR.all=var2_all_inputs;
    GpioMuxRegs.GPEDIR.all=var2_all_inputs;
    GpioMuxRegs.GPFDIR.all=var2_all_inputs;
    GpioMuxRegs.GPGDIR.all=var2_all_inputs;

    var3= 0x0000;		            // sets the Input qualifier values
    GpioMuxRegs.GPAQUAL.all=var3;   // 0x00 No qualification (just SYNC to SYSCLKOUT)
    GpioMuxRegs.GPBQUAL.all=var3;   // eg: no de-bounce
    GpioMuxRegs.GPDQUAL.all=var3;
    GpioMuxRegs.GPEQUAL.all=var3;

    EDIS;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  F1I -- FPGA #1 Interrupt on pin Xint1
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_f1iInit(void){
    // Setup the GPIO to use IO pin D9 As XINT1 interrupt

#ifndef	TB3CMA_GPIO
	// TB3CMB uses GPIOE0 as I2CEE_DATA

	EALLOW;
    GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0 = 1;      //1-> XINT1, 0-> GPIOE0
    EDIS;
#endif

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  F2I -- FPGA #2 Interrupt on pin Xint13
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_f2iInit(void){
    // Setup the GPIO to use IO pin E2 As XNMI_XINT13 interrupt
    EALLOW;
    GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2 = 1;      //1-> XNMI_XINT13, 0-> GPIOE2
    EDIS;
}



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  RS232
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_rs232Init(void){
    // Setup the GPIO to use IO pins F4 and F5 for SciA Tx & Rx respectively
    EALLOW;
    GpioMuxRegs.GPFMUX.bit.SCITXDA_GPIOF4 = 1;      //SciA Tx
    GpioMuxRegs.GPFMUX.bit.SCIRXDA_GPIOF5 = 1;      //SciA Rx
    EDIS;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  SCI2 -- RS232/485 on TB3IOM
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_sci2Init(void){
    // Setup the GPIO to use IO pins F4 and F5 for SciA Tx & Rx respectively
    EALLOW;
    GpioMuxRegs.GPGMUX.bit.SCITXDB_GPIOG4 = 1;      //SciB Tx
    GpioMuxRegs.GPGMUX.bit.SCIRXDB_GPIOG5 = 1;      //SciB Rx
    EDIS;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  SPI
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_SpiInit(void){
    // Setup the GPIO to use IO pins:
	// F0 SPI data out
	// F1 SPI Data IN
	// F2 SPI Clock Out
	// (obs) F3 ~CS for Spi EEProm on TB3CM -- out, 1

	// before initializing GPIO F3 as an output line (below)
	// initialize it's output value so as to NOT enable the EEPromOnTB3CM
	// SPI_EEProm was used in TB3CMA, not in TB3CMB and subsequent designs
	//GpioU_SpiDisableEEPromOnTB3CM();  // set ~CS, F3 => 1

    EALLOW;
    GpioMuxRegs.GPFMUX.bit.SPISIMOA_GPIOF0 = 1; //SPISIMO (O)
    GpioMuxRegs.GPFMUX.bit.SPISOMIA_GPIOF1 = 1; //SPISOMI (I)
    GpioMuxRegs.GPFMUX.bit.SPICLKA_GPIOF2 = 1;  //SPICLK (I/O)

    //Following used in TB3CMA for experimental SPI_EEProm.
    //The SPI_EEProm was not included in TB3CMB
    //GpioMuxRegs.GPFDIR.bit.GPIOF3 = 1;  //output
    EDIS;

}

//void GpioU_SpiDisableEEPromOnTB3CM(void){
//    // set GPIO bit F3 HI to disable the Chip Select for the EEProm on TB3CM
//    GpioDataRegs.GPFSET.all |= 0x0008;  // set ~CS, F3 => 1
//
//}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  NOTES ON IO PIN USE IN TB3CMB VS TB3CMA
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//  These IO pins are used for . . .
//	        TB3CMB:         TB3CMA:
//	GPIOA8   I2CEE_CLK       unused
//	GPIOA11  I2CEE_DATA      unused
//	GPIOA14  I2EEC_CLK_EN    unused
//	GPIOA15  I2CEE_WP        unused
//
//  These functions use these IO pins . . .
//	             TB3CMB:   TB3CMA:
//	I2CEE_CLK     GPIOA8    GPIOF14
//	I2CEE_DATA    GPIOA11   GPIOE0
//	I2EEC_CLK_EN  GPIOA14   (from CPLD)
//	I2CEE_WP      GPIOA15   (from CPLD)
//
//  These IO pins are used for . . .
//	        TB3CMB:       TB3CMA:
//	GPIOE2   IO_INT_2       IO_INT_2
//	GPIOE0   IO_INT_1       I2CEE_DATA
//	GPIOE1   PM_INT_3       gnd
//

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  I2C EEPROM on TB3IOMA
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_i2cee_Init(void){
	// Not Busy: Both Clk and Data are High
	// Rem: GpioU_defaultInit() sets GPIO Muxs as I/Os, sets all GPIO as Inputs
	//	             TB3CMB:   TB3CMA:
	//	I2CEE_CLK     GPIOA8    GPIOF14
	//	I2CEE_DATA    GPIOA11   GPIOE0
	//	I2EEC_CLK_EN  GPIOA14   (from CPLD)
	//	I2CEE_WP      GPIOA15   (from CPLD)


#ifdef TB3CMA_GPIO
    //
    //TB3CMA:
    //	I2CEE_CLK     GPIOF14
    //	I2CEE_DATA    GPIOE0
    //	I2EEC_CLK_EN  cpld (eeprom on TB3IOMB)
    //	I2CEE_WP      cpld (eeprom on TB3IOMB)
    //
    EALLOW;
    GpioDataRegs.GPESET.all    |= 0x0001; // Data = HI
    GpioMuxRegs.GPEDIR.all     |= 0x0001; // Data, bit 0, as output
    GpioDataRegs.GPFSET.all    |= 0x4000; // CLK = HI
    GpioMuxRegs.GPFDIR.all     |= 0x4000; // Clk, bit 14, as output
    EDIS;
#endif

#ifdef TB3CMB_GPIO
    //
    //TB3CMB:
    //	I2CEE_CLK     GPIOA8
    //	I2CEE_DATA    GPIOA11
    //	I2EEC_CLK_EN  GPIOA14 (eeprom on TB3CMA)
    //	I2CEE_WP      GPIOA15 (eeprom on TB3CMA)
    //
    EALLOW;
    GpioDataRegs.GPASET.all    |= 0xC900; // Data = HI, CLK = HI, WP = HI, ~EN = HI
    GpioMuxRegs.GPADIR.all     |= 0xC900; // Data, CLK, WP, ~EN ==> outputs
    EDIS;
#endif

}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  LEDs / TIMER0
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_timer0Init(void){
    // Setup the GPIO to use IO pins B0, B1, B2, & B3 as outputs
    Uint16 var2_LEDs;
    var2_LEDs       = 0x000F;       // 0:3 are outputs for LEDs

    EALLOW;
    GpioMuxRegs.GPBDIR.all |= var2_LEDs;
    EDIS;

}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Set all GPIO's On or Off -- in this case, they are all OFF ("clear")
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_initGpiosAllZero(void)
{
   // Set pins to a known initial state
   // Set all GPIO to 0, "clear"

//   GpioDataRegs.GPASET.all    =0xAAAA;
	 GpioDataRegs.GPACLEAR.all  =0xFFFF;

//	 GpioDataRegs.GPBSET.all    =0x0000;
     GpioDataRegs.GPBCLEAR.all  =0xFFFF;

//   GpioDataRegs.GPDSET.all    =0x0022;
     GpioDataRegs.GPDCLEAR.all  =0x0063;    // Four I/Os only

//   GpioDataRegs.GPESET.all    =0x0002;
     GpioDataRegs.GPECLEAR.all  =0x0007;    // ThreeI/Os only

//   GpioDataRegs.GPFSET.all    =0x0000;
     GpioDataRegs.GPFCLEAR.all  =0xFFFF;

//   GpioDataRegs.GPGSET.all    =0x0020;
     GpioDataRegs.GPGCLEAR.all  =0x0030;    // Two  I/Os only
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Gpio_example3C: from H_004_F281x_CpuTimer_n_LED_Flash
//   not called from anywhere, preserved as an example
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void Gpio_example3C(void)
{
   // Use TOGGLE registers to flip the state of
   // the pins.
   // Any bit set to a 1 will flip state (toggle)
   // Any bit set to a 0 will not toggle.
//   GpioDataRegs.GPATOGGLE.all = 0xFFFF;
//   GpioDataRegs.GPBTOGGLE.all = 0xFFFF;
	 GpioDataRegs.GPBTOGGLE.all = 0x0000F; // toggle only LS 4 bits of GPIOB

//   GpioDataRegs.GPDTOGGLE.all = 0xFFFF;
//   GpioDataRegs.GPETOGGLE.all = 0xFFFF;
//   GpioDataRegs.GPFTOGGLE.all = 0xFFFF;
//   GpioDataRegs.GPGTOGGLE.all = 0xFFFF;
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Test various I/O lines and features, activated from RS232 / Comint
//  Command C2100:xxxx   ***** TB3CMA / TB3IOMA ***** FPGA1
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_ioTest2100(Uint16 dataWord){
	switch(dataWord){
    case 0x0001: // Three functions controlling GPIO B14 F_INIT_A, re: FPGA Programming
    	// 0x0001 -- initialize GPIO B14 as an output
    	// 0x0002 -- Set GPIO B14 F_INIT_A HI
    	// 0x0003 -- Set GPIO B14 F_INIT_A LOW
    	// 0x0004 -- initialize GPIO B14 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all |=0x4000; // GPIO bit 14 as output
        EDIS;
        break;
    case 0x0002: // Set GPIO B14 F_INIT_A HI
        GpioDataRegs.GPBSET.all    |= 0x4000;
        break;
    case 0x0003: // Set GPIO B14 F_INIT_A LOW
        GpioDataRegs.GPBCLEAR.all  |= 0x4000;
        break;
    case 0x0004: // initialize GPIO B14 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all &=0xBFFF; // GPIO bit 14 as output
        EDIS;
        break;

    case 0x0011: // Three functions controlling GPIO F9 F_PROG, re: FPGA Programming
    	// 0x0011 -- initialize GPIO F9 as an output
    	// 0x0012 -- Set GPIO F9 F_INIT_A HI
    	// 0x0013 -- Set GPIO F9 F_INIT_A LOW
    	// 0x0014 -- initialize GPIO F9 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPFDIR.all |=0x0200; // GPIO bit F9 as output
        EDIS;
        break;
    case 0x0012: // Set GPIO F9 F_INIT_A HI
        GpioDataRegs.GPFSET.all    |= 0x0200;
        break;
    case 0x0013: // Set GPIO F9 F_INIT_A LOW
        GpioDataRegs.GPFCLEAR.all  |= 0x0200;
        break;
    case 0x0014: // initialize GPIO F9 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPFDIR.all &=0xFDFF; // GPIO bit F9 as input
        EDIS;
        break;

    case 0x0020: // 20 = set GPIO-B14-F_INIT & GPIO-F9-F_PROG as outputs, HI
        // This puts FPGA ready for JTAG programming.
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all |=0x4000; // GPIO bit B14 as output
        GpioMuxRegs.GPFDIR.all |=0x0200; // GPIO bit F9 as output
        EDIS;
        GpioDataRegs.GPBSET.all    |= 0x4000;  // GPIO bit B14 HI
        GpioDataRegs.GPFSET.all    |= 0x0200;  // GPIO bit F9 HI
        break;

    default:
    	break;
    }

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Test various I/O lines and features, activated from RS232 / Comint
//  Command C2101:xxxx  ***** TB3CMB / TB3IOMC ***** FPGA1
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_ioTest2101(Uint16 dataWord){
	volatile Uint16 *extData; // pointer for external bus address to access CPLD
    volatile Uint16 data_in;

	switch(dataWord){
    case 0x0001: // Three functions controlling GPIO B14 F_INIT_A, re: FPGA Programming
    	// 0x0001 -- initialize GPIO B12 as an output
    	// 0x0002 -- Set GPIO B12 F_INIT_A HI
    	// 0x0003 -- Set GPIO B12 F_INIT_A LOW
    	// 0x0004 -- initialize GPIO B12 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all |=0x1000; // GPIO bit 12 as output
        EDIS;
        break;
    case 0x0002: // Set GPIO B12 F_INIT_A HI
        GpioDataRegs.GPBSET.all    |= 0x1000;
        break;
    case 0x0003: // Set GPIO B12 F_INIT_A LOW
        GpioDataRegs.GPBCLEAR.all  |= 0x1000;
        break;
    case 0x0004: // initialize GPIO B12 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all &=0xEFFF; // GPIO bit 12 as input
        EDIS;
        break;

    case 0x0011: // Three functions controlling GPIO F8 F_PROG, re: FPGA Programming
    	// 0x0011 -- initialize GPIO F8 as an output
    	// 0x0012 -- Set GPIO F8 F_INIT_A HI
    	// 0x0013 -- Set GPIO F8 F_INIT_A LOW
    	// 0x0014 -- initialize GPIO F8 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPFDIR.all |=0x0100; // GPIO bit F8 as output
        EDIS;
        break;
    case 0x0012: // Set GPIO F8 F_INIT_A HI
        GpioDataRegs.GPFSET.all    |= 0x0100;
        break;
    case 0x0013: // Set GPIO F8 F_INIT_A LOW
        GpioDataRegs.GPFCLEAR.all  |= 0x0100;
        break;
    case 0x0014: // initialize GPIO F8 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPFDIR.all &=0xFEFF; // GPIO bit F8 as input
        EDIS;
        break;

    case 0x0020: // 20 = set GPIO-B12-F_INIT & GPIO-F8-F_PROG as outputs, HI
        // This puts FPGA ready for JTAG programming.
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all |=0x1000; // GPIO bit B12 as output
        GpioMuxRegs.GPFDIR.all |=0x0100; // GPIO bit F8 as output
        EDIS;
        GpioDataRegs.GPBSET.all    |= 0x1000;  // GPIO bit B12 HI
        GpioDataRegs.GPFSET.all    |= 0x0100;  // GPIO bit F8 HI
        break;

    case 0x0030: // 30 = set ~FPGA1_RESET LOW
    	extData = CPLD_XINTF_ADDR(TBIOM_FPGA1_RESET); // read to CPLD sets it LOW
    	data_in = *extData; // data not important, read sets ~RESET LOW,
        break;

    case 0x0031: // 31 = set ~FPGA1_RESET HI
 	   extData = CPLD_XINTF_ADDR(TBIOM_FPGA1_RESET);
 	   *extData = 1; // data not important, write sets ~RESET HI,
        break;

    default:
    	break;
    }
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Test various I/O lines and features, activated from RS232 / Comint
//  Command C2102:xxxx  ***** TB3CMB / TB3IOMC ***** FPGA2
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void GpioU_ioTest2102(Uint16 dataWord){
	switch(dataWord){
    case 0x0001: // Three functions controlling GPIO B13 F_INIT_A, re: FPGA Programming
    	// 0x0001 -- initialize GPIO B13 as an output
    	// 0x0002 -- Set GPIO B13 F_INIT_A HI
    	// 0x0003 -- Set GPIO B13 F_INIT_A LOW
    	// 0x0004 -- initialize GPIO B13 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all |=0x2000; // GPIO bit 13 as output
        EDIS;
        break;
    case 0x0002: // Set GPIO B13 F_INIT_A HI
        GpioDataRegs.GPBSET.all    |= 0x2000;
        break;
    case 0x0003: // Set GPIO B13 F_INIT_A LOW
        GpioDataRegs.GPBCLEAR.all  |= 0x2000;
        break;
    case 0x0004: // initialize GPIO B13 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all &=0xDFFF; // GPIO bit 13 as input
        EDIS;
        break;

    case 0x0011: // Three functions controlling GPIO F9 F_PROG, re: FPGA Programming
    	// 0x0011 -- initialize GPIO F8 as an output
    	// 0x0012 -- Set GPIO F9 F_INIT_A HI
    	// 0x0013 -- Set GPIO F9 F_INIT_A LOW
    	// 0x0014 -- initialize GPIO F9 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPFDIR.all |=0x0200; // GPIO bit F9 as output
        EDIS;
        break;
    case 0x0012: // Set GPIO F9 F_INIT_A HI
        GpioDataRegs.GPFSET.all    |= 0x0200;
        break;
    case 0x0013: // Set GPIO F9 F_INIT_A LOW
        GpioDataRegs.GPFCLEAR.all  |= 0x0200;
        break;
    case 0x0014: // initialize GPIO F9 as an input
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPFDIR.all &=0xFDFF; // GPIO bit F9 as input
        EDIS;
        break;

    case 0x0020: // 20 = set GPIO-B13-F_INIT & GPIO-F9-F_PROG as outputs, HI
        // This puts FPGA ready for JTAG programming.
    	EALLOW;
        //GpioU_defaultInit() called at startup, sets all mux settings as I/O
    	// I don't know what the default output value, HI/LOW is
        GpioMuxRegs.GPBDIR.all |=0x2000; // GPIO bit B13 as output
        GpioMuxRegs.GPFDIR.all |=0x0200; // GPIO bit F9 as output
        EDIS;
        GpioDataRegs.GPBSET.all    |= 0x2000;  // GPIO bit B13 HI
        GpioDataRegs.GPFSET.all    |= 0x0200;  // GPIO bit F9 HI
        break;

    default:
    	break;
    }
}

