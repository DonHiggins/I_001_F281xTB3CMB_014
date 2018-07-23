// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     CPLD.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef CPLDx_H
#define CPLDx_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types

//--------------------------------------------------------------------------
// New Scheme: will work for both TB3IOMA & B
//
// CPLD addresses are each represented by a nmemonic in a 16-bit enum.
// Shift enum value left 8 bits to get the desired address.
// Note that the CPLD doesn't even see the ls 8 bits on its address bus.
// To put it another way, the enum value is address_bus_bits8:18
// And here we are talking about internal DSP address values which
// map to the external bus addresses that address the CPLD & FPGAs.
//
// Macros are provided below the enum to conveniently convert enum values
// to pointer address.
//
// FPGA addresses: have defines for CPLD_F1_XA(y) and CPLD_F2_XA(y) which
// return a (Uint16*) pointer, given an enum FPGA1_ADDR or enum FPGA2_ADDR value
// identifying an address in one of the FPGAs
// At present, the FPGA programs contain an include file, Bus_Defs.v which
// gives addresses for all public FPGA1 & 2 functions.
//--------------------------------------------------------------------------

#define TB3IOMA 0

#if TB3IOMA

// CPLD Addresses for TB3IOMA
enum CPLD_ADDR {
	TBIOM_CPLD_LED_0          = 0x080A,
	TBIOM_CPLD_LED_1          = 0x0C09,
	TBIOM_CPLD_LED_2          = 0x080C,
	TBIOM_CPLD_LED_3          = 0x0C0B,
	TBIOM_CPLD_TESTPOINT_0    = 0x0800,
	TBIOM_CPLD_TESTPOINT_1    = 0x0C01,
	TBIOM_CPLD_TESTPOINT_2    = 0x0802,
	TBIOM_CPLD_TESTPOINT_3    = 0x0C03,
	TBIOM_CS_IO_FLSH          = 0x0804,
	TBIOM_CS_IO_EEPRM         = 0x0C05,
	TBIOM_MISO_ENA            = 0x0C08,
	TBIOM_EE_I2C_WP           = 0x0806,
	TBIOM_F_DONE_ENA          = 0x0C07,
	TBIOM_FPGA_1_BASE_ADDR    = 0x0900,
	TBIOM_FPGA_2_BASE_ADDR    = 0x0E00
};

#else
// CPLD Addresses for TB3IOMB

enum CPLD_ADDR_IO {
	TBIOM_RS_232			  =	0x0801,
	TBIOM_TX2_BUF_ENA		  = 0x0802,
	TBIOM_EE_I2C_WP           = 0x0803,
	TBIOM_EE_I2C_CLK_ENA	  = 0x0804,
	TBIOM_FPGA1_RESET		  = 0x0805,
	TBIOM_FPGA2_RESET		  = 0x0806,
	TBIOM_MISO_ENA            = 0x0807,
	TBIOM_WP_FLASH_2		  = 0x0808,
	TBIOM_CS_IO_FLSH_1		  = 0x0809,
	TBIOM_CS_IO_FLSH_2        = 0x080A,
	TBIOM_WP_FLASH_1		  = 0x080B,
	TBIOM_CPLD_TESTPOINT_0    = 0x080C,
	TBIOM_CPLD_TESTPOINT_1    = 0x080D,
	TBIOM_CPLD_TESTPOINT_2    = 0x080E,
	TBIOM_CPLD_TESTPOINT_3    = 0x080F,
	TBIOM_CPLD_LED_0          = 0x0810,
	TBIOM_CPLD_LED_1          = 0x0811,
	TBIOM_F_PRGM_2            = 0x0812,
	TBIOM_CPLD_LED_2          = 0x08FF,  //not present in TB3IOMB
	TBIOM_CPLD_LED_3          = 0x08FF,  //not present in TB3IOMB
	TBIOM_CS_IO_EEPRM         = 0x08FF,  //not present in TB3IOMB
	TBIOM_F_DONE_ENA          = 0x08FF,  //not present in TB3IOMB

	TBIOM_FPGA_1_BASE_ADDR    = 0x0900,
	TBIOM_FPGA_2_BASE_ADDR    = 0x0A00
};

enum CPLD_ADDR_PM {
	TBPM_CPLD_LED_0          = 0x0C01,
	TBPM_CPLD_LED_1          = 0x0C02,
	TBPM_CPLD_TESTPOINT_0    = 0x0C03,
	TBPM_CPLD_TESTPOINT_1    = 0x0C04,
	TBPM_CPLD_TESTPOINT_2    = 0x0C05,
	TBPM_CPLD_TESTPOINT_3    = 0x0C06,
	TBPM_EE_I2C_WP           = 0x0C07,
	TBPM_EE_I2C_CLK_ENA	  	 = 0x0C08,
	TBPM_FPGA3_RESET		 = 0x0C09,
	TBPM_MISO_ENA            = 0x0C0A,
	TBPM_WP_FLASH_3		     = 0x0C0B,
	TBPM_CS_PM_FLSH_3		 = 0x0C0C,

	TBPM_FPGA_3_BASE_ADDR    = 0x0D00
};


// Address Offsets for FPGA_1
// This is coordinated with Address_Bus_Defs.v file in FPGA src code for FPGA1.
enum FPGA1_ADDR {
	FPGA1_WRITE_STORED_VAL_1		= 0x0001,
	FPGA1_WRITE_STORED_VAL_2		= 0x0002,
	FPGA1_WRITE_RESET_COUNT_CLK		= 0x0003,
	FPGA1_WRITE_LED_FUNCTION		= 0x0004,
	FPGA1_WRITE_LED_DIRECTLY		= 0x0005,

	FPGA1_WRITE_DIGOUT_BANK_A_FUNCT = 0x0006,
	FPGA1_WRITE_DIGOUT_BANK_B_FUNCT = 0x0007,
	FPGA1_WRITE_DIGOUT_BANK_C_FUNCT = 0x0008,
	FPGA1_WRITE_DIGOUT_BANK_D_FUNCT = 0x0009,
	FPGA1_WRITE_DIGOUT_BANK_E_FUNCT = 0x000A,
	FPGA1_WRITE_DIGOUT_BANK_F_FUNCT = 0x000B,

	FPGA1_WRITE_DIFFOUT_1234_FUNCT  = 0x000C,
	FPGA1_WRITE_DIFFOUT_5678_FUNCT  = 0x000D,

	FPGA1_WRITE_DIGOUT_BANK_A_RAILS = 0x000E,
	FPGA1_WRITE_DIGOUT_BANK_B_RAILS = 0x000F,
	FPGA1_WRITE_DIGOUT_BANK_C_RAILS = 0x0010,
	FPGA1_WRITE_DIGOUT_BANK_D_RAILS = 0x0011,
	FPGA1_WRITE_DIGOUT_BANK_E_RAILS = 0x0012,
	FPGA1_WRITE_DIGOUT_BANK_F_RAILS = 0x0013,

	FPGA1_WRITE_DIGOUT_BANK_A_MODE  = 0x0014,
	FPGA1_WRITE_DIGOUT_BANK_B_MODE  = 0x0015,
	FPGA1_WRITE_DIGOUT_BANK_C_MODE  = 0x0016,
	FPGA1_WRITE_DIGOUT_BANK_D_MODE  = 0x0017,
	FPGA1_WRITE_DIGOUT_BANK_E_MODE  = 0x0018,
	FPGA1_WRITE_DIGOUT_BANK_F_MODE  = 0x0019,

	FPGA1_WRITE_DIGOUT_BANK_AB_STATE = 0x001A,
	FPGA1_WRITE_DIGOUT_BANK_CD_STATE = 0x001B,
	FPGA1_WRITE_DIGOUT_BANK_EF_STATE = 0x001C,

	FPGA1_WRITE_DIFFOUT_ENABLE 		= 0x001D,
	FPGA1_WRITE_DIFFOUT_STATE 		= 0x001E,

	FPGA1_WRITE_DAC_ANLG_OUT_1			= 0x0020,
	FPGA1_WRITE_DAC_ANLG_OUT_2			= 0x0021,
	FPGA1_WRITE_DAC_ANLG_OUT_3			= 0x0022,
	FPGA1_WRITE_DAC_ANLG_OUT_4			= 0x0023,
	FPGA1_WRITE_DAC_ANLG_OUT_5			= 0x0024,
	FPGA1_WRITE_DAC_ANLG_OUT_6			= 0x0025,
	FPGA1_WRITE_DAC_ANLG_OUT_7			= 0x0026,
	FPGA1_WRITE_DAC_ANLG_OUT_8			= 0x0027,
	FPGA1_WRITE_DAC_DIG_IN_A			= 0x0028,
	FPGA1_WRITE_DAC_DIG_IN_B			= 0x0029,
	FPGA1_WRITE_DAC_DIG_IN_C			= 0x002A,
	FPGA1_WRITE_DAC_DIG_IN_D			= 0x002B,

    // commented out addresses for TB3IOMB
	//FPGA1_WRITE_DAC_RES_COS				= 0x002C,
	//FPGA1_WRITE_DAC_RES_SIN				= 0x002D,
	//FPGA1_WRITE_DAC_SSE_COS				= 0x002E,
	//FPGA1_WRITE_DAC_SSE_SIN				= 0x002F,

	// replaced with addresses valid for TB3IOMC
	// Most DAC's are DAC7311's
	// but Resolver uses AD5449, so we moved the 2
	// resolver addresses last in the list.
	FPGA1_WRITE_DAC_SSE_COS				= 0x002C,
	FPGA1_WRITE_DAC_SSE_SIN				= 0x002D,
	FPGA1_WRITE_DAC_RES_COS				= 0x002E,
	FPGA1_WRITE_DAC_RES_SIN				= 0x002F,

	FPGA1_WRITE_PWM_FREQ_LS16			= 0x0030,
	FPGA1_WRITE_PWM_FREQ_MS16			= 0x0031,
	FPGA1_WRITE_PWM_DTY_CYCL_LS16		= 0x0032,
	FPGA1_WRITE_PWM_DTY_CYCL_MS16		= 0x0033,

	FPGA1_WRITE_ENC1_FREQ_LS16    		= 0x0034,
	FPGA1_WRITE_ENC1_FREQ_MS16    		= 0x0035,
	FPGA1_WRITE_ENC1_INDEX_COUNT_LS16	= 0x0036,
	FPGA1_WRITE_ENC1_INDEX_COUNT_MS16	= 0x0037,
	FPGA1_WRITE_ENC1_DIR          		= 0x0038,
	FPGA1_WRITE_ENC1_STOP_AFTER_LS16	= 0x0039,
	FPGA1_WRITE_ENC1_STOP_AFTER_MS16	= 0x003A,
	FPGA1_WRITE_ENC1_MANUAL_STOP		= 0x003B,

	FPGA1_WRITE_ENC2_FREQ_LS16    		= 0x003C,
	FPGA1_WRITE_ENC2_FREQ_MS16    		= 0x003D,
	FPGA1_WRITE_ENC2_INDEX_COUNT_LS16	= 0x003E,
	FPGA1_WRITE_RELOAD_COUNT_CLK = 0x003F, // Legacy from proof of concept for Bi-Dir Bus, TB3IOMA
	FPGA1_WRITE_ENC2_INDEX_COUNT_MS16	= 0x0040,
	FPGA1_WRITE_ENC2_DIR          		= 0x0041,
	FPGA1_WRITE_ENC2_STOP_AFTER_LS16	= 0x0042,
	FPGA1_WRITE_ENC2_STOP_AFTER_MS16	= 0x0043,
	FPGA1_WRITE_ENC2_MANUAL_STOP		= 0x0044,

	FPGA1_WRITE_HALL_FREQ_LS16			= 0x0045,
	FPGA1_WRITE_HALL_FREQ_MS16			= 0x0046,
	FPGA1_WRITE_HALL_DIR          		= 0x0047,
	FPGA1_WRITE_HALL_PHASE         		= 0x0048,

		// ----

	FPGA1_READ_STORED_VAL_1			= 0x0005,
	FPGA1_READ_STORED_VAL_2			= 0x0006,
	FPGA1_READ_STORED_VAL_1XOR2		= 0x0007,

	FPGA1_READ_COUNT_CLK_LOW 		= 0x0008,
	FPGA1_READ_COUNT_CLK_HIGH		= 0x0009,
	FPGA1_READ_COUNT_WR_SV_1 		= 0x000A,

	FPGA1_READ_0x0000        		= 0x000B,
	FPGA1_READ_0xFFFF        		= 0x000C,
	FPGA1_READ_0xA5A5        		= 0x000D,
	FPGA1_READ_0x5A5A        		= 0x000E,
	FPGA1_READ_BUS_ID        		= 0x000F,
	FPGA1_READ_F_INIT_DONE_2 		= 0x0010,


	FPGA_READ_FIRMWARE_TIMESTAMP_1  = 0xF8,
	FPGA_READ_FIRMWARE_TIMESTAMP_2  = 0xF9,
	FPGA_READ_FIRMWARE_TIMESTAMP_3  = 0xFA,
	FPGA_READ_FIRMWARE_REVISION_1 	= 0xFB,
	FPGA_READ_FIRMWARE_REVISION_2 	= 0xFC

	// note1: timestamps and rev info are at same address in all FPGAs
};

// Address Offsets for FPGA_2
// This is coordinated with Address_Bus_Defs.v file in FPGA src code for FPGA2.
enum FPGA2_ADDR {
	FPGA2_WRITE_STORED_VAL_1		= 0x0001,
	FPGA2_WRITE_STORED_VAL_2		= 0x0002,
	FPGA2_WRITE_RESET_COUNT_CLK		= 0x0003,
	FPGA2_WRITE_LED_FUNCTION		= 0x0004,
	FPGA2_WRITE_LED_DIRECTLY		= 0x0005,
	FPGA2_WRITE_ADC_CAPTURE         = 0x0006,

	FPGA2_WRITE_IO_PIN_SWITCHES		= 0x0007,
	FPGA2_WRITE_SELF_TEST_SWITCHES	= 0x0008,
	FPGA2_WRITE_INTEGRATOR_SWITCH	= 0x0009,
	FPGA2_WRITE_LOOPBACK_MUX		= 0x000A,

	FPGA2_WRITE_ADC_CAPTURE_MODE 	= 0x000B,
	FPGA2_WRITE_ANLG_IN_FLTR_CLK    = 0x000C,
	FPGA2_WRITE_SSE_ACTION			= 0x000D,
	FPGA2_WRITE_SSE_DATA_1ST_16		= 0x000E,
	FPGA2_WRITE_SSE_DATA_2ND_16		= 0x000F,
	FPGA2_WRITE_SSE_CRC5			= 0x0010,


	FPGA2_WRITE_AD7175_ACTION_CMD	= 0x0011,
	FPGA2_WRITE_AD7175_DATA_MS_16	= 0x0012,
	FPGA2_WRITE_AD7175_DATA_LS_16	= 0x0013,

	FPGA2_WRITE_ANLG_IN_B1_SWITCH	= 0x0014,

	FPGA2_WRITE_DIGINMACHINE_ENC_MAP	= 0x15,
	FPGA2_WRITE_DIGINMACHINE_PWM_MAP	= 0x16,
	FPGA2_WRITE_DIGINMACHINE_RESET_ENC_COUNTS = 0x17,
	FPGA2_WRITE_DIGINMACHINE_ENC_INDEX_FREQ_DIV = 0x18,
	FPGA2_WRITE_RELOAD_COUNT_CLK	= 0x003F,

	// ----

	FPGA2_READ_STORED_VAL_1 = 0x05,
	FPGA2_READ_STORED_VAL_2 = 0x06,
	FPGA2_READ_STORED_VAL_1XOR2 = 0x07,

	FPGA2_READ_COUNT_CLK_LOW = 0x08,
	FPGA2_READ_COUNT_CLK_HIGH = 0x09,
	FPGA2_READ_COUNT_WR_SV_1 = 0x0A,

	FPGA2_READ_0x0000 = 0x0B,
	FPGA2_READ_0xFFFF = 0x0C,
	FPGA2_READ_0xA5A5 = 0x0D,
	FPGA2_READ_0x5A5A = 0x0E,

	FPGA2_READ_IO_PIN_SWITCHES		= 0x0F,
	FPGA2_READ_SELF_TEST_SWITCHES	= 0x10,
	FPGA2_READ_INTEGRATOR_SWITCH	= 0x11,
	FPGA2_READ_LOOPBACK_MUX		= 0x12,

	FPGA2_READ_ADC_A1 = 0x13,
	FPGA2_READ_ADC_A2 = 0x14,
	FPGA2_READ_ADC_A3 = 0x15,
	FPGA2_READ_ADC_A4 = 0x16,
	FPGA2_READ_ADC_A5 = 0x17,
	FPGA2_READ_ADC_A6 = 0x18,
	FPGA2_READ_ADC_A7 = 0x19,
	FPGA2_READ_ADC_A8 = 0x1A,

	FPGA2_READ_DIFF_IN = 0x1B,
	FPGA2_READ_DIG_IN = 0x1C,
	FPGA2_READ_SSE_1ST_16 = 0x1D,
	FPGA2_READ_SSE_2ND_16 = 0x1E,
	FPGA2_READ_SSE_STATUS = 0x1F,
	FPGA2_READ_SSE_DIAG_1 = 0x20,
	FPGA2_READ_SSE_DIAG_2 = 0x21,

	FPGA2_READ_AD7175_STATUS		= 0x22,
	FPGA2_READ_AD7175_DATA_MS_16	= 0x23,
	FPGA2_READ_AD7175_DATA_LS_16	= 0x24,

	FPGA2_READ_ANLG_IN_B1_SWITCH	= 0x0025,

	FPGA2_READ_DIGINMACHINE_ENC_PERIOD_MS_16	= 0x26,
	FPGA2_READ_DIGINMACHINE_ENC_PERIOD_LS_16	= 0x27,
	FPGA2_READ_DIGINMACHINE_ENCA_ON_TIME_MS_16	= 0x28,
	FPGA2_READ_DIGINMACHINE_ENCA_ON_TIME_LS_16	= 0x29,
	FPGA2_READ_DIGINMACHINE_ENCI_PERIOD_MS_16	= 0x2A,
	FPGA2_READ_DIGINMACHINE_ENCI_PERIOD_LS_16	= 0x2B,
	FPGA2_READ_DIGINMACHINE_ENC_DIR				= 0x2C,
	//
	FPGA2_READ_DIGINMACHINE_PWM1_PERIOD_MS_16	= 0x2D,
	FPGA2_READ_DIGINMACHINE_PWM1_PERIOD_LS_16	= 0x2E,
	FPGA2_READ_DIGINMACHINE_PWM1_ON_TIME_MS_16	= 0x2F,
	FPGA2_READ_DIGINMACHINE_PWM1_ON_TIME_LS_16	= 0x30,
	FPGA2_READ_DIGINMACHINE_PWM2_PERIOD_MS_16	= 0x31,
	FPGA2_READ_DIGINMACHINE_PWM2_PERIOD_LS_16	= 0x32,
	FPGA2_READ_DIGINMACHINE_PWM2_ON_TIME_MS_16	= 0x33,
	FPGA2_READ_DIGINMACHINE_PWM2_ON_TIME_LS_16	= 0x34,
    //
	FPGA2_READ_DIGINMACHINE_ENC_COUNTS_MS_16	= 0x35,
	FPGA2_READ_DIGINMACHINE_ENC_COUNTS_LS_16	= 0x36

};

enum FPGA3_ADDR {
	FPGA3_WRITE_STORED_VAL_1		= 0x0001,
	FPGA3_WRITE_STORED_VAL_2		= 0x0002,
	FPGA3_WRITE_RESET_COUNT_CLK		= 0x0003,
	FPGA3_WRITE_LED_FUNCTION		= 0x0004,
	FPGA3_WRITE_LED_DIRECTLY		= 0x0005,

	// ----

	FPGA3_READ_STORED_VAL_1			= 0x0005,
	FPGA3_READ_STORED_VAL_2			= 0x0006,
	FPGA3_READ_STORED_VAL_1XOR2		= 0x0007,

	FPGA3_READ_COUNT_CLK_LOW 		= 0x0008,
	FPGA3_READ_COUNT_CLK_HIGH		= 0x0009,
	FPGA3_READ_COUNT_WR_SV_1 		= 0x000A,

	FPGA3_READ_0x0000        		= 0x000B,
	FPGA3_READ_0xFFFF        		= 0x000C,
	FPGA3_READ_0xA5A5        		= 0x000D,
	FPGA3_READ_0x5A5A        		= 0x000E,
	FPGA3_READ_BUS_ID        		= 0x000F

	// note: timestamps and rev info are at same address in all FPGAs: 0xFA-0xFC

};


#endif

// CPLD_XINTF_ADDR(x) macro provides proper shifting and casting to assign a CPLD_ADDR
// enum value to an int* pointer.
#define CPLD_XINTF_ADDR(x) (Uint16 *)(((Uint32)(x))<<8)

// CPLD_FPGA_XINTF_ADDR(x,y) macro provides proper shifting and casting to  a CPLD_ADDR
// enum value, AND add an offset to that address
#define CPLD_FPGA_XINTF_ADDR(x,y) (Uint16 *)((((Uint32)(x))<<8) + y)

// CPLD_F1_XA(y) maco gives the proper address for a given offset, y, into FPGA1
#define CPLD_F1_XA(y) (Uint16 *)((((Uint32)(TBIOM_FPGA_1_BASE_ADDR))<<8) + y)
// CPLD_F2_XA(y) maco gives the proper address for a given offset, y, into FPGA2
#define CPLD_F2_XA(y) (Uint16 *)((((Uint32)(TBIOM_FPGA_2_BASE_ADDR))<<8) + y)
// CPLD_F3_XA(y) maco gives the proper address for a given offset, y, into FPGA3 on TB3PM
#define CPLD_F3_XA(y) (Uint16 *)((((Uint32)(TBPM_FPGA_3_BASE_ADDR))<<8) + y)


//--------------------------------------------------------------------------
// HISTORY: Original Scheme, Now Obsolete
//
// We used #defines representing 32-bit address values
//--------------------------------------------------------------------------
// Primary addresses for all CPLD functions *** NOT COMPLETE ***
// For J/K flip-flop functions, Write sets them HI. Read sets them LOW.
//      CS_IO_FLSH, FLASH SPI enable, J/K flip-flop, Active Low, Initialized to 1
//#define CS_IO_FLSH 0x080400
//#define CS_IO_FLSH_ALT 0x002400
//      CS_IO_EEPRM, EEPROM SPI enable, J/K flip-flop, Active Low, Initialized to 1
//#define CS_IO_EEPRM 0x0C0500
//#define CS_IO_EEPRM_ALT 0x080500
//      EE_I2C_WP, Write Protect for I2C EEProm J/K flip-flop, Active HI, Initialized to 0
//#define EE_I2C_WP 0x080600
//#define EE_I2C_WP_ALT 0x0x002600
//      F_DONE_ENA, Enable 3-state buffer, J/K flip-flop, Active Low, Initialized to 1
//#define F_DONE_ENA 0x0C0700
//#define F_DONE_ENA_ALT 0x080700
//      MISO_ENA, Enable 3-state buffer, J/K flip-flop, Active Low, Initialized to 1
//#define MISO_ENA 0x0C0800
//#define MISO_ENA_ALT 0x080900
// TB3CMA included several "experiments", one involved having
// 2 different addressing schemes, in case one didn't work.
// Every CPLD function has a primary address and an
// alternate address.  Both addresses happen to work
// at the current time.  The Alternate set may go away later.

#endif /* CPLDx_H */
