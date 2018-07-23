// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     LED.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef LEDx_H
#define LEDx_H

void led_cpldLedIoInit(void);
void led_toggle4DspLeds(void);
void led_init4DspLeds(void);
void led_manageDspLEDsUnderTimer0(void);
void led_manageCpldIoLEDsUnderTimer0(void);
void led_manageFpgaLedsUnderTimer0(void);
void led_DspLedInit(void);
void led_FpgaLedInit(void);
void led_synchronizedSlowHeartbeat(void);

enum LED_PATTERN {
	LED_PATTERN_DIRECT               =  0x000F,
	LED_PATTERN_INCREMENT            =  0x1001,
	LED_PATTERN_BLINK_ERROR          =  0x1002,
	LED_PATTERN_SLOW_HEARTBEAT       =  0x1003
};

enum LED_ERROR_NUMBER {
	LED_ERROR_BAD_FLASH_BLOCK_0000   =  2, // Fail loading FPGA from Flash: Bad hash in header block
	LED_ERROR_BAD_INIT_B_NOT_LOW     =  3, // Fail loading FPGA from Flash: INIT_B line did not go LOW
	LED_ERROR_BAD_INIT_B_NOT_HI      =  4, // Fail loading FPGA from Flash: INIT_B line did not go HI
	LED_ERROR_FPGA_CRC               =  5, // Fail loading FPGA from Flash: CRC failure
	LED_ERROR_DONE_LINE_LOW          =  6, // Fail loading FPGA from Flash: DONE Line did not go HI
	LED_ERROR_7                      =  7, // Used temporarily for testing 2/11/2015
	LED_ERROR_NO_POWER_GOOD_IO       =  8  // main() did not see Power_Good, skipped loading TB3IOMC FPGAs
};

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for Timer0 non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
void led_setDspLedPattern(enum LED_PATTERN pattern);
enum LED_PATTERN led_getDspLedPattern(void);
void led_dspLedErrMsg(enum LED_ERROR_NUMBER count);
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

enum LED_CPLD_PATTERN {
	LED_CPLD_PATTERN_DIRECT_0             =  0x0000, //value from 0-F writes pattern directly in LED's
	LED_CPLD_PATTERN_DIRECT               =  0x000F, //  directly to LED's
	LED_CPLD_PATTERN_SLOW_HEARTBEAT       =  0x1003,
	LED_CPLD_PATTERN_DISABLED             =  0x2003
};

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for Timer0 non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
void led_setLedCpldIoPattern(enum LED_CPLD_PATTERN pattern);
void led_setLedCpldPmPattern(enum LED_CPLD_PATTERN pattern);
enum LED_CPLD_PATTERN led_getCpldIoLedPattern(void);
void led_LedCpldIoInit(void);
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

enum LED_FPGA_PATTERN {
	LED_FPGA_PATTERN_DIRECT_0						=  0x0000,  //value from 0-F writes pattern directly in LED's
	LED_FPGA_PATTERN_DIRECT							=  0x000F,  //  directly to LED's
	LED_FPGA_PATTERN_COUNT_CLOCK					=  0x0010,  //bits[28:25] counts up at 0.89 sec
	LED_FPGA_FROM_COUNT_WR_STORERD_VAL_1			=  0x0011,  //display values from FPGA test
	LED_FPGA_PATTERN_INTERNAL_SLOW_HEARTBEAT		=  0x0012,  //heartbeat generated within FPGA
	LED_FPGA_PATTERN_DSP_SLOW_HEARTBEAT				=  0x0013,  //heartbeat sync'd with DSP
	LED_FPGA_PATTERN_DISABLED						=  0x2003
};

void led_setLedFpga1Pattern(enum LED_FPGA_PATTERN pattern);
void led_setLedFpga2Pattern(enum LED_FPGA_PATTERN pattern);
void led_setLedFpga3Pattern(enum LED_FPGA_PATTERN pattern);

extern enum LED_FPGA_PATTERN led_fpga1Pattern;
extern enum LED_FPGA_PATTERN led_fpga2Pattern;
extern enum LED_FPGA_PATTERN led_fpga3Pattern;

extern enum LED_CPLD_PATTERN led_cpldIoPattern;
extern enum LED_CPLD_PATTERN led_cpldPmPattern;

#endif
