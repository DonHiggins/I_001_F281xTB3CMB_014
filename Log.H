// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Log.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef LOGx_H
#define LOGx_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// How many log records shall we allocate space for in our circular buffer?
// Actual space allocated is  (MAX_LOG_EVENT_INDEX + 1) * sizeof(LOG_EVENT_RECORD).
// Log records are 8-words (as of 8/24/2016) but see struct LOG_EVENT_RECORD, below.
// Not necessary to zero this, as circular buffer is not allocated unless we
// enable logging by un-commenting the #define for one of the LOG_ENABLE_xxx macros below.
#define MAX_LOG_EVENT_INDEX 50
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Turn on/off logging for Limit_Check (comment it out to turn off)
#define LOG_LIMCHK_ENABLE_LIMIT_CHECK_CALCULATIONS
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef LOG_LIMCHK_ENABLE_LIMIT_CHECK_CALCULATIONS
#define LOG_ENABL_CORE_INFRASTRUCTURE
#define LOG_LIMCHK_ADDTOLOG(X,Y) log_addToLog(X,Y)
#define LOG_LIMCHK_ADDTOLOG3(W,X,Y,Z) log_addToLogWith3Params(W,X,Y,Z)
#else
// if LOG_ENABLE_ANLG_IN_CALIBRATION is not #defined above, then the macros
// are defined as blank/null/void/do-nothing.
#define LOG_LIMCHK_ADDTOLOG(X,Y)
#define LOG_LIMCHK_ADDTOLOG3(W,X,Y,Z)
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Turn on/off logging for Anlg_In_Calibration (comment it out to turn off)
//#define LOG_AINCAL_ENABLE_ANLG_IN_CALIBRATION
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef LOG_AINCAL_ENABLE_ANLG_IN_CALIBRATION
#define LOG_ENABL_CORE_INFRASTRUCTURE
#define LOG_AINCAL_ADDTOLOG(X,Y) log_addToLog(X,Y)
#define LOG_AINCAL_ADDTOLOG3(W,X,Y,Z) log_addToLogWith3Params(W,X,Y,Z)
#else
// if LOG_ENABLE_ANLG_IN_CALIBRATION is not #defined above, then the macros
// are defined as blank/null/void/do-nothing.
#define LOG_AINCAL_ADDTOLOG(X,Y)
#define LOG_AINCAL_ADDTOLOG3(W,X,Y,Z)
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Turn on/off logging for FPGA Load (comment it out to turn off)
//#define LOG_FLOAD_ENABLE_FPGA_LOAD
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef LOG_FLOAD_ENABLE_FPGA_LOAD
#define LOG_ENABL_CORE_INFRASTRUCTURE
#define LOG_FLOAD_ADDTOLOG(X,Y) log_addToLog(X,Y)
#define LOG_FLOAD_ADDTOLOG3(W,X,Y,Z) log_addToLogWith3Params(W,X,Y,Z)
#else
// if LOG_FLOAD_ENABLE_FPGA_LOAD is not #defined above, then the macros
// are defined as blank/null/void/do-nothing.
#define LOG_FLOAD_ADDTOLOG(X,Y)
#define LOG_FLOAD_ADDTOLOG3(W,X,Y,Z)
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Turn on/off logging for Anlg_In_Classic (comment it out to turn off)
//#define LOG_ACLAS_ENABLE_ANLG_IN_CLASSIC_MATH
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef LOG_ACLAS_ENABLE_ANLG_IN_CLASSIC_MATH
#define LOG_ENABL_CORE_INFRASTRUCTURE
#define LOG_ACLAS_ADDTOLOG(X,Y) log_addToLog(X,Y)
#define LOG_ACLAS_ADDTOLOG3(W,X,Y,Z) log_addToLogWith3Params(W,X,Y,Z)
#else
// if LOG_ACLAS_ENABLE_ANLG_IN_CLASSIC_MATH is not #defined above, then the macros
// are defined as blank/null/void/do-nothing.
#define LOG_ACLAS_ADDTOLOG(X,Y)
#define LOG_ACLAS_ADDTOLOG3(W,X,Y,Z)
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Turn on/off logging for I2cEEProm (comment it out to turn off)
#define LOG_I2CEE_ENABLE_I2CEE_READ_TIMING
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef LOG_I2CEE_ENABLE_I2CEE_READ_TIMING
#define LOG_ENABL_CORE_INFRASTRUCTURE
#define LOG_I2CEE_ADDTOLOG(X,Y) log_addToLog(X,Y)
#define LOG_I2CEE_ADDTOLOG3(W,X,Y,Z) log_addToLogWith3Params(W,X,Y,Z)
#else
// if LOG_I2CEE_ENABLE_I2CEE_READ_TIMING (includes Reads & writes)is not #defined above, then the macros
// are defined as blank/null/void/do-nothing.
#define LOG_I2CEE_ADDTOLOG(X,Y)
#define LOG_I2CEE_ADDTOLOG3(W,X,Y,Z)
#endif


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// log_init() is instantiated whenever any other logging application is
// turned on.  main() makes a call to log_init()early on it its initialization code.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef LOG_ENABL_CORE_INFRASTRUCTURE
#define LOG_INIT log_init()
#else
// if LOG_ENABL_CORE_INFRASTRUCTURE is not #defined above, then the macros
// are defined as blank/null/void/do-nothing.
#define LOG_INIT
#endif


// NOTE: we have an Excel macro to automatically scan the Log.H
// file and extract info from this enum.  Out of laziness it is
// coded to match specifically on "LOG_EVENT_ID {" with 1 space
// between "LOG_EVENT_ID" and "{" so don't change the formatting
// of the Log.H file, or we won't match any more.
enum LOG_EVENT_ID {
	LOG_EVENT_ANLG_IN_CAL		= 0x10,
	LOG_EVENT_FPGA_LOAD			= 0x20,
	LOG_EVENT_ANLG_IN_CLASSIC	= 0x30,
	LOG_EVENT_I2CEE_READ_TIMING = 0x40,
	LOG_EVENT_I2CEE_WRITE_TIMING = 0x50,
	LOG_EVENT_LIMCHK_CALC		= 0x60,
	LOG_EVENT_LIMCHK_SEND_AVG	= 0x80,
	LOG_EVENT_LIMCHK_LOG_TEST	= 0x70,
	LOG_EVENT_OTHER             = 0x90
};


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef LOG_ENABL_CORE_INFRASTRUCTURE
void log_init(void);
void log_addToLogWith3Params(enum LOG_EVENT_ID logEventID, Uint16 param_1, Uint16 param_2, Uint16 param_3);
void log_addToLog(enum LOG_EVENT_ID logEventID, Uint16 param_1);
enum CANOPEN_STATUS log_readLog(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS log_startReadingLog(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS log_clear(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS log_t0Period(const struct CAN_COMMAND* can_command, Uint16* data);
#endif



typedef struct  LOG_EVENT_RECORD {     // bits  description
   Uint32 t0_interrupt_ctr;
   Uint32 t0_count_reg;
   Uint16 param_1;
   enum LOG_EVENT_ID event_id;
   Uint16 param_3;
   Uint16 param_2;
} ler;
// Had to make this a typedef and give it a type name, "ler" in order to
// use the sizeof macro to get the size of the structure.
// evidently, in this environment sizeof gives size in 16-bit words.


#endif
