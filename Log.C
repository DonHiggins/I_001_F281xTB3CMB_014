// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Log.C
//
// This provides diagnostic facility to capture and store info about events,
// which can subsequently be communicated to the host for display and analysis.
//
// My intention is to code this so that a single switch (define value) can either
// instantiate it in the code, or else obliterate all traces from the compilation.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File

#include "stdbool.h"            // needed for bool data types
#include "timer0.h"
#include "CanOpen.h"
#include "log.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// LOG_ENABL_CORE_INFRASTRUCTURE
//
// Following are variables and code implementing the "core infrastructure"
// for the logging facility.
// This includes procedures called from elsewhere to record a log entry,
// and also utilities used to review the log, for ex. via CAN.
// If the macro LOG_ENABL_CORE_INFRASTRUCTURE is defined in code module
// Log.H, then the following variables and code are instantiated,
// otherwise they are entirely omitted from the build, and consume
// no resources.  Typically, LOG_ENABL_CORE_INFRASTRUCTURE is not itself
// defined directly, but instead it is turned on or off by other macro defines
// in Log.H to turn on/off logging in specific applications.
// For example, in Log.H, the macro LOG_ENABLE_ANLG_IN_CALIBRATION, when
// defined, turns on procedure calls in the Analog In Calibration code
// in (AnlgIn.C) to call routines in the logging "core infrastructure" code,
// here, and it also defines the macro LOG_ENABL_CORE_INFRASTRUCTURE to
// instantiate the logging "core infrastructure" code and variables.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef LOG_ENABL_CORE_INFRASTRUCTURE
Uint16 log_eventIndex;
Uint16 log_eventRecCount;
// EXPERIMENTAL: try locating log data in external RAM chip
#pragma DATA_SECTION(log_events, ".extram");
struct  LOG_EVENT_RECORD log_events[MAX_LOG_EVENT_INDEX + 1];

void log_init(void){
	log_eventIndex = 0;     // index identifies the next spot in buffer to write to.
	log_eventRecCount = 0;  // total count of records writen to buffer since power up.
}

void log_addToLogWith3Params(enum LOG_EVENT_ID logEventID, Uint16 param_1, Uint16 param_2, Uint16 param_3){
	// here we do something to save the log info
	if (log_eventIndex <= MAX_LOG_EVENT_INDEX){
		log_events[log_eventIndex].event_id = logEventID;
		log_events[log_eventIndex].param_1 = param_1;
		log_events[log_eventIndex].param_2 = param_2;
		log_events[log_eventIndex].param_3 = param_3;
		log_events[log_eventIndex].t0_interrupt_ctr = timer0_interrupt_count_value();
		log_events[log_eventIndex].t0_count_reg = timer0_count_reg_value();
		log_eventIndex++;
		log_eventRecCount++;
		if (log_eventIndex > MAX_LOG_EVENT_INDEX){
			log_eventIndex = 0; // we are a circular buffer and we wrap.
		}
	}
}

void log_addToLog(enum LOG_EVENT_ID logEventID, Uint16 param_1){
	// here we do something to save the log info
	if (log_eventIndex <= MAX_LOG_EVENT_INDEX){
		log_events[log_eventIndex].event_id = logEventID;
		log_events[log_eventIndex].param_1 = param_1;
		log_events[log_eventIndex].param_2 = 0;
		log_events[log_eventIndex].param_3 = 0;
		log_events[log_eventIndex].t0_interrupt_ctr = timer0_interrupt_count_value();
		log_events[log_eventIndex].t0_count_reg = timer0_count_reg_value();
		log_eventIndex++;
		log_eventRecCount++;
		if (log_eventIndex > MAX_LOG_EVENT_INDEX){
			log_eventIndex = 0; // we are a circular buffer and we wrap.
		}
	}
}

Uint16 log_readLogRecNum;
Uint16 log_readLogOffsetInRec;

enum CANOPEN_STATUS log_readLog(const struct CAN_COMMAND* can_command, Uint16* data){
	// transmit next 32 bits from log_events buffer
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16* log_readPtr;
	Uint16  recNumInCircularBuf;

	// Since buffer is circular, if we have already looped,
	// then the oldest record to return is at index log_eventIndex.
	// While log_readLogRecNum starts at 0, and counts up as we read log records,
	// we add log_eventIndex + log_readLogRecNum and wrap that number back to 0
	// if it is > MAX_LOG_EVENT_INDEX, the maximum index of log records
	// stored in the circular buffer.
	if (log_eventRecCount <= MAX_LOG_EVENT_INDEX) {
		recNumInCircularBuf = log_readLogRecNum;
	} else {
		recNumInCircularBuf = log_eventIndex + log_readLogRecNum;
		if (recNumInCircularBuf > MAX_LOG_EVENT_INDEX) {
			recNumInCircularBuf -= (MAX_LOG_EVENT_INDEX + 1);
		}
	}

	log_readPtr = (Uint16*)&log_events[recNumInCircularBuf];
	*(data+2) = *(log_readPtr+(log_readLogOffsetInRec++)); //MboxC, LSW
	*(data+3) = *(log_readPtr+(log_readLogOffsetInRec++));   //MboxD, MSW

	if (log_readLogOffsetInRec >= sizeof(ler)) {   // size of LOG_EVENT_RECORD
		log_readLogRecNum++;
		log_readLogOffsetInRec = 0;
	}
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS log_startReadingLog(const struct CAN_COMMAND* can_command, Uint16* data){
	// Returns 2 16-bit values:
	//    the number of log records available, msw
	//    length of a log record (in bytes), lsw.
	// This also initializes  counters used by log_readLog(), above to sequentially
	// read all log data in 32-bit chunks.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	if (log_eventRecCount > MAX_LOG_EVENT_INDEX){
		*(data+3) = (MAX_LOG_EVENT_INDEX + 1);   //MboxD
	} else {
		*(data+3) = log_eventRecCount;   //MboxD
	}
	*(data+2) = sizeof(ler); //MboxC -- size of LOG_EVENT_RECORD

	log_readLogRecNum = 0;		// counters used by log_readLog()
	log_readLogOffsetInRec = 0;
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS log_clear(const struct CAN_COMMAND* can_command, Uint16* data){
	// Empty out the log data
	log_init();
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS log_t0Period(const struct CAN_COMMAND* can_command, Uint16* data){
	// Sends 16-bit value equal to number of uSec in the t0 timer period
	// PC client uses this to interpret the time log information.
	// DSP uses  2 32-bit counters: t0 interrupt counter, and t0 count register.
	// Specifically, 1 count for t0 interrupt counter == t0 timer period.
	// t0 count register counts at 150MHz == 6.667E-9 per count
	// For each t0 interrupt, the count register starts at N and counts down to 0 . . .
	// where N = (t0 timer period in uSec / 1,000,000) * 150,000,000
	// Computationally, 1 count register count = (t0 timer period)/N
	//
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	*(data+2) = TIMER_0_PERIOD_IN_USEC; //MboxC, LSW
	*(data+3) = 0;   //MboxD, MSW
	return CANOPEN_NO_ERR;
}

#endif
