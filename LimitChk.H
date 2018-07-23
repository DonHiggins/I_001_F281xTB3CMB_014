// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//  LimitChk.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef LIMITCHKx_H
#define LIMITCHKx_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types

enum LIMIT_CHECK_COMPARISON {
	LCC_WITHIN_INNER_LIMITS    =  0,
	LCC_ABOVE_OUTER_HI_LIMIT   =  1,
	LCC_ABOVE_INNER_HI_LIMIT   =  2,
	LCC_BELOW_INNER_LOW_LIMIT  =  3,
	LCC_BELOW_OUTER_LOW_LIMIT  =  4
};

enum LC_TEST_ENABLE {
	LIMCHK_TEST_DISABLED    =  0,
	LIMCHK_TEST_ENABLED   =  1
};

enum LC_REQUEST_ENABLE_DISABLE {
	LIMCHK_NO_PENDING_REQ    =  0,
	LIMCHK_REQ_TEST_ENABLE   =  1,
	LIMCHK_REQ_TEST_DISABLE   =  2
};

enum LC_REQUEST_SYNC {
	LIMCHK_NO_PENDING_SYNC	=  0,
	LIMCHK_REQ_SYNC   		=  1
};

enum IO_TYPE {
	IO_TYPE_UNDEFINED		=  0,
	IO_TYPE_ANLG_IN_1234	=  1,
	IO_TYPE_ANLG_IN_5678	=  2,
	IO_TYPE_DIG_IN_16		=  3,
	IO_TYPE_DIFF_IN_8		=  4,
	IO_TYPE_FREQ			=  5,
	IO_TYPE_LEVEL			=  6
};

enum LIMIT_CHECK_INPUT_INDEX {
	LCI_ANLG_IN_A1		=  0,
	LCI_ANLG_IN_A2		=  1,
	LCI_ANLG_IN_A3		=  2,
	LCI_ANLG_IN_A4		=  3,
	LCI_ANLG_IN_B1		=  4,
	LCI_ANLG_IN_B2		=  5,
	LCI_ANLG_IN_B3		=  6,
	LCI_ANLG_IN_B4		=  7,
	LCI_PS_I			=  25,
	LCI_PS_V			=  26,
	LCI_24VPS_I			=  37
};

// following values are used in Classic test station to
// indicate status of each of the 8 limit-check test channels
enum LIMIT_CHECK_TEST_STATUS {
	NOT_TESTING			=	0,
	TESTING				=	1,
	FINISHED_TEST		=	2,
	SPIKES_FAILED		=	8,
	LIMITS_FAILED		=	9
};

enum LIMIT_CHECK_STATE_MACHINE {
	LCSM_PERFORM_TEST_END		=	0,
	LCSM_PERFORM_TEST			=	1,
	LCSM_PERFORM_TEST_WAIT		=	2,
	LCSM_PERFORM_TEST_UNTIL		=	3,
	LCSM_WAIT_FOR_SYNC			=	4
};

union LC16_32 {
	Uint32 		all;
	struct TWO_16S {
		Uint16 lsw;
		Uint16 msw;
	} words;
};


// In Classic test station outer limits were called "Noise" limits
// inner limits were called "Limits"'
struct HI_LOW_IN_OUT_UINT16_LIMITS
	{
		Uint32 Hi_Outer;
		Uint32 Hi_Inner;
		Uint32 Low_Inner;
		Uint32 Low_Outer;
	};

struct LIMIT_CHECK_PARAMETERS
	{
	   Uint32 mSecEnableTimeStamp;
	   enum LC_TEST_ENABLE enableTest;				// milisec state machine implements changes to enableTest
	   enum LC_REQUEST_ENABLE_DISABLE reqEnable;	// async requests are logged in reqEnable
	   enum LC_REQUEST_SYNC reqSync;				// async requests are logged in reqSync

	   enum LIMIT_CHECK_STATE_MACHINE limitCheckState;
	   enum LIMIT_CHECK_INPUT_INDEX limitCheckInput;
	   Uint16 timeStart;
	   Uint16 timeEnd;
	   Uint16 allowedFails;
	   Uint16 detectedFails;
	   Uint16 noHalt;	// 0x01 --> don't stop when detectedFails >= allowedFails
	   enum IO_TYPE ioType;
	   struct HI_LOW_IN_OUT_UINT16_LIMITS limits;
	   Uint32 measValue;
	   Uint32 minValue;
	   Uint32 maxValue;
	   Uint32 avgValue;
	   Uint32 sumValue;
	   Uint32 sumCount;
	   enum LIMIT_CHECK_TEST_STATUS testStatus;
	   enum LIMIT_CHECK_COMPARISON comparisonResult;

	};

struct LIMIT_CHECK_INPUTS
	{
		enum IO_TYPE ioType;
		// function pointer for checking present measurement against limits
		enum LIMIT_CHECK_COMPARISON (*limitCheckProcess)(struct HI_LOW_IN_OUT_UINT16_LIMITS* limits,
				                      Uint16* offset, Uint16* readAnlgInFpga, Uint32* measValue);
        Uint16* offset;
        Uint16* readFpga;
		// and lots more later //
	};

extern Uint16 limChkAnlgInChannel;
extern struct HI_LOW_IN_OUT_UINT16_LIMITS limChkAnlgInLimits;
extern struct HI_LOW_IN_OUT_UINT16_LIMITS limChkAnlgInLimitsClassic;
extern struct LIMIT_CHECK_PARAMETERS limitCheckParams[8];

enum CANOPEN_STATUS limChkAnlgInComparison(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limChkAnlgInClassic(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS limchkSendLimits(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkRecvLimits(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkSendWhichInput(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkRecvWhichInput(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkSendTestStatus(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkRecvTimeStartEnd(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkSendTimeStartEnd(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkSendFails(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkSendAvgValue(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkSendMinMaxValue(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkRecvEnable(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkSendMeasValueClassic(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkRecvSync(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS limchkResetOneChannel(const struct CAN_COMMAND* can_command, Uint16* data);


void limchkInit(void);
void limchkInitOneChannel(Uint16 limitCheckChannel);
void limChkBackgroundMeasurements(void);
void limChkStateMachine(void);
void limChkStateMachineChannel(Uint16 limitCheckChannel);
void limchkPreTestInit(Uint16 limitCheckChannel);
void limChkOpenHvPsRelay(void) ;
void limChkHvPsShutdown(void);
void limChkStateMachineOnTestFail(Uint16 limitCheckChannel);
void limChkStateMachineResetOneChannel(Uint16 limitCheckChannel);
void limChkBackgroundMinMaxAvg(struct LIMIT_CHECK_PARAMETERS* lcParams);
void limChkBackgroundMinMaxAvgInit(struct LIMIT_CHECK_PARAMETERS* lcParams);
void limChkBackgroundComputeAvg(struct LIMIT_CHECK_PARAMETERS* lcParams);


enum LIMIT_CHECK_COMPARISON limChkAnlgIn5678(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* ain_offset,
		                                     Uint16* readAnlgInFpga, Uint32* measValue);
enum LIMIT_CHECK_COMPARISON limChkAnlgIn1234(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* ain_offset,
		                                     Uint16* readAnlgInFpga, Uint32* measValue);
enum LIMIT_CHECK_COMPARISON limChkDigIn16(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
		                                     Uint16* readInFpga, Uint32* measValue);
enum LIMIT_CHECK_COMPARISON limChkDiffIn8(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
		                                     Uint16* readInFpga, Uint32* measValue);
enum LIMIT_CHECK_COMPARISON limChkFreqTest(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
        										Uint16* readInFpga, Uint32* measValue);
enum LIMIT_CHECK_COMPARISON limChkLevelTest(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
												Uint16* readInFpga, Uint32* measValue);
enum LIMIT_CHECK_COMPARISON limChkUndefinedTest(struct HI_LOW_IN_OUT_UINT16_LIMITS* InLimits, Uint16* offset,
        										Uint16* readInFpga, Uint32* measValue);
#define NUM_LIM_CHK_CHANNELS 8
#define CAN_INDEX_OF_LIM_CHK_CHANNEL_1 0x2023


#endif /* LIMITCHKx_H */
