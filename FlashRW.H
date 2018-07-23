// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     FlashRW.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef FLASHRWx_H
#define FLASHRWx_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types
#include "CanOpen.H"

//extern Uint16 frwFlashAddr16[2];
extern union CANOPEN16_32 frwFlashAddr;
extern Uint16 mcsFileRecvParseStatus;
extern union CANOPEN16_32 mcsFileSendByteCount;
extern Uint16 mcsFileSendByteAddr;


void diagRs232readFlashStatusReg(Uint16 status,Uint16 functionCode);
void diagRs232FlashRWBuff(Uint16 countWords);
void diagRs232readFlashRDID(Uint16 status,Uint16 status_2);

void frw_test2005(Uint16 dataWord);
void frw_test4000(Uint16 dataWord);
void frw_test4001(Uint16 dataWord);
void frw_test4002(Uint16 dataWord);
void frw_test4003(Uint16 dataWord);
void frw_test4004(Uint16 bytesToWrite);
void frw_test4005(Uint16 seedValue);
void frw_test4006(Uint16 whichFlash);

void frw_SpiFlashInit();
void frw_SpiFlashTask();
void frw_MiscFlashTasks();
void frw_diagFlashTasks();
void frw_SetWhichFlash(Uint16 whichFlash); //  1=FLASH_1, 2=FLASH_2
Uint16 frw_GetWhichFlash(void);
void frw_initFpgaLoadPins(void);

enum CANOPEN_STATUS frw_readFlashStatusReg(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_readFlashRDID(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_releasePowerdownRES(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_startMcsFileRecv(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_mcsFileRecvStatus(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_mcsFileRecvData(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_bulkEraseFlashSend(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_bulkEraseFlashRecv(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_diagDisplFlashPage(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_mcsFileSendData(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_startLoadFpgaFromFlash(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_turnOnFlashWriteProtect(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_readWhichFlashChip(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_writeWhichFlashChip(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_fastFileRecvData(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS frw_runTest2005(const struct CAN_COMMAND* can_command, Uint16* data);


// Used in MCS File Recv to describe status conditions
enum MCS_FILE_RECV_STATUS {
	MCS_FILE_RECV_IDLE               =  0,
	MCS_FILE_READY_TO_RECEIVE        =  1,
	MCS_FILE_STOPPED_FOR_ERROR       =  2,
	MCS_FILE_RECV_BKGND_OPERATION    =  3,
	MCS_FILE_RECV_BKGND_PROBLEM      =  4,
	MCS_FILE_RECV_EOF_BKGND          =  5,
	FAST_FILE_READY_TO_RECEIVE		 =  6
};

enum MCS_FILE_RECV_ERROR {
	MCS_ERR_NO_ERROR                 =  0,
	MCS_ERR_RECV_NOT_IN_PROG         =  1,
	MCS_ERR_BAD_ADDRESS              =  2,
	MCS_ERR_WRITE_IN_PROG            =  3,
	MCS_ERR_MCS_PARSE_ERR            =  4,
	MCS_ERR_RECV_BUFF_OVERRUN        =  5

};
enum MISC_FLASH_TASK_OPERATION {
	MISC_FLASH_TASK_NO_OP            =  0,
	MISC_FLASH_TASK_WRITE_FLASH      =  1
};
enum LOAD_FPGA_FROM_FLASH_STATE {
	LOAD_FPGA_NO_OP                  =  0,
	LOAD_FPGA_AT_STARTUP             =  1,
	LOAD_FPGA_TEMP_EXIT              =  2,
	LOAD_FPGA_ON_CAN_REQUEST         =  3,
	LOAD_FPGA_READ_OFFSET_LENGTH     =  4,
	LOAD_FPGA_PROG_AND_INIT          =  5,
	LOAD_FPGA_CLOCK_DATA_IN          =  6,
	LOAD_FPGA_COMPLETE_CONFIGURATION =  7,
	LOAD_FPGA_ERROR_CLEANUP          =  99
};

enum LOAD_FPGAS_AT_STARTUP_STATUS {
	F_LOAD_NOT_STARTED			= 0,
	F_LOAD_IN_PROGRESS			= 1,
	F_LOAD_COMPLETE				= 2,
	F_LOAD_ERROR				= 3
};

enum F_LOAD_CTRL_OPS {
	F_LOAD_RESET_LOW           = 0,
	F_LOAD_PROG_OUT_HI         = 1,
	F_LOAD_PROG_LOW            = 2,
	F_LOAD_READ_INIT           = 3,
	F_LOAD_PROG_IN             = 4,
	F_LOAD_READ_DONE           = 5,
	F_LOAD_RESET_HI            = 6
};

void frw_SetFpgaCtrlLines(enum F_LOAD_CTRL_OPS ctrlLineOp);
Uint16 frw_ReadFpgaCtrlLines(enum F_LOAD_CTRL_OPS ctrlLineOp);
enum LOAD_FPGAS_AT_STARTUP_STATUS frw_GetLoadFpgasAtStartupStatus(void);

#endif /* FLASHRWx_H */
