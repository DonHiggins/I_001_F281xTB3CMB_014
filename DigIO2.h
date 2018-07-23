// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     DigIO2.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef DIGIO2x_H
#define DIGIO2x_H

extern Uint16 digio2_EncInMap;
extern Uint16 digio2_HallInMap;
extern Uint16 digio2_PwmInMap;
extern Uint16 digio2_EncIndexDivisor;
extern Uint16 digio2_DigOutSignalAssignment[6];
extern Uint16 digio2_DiffOutSignalAssignment[2];
extern Uint16 digio2_DigOutMode[6];
extern Uint16 digio2_DigOutRails[6];
extern Uint16 digio2_DigOutLevel[3];
extern Uint16 digio2_DiffOutLevel;
extern Uint16 digio2_DiffOutEnable;

enum CANOPEN_STATUS digio2_recvHallInMap(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_recvEncInMap(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_recvPwmInMap(const struct CAN_COMMAND *can_command,Uint16 *data) ;

enum CANOPEN_STATUS digio2_sendPwmIn1Period(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendPwmIn1OnTime(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendPwmIn2Period(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendPwmIn2OnTime(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendEncAInPeriod(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendEncAInOnTime(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendEncIInPeriod(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendEncIInOnTime(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendEncInDir(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendPwmReadingClassic(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio2_sendEncCounts(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_sendEncReadingClassic(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_recvEncIndexDivisor(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_sendHallReadingClassic(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_recvDigOutSignalAssignment(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_recvDiffOutSignalAssignment(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_recvDigOutMode(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_recvDigOutRails(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_recvDigOutLevel(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_recvDiffOutLevel(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio2_recvDiffOutEnable(const struct CAN_COMMAND *can_command,Uint16 *data);

void digio2_Init(void);
void digio2_Init_Enc_Index_Freq_Div(void);
void digio2_initDigOutSignalAssignment(void);
void digio2_initDiffOutSignalAssignment(void);
void digio2_initDigOutMode(void);
void digio2_initDigOutRails(void);
void digio2_initDigOutLevel(void);
void digio2_initDiffOutLevel(void);
void digio2_initDiffOutEnable(void);

#endif
