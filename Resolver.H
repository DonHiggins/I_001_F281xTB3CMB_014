// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Resolver.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef RESOLVERx_H
#define RESOLVERx_H

#include "CanOpen.H"

extern Uint16 res_FixedShaftAngle;
extern Uint16 res_HiPrecisShaftAngle;
extern Uint16 res_attenuation;
extern Uint16 res_velocity;
extern Uint16 res_AmpRefInOut;
extern Uint16 res_RefFreq;
extern Uint16 res_RefDacValue;

void res_ShaftAngleOutTask(void);
void res_ConstVelocityTimerRoutine(void);
void res_init(void);
void res_calcSinCosFromHiPrecisShaftAngle(Uint16 res_HiPrecisShaftAngle,Uint16* sin,Uint16* cos);


enum CANOPEN_STATUS res_recvAmpRef(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS res_recvRefFreq(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS res_recvRefDacValue(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS res_recvShaftAngle(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS res_recvShaftAngleIincreasedPrecision(const struct CAN_COMMAND *can_command,Uint16 *data);


#endif
