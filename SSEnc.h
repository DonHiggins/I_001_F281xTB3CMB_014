// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     SSEnc.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef SSENCx_H
#define SSENCx_H

#include "CanOpen.H"


extern Uint16 ssEnc_velocity;
extern Uint16 ssEnc_HiPrecisShaftAngle;

enum CANOPEN_STATUS ssEnc_recvShaftAngleIincreasedPrecision(const struct CAN_COMMAND *can_command,Uint16 *data);

void ssEnc_init(void);
void ssEnc_ShaftAngleOutTask(void);
void ssEnc_ConstVelocityTimerRoutine(void);


#endif
