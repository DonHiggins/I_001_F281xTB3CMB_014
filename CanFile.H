// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     CanFile.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef CANFILEx_H
#define CANFILEx_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types

enum DIAG_ON_OFF {
	DIAG_ON_OFF_MULTI_PACKET_ASCII  =  0x01,
	DIAG_ON_OFF_MULTI_PACKET_HEX    =  0x02,
	DIAG_ON_OFF_RECV_MULTI_PACK_MSG =  0x04,
	DIAG_ON_OFF_RECV_CAN_MBOX       =  0x08
};

void diagRs232recvMultiPacket(struct MULTI_PACKET_BUF* mpb);
void diagRs232CanRecvMsg(Uint16 mbxNumber, Uint16 *msg);
void diagRs232sendDummyFile();
void canF_appendIntoDummyFileBuf(char *src, Uint16 charCount) ;
void canF_zeroDummyFileBufCounts(void) ;
void canF_zeroDummyFileBufOutCount(void);
Uint16 canF_readOutOfDummyFileBuf(char *dest, Uint16 reqCharCount);

enum CANOPEN_STATUS canF_recvDummyFile(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS canF_sendDummyFile(const struct CAN_COMMAND* can_command, Uint16* data);

extern Uint16 canF_diagOnOff;
extern Uint16 dummyFileBufInCharCount;
extern Uint16 dummyFileBufOutCharCount;
extern Uint16 dummyFileBufOutPacketSize;

#endif
