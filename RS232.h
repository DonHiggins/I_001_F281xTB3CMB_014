// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     RS232.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef RS232x_H
#define RS232x_H

#include "stdbool.h"            // needed for bool data types

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for rs232_Comint_Ack() non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
void rs232_setCommandAck(Uint16 commandCode,Uint16 dataWord, bool dataWordReceived);
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-




void rs232_store_int_vectors_in_PIE(void);
void rs232_scia_fifo_init();
void rs232_setBaudRateDefault(void);
void rs232_enable_PIE_int();
void rs232_BgTask_ooad(void);
void rs232_BgTask_ts3StartUp(void);
void rs232_BgTaskInit(void);
void rs232_BgTask_TxDone(void);
void rs232_commandDecode(void);
void rs232_Init(void);
void rs232_changeBaudParityEtc(char* params,int length);
void rs232_BgTask_Rs232BaudChange(void);
void rs232_reportBaudParityEtc(void);

bool rs232_getBaudRateFromBrrValue(Uint16 baudRateRegisterValue, Uint32 *baud32);
bool rs232_getBrrValueFromBaudRate(Uint32 baud32,Uint16 *baudRateRegisterValue);
void rs232_BgTask_Rs232BreakOrError(void);
void rs232_BgTask_AckAutobaud(void);

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   RS232 Status -- status of firmware operations, not H/W
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
struct  RS232_STATUS_BITS {  // bit    description
   Uint16 TX_BUSY:1;         // 0      Transmission in progress
   Uint16 rsvd1:15;          // 15:1   reserved
};

union RS232_STATUS_UNION {
   Uint16                    all;
   struct RS232_STATUS_BITS  bit;
};

struct RS232_STATUS {
	union RS232_STATUS_UNION RS232_STAT;
};

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for RS232 non-public variable : Rs232Status
bool rs232_transmit_status_busy(void);
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for RS232 non-public variable :kicking off a transmit operation
void rs232_transmit_characters(char* chars_to_transmit,int count);
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for RS232 indicator that tx FIFO is busy
bool rs232_txFifo_Busy(void);
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

#endif
