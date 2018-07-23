// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Rs232Out.H
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef RS232OUTx_H
#define RS232OUTx_H
#include "stdbool.h"            // needed for bool data types

bool r232Out_outChars(char* outChars, int charLength);
bool r232Out_outCharsNT(char* outChars);
void r232Out_circBufOutput(void);

void r232Out_Command_Ack(Uint16 comintCode,Uint16 Data, bool dataPresent,char commandChar);
void r232Out_Command_Nak(char* rxdata,Uint16 rxdataLength);
bool r232Out_transmit_status_busy(void);
void rs232_DiagPortalMsg(void);


#endif
