// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Comint.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef COMINTx_H
#define COMINTx_H

#include "stdbool.h"

void comint_comint(Uint16 command,Uint16 dataWord,bool dataPresent);
void comint_comintSpeedDial(Uint16 command);
void comint_DisplaySpeedDialList(void);
void comint_comintTextCommand(Uint16 commmand, char* textData, Uint16 length);
void comint_comintHelp(void);
void comint_DisplayCpldAddrConfig(void);


#endif
