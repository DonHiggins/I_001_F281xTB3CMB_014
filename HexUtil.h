// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     HexUtil.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef HEXUTILx_H
#define HEXUTILx_H

#include "stdbool.h"

Uint16 hexUtil_hexCharToBin(char c);
bool hexUtil_isValidAsciiHexDigits(char *c,Uint16);
char *hexUtil_binTo4HexAsciiChars(char *c,Uint16 dataWord);
char *hexUtil_binTo2HexAsciiChars(char *c,Uint16 dataWord);
char *hexUtil_hexCharsToBin(char *hexAscii, Uint16 numDigits, Uint16 *result) ;
char *hexUtil_binTo5DecAsciiChars(char *c,Uint16 dataWord);
char *hexUtil_binToDecAsciiCharsZeroSuppress(char *c,Uint16 dataWord);
char* hexUtil_decCharsToBin(char *decAscii, Uint16 *result);
char* hexUtil_decCharsToBin32(char *decAscii, Uint32 *result32);
char *hexUtil_bin32To10DecAsciiChars(char *c,Uint32 dataWord);
char *hexUtil_bin32ToDecAsciiCharsZeroSuppress(char *c,Uint32 dataWord);

#endif
