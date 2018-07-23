// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     HexUtil.C
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
// NOTE: "DSP281x_Device.h" defines "Uint16" data type

#include "HexUtil.H"

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   Convert one Ascii Hex digit to its 4-bit binary value
//   Assumes we already checked for a legal Ascii-Hex Digit
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Uint16 hexUtil_hexCharToBin(char c) {
	Uint16 result;
    result = c & 0x000F;
    if (c > 0x40) {
    	result += 9;
    }
    return result;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   Convert a fixed # of Ascii Hex digits to a 16-bit binary value
//   Assumes we already checked for a legal Ascii-Hex Digit
//   Return char pointer to next character after the ones we converted to binary.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
char* hexUtil_hexCharsToBin(char *hexAscii, Uint16 numDigits, Uint16 *result) {

	*result = 0;
	while (numDigits-- > 0){
		*result <<= 4;
		*result += *hexAscii & 0x000F;
		if (*hexAscii > 0x40) {
			*result += 9;
		}
		hexAscii++; // increment pointer
	}
    return hexAscii;
}


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   Check a contiguous set of characters and verify that they are
//   all legal Ascii-Hex Digits
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool hexUtil_isValidAsciiHexDigits(char *c,Uint16 num_chars) {
	Uint16 i;

	for (i=0;i<num_chars;i++){
		if (((*c >= 0x30) && (*c<= 0x39))
			|| ((*c >= 0x41) && (*c<= 0x46))
			|| ((*c >= 0x61) && (*c<= 0x66))) {
			c++;
			continue;
		} else {
			return false;
		}
	}
	return true;
}

char *hexUtil_binTo4HexAsciiChars(char *c,Uint16 dataWord){
	// given a 16-bit dataWord, break it into 4 nibbles, convert each nibble to a displayable ascii char 0-9,A-F
	// write the ascii into char*c buffer, advancing the buffer pointer
	// return the advanced buffer pointer char*c
    Uint16 i;

    for (i=0;i<4;i++){
      *c = ((dataWord & 0xF000) >> 12) + 0x0030;
      if (*c > 0x0039) {
    	  *c += 7;
      }
      dataWord <<= 4;
      c++;
    }
    return c;
}

char *hexUtil_binTo2HexAsciiChars(char *c,Uint16 dataWord){
	// given LS 8 bits of data in a 16-bit dataWord, break it into 2 nibbles,
	// convert each nibble to a displayable ascii char 0-9,A-F
	// write the ascii into char*c buffer, advancing the buffer pointer
	// return the advanced buffer pointer char*c
    Uint16 i;
    Uint16 k;

    k = dataWord << 8;
    for (i=0;i<2;i++){
      *c = ((k & 0xF000) >> 12) + 0x0030;
      if (*c > 0x0039) {
    	  *c += 7;
      }
      k <<= 4;
      c++;
    }
    return c;
}

char *hexUtil_binTo5DecAsciiChars(char *c,Uint16 dataWord){
	// given a 16-bit dataWord, convert to 5 decimal digits in displayable ascii char 0-9
	// write the ascii into char*c buffer, advancing the buffer pointer
	// return the advanced buffer pointer char*c
	//
	// -- THIS LOOKS LIKE A POOR ALGORITHM, LET'S CHECK KNUTH AND REPLACE IT
	// -- but after looking for a better replacement, they really don't look
	// -- that much better.  I'll keep looking, but in the mean time I'll
	// -- use this one. -- DH 10/20/2014
    Uint16 i;
    Uint16 j;

    i = dataWord;
    j = i % 10000;
    i = (i - j) / 10000;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 1000;
    i = (i - j) / 1000;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 100;
    i = (i - j) / 100;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 10;
    i = (i - j) / 10;
    *(c++) = i + 0x0030;

    *(c++) = j + 0x0030;

    return c;
}

char *hexUtil_binToDecAsciiCharsZeroSuppress(char *c,Uint16 dataWord){
	// given a 16-bit dataWord, convert to decimal digits in displayable ascii char 0-9
	// write the ascii into char*c buffer, advancing the buffer pointer
	// return the advanced buffer pointer char*c.
	// Suppress up to 4 leading "0"'s.  IE: we never suppress the units place.
	char fiveDecimalDigits[5];
	Uint16 i;
	Uint16 suppress;

	suppress = 1;
	hexUtil_binTo5DecAsciiChars(fiveDecimalDigits, dataWord);
	for (i=0;i<4;i++){
		if((fiveDecimalDigits[i] != '0') | (suppress == 0)){
			suppress = 0;
			*(c++) = fiveDecimalDigits[i];
		}
	}
	*(c++) = fiveDecimalDigits[4];

    return c;
}

char *hexUtil_bin32To10DecAsciiChars(char *c,Uint32 dataWord){
	// given a 32-bit dataWord, convert to 10 decimal digits in displayable ascii char 0-9
	// write the ascii into char*c buffer, advancing the buffer pointer
	// return the advanced buffer pointer char*c
	//
	// MAY BE ABLE TO FIND A BETTER ALGORITHM

    Uint32 i;
    Uint32 j;

    i = dataWord;
    j = i % 1000000000L;
    i = (i - j) / 1000000000L;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 100000000L;
    i = (i - j) / 100000000L;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 10000000L;
    i = (i - j) / 10000000L;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 1000000L;
    i = (i - j) / 1000000L;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 100000L;
    i = (i - j) / 100000L;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 10000L;
    i = (i - j) / 10000L;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 1000L;
    i = (i - j) / 1000L;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 100L;
    i = (i - j) / 100L;
    *(c++) = i + 0x0030;

    i = j;
    j = i % 10l;
    i = (i - j) / 10L;
    *(c++) = i + 0x0030;

    *(c++) = j + 0x0030;

    return c;

}

char *hexUtil_bin32ToDecAsciiCharsZeroSuppress(char *c,Uint32 dataWord){
	// given a 32-bit dataWord, convert to decimal digits in displayable ascii char 0-9
	// write the ascii into char*c buffer, advancing the buffer pointer
	// return the advanced buffer pointer char*c.
	// Suppress up to 9 leading "0"'s.  IE: we never suppress the units place.
	char tenDecimalDigits[10];
	Uint16 i;
	Uint16 suppress;

	suppress = 1;
	hexUtil_bin32To10DecAsciiChars(tenDecimalDigits, dataWord);
	for (i=0;i<9;i++){
		if((tenDecimalDigits[i] != '0') | (suppress == 0)){
			suppress = 0;
			*(c++) = tenDecimalDigits[i];
		}
	}
	*(c++) = tenDecimalDigits[9];

    return c;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   Convert  Ascii Decimal digits to a 16-bit binary value.
//   Stops at first character not a valid decimal digit.
//   Return char pointer to next character after the ones we converted to binary.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
char* hexUtil_decCharsToBin(char *decAscii, Uint16 *result) {
	Uint16 i;

	*result = 0;
	while ((*decAscii >= '0') && (*decAscii <= '9')){
		i = ((*result)<<1) & 0xFFFE; // result x 2
		*result = (((*result) << 3) & 0xFFF8) + i; //result x 10
		*result += *decAscii & 0x000F;
		decAscii++; // increment pointer
	}
    return decAscii;
}

char* hexUtil_decCharsToBin32(char *decAscii, Uint32 *result32) {
	// hexUtil_decCharsToBin modified to give Uint32 result
	Uint32 i;

	*result32 = 0L;
	while ((*decAscii >= '0') && (*decAscii <= '9')){
		i = ((*result32)<<1) & 0xFFFFFFFEL; // result x 2
		*result32 = (((*result32) << 3) & 0xFFFFFFF8L) + i; //result x 10
		*result32 += ((Uint32)(*decAscii)) & 0x0000000FL;
		decAscii++; // increment pointer
	}
    return decAscii;
}


