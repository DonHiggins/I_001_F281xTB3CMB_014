// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     McsParse.C
//
//   Utility routines to parse data from MCS format Files
//       for example, the MCS file of FPGA program data, loaded into the Flash
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"
#include "McsParse.H"


Uint16  mcsParseReceivedData(char *mcsDataIn,Uint16 countCharIn,
		                     Uint16 *mcsByteCount,Uint16 *mcsRecType,Uint16 *mcsAddr,
		                     Uint16 *numWordsFromMcs,Uint16 *wordsFromMcs)
{
	char *mcsData;
	Uint16 binFromMcs;
	Uint16 i;
	Uint16 checkSumAccumulator;

    // ids the first char a ":"
	if (*mcsDataIn != ':') {
	   return MCS_PARSE_BOGUS_FIRST_CHAR;
   }

	// are the remaining characters all hex/ascii digits?
   if ((hexUtil_isValidAsciiHexDigits((mcsDataIn+1),(countCharIn -1)))!= true) {
	   return MCS_PARSE_NON_HEX_CHAR;
   }

   // did we receive too many or too few characters?
   if ((countCharIn < 11) || (countCharIn > 45)) {
	   return MCS_PARSE_BAD_CHAR_COUNT;
   }

   // In the MCS data, first 2 hex chars are byte-count
   mcsData = hexUtil_hexCharsToBin((mcsDataIn+1), 2, mcsByteCount) ;

   // In the MCS data, next 4 hex chars are address
   mcsData = hexUtil_hexCharsToBin(mcsData, 4, mcsAddr) ;

   // In the MCS data, next 2 hex chars are record type
   mcsData = hexUtil_hexCharsToBin(mcsData, 2, mcsRecType) ;

   // Does the MCS character count agree with the number of characters we have received?
   // the 2-character window should allow the inclusion of CrLf, or not
   if ((countCharIn < (11 + ((*mcsByteCount)<<1)))
    || (countCharIn > (13 + ((*mcsByteCount)<<1)))) {
	   return MCS_PARSE_BAD_CHAR_COUNT;
   }

   //factor mcsByteCount, mcsAddr, and mcsRecType into check sum accumulator
   checkSumAccumulator = *mcsByteCount
		               + (*mcsAddr & 0xFF)
		               + ((*mcsAddr >> 8) & 0xFF)
		               + *mcsRecType;

   // Now read data bytes, and pack into words
   *numWordsFromMcs = 0;
   if (*mcsByteCount > 0) {

	   for (i=0;i<*mcsByteCount;i++) {
		   mcsData = hexUtil_hexCharsToBin(mcsData, 2, &binFromMcs);
		   checkSumAccumulator += binFromMcs;
		   // pack 2 bytes into each word
		   if ((i & 1) == 0) {
			   *wordsFromMcs = binFromMcs<<8;
		   } else {
			   *(wordsFromMcs++) |= binFromMcs;
			   (*numWordsFromMcs)++;
		   }
	   }
   }

   //next two characters in mcsData are check sum, see if it adds up
   mcsData = hexUtil_hexCharsToBin(mcsData, 2, &binFromMcs);
   checkSumAccumulator += binFromMcs;
   if ((checkSumAccumulator & 0xFF) != 0) {
	   return MCS_PARSE_BAD_CHECK_SUM;
   }

   if ((*mcsRecType == 0) && (*mcsByteCount > 0)){
      return MCS_PARSE_RECEIVED_SOME_DATA;
   } else if ((*mcsRecType == 4) && (*mcsByteCount == 2)){
      return MCS_PARSE_RECEIVED_ADDR_EXTEN;
   } else if ((*mcsRecType == 1) && (*mcsByteCount == 0)){
      return MCS_PARSE_RECEIVED_EOF;
   } else  if ((*mcsRecType != 0)&& (*mcsRecType != 1)&& (*mcsRecType != 4)){
	  return MCS_PARSE_BAD_REC_TYPE;
   }

   // Ran into something in the data that we weren't anticipating
   return MCS_PARSE_BAD_REC;
}
