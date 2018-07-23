// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     McsParse.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef MCSPARSEx_H
#define MCSPARSEx_H

enum MCS_PARSE_ERR {
	MCS_PARSE_NO_ERR =              0,
	MCS_PARSE_RECEIVED_SOME_DATA =  1,
	MCS_PARSE_RECEIVED_ADDR_EXTEN = 2,
	MCS_PARSE_RECEIVED_EOF =        3,
	MCS_PARSE_BOGUS_FIRST_CHAR =    4,
	MCS_PARSE_NON_HEX_CHAR =        5,
	MCS_PARSE_BAD_CHAR_COUNT =      6,
	MCS_PARSE_BAD_CHECK_SUM =       7,
	MCS_PARSE_BAD_REC_TYPE =        8,
	MCS_PARSE_BAD_REC =             9
};

Uint16  mcsParseReceivedData(char *mcsDataIn,Uint16 countCharIn,
		                     Uint16 *mcsByteCount,Uint16 *mcsRecType,Uint16 *mcsAddr,
		                     Uint16 *numWordsFromMcs,Uint16 *wordsFromMcs);

#endif /* MCSPARSEx_H */
