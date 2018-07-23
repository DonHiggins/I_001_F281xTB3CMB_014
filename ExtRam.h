// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     ExtRam.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef ExtRamx_H
#define ExtRamx_H


void xram_testExternalRam(Uint16 dataWord);
void xram_ExtRamRWTest();
bool xram_RWPassThruWholeChip(Uint16 seed);

union EXTRAM16_32 {
	Uint32 		all;
	struct TWO_WORDS_ {
		Uint16 lsw;
		Uint16 msw;
	} words;
};

#endif
