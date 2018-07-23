// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Xintf.H
//
//  Peripheral registers are defined in DSP281x_Xintf.h
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef XINTFx_H
#define XINTFx_H

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessors for Timer0 non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
void xintf_setTestData2001(Uint16 dataWord);
void xintf_setTestData2002(Uint16 dataWord);
void xintf_setTestData2003(Uint16 dataWord);
void xintf_setTestSelection2004(Uint16 dataWord);
Uint16 xintf_getTestSelection2004(void);
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

void xintf_InitXintf(void);
void xintf_initTest(void);
void xintf_testUnderTimer0(void);
void xintf_test2010(Uint16 dataword);
void xintf_test2011(void);

#endif
