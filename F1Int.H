// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     F1Int.H
//         support for FPGA1 to interrupt DSP
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef F1INTx_H
#define F1INTx_H

void f1i_enable_interrupt (void);
void f1i_store_int_vectors_in_PIE(void);
void f1i_initialize_interrupt (void);
interrupt void f1i_isr(void);
void f1i_BgTask(void);

enum F1I_STATE {
	F1I_ST_IDLE     = 0,
	F1I_ST_DUMMY1 	= 1,
	F1I_ST_DUMMY2   = 2
};


#endif
