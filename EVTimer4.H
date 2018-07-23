// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     EVTimer4.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef EVTIMER4x_H
#define EVTIMER4x_H

void evtimer4_store_int_vectors_in_PIE(void);
void evtimer4_initConfig_n_Start(void);
interrupt void evtimer4_isr(void);
void evtimer4_enable_int(void);
void evtimer4_turn_off (void);
void evtimer4_turn_on (Uint16 period, Uint16 prescale);

enum EVTIMRE4_STATE {
	EVT4_ST_IDLE     = 0,
	EVT4_ST_TURN_OFF = 1,
	EVT4_ST_DUMMY1   = 2
};
#endif
