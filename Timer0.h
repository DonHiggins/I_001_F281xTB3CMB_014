// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Timer0.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef TIMER0x_H
#define TIMER0x_H

interrupt void timer0_isr(void);
void timer0_store_int_vectors_in_PIE(void);
void timer0_initConfig_n_Start(void);
void timer0_init_03(void);
void timer0_task(void);
void timer0_tenthOfSecTask(void);
void timer0_task_init();
void timer0_miliSecTask(void);
Uint32 timer0_interrupt_count_value();
Uint32 timer0_count_reg_value();
Uint32 timer0_fetchSystemMiliSecCount(void);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        P E R I O D   F O R   T I M E R   0

#define TIMER_0_PERIOD_IN_USEC 200
#define T_0_COUNTS_TO_1_MILISEC 5
#define T_0_COUNTS_TO_1_TENTH_SEC 500
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



#endif
