// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     GpioUtil.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef GPIOUTILx_H
#define GPIOUTILx_H

// Redefined GPIO Pin assignments moving from TB3CMA to B
//#define TB3CMA_GPIO
#define TB3CMB_GPIO

void GpioU_defaultInit(void);
void GpioU_rs232Init(void);
void GpioU_sci2Init(void);
void GpioU_SpiInit(void);
// void GpioU_SpiDisableEEPromOnTB3CM(void);
void GpioU_timer0Init(void);
void GpioU_initGpiosAllZero(void);
void GpioU_ioTest2100(Uint16);
void GpioU_ioTest2101(Uint16);
void GpioU_ioTest2102(Uint16);
void GpioU_i2cee_Init(void);
void GpioU_f1iInit(void);
void GpioU_f2iInit(void);

#endif
