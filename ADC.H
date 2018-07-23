// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     ADC.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef ADCx_H
#define ADCx_H

void adc_init(void);
void adc_configureChannels(void);
void adc_readOneAdcChannelToRs232(Uint16 adcChannelNum);
void adc_addOneAdcChannelToList(Uint16 adcChannelNum);
void adc_DisplayAdcResults(void);
void adc_repeatPrevAdcTest(void);
void adc_readAll16Channels(void);
#endif
