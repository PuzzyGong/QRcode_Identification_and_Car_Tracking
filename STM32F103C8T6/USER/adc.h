#ifndef _ADC_H_
#define _ADC_H_

#include "stm32f10x.h"

void adc_init(void);
void adc_getdata(uint16_t* u16_value);

#endif
