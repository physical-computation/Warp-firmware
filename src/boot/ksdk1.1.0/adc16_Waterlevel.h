// adc16_Wterlevel.h //
#ifndef __ADC16_Waterlevel_H__
#define __ADC16_Waterlevel_H__

#include <stdio.h>
#include "fsl_adc16_driver.h"

int ADC16_TEST_Blocking(uint32_t instance, uint32_t chnGroup, uint8_t chn);
int ADC16_Blocking(void);

#endif // __ADC16_Waterlevel_h__ //
