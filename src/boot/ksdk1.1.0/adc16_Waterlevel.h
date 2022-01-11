// adc16_test.h //
#ifndef __ADC16_Waterlevel_H__
#define __ADC16_Waterlevel_H__

#include <stdio.h>
//#include <stdbool.h>
//#include <stdint.h>
//#include "board.h"
//#include "fsl_os_abstraction.h"
//#include "fsl_debug_console.h"
#include "fsl_adc16_driver.h"

#define ADC16_TEST_CHN_TEMPERATURE (26U) // Temperature Sensor. //
#define ADC16_TEST_CHN_GROUT_NUM (0U)
#define ADC16_TEST_LOOP_CYCLE (4U)

//void ADC16_TEST_InstallCallback(uint32_t instance, void (*callbackFunc)(void) );
//void ADC16_TEST_OneTimeTrigger(uint32_t instance, uint32_t chnGroup, uint8_t chn);
int ADC16_TEST_Blocking(uint32_t instance, uint32_t chnGroup, uint8_t chn);
//void ADC16_TEST_IntMode(uint32_t instance, uint32_t chnGroup, uint8_t chn);
int ADC16_Blocking(void);
/*!
 * @brief ADC Interrupt handler. Get current ADC value and set conversionCompleted flag.
 */
//void ADC0IRQHandler(void);

/*!
 * @brief Calculate current Water level.
 *
 * @return uint32_t Returns current water level.
 */
// int32_t GetCurrentWaterlevelValue(void);

/*!
 * @brief User-defined function to read conversion value in ADC ISR.
 */
// uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup);


#endif // __ADC16_Waterlevel_H__ //
