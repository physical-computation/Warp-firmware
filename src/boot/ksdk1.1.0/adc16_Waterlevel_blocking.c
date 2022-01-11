// adc16_Waterlevel_blocking.c //

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "config.h"
#include "adc16_Waterlevel.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"
///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

//#define ADC_0                   (0U)
//#define CHANNEL_0               (0U)
//#define CHANNEL_1               (1U)

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////

//static uint32_t adcValue = 0;               /*! ADC value */
//volatile bool conversionCompleted = false;  /*! Conversion is completed Flag */
//const uint32_t gSimBaseAddr[] = SIM_BASE_ADDRS;
//static smc_power_mode_config_t smcConfig;

///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

/* ADC Interrupt Handler
void ADC0IRQHandler(void)
{
    // Get current ADC value
    adcValue = ADC_TEST_GetConvValueRAWInt (ADC_0, CHANNEL_0);
    // Set conversionCompleted flag. This prevents an wrong conversion in main function
    conversionCompleted = true;
}
*/
int ADC16_TEST_Blocking(uint32_t instance, uint32_t chnGroup, uint8_t chn)
{
  #if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t MyAdcCalibraitionParam;
  #endif // FSL_FEATURE_ADC16_HAS_CALIBRATION //
    adc16_user_config_t MyAdcUserConfig;
    adc16_chn_config_t MyChnConfig;
    uint16_t MyAdcValue;
    //uint32_t i;

  #if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibraion. //
    ADC16_DRV_GetAutoCalibrationParam(instance, &MyAdcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(instance, &MyAdcCalibraitionParam);
  #endif // FSL_FEATURE_ADC16_HAS_CALIBRATION //

  // Initialize the ADC converter. //
  ADC16_DRV_StructInitUserConfigDefault(&MyAdcUserConfig);
  MyAdcUserConfig.continuousConvEnable = true; // Enable continuous conversion. //
  ADC16_DRV_Init(instance, &MyAdcUserConfig);

  // Configure the ADC channel and take an initial trigger. //
  MyChnConfig.chnNum = chn;
  MyChnConfig.diffEnable= false;
  MyChnConfig.intEnable = false;
  MyChnConfig.chnMux = kAdcChnMuxOfA;
  ADC16_DRV_ConfigConvChn(instance, chnGroup, &MyChnConfig);
  //for (i = 0U; i < 4U; i++)
  //{
    // Wait for the most recent conversion.//
  ADC16_DRV_WaitConvDone(instance, chnGroup);

  //warpPrint("\r\tADC16_Blocking After ConvDone\n");
  // Fetch the conversion value and format it. //
  MyAdcValue = ADC16_DRV_GetConvValueRAW(instance, chnGroup);

  //warpPrint("\r\tADC16_Blocking After ConvValueRaw [%d] \n", MyAdcValue);

  //printf("ADC16_DRV_GetConvValueRAW: 0x%X\t", MyAdcValue);
  //printf("ADC16_DRV_ConvRAWData: %ld\r\n", ADC16_DRV_ConvRAWData(MyAdcValue, false,kAdcResolutionBitOfSingleEndAs12) );

  uint16_t WaterlevelValue; // 32?
  WaterlevelValue = ADC16_DRV_ConvRAWData(MyAdcValue, false, kAdcResolutionBitOfSingleEndAs12);

  warpPrint("%u, ", WaterlevelValue);

  //OSA_TimeDelay(20);
  return WaterlevelValue;
  //}
  warpPrint("\r\tADC16_Blocking Before PauseConv\n");
  // Pause the conversion after testing. //
  ADC16_DRV_PauseConv(instance, chnGroup);
  // Disable the ADC. //
  ADC16_DRV_Deinit(instance);
}

/*!
 * @brief main function
 */
int ADC16_Blocking(void){

//warpPrint("\r\tADC16_Blocking Start...\n");
  //uint16_t WaterlevelValue = 1;

  //uint32_t updateBoundariesCounter = 0;
  //int32_t tempArray[UPDATE_BOUNDARIES_TIME * 2];
  //lowPowerAdcBoundaries_t boundaries;

  // Init hardware
  //hardware_init();

  // Call this function to initialize the console UART.  This function
  // enables the use of STDIO functions (printf, scanf, etc.)
  //dbg_uart_init();

  // Initialize GPIO pins
  //GPIO_DRV_Init(accelIntPins, ledPins);
  //ADCPin = GPIO_MAKE_PIN(HW_GPIOB, 1);


  // PORT_HAL_SetMuxMode(PORTB_BASE, 1, 0U);
  // or PORT_HAL_SetMuxMode(PORTB_BASE, 1u, kPortMuxAsGpio);


  // Initialize ADC
  uint16_t Value;
  Value  = ADC16_TEST_Blocking(0U, 0U, 8U);  // 0 all?
  //warpPrint("\r\n\tADC16_Blocking After ADC...\n");
  /*if (Value < 1500)
    {devSSD1331Green();}
  else if(Value >= 1500 && Value < 1800)
    {devSSD1331Orange();}
  else if(Value >= 1800)
    {devSSD1331Red();}
    */
  OSA_TimeDelay(10);
  return Value;
}
