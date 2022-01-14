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

#define ADC_0                   (0U)
#define CHANNEL_0               (0U)

///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

// ADC initialisation Function
// Includes conversion of values and printing of the ADC16 values
// Modified from Kinetis SDK v.1.1 API Reference Manual
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
    // Auto calibration. //
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

    // Wait for the most recent conversion.//
  ADC16_DRV_WaitConvDone(instance, chnGroup);

  // Fetch the conversion value and format it. //
  MyAdcValue = ADC16_DRV_GetConvValueRAW(instance, chnGroup);

  // Get value and print the output of the ADC //
  uint16_t WaterlevelValue;
  WaterlevelValue = ADC16_DRV_ConvRAWData(MyAdcValue, false, kAdcResolutionBitOfSingleEndAs12);
  warpPrint("%u, ", WaterlevelValue);

  return WaterlevelValue;
}

// Disable ADC Function
int Disable_ADC(uint32_t instance, uint32_t chnGroup){
  // Pause the conversion after testing. //
  ADC16_DRV_PauseConv(instance, chnGroup);
  // Disable the ADC. //
  ADC16_DRV_Deinit(instance);

  return 0;
}

// Main functionality Function
// Turn screen flashing green if the output of the water level sensor < 200
// Orange if the output is between 200 and 2000
// Red if the output is > 2200
int ADC16_Blocking(void){
  // Initialize ADC and get converted value from the ADC
  uint16_t Value;
  Value  = ADC16_TEST_Blocking(ADC_0, CHANNEL_0, 8U); // 8th channel PTB1 -

  if (Value < 200)
    {devSSD1331Green();}
  else if(Value >= 200 && Value < 2200)
    {devSSD1331Orange();}
  else if(Value >= 2200)
    {devSSD1331Red();}

  return 0;
}
