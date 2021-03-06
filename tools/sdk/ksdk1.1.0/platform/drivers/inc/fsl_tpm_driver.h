/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __FSL_TPM_DRIVER_H__
#define __FSL_TPM_DRIVER_H__

#include "fsl_tpm_hal.h"
#include "fsl_interrupt_manager.h"

/*!
 * @addtogroup tpm_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Table of base addresses for TPM instances. */
extern const uint32_t g_tpmBaseAddr[];

/*! @brief Table to save TPM IRQ numbers for TPM instances. */
extern const IRQn_Type g_tpmIrqId[];

/*! @brief TPM clock source selection.*/
typedef enum _tpm_clock_source
{
    kTpmClockSourcNone = 0,          /*!< TPM clock source, None */
    kTpmClockSourceModuleHighFreq,   /*!< TPM clock source, IRC48MHz or FLL/PLL depending on SoC */
    kTpmClockSourceModuleOSCERCLK,   /*!< TPM clock source, OSCERCLK */
    kTpmClockSourceModuleMCGIRCLK,   /*!< TPM clock source, MCGIRCLK */
    kTpmClockSourceExternalCLKIN0,   /*!< TPM clock source, TPM_CLKIN0 */
    kTpmClockSourceExternalCLKIN1,   /*!< TPM clock source, TPM_CLKIN1 */
    kTpmClockSourceReserved          /*!< TPM clock source, Reserved */
}tpm_clock_source_t;

/*! @brief Internal driver state information grouped by naming. User  needs to set the relevant ones.*/
typedef struct TpmGeneralConfig {
    bool isDBGMode;          /*!< DBGMode behavioral, false to pause, true to continue run in DBG mode */
    bool isGlobalTimeBase;   /*!< If Global time base enabled, true to enable, false to disable */
    bool isTriggerMode;      /*!< If Trigger mode enabled, true to enable, false to disable*/
    bool isStopCountOnOveflow; /*!< True to stop counter after overflow, false to continue running */
    bool isCountReloadOnTrig;  /*!< True to reload counter on trigger, false means counter is not reloaded */
    tpm_trigger_source_t triggerSource; /*!< Trigger source if trigger mode enabled*/
}tpm_general_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the TPM driver.
 *
 * @param instance The TPM peripheral instance number.
 * @param info The TPM peripheral instance info
 */
void TPM_DRV_Init(uint8_t instance, tpm_general_config_t * info);

/*!
 * @brief Stops the channel PWM.
 *
 * @param instance The TPM peripheral instance number.
 * @param param PWM parameter to configure PWM options
 * @param channel The channel number.
 */
void TPM_DRV_PwmStop(uint8_t instance, tpm_pwm_param_t *param, uint8_t channel);

/*!
 * @brief Configures duty cycle, frequency, and starts outputting PWM on a specified channel.
 *
 * @param instance The TPM peripheral instance number.
 * @param param PWM parameter to configure PWM options
 * @param channel The channel number.
 *
 * @return true: if successful, false: failure to generate the PWM signal
 */
bool TPM_DRV_PwmStart(uint8_t instance, tpm_pwm_param_t *param, uint8_t channel);

/*!
 * @brief Enables or disables the timer overflow interrupt.
 *
 * @param instance The TPM peripheral instance number.
 * @param overflowEnable true: enable the timer overflow interrupt, false: disable
 */
void TPM_DRV_SetTimeOverflowIntCmd(uint32_t instance, bool overflowEnable);

/*!
 * @brief Enables or disables the channel interrupt.
 *
 * @param instance The TPM peripheral instance number.
 * @param channelNum The channel number.
 * @param enable true: enable the channel interrupt, false: disable
 */
void TPM_DRV_SetChnIntCmd(uint32_t instance, uint8_t channelNum, bool enable);

/*!
 * @brief Sets the TPM clock source.
 *
 * @param instance The TPM peripheral instance number.
 * @param clock The TPM peripheral clock selection. Options are listed in tpm_clock_source_t
 * @param clockPs The TPM peripheral clock prescale factor selection listed in tpm_clock_ps_t
 */
void TPM_DRV_SetClock(uint8_t instance, tpm_clock_source_t clock, tpm_clock_ps_t clockPs);

/*!
 * @brief Gets the TPM clock frequency.
 *
 * @param instance The TPM peripheral instance number.
 * @return The function returns the frequency of the TPM clock.
 */
uint32_t TPM_DRV_GetClock(uint8_t instance);

/*!
 * @brief Starts the TPM counter.
 *
 * This function provides access to the TPM counter. The counter can be run in
 * Up-counting and Up-down counting modes.
 *
 * @param instance The TPM peripheral instance number.
 * @param countMode The TPM counter mode defined by tpm_counting_mode_t.
 * @param countFinalVal The final value that is stored in the MOD register.
 * @param enableOverflowInt true: enable timer overflow interrupt; false: disable
 */
void TPM_DRV_CounterStart(uint8_t instance, tpm_counting_mode_t countMode, uint32_t countFinalVal,
                                 bool enableOverflowInt);

/*!
 * @brief Stops the TPM counter.
 *
 * @param instance The TPM peripheral instance number.
 */
void TPM_DRV_CounterStop(uint8_t instance);

/*!
 * @brief Reads back the current value of the TPM counter.
 *
 * @param instance The TPM peripheral instance number.
 */
uint32_t TPM_DRV_CounterRead(uint8_t instance);

/*!
 * @brief TPM input capture mode setup.
 *
 * @param instance The TPM peripheral instance number.
 * @param channel The channel number.
 * @param mode The TPM input mode defined by tpm_input_capture_mode_t.
 * @param countFinalVal The final value that is stored in the MOD register.
 * @param intEnable true: enable channel interrupt; false: disable
 */
void TPM_DRV_InputCaptureEnable(uint8_t instance, uint8_t channel, tpm_input_capture_mode_t mode,
                                         uint32_t countFinalVal, bool intEnable);

/*!
 * @brief Reads back the current value of the TPM channel value.
 *
 * @param instance The TPM peripheral instance number.
 * @param channel The channel number.
 */
uint32_t TPM_DRV_GetChnVal(uint8_t instance, uint8_t channel);

/*!
 * @brief TPM output compare mode setup.
 *
 * @param instance The TPM peripheral instance number.
 * @param channel The channel number.
 * @param mode The TPM output mode defined by tpm_output_compare_mode_t.
 * @param countFinalVal The final value that is stored in the MOD register.
 * @param matchVal The channel compare value stored in the CnV register
 * @param intEnable true: enable channel interrupt; false: disable
 */
void TPM_DRV_OutputCompareEnable(uint8_t instance, uint8_t channel, tpm_output_compare_mode_t mode,
                                            uint32_t countFinalVal, uint32_t matchVal, bool intEnable);

/*!
 * @brief Shuts down the TPM driver.
 *
 * @param instance The TPM peripheral instance number.
 */
void TPM_DRV_Deinit(uint8_t instance);

/*!
 * @brief Action to take when an TPM interrupt is triggered.
 *
 * The timer overflow flag is checked and cleared if set.
 *
 * @param instance   Instance number of the TPM module.
 */
void TPM_DRV_IRQHandler(uint8_t instance);

/*Other API functions are for input capture, output compare, dual edge capture, and quadrature. */
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_TPM_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

