/*
* Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#if !defined(__FSL_SIM_HAL_KL03Z4_H__)
#define __FSL_SIM_HAL_KL03Z4_H__

/*!
 * @addtogroup sim_hal_kl03z4
 * @{
 */
/*! @file*/

/*******************************************************************************
* Definitions
******************************************************************************/
/*! @brief COP clock source selection.*/
typedef enum _clock_cop_src {
    kClockCopSrcLpoClk,							 /*!< LPO clock,1K HZ.*/
    kClockCopSrcMcgIrClk, 						 /*!< MCG IRC Clock   */
    kClockCopSrcOsc0erClk, 						 /*!< OSCER Clock	  */
    kClockCopSrcBusClk							 /*!< BUS clock 	  */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
}clock_cop_src_kl03z4_t;
#else
}clock_cop_src_t;
#endif

/*! @brief SIM external reference clock source select (OSC32KSEL). */
typedef enum _clock_er32k_src
{
    kClockEr32kSrcOsc0 = 0U, /*!< OSC0 clock (OSC032KCLK).  */
    kClockEr32kSrcRtc  = 2U, /*!< RTC 32k clock .           */
    kClockEr32kSrcLpo  = 3U, /*!< LPO clock.                */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_er32k_src_kl03z4_t;
#else
} clock_er32k_src_t;
#endif

/*! @brief SIM external reference clock output pin select (OSC32KOUT). */
typedef enum _clock_osc32kout_sel
{
    kClockOsc32koutNone,   /*!< ERCLK32K is not output. */
    kClockOsc32koutPtb13,  /*!< ERCLK32K output on PTB13 */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_osc32kout_sel_kl03z4_t;
#else
} clock_osc32kout_sel_t;
#endif


/*! @brief SIM LPUART0 clock source */
typedef enum _clock_lpuart_src
{
    kClockLpuartSrcNone,          /*!< disabled */
    kClockLpuartSrcIrc48M,	      /*!< IRC48M */
    kClockLpuartSrcOsc0erClk,	  /*!< OSCER clock */
    kClockLpuartSrcMcgIrClk 	  /*!< MCGIR clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_lpuart_src_kl03z4_t;
#else
} clock_lpuart_src_t;
#endif

/*! @brief SIM TPM clock source */
typedef enum _clock_tpm_src
{
    kClockTpmSrcNone,   /*!< disabled */
    kClockTpmSrcIrc48M,	  /*!< IRC48M */
    kClockTpmSrcOsc0erClk,	  /*!< OSCER clock */
    kClockTpmSrcMcgIrClk 	  /*!< MCGIR clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_tpm_src_kl03z4_t;
#else
} clock_tpm_src_t;
#endif

/*! @brief LPTMR clock source select */
typedef enum _clock_lptmr_src_t
{
    kClockLptmrSrcMcgIrClk,        /*!< MCG out clock  */
    kClockLptmrSrcLpoClk,          /*!< LPO clock      */
    kClockLptmrSrcEr32kClk,        /*!< ERCLK32K clock */
    kClockLptmrSrcOsc0erClk,       /*!< OSCERCLK clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_lptmr_src_kl03z4_t;
#else
} clock_lptmr_src_t;
#endif

/*! @brief SIM CLKOUT_SEL clock source select */
typedef enum _clock_clkout_src
{
    kClockClkoutSelFlashClk  = 2U,  /*!< Flash clock    */
    kClockClkoutSelLpoClk    = 3U,  /*!< LPO clock      */
    kClockClkoutSelMcgIrClk  = 4U,  /*!< MCG out clock  */
    kClockClkoutSelOsc0erClk = 6U,  /*!< OSCER clock */
    kClockClkoutSelIrc48M    = 7U   /*!< IRC48M clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_clkout_src_kl03z4_t;
#else
} clock_clkout_src_t;
#endif

/*! @brief SIM RTCCLKOUTSEL clock source select */
typedef enum _clock_rtcout_src
{
    kClockRtcoutSrc1Hz,        /*!< 1Hz clock      */
    kClockRtcoutSrcOsc0erClk,  /*!< OSCER clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_rtcout_src_kl03z4_t;
#else
} clock_rtcout_src_t;
#endif

/*! @brief SIM ADCx pre-trigger select */
typedef enum _sim_adc_pretrg_sel
{
    kSimAdcPretrgselA,              /*!< Pre-trigger A selected for ADCx */
    kSimAdcPretrgselB               /*!< Pre-trigger B selected for ADCx */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_adc_pretrg_sel_kl03z4_t;
#else
} sim_adc_pretrg_sel_t;
#endif

/*! @brief SIM ADCx trigger select */
typedef enum _sim_adc_trg_sel
{
    kSimAdcTrgselExt	= 0U,               /*!< External trigger                */
    kSimAdcTrgSelHighSpeedComp0 	= 1U,    /*!< High speed comparator 0 output  */
    kSimAdcTrgSelTpm0		= 8U,              /*!< TPM0 trigger                    */
    kSimAdcTrgSelTpm1		= 9U,              /*!< TPM1 trigger                    */
    kSimAdcTrgSelRtcAlarm	= 12U,          /*!< RTC alarm                       */
    kSimAdcTrgSelRtcSec		= 13U,            /*!< RTC seconds                     */
    kSimAdcTrgSelLptimer	= 14U,           /*!< Low-power timer trigger         */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_adc_trg_sel_kl03z4_t;
#else
} sim_adc_trg_sel_t;
#endif

/*! @brief SIM receive data source select */
typedef enum _sim_uart_rxsrc
{
    kSimUartRxsrcPin,               /*!< UARTx_RX Pin  */
    kSimUartRxsrcCmp0,              /*!< CMP0          */
    kSimUartRxsrcCmp1,              /*!< CMP1          */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_uart_rxsrc_kl03z4_t;
#else
} sim_uart_rxsrc_t;
#endif

/*! @brief SIM transmit data source select */
typedef enum _sim_uart_txsrc
{
    kSimUartTxsrcPin,    /*!< UARTx_TX Pin */
    kSimUartTxsrcFtm1,   /*!< UARTx_TX pin modulated with FTM1 channel 0 output */
    kSimUartTxsrcFtm2,   /*!< UARTx_TX pin modulated with FTM2 channel 0 output */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_uart_txsrc_kl03z4_t;
#else
} sim_uart_txsrc_t;
#endif

/*! @brief SIM Timer/PWM external clock select */
typedef enum _sim_tpm_clk_sel
{
    kSimTpmClkSel0,                 /*!< Timer/PWM TPM_CLKIN0 pin. */
    kSimTpmClkSel1                  /*!< Timer/PWM TPM_CLKIN1 pin. */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_tpm_clk_sel_kl03z4_t;
#else
} sim_tpm_clk_sel_t;
#endif

/*! @brief SIM Timer/PWM x channel y input capture source select */
typedef enum _sim_tpm_ch_src
{
    kSimTpmChSrc0,                 /*!< TPMx_CH0 signal */
    kSimTpmChSrc1                  /*!< CMP0 output */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_tpm_ch_src_kl03z4_t;
#else
} sim_tpm_ch_src_t;
#endif

/*! @brief SIM PTD7 pad drive strength */
typedef enum _sim_ptd7pad_strengh
{
    kSimPtd7padSinglePad,           /*!< Single-pad drive strength for PTD7 */
    kSimPtd7padDualPad              /*!< Dual-pad drive strength for PTD7 */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_ptd7pad_strengh_kl03z4_t;
#else
} sim_ptd7pad_strengh_t;
#endif

/*! @brief SIM SCGC bit index. */
#define FSL_SIM_SCGC_BIT(SCGCx, n) (((SCGCx-1U)<<5U) + n)

/*! @brief Clock gate name used for SIM_HAL_EnableClock/SIM_HAL_DisableClock. */
typedef enum _sim_clock_gate_name
{
    kSimClockGateI2c0      = FSL_SIM_SCGC_BIT(4U, 6U),
    kSimClockGateCmp0      = FSL_SIM_SCGC_BIT(4U, 19U),
    kSimClockGateVref0     = FSL_SIM_SCGC_BIT(4U, 20U),
    kSimClockGateSpi0      = FSL_SIM_SCGC_BIT(4U, 22U),
    kSimClockGateLptmr0    = FSL_SIM_SCGC_BIT(5U, 0U),
    kSimClockGatePortA     = FSL_SIM_SCGC_BIT(5U, 9U),
    kSimClockGatePortB     = FSL_SIM_SCGC_BIT(5U, 10U),
    kSimClockGateLpUart0   = FSL_SIM_SCGC_BIT(5U, 20U),
    kSimClockGateFtf0      = FSL_SIM_SCGC_BIT(6U, 0U),
    kSimClockGateTpm0      = FSL_SIM_SCGC_BIT(6U, 24U),
    kSimClockGateTpm1      = FSL_SIM_SCGC_BIT(6U, 25U),
    kSimClockGateAdc0      = FSL_SIM_SCGC_BIT(6U, 27U),
    kSimClockGateRtc0      = FSL_SIM_SCGC_BIT(6U, 29U),
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_clock_gate_name_kl03z4_t;
#else
} sim_clock_gate_name_t;
#endif

/*@}*/

/*******************************************************************************
* API
******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @addtogroup sim_hal
 * @{
 */

/*!
 * @brief Enable the clock for specific module.
 *
 * This function enables the clock for specific module.
 *
 * @param baseAddr Base address for current SIM instance.
 * @param name Name of the module to enable.
 */
static inline void SIM_HAL_EnableClock(uint32_t baseAddr, sim_clock_gate_name_t name)
{
    BW_SIM_SCGC_BIT(baseAddr, name, 1U);
}

/*!
* @brief Disable the clock for specific module.
*
* This function disables the clock for specific module.
*
* @param baseAddr Base address for current SIM instance.
* @param name Name of the module to disable.
*/
static inline void SIM_HAL_DisableClock(uint32_t baseAddr, sim_clock_gate_name_t name)
{
    BW_SIM_SCGC_BIT(baseAddr, name, 0U);
}

/*!
* @brief Get the the clock gate state for specific module.
*
* This function will get the clock gate state for specific module.
*
* @param baseAddr Base address for current SIM instance.
* @param name Name of the module to get.
* @return state true - ungated(Enabled), false - gated (Disabled)
*/
static inline bool SIM_HAL_GetGateCmd(uint32_t baseAddr, sim_clock_gate_name_t name)
{
    return (bool)BR_SIM_SCGC_BIT(baseAddr, name);
}

/*!
* @brief Set the clock selection of LPUART0.
*
* This function sets the clock selection of LPUART0.
*
* @param baseAddr Base address for current SIM instance.
* @param instance LPUART instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetLpuartSrc(uint32_t baseAddr,
                                          uint32_t instance,
                                           clock_lpuart_src_t setting)
{
    BW_SIM_SOPT2_LPUART0SRC(baseAddr, setting);
}

/*!
* @brief Get the clock selection of LPUART.
*
* This function gets the clock selection of LPUART.
*
* @param baseAddr Base address for current SIM instance.
* @param instance LPUART instance.
* @return Current selection.
*/
static inline clock_lpuart_src_t CLOCK_HAL_GetLpuartSrc(uint32_t baseAddr,
                                                         uint32_t instance)
{
    return (clock_lpuart_src_t)BR_SIM_SOPT2_LPUART0SRC(baseAddr);
}

/*!
* @brief Set the clock selection of TPM.
*
* This function sets the clock selection of TPM.
*
* @param baseAddr Base address for current SIM instance.
* @param instance IP instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetTpmSrc(uint32_t baseAddr,
                                       uint32_t instance,
                                       clock_tpm_src_t setting)
{
    BW_SIM_SOPT2_TPMSRC(baseAddr, setting);
}

/*!
* @brief Get the clock selection of TPM.
*
* This function gets the clock selection of TPM.
*
* @param baseAddr Base address for current SIM instance.
* @param instance IP instance.
* @return Current selection.
*/
static inline clock_tpm_src_t CLOCK_HAL_GetTpmSrc(uint32_t baseAddr,
                                                  uint32_t instance)
{
    return (clock_tpm_src_t)BR_SIM_SOPT2_TPMSRC(baseAddr);
}

/*!
* @brief Set the clock ERCLK32K output on selected pin.
*
* This function sets ERCLK32K output on selected pin.
*
* @param baseAddr Base address for current SIM instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetOsc32kOutSel(uint32_t baseAddr,
                                             clock_osc32kout_sel_t setting)
{
    BW_SIM_SOPT1_OSC32KOUT(baseAddr, setting);
}

/*!
* @brief Get output status of ERCLK32K.
*
* This function gets the output status of ERCLK32K.
*
* @param baseAddr Base address for current SIM instance.
* @return Current selection.
*/
static inline clock_osc32kout_sel_t CLOCK_HAL_GetOsc32kOutSel(uint32_t baseAddr)
{
    return (clock_osc32kout_sel_t)BR_SIM_SOPT1_OSC32KOUT(baseAddr);
}


/*!
* @brief Set the clock selection of ERCLK32K.
*
* This function sets the clock selection of ERCLK32K.
*
* @param baseAddr Base address for current SIM instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetExternalRefClock32kSrc(uint32_t baseAddr,
                                                       clock_er32k_src_t setting)
{
    BW_SIM_SOPT1_OSC32KSEL(baseAddr, setting);
}

/*!
* @brief Get the clock selection of ERCLK32K.
*
* This function gets the clock selection of ERCLK32K.
*
* @param baseAddr Base address for current SIM instance.
* @return Current selection.
*/
static inline clock_er32k_src_t CLOCK_HAL_GetExternalRefClock32kSrc(uint32_t baseAddr)
{
    return (clock_er32k_src_t)BR_SIM_SOPT1_OSC32KSEL(baseAddr);
}

/*!
* @brief Set CLKOUTSEL selection.
*
* This function sets the selection of the clock to output on the CLKOUT pin.
*
* @param baseAddr Base address for current SIM instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetClkOutSel(uint32_t baseAddr, clock_clkout_src_t setting)
{
    BW_SIM_SOPT2_CLKOUTSEL(baseAddr, setting);
}

/*!
* @brief Get CLKOUTSEL selection.
*
* This function gets the selection of the clock to output on the CLKOUT pin.
*
* @param baseAddr Base address for current SIM instance.
* @return Current selection.
*/
static inline clock_clkout_src_t CLOCK_HAL_GetClkOutSel(uint32_t baseAddr)
{
    return (clock_clkout_src_t)BR_SIM_SOPT2_CLKOUTSEL(baseAddr);
}

/*!
* @brief Set RTCCLKOUTSEL selection.
*
* This function sets the selection of the clock to output on the RTC_CLKOUT pin.
*
* @param baseAddr Base address for current SIM instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetRtcClkOutSel(uint32_t baseAddr,
                                             clock_rtcout_src_t setting)
{
    BW_SIM_SOPT2_RTCCLKOUTSEL(baseAddr, setting);
}

/*!
* @brief Get RTCCLKOUTSEL selection.
*
* This function gets the selection of the clock to output on the RTC_CLKOUT pin.
*
* @param baseAddr Base address for current SIM instance.
* @return Current selection.
*/
static inline clock_rtcout_src_t CLOCK_HAL_GetRtcClkOutSel(uint32_t baseAddr)
{
    return (clock_rtcout_src_t)BR_SIM_SOPT2_RTCCLKOUTSEL(baseAddr);
}


/*!
* @brief Set the clock selection of COP.
*
* This function sets the clock selection of COP.
*
* @param baseAddr Base address for current SIM instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetCopSrc(uint32_t baseAddr,
                                       clock_cop_src_t setting)
{
    BW_SIM_COPC_COPCLKSEL(baseAddr, setting);
}

/*!
* @brief Get the clock selection of COP.
*
* This function gets the clock selection of COP.
*
* @param baseAddr Base address for current SIM instance.
* @return Current selection.
*/
static inline clock_cop_src_t CLOCK_HAL_GetCopSrc(uint32_t baseAddr)
{
    return (clock_cop_src_t)BR_SIM_COPC_COPCLKSEL(baseAddr);
}


/*!
* @brief Set OUTDIV1.
*
* This function sets divide value OUTDIV1.
*
* @param baseAddr Base address for current SIM instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetOutDiv1(uint32_t baseAddr, uint8_t setting)
{
    BW_SIM_CLKDIV1_OUTDIV1(baseAddr, setting);
}

/*!
* @brief Get OUTDIV1.
*
* This function gets divide value OUTDIV1.
*
* @param baseAddr Base address for current SIM instance.
* @return Current divide value.
*/
static inline uint8_t CLOCK_HAL_GetOutDiv1(uint32_t baseAddr)
{
    return BR_SIM_CLKDIV1_OUTDIV1(baseAddr);
}

/*!
* @brief Set OUTDIV4.
*
* This function sets divide value OUTDIV4.
*
* @param baseAddr Base address for current SIM instance.
* @param setting  The value to set.
*/
static inline void CLOCK_HAL_SetOutDiv4(uint32_t baseAddr, uint8_t setting)
{
    BW_SIM_CLKDIV1_OUTDIV4(baseAddr, setting);
}

/*!
* @brief Get OUTDIV4.
*
* This function gets divide value OUTDIV4.
*
* @param baseAddr Base address for current SIM instance.
* @return Current divide value.
*/
static inline uint8_t CLOCK_HAL_GetOutDiv4(uint32_t baseAddr)
{
    return BR_SIM_CLKDIV1_OUTDIV4(baseAddr);
}

/*!
 * @brief Sets the clock out dividers setting.
 *
 * This function sets the setting for all clock out dividers at the same time.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param outdiv1      Outdivider1 setting
 * @param outdiv2      Outdivider2 setting
 * @param outdiv3      Outdivider3 setting
 * @param outdiv4      Outdivider4 setting
 */
void CLOCK_HAL_SetOutDiv(uint32_t baseAddr,
                         uint8_t outdiv1,
                         uint8_t outdiv2,
                         uint8_t outdiv3,
                         uint8_t outdiv4);

/*!
 * @brief Gets the clock out dividers setting.
 *
 * This function gets the setting for all clock out dividers at the same time.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param outdiv1      Outdivider1 setting
 * @param outdiv2      Outdivider2 setting
 * @param outdiv3      Outdivider3 setting
 * @param outdiv4      Outdivider4 setting
 */
void CLOCK_HAL_GetOutDiv(uint32_t baseAddr,
                         uint8_t *outdiv1,
                         uint8_t *outdiv2,
                         uint8_t *outdiv3,
                         uint8_t *outdiv4);

/*!
* @brief Sets the ADCx alternate trigger enable setting.
*
* This function enables/disables the alternative conversion triggers for ADCx.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @param enable Enable alternative conversion triggers for ADCx
*               - true:  Select alternative conversion trigger.
*               - false: Select PDB trigger.
*/
static inline void SIM_HAL_SetAdcAlternativeTriggerCmd(uint32_t baseAddr,
                                                       uint32_t instance,
                                                       bool enable)
{
    BW_SIM_SOPT7_ADC0ALTTRGEN(baseAddr, enable ? 1 : 0);
    
}    
/*!
* @brief Gets the ADCx alternate trigger enable setting.
*
* This function gets the ADCx alternate trigger enable setting.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @return enabled True if  ADCx alternate trigger is enabled
*/

static inline bool SIM_HAL_GetAdcAlternativeTriggerCmd(uint32_t baseAddr, uint32_t instance)
{
    bool retValue = false;
    
    retValue = BR_SIM_SOPT7_ADC0ALTTRGEN(baseAddr);
    
    return retValue;
}


/*!
* @brief Sets the ADCx pre-trigger select setting.
*
* This function selects the ADCx pre-trigger source when the alternative
* triggers are enabled through ADCxALTTRGEN.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @param select pre-trigger select setting for ADCx
*/
static inline void SIM_HAL_SetAdcPreTriggerMode(uint32_t baseAddr,
                                                uint32_t instance,
                                                sim_adc_pretrg_sel_t select)
{
    BW_SIM_SOPT7_ADC0PRETRGSEL(baseAddr, select);
    
}
/*!
* @brief Gets the ADCx pre-trigger select setting.
*
* This function  gets the ADCx pre-trigger select setting.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @return select ADCx pre-trigger select setting
*/
static inline sim_adc_pretrg_sel_t SIM_HAL_GetAdcPreTriggerMode(uint32_t baseAddr,
                                                                uint32_t instance)
{
    sim_adc_pretrg_sel_t retValue = (sim_adc_pretrg_sel_t)0;
    
    retValue = (sim_adc_pretrg_sel_t)BR_SIM_SOPT7_ADC0PRETRGSEL(baseAddr);
    
    return retValue;
}

/*!
* @brief Sets the ADCx trigger select setting.
*
* This function  selects the ADCx trigger source when alternative triggers
* are enabled through ADCxALTTRGEN.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @param select       trigger select setting for ADCx
*/
static inline void SIM_HAL_SetAdcTriggerMode(uint32_t baseAddr,
                                             uint32_t instance,
                                             sim_adc_trg_sel_t select)
{
    BW_SIM_SOPT7_ADC0TRGSEL(baseAddr, select);
    
}

/*!
* @brief Gets the ADCx trigger select setting.
*
* This function  gets the ADCx trigger select setting.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @return ADCx trigger select setting
*/
static inline sim_adc_trg_sel_t SIM_HAL_GetAdcTriggerMode(uint32_t baseAddr, uint32_t instance)
{
    sim_adc_trg_sel_t retValue = (sim_adc_trg_sel_t)0;
    
    retValue = (sim_adc_trg_sel_t)BR_SIM_SOPT7_ADC0TRGSEL(baseAddr);
    
    return retValue;
}

/*!
* @brief Sets the ADCx trigger select setting in one function.
*
* This function sets ADC alternate trigger, pre-trigger mode and trigger mode.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @param altTrigEn    Alternative trigger enable or not.
* @param preTrigSel   Pre-trigger mode.
* @param trigSel      Trigger mode.
*/
void SIM_HAL_SetAdcTriggerModeOneStep(uint32_t baseAddr,
                                      uint32_t instance,
                                      bool    altTrigEn,
                                      sim_adc_pretrg_sel_t preTrigSel,
                                      sim_adc_trg_sel_t trigSel);

/*!
* @brief Sets the LPUARTx receive data source select setting.
*
* This function  selects the source for the LPUARTx receive data.
*
* @param baseAddr Register base address of SIM.
* @param instance LPUART instance.
* @param select the source for the LPUARTx receive data
*				 - 00: LPUARTx_RX pin.
*				 - 01: CMP0.
*				 - 11: Reserved.
*/
static inline void SIM_HAL_SetLpUartRxSrcMode(uint32_t baseAddr, uint32_t instance, sim_uart_rxsrc_t select)
{
    BW_SIM_SOPT5_LPUART0RXSRC(baseAddr, select);
    
}

/*!
* @brief Gets the LPUARTx receive data source select setting.
*
* This function  gets the LPUARTx receive data source select setting.
*
* @param baseAddr Register base address of SIM.
* @param instance LPUART instance.
*
* @return select UARTx receive data source select setting
*/
static inline sim_uart_rxsrc_t SIM_HAL_GetLpUartRxSrcMode(uint32_t baseAddr, uint32_t instance)
{
    sim_uart_rxsrc_t retValue = (sim_uart_rxsrc_t)0;
    
    retValue = (sim_uart_rxsrc_t)BR_SIM_SOPT5_LPUART0RXSRC(baseAddr);
    
    return retValue;
}

/*!
* @brief Sets the LPUARTx transmit data source select setting.
*
* This function  selects the source for the LPUARTx transmit data.
*
* @param baseAddr Register base address of SIM.
* @param instance LPUART instance.
* @param select the source for the UARTx transmit data
*				 - 00: LPUARTx_TX pin.
*				 - 01: LPUARTx_TX pin modulated with FTM1 channel 0 output.
*				 - 11: Reserved.
*/
static inline void SIM_HAL_SetLpUartTxSrcMode(uint32_t baseAddr, uint32_t instance, sim_uart_txsrc_t select)
{
    BW_SIM_SOPT5_LPUART0TXSRC(baseAddr, select);
    
}

/*!
* @brief Gets the LPUARTx transmit data source select setting.
*
* This function  gets the LPUARTx transmit data source select setting.
*
* @param baseAddr Register base address of SIM.
* @param instance LPUART instance.
* @return select UARTx transmit data source select setting
*/
static inline sim_uart_txsrc_t SIM_HAL_GetLpUartTxSrcMode(uint32_t baseAddr, uint32_t instance)
{
    sim_uart_txsrc_t retValue =(sim_uart_txsrc_t)0;
    
    retValue = (sim_uart_txsrc_t)BR_SIM_SOPT5_LPUART0TXSRC(baseAddr);
    
    return retValue;
}

/*!
* @brief Sets the LPUARTx Open Drain Enable setting.
*
* This function  enables/disables the LPUARTx Open Drain.
*
* @param baseAddr Register base address of SIM.
* @param instance LPUART instance.
* @param enable Enable/disable LPUARTx Open Drain
*				 - True: Enable LPUARTx Open Drain
*				 - False: Disable LPUARTx Open Drain
*/
static inline void SIM_HAL_SetLpUartOpenDrainCmd(uint32_t baseAddr, uint32_t instance, bool enable)
{
    BW_SIM_SOPT5_LPUART0ODE(baseAddr, enable ? 1 : 0);
    
}

/*!
* @brief Gets the LPUARTx Open Drain Enable setting.
*
* This function  gets the LPUARTx Open Drain Enable setting.
*
* @param baseAddr Register base address of SIM.
* @param instance LPUART instance.
* @return enabled True if LPUARTx Open Drain is enabled.
*/
static inline bool SIM_HAL_GetLpUartOpenDrainCmd(uint32_t baseAddr, uint32_t instance)
{
    bool retValue = false;
    
    retValue = BR_SIM_SOPT5_LPUART0ODE(baseAddr);
    
    return retValue;
}

/*!
* @brief Sets the Timer/PWM x channel y input capture source select setting.
*
* This function  selects the Timer/PWM x channel y input capture source.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @param channel      TPM channel y
* @param select Timer/PWM x channel y input capture source
*/
static inline void SIM_HAL_SetTpmChSrcMode(uint32_t baseAddr,
                                           uint32_t instance,
                                           uint8_t channel,
                                           sim_tpm_ch_src_t select)
{
    if(instance == 1)
    {
        BW_SIM_SOPT4_TPM1CH0SRC(baseAddr, select);
    }
}

/*!
* @brief Gets the Timer/PWM x channel y input capture source select setting.
*
* This function  gets the Timer/PWM x channel y input capture source select setting.
*
* @param baseAddr     Base address for current SIM instance.
* @param instance     device instance.
* @param channel      Tpm channel y
* @return select Timer/PWM x channel y input capture source select setting
*/
static inline sim_tpm_ch_src_t SIM_HAL_GetTpmChSrcMode(uint32_t baseAddr,
                                                       uint32_t instance,
                                                       uint8_t channel)
{
    sim_tpm_ch_src_t retValue = (sim_tpm_ch_src_t)0;
    
    if(instance == 1)
    {
        
	retValue = (sim_tpm_ch_src_t)BR_SIM_SOPT4_TPM1CH0SRC(baseAddr);
    }   
    
    return retValue;
}

/*!
* @brief Gets the Kinetis Sub-Family ID in the System Device ID register (SIM_SDID).
*
* This function  gets the Kinetis Sub-Family ID in System Device ID register.
*
* @param baseAddr     Base address for current SIM instance.
* @return id Kinetis Sub-Family ID
*/
static inline uint32_t SIM_HAL_GetSubFamilyId(uint32_t baseAddr)
{
    return BR_SIM_SDID_SUBFAMID(baseAddr);
}

/*!
* @brief Gets the Kinetis SeriesID in the System Device ID register (SIM_SDID).
*
* This function  gets the Kinetis Series ID in System Device ID register.
*
* @param baseAddr     Base address for current SIM instance.
* @return id Kinetis Series ID
*/
static inline uint32_t SIM_HAL_GetSeriesId(uint32_t baseAddr)
{
    return BR_SIM_SDID_SERIESID(baseAddr);
}

/*!
* @brief Gets the Kinetis Fam ID in System Device ID register (SIM_SDID).
*
* This function  gets the Kinetis Fam ID in System Device ID register.
*
* @param baseAddr     Base address for current SIM instance.
* @return id Kinetis Fam ID
*/
static inline uint32_t SIM_HAL_GetFamId(uint32_t baseAddr)
{
    return BR_SIM_SDID_FAMID(baseAddr);
}

/*!
* @brief Gets the Kinetis Pincount ID in System Device ID register (SIM_SDID).
*
* This function  gets the Kinetis Pincount ID in System Device ID register.
*
* @param baseAddr     Base address for current SIM instance.
* @return id Kinetis Pincount ID
*/
static inline uint32_t SIM_HAL_GetPinCntId(uint32_t baseAddr)
{
    return BR_SIM_SDID_PINID(baseAddr);
}

/*!
* @brief Gets the Kinetis Revision ID in the System Device ID register (SIM_SDID).
*
* This function  gets the Kinetis Revision ID in System Device ID register.
*
* @param baseAddr     Base address for current SIM instance.
* @return id Kinetis Revision ID
*/
static inline uint32_t SIM_HAL_GetRevId(uint32_t baseAddr)
{
    return BR_SIM_SDID_REVID(baseAddr);
}

/*!
* @brief Gets the Kinetis Die ID in the System Device ID register (SIM_SDID).
*
* This function  gets the Kinetis Die ID in System Device ID register.
*
* @param baseAddr     Base address for current SIM instance.
* @return id Kinetis Die ID
*/
static inline uint32_t SIM_HAL_GetDieId(uint32_t baseAddr)
{
    return BR_SIM_SDID_DIEID(baseAddr);
}

/*!
* @brief Gets the program flash size in the Flash Configuration Register 1 (SIM_FCFG).
*
* This function  gets the program flash size in the Flash Configuration Register 1.
*
* @param baseAddr     Base address for current SIM instance.
* @return size Program flash Size
*/
static inline uint32_t SIM_HAL_GetProgramFlashSize(uint32_t baseAddr)
{
    return BR_SIM_FCFG1_PFSIZE(baseAddr);
}

/*!
* @brief Sets the Flash Doze in the Flash Configuration Register 1  (SIM_FCFG).
*
* This function  sets the Flash Doze in the Flash Configuration Register 1.
*
* @param baseAddr     Base address for current SIM instance.
* @param setting Flash Doze setting
*/
static inline void SIM_HAL_SetFlashDoze(uint32_t baseAddr, uint32_t setting)
{
    BW_SIM_FCFG1_FLASHDOZE(baseAddr, setting);
}

/*!
* @brief Gets the Flash Doze in the Flash Configuration Register 1  (SIM_FCFG).
*
* This function  gets the Flash Doze in the Flash Configuration Register 1.
*
* @param baseAddr     Base address for current SIM instance.
* @return setting Flash Doze setting
*/
static inline uint32_t SIM_HAL_GetFlashDoze(uint32_t baseAddr)
{
    return BR_SIM_FCFG1_FLASHDOZE(baseAddr);
}

/*!
* @brief Sets the Flash disable setting.
*
* This function  sets the Flash disable setting in the
* Flash Configuration Register 1.
*
* @param baseAddr     Base address for current SIM instance.
* @param disable      Flash disable setting
*/
static inline void SIM_HAL_SetFlashDisableCmd(uint32_t baseAddr, bool disable)
{
    BW_SIM_FCFG1_FLASHDIS(baseAddr, disable);
}

/*!
* @brief Gets the Flash disable setting.
*
* This function  gets the Flash disable setting in the
* Flash Configuration Register 1.
*
* @param baseAddr     Base address for current SIM instance.
* @return setting Flash disable setting
*/
static inline bool SIM_HAL_GetFlashDisableCmd(uint32_t baseAddr)
{
    return (bool)BR_SIM_FCFG1_FLASHDIS(baseAddr);
}

/*!
* @brief Gets the Flash maximum address block 0 in the Flash Configuration Register 1  (SIM_FCFG).
*
* This function gets the Flash maximum block 0 in Flash Configuration Register 2.
*
* @param baseAddr     Base address for current SIM instance.
* @return address Flash maximum block 0 address
*/
static inline uint32_t SIM_HAL_GetFlashMaxAddrBlock0(uint32_t baseAddr)
{
    return BR_SIM_FCFG2_MAXADDR0(baseAddr);
}

/*!
* @brief Sets the Timer/PWM x external clock pin select setting.
*
* This function  selects the source of the Timer/PWM x external clock pin select.
*
* @param baseAddr	   Base address for current SIM instance.
* @param instance	   device instance.
* @param select Timer/PWM x external clock pin select
*				 - 0: Timer/PWM x external clock driven by the TPM_CLKIN0 pin.
*				 - 1: Timer/PWM x external clock driven by the TPM_CLKIN1 pin.
*/
void SIM_HAL_SetTpmExternalClkPinSelMode(uint32_t baseAddr, uint32_t instance, sim_tpm_clk_sel_t select);

/*!
* @brief Gets the Timer/PWM x external clock pin select setting.
*
* This function  gets the Timer/PWM x external clock pin select setting.
*
* @param baseAddr	   Base address for current SIM instance.
* @param instance	   device instance.
* @return select Timer/PWM x external clock pin select setting
*/
sim_tpm_clk_sel_t SIM_HAL_GetTpmExternalClkPinSelMode(uint32_t baseAddr, uint32_t instance);


#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_SIM_HAL_KL03Z4_H__*/
/*******************************************************************************
* EOF
******************************************************************************/
