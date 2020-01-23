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
#ifndef __FSL_LPSCI_HAL_H__
#define __FSL_LPSCI_HAL_H__

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup lpsci_hal
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LPSCI_SHIFT (8U)

/*! @brief Error codes for the LPSCI driver. */
typedef enum _lpsci_status
{
    kStatus_LPSCI_Success                  = 0x00U,
    kStatus_LPSCI_BaudRateCalculationError = 0x01U,
    kStatus_LPSCI_RxStandbyModeError       = 0x02U,
    kStatus_LPSCI_ClearStatusFlagError     = 0x03U,
    kStatus_LPSCI_TxNotDisabled            = 0x04U,
    kStatus_LPSCI_RxNotDisabled            = 0x05U,
    kStatus_LPSCI_TxOrRxNotDisabled        = 0x06U,
    kStatus_LPSCI_TxBusy                   = 0x07U,
    kStatus_LPSCI_RxBusy                   = 0x08U,
    kStatus_LPSCI_NoTransmitInProgress     = 0x09U,
    kStatus_LPSCI_NoReceiveInProgress      = 0x0AU,
    kStatus_LPSCI_Timeout                  = 0x0BU,
    kStatus_LPSCI_Initialized              = 0x0CU,
    kStatus_LPSCI_NoDataToDeal             = 0x0DU,
    kStatus_LPSCI_RxOverRun                = 0x0EU
} lpsci_status_t;

/*!
 * @brief LPSCI number of stop bits.
 *
 * These constants define the number of allowable stop bits to configure in a LPSCI baseAddr.
 */
typedef enum _lpsci_stop_bit_count {
    kLpsciOneStopBit = 0U,  /*!< one stop bit */
    kLpsciTwoStopBit = 1U,  /*!< two stop bits */
} lpsci_stop_bit_count_t;

/*!
 * @brief LPSCI parity mode.
 *
 * These constants define the LPSCI parity mode options: disabled or enabled of type even or odd.
 */
typedef enum _lpsci_parity_mode {
    kLpsciParityDisabled = 0x0U,  /*!< parity disabled */
    kLpsciParityEven     = 0x2U,  /*!< parity enabled, type even, bit setting: PE|PT = 10 */
    kLpsciParityOdd      = 0x3U,  /*!< parity enabled, type odd,  bit setting: PE|PT = 11 */
} lpsci_parity_mode_t;

/*!
 * @brief LPSCI number of bits in a character.
 *
 * These constants define the number of allowable data bits per LPSCI character. Note, check the
 * LPSCI documentation to determine if the desired LPSCI baseAddr supports the desired number
 * of data bits per LPSCI character.
 */
typedef enum  _lpsci_bit_count_per_char {
    kLpsci8BitsPerChar = 0U,   /*!< 8-bit data characters */
    kLpsci9BitsPerChar = 1U,   /*!< 9-bit data characters */
} lpsci_bit_count_per_char_t;

/*!
 * @brief LPSCI operation configuration constants.
 *
 * This provides constants for LPSCI operational states: "operates normally"
 * or "stops/ceases operation"
 */
typedef enum _lpsci_operation_config {
    kLpsciOperates = 0U,  /*!< LPSCI continues to operate normally */
    kLpsciStops = 1U,     /*!< LPSCI ceases operation */
} lpsci_operation_config_t;

/*! @brief LPSCI receiver source select mode. */
typedef enum _lpsci_receiver_source {
    kLpsciLoopBack = 0U,  /*!< Internal loop back mode. */
    kLpsciSingleWire = 1U,/*!< Single wire mode. */
} lpsci_receiver_source_t ;

/*!
 * @brief LPSCI wakeup from standby method constants.
 *
 * This provides constants for the two LPSCI wakeup methods: idle-line or address-mark.
 */
typedef enum _lpsci_wakeup_method {
    kLpsciIdleLineWake = 0U,  /*!< The idle-line wakes LPSCI receiver from standby */
    kLpsciAddrMarkWake = 1U,  /*!< The address-mark wakes LPSCI receiver from standby */
} lpsci_wakeup_method_t;

/*!
 * @brief LPSCI idle-line detect selection types.
 *
 * This provides constants for the LPSCI idle character bit-count start: either after start or
 * stop bit.
 */
typedef enum _lpsci_idle_line_select {
    kLpsciIdleLineAfterStartBit = 0U,  /*!< LPSCI idle character bit count start after start bit */
    kLpsciIdleLineAfterStopBit = 1U,   /*!< LPSCI idle character bit count start after stop bit */
} lpsci_idle_line_select_t;

/*!
 * @brief LPSCI break character length settings for transmit/detect.
 *
 * This provides constants for the LPSCI break character length for both transmission and detection
 * purposes. Note that the actual maximum bit times may vary depending on the LPSCI baseAddr.
 */
typedef enum _lpsci_break_char_length {
    kLpsciBreakChar10BitMinimum = 0U, /*!< LPSCI break char length 10 bit times (if M = 0, SBNS = 0) or
                                     11 (if M = 1, SBNS = 0 or M = 0, SBNS = 1) or 12 (if M = 1,
                                     SBNS = 1 or M10 = 1, SNBS = 0) or 13 (if M10 = 1, SNBS = 1) */
    kLpsciBreakChar13BitMinimum = 1U, /*!< LPSCI break char length 13 bit times (if M = 0, SBNS = 0) or
                                     14 (if M = 1, SBNS = 0 or M = 0, SBNS = 1) or 15 (if M = 1,
                                     SBNS = 1 or M10 = 1, SNBS = 0) or 16 (if M10 = 1, SNBS = 1) */
} lpsci_break_char_length_t;

/*!
 * @brief LPSCI single-wire mode transmit direction.
 *
 *  This provides constants for the LPSCI transmit direction when configured for single-wire mode.
 *  The transmit line TXDIR is either an input or output.
 */
typedef enum _lpsci_singlewire_txdir {
    kLpsciSinglewireTxdirIn = 0U,  /*!< LPSCI Single-Wire mode TXDIR input */
    kLpsciSinglewireTxdirOut = 1U, /*!< LPSCI Single-Wire mode TXDIR output */
} lpsci_singlewire_txdir_t;

/*!
 * @brief LPSCI infrared transmitter pulse width options.
 *
 * This provides constants for the LPSCI infrared (IR) pulse widths. Options include 3/16, 1/16
 * 1/32, and 1/4 pulse widths.
 */
typedef enum _lpsci_ir_tx_pulsewidth {
    kLpsciIrThreeSixteenthsWidth = 0U,   /*!< 3/16 pulse */
    kLpsciIrOneSixteenthWidth = 1U,      /*!< 1/16 pulse */
    kLpsciIrOneThirtysecondsWidth = 2U,  /*!< 1/32 pulse */
    kLpsciIrOneFourthWidth = 3U,         /*!< 1/4 pulse */
} lpsci_ir_tx_pulsewidth_t;

/*!
 * @brief LPSCI status flags.
 *
 * This provides constants for the LPSCI status flags for use in the LPSCI functions.
 */
typedef enum _lpsci_status_flag {
    kLpsciTxDataRegEmpty = 0U << LPSCI_SHIFT | BP_UART0_S1_TDRE, /*!< Tx data register empty flag, sets when Tx buffer is empty */
    kLpsciTxComplete     = 0U << LPSCI_SHIFT | BP_UART0_S1_TC,   /*!< Transmission complete flag, sets when transmission activity complete */
    kLpsciRxDataRegFull  = 0U << LPSCI_SHIFT | BP_UART0_S1_RDRF, /*!< Rx data register full flag, sets when the receive data buffer is full */
    kLpsciIdleLineDetect = 0U << LPSCI_SHIFT | BP_UART0_S1_IDLE, /*!< Idle line detect flag, sets when idle line detected */
    kLpsciRxOverrun      = 0U << LPSCI_SHIFT | BP_UART0_S1_OR,   /*!< Rxr Overrun, sets when new data is received before data is read from receive register */
    kLpsciNoiseDetect    = 0U << LPSCI_SHIFT | BP_UART0_S1_NF,   /*!< Rxr takes 3 samples of each received bit.  If any of these samples differ, noise flag sets */
    kLpsciFrameErr       = 0U << LPSCI_SHIFT | BP_UART0_S1_FE,   /*!< Frame error flag, sets if logic 0 was detected where stop bit expected */
    kLpsciParityErr      = 0U << LPSCI_SHIFT | BP_UART0_S1_PF,   /*!< If parity enabled, sets upon parity error detection */
    kLpsciLineBreakDetect    = 1U << LPSCI_SHIFT | BP_UART0_S2_LBKDIF,  /*!< LIN break detect interrupt flag, sets when LIN break char detected and LIN circuit enabled */
    kLpsciRxActiveEdgeDetect = 1U << LPSCI_SHIFT | BP_UART0_S2_RXEDGIF, /*!< Rx pin active edge interrupt flag, sets when active edge detected */
    kLpsciRxActive           = 1U << LPSCI_SHIFT | BP_UART0_S2_RAF,     /*!< Receiver Active Flag (RAF), sets at beginning of valid start bit */
#if FSL_FEATURE_LPSCI_HAS_EXTENDED_DATA_REGISTER_FLAGS
    kLpsciNoiseInCurrentWord     = 2U << LPSCI_SHIFT | BP_UART0_ED_NOISY,   /*!< NOISY bit, sets if noise detected in current data word */
    kLpsciParityErrInCurrentWord = 2U << LPSCI_SHIFT | BP_UART0_ED_PARITYE, /*!< PARITYE bit, sets if noise detected in current data word */
#endif
} lpsci_status_flag_t;

/*!
 * @brief LPSCI interrupt configuration structure, default settings are 0 (disabled).
 *
 * This structure contains the settings for all of the LPSCI interrupt configurations.
 */
typedef enum _lpsci_interrupt {
    kLpsciIntLinBreakDetect  = 0U << LPSCI_SHIFT | BP_UART0_BDH_LBKDIE,  /*!< LIN break detect. */
    kLpsciIntRxActiveEdge    = 0U << LPSCI_SHIFT | BP_UART0_BDH_RXEDGIE, /*!< RX Active Edge. */
    kLpsciIntTxDataRegEmpty  = 1U << LPSCI_SHIFT | BP_UART0_C2_TIE,      /*!< Transmit data register empty. */
    kLpsciIntTxComplete      = 1U << LPSCI_SHIFT | BP_UART0_C2_TCIE,     /*!< Transmission complete. */
    kLpsciIntRxDataRegFull   = 1U << LPSCI_SHIFT | BP_UART0_C2_RIE,     /*!< Receiver data register full. */
    kLpsciIntIdleLine        = 1U << LPSCI_SHIFT | BP_UART0_C2_ILIE,     /*!< Idle line. */
    kLpsciIntRxOverrun       = 2U << LPSCI_SHIFT | BP_UART0_C3_ORIE,     /*!< Receiver Overrun. */
    kLpsciIntNoiseErrFlag    = 2U << LPSCI_SHIFT | BP_UART0_C3_NEIE,     /*!< Noise error flag. */
    kLpsciIntFrameErrFlag    = 2U << LPSCI_SHIFT | BP_UART0_C3_FEIE,     /*!< Framing error flag. */
    kLpsciIntParityErrFlag   = 2U << LPSCI_SHIFT | BP_UART0_C3_PEIE,     /*!< Parity error flag. */
} lpsci_interrupt_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPSCI Common Configurations
 * @{
 */

/*!
 * @brief Initializes the LPSCI controller.
 *
 * This function initializes the module to a known state.
 *
 * @param   baseAddr LPSCI module base address.
 */
void LPSCI_HAL_Init(uint32_t baseAddr);

/*!
 * @brief Enables the LPSCI transmitter.
 *
 * This function allows the user to enable the LPSCI transmitter.
 *
 * @param   baseAddr LPSCI module base address.
 */
static inline void LPSCI_HAL_EnableTransmitter(uint32_t baseAddr)
{
    BW_UART0_C2_TE(baseAddr, 1U);
}

/*!
 * @brief Disables the LPSCI transmitter.
 *
 * This function allows the user to disable the LPSCI transmitter.
 *
 * @param   baseAddr LPSCI module base address.
 */
static inline void LPSCI_HAL_DisableTransmitter(uint32_t baseAddr)
{
    BW_UART0_C2_TE(baseAddr, 0U);
}

/*!
 * @brief Gets the LPSCI transmitter enabled/disabled configuration setting.
 *
 * This function allows the user to get the setting of the LPSCI transmitter.
 *
 * @param   baseAddr LPSCI module base address.
 * @return The state of LPSCI transmitter enable(true)/disable(false) setting.
 */
static inline bool LPSCI_HAL_IsTransmitterEnabled(uint32_t baseAddr)
{
    return (bool)BR_UART0_C2_TE(baseAddr);
}

/*!
 * @brief Enables the LPSCI receiver.
 *
 * This function allows the user to enable the LPSCI receiver.
 *
 * @param   baseAddr LPSCI module base address.
 */
static inline void LPSCI_HAL_EnableReceiver(uint32_t baseAddr)
{
    BW_UART0_C2_RE(baseAddr, 1U);
}

/*!
 * @brief Disables the LPSCI receiver.
 *
 *  This function allows the user to disable the LPSCI receiver.
 *
 * @param   baseAddr LPSCI module base address.
 */
static inline void LPSCI_HAL_DisableReceiver(uint32_t baseAddr)
{
    BW_UART0_C2_RE(baseAddr, 0U);
}

/*!
 * @brief Gets the LPSCI receiver enabled/disabled configuration setting.
 *
 *  This function allows the user to get the setting of the LPSCI receiver.
 *
 * @param   baseAddr LPSCI module base address.
 * @return The state of LPSCI receiver enable(true)/disable(false) setting.
 */
static inline bool LPSCI_HAL_IsReceiverEnabled(uint32_t baseAddr)
{
    return (bool)BR_UART0_C2_RE(baseAddr);
}

/*!
 * @brief Configures the LPSCI baud rate.
 *
 * This function programs the LPSCI baud rate to the desired value passed in by the user. The user
 * must also pass in the module source clock so that the function can calculate the baud
 * rate divisors to their appropriate values.
 * In some LPSCI baseAddrs it is required that the transmitter/receiver be disabled
 * before calling this function.
 * Generally this is applied to all LPSCIs to ensure safe operation.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   sourceClockInHz LPSCI source input clock in Hz.
 * @param   baudRate LPSCI desired baud rate.
 * @return  An error code or kStatus_LPSCI_Success
 */
lpsci_status_t LPSCI_HAL_SetBaudRate(uint32_t baseAddr, uint32_t sourceClockInHz, uint32_t baudRate);

/*!
 * @brief Sets the LPSCI baud rate modulo divisor value.
 *
 * This function allows the user to program the baud rate divisor directly in situations
 * where the divisor value is known. In this case, the user may not want to call the
 * LPSCI_HAL_SetBaudRate() function, as the divisor is already known.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   baudRateDivisor The baud rate modulo division "SBR" value.
 */
void LPSCI_HAL_SetBaudRateDivisor(uint32_t baseAddr, uint16_t baudRateDivisor);

#if FSL_FEATURE_LPSCI_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT
/*!
 * @brief Sets the LPSCI baud rate fine adjust. (Note: Feature available on select
 *        LPSCI baseAddrs used in conjunction with baud rate programming)
 *
 * This function, which programs the baud rate fine adjust, is used together with
 * programming the baud rate modulo divisor in situations where these divisors value are known.
 * In this case, the user may not want to call the LPSCI_HAL_SetBaudRate() function, as the
 * divisors are already known.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   baudFineAdjust Value of 5-bit field used to add more timing resolution to average
 *                          baud rate frequency is 1/32 increments.
 */
static inline void LPSCI_HAL_SetBaudRateFineAdjust(uint32_t baseAddr, uint8_t baudFineAdjust)
{
    assert(baudFineAdjust < 0x1F);
    BW_UART0_C4_BRFA(baseAddr, baudFineAdjust);
}
#endif

/*!
 * @brief Configures the number of bits per character in the LPSCI controller.
 *
 * This function allows the user to configure the number of bits per character according to the
 * typedef lpsci_bit_count_per_char_t.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   bitCountPerChar Number of bits per char (8, 9, or 10, depending on the LPSCI baseAddr).
 */
static inline void LPSCI_HAL_SetBitCountPerChar(uint32_t baseAddr,
                                          lpsci_bit_count_per_char_t bitCountPerChar)
{
    /* config 8- (M=0) or 9-bits (M=1) */
    BW_UART0_C1_M(baseAddr, bitCountPerChar);
}

/*!
 * @brief Configures the parity mode in LPSCI controller.
 *
 * This function allows the user to configure the parity mode of the LPSCI controller to disable
 * it or enable it for even parity or for odd parity.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   parityMode Parity mode setting (enabled, disable, odd, even - see
 *          parity_mode_t struct).
 */
void LPSCI_HAL_SetParityMode(uint32_t baseAddr, lpsci_parity_mode_t parityMode);

#if FSL_FEATURE_LPSCI_HAS_STOP_BIT_CONFIG_SUPPORT
/*!
 * @brief Configures the number of stop bits in the LPSCI controller.
 *
 * This function allows the user to configure the number of stop bits in the LPSCI controller
 * to be one or two stop bits.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   stopBitCount Number of stop bits setting (1 or 2 - see lpsci_stop_bit_count_t struct).
 * @return  An error code (an unsupported setting in some LPSCIs) or kStatus_LPSCI_Success.
 */
static inline void LPSCI_HAL_SetStopBitCount(uint32_t baseAddr, lpsci_stop_bit_count_t stopBitCount)
{
    BW_UART0_BDH_SBNS(baseAddr, stopBitCount);
}
#endif

/*!
 * @brief Enable or disable the transmit inversion control in LPSCI controller.
 *
 * This function allows the user to invert the transmit signals.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable Enable (true) or disable (false) transmit inversion.
 */
static inline void LPSCI_HAL_SetTxInversionCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_C3_TXINV(baseAddr, enable);
}

/*!
 * @brief Enable or disable the receive inversion control in LPSCI controller.
 *
 * This function allows the user to invert the receive signals.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable Enable (true) or disable (false) receive inversion.
 */
static inline void LPSCI_HAL_SetRxInversionCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_S2_RXINV(baseAddr, enable);
}

/*@}*/

/*!
 * @name LPSCI Interrupts and DMA
 * @{
 */

/*!
 * @brief Configures the LPSCI module interrupts to enable/disable various interrupt sources.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   interrupt LPSCI interrupt configuration data.
 * @param   enable   true: enable, false: disable.
 */
void LPSCI_HAL_SetIntMode(uint32_t baseAddr, lpsci_interrupt_t interrupt, bool enable);

/*!
 * @brief Returns whether the LPSCI module interrupts is enabled/disabled.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   interrupt LPSCI interrupt configuration data.
 * @return  true: enable, false: disable.
 */
bool LPSCI_HAL_GetIntMode(uint32_t baseAddr, lpsci_interrupt_t interrupt);

/*!
 * @brief Enables or disables the tx_data_register_empty_interrupt.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable   true: enable, false: disable.
 */
static inline void LPSCI_HAL_SetTxDataRegEmptyIntCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_C2_TIE(baseAddr, (uint8_t)enable);
}

/*!
 * @brief Gets the configuration of the tx_data_register_empty_interrupt enable setting.
 *
 * @param   baseAddr LPSCI module base address.
 * @return  setting of the interrupt enable bit.
 */
static inline bool LPSCI_HAL_GetTxDataRegEmptyIntCmd(uint32_t baseAddr)
{
    return (bool)BR_UART0_C2_TIE(baseAddr);
}

/*!
 * @brief Disables the rx_data_register_full_interrupt.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable   true: enable, false: disable.
 */
static inline void LPSCI_HAL_SetRxDataRegFullIntCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_C2_RIE(baseAddr, (uint8_t)enable);
}

/*!
 * @brief Gets the configuration of the rx_data_register_full_interrupt enable setting.
 *
 * @param   baseAddr LPSCI module base address.
 * @return Bit setting of the interrupt enable bit.
 */
static inline bool LPSCI_HAL_GetRxDataRegFullIntCmd(uint32_t baseAddr)
{
    return (bool)BR_UART0_C2_RIE(baseAddr);
}

#if FSL_FEATURE_LPSCI_HAS_DMA_ENABLE
/*!
 * @brief  Enable or disable LPSCI DMA request for Transmitter.
 *
 * This function allows the user to configure the receive data register full
 * flag to generate a DMA request.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable Transmit DMA request configuration setting (enable: true /disable: false).
 */
void LPSCI_HAL_SetTxDmaCmd(uint32_t baseAddr, bool enable);

/*!
 * @brief  Enable or disable LPSCI DMA request for Receiver.
 *
 * This function allows the user to configure the receive data register full
 * flag to generate a DMA request.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable Receive DMA request configuration setting (enable: true/disable: false).
 */
void LPSCI_HAL_SetRxDmaCmd(uint32_t baseAddr, bool enable);

/*!
 * @brief  Gets the LPSCI Transmit DMA request configuration setting.
 *
 * This function returns the configuration setting of the Transmit DMA request.
 *
 * @param   baseAddr LPSCI module base address.
 * @return  Transmit DMA request configuration setting (enable: true /disable: false).
 */
static inline bool LPSCI_HAL_GetTxDmaCmd(uint32_t baseAddr)
{
    return BR_UART0_C5_TDMAE(baseAddr);
}

/*!
 * @brief  Gets the LPSCI Receive DMA request configuration setting.
 *
 * This function returns the configuration setting of the Receive DMA request.
 *
 * @param   baseAddr LPSCI module base address.
 * @return  Receive DMA request configuration setting (enable: true /disable: false).
 */
static inline bool LPSCI_HAL_GetRxDmaCmd(uint32_t baseAddr)
{
    return BR_UART0_C5_RDMAE(baseAddr);
}
#endif /* FSL_FEATURE_LPSCI_HAS_DMA_ENABLE */

/*!
 * @brief  Get LPSCI tx/rx data register address.
 *
 * This function is used for DMA transfer.
 *
 * @return  LPSCI tx/rx data register address.
 */
static inline uint32_t LPSCI_HAL_GetDataRegAddr(uint32_t baseAddr)
{
    return (uint32_t)HW_UART0_D_ADDR(baseAddr);
}

/*@}*/

/*!
 * @name LPSCI Transfer Functions
 * @{
 */

/*!
 * @brief This function allows the user to send an 8-bit character from the LPSCI data register.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   data The data to send of size 8-bit.
 */
static inline void LPSCI_HAL_Putchar(uint32_t baseAddr, uint8_t data)
{
    HW_UART0_D_WR(baseAddr, data);
}

/*!
 * @brief This function allows the user to send a 9-bit character from the LPSCI data register.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   data The data to send of size 9-bit.
 */
void LPSCI_HAL_Putchar9(uint32_t baseAddr, uint16_t data);

/*!
 * @brief This function allows the user to send a 10-bit character from the LPSCI data register.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   data The data to send of size 10-bit.
 */
void LPSCI_HAL_Putchar10(uint32_t baseAddr, uint16_t data);

/*!
 * @brief This function gets a received 8-bit character from the LPSCI data register.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   readData The received data read from data register of size 8-bit.
 */
static inline void LPSCI_HAL_Getchar(uint32_t baseAddr, uint8_t *readData)
{
    *readData = HW_UART0_D_RD(baseAddr);
}

/*!
 * @brief This function gets a received 9-bit character from the LPSCI data register.
 *
 * @param baseAddr LPSCI module base address.
 * @param readData The received data read from data register of size 9-bit.
 */
void LPSCI_HAL_Getchar9(uint32_t baseAddr, uint16_t *readData);

/*!
 * @brief This function gets a received 10-bit character from the LPSCI data register.
 *
 * @param baseAddr LPSCI module base address.
 * @param readData The received data read from data register of size 10-bit.
 */
void LPSCI_HAL_Getchar10(uint32_t baseAddr, uint16_t *readData);

/*!
 * @brief Send out multiple bytes of data using polling method.
 *
 * This function only supports 8-bit transaction.
 *
 * @param baseAddr LPSCI module base address.
 * @param txBuff The buffer pointer which saves the data to be sent.
 * @param txSize Size of data to be sent in unit of byte.
 */
void LPSCI_HAL_SendDataPolling(uint32_t baseAddr, const uint8_t *txBuff, uint32_t txSize);

/*!
 * @brief Receive multiple bytes of data using polling method.
 *
 * This function only supports 8-bit transaction.
 *
 * @param baseAddr LPSCI module base address.
 * @param rxBuff The buffer pointer which saves the data to be received.
 * @param rxSize Size of data need to be received in unit of byte.
 * @return Whether the transaction is success or rx overrun.
 */
lpsci_status_t LPSCI_HAL_ReceiveDataPolling(uint32_t baseAddr, uint8_t *rxBuff, uint32_t rxSize);

#if FSL_FEATURE_LPSCI_HAS_EXTENDED_DATA_REGISTER_FLAGS
/*!
 * @brief Configures the LPSCI bit 10 (if enabled) or bit 9 (if disabled) as the parity bit in the
 *         serial transmission.
 *
 * This function configures bit 10 or bit 9 to be the parity bit.  To configure bit 10 as the parity
 * bit, the function sets LPSCIx_C4[M10]; it also sets LPSCIx_C1[M] and LPSCIx_C1[PE] as required.
 *
 * @param baseAddr LPSCI module base address.
 * @param enable The setting to enable (true), which configures bit 10 as the parity bit or to
 *  disable (false), which configures bit 9 as the parity bit in the serial transmission.
 */
static inline void LPSCI_HAL_SetBit10AsParitybit(uint32_t baseAddr, bool enable)
{
    /* to enable the parity bit as the tenth data bit, along with enabling LPSCIx_C4[M10]
     * need to also enable parity and set LPSCIx_C1[M] bit
     * assumed that the user has already set the appropriate bits */
    BW_UART0_C4_M10(baseAddr, enable);
}

/*!
 * @brief Gets the configuration of the LPSCI bit 10 (if enabled) or bit 9 (if disabled) as the
 *         parity bit in the serial transmission.
 *
 * This function returns true if bit 10 is configured as the parity bit, otherwise it returns
 * false if bit 9 is configured as the parity bit.
 *
 * @param  baseAddr LPSCI module base address.
 * @return The configuration setting of bit 10 (true), or bit 9 (false) as the parity bit in
 * the serial transmission.
 */
static inline bool LPSCI_HAL_IsBit10SetAsParitybit(uint32_t baseAddr)
{
    /* to see if the parity bit is set as the tenth data bit,
     * return value of LPSCIx_C4[M10] */
    return BR_UART0_C4_M10(baseAddr);
}

/*!
 * @brief  Determines whether the LPSCI received data word was received with noise.
 *
 * This function returns true if the received data word was received with noise. Otherwise,
 * it returns false indicating no noise was detected.
 *
 * @param   baseAddr LPSCI module base address.
 * @return  The status of the NOISY bit in the LPSCI extended data register.
 */
static inline bool LPSCI_HAL_IsCurrentDataWithNoise(uint32_t baseAddr)
{
    return BR_UART0_ED_NOISY(baseAddr);
}

/*!
 * @brief  Determines whether the LPSCI received data word was received with a parity error.
 *
 * This function returns true if the received data word was received with a parity error.
 * Otherwise, it returns false indicating no parity error was detected.
 *
 * @param   baseAddr LPSCI module base address.
 * @return  The status of the PARITYE (parity error) bit in the LPSCI extended data register.
 */
static inline bool LPSCI_HAL_IsCurrentDataWithParityError(uint32_t baseAddr)
{
    return BR_UART0_ED_PARITYE(baseAddr);
}

#endif  /* FSL_FEATURE_LPSCI_HAS_EXTENDED_DATA_REGISTER_FLAGS*/

/*@}*/

/*!
 * @name LPSCI Special Feature Configurations
 * @{
 */

/*!
 * @brief Configures the LPSCI to either operate or cease to operate in WAIT mode.
 *
 * The function configures the LPSCI to either operate or cease to operate when WAIT mode is
 * entered.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   mode The LPSCI WAIT mode operation - operates or ceases to operate in WAIT mode.
 */
static inline void LPSCI_HAL_SetWaitModeOperation(uint32_t baseAddr, lpsci_operation_config_t mode)
{
	/*In CPU wait mode: 0 - lpsci is enabled; 1 - lpsci is disabled */
	BW_UART0_C1_DOZEEN(baseAddr, mode);
}

/*!
 * @brief Determines if the LPSCI operates or ceases to operate in WAIT mode.
 *
 * This function returns kLpsciOperates if the LPSCI has been configured to operate in WAIT mode.
 * Else it returns KLpsciStops if the LPSCI has been configured to cease-to-operate in WAIT mode.
 *
 * @param   baseAddr LPSCI module base address.
 * @return The LPSCI WAIT mode operation configuration, returns either kLpsciOperates or KLpsciStops.
 */
static inline lpsci_operation_config_t LPSCI_HAL_GetWaitModeOperation(uint32_t baseAddr)
{
    /*In CPU wait mode: 0 - lpsci is enabled; 1 - lpsci is disabled */
    return (lpsci_operation_config_t)BR_UART0_C1_DOZEEN(baseAddr);
}

/*!
 * @brief Configures the LPSCI loopback operation.
 *
 * This function enables or disables the LPSCI loopback operation.
 *
 * @param baseAddr LPSCI module base address.
 * @param enable The LPSCI loopback mode configuration, either disabled (false) or enabled (true).
 */
static inline void LPSCI_HAL_SetLoopCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_C1_LOOPS(baseAddr, enable);
}

/*!
 * @brief Configures the LPSCI single-wire operation.
 *
 * This function enables or disables the LPSCI single-wire operation.
 * In some LPSCI baseAddrs it is required that the transmitter/receiver be disabled
 * before calling this function.
 * This may be applied to all LPSCIs to ensure safe operation.
 *
 * @param baseAddr LPSCI module base address.
 * @param enable The LPSCI single-wire mode configuration, either disabled (false) or enabled (true).
 */
static inline void LPSCI_HAL_SetReceiverSource(uint32_t baseAddr, lpsci_receiver_source_t source)
{
    BW_UART0_C1_RSRC(baseAddr, source);
}
/*!
 * @brief Configures the LPSCI transmit direction while in single-wire mode.
 *
 * This function configures the transmitter direction when the LPSCI is configured for single-wire
 * operation.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   direction The LPSCI single-wire mode transmit direction configuration of type
 *                    lpsci_singlewire_txdir_t (either kLpsciSinglewireTxdirIn or
 *                    kLpsciSinglewireTxdirOut.
 */
static inline void LPSCI_HAL_SetTransmitterDir(uint32_t baseAddr, lpsci_singlewire_txdir_t direction)
{
    /* configure LPSCI transmit direction (input or output) when in single-wire mode
     * it is assumed LPSCI is in single-wire mode */
    BW_UART0_C3_TXDIR(baseAddr, direction);
}

/*!
 * @brief  Places the LPSCI receiver in standby mode.
 *
 * This function, when called, places the LPSCI receiver into standby mode.
 * In some LPSCI baseAddrs, there are conditions that must be met before placing Rx in standby mode.
 * Before placing LPSCI in standby, determine if receiver is set to
 * wake on idle, and if receiver is already in idle state.
 * NOTE: RWU should only be set with C1[WAKE] = 0 (wakeup on idle) if the channel is currently
 * not idle.
 * This can be determined by the S2[RAF] flag. If set to wake up FROM an IDLE event and the channel
 * is already idle, it is possible that the LPSCI will discard data because data must be received
 * (or a LIN break detect) after an IDLE is detected before IDLE is allowed to be reasserted.
 *
 * @param baseAddr LPSCI module base address.
 * @return Error code or kStatus_LPSCI_Success.
 */
lpsci_status_t LPSCI_HAL_PutReceiverInStandbyMode(uint32_t baseAddr);

/*!
 * @brief  Places the LPSCI receiver in normal mode (disable standby mode operation).
 *
 * This function, when called, places the LPSCI receiver into normal mode and out of
 * standby mode.
 *
 * @param   baseAddr LPSCI module base address.
 */
static inline void LPSCI_HAL_PutReceiverInNormalMode(uint32_t baseAddr)
{
    HW_UART0_C2_CLR(baseAddr, BM_UART0_C2_RWU);
}

/*!
 * @brief  Determines if the LPSCI receiver is currently in standby mode.
 *
 * This function determines the state of the LPSCI receiver. If it returns true, this means
 * that the LPSCI receiver is in standby mode; if it returns false, the LPSCI receiver
 * is in normal mode.
 *
 * @param   baseAddr LPSCI module base address.
 * @return The LPSCI receiver is in normal mode (false) or standby mode (true).
 */
static inline bool LPSCI_HAL_IsReceiverInStandby(uint32_t baseAddr)
{
    return BR_UART0_C2_RWU(baseAddr);
}

/*!
 * @brief  Selects the LPSCI receiver wakeup method (idle-line or address-mark) from standby mode.
 *
 * This function configures the wakeup method of the LPSCI receiver from standby mode.  The options
 * are idle-line wake or address-mark wake.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   method The LPSCI receiver wakeup method options: kLpsciIdleLineWake - Idle-line wake or
 *                 kLpsciAddrMarkWake - address-mark wake.
 */
static inline void LPSCI_HAL_SetReceiverWakeupMethod(uint32_t baseAddr, lpsci_wakeup_method_t method)
{
    BW_UART0_C1_WAKE(baseAddr, method);
}

/*!
 * @brief  Gets the LPSCI receiver wakeup method (idle-line or address-mark) from standby mode.
 *
 * This function returns how the LPSCI receiver is configured to wake from standby mode. The
 * wake method options that can be returned are kLpsciIdleLineWake or kLpsciAddrMarkWake.
 *
 * @param   baseAddr LPSCI module base address.
 * @return  The LPSCI receiver wakeup from standby method, false: kLpsciIdleLineWake (idle-line wake)
 *          or true: kLpsciAddrMarkWake (address-mark wake).
 */
static inline lpsci_wakeup_method_t LPSCI_HAL_GetReceiverWakeupMethod(uint32_t baseAddr)
{
    return (lpsci_wakeup_method_t)BR_UART0_C1_WAKE(baseAddr);
}

/*!
 * @brief  Configures the operation options of the LPSCI idle line detect.
 *
 * This function allows the user to configure the LPSCI idle-line detect operation. There are two
 * separate operations for the user to configure, the idle line bit-count start and the receive
 * wake up affect on IDLE status bit. The user will pass in a structure of type
 * lpsci_idle_line_config_t.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   idleLine Idle bit count start: 0 - after start bit (default), 1 - after stop bit
 * @param   rxWakeIdleDetect Receiver Wake Up Idle Detect. IDLE status bit operation during receive
 *          standby. Controls whether idle character that wakes up receiver will also set IDLE status
 *          bit. 0 - IDLE status bit doesn't get set (default), 1 - IDLE status bit gets set
 */
void LPSCI_HAL_ConfigIdleLineDetect(uint32_t baseAddr, uint8_t idleLine, uint8_t rxWakeIdleDetect);

/*!
 * @brief  Configures the LPSCI break character transmit length.
 *
 * This function allows the user to configure the LPSCI break character transmit length. Refer to
 * the typedef lpsci_break_char_length_t for setting options.
 * In some LPSCI baseAddrs it is required that the transmitter be disabled before calling
 * this function. This may be applied to all LPSCIs to ensure safe operation.
 *
 * @param baseAddr LPSCI module base address.
 * @param length The LPSCI break character length setting of type lpsci_break_char_length_t, either a
 *               minimum 10-bit times or a minimum 13-bit times.
 */
static inline void LPSCI_HAL_SetBreakCharTransmitLength(uint32_t baseAddr,
                                                        lpsci_break_char_length_t length)
{
    /* Configure BRK13 - Break Character transmit length configuration
     * LPSCI break character length setting:
     * 0 - minimum 10-bit times (default),
     * 1 - minimum 13-bit times */
    BW_UART0_S2_BRK13(baseAddr, length);
}

/*!
 * @brief  Configures the LPSCI break character detect length.
 *
 * This function allows the user to configure the LPSCI break character detect length. Refer to
 * the typedef lpsci_break_char_length_t for setting options.
 *
 * @param baseAddr LPSCI module base address.
 * @param length The LPSCI break character length setting of type lpsci_break_char_length_t, either a
 *               minimum 10-bit times or a minimum 13-bit times.
 */
static inline void LPSCI_HAL_SetBreakCharDetectLength(uint32_t baseAddr,
                                                      lpsci_break_char_length_t length)
{
    /* Configure LBKDE - Break Character detect length configuration
     * LPSCI break character length setting:
     * 0 - minimum 10-bit times (default),
     * 1 - minimum 13-bit times */
    BW_UART0_S2_LBKDE(baseAddr, length);
}

/*!
 * @brief  Configures the LPSCI transmit send break character operation.
 *
 * This function allows the user to queue a LPSCI break character to send.  If true is passed into
 * the function, then a break character is queued for transmission.  A break character will
 * continuously be queued until this function is called again when a false is passed into this
 * function.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable If false, the LPSCI normal/queue break character setting is disabled, which
 *                 configures the LPSCI for normal transmitter operation. If true, a break
 *                 character is queued for transmission.
 */
static inline void LPSCI_HAL_SetBreakCharCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_C2_SBK(baseAddr, enable);
}

/*!
 * @brief  Configures the LPSCI match address mode control operation. (Note: Feature available on
 *         select LPSCI baseAddrs)
 *
 * The function allows the user to configure the LPSCI match address control operation. The user
 * has the option to enable the match address mode and to program the match address value. There
 * are two match address modes, each with its own enable and programmable match address value.
 *
 * @param  baseAddr LPSCI module base address.
 * @param  matchAddrMode1 If true, this enables match address mode 1 (MAEN1), where false disables.
 * @param  matchAddrMode2 If true, this enables match address mode 2 (MAEN2), where false disables.
 * @param  matchAddrValue1 The match address value to program for match address mode 1.
 * @param  matchAddrValue2 The match address value to program for match address mode 2.
 */
void LPSCI_HAL_SetMatchAddress(uint32_t baseAddr, bool matchAddrMode1, bool matchAddrMode2,
                              uint8_t matchAddrValue1, uint8_t matchAddrValue2);

#if FSL_FEATURE_LPSCI_HAS_BIT_ORDER_SELECT
/*!
 * @brief Configures the LPSCI to send data MSB first
 * (Note: Feature available on select LPSCI baseAddrs)
 *
 * The function allows the user to configure the LPSCI to send data MSB first or LSB first.
 * In some LPSCI baseAddrs it is required that the transmitter/receiver be disabled
 * before calling this function.
 * This may be applied to all LPSCIs to ensure safe operation.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable This configures send MSB first mode configuration. If true, the data is sent MSB
 *                 first; if false, it is sent LSB first.
 */
static inline void LPSCI_HAL_SetSendMsbFirstCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_S2_MSBF(baseAddr, enable);
}
#endif

#if FSL_FEATURE_LPSCI_HAS_MODEM_SUPPORT
/*!
 * @brief  Enables the LPSCI receiver request-to-send functionality.
 *
 * This function allows the user to enable the LPSCI receiver request-to-send (RTS) functionality.
 * By enabling, it allows the RTS output to control the CTS input of the transmitting device to
 * prevent receiver overrun. RTS is deasserted if the number of characters in the receiver data
 * register (FIFO) is equal to or greater than RWFIFO[RXWATER]. RTS is asserted when the
 * number of characters in the receiver data register (FIFO) is less than RWFIFO[RXWATER].
 * Do not set both RXRTSE and TXRTSE.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable   Enable or disable receiver rts.
 */
static inline void LPSCI_HAL_SetReceiverRtsCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_MODEM_RXRTSE(baseAddr, enable);
}

/*!
 * @brief  Enables the LPSCI transmitter request-to-send functionality.
 *
 * This function allows the user to enable the LPSCI transmitter request-to-send (RTS) functionality.
 * When enabled, it allows the LPSCI to control the RTS assertion before and after a transmission
 * such that when a character is placed into an empty transmitter data buffer, RTS
 * asserts one bit time before the start bit is transmitted. RTS deasserts one bit time after all
 * characters in the transmitter data buffer and shift register are completely sent, including
 * the last stop bit.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable   Enable or disable transmitter RTS.
 */
static inline void LPSCI_HAL_SetTransmitterRtsCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_MODEM_TXRTSE(baseAddr, enable);
}

/*!
 * @brief  Configures the LPSCI transmitter RTS polarity.
 *
 * This function allows the user configure the transmitter RTS polarity to be either active low
 * or active high.
 *
 * @param baseAddr LPSCI module base address.
 * @param polarity The LPSCI transmitter RTS polarity setting (false - active low,
 *                 true - active high).
 */
static inline void LPSCI_HAL_SetTransmitterRtsPolarityMode(uint32_t baseAddr, bool polarity)
{
    BW_UART0_MODEM_TXRTSPOL(baseAddr, polarity);
}

/*!
 * @brief  Enables the LPSCI transmitter clear-to-send functionality.
 *
 * This function allows the user to enable the LPSCI transmitter clear-to-send (CTS) functionality.
 * When enabled, the transmitter checks the state of CTS each time it is ready to send a character.
 * If CTS is asserted, the character is sent. If CTS is deasserted, the signal TXD remains in
 * the mark state and transmission is delayed until CTS is asserted. Changes in CTS as a
 * character is being sent do not affect its transmission.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable   Enable or disable transmitter CTS.
 */
static inline void LPSCI_HAL_SetTransmitterCtsCmd(uint32_t baseAddr, bool enable)
{
    BW_UART0_MODEM_TXCTSE(baseAddr, enable);
}

#endif  /* FSL_FEATURE_LPSCI_HAS_MODEM_SUPPORT*/

#if FSL_FEATURE_LPSCI_HAS_IR_SUPPORT
/*!
 * @brief  Configures the LPSCI infrared operation.
 *
 * The function allows the user to enable or disable the LPSCI infrared (IR) operation
 * and to configure the IR pulse width.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   enable Enable (true) or disable (false) the infrared operation.
 * @param   pulseWidth The LPSCI transmit narrow pulse width setting of type lpsci_ir_tx_pulsewidth_t.
 */
void LPSCI_HAL_SetInfraredOperation(uint32_t baseAddr,
                                    bool enable,
                                    lpsci_ir_tx_pulsewidth_t pulseWidth);
#endif  /* FSL_FEATURE_LPSCI_HAS_IR_SUPPORT*/

/*@}*/

/*!
 * @name LPSCI Status Flags
 * @{
 */

/*!
 * @brief  Gets all  LPSCI status flag states.
 *
 * @param   baseAddr LPSCI module base address.
 * @param   statusFlag Status flag name.
 */
bool LPSCI_HAL_GetStatusFlag(uint32_t baseAddr, lpsci_status_flag_t statusFlag);

/*!
 * @brief  Gets the LPSCI Transmit data register empty flag.
 *
 * This function returns the state of the LPSCI Transmit data register empty flag.
 *
 * @param baseAddr LPSCI module base address.
 * @return The status of Transmit data register empty flag, which is set when transmit buffer
 *          is empty.
 */
static inline bool LPSCI_HAL_IsTxDataRegEmpty(uint32_t baseAddr)
{
    return BR_UART0_S1_TDRE(baseAddr);
}

/*!
 * @brief  Gets the LPSCI Transmission complete flag.
 *
 * This function returns the state of the LPSCI Transmission complete flag.
 *
 * @param baseAddr LPSCI module base address.
 * @return The status of Transmission complete flag, which is set when the transmitter is idle
 *         (transmission activity complete).
 */
static inline bool LPSCI_HAL_IsTxComplete(uint32_t baseAddr)
{
    return BR_UART0_S1_TC(baseAddr);
}

/*!
 * @brief  Gets the LPSCI Receive data register full flag.
 *
 * This function returns the state of the LPSCI Receive data register full flag.
 *
 * @param baseAddr LPSCI module base address.
 * @return The status of Receive data register full flag, which is set when the receive data buffer
 *         is full.
 */
static inline bool LPSCI_HAL_IsRxDataRegFull(uint32_t baseAddr)
{
    return BR_UART0_S1_RDRF(baseAddr);
}

/*!
 * @brief  Clears an individual and specific LPSCI status flag.
 *
 * This function allows the user to clear an individual and specific LPSCI status flag. Refer to
 * structure definition lpsci_status_flag_t for list of status bits.
 *
 * @param baseAddr LPSCI module base address.
 * @param statusFlag The desired LPSCI status flag to clear.
 * @return An error code or kStatus_LPSCI_Success.
 */
lpsci_status_t LPSCI_HAL_ClearStatusFlag(uint32_t baseAddr, lpsci_status_flag_t statusFlag);

/*@}*/


#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPSCI_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

