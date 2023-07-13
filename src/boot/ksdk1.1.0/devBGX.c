/*
	Authored 2021, Phillip Stanley-Marbell.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_lpuart_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile lpuart_state_t			lpuartState;
extern volatile lpuart_user_config_t	lpuartUserConfig;
extern volatile WarpUARTDeviceState		deviceBGXState;

static void				uartRxCallback(uint32_t instance, void * uartState);
static void				powerUpBGX(void);
static void				powerDownBGX(void);

void
initBGX(uint16_t operatingVoltageMillivolts)
{
	/*
	 *	Initialize UART and setup callback function.
	 */
	lpuartState.txBuff = (uint8_t *)deviceBGXState.uartTXBuffer;
	lpuartState.rxBuff = (uint8_t *)deviceBGXState.uartRXBuffer;

	LPUART_DRV_InstallRxCallback(	0,						/*	uint32_t instance		*/
					&uartRxCallback,				/*	lpuart_rx_callback_t function	*/
					(uint8_t *)deviceBGXState.uartRXBuffer,	/*	uint8_t ∗ rxBuff		*/
					(void *)0, 					/*	void ∗callbackParam		*/
					1						/*	bool alwaysEnableRxIrq		*/);

	/*
	 *	powerUpBGX() depends on the operating voltage configuration,
	 *	so need to make sure to do that first.
	 */
	deviceBGXState.operatingVoltageMillivolts = operatingVoltageMillivolts;
	powerUpBGX();
	deviceBGXState.isInitialized = true;
}

void
deinitBGX(WarpUARTDeviceState volatile *  deviceStatePointer)
{
	powerDownBGX();
	deviceBGXState.isInitialized = false;
}

void
powerUpBGX(void)
{
	/*
	 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
	 *
	 *	Setup:
	 *		PTA5/kWarpPinBGX_nRST for GPIO
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);

	warpScaleSupplyVoltage(deviceBGXState.operatingVoltageMillivolts);

	/*
	 *	The BGX datasheet says
	 *
	 *		"Reset input, active low. To apply an external reset source to this pin,
	 *		it is required to only drive this pin low during reset, and let the
	 *		internal pull-up ensure that reset is released."
	 *
	 *	Drive /RST on the BGX high
	 */
	GPIO_DRV_SetPinOutput(kWarpPinBGX_nRST);
}

void
powerDownBGX()
{
	/*
	 *	The BGX datasheet says
	 *
	 *		"Reset input, active low. To apply an external reset source to this pin,
	 *		it is required to only drive this pin low during reset, and let the
	 *		internal pull-up ensure that reset is released."
	 *
	 *	Since it is not connected to an open-drain input in Warp revC
	 *	do nothing special with kWarpPinBGX_nRST on powerdown.
	 */

	return;
}

void
uartRxCallback(uint32_t instance, void *  uartState)
{
	/*
	 *	We don't do anything special upon receipt of UART bytes.
	 *	If we wanted to, that code would go here.
	 */
	//warpPrint("In uartRxCallback(), deviceBGXState.uartRXBuffer [%s]\n", deviceBGXState.uartRXBuffer);
	//SEGGER_RTT_WriteString(0, "In uartRxCallback()...\n");

	return;
}
