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

extern volatile lpuart_state_t		lpuartState;
extern volatile lpuart_user_config_t	lpuartUserConfig;
extern WarpUARTDeviceState		deviceBGXState;

static void	uartRxCallback(uint32_t instance, void * uartState);


void
initBGX()
{
	/*
	 *	Initialize UART and setup callback function.
	 */
	GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	lpuartState.txBuff = deviceBGXState.uartTXBuffer;
	lpuartState.rxBuff = deviceBGXState.uartRXBuffer;
	
	LPUART_DRV_InstallRxCallback(	0,
					&uartRxCallback,
					deviceBGXState.uartRXBuffer,
					(void*)0,
					1);
}

void
activateBGX()
{
	/*
	 *	Make sure regulator set to 3.3V and enable the LOAD pin / VS2
	 */
	GPIO_DRV_SetPinOutput(kWarpPinTPS6274X_VSEL1);
	GPIO_DRV_SetPinOutput(kWarpPinTPS6274X_VSEL2);
	GPIO_DRV_SetPinOutput(kWarpPinTPS6274X_VSEL3);
	GPIO_DRV_SetPinOutput(kWarpPinTPS6274X_VSEL4);
	GPIO_DRV_SetPinOutput(kWarpPinTPS6274X_REGCTRL);

	/*
	 *	Drive /RST on the BGX high
	 */
	GPIO_DRV_SetPinOutput(kWarpPinBGX_nRST);

}

void
deactivateBGX()
{
	/*
	 *	Leave the regulator as-is but disable the LOAD pin
	 */
	//xxx

	/*
	 *	Drive /RST on the BGX low
	 */
	//xxx
}

void
uartRxCallback(uint32_t instance, void * uartState)
{
	/*
	 *	Received data is in deviceBGXState.uartRXBuffer
	 */
	if (deviceBGXState.uartRXBuffer[0] == 0)
	{
		
		/*
		 *	For now, we do nothing with the received data
		 */
	}

	return;
}
