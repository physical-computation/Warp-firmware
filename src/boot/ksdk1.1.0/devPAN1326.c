/*
	Authored 2016-2018. Phillip Stanley-Marbell.

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
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"



static void
initPAN132x(WarpUARTDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->signalType	= kWarpTypeMaskTemperature;

	/*
	 *	Start the 32 kHz oscillator in order to run the PAN1323ETU module.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);

	/*
	 *	Route the internal signal to the pin from the internal logic.
	 *	See Section "5.7.4 RTC_CLKOUT and CLKOUT32K clocking" in 
	 *	KL03P24M48SF0RM.pdf. We set the OSC32KOUT field of the SIM_SOPT1
	 *	register to (1 << 16) (default is 0x00) (See "14.5.1 System Options
	 *	Register 1 (SIM_SOPT1)" of KL03P24M48SF0RM.pdf). 
	 */
	SIM->SOPT1 |= (1 << 16);
}

void
initPAN1326B(WarpUARTDeviceState volatile *  deviceStatePointer)
{
	initPAN132x(deviceStatePointer);

	/*
	 *	Shutdown the Module
	 */
	#ifndef	WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
	#endif
}

void
initPAN1323ETU(WarpUARTDeviceState volatile *  deviceStatePointer)
{
	initPAN132x(deviceStatePointer);
}
