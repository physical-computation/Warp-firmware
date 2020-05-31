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


//TODO: need a better way to enable variants of the firmware instead of including, e.g., glaux.h which defines WARP_BUILD_ENABLE_GLAUX_VARIANT as the means of enabling Glauc build.
//TODO: one possibility is to -DWARP_BUILD_ENABLE_GLAUX_VARIANT as a build flag 
/*
 *	Glaux.h needs to come before gpio_pins.h
 */
#include "glaux.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpSPIDeviceState	deviceIS25xPState;
extern volatile uint32_t		gWarpSPIBaudRateKbps;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;


void
initIS25xP(WarpSPIDeviceState volatile *  deviceStatePointer)
{
	/*
	 *	TODO: Need to decide/define on singal type for devices that are not sensors
	 */
	deviceStatePointer->signalType	= (
						kWarpTypeMaskMax
					);
	return;
}

//TODO: rather than listing out the ops, take in an array
WarpStatus
spiTransactionIS25xP(uint8_t op0, uint8_t op1, uint8_t op2, uint8_t op3, uint8_t op4, uint8_t op5, uint8_t op6, int opCount)
{
	//TODO: need to allow each SPI device to specify how many entries are in its source and sink buffer, rather than one size fits all. for the IS25xP, we need up to 6

	/*
	 *	Populate the shift-out register with the read-register command,
	 *	followed by the register to be read, followed by a zero byte.
	 */
	deviceIS25xPState.spiSourceBuffer[0] = op0;
	deviceIS25xPState.spiSourceBuffer[1] = op1;
	deviceIS25xPState.spiSourceBuffer[2] = op2;
	deviceIS25xPState.spiSourceBuffer[3] = op3;
	deviceIS25xPState.spiSourceBuffer[4] = op4;
	deviceIS25xPState.spiSourceBuffer[5] = op5;
	deviceIS25xPState.spiSourceBuffer[5] = op6;

	deviceIS25xPState.spiSinkBuffer[0] = 0x00;
	deviceIS25xPState.spiSinkBuffer[1] = 0x00;
	deviceIS25xPState.spiSinkBuffer[2] = 0x00;
	deviceIS25xPState.spiSinkBuffer[3] = 0x00;
	deviceIS25xPState.spiSinkBuffer[4] = 0x00;
	deviceIS25xPState.spiSinkBuffer[5] = 0x00;
	deviceIS25xPState.spiSinkBuffer[6] = 0x00;

	/*
	 *	First, create a falling edge on chip-select.
	 */
	// TODO: need a better architecture for handling the design variants and their different chip selects...
#ifdef WARP_BUILD_ENABLE_GLAUX_VARIANT
	GPIO_DRV_SetPinOutput(kGlauxPinFlash_CS);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kGlauxPinFlash_CS);
#else
	//TODO
#endif

	/*
	 *	The result of the SPI transaction will be stored in deviceADXL362State.spiSinkBuffer.
	 *
	 *	Providing a device structure here is optional since it 
	 *	is already provided when we did SPI_DRV_MasterConfigureBus(),
	 *	so we pass in NULL.
	 *
	 *	TODO: the "master instance" is always 0 for the KL03 since
	 *	there is only one SPI peripheral. We however should remove
	 *	the '0' magic number and place this in a Warp-HWREV0 header
	 *	file.
	 *
	 *	TODO: the transfer size should be the per-device sized spiSinkBuffer length, not the current magic number
	 */
	enableSPIpins();
	deviceIS25xPState.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceIS25xPState.spiSourceBuffer,
					(uint8_t * restrict)deviceIS25xPState.spiSinkBuffer,
					opCount /* transfer size */,
					gWarpSpiTimeoutMicroseconds);
	disableSPIpins();

	/*
	 *	Disengage the IS25xP
	 */
	// TODO: need a better architecture for handling the design variants and their different chip selects...
#ifdef WARP_BUILD_ENABLE_GLAUX_VARIANT
	GPIO_DRV_SetPinOutput(kGlauxPinFlash_CS);
#else
	//TODO
#endif

	return kWarpStatusOK;
}
