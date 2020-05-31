/*
	Authored 2019. Vasileios Tsoutsouras.

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
#include "devISL23415.h"

extern volatile WarpSPIDeviceState	deviceISL23415State;
extern volatile uint32_t		gWarpSPIBaudRateKbps;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;

/*
 *	From device manual, Rev. B, Page 19 of 44:
 *
 *		"
 *		The ISL23415 contains two volatile 8-bit registers: the Wiper
 *		Register (WR) and the Access Control Register (ACR).
 *
 *		-	0x10: ACR - DEFAULT SETTING: 0x40
 *		-	0x00: WR - DEFAULT SETTING: 0x80
 *		"
 */
void		
initISL23415(WarpSPIDeviceState volatile *  deviceStatePointer) 
{
	deviceStatePointer->signalType	= (
					kWarpTypeMaskMax
				); /* FIXME */
	return;
}

WarpStatus	
readDeviceRegisterISL23415(uint8_t deviceRegister, int numberOfBytes) 
{
	/*
	 *	Two ISL23415 configured in Daisy Chain Configuration.
	 */

	/*
	 *	Populate the shift-out register with the read-register command,
	 *	followed by the register to be read, followed by a zero byte.
	 */
	if (deviceRegister == kWarpISL23415RegACR) 
	{
		//deviceISL23415State.spiSourceBuffer[0] = 0b00100000; /* ACR READ */
		deviceISL23415State.spiSourceBuffer[0] = 0b10010000; /* ACR READ of DCP1 */
		deviceISL23415State.spiSourceBuffer[1] = 0x00; /* Dummy data - NOP */
		deviceISL23415State.spiSourceBuffer[2] = 0b10010000; /* ACR READ of DCP0 */
		deviceISL23415State.spiSourceBuffer[3] = 0x00; /* Dummy data - NOP */

	} 
	else if (deviceRegister == kWarpISL23415RegWR) 
	{
		deviceISL23415State.spiSourceBuffer[0] = 0b10000000; /* WR0 READ of DCP1 */
		deviceISL23415State.spiSourceBuffer[1] = 0x00; /* Dummy data - NOP */
		deviceISL23415State.spiSourceBuffer[2] = 0b10000000; /* WR0 READ of DCP0 */
		deviceISL23415State.spiSourceBuffer[3] = 0x00; /* Dummy data - NOP */
	} 
	else 
	{
		/* FIXME */
	}

	deviceISL23415State.spiSinkBuffer[0] = 0xFF;
	deviceISL23415State.spiSinkBuffer[1] = 0xFF;
	deviceISL23415State.spiSinkBuffer[2] = 0xFF;
	deviceISL23415State.spiSinkBuffer[3] = 0xFF;

	enableSPIpins();
	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kWarpPinISL23415_nCS);
	
	/*
	 *	The result of the SPI transaction will be stored in deviceISL23415State.spiSinkBuffer.
	 *
	 *	Providing a device structure here is optional since it 
	 *	is already provided when we did SPI_DRV_MasterConfigureBus(),
	 *	so we pass in NULL.
	 *
	 *	TODO: the "master instance" is always 0 for the KL03 since
	 *	there is only one SPI peripheral. We however should remove
	 *	the '0' magic number and place this in a Warp-HWREV0 header
	 *	file.
	 */
	
	deviceISL23415State.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(
					0 /* master instance */, 	
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceISL23415State.spiSourceBuffer,
					(uint8_t * restrict)deviceISL23415State.spiSinkBuffer,
					numberOfBytes /* transfer size */,
					gWarpSpiTimeoutMicroseconds);

	/* Drive /CS up. */
	OSA_TimeDelay(50);
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);

	deviceISL23415State.spiSourceBuffer[0] = 0x00; /* NOP */
	deviceISL23415State.spiSourceBuffer[1] = 0x00; /* NOP */
	deviceISL23415State.spiSourceBuffer[2] = 0x00; /* NOP */
	deviceISL23415State.spiSourceBuffer[3] = 0x00; /* NOP */

	deviceISL23415State.spiSinkBuffer[0] = 0xFF;
	deviceISL23415State.spiSinkBuffer[1] = 0xFF;
	deviceISL23415State.spiSinkBuffer[2] = 0xFF;
	deviceISL23415State.spiSinkBuffer[3] = 0xFF;

	/* Drive /CS down. */
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kWarpPinISL23415_nCS);
	
	/*
	 *	The result of the SPI transaction will be stored in deviceISL23415State.spiSinkBuffer.
	 *
	 *	Providing a device structure here is optional since it 
	 *	is already provided when we did SPI_DRV_MasterConfigureBus(),
	 *	so we pass in NULL.
	 *
	 *	TODO: the "master instance" is always 0 for the KL03 since
	 *	there is only one SPI peripheral. We however should remove
	 *	the '0' magic number and place this in a Warp-HWREV0 header
	 *	file.
	 */
	
	deviceISL23415State.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(
					0 /* master instance */, 	
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceISL23415State.spiSourceBuffer,
					(uint8_t * restrict)deviceISL23415State.spiSinkBuffer,
					numberOfBytes /* transfer size */,
					gWarpSpiTimeoutMicroseconds);
	
	/* Drive /CS up. */
	OSA_TimeDelay(50);
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);

	disableSPIpins();	


	return kWarpStatusOK;
}

WarpStatus	
writeDeviceRegisterISL23415(uint8_t deviceRegister, uint8_t writeValue[2], int numberOfBytes)
{
	/*
	 *	Populate the shift-out register with the read-register command,
	 *	followed by the register to be read, followed by a zero byte XXX.
	 */
	if (deviceRegister == kWarpISL23415RegACR) 
	{
		deviceISL23415State.spiSourceBuffer[0] = 0b01100000; /* ACR WRITE of DCP 1 */
		deviceISL23415State.spiSourceBuffer[1] = writeValue[0];//writeValue;
		deviceISL23415State.spiSourceBuffer[2] = 0b01100000; /* ACR WRITE of DCP 0 */
		deviceISL23415State.spiSourceBuffer[3] = writeValue[1];
	} 
	else if (deviceRegister == kWarpISL23415RegWR) 
	{
		deviceISL23415State.spiSourceBuffer[0] = 0b11000000; /* WR0 WRITE of DCP 1 */
		deviceISL23415State.spiSourceBuffer[1] = writeValue[0];//writeValue;
		deviceISL23415State.spiSourceBuffer[2] = 0b11000000; /* WR0 WRITE of DCP 0 */
		deviceISL23415State.spiSourceBuffer[3] = writeValue[1];
	} 
	else 
	{
		/* FIXME */
	}
	
	deviceISL23415State.spiSinkBuffer[0] = 0x00;
	deviceISL23415State.spiSinkBuffer[1] = 0x00;
	deviceISL23415State.spiSinkBuffer[2] = 0x00;
	deviceISL23415State.spiSinkBuffer[3] = 0x00;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kWarpPinISL23415_nCS);

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
	 */
	enableSPIpins();
	deviceISL23415State.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */, 	
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceISL23415State.spiSourceBuffer,
					(uint8_t * restrict)deviceISL23415State.spiSinkBuffer,
					numberOfBytes /* transfer size */,
					gWarpSpiTimeoutMicroseconds);

	/* Drive /CS up. */
	OSA_TimeDelay(50);
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);

	disableSPIpins();

	return kWarpStatusOK;
}

