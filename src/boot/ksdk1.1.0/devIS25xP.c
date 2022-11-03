/*
	Authored 2020-2022. Phillip Stanley-Marbell, Orestis Kaparounakis.

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
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devIS25xP.h"

extern volatile WarpSPIDeviceState	deviceIS25xPState;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;
extern uint8_t				gWarpSpiCommonSourceBuffer[];
extern uint8_t				gWarpSpiCommonSinkBuffer[];


void
initIS25xP(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts)
{
	deviceIS25xPState.chipSelectIoPinID		= chipSelectIoPinID;
	deviceIS25xPState.spiSourceBuffer		= gWarpSpiCommonSourceBuffer;
	deviceIS25xPState.spiSinkBuffer			= gWarpSpiCommonSinkBuffer;
	deviceIS25xPState.spiBufferLength		= kWarpMemoryCommonSpiBufferBytes;
	deviceIS25xPState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	// PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	return;
}

WarpStatus
spiTransactionIS25xP(uint8_t ops[], size_t opCount)
{
	spi_status_t	status;

	if (opCount > deviceIS25xPState.spiBufferLength)
	{
		return kWarpStatusBadDeviceCommand;
	}

	warpScaleSupplyVoltage(deviceIS25xPState.operatingVoltageMillivolts);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

	for (int i = 0; (i < opCount) && (i < deviceIS25xPState.spiBufferLength); i++)
	{
		deviceIS25xPState.spiSourceBuffer[i] = ops[i];
		deviceIS25xPState.spiSinkBuffer[i] = 0x00;
	}

	/*
	 *	Create a falling edge on chip-select.
	 */
	// PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	GPIO_DRV_SetPinOutput(deviceIS25xPState.chipSelectIoPinID);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(deviceIS25xPState.chipSelectIoPinID);

	/*
	 *	The result of the SPI transaction will be stored in deviceADXL362State.spiSinkBuffer.
	 *
	 *	Providing a spi_master_user_config_t is optional since it is already provided when we did
	 *	SPI_DRV_MasterConfigureBus(), so we pass in NULL. The "master instance" is always 0 for
	 *	the KL03 since there is only one SPI peripheral.
	 */
	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceIS25xPState.spiSourceBuffer,
					(uint8_t * restrict)deviceIS25xPState.spiSinkBuffer,
					opCount /* transfer size */,
					gWarpSpiTimeoutMicroseconds);
	warpDisableSPIpins();

	/*
	 *	Deassert the IS25xP
	 */
	GPIO_DRV_SetPinOutput(deviceIS25xPState.chipSelectIoPinID);

	if (status != kStatus_SPI_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


WarpStatus
readMemoryIS25xP(uint32_t startAddress, size_t nbyte, void *  buf)
{
	WarpStatus	status;

	// if (NOT RUN MODE)
	// {
	// 	return kWarpStatusBadPowerModeSpecified;
	// }

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x03;	/* NORD */
	ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	ops[3] = (uint8_t)((startAddress & 0x000F));

	status = spiTransactionIS25xP(ops, nbyte+4);
	if (status != kWarpStatusOK)
	{
		return status;
	}

	for (size_t i = 0; i < nbyte; i++)
	{
		((uint8_t*)buf)[i] = deviceIS25xPState.spiSinkBuffer[i+4];
	}

	return kWarpStatusOK;
}

WarpStatus
programPageIS25xP(uint32_t startAddress, size_t nbyte, void *  buf)
{
	WarpStatus	status;

	// if (NOT RUN MODE)
	// {
	// 	return kWarpStatusBadPowerModeSpecified;
	// }

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x02;	/* PP */
	ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	ops[3] = (uint8_t)((startAddress & 0x000F));
	for (size_t i = 0; i < nbyte; i++)
	{
		ops[i+4] = ((uint8_t*)buf)[i];
	}

	return spiTransactionIS25xP(ops, nbyte+4);
}

WarpStatus
eraseSectorIS25xP(uint32_t address)
{
	WarpStatus	status;

	uint8_t	ops[4] = {0};

	ops[0] = 0xD7;	/* SER (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionIS25xP(ops, 4);
}

WarpStatus
erase32kBlockIS25xP(uint32_t address)
{
	WarpStatus	status;

	uint8_t	ops[4] = {0};

	ops[0] = 0x52;	/* BER32K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionIS25xP(ops, 4);
}

WarpStatus
erase64kBlockIS25xP(uint32_t address)
{
	WarpStatus	status;

	uint8_t	ops[4] = {0};

	ops[0] = 0xD8;	/* BER64K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionIS25xP(ops, 4);
}

WarpStatus
chipEraseIS25xP()
{
	WarpStatus	status;

	uint8_t	ops[1] = {0};

	ops[0] = 0xC7;	/* CER (SPI Mode) */

	return spiTransactionIS25xP(ops, 1);
}
