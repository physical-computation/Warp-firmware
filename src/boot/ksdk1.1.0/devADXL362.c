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


extern volatile WarpSPIDeviceState	deviceADXL362State;
extern volatile uint32_t		gWarpSPIBaudRateKbps;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;


/*
 *	Analog Devices ADXL362.
 *
 *	From device manual, Rev. B, Page 19 of 44:
 *
 *		"
 *		The SPI port uses a multibyte structure 
 *		wherein the first byte is a command. The 
 *		ADXL362 command set is:
 *
 *		-	0x0A: write register
 *		-	0x0B: read register
 *		-	0x0D: read FIFO
 *		"
 */
void
initADXL362(WarpSPIDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->signalType	= (	kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ |
						kWarpTypeMaskTemperature
					);
	return;
}

WarpStatus
writeSensorRegisterADXL362(uint8_t command, uint8_t deviceRegister, uint8_t writeValue, int numberOfBytes)
{	
	/*
	 *	Populate the shift-out register with the read-register command,
	 *	followed by the register to be read, followed by a zero byte.
	 */
	deviceADXL362State.spiSourceBuffer[0] = command;
	deviceADXL362State.spiSourceBuffer[1] = deviceRegister;
	deviceADXL362State.spiSourceBuffer[2] = writeValue;

	deviceADXL362State.spiSinkBuffer[0] = 0x00;
	deviceADXL362State.spiSinkBuffer[1] = 0x00;
	deviceADXL362State.spiSinkBuffer[2] = 0x00;

	/*
	 *	First, create a falling edge on chip-select.
	 *
	 */
	GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kWarpPinADXL362_CS);

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
	deviceADXL362State.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceADXL362State.spiSourceBuffer,
					(uint8_t * restrict)deviceADXL362State.spiSinkBuffer,
					numberOfBytes /* transfer size */,
					gWarpSpiTimeoutMicroseconds);
	disableSPIpins();

	/*
	 *	Disengage the ADXL362
	 */
	GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterADXL362(uint8_t deviceRegister, int numberOfBytes)
{	
	return writeSensorRegisterADXL362(0x0B /* command == read register */, deviceRegister, 0x00 /* writeValue */, numberOfBytes);
}

WarpStatus
readSensorSignalADXL362(WarpTypeMask		signal,
			WarpSignalPrecision	precision,
			WarpSignalAccuracy	accuracy,
			WarpSignalReliability	reliability,
			WarpSignalNoise		noise)
{
	if (signal & kWarpTypeMaskTemperature)
	{
		//... etc.
	}

	return kWarpStatusOK;
}