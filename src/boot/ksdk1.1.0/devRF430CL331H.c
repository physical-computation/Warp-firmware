/*
	Authored 2022. James T. Meech.

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

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
extern volatile WarpI2CDeviceState	deviceRF430CL331HState;

void
initRF430CL331H(const uint8_t i2cAddress)
{
	deviceRF430CL331HState.i2cAddress			= i2cAddress;
	return;
}

WarpStatus
readSensorRegisterRF430CL331H(uint8_t deviceRegisterMSB, uint8_t deviceRegisterLSB, int numberOfBytes)
{
	uint8_t cmdBuf[2] = {0xFF, 0xFF};
	i2c_status_t status;

	i2c_device_t slave =
	    {
		.address = deviceRF430CL331HState.i2cAddress,
		.baudRate_kbps = kWarpDefaultI2cBaudRateKbps};

	cmdBuf[0] = deviceRegisterMSB;
	cmdBuf[1] = deviceRegisterLSB;

	warpEnableI2Cpins();
	status = I2C_DRV_MasterReceiveDataBlocking(
	    0 /* I2C peripheral instance */,
	    &slave,
	    cmdBuf,
	    2,
	    (uint8_t *)deviceRF430CL331HState.i2cBuffer,
	    numberOfBytes,
	    kWarpDefaultI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		warpPrint("Communication failed: %d\n", status);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
writeSensorRegisterRF430CL331H(uint8_t deviceRegisterMSB, uint8_t deviceRegisterLSB, uint16_t payload)
{
	uint8_t		payloadByte[2], commandByte[2];
	i2c_status_t	returnValue;

	i2c_device_t slave =
	{
		.address = deviceRF430CL331HState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpEnableI2Cpins();

	commandByte[0] = deviceRegisterMSB;
	commandByte[1] = deviceRegisterLSB;
	payloadByte[0] = (payload>>8) & 0xFF; /* MSB first */
	payloadByte[1] = payload & 0xFF; /* LSB */
	
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							2,
							payloadByte,
							2,
							1000);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

