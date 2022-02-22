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
	/*
	 *	The sensor has only 3 real registers: STATUS Register 0x00, WRITE Register 0x01 and READ register 0x02.
	 */
	uint8_t		cmdBuf_write[2]		= {deviceRegisterMSB, deviceRegisterLSB};
	uint8_t		cmdBuf_read[2]		= {0xFF, 0xFF};
	i2c_status_t	returnValue;

	USED(numberOfBytes);

	i2c_device_t slave =
	{ 
		.address = 0b00011111,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpEnableI2Cpins();

	/*
	 *	See Page 8 to Page 11 of AS726X Design Considerations for writing to and reading from virtual registers.
	 *	Write transaction writes the value of the virtual register one wants to read from to the WRITE register 0x01.
	 */

	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_write /* The pointer to the commands to be transferred */,
							2 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}
	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							NULL /* The pointer to the commands to be transferred */,
							0 /* The length in bytes of the commands to be transferred */,
							(uint8_t *)deviceRF430CL331HState.i2cBuffer /* The pointer to the data to be transferred */,
							numberOfBytes /* The length in bytes of the data to be transferred and data is transferred from the sensor to master via bus */,
							gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
writeSensorRegisterHRF430CL331H(uint8_t deviceRegisterMSB, uint8_t deviceRegisterLSB, uint16_t payload)
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

