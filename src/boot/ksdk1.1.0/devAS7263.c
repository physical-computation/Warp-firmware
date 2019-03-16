/*
	Authored 2018. Rae Zhao.

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
#include "devAS726x.h"


extern volatile WarpI2CDeviceState	deviceAS7263State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;



void
initAS7263(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (
						kWarpTypeMaskTemperature|
						kWarpTypeMaskLambda610R |
						kWarpTypeMaskLambda680S |
						kWarpTypeMaskLambda730T |
						kWarpTypeMaskLambda760U |
						kWarpTypeMaskLambda810V |
						kWarpTypeMaskLambda860W
					);
	return;
}

WarpStatus
readSensorRegisterAS7263(uint8_t deviceRegister, int numberOfBytes)
{
	/*
	 *	The sensor has only 3 real registers: STATUS Register 0x00, WRITE Register 0x01 and READ register 0x02.
	 */
	uint8_t		cmdBuf_write[2]		= {kWarpI2C_AS726x_SLAVE_WRITE_REG, 0xFF};
	uint8_t		cmdBuf_LEDCTRL[2]	= {kWarpI2C_AS726x_SLAVE_WRITE_REG, 0x87};
	uint8_t		cmdBuf_LEDON[2]		= {kWarpI2C_AS726x_SLAVE_WRITE_REG, 0x1B};
	uint8_t		cmdBuf_LEDOFF[2]	= {kWarpI2C_AS726x_SLAVE_WRITE_REG, 0x00};
	uint8_t		cmdBuf_read[1]		= {kWarpI2C_AS726x_SLAVE_READ_REG};
	i2c_status_t	returnValue;


	USED(numberOfBytes);
	if (deviceRegister > 0x2B)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{ 
		.address = deviceAS7263State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf_write[1] = deviceRegister;


	/*
	 *	The LED control register details can be found in Figure 27 of AS7263 detailed descriptions on page 24.
	 */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_LEDCTRL /* The pointer to the commands to be transferred */,
							2 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}



	/*
	 *	This turns on the LED before reading the data
	 */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_LEDON /* The pointer to the commands to be transferred */,
							2 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}



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



	/*
	 *	Read transaction which reads from the READ register 0x02.
	 *	The read transaction requires one to first write to the register address one wants to focus on and then read from that address.
	 */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_read /* The pointer to the commands to be transferred */,
							1 /* The length in bytes of the commands to be transferred */,
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
							cmdBuf_read /* The pointer to the commands to be transferred */,
							1 /* The length in bytes of the commands to be transferred */,
							(uint8_t *)deviceAS7263State.i2cBuffer /* The pointer to the data to be transferred */,
							1 /* The length in bytes of the data to be transferred and data is transferred from the sensor to master via bus */,
							gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}



	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_LEDCTRL /* The pointer to the commands to be transferred */,
							2 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}



	/*
	 *	This turns off the LED after finish reading the data
	 */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_LEDOFF /* The pointer to the commands to be transferred */,
							2 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}
