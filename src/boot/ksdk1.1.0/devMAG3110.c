/*
	Authored 2016-2018. Phillip Stanley-Marbell, Youchao Wang.

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


extern volatile WarpI2CDeviceState	deviceMAG3110State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initMAG3110(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (
						kWarpTypeMaskMagneticX |
						kWarpTypeMaskMagneticY |
						kWarpTypeMaskMagneticZ |
						kWarpTypeMaskTemperature
					);

	return;
}

WarpStatus
writeSensorRegisterMAG3110(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	returnValue;

	switch (deviceRegister)
	{
		case 0x09: case 0x0A: case 0x0B: case 0x0C:
		case 0x0D: case 0x0E: case 0x10: case 0x11:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMAG3110State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMAG3110(uint8_t payloadCTRL_REG1, uint8_t payloadCTRL_REG2, uint16_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2, i2cWriteStatus3;

	i2cWriteStatus1 = writeSensorRegisterMAG3110(kWarpSensorConfigurationRegisterMAG3110CTRL_REG1 /* register address CTRL_REG1 */,
							payloadCTRL_REG1 /* payload */,
							menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterMAG3110(kWarpSensorConfigurationRegisterMAG3110CTRL_REG2 /* register address CTRL_REG2 */,
							payloadCTRL_REG2 /* payload */,
							menuI2cPullupValue);

	i2cWriteStatus3 = writeSensorRegisterMAG3110(kWarpSensorConfigurationRegisterMAG3110CTRL_REG1 /* register address CTRL_REG1 */,
							0x01 /* payload: ACTIVE mode */,
							menuI2cPullupValue);

	return (i2cWriteStatus1 | i2cWriteStatus2 | i2cWriteStatus3);
}

WarpStatus
readSensorRegisterMAG3110(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1]	= {0xFF};
	i2c_status_t	status1, status2;


	USED(numberOfBytes);
	i2c_device_t slave =
	{
		.address = deviceMAG3110State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	/*
	 *	Steps (Repeated single-byte read. See Section 4.2.2 of MAG3110 manual.):
	 *
	 *	(1) Write transaction beginning with start condition, slave address, and pointer address.
	 *
	 *	(2) Read transaction beginning with start condition, followed by slave address, and read 1 byte payload
	*/

	cmdBuf[0] = deviceRegister;

	status1 = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							NULL,
							0,
							gWarpI2cTimeoutMilliseconds);

	status2 = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMAG3110State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if ((status1 != kStatus_I2C_Success) || (status2 != kStatus_I2C_Success))
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataMAG3110(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	int8_t		readSensorRegisterSignedByte;
	WarpStatus	i2cReadStatus;


	i2cReadStatus = readSensorRegisterMAG3110(kWarpSensorOutputRegisterMAG3110OUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMAG3110State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}


	i2cReadStatus = readSensorRegisterMAG3110(kWarpSensorOutputRegisterMAG3110OUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMAG3110State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}


	i2cReadStatus = readSensorRegisterMAG3110(kWarpSensorOutputRegisterMAG3110OUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMAG3110State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}


	i2cReadStatus = readSensorRegisterMAG3110(kWarpSensorOutputRegisterMAG3110DIE_TEMP, 1 /* numberOfBytes */);
	readSensorRegisterSignedByte = deviceMAG3110State.i2cBuffer[0];

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int8_t
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x,", deviceMAG3110State.i2cBuffer[0]);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterSignedByte);
		}
	}
}