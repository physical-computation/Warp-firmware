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


extern volatile WarpI2CDeviceState	deviceL3GD20HState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initL3GD20H(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskAngularRateX |
						kWarpTypeMaskAngularRateY |
						kWarpTypeMaskAngularRateZ |
						kWarpTypeMaskTemperature
					);
	return;
}

WarpStatus
writeSensorRegisterL3GD20H(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x20: case 0x21: case 0x22: case 0x23:
		case 0x24: case 0x25: case 0x2E: case 0x30:
		case 0x32: case 0x33: case 0x34: case 0x35:
		case 0x36: case 0x37: case 0x38: case 0x39:
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
		.address = deviceL3GD20HState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorL3GD20H(uint8_t payloadCTRL1, uint8_t payloadCTRL2, uint8_t payloadCTRL5, uint16_t menuI2cPullupValue)
{
	WarpStatus	status1, status2, status3;

	status1 = writeSensorRegisterL3GD20H(kWarpSensorConfigurationRegisterL3GD20HCTRL1 /* register address CTRL1 */,
							payloadCTRL1 /* payload */,
							menuI2cPullupValue);

	status2 = writeSensorRegisterL3GD20H(kWarpSensorConfigurationRegisterL3GD20HCTRL2 /* register address CTRL2 */,
							payloadCTRL2 /* payload */,
							menuI2cPullupValue);

	status3 = writeSensorRegisterL3GD20H(kWarpSensorConfigurationRegisterL3GD20HCTRL5 /* register address CTRL5 */,
							payloadCTRL5 /* payload */,
							menuI2cPullupValue);

	return (status1 | status2 | status3);
}

WarpStatus
readSensorRegisterL3GD20H(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status1, status2;


	USED(numberOfBytes);
	if ((deviceRegister < 0x0F) || (deviceRegister > 0x39))
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceL3GD20HState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;


	/*
	 *	Steps (Repeated single-byte read. See STable 15 and Table 16 of L3GD20H manual.):
	 *
	 *	(1) Write transaction beginning with start condition, slave address, and sub address.
	 *
	 *	(2) Read transaction beginning with start condition, followed by slave address, and read 1 byte payload
	 */

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
							(uint8_t *)deviceL3GD20HState.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if ((status1 != kStatus_I2C_Success) || (status2 != kStatus_I2C_Success))
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


void
printSensorDataL3GD20H(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	int8_t		readSensorRegisterSignedByte;
	WarpStatus	i2cReadStatusLow, i2cReadStatusHigh;


	i2cReadStatusLow = readSensorRegisterL3GD20H(kWarpSensorOutputRegisterL3GD20HOUT_X_L, 1 /* numberOfBytes */);
	readSensorRegisterValueLSB = deviceL3GD20HState.i2cBuffer[0];
	i2cReadStatusHigh = readSensorRegisterL3GD20H(kWarpSensorOutputRegisterL3GD20HOUT_X_H, 1 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceL3GD20HState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if ((i2cReadStatusLow != kWarpStatusOK) || (i2cReadStatusHigh != kWarpStatusOK))
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

	i2cReadStatusLow = readSensorRegisterL3GD20H(kWarpSensorOutputRegisterL3GD20HOUT_Y_L, 1 /* numberOfBytes */);
	readSensorRegisterValueLSB = deviceL3GD20HState.i2cBuffer[0];
	i2cReadStatusHigh = readSensorRegisterL3GD20H(kWarpSensorOutputRegisterL3GD20HOUT_Y_H, 1 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceL3GD20HState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if ((i2cReadStatusLow != kWarpStatusOK) || (i2cReadStatusHigh != kWarpStatusOK))
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

	i2cReadStatusLow = readSensorRegisterL3GD20H(kWarpSensorOutputRegisterL3GD20HOUT_Z_L, 1 /* numberOfBytes */);
	readSensorRegisterValueLSB = deviceL3GD20HState.i2cBuffer[0];
	i2cReadStatusHigh = readSensorRegisterL3GD20H(kWarpSensorOutputRegisterL3GD20HOUT_Z_H, 1 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceL3GD20HState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if ((i2cReadStatusLow != kWarpStatusOK) || (i2cReadStatusHigh != kWarpStatusOK))
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

	i2cReadStatusLow = readSensorRegisterL3GD20H(kWarpSensorOutputRegisterL3GD20HOUT_TEMP, 1 /* numberOfBytes */);
	readSensorRegisterSignedByte = deviceL3GD20HState.i2cBuffer[0];

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int8_t
	 */

	if (i2cReadStatusLow != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x,", deviceL3GD20HState.i2cBuffer[0]);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterSignedByte);
		}
	}
}