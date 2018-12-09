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
	i2c_status_t	returnValue;

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

	enableI2Cpins(menuI2cPullupValue);

	/*
	 *	Wait for supply and pull-ups to settle.
	 */
	OSA_TimeDelay(100);

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							1000);
	if (returnValue != kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", returnValue);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
configureSensorL3GD20H(uint8_t payloadCTRL1, uint8_t payloadCTRL2, uint8_t payloadCTRL5, uint8_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus;
	i2cWriteStatus = writeSensorRegisterL3GD20H(kWarpSensorL3GD20HCTRL1 /* register address CTRL1 */,
							payloadCTRL1 /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterL3GD20H(kWarpSensorL3GD20HCTRL2 /* register address CTRL2 */,
							payloadCTRL2 /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterL3GD20H(kWarpSensorL3GD20HCTRL5 /* register address CTRL5 */,
							payloadCTRL5 /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Write Error, error %d", i2cWriteStatus);
	}
}

WarpStatus
readSensorRegisterL3GD20H(uint8_t deviceRegister)
{
	uint8_t 	cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


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

	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							NULL,
							0,
							500 /* timeout in milliseconds */);

	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterSendDataBlocking returned [%d] (set pointer)\n", returnValue);
		
	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceL3GD20HState.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d] (read register)\n", returnValue);
	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d] (retrieve measurement)\n", returnValue);

	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceL3GD20HState.i2cBuffer[0]);
	}
	else
	{
		//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataL3GD20H(void)
{
	uint8_t readSensorRegisterValueLSB;
	uint8_t readSensorRegisterValueMSB;
	uint16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterL3GD20H(kWarpSensorL3GD20HOUT_X_L);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceL3GD20HState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterL3GD20H(kWarpSensorL3GD20HOUT_X_H);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceL3GD20HState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterL3GD20H(kWarpSensorL3GD20HOUT_Y_L);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceL3GD20HState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterL3GD20H(kWarpSensorL3GD20HOUT_Y_H);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceL3GD20HState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);
	
	i2cReadStatus = readSensorRegisterL3GD20H(kWarpSensorL3GD20HOUT_Z_L);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceL3GD20HState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterL3GD20H(kWarpSensorL3GD20HOUT_Z_H);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceL3GD20HState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterL3GD20H(kWarpSensorL3GD20HOUT_TEMP);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "L3GD20H Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceL3GD20HState.i2cBuffer[0];
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueMSB);
}