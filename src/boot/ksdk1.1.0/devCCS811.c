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



extern volatile WarpI2CDeviceState	deviceCCS811State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;



/*
 *	CCS811.
 */
void
initCCS811(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskTotalVOC 		|
						kWarpTypeMaskEquivalentCO2	|
						kWarpTypeMaskHumidity 		|
						kWarpTypeMaskTemperature
					);
	return;
}

WarpStatus
writeSensorRegisterCCS811(uint8_t deviceRegister, uint8_t *payload, uint16_t menuI2cPullupValue)
{
	uint8_t			commandByte[1];
	uint8_t			payloadSize;
	i2c_status_t	returnValue;

	switch (deviceRegister)
	{
		case 0x01:
		{
			payloadSize = 1;
			/* OK */
			break;
		}
		case 0x11:
		{
			payloadSize = 2;
			/* OK */
			break;
		}
		case 0x05: case 0xF1: case 0xFF:
		{
			payloadSize = 4;
			/* OK */
			break;
		}
		case 0x10:
		{
			payloadSize = 5;
			/* OK */
			break;
		}
		case 0xF2:
		{
			payloadSize = 9;
			/* OK */
			break;
		}
		case 0xF3: case 0xF4:
		{
			payloadSize = 0;
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
		.address = deviceCCS811State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	enableI2Cpins(menuI2cPullupValue);

	/*
	 *	Wait for supply and pull-ups to settle.
	 */
	OSA_TimeDelay(100);

	commandByte[0] = deviceRegister;

	if(payloadSize)
	{
		returnValue = I2C_DRV_MasterSendDataBlocking(
								0 /* I2C instance */,
								&slave,
								commandByte,
								1,
								payload,
								payloadSize,
								500);
	}
	else
	{
		returnValue = I2C_DRV_MasterSendDataBlocking(
						0 /* I2C instance */,
						&slave,
						commandByte,
						1,
						NULL,
						0,
						500);
	}

	if (returnValue != kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", returnValue);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
configureSensorCCS811(uint8_t *payloadMEAS_MODE, uint8_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus;

	/*
	 *	See https://narcisaam.github.io/Init_Device/ for more information 
	 *	on how to initialize and configure CCS811
	 */
	i2cWriteStatus = writeSensorRegisterCCS811(kWarpSensorCCS811APP_START /* register address APP_START */,
							payloadMEAS_MODE /* Dummy value */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "CCS811 Write Error, error %d", i2cWriteStatus);
	}

	/*
	 *	Wait for the sensor to change to application mode
	 */
	OSA_TimeDelay(500);

	i2cWriteStatus = writeSensorRegisterCCS811(kWarpSensorCCS811MEAS_MODE /* register address MEAS_MODE */,
							payloadMEAS_MODE /* payload: 3F initial reset */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "CCS811 Write Error, error %d", i2cWriteStatus);
	}

	/*
	 *	After writing to MEAS_MODE to configure the sensor in mode 1-4, 
	 *	run CCS811 for 20 minutes, before accurate readings are generated.
	 */
}

WarpStatus
readSensorRegisterCCS811(uint8_t deviceRegister)
{
	uint8_t 	cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


	if (deviceRegister > 0xFF)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceCCS811State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;


	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceCCS811State.i2cBuffer,
							2,
							500 /* timeout in milliseconds */);

	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d]\n", returnValue);

	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceCCS811State.i2cBuffer[0]);
	}
	else
	{
		//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);
		
		return kWarpStatusDeviceCommunicationFailed;
	}	

	return kWarpStatusOK;
}

void
printSensorDataCCS811(void)
{
	uint8_t readSensorRegisterValueLSB;
	uint8_t readSensorRegisterValueMSB;
	uint16_t readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	i2cReadStatus = readSensorRegisterCCS811(0x03);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "CCS811 Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceCCS811State.i2cBuffer[1];
	readSensorRegisterValueMSB = deviceCCS811State.i2cBuffer[0];
	readSensorRegisterValueCombined =
					((readSensorRegisterValueMSB & 0x03)<<8) + (readSensorRegisterValueLSB & 0xFF);/* Raw ADC value */
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);
}