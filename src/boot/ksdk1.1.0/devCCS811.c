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
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



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
	uint8_t		commandByte[1];
	uint8_t		payloadSize;
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x01:
		{
			payloadSize = 1;
			break;
		}

		case 0x11:
		{
			payloadSize = 2;
			break;
		}

		case 0x05:
		case 0xF1:
		case 0xFF:
		{
			payloadSize = 4;
			break;
		}

		case 0x10:
		{
			payloadSize = 5;
			break;
		}

		case 0xF2:
		{
			payloadSize = 9;
			break;
		}

		case 0xF3:
		case 0xF4:
		{
			payloadSize = 0;
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

	commandByte[0] = deviceRegister;

	if(payloadSize)
	{
		status = I2C_DRV_MasterSendDataBlocking(
								0 /* I2C instance */,
								&slave,
								commandByte,
								1,
								payload,
								payloadSize,
								gWarpI2cTimeoutMilliseconds);
	}
	else
	{
		status = I2C_DRV_MasterSendDataBlocking(
						0 /* I2C instance */,
						&slave,
						commandByte,
						1,
						NULL,
						0,
						gWarpI2cTimeoutMilliseconds);
	}

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorCCS811(uint8_t *payloadMEAS_MODE, uint16_t menuI2cPullupValue)
{
	WarpStatus	status1, status2;

	/*
	 *	See https://narcisaam.github.io/Init_Device/ for more information
	 *	on how to initialize and configure CCS811
	 */

	/*
	 *	Delay needed before start of i2c.
	 */

	OSA_TimeDelay(20);

	status1 = writeSensorRegisterCCS811(kWarpSensorConfigurationRegisterCCS811APP_START /* register address APP_START */,
							payloadMEAS_MODE /* Dummy value */,
							menuI2cPullupValue);

	/*
	 *	Wait for the sensor to change to application mode
	 */
	OSA_TimeDelay(500);

	status2 = writeSensorRegisterCCS811(kWarpSensorConfigurationRegisterCCS811MEAS_MODE /* register address MEAS_MODE */,
							payloadMEAS_MODE /* payload: 3F initial reset */,
							menuI2cPullupValue);

	/*
	 *	After writing to MEAS_MODE to configure the sensor in mode 1-4,
	 *	run CCS811 for 20 minutes, before accurate readings are generated.
	 */

	return (status1 | status2);
}

WarpStatus
readSensorRegisterCCS811(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	returnValue;


	if ((deviceRegister > 0xFF) || (numberOfBytes > kWarpSizesI2cBufferBytes))
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
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataCCS811(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	int16_t		equivalentCO2, TVOC;
	WarpStatus	i2cReadStatus;


	i2cReadStatus	= readSensorRegisterCCS811(kWarpSensorOutputRegisterCCS811ALG_DATA, 4 /* numberOfBytes */);
	equivalentCO2	= (deviceCCS811State.i2cBuffer[0] << 8) | deviceCCS811State.i2cBuffer[1];
	TVOC		= (deviceCCS811State.i2cBuffer[2] << 8) | deviceCCS811State.i2cBuffer[3];
	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----, ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x, 0x%02x 0x%02x,",
				deviceCCS811State.i2cBuffer[3],
				deviceCCS811State.i2cBuffer[2],
				deviceCCS811State.i2cBuffer[1],
				deviceCCS811State.i2cBuffer[0]);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d, %d,", equivalentCO2, TVOC);
		}
	}

	i2cReadStatus = readSensorRegisterCCS811(kWarpSensorOutputRegisterCCS811RAW_DATA, 2 /* numberOfBytes */);
	readSensorRegisterValueLSB = deviceCCS811State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceCCS811State.i2cBuffer[1];

	/*
	 *	RAW ADC value. See CCS811 manual, Figure 15:
	 */
	readSensorRegisterValueCombined =
						((readSensorRegisterValueLSB & 0x03) << 8) |
						(readSensorRegisterValueMSB & 0xFF);
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
	/*
	 *	Get Voltage across V_REF
	 */
	i2cReadStatus = readSensorRegisterCCS811(kWarpSensorOutputRegisterCCS811RAW_REF_NTC, 4 /* numberOfBytes */);
	readSensorRegisterValueLSB = deviceCCS811State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceCCS811State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB);

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
  /*
	 *	Get Voltage across R_NTC
	 */
	readSensorRegisterValueLSB = deviceCCS811State.i2cBuffer[2];
	readSensorRegisterValueMSB = deviceCCS811State.i2cBuffer[3];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB);

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
}
