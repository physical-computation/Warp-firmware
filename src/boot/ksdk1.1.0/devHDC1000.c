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


extern volatile WarpI2CDeviceState	deviceHDC1000State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initHDC1000(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskHumidity | kWarpTypeMaskTemperature);

	return;
}

WarpStatus
writeSensorRegisterHDC1000(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	returnValue;

	switch (deviceRegister)
	{
		case 0x02:
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
		.address = deviceHDC1000State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = (payload>>8) & 0xFF; /* MSB first */
	payloadByte[1] = payload & 0xFF; /* LSB */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							2,
							1000);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterHDC1000(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status1, status2;


	i2c_device_t slave =
	{
		.address = deviceHDC1000State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	USED(numberOfBytes);
	if (deviceRegister == 0 || deviceRegister == 1)
	{
		/*
			Steps:
			(1)	Write transaction beginning with start condition, slave address, 
				and pointer address (0x02 == configuration register) and 2
				bytes of config values to be written  (total payload of 3 bytes).

			(2)	Write transaction beginning with start condition, slave address, 
				and pointer address (0x00/0x01 == humidity/temperature). Total
				payload is just 1 byte. (Trigger measurement)

			(3)	Wait 10ms for conversion to complete.

			(4)	Read transaction beginning with start condition, followed by
				slave address, and read 2 byte payload

			Note: to get this to work, had to make the following changes to the default KSDK I2C driver:

			diff KSDK_1.1.0/platform/drivers/src/i2c/fsl_i2c_master_driver.c KSDK_1.1.0/platform/drivers/src/i2c/fsl_i2c_master_driver.c.orig.pip
			617c617
			< //    assert(txBuff);
			---
			>     assert(txBuff);
			657,664d656
			<     //
			<     //  If txSize is zero, don't try to send anything
			<     //
			<     if (txSize == 0)
			<     {
			<             goto skip;
			<     }
			<     
			683d674
			< skip:

			Also added a check to KSDK_1.1.0/platform/hal/src/i2c/fsl_i2c_hal.c so we don't assert whern can't find the device on I2C
		*/

		/*
		 *	Step 1: Trigger temperature/humidity measurement
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

		/*
		 *	Step 2: Wait for max 6.5ms for conversion completion (see Table 7.5 of HDC1000 datasheet)
		 */
		OSA_TimeDelay(7);

		/*
		 *	Step 3: Read temp/humidity
		 */
		status2 = I2C_DRV_MasterReceiveDataBlocking(
								0 /* I2C peripheral instance */,
								&slave,
								NULL,
								0,
								(uint8_t *)deviceHDC1000State.i2cBuffer,
								numberOfBytes,
								gWarpI2cTimeoutMilliseconds);

		if ((status1 != kStatus_I2C_Success) || (status2 != kStatus_I2C_Success))
		{
			return kWarpStatusDeviceCommunicationFailed;
		}
	}
	else
	{
		cmdBuf[0] = deviceRegister;

		status1 = I2C_DRV_MasterReceiveDataBlocking(
								0 /* I2C peripheral instance */,
								&slave,
								cmdBuf,
								1,
								(uint8_t *)deviceHDC1000State.i2cBuffer,
								numberOfBytes,
								gWarpI2cTimeoutMilliseconds);

		if (status1 != kStatus_I2C_Success)
		{
			return kWarpStatusDeviceCommunicationFailed;
		}
	}

	return kWarpStatusOK;
}

void
printSensorDataHDC1000(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	i2cReadStatus = readSensorRegisterHDC1000(kWarpSensorOutputRegisterHDC1000Temperature, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceHDC1000State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceHDC1000State.i2cBuffer[1];
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
			/*
			 *	See Section 8.6.1 of the HDC1000 manual for the conversion to temperature.
			 */
			SEGGER_RTT_printf(0, " %d,", (readSensorRegisterValueCombined*165 / (1u << 16)) - 40);
		}
	}

	i2cReadStatus = readSensorRegisterHDC1000(kWarpSensorOutputRegisterHDC1000Humidity, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceHDC1000State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceHDC1000State.i2cBuffer[1];
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
			/*
			 *	See Section 8.6.2 of the HDC1000 manual for the conversion to temperature.
			 */
			SEGGER_RTT_printf(0, " %d,", (readSensorRegisterValueCombined*100 / (1u << 16)));
		}
	}
}