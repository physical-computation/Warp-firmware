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


extern volatile WarpI2CDeviceState	deviceHDC1000State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;



void
initHDC1000(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskHumidity | kWarpTypeMaskTemperature);

	return;
}

WarpStatus
readSensorRegisterHDC1000(uint8_t deviceRegister)
{
	uint8_t cmdBuf[1]	= {0xFF};
	uint8_t txBuf[2]	= {0xFF, 0xFF};
	i2c_status_t		returnValue;


	i2c_device_t slave =
	{
		.address = deviceHDC1000State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	//SEGGER_RTT_printf(0, "\rreadSensorRegisterHDC1000() got deviceRegister [0x%02x]\n", deviceRegister);

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
		 *	Step 1: Configure. Configuration data (0x00 0x00) in txBuf
		 */
		txBuf[0] = 0x00;
		txBuf[1] = 0x00;
		cmdBuf[0] = 0x02;

		returnValue = I2C_DRV_MasterSendDataBlocking(
								0 /* I2C peripheral instance */,
								&slave,
								cmdBuf,
								1,
								txBuf,
								2,
								100 /* timeout in milliseconds */);

		//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterSendData returned [%d] (ptr+config)\n", returnValue);

		/*
		 *	Step 2: Trigger temperature/humidity measurement
		 */
		cmdBuf[0] = deviceRegister;

		//SEGGER_RTT_printf(0, "\rBefore I2C_DRV_MasterSendData...\n");

		returnValue = I2C_DRV_MasterSendDataBlocking(
								0 /* I2C peripheral instance */,
								&slave,
								cmdBuf,
								1,
								NULL,
								0,
								100 /* timeout in milliseconds */);
		
		//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterSendData returned [%d] (ptr write)\n", returnValue);

		/*
		 * Step 3: Wait for conversion
		 */
		OSA_TimeDelay(100);


		/*
		 *	Step 4: Read temp/humidity
		 */
		returnValue = I2C_DRV_MasterReceiveDataBlocking(
								0 /* I2C peripheral instance */,
								&slave,
								NULL,
								0,
								(uint8_t *)deviceHDC1000State.i2cBuffer,
								2,
								500 /* timeout in milliseconds */);

		//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d] (retrieve measurement)\n", returnValue);

		if (returnValue == kStatus_I2C_Success)
		{
			//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x 0x%02x\n", cmdBuf[0], deviceHDC1000State.i2cBuffer[0], deviceHDC1000State.i2cBuffer[1]);
		}
		else
		{
			//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

			return kWarpStatusDeviceCommunicationFailed;
		}
	}
	else
	{
		cmdBuf[0] = deviceRegister;

		returnValue = I2C_DRV_MasterReceiveDataBlocking(
								0 /* I2C peripheral instance */,
								&slave,
								cmdBuf,
								1,
								(uint8_t *)deviceHDC1000State.i2cBuffer,
								2,
								500 /* timeout in milliseconds */);

		//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d] (retrieve measurement)\n", returnValue);

		if (returnValue == kStatus_I2C_Success)
		{
			//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x 0x%02x\n", cmdBuf[0], deviceHDC1000State.i2cBuffer[0], deviceHDC1000State.i2cBuffer[1]);
		}
		else
		{
			//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

			return kWarpStatusDeviceCommunicationFailed;
		}
	}

	return kWarpStatusOK;
}
