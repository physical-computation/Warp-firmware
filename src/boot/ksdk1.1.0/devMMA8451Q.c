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


extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;



void
initMMA8451Q(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ |
						kWarpTypeMaskTemperature
					);
	return;
}

WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister)
{
	uint8_t cmdBuf[1]	= {0xFF};
	i2c_status_t		returnValue;


	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
		{
			/* OK */
			break;
		}
		
		default:
		{
			//SEGGER_RTT_printf(0, "\rreadSensorRegisterMMA8451Q() received bad deviceRegister\n");
	
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMMA8451QState.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);
	
	//SEGGER_RTT_printf(0, "\nI2C_DRV_MasterReceiveData returned [%d] (read register)\n", returnValue);

	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceMMA8451QState.i2cBuffer[0]);
	}
	else
	{
		//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}
