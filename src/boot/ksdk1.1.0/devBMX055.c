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


extern volatile WarpI2CDeviceState	deviceBMX055accelState;
extern volatile WarpI2CDeviceState	deviceBMX055gyroState;
extern volatile WarpI2CDeviceState	deviceBMX055magState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;



/*
 *	Bosch Sensortec BMX055.
 */
void
initBMX055accel(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
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
writeSensorRegisterBMX055accel(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	returnValue;

	if (deviceRegister > 0x3F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBMX055accelState.i2cAddress,
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
							200);
	if (returnValue != kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", returnValue);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
configureSensorBMX055accel(uint8_t payloadPMU_RANGE, uint8_t payloadACCD_HBW, uint8_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus;

	i2cWriteStatus = writeSensorRegisterBMX055accel(kWarpSensorBMX055accelPMU_RANGE /* register address PMU_RANGE */,
							payloadPMU_RANGE /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterBMX055accel(kWarpSensorBMX055accelACCD_HBW /* register address ACCD_HBW */,
							payloadACCD_HBW /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Write Error, error %d", i2cWriteStatus);
	}
}

WarpStatus
readSensorRegisterBMX055accel(uint8_t deviceRegister)
{
	uint8_t 	cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


	if (deviceRegister > 0x3F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBMX055accelState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;


	/*
	 *	Setup the approapriate regulators.
	 *
	 *	TODO: this should later on be set based on the require precision/noise/accuracy/etc.
	 */
	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBMX055accelState.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d]\n", returnValue);

	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceBMX055accelState.i2cBuffer[0]);
	}
	else
	{
		//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);
		
		return kWarpStatusDeviceCommunicationFailed;
	}	


	return kWarpStatusOK;
}

void
initBMX055mag(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskMagneticX |
						kWarpTypeMaskMagneticY |
						kWarpTypeMaskMagneticZ |
						kWarpTypeMaskTemperature
					);

	return;
}

WarpStatus
writeSensorRegisterBMX055mag(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	returnValue;

	if (deviceRegister > 0x52 || deviceRegister < 0x40)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBMX055magState.i2cAddress,
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
							200);
	if (returnValue != kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", returnValue);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
configureSensorBMX055mag(uint8_t payloadPowerCtrl, uint8_t payloadOpMode, uint8_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus;

	i2cWriteStatus = writeSensorRegisterBMX055mag(
							kWarpSensorBMX055magPowerCtrl /* Power and operation modes, self-test, data output rate control registers */,
							payloadPowerCtrl /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterBMX055mag(
							kWarpSensorBMX055magOpMode /* Operation mode, output data rate and self-test control register */,
							payloadOpMode /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Write Error, error %d", i2cWriteStatus);
	}
}

WarpStatus
readSensorRegisterBMX055mag(uint8_t deviceRegister)
{
	uint8_t 	cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


	if (deviceRegister > 0x52 || deviceRegister < 0x40)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBMX055magState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;


	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBMX055magState.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);


	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d]\n", returnValue);

	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceBMX055magState.i2cBuffer[0]);
	}
	else
	{
		//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
initBMX055gyro(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
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
writeSensorRegisterBMX055gyro(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	returnValue;

	if (deviceRegister > 0x3F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBMX055magState.i2cAddress,
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
							200);
	if (returnValue != kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", returnValue);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
configureSensorBMX055gyro(uint8_t payloadRANGE, uint8_t payloadBW, uint8_t payloadLPM1, uint8_t payloadRATE_HBW, uint8_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus;
	i2cWriteStatus = writeSensorRegisterBMX055gyro(kWarpSensorBMX055gyroRANGE /* register address RANGE */,
							payloadRANGE /* payload */, 
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterBMX055gyro(kWarpSensorBMX055gyroBW/* register address filter bandwidth */,
							payloadBW /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterBMX055gyro(kWarpSensorBMX055gyroLPM1/* register address LPM1 */,
							payloadLPM1 /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Write Error, error %d", i2cWriteStatus);
	}

	i2cWriteStatus = writeSensorRegisterBMX055gyro(kWarpSensorBMX055gyroRATE_HBW/* register address RATE_HBW */,
							payloadLPM1 /* payload */,
							menuI2cPullupValue);
	if (i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Write Error, error %d", i2cWriteStatus);
	}
}

WarpStatus
readSensorRegisterBMX055gyro(uint8_t deviceRegister)
{
	uint8_t 	cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


	if (deviceRegister > 0x3F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBMX055gyroState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBMX055gyroState.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d]\n", returnValue);

	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceBMX055gyroState.i2cBuffer[0]);
	}
	else
	{
		//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataBMX055accel(void)
{
	uint8_t readSensorRegisterValueLSB;
	uint8_t readSensorRegisterValueMSB;
	uint16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterBMX055accel(kWarpSensorBMX055accelACCD_X_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055accelState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055accel(kWarpSensorBMX055accelACCD_X_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055accelState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBMX055accel(kWarpSensorBMX055accelACCD_Y_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055accelState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055accel(kWarpSensorBMX055accelACCD_Y_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055accelState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBMX055accel(kWarpSensorBMX055accelACCD_Z_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055accelState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055accel(kWarpSensorBMX055accelACCD_Z_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055accelState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBMX055accel(kWarpSensorBMX055accelACCD_TEMP);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055accel Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055accelState.i2cBuffer[0];
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueMSB);
}

void
printSensorDataBMX055gyro(void)
{
	uint8_t readSensorRegisterValueLSB;
	uint8_t readSensorRegisterValueMSB;
	uint16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterBMX055gyro(kWarpSensorBMX055gyroRATE_X_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055gyroState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055gyro(kWarpSensorBMX055gyroRATE_X_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055gyroState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBMX055gyro(kWarpSensorBMX055gyroRATE_Y_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055gyroState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055gyro(kWarpSensorBMX055gyroRATE_Y_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055gyroState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBMX055gyro(kWarpSensorBMX055gyroRATE_Z_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055gyroState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055gyro(kWarpSensorBMX055gyroRATE_Z_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055gyro Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055gyroState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);
}

void
printSensorDataBMX055mag(void)
{
	uint8_t readSensorRegisterValueLSB;
	uint8_t readSensorRegisterValueMSB;
	uint16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterBMX055mag(kWarpSensorBMX055magX_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055magState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055mag(kWarpSensorBMX055magX_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055magState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBMX055mag(kWarpSensorBMX055magY_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055magState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055mag(kWarpSensorBMX055magY_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055magState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBMX055mag(kWarpSensorBMX055magZ_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055magState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055mag(kWarpSensorBMX055magZ_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055magState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);

	i2cReadStatus = readSensorRegisterBMX055mag(kWarpSensorBMX055magRHALL_LSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueLSB = deviceBMX055magState.i2cBuffer[0];
	i2cReadStatus = readSensorRegisterBMX055mag(kWarpSensorBMX055magRHALL_MSB);
	if(i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "BMX055mag Read Error, error %d", i2cReadStatus);
	}
	readSensorRegisterValueMSB = deviceBMX055magState.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);
}