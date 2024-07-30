#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

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


extern volatile WarpI2CDeviceState	deviceBNO055State;
extern volatile WarpI2CDeviceState	deviceBNO055accelState;
extern volatile WarpI2CDeviceState	deviceBNO055gyroState;
extern volatile WarpI2CDeviceState	deviceBNO055magState;
extern volatile uint32_t			gWarpI2cBaudRateKbps;
extern volatile uint32_t			gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t			gWarpSupplySettlingDelayMilliseconds;

void
initBNO055(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceBNO055State.i2cAddress					= i2cAddress;
	deviceBNO055State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterBNO055(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t			payloadByte[1], commandByte[1];
	i2c_status_t	status;

	if (deviceRegister > 0x3F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
		{
		.address 		= deviceBNO055State.i2cAddress,
		.baudRate_kbps 	= gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;

	warpScaleSupplyVoltage(deviceBNO055State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
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
configureSensorRegisterBNO055(uint8_t payloadOP_Mode, uint8_t payloadPWR_Mode)
{
	WarpStatus status1, status2;

	warpScaleSupplyVoltage(deviceBNO055State.operatingVoltageMillivolts);
	status1 = writeSensorRegisterBNO055(kWarpSensorConfigurationRegisterBNO055_OPR_MODE /* register address PMU_RANGE */,
											 payloadOP_Mode /* payload */
	);
	status2 = writeSensorRegisterBNO055(kWarpSensorConfigurationRegisterBNO055_PWR_MODE /* register address PMU_RANGE */,
											 payloadPWR_Mode /* payload */
	);

    OSA_TimeDelay(7);

	
	return (status1 | status2);
}

WarpStatus
readSensorRegisterBNO055(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t			cmdBuf[1] = {0xFF};
	i2c_status_t	status;

	i2c_device_t slave =
		{
		.address 		= deviceBNO055State.i2cAddress,
		.baudRate_kbps 	= gWarpI2cBaudRateKbps
	};
	cmdBuf[0] = deviceRegister;
	warpScaleSupplyVoltage(deviceBNO055State.operatingVoltageMillivolts);
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBNO055State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}
	return kWarpStatusOK;
}

void
printSensorDataBNO055(bool hexModeFlag) {
	int16_t		readSensorRegisterValueLSB;
	int16_t		readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;
	int16_t accX, accY, accZ, magX, magY, magZ, gyrX, gyrY, gyrZ;

    i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_X_LSB, 6);
    
    accX = (int16_t)((deviceBNO055State.i2cBuffer[1] << 8) | deviceBNO055State.i2cBuffer[0]);
    accY = (int16_t)((deviceBNO055State.i2cBuffer[3] << 8) | deviceBNO055State.i2cBuffer[2]);
    accZ = (int16_t)((deviceBNO055State.i2cBuffer[5] << 8) | deviceBNO055State.i2cBuffer[4]);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d, %d, %d,", accX, accY, accZ);
		}
	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_X_LSB, 6);

	magX = (int16_t)((deviceBNO055State.i2cBuffer[1] << 8) | deviceBNO055State.i2cBuffer[0]);
	magY = (int16_t)((deviceBNO055State.i2cBuffer[3] << 8) | deviceBNO055State.i2cBuffer[2]);
	magZ = (int16_t)((deviceBNO055State.i2cBuffer[5] << 8) | deviceBNO055State.i2cBuffer[4]);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,%d, %d,", magX, magY, magZ);
		}
	}

	warpScaleSupplyVoltage(deviceBNO055State.operatingVoltageMillivolts);
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_X_LSB, 6);

	gyrX = (int16_t)((deviceBNO055State.i2cBuffer[1] << 8) | deviceBNO055State.i2cBuffer[0]);
	gyrY = (int16_t)((deviceBNO055State.i2cBuffer[3] << 8) | deviceBNO055State.i2cBuffer[2]);
	gyrZ = (int16_t)((deviceBNO055State.i2cBuffer[5] << 8) | deviceBNO055State.i2cBuffer[4]);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,%d,%d,", gyrX, gyrY, gyrZ);
		}
	}

}

WarpStatus
StateBNO055()
{
	WarpStatus status;
	status = readSensorRegisterBNO055(0x3E  , 1);
	warpPrint("BNO055 PWR_MODE: %x\n", deviceBNO055State.i2cBuffer[0]);
	status = readSensorRegisterBNO055(0x3D  , 1);
	warpPrint("BNO055 OP_MODE: %x\n", deviceBNO055State.i2cBuffer[0]);
	return status;
}

uint8_t
appendSensorDataBNO055(uint8_t* buf)
{
	uint8_t index = 0;

	int16_t		readSensorRegisterValueLSB;
	int16_t		readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	warpScaleSupplyVoltage(deviceBNO055State.operatingVoltageMillivolts);

	/*
	 *	First, trigger a measurement
	 */

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_X_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;
	

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_Y_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_Z_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_X_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_Y_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_Z_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_X_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_Y_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_Z_LSB, 2);
	readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
	readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

	if ((i2cReadStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 24);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 16);
		index += 1;

	}

	/*
	 * total number of bytes written
	 */
	return index;
}