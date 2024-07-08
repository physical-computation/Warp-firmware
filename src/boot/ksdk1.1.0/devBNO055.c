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
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

void
initBNO055(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceBNO055State.i2cAddress			= i2cAddress;
	deviceBNO055State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}
WarpStatus
writeSensorRegisterBNO055(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;
	switch (deviceRegister)
	{
        case 0x38: case 0x3B: case 0x3D: case 0x3E:
        case 0x3F: case 0x40: case 0x41: case 0x42:
        case 0x55: case 0x56: case 0x57: case 0x58:
        case 0x59: case 0x5A: case 0x5B: case 0x5C:
        case 0x5D: case 0x5E: case 0x5F: case 0x60:
        case 0x61: case 0x62: case 0x63: case 0x64:
        case 0x65: case 0x66: case 0x67: case 0x68:
        case 0x69: case 0x6A:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	if (deviceRegister > 0x3F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
		{
		.address = deviceBNO055State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
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
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
        case 0x00: case 0x01: case 0x02: case 0x03:
        case 0x04: case 0x05: case 0x06: case 0x07:
        case 0x08: case 0x09: case 0x0A: case 0x0B:
        case 0x0C: case 0x0D: case 0x0E: case 0x0F:
        case 0x10: case 0x11: case 0x12: case 0x13:
        case 0x14: case 0x15: case 0x16: case 0x17:
        case 0x18: case 0x19: case 0x1A: case 0x1B:
        case 0x1C: case 0x1D: case 0x1E: case 0x1F:
        case 0x20: case 0x21: case 0x22: case 0x23:
        case 0x24: case 0x25: case 0x26: case 0x27:
        case 0x28: case 0x29: case 0x2A: case 0x2B:
        case 0x2C: case 0x2D: case 0x2E: case 0x2F:
        case 0x30: case 0x31: case 0x32: case 0x33:
        case 0x34: case 0x35: case 0x36: case 0x37:
        case 0x38: case 0x39: case 0x3A: case 0x3B:
        case 0x3C: case 0x3D: case 0x3E: case 0x3F:
        case 0x40: case 0x41: case 0x42: case 0x43:
        case 0x44: case 0x45: case 0x46: case 0x47:
        case 0x48: case 0x49: case 0x4A: case 0x4B:
        case 0x4C: case 0x4D: case 0x4E: case 0x4F:
        case 0x50: case 0x51: case 0x52: case 0x53:
        case 0x54: case 0x55: case 0x56: case 0x57:
        case 0x58: case 0x59: case 0x5A: case 0x5B:
        case 0x5C: case 0x5D: case 0x5E: case 0x5F:
        case 0x60: case 0x61: case 0x62: case 0x63:
        case 0x64: case 0x65: case 0x66: case 0x67:
        case 0x68: case 0x69: case 0x6A: case 0x6B:
        case 0x6C: case 0x6D: case 0x6E: case 0x6F:
        case 0x70: case 0x71: case 0x72: case 0x73:
        case 0x74: case 0x75: case 0x76: case 0x77:
        case 0x78: case 0x79: case 0x7A: case 0x7B:
        case 0x7C: case 0x7D: case 0x7E: case 0x7F:
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
		.address = deviceBNO055State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
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
    int16_t	readSensorRegisterValueLSB;
	int16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

    i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_X_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

    i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_Y_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

    i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_Z_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_X_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_Y_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_Z_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_X_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_Y_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_Z_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

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
			warpPrint(" %d,", readSensorRegisterValueCombined);
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

	int16_t	readSensorRegisterValueLSB;
	int16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus, triggerStatus;

	warpScaleSupplyVoltage(deviceBNO055State.operatingVoltageMillivolts);

	/*
	 *	First, trigger a measurement
	 */
	triggerStatus               = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBNO055_OPR_MODE,
															0x3D);

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_X_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;
	

	if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_Y_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

    if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

    i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Accel_Data_Z_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

    if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_X_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

    if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_Y_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

    if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Mag_Data_Z_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

    if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_X_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

    if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_Y_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

    if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	i2cReadStatus = readSensorRegisterBNO055(kWarpSensourOutputRegisterBNO055Gyro_Data_Z_LSB, 2);
    readSensorRegisterValueLSB = deviceBNO055State.i2cBuffer[0];
    readSensorRegisterValueMSB = deviceBNO055State.i2cBuffer[1];
    readSensorRegisterValueCombined = (readSensorRegisterValueMSB << 8) | readSensorRegisterValueLSB;

    if ((i2cReadStatus != kWarpStatusOK) || (triggerStatus != kWarpStatusOK))
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;

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

		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	/*
	 * total number of bytes written
	 */
	return index;
}