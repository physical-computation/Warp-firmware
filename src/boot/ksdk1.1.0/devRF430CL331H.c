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


extern volatile WarpI2CDeviceState	deviceRF430CL331HState;
extern volatile uint32_t			gWarpI2cBaudRateKbps;
extern volatile uint32_t			gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t			gWarpSupplySettlingDelayMilliseconds;

void
initRF430CL331H(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceRF430CL331HState.i2cAddress					= i2cAddress;
	deviceRF430CL331HState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterRF430CL331H(uint16_t deviceRegister, uint16_t payload)
{
	uint8_t			payloadByte[2], commandByte[2];
	i2c_status_t	status;

	i2c_device_t slave =
		{
		.address 		= deviceRF430CL331HState.i2cAddress,
		.baudRate_kbps 	= gWarpI2cBaudRateKbps
	};

	commandByte[0] = (uint8_t)((deviceRegister >> 8) & 0xFF); /* MSB first */
	commandByte[1] = (uint8_t)(deviceRegister & 0xFF);        /* LSB */
	payloadByte[0] =(uint8_t) ((payload >> 8) & 0xFF); /* MSB first */
	payloadByte[1] = (uint8_t)(payload & 0xFF);        /* LSB */

	warpScaleSupplyVoltage(deviceRF430CL331HState.operatingVoltageMillivolts);
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
configureSensorRegisterRF430CL331H(uint16_t payloadOP_Mode)
{
	WarpStatus status1, status2;

	warpScaleSupplyVoltage(deviceRF430CL331HState.operatingVoltageMillivolts);
	status1 = writeSensorRegisterRF430CL331H(kWarpSensorConfigureationRegisterRF430CL331H_Genral /* register address PMU_RANGE */,
											 payloadOP_Mode /* payload */
	);
	
	return (status1);
}

WarpStatus
readSensorRegisterRF430CL331H(uint16_t deviceRegister, int numberOfBytes)
{
	uint8_t			cmdBuf[2] = {0xFF};
	i2c_status_t	status;

	i2c_device_t slave =
		{
		.address 		= deviceRF430CL331HState.i2cAddress,
		.baudRate_kbps 	= gWarpI2cBaudRateKbps
	};
	cmdBuf[0] = (uint8_t)((deviceRegister >> 8) & 0xFF); /* MSB first */
	cmdBuf[1] = (uint8_t)(deviceRegister & 0xFF);        /* LSB */
	warpScaleSupplyVoltage(deviceRF430CL331HState.operatingVoltageMillivolts);
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceRF430CL331HState.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}
	return kWarpStatusOK;
}

WarpStatus
StatusRF430CL331H()
{
	WarpStatus status;
	status = readSensorRegisterRF430CL331H(kWarpSensorConfigureationRegisterRF430CL331H_Genral  , 2);
	warpPrint("RF430Status Status = [" BYTE_TO_BINARY_PATTERN "]\n",
							BYTE_TO_BINARY(deviceRF430CL331HState.i2cBuffer[0]));	
	return status;

}