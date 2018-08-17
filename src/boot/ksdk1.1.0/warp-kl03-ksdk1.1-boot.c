/*
	Authored 2016. Phillip Stanley-Marbell.

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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



#define					kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define					kWarpConstantStringInvalidVoltage	"\r\n\n\nInvalid supply voltage [%d] mV\n\n\n"

enum {
	kWarpI2C_AS726x_SLAVE_WRITE_REG	= 0x01,
	kWarpI2C_AS726x_SLAVE_READ_REG	= 0x02
};

volatile WarpSPIDeviceState		deviceADXL362State;
volatile WarpI2CDeviceState		deviceBMX055accelState;
volatile WarpI2CDeviceState		deviceBMX055gyroState;
volatile WarpI2CDeviceState		deviceBMX055magState;
volatile WarpI2CDeviceState		deviceMMA8451QState;
volatile WarpI2CDeviceState		deviceMAG3110State;
volatile WarpI2CDeviceState		deviceL3GD20HState;
volatile WarpI2CDeviceState		deviceBMP180State;
volatile WarpI2CDeviceState		deviceTMP006BState;
volatile WarpUARTDeviceState		devicePAN1326BState;
volatile WarpI2CDeviceState		deviceAS7262State;
volatile WarpI2CDeviceState		deviceAS7263State;
volatile WarpI2CDeviceState		deviceSCD30State;


/*
 *	Initialization: Devices hanging off I2C
 */
void					initBMX055accel(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initBMX055gyro(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initBMX055mag(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initMMA8451Q(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initMAG3110(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initL3GD20H(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initBMP180(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initTMP006B(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initAS7262(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initAS7263(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
void					initSCD30(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

/*
 *	Initialization: Devices hanging off SPI
 */
void					initADXL362(WarpSPIDeviceState volatile *  deviceStatePointer);

/*
 *	Initialization: Devices hanging off UART
 */
void					initPAN1326B(WarpUARTDeviceState volatile *  deviceStatePointer);


/*
 *	For devices that need to be explicitly put into sleep/low-power mode after POR
 */
void					sleepBMX055accel(void);
void					sleepBMX055gyro(void);
void					sleepBMX055mag(void);
void					sleepMMA8451Q(void);
void					sleepMAG3110(void);
void					sleepL3GD20H(void);
void					sleepBMP180(void);
void					sleepTMP006B(void);
void					sleepADXL362(void);
void					sleepPAN1326B(void);
void					sleepAS7262(void);
void					sleepAS7263(void);
void					sleepSCD30(void);


/*
 *	Reading from sensor registers
 */
WarpStatus				readSensorRegisterADXL362(uint8_t deviceRegister);
WarpStatus				readSensorRegisterBMX055accel(uint8_t deviceRegister);
WarpStatus				readSensorRegisterBMX055gyro(uint8_t deviceRegister);
WarpStatus				readSensorRegisterBMX055mag(uint8_t deviceRegister);
WarpStatus				readSensorRegisterMMA8451Q(uint8_t deviceRegister);
WarpStatus				readSensorRegisterMAG3110(uint8_t deviceRegister);
WarpStatus				readSensorRegisterL3GD20H(uint8_t deviceRegister);
WarpStatus				readSensorRegisterBMP180(uint8_t deviceRegister);
WarpStatus				readSensorRegisterTMP006B(uint8_t deviceRegister);
WarpStatus				readSensorRegisterAS7262(uint8_t deviceRegister);
WarpStatus				readSensorRegisterAS7263(uint8_t deviceRegister);
WarpStatus				readSensorRegisterSCD30(uint8_t deviceRegister);

WarpStatus				writeSensorRegisterADXL362(uint8_t command, uint8_t deviceRegister, uint8_t writeValue);

void					disableSWDEnablePTA1x2x3xGpio(void);
void					enableSWDDisablePTA1x2x3xGpio(void);
void					brieflyToggleEnablingSWD(void);
void					lowPowerPinStates(void);
void					disableTPS82675(void);
void					enableTPS82675(bool mode);
void					disableTPS82740A(void);
void					enableTPS82740A(uint16_t voltageMillivolts);
void					printPinDirections(void);
void					enableI2Cpins(bool pullupEnable);
void					disableI2Cpins(void);
void					enableSPIpins(bool driveI2cPinsHighToMatchSupply);
void					disableSPIpins(bool driveI2cPinsLow);
void					dumpProcessorStateWithSwdToggles(void);
void					repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, 
								bool pullupEnable, bool autoIncrement, int chunkReadsPerAddress, bool chatty,
								int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
								uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);

int					char2int(int character);
void					enableSssupply(uint16_t voltageMillivolts);
void					disableSssupply(void);
void					activateAllLowPowerSensorModes(void);
void					powerupAllSensors(void);
uint8_t					readHexByte(void);
int					read4digits(void);

WarpStatus				writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus				writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength, bool driveI2cPinsHighToMatchSupply, bool driveI2cPinsLow);
void					warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);



/*
 *	TODO: move this and possibly others into a structure
 */
volatile i2c_master_state_t		i2cMasterState;
volatile spi_master_state_t		spiMasterState;
volatile spi_master_user_config_t	spiUserConfig;


/*
 *	TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t			gWarpI2cBaudRateKbps	= 1;
volatile uint32_t			gWarpUartBaudRateKbps	= 1;
volatile uint32_t			gWarpSpiBaudRateKbps	= 1;
volatile uint32_t			gWarpSleeptimeSeconds	= 0;




/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *  notify,
					power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}

/*
 *	From KSDK power_manager_demo.c <<END>>>
 */








/*
 *	Analog Devices ADXL362.
 *
 *	From device manual, Rev. B, Page 19 of 44:
 *
 *		"
 *		The SPI port uses a multibyte structure 
 *		wherein the first byte is a command. The 
 *		ADXL362 command set is:
 *
 *		-	0x0A: write register
 *		-	0x0B: read register
 *		-	0x0D: read FIFO
 *		"
 */
void
initADXL362(WarpSPIDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->signalType	= (	kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ |
						kWarpTypeMaskTemperature
					);
	return;
}

WarpStatus
readSensorRegisterADXL362(uint8_t deviceRegister)
{	
	return writeSensorRegisterADXL362(0x0B /* command == read register */, deviceRegister, 0x00 /* writeValue */);
}

WarpStatus
writeSensorRegisterADXL362(uint8_t command, uint8_t deviceRegister, uint8_t writeValue)
{	
	/*
	 *	Populate the shift-out register with the read-register command,
	 *	followed by the register to be read, followed by a zero byte.
	 */
	deviceADXL362State.spiSourceBuffer[0] = command;
	deviceADXL362State.spiSourceBuffer[1] = deviceRegister;
	deviceADXL362State.spiSourceBuffer[2] = writeValue;

	deviceADXL362State.spiSinkBuffer[0] = 0x00;
	deviceADXL362State.spiSinkBuffer[1] = 0x00;
	deviceADXL362State.spiSinkBuffer[2] = 0x00;

	/*
	 *	First, create a falling edge on chip-select.
	 *
	 *	NOTE: we keep the kWarpPinADXL362_CS_PAN1326_nSHUTD
	 *	low at all times so that PAN1326 is shut down.
	 *
	 */
	GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS_PAN1326_nSHUTD);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kWarpPinADXL362_CS_PAN1326_nSHUTD);


	/*
	 *	The result of the SPI transaction will be stored in deviceADXL362State.spiSinkBuffer.
	 *
	 *	Providing a device structure here is optional since it 
	 *	is already provided when we did SPI_DRV_MasterConfigureBus(),
	 *	so we pass in NULL.
	 *
	 *	TODO: the "master instance" is always 0 for the KL03 since
	 *	there is only one SPI peripheral. We however should remove
	 *	the '0' magic number and place this in a Warp-HWREV0 header
	 *	file.
	 */
	enableSPIpins(true /* driveI2cPinsHighToMatchSupply */);
	deviceADXL362State.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceADXL362State.spiSourceBuffer,
					(uint8_t * restrict)deviceADXL362State.spiSinkBuffer,
					3 /* transfer size */,
					1000 /* timeout in microseconds (unlike I2C which is ms) */);
	disableSPIpins(true /* driveI2cPinsLow */);

	/*
	 *	NOTE: we leave the kWarpPinADXL362_CS_PAN1326_nSHUTD
	 *	low at all times so that PAN1326 is shut down.
	 */

	return kWarpStatusOK;
}



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

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBMX055accelState.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
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

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
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

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



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

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



void
initMAG3110(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (
						kWarpTypeMaskMagneticX |
						kWarpTypeMaskMagneticY |
						kWarpTypeMaskMagneticZ |
						kWarpTypeMaskTemperature
					);

	return;
}

WarpStatus
readSensorRegisterMAG3110(uint8_t deviceRegister)
{
	uint8_t		cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


	i2c_device_t slave =
	{
		.address = deviceMAG3110State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	/*
	 *	Steps (Repeated single-byte read. See Section 4.2.2 of MAG3110 manual.):
	 *
	 *	(1) Write transaction beginning with start condition, slave address, and pointer address.
	 *
	 *	(2) Read transaction beginning with start condition, followed by slave address, and read 1 byte payload
	*/

	cmdBuf[0] = deviceRegister;

	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							NULL,
							0,
							500 /* timeout in milliseconds */);

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMAG3110State.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



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

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceL3GD20HState.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



void
initBMP180(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskPressure | kWarpTypeMaskTemperature);

	return;
}

WarpStatus
readSensorRegisterBMP180(uint8_t deviceRegister)
{
	uint8_t 	cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


	switch (deviceRegister)
	{
		case 0xAA: case 0xAB: case 0xAC: case 0xAD: case 0xAE: case 0xAF:
		case 0xB0: case 0xB1: case 0xB2: case 0xB3: case 0xB4: case 0xB5:
		case 0xB6: case 0xB7: case 0xB8: case 0xB9: case 0xBA: case 0xBB:
		case 0xBC: case 0xBD: case 0xBE: case 0xBF:
		case 0xD0: case 0xE0: case 0xF4: case 0xF6: case 0xF7: case 0xF8:
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
		.address = deviceBMP180State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;


	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBMP180State.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



void
initTMP006B(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress 	= i2cAddress;
	deviceStatePointer->signalType	= kWarpTypeMaskTemperature;

	return;
}

WarpStatus
readSensorRegisterTMP006B(uint8_t deviceRegister)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	returnValue;

	i2c_device_t slave =
	{
		.address = deviceTMP006BState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceTMP006BState.i2cBuffer,
							2,
							500 /* timeout in milliseconds */);

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}


	return kWarpStatusOK;
}


void
initPAN1326B(WarpUARTDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->signalType	= kWarpTypeMaskTemperature;

	/*
	 *	Shutdown the Module
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinADXL362_CS_PAN1326_nSHUTD);
}


void
initAS7262(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskLambda450V |
						kWarpTypeMaskLambda500B |
						kWarpTypeMaskLambda550G |
						kWarpTypeMaskLambda570Y |
						kWarpTypeMaskLambda600O |
						kWarpTypeMaskLambda650R
					);
	return;
}

WarpStatus
readSensorRegisterAS7262(uint8_t deviceRegister)
{
	/*
	 *	The sensor has only 3 real registers: STATUS Register 0x00, WRITE Register 0x01 and READ register 0x02.
	 */
	uint8_t 	cmdBuf_write[2]	= {kWarpI2C_AS726x_SLAVE_WRITE_REG,0xFF}; 
	uint8_t 	cmdBuf_read[1]	= {kWarpI2C_AS726x_SLAVE_READ_REG};
	i2c_status_t	returnValue;


	if (deviceRegister > 0x2B)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{ 
		.address = deviceAS7262State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf_write[1] = deviceRegister;
	
	/*
	 *	See Page 8 to Page 11 of AS726X Design Considerations for writing to and reading from virtual registers.
	 *	Write transaction writes the value of the virtual register one wants to read from to the WRITE register 0x01.
	 */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_write /* The pointer to the commands to be transferred */,
							2 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							500 /* timeout in milliseconds */);		

	/*
	 *	Read transaction which reads from the READ register 0x02.
	 *	The read transaction requires one to first write to the register address one wants to focus on and then read from that address.
	 */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_read /* The pointer to the commands to be transferred */,
							1 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							500 /* timeout in milliseconds */);			

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_read /* The pointer to the commands to be transferred */,
							1 /* The length in bytes of the commands to be transferred */,
							(uint8_t *)deviceAS7262State.i2cBuffer /* The pointer to the data to be transferred */,
							1 /* The length in bytes of the data to be transferred and data is transferred from the sensor to master via bus */,
							500 /* timeout in milliseconds */);

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
initAS7263(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskLambda610R |
						kWarpTypeMaskLambda680S |
						kWarpTypeMaskLambda730T |
						kWarpTypeMaskLambda760U |
						kWarpTypeMaskLambda810V |
						kWarpTypeMaskLambda860W
					);
	return;
}

WarpStatus
readSensorRegisterAS7263(uint8_t deviceRegister)
{
	/*
	 *	The sensor has only 3 real registers: STATUS Register 0x00, WRITE Register 0x01 and READ register 0x02.
	 */
	uint8_t 	cmdBuf_write[2]	= {kWarpI2C_AS726x_SLAVE_WRITE_REG,0xFF}; 
	uint8_t 	cmdBuf_read[1]	= {kWarpI2C_AS726x_SLAVE_READ_REG};
	i2c_status_t	returnValue;


	if (deviceRegister > 0x2B)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{ 
		.address = deviceAS7263State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf_write[1] = deviceRegister;
	
	/*
	 *	See Page 8 to Page 11 of AS726X Design Considerations for writing to and reading from virtual registers.
	 *	Write transaction writes the value of the virtual register one wants to read from to the WRITE register 0x01.
	 */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_write /* The pointer to the commands to be transferred */,
							2 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							500 /* timeout in milliseconds */);		

	/*
	 *	Read transaction which reads from the READ register 0x02.
	 *	The read transaction requires one to first write to the register address one wants to focus on and then read from that address.
	 */
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_read /* The pointer to the commands to be transferred */,
							1 /* The length in bytes of the commands to be transferred */,
							NULL /* The pointer to the data to be transferred */,
							0 /* The length in bytes of the data to be transferred */,
							500 /* timeout in milliseconds */);			

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave /* The pointer to the I2C device information structure */,
							cmdBuf_read /* The pointer to the commands to be transferred */,
							1 /* The length in bytes of the commands to be transferred */,
							(uint8_t *)deviceAS7263State.i2cBuffer /* The pointer to the data to be transferred */,
							1 /* The length in bytes of the data to be transferred and data is transferred from the sensor to master via bus */,
							500 /* timeout in milliseconds */);

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
initSCD30(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskTemperature	|
						kWarpTypeMaskHumidity		|
						kWarpTypeMaskC02Concentration
					);
	return;
}

WarpStatus
readSensorRegisterSCD30(uint8_t deviceRegister)
{
	uint8_t		cmdBuf[1]	= {0xFF};
	
	i2c_status_t	returnValue;

	i2c_device_t slave =
	{
		.address = deviceSCD30State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};
	cmdBuf[0] = deviceRegister;

	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							2,
							NULL,
							1,
							500 /* timeout in milliseconds */);

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceSCD30State.i2cBuffer,
							18,
							500 /* timeout in milliseconds */);

	if (returnValue == kStatus_I2C_Success)
	{
		//...
	}
	else
	{
		return kWarpStatusDeviceCommunicationFailed;
	}
	return kWarpStatusOK;
}

void
enableSPIpins(bool driveI2cPinsHighToMatchSupply)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	Warp KL03_SPI_MISO	--> PTA6	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	/*	Warp KL03_SPI_SCK	--> PTA9	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAlt3);


	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 *
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);

	/*
	 *	Whenever we drive the SPI_SCK high, it will connect the
	 *	I2C_PULLUP to SSSUPPLY. We therefore want both I2C pins
	 *	driven high when SPI is active.
	 *
	 *	Drive the I2C pins high:
	 */
	if (driveI2cPinsHighToMatchSupply)
	{
		GPIO_DRV_SetPinOutput(kWarpPinI2C0_SDA);
		GPIO_DRV_SetPinOutput(kWarpPinI2C0_SCL);	
	}
}



void
disableSPIpins(bool driveI2cPinsLow)
{
	SPI_DRV_MasterDeinit(0);


	/*	Warp KL03_SPI_MISO	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	/*	Warp KL03_SPI_SCK	--> PTA9	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK_I2C_PULLUP_EN);

	/*
	 *	Drive the I2C pins low (we drove them high when we enabled SPI so that the SPI_SCK toggle, which controls pullup enable, is not causing drain.):
	 */
	if (driveI2cPinsLow)
	{
		GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
		GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
	}

	CLOCK_SYS_DisableSpiClock(0);
}



void
enableI2Cpins(bool pullupEnable)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	/*	Warp KL03_SPI_SCK	--> PTA9	(ALT1 = GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);


	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);


	/*
	 *	Drive kWarpPinSPI_SCK_I2C_PULLUP_EN high to turn on the pullups
	 */
	if (pullupEnable)
	{
		GPIO_DRV_SetPinOutput(kWarpPinSPI_SCK_I2C_PULLUP_EN);
	}
}



void
disableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);	


	/*	Warp KL03_I2C0_SCL	--> PTB3	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);
	
	/*	Warp KL03_SPI_SCK	--> PTA9	(ALT1 = GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	
	/*
	 *	Clear the pullup.
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK_I2C_PULLUP_EN);

	/*
	 *	Drive the I2C pins low
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);


	CLOCK_SYS_DisableI2cClock(0);
}



void
disableSWDEnablePTA1x2x3xGpio(void)
{
	/*	Warp LED 1 / TS5A3154_IN	--> PTA0	(ALT1)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);

	/*	Warp LED 2 / TS5A3154_EN	--> PTA1	(ALT1)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);

	/*	Warp LED 3 / SI4705_nRST	--> PTA2	(ALT1)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
}



void
enableSWDDisablePTA1x2x3xGpio(void)
{
	/*
	 *	NOTE: we switch them to ALT3 to get back the SWD functionality.
	 *
	 *	Because we have set FOPT, the default on PTA1 is no longer RESET_b.
	 */

	/*	Warp LED 1 / TS5A3154_IN	--> SWD_CLK	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);

	/*	Warp LED 3 / SI4705_nRST	--> SWD_DIO	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);
}



void
brieflyToggleEnablingSWD(void)
{
	/*
	 *	Briefly enable SWD to let attached JLINK read the RTT buffer for print().
	 */
	enableSWDDisablePTA1x2x3xGpio();
	OSA_TimeDelay(50);
	disableSWDEnablePTA1x2x3xGpio();	
}



void
lowPowerPinStates(void)
{
	/*
	 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
	 *	we configure all pins as output and set them to a known state. We choose
	 *	to set them all to '0' since it happens that the devices we want to keep
	 *	deactivated (SI4705, PAN1326) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
	
	/*
	 *	PTA3 and PTA4 are the EXTAL/XTAL
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	
	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */

	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);

	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	
	/*
	 *	PTB1 is connected to VDD. Keep 'disabled as analog'
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);
	
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB8 or PTB9
	 */

	/*
	 *		PTB10 is unconnected in Rev 0.2 HW
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB12
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);

	/*
	 *	Now, set all the pins (except kWarpPinKL03_VDD_ADC) to 0
	 */
	/* GPIO_DRV_ClearPinOutput(kWarpPinKL03_VDD_ADC); */	
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82675_MODE);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK_I2C_PULLUP_EN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL2);
	GPIO_DRV_ClearPinOutput(kWarpPinADXL362_CS_PAN1326_nSHUTD);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82675_EN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL1);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL3);
	GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);

	GPIO_DRV_ClearPinOutput(kWarpPinLED1_TS5A3154_IN);
	GPIO_DRV_ClearPinOutput(kWarpPinLED2_TS5A3154_nEN);
	GPIO_DRV_ClearPinOutput(kWarpPinLED3_SI4705_nRST);

	GPIO_DRV_ClearPinOutput(kWarpPinUnusedPTB6);
	GPIO_DRV_ClearPinOutput(kWarpPinUnusedPTB7);
	GPIO_DRV_ClearPinOutput(kWarpPinUnusedPTB10);

	/*
	 *	HCI_RX / kWarpPinI2C0_SCL is an input. Set it low.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SCL);

	/*
	 *	HCI_TX / kWarpPinI2C0_SDA is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SDA);

	/*
	 *	HCI_RTS / kWarpPinSPI_MISO is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO);

	/*
	 *	From PAN1326 manual, page 10:
	 *
	 *		"When HCI_CTS is high, then CC256X is not allowed to send data to Host device"
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI);
}



void
disableTPS82675(void)
{
	/*
	 *	Disable the TPS8267[1,5]. From Manual:
	 *
	 *		"MODE = LOW: The device is operating in regulated frequency
	 *		pulse width modulation mode (PWM) at high-load currents and
	 *		in pulse frequency modulation mode (PFM) at light load currents.
	 *		MODE = HIGH: Low-noise mode is enabled and regulated frequency
	 *		PWM operation is forced."
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82675_MODE);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82675_EN);
}



void
enableTPS82675(bool mode)
{
	/*
	 *	Enable the TPS8267[1,5]. From Manual:
	 *
	 *		"MODE = LOW: The device is operating in regulated frequency
	 *		pulse width modulation mode (PWM) at high-load currents and
	 *		in pulse frequency modulation mode (PFM) at light load currents.
	 *		MODE = HIGH: Low-noise mode is enabled and regulated frequency
	 *		PWM operation is forced."
	 */
	if (mode)
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82675_MODE);
	}
	else
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82675_MODE);
	}

	GPIO_DRV_SetPinOutput(kWarpPinTPS82675_EN);
	OSA_TimeDelay(1);


	/*
	 *	Make sure the TS5A3154 power switch is enabled.
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinLED2_TS5A3154_nEN);

	/*
	 *	Select the TS5A3154 to use the output of the TPS82675
	 *
	 *	IN = high selects the output of the TPS82740:
	 *	IN = low selects the output of the TPS82675:
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinLED1_TS5A3154_IN);
}



void
disableTPS82740A(void)
{
	/*
	 *	Enable/disable the TPS82740A. From Manual:
	 *
	 *		VSEL1 VSEL2 VSEL3:	000-->1.8V, 111-->2.5V
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL1);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL2);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL3);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
}



void
enableTPS82740A(uint16_t voltageMillivolts)
{
	switch(voltageMillivolts)
	{
		case 1800:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL3);
			
			break;
		}

		case 1900:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL3);
			
			break;
		}

		case 2000:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL3);
			
			break;
		}

		case 2100:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL2);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL3);
			
			break;
		}

		case 2200:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL3);
			
			break;
		}

		case 2300:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL1);
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL3);
			
			break;
		}

		case 2400:
		{
			GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL3);
			
			break;
		}

		case 2500:
		{
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL1);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL2);
			GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_VSEL3);
			
			break;
		}

		/*
		 *	NOTE/TODO: If voltage passed in does not match a desired level,
		 *	we match it to closest in supportedrange.
		 */
		default:
		{
			SEGGER_RTT_printf(0, kWarpConstantStringInvalidVoltage, voltageMillivolts);brieflyToggleEnablingSWD();
		}
	}

	GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_CTLEN);
	OSA_TimeDelay(1);


	/*
	 *	Make sure the TS5A3154 power switch is enabled.
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinLED2_TS5A3154_nEN);

	/*
	 *	Select the TS5A3154 to use the output of the TPS82740
	 *
	 *		IN = high selects the output of the TPS82740:
	 *		IN = low selects the output of the TPS82675:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinLED1_TS5A3154_IN);
}



void
enableSssupply(uint16_t voltageMillivolts)
{
	if (voltageMillivolts == 1800)
	{
		enableTPS82675(0 /* mode */);
		disableTPS82740A();
	}
	else if (voltageMillivolts > 1800 && voltageMillivolts <= 2500)
	{
		enableTPS82740A(voltageMillivolts);
		disableTPS82675();
	}
	else
	{
		SEGGER_RTT_printf(0, kWarpConstantStringInvalidVoltage, voltageMillivolts);brieflyToggleEnablingSWD();
	}
}



void
disableSssupply(void)
{
	disableTPS82740A();
	disableTPS82675();
	
	/*
	 *	Clear the pin. This sets the TS5A3154 to use the output of the TPS82675,
	 *	which shouldn't matter in any case. The main objective here is to clear
	 *	the pin to reduce pwoer drain.
	 *
	 *		IN = high selects the output of the TPS82740:
	 *		IN = low selects the output of the TPS82675:
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinLED1_TS5A3154_IN);
}



void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	/*
	 *	Set all pins into low-power states. We don't just disable all pins, as the various devices hanging off will be left in higher power draw state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);
	warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
}




int
main(void)
{
	uint8_t					key;
	WarpSensorDevice			menuTargetSensor = kWarpSensorADXL362;
	bool					menuI2cPullupEnable = true;
	uint8_t					menuRegisterAddress = 0x00;
	uint16_t				menuSupplyVoltage = 0;


	rtc_datetime_t				warpBootDate;

	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: This order is depended on by POWER_SYS_SetMode()
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure			powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};



	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);



	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;



	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();



	/*
	 *	Configure all the pins (GPIO, ADC, I2C, SPI, UART, etc.)
	 *
	 *	NOTE:	Be careful not to assign SWD pins as GPIO, since 
	 *	this can prevent debugger from gaining control of the 
	 *	processor.
	 *
	 *	Before taking over the SWD pins, wait 
	 *	to give debugger a chance to force CPU into debug mode
	 *	if attached. This requires the SWD_DIO and SWD_CLK pins
	 *	to not yet be configured as GPIO. The SWD_RST pin could
	 *	indeed be set as GPIO by the FOPT byte, but that is not
	 *	needed to get access to the CPU. See jlink manual for
	 *	"Reset strategy" and "reset types".
	 *
	 */
	OSA_TimeDelay(3000);


	/*
	 *	An additional busy delay loop to make assurance doubly-sure.
	 */
	do
	{
		int32_t	i;
		for (i = 0; i < 500; i++)
		{
			__asm("nop");
		}
	} while (0);



	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_Init();
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);


	SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Warp in 3... ");
	OSA_TimeDelay(1000);
	SEGGER_RTT_WriteString(0, "2... ");
	OSA_TimeDelay(1000);
	SEGGER_RTT_WriteString(0, "1...\n\r");
	OSA_TimeDelay(1000);



	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM,
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);


	/*
	 *	Initialize RTC Driver
	 */
	RTC_DRV_Init(0);



	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);



	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));


	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;
	
	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;
	
	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;
	
	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);


	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);



	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	lowPowerPinStates();



	/*
	 *	Initialize all the sensors
	 */
	initBMX055accel(0x18	/* i2cAddress */,	&deviceBMX055accelState	);
	initBMX055gyro(	0x68	/* i2cAddress */,	&deviceBMX055gyroState	);
	initBMX055mag(	0x10	/* i2cAddress */,	&deviceBMX055magState	);
	initMMA8451Q(	0x1C	/* i2cAddress */,	&deviceMMA8451QState	);	
	initMAG3110(	0x0E	/* i2cAddress */,	&deviceMAG3110State	);
	initL3GD20H(	0x6A	/* i2cAddress */,	&deviceL3GD20HState	);
	initBMP180(	0x77	/* i2cAddress */,	&deviceBMP180State	);
	initTMP006B(	0x45	/* i2cAddress */,	&deviceTMP006BState	);
	initAS7262(	0x49	/* i2cAddress */,	&deviceAS7262State	);
	initAS7263(	0x49	/* i2cAddress */,	&deviceAS7263State	);
	initSCD30(	0x61	/* i2cAddress */,	&deviceSCD30State	);



	/*
	 *	Initialization: Devices hanging off SPI
	 */
	initADXL362(&deviceADXL362State);



	/*
	 *	Initialization: Devices hanging off UART
	 */
	initPAN1326B(&devicePAN1326BState);



	while (1)
	{
		/*
		 *	Do not, e.g., lowPowerPinStates() on each iteration, because we actually
		 *	want to use menu to progressively change the machine state with various
		 *	commands.
		 */
		SEGGER_RTT_WriteString(0, "\r\n\n\n\n[ *\t\t\t\tW\ta\tr\tp\t\t\t* ]\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r[  \t\t\t\t    Cambridge / Physcomplab / PSM\t\t  ]\n\n");brieflyToggleEnablingSWD();
		
		SEGGER_RTT_printf(0, "\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
					menuSupplyVoltage, menuRegisterAddress);brieflyToggleEnablingSWD();
		SEGGER_RTT_printf(0, "\r\tI2C=%dkb/s,\tSPI=%dkb/s,\tUART=%dkb/s,\tI2C Pull-Up=%d\n\n",
					gWarpI2cBaudRateKbps, gWarpSpiBaudRateKbps, gWarpUartBaudRateKbps, menuI2cPullupEnable);brieflyToggleEnablingSWD();
		
		SEGGER_RTT_WriteString(0, "\rSelect:\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'a': set default sensor.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'b': set I2C baud rate.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'c': set SPI baud rate.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'd': set UART baud rate.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'e': set default register address.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'f': write byte to sensor.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'g': set default SSSUPPLY.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'h': powerdown command to all sensors.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'i': set pull-up enable flag.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_printf(0, "\r- 'j': repeat read reg 0x%02x on device%d.\n", menuRegisterAddress, menuTargetSensor);brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'k': sleep for 30 seconds.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'l': send repeated byte on I2C.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'm': send repeated byte on SPI.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'n': enable SSSUPPLY.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'o': disable SSSUPPLY.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'p': switch to VLPR mode.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'r': switch to RUN mode.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 's': power up all sensors.\n");brieflyToggleEnablingSWD();
		SEGGER_RTT_WriteString(0, "\r- 'x': disable SWD and spin for 10 secs.\n");brieflyToggleEnablingSWD();

		SEGGER_RTT_WriteString(0, "\rEnter selection> ");brieflyToggleEnablingSWD();
		enableSWDDisablePTA1x2x3xGpio();
		key = SEGGER_RTT_WaitKey();
		SEGGER_RTT_WriteString(0, "    \r\n");

		switch (key)
		{
			/*
			 *	Select sensor
			 */
			case 'a':
			{
				SEGGER_RTT_WriteString(0, "\r\tSelect:\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- '1' ADXL362		(0x00--0x2D): 1.6V  -- 3.5V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- '2' BMX055accel	(0x00--0x3F): 2.4V  -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- '3' BMX055gyro		(0x00--0x3F): 2.4V  -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- '4' BMX055mag		(0x40--0x52): 2.4V  -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- '5' MMA8451Q		(0x00--0x31): 1.95V -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- '7' MAG3110		(0x00--0x11): 1.95V -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- '9' SCD30		(no regs, uses commands): 3.3V -- 5.5V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- 'a' L3GD20H		(0x0F--0x39): 2.2V  -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- 'b' BMP180		(0xAA--0xF8): 1.6V  -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- 'c' TMP006B		(0x00--0xFF): 2.2V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- 'd' AS7262		(0x00--0x2B): 2.7V -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- 'e' AS7263		(0x00--0x2B): 2.7V -- 3.6V\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\t- 'f' PAN1326		(n/a)\n");brieflyToggleEnablingSWD();
				SEGGER_RTT_WriteString(0, "\r\tEnter selection> ");brieflyToggleEnablingSWD();

				enableSWDDisablePTA1x2x3xGpio();
				key = SEGGER_RTT_WaitKey();
				disableSWDEnablePTA1x2x3xGpio();

				switch(key)
				{
					case '1':
					{
						menuTargetSensor = kWarpSensorADXL362;

						break;
					}

					case '2':
					{
						menuTargetSensor = kWarpSensorBMX055accel;

						break;
					}

					case '3':
					{
						menuTargetSensor = kWarpSensorBMX055gyro;

						break;
					}

					case '4':
					{
						menuTargetSensor = kWarpSensorBMX055mag;

						break;
					}

					case '5':
					{
						menuTargetSensor = kWarpSensorMMA8451Q;

						break;
					}

					case '6':
					{
						break;
					}

					case '7':
					{
						menuTargetSensor = kWarpSensorMAG3110;

						break;
					}

					case '9':
					{
						menuTargetSensor = kWarpSensorSCD30;

						break;
					}

					case 'a':
					{
						menuTargetSensor = kWarpSensorL3GD20H;

						break;
					}

					case 'b':
					{
						menuTargetSensor = kWarpSensorBMP180;

						break;
					}

					case 'c':
					{
						menuTargetSensor = kWarpSensorTMP006B;

						break;
					}

					case 'd':
					{
						menuTargetSensor = kWarpSensorAS7262;

						break;
					}

					case 'e':
					{
						menuTargetSensor = kWarpSensorAS7263;

						break;
					}

					case 'f':
					{
						menuTargetSensor = kWarpSensorPAN1326;

						break;
					}

					default:
					{
						SEGGER_RTT_printf(0, "\r\tInvalid selection '%c' !\n", key);brieflyToggleEnablingSWD();
					}
				}
				break;
			}

			/*
			 *	Change default I2C baud rate
			 */
			case 'b':
			{
				SEGGER_RTT_WriteString(0, "\r\n\tSet I2C baud rate in kbps (e.g., '0001')> ");brieflyToggleEnablingSWD();
				gWarpI2cBaudRateKbps = read4digits();

				/*
				 *	Round 9999kbps to 10Mbps
				 */
				if (gWarpI2cBaudRateKbps == 9999)
				{
					gWarpI2cBaudRateKbps = 10000;
				}

				SEGGER_RTT_printf(0, "\r\n\tI2C baud rate set to %d kb/s", gWarpI2cBaudRateKbps);brieflyToggleEnablingSWD();

				break;
			}

			/*
			 *	Change default SPI baud rate
			 */
			case 'c':
			{
				SEGGER_RTT_WriteString(0, "\r\n\tSet SPI baud rate in kbps (e.g., '0001')> ");brieflyToggleEnablingSWD();
				gWarpSpiBaudRateKbps = read4digits();

				/*
				 *	Round 9999kbps to 10Mbps
				 */
				if (gWarpSpiBaudRateKbps == 9999)
				{
					gWarpSpiBaudRateKbps = 10000;
				}

				SEGGER_RTT_printf(0, "\r\n\tSPI baud rate: %d kb/s", gWarpSpiBaudRateKbps);brieflyToggleEnablingSWD();

				break;
			}

			/*
			 *	Change default UART baud rate
			 */
			case 'd':
			{
				SEGGER_RTT_WriteString(0, "\r\n\tSet UART baud rate in kbps (e.g., '0001')> ");brieflyToggleEnablingSWD();
				gWarpUartBaudRateKbps = read4digits();
				SEGGER_RTT_printf(0, "\r\n\tUART baud rate: %d kb/s", gWarpUartBaudRateKbps);brieflyToggleEnablingSWD();

				break;
			}

			/*
			 *	Set register address for subsequent operations
			 */
			case 'e':
			{
				SEGGER_RTT_WriteString(0, "\r\n\tEnter 2-nybble register hex address (e.g., '3e')> ");brieflyToggleEnablingSWD();
				menuRegisterAddress = readHexByte();
				SEGGER_RTT_printf(0, "\r\n\tEntered [0x%02x].\n\n", menuRegisterAddress);brieflyToggleEnablingSWD();

				break;
			}

			/*
			 *	Write byte to sensor
			 */
			case 'f':
			{
				uint8_t		i2cAddress, payloadByte[1], commandByte[1];
				i2c_status_t	i2cStatus;
				WarpStatus	status;
	

				SEGGER_RTT_WriteString(0, "\r\n\tEnter I2C addr. (e.g., '0f') or '99' for SPI > ");brieflyToggleEnablingSWD();
				i2cAddress = readHexByte();
				SEGGER_RTT_printf(0, "\r\n\tEntered [0x%02x].\n", i2cAddress);brieflyToggleEnablingSWD();


				SEGGER_RTT_WriteString(0, "\r\n\tEnter hex byte to send (e.g., '0f')> ");brieflyToggleEnablingSWD();
				payloadByte[0] = readHexByte();
				SEGGER_RTT_printf(0, "\r\n\tEntered [0x%02x].\n", payloadByte[0]);brieflyToggleEnablingSWD();

				if (i2cAddress == 0x99)
				{
					SEGGER_RTT_printf(0, "\r\n\tWriting [0x%02x] to SPI register [0x%02x]...\n", payloadByte[0], menuRegisterAddress);brieflyToggleEnablingSWD();
					status = writeSensorRegisterADXL362(	0x0A			/* command == write register	*/,
										menuRegisterAddress,
										payloadByte[0]		/* writeValue			*/
									);
					if (status != kWarpStatusOK)
					{
						SEGGER_RTT_printf(0, "\r\n\tSPI write failed, error %d.\n\n", status);brieflyToggleEnablingSWD();
					}
				}
				else
				{
					i2c_device_t slave =
					{
						.address = i2cAddress,
						.baudRate_kbps = gWarpI2cBaudRateKbps
					};

					enableSssupply(menuSupplyVoltage);
					enableI2Cpins(menuI2cPullupEnable);

					/*
					 *	Wait for supply and pull-ups to settle.
					 */
					OSA_TimeDelay(1000);

					commandByte[0] = menuRegisterAddress;
					i2cStatus = I2C_DRV_MasterSendDataBlocking(
											0 /* I2C instance */,
											&slave,
											commandByte,
											1,
											payloadByte,
											1,
											1000);
					if (i2cStatus != kStatus_I2C_Success)
					{
						SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", i2cStatus);brieflyToggleEnablingSWD();
					}
					disableI2Cpins();
				}

				/*
				 *	NOTE: do not disable the supply here, because we typically want to build on the effect of this register write command.
				 */

				break;
			}

			/*
			 *	Configure default TPS82740 voltage
			 */
			case 'g':
			{
				SEGGER_RTT_WriteString(0, "\r\n\tOverride SSSUPPLY in mV (e.g., '1800')> ");brieflyToggleEnablingSWD();
				menuSupplyVoltage = read4digits();
				SEGGER_RTT_printf(0, "\r\n\tOverride SSSUPPLY set to %d mV", menuSupplyVoltage);brieflyToggleEnablingSWD();

				break;
			}

			/*
			 *	Activate low-power modes in all sensors.
			 */
			case 'h':
			{
				activateAllLowPowerSensorModes();

				break;
			}

			/*
			 *	Configure default pullup enable flag
			 */
			case 'i':
			{
				SEGGER_RTT_WriteString(0, "\r\n\tDefault pullup enable flag ['0' | '1']> ");brieflyToggleEnablingSWD();
				enableSWDDisablePTA1x2x3xGpio();
				menuI2cPullupEnable = SEGGER_RTT_WaitKey() - '0';
				disableSWDEnablePTA1x2x3xGpio();
				SEGGER_RTT_printf(0, "\r\n\tI2cPullupEnable Flag set to %d\n", menuI2cPullupEnable);brieflyToggleEnablingSWD();
				
				break;
			}

			/*
			 *	Start repeated read
			 */
			case 'j':
			{
				bool		autoIncrement, chatty;
				int		spinDelay, repetitionsPerAddress, chunkReadsPerAddress;
				int		adaptiveSssupplyMaxMillivolts;
				uint8_t		referenceByte;


				SEGGER_RTT_printf(0, "\r\n\tAuto-increment from base address 0x%02x? ['0' | '1']> ", menuRegisterAddress);brieflyToggleEnablingSWD();
				enableSWDDisablePTA1x2x3xGpio();
				autoIncrement = SEGGER_RTT_WaitKey() - '0';
				disableSWDEnablePTA1x2x3xGpio();

				SEGGER_RTT_WriteString(0, "\r\n\tChunk reads per address (e.g., '1')> ");brieflyToggleEnablingSWD();
				enableSWDDisablePTA1x2x3xGpio();
				chunkReadsPerAddress = SEGGER_RTT_WaitKey() - '0';
				disableSWDEnablePTA1x2x3xGpio();

				SEGGER_RTT_WriteString(0, "\r\n\tChatty? ['0' | '1']> ");brieflyToggleEnablingSWD();
				enableSWDDisablePTA1x2x3xGpio();
				chatty = SEGGER_RTT_WaitKey() - '0';
				disableSWDEnablePTA1x2x3xGpio();

				SEGGER_RTT_WriteString(0, "\r\n\tInter-operation spin delay in milliseconds (e.g., '0000')> ");brieflyToggleEnablingSWD();
				spinDelay = read4digits();

				SEGGER_RTT_WriteString(0, "\r\n\tRepetitions per address (e.g., '0000')> ");brieflyToggleEnablingSWD();
				repetitionsPerAddress = read4digits();

				SEGGER_RTT_WriteString(0, "\r\n\tMaximum voltage for adaptive supply (e.g., '0000')> ");brieflyToggleEnablingSWD();
				adaptiveSssupplyMaxMillivolts = read4digits();

				SEGGER_RTT_WriteString(0, "\r\n\tReference byte for comparisons (e.g., '3e')> ");brieflyToggleEnablingSWD();
				referenceByte = readHexByte();

				SEGGER_RTT_printf(0, "\r\n\tRepeating dev%d @ 0x%02x, reps=%d, pull=%d, delay=%dms:\n\n",
					menuTargetSensor, menuRegisterAddress, repetitionsPerAddress, menuI2cPullupEnable, spinDelay);brieflyToggleEnablingSWD();

				repeatRegisterReadForDeviceAndAddress(	menuTargetSensor /*warpSensorDevice*/, 
									menuRegisterAddress /*baseAddress */,
									menuI2cPullupEnable,
									autoIncrement /*autoIncrement*/,
									chunkReadsPerAddress,
									chatty,
									spinDelay,
									repetitionsPerAddress,
									menuSupplyVoltage,
									adaptiveSssupplyMaxMillivolts,
									referenceByte
								);

				break;
			}

			/*
			 *	Sleep for 30 seconds.
			 */
			case 'k':
			{
				SEGGER_RTT_WriteString(0, "\r\n\tSleeping for 30 seconds...\n");brieflyToggleEnablingSWD();
				warpLowPowerSecondsSleep(30, true /* forceAllPinsIntoLowPowerState */);
				SEGGER_RTT_WriteString(0, "\r\tDone.\n\n");brieflyToggleEnablingSWD();

				break;
			}

			/*
			 *	Send repeated byte on I2C or SPI
			 */
			case 'l':
			case 'm':
			{
				uint8_t		outBuffer[1];
				int		repetitions;

				SEGGER_RTT_WriteString(0, "\r\n\tByte to send (e.g., 'F0')> ");brieflyToggleEnablingSWD();
				outBuffer[0] = readHexByte();

				SEGGER_RTT_WriteString(0, "\r\n\tRepetitions (e.g., '0000')> ");brieflyToggleEnablingSWD();
				repetitions = read4digits();

				if (key == 'l')
				{
					SEGGER_RTT_printf(0, "\r\n\tSending %d repetitions of [0x%02x] on I2C, i2cPullupEnable=%d, SSSUPPLY=%dmV\n\n",
						repetitions, outBuffer[0], menuI2cPullupEnable, menuSupplyVoltage);brieflyToggleEnablingSWD();
					for (int i = 0; i < repetitions; i++)
					{
						writeByteToI2cDeviceRegister(0xFF, true /* sedCommandByte */, outBuffer[0] /* commandByte */, false /* sendPayloadByte */, 0 /* payloadByte */);
					}
				}
				else
				{
					SEGGER_RTT_printf(0, "\r\n\tSending %d repetitions of [0x%02x] on SPI, SSSUPPLY=%dmV\n\n",
						repetitions, outBuffer[0], menuSupplyVoltage);brieflyToggleEnablingSWD();
					for (int i = 0; i < repetitions; i++)
					{
						writeBytesToSpi(outBuffer /* payloadByte */, 1 /* payloadLength */, false /* driveI2cPinsHighToMatchSupply */, false /* driveI2cPinsLow */);
					}
				}

				break;
			}


			/*
			 *	enable SSSUPPLY
			 */
			case 'n':
			{
				enableSssupply(menuSupplyVoltage);
				break;
			}

			/*
			 *	disable SSSUPPLY
			 */
			case 'o':
			{
				disableSssupply();
				break;
			}

			/*
			 *	Switch to VLPR
			 */
			case 'p':
			{
				warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */);
				break;
			}

			/*
			 *	Switch to RUN
			 */
			case 'r':
			{
				warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */);
				break;
			}

			/*
			 *	Power up all sensors
			 */
			case 's':
			{
				powerupAllSensors();
				break;
			}

			/*
			 *	Simply spin for 10 seconds. Since the SWD pins should only be enabled when we are waiting for key at top of loop (or toggling after printf), during this time there should be no interference from the SWD.
			 */
			case 'x':
			{
				SEGGER_RTT_WriteString(0, "\r\n\tSpinning for 10 seconds...\n");brieflyToggleEnablingSWD();
				OSA_TimeDelay(10000);
				SEGGER_RTT_WriteString(0, "\r\tDone.\n\n");brieflyToggleEnablingSWD();

				break;
			}

			/*
			 *	Ignore naked returns.
			 */
			case '\n':
			{
				SEGGER_RTT_WriteString(0, "\r\tPayloads make rockets more than just fireworks.");brieflyToggleEnablingSWD();
				break;
			}

			default:
			{
				SEGGER_RTT_printf(0, "\r\tInvalid selection '%c' !\n", key);brieflyToggleEnablingSWD();
			}
		}
	}

	return 0;
}



void
repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, bool pullupEnable, bool autoIncrement, int chunkReadsPerAddress, bool chatty, int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts, uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte)
{
	WarpStatus		status;
	uint8_t			address = baseAddress;
	int			readCount = repetitionsPerAddress + 1;
	int			nSuccesses = 0;
	int			nFailures = 0;
	int			nCorrects = 0;
	int			nBadCommands = 0;
	uint16_t		actualSssupplyMillivolts = sssupplyMillivolts;
	uint16_t		voltageTrace[readCount];


	enableSssupply(actualSssupplyMillivolts);
	OSA_TimeDelay(100);

	if (warpSensorDevice != kWarpSensorADXL362)
	{
		enableI2Cpins(pullupEnable);
	}

	switch (warpSensorDevice)
	{
		case kWarpSensorADXL362:
		{
			/*
			 *	ADXL362: VDD 1.6--3.5
			 */
			SEGGER_RTT_WriteString(0, "\r\nADXL362:\n\r");brieflyToggleEnablingSWD();
			ADXL362loop: if (address < 0x2E)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterADXL362(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceADXL362State.spiSinkBuffer[2])
						{
							nCorrects++;
						}

						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> [0x%02x 0x%02x 0x%02x]\n",
								address+j,
								deviceADXL362State.spiSinkBuffer[0],
								deviceADXL362State.spiSinkBuffer[1],
								deviceADXL362State.spiSinkBuffer[2]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();			
						
						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto ADXL362loop;
				}
			}
			break;
		}

		case kWarpSensorMMA8451Q:
		{
			/*
			 *	MMA8451Q: VDD 1.95--3.6
			 */
			SEGGER_RTT_WriteString(0, "\r\nMMA8451Q:\n\r");brieflyToggleEnablingSWD();
			MMA8451Qloop: if (address < 0x32)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterMMA8451Q(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceMMA8451QState.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceMMA8451QState.i2cBuffer[0]);brieflyToggleEnablingSWD();			
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();

						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto MMA8451Qloop;
				}
			}

			break;
		}

		case kWarpSensorBMP180:
		{
			/*
			 *	BMP180: VDD 1.6--3.6
			 */
			SEGGER_RTT_WriteString(0, "\r\nBMP180:\n\r");brieflyToggleEnablingSWD();
			BMP180loop: if (address >= 0xAA && address <= 0xF8)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterBMP180(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceBMP180State.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceBMP180State.i2cBuffer[0]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();

						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto BMP180loop;
				}
			}

			break;
		}

		case kWarpSensorBMX055accel:
		{
			/*
			 *	BMX055accel: VDD 2.4V -- 3.6V
			 */
			SEGGER_RTT_WriteString(0, "\r\nBMX055accel:\n\r");brieflyToggleEnablingSWD();
			BMX055accelloop: if (address < 0x40)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterBMX055accel(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceBMX055accelState.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceBMX055accelState.i2cBuffer[0]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();

						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto BMX055accelloop;
				}
			}

			break;
		}

		case kWarpSensorBMX055gyro:
		{
			/*
			 *	BMX055gyro: VDD 2.4V -- 3.6V
			 */
			SEGGER_RTT_WriteString(0, "\r\nBMX055gyro:\n\r");brieflyToggleEnablingSWD();
			BMX055gyroloop: if (address < 0x40)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterBMX055gyro(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceBMX055gyroState.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceBMX055gyroState.i2cBuffer[0]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();

						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto BMX055gyroloop;
				}
			}

			break;
		}

		case kWarpSensorBMX055mag:
		{
			/*
			 *	BMX055mag: VDD 2.4V -- 3.6V
			 */
			/*
			SEGGER_RTT_WriteString(0, "\r\nBMX055mag:\n\r");brieflyToggleEnablingSWD();
			BMX055magloop: if (address >= 0x40 && address < 0x53)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterBMX055mag(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceBMX055magState.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceBMX055magState.i2cBuffer[0]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();

						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto BMX055magloop;
				}
			}
			*/

			break;
		}

		case kWarpSensorTMP006B:
		{
			/*
			 *	TMP006B: VDD 2.2V
			 */
			SEGGER_RTT_WriteString(0, "\r\nTMP006B:\n\r");brieflyToggleEnablingSWD();
			TMP006Bloop1: if (address <= 0x02)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterTMP006B(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						/*
						 *	NOTE: Unlike for other snesors, we compare the reference 
						 *	byte to i2cBuffer[1], not i2cBuffer[0].
						 */
						if (referenceByte == deviceTMP006BState.i2cBuffer[1])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> [0x%02x 0x%02x]\n",
								address,
								deviceTMP006BState.i2cBuffer[0],
								deviceTMP006BState.i2cBuffer[1]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					if (address > 0x02) address = 0xFE;
					goto TMP006Bloop1;
				}
			}
			TMP006Bloop2: if (address >= 0xFE && address <= 0xFF)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterTMP006B(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						/*
						 *	NOTE: Unlike for other snesors, we compare the reference 
						 *	byte to i2cBuffer[1], not i2cBuffer[0].
						 */
						if (referenceByte == deviceTMP006BState.i2cBuffer[1])
						{
							nCorrects++;
						}

						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> [0x%02x 0x%02x]\n",
								address,
								deviceTMP006BState.i2cBuffer[0],
								deviceTMP006BState.i2cBuffer[1]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto TMP006Bloop2;
				}
			}

			break;
		}

		case kWarpSensorMAG3110:
		{
			/*
			 *	MAG3110: VDD 1.95 -- 3.6
			 */
			SEGGER_RTT_WriteString(0, "\r\nMAG3110:\n\r");brieflyToggleEnablingSWD();
			MAG3110loop: if (address < 0x12)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterMAG3110(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceMAG3110State.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceMAG3110State.i2cBuffer[0]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();
						
						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto MAG3110loop;
				}
			}

			break;
		}

		case kWarpSensorAS7262:
		{
			/*
			 *	AS7262: VDD 2.7--3.6
			 */
			SEGGER_RTT_WriteString(0, "\r\nAS7262:\n\r");brieflyToggleEnablingSWD();
			AS7262loop: if (address <= 0x2B)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterAS7262(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceAS7262State.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceAS7262State.i2cBuffer[0]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();

						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto AS7262loop;
				}
			}

			break;
		}

		case kWarpSensorAS7263:
		{
			/*
			 *	AS7263: VDD 2.7--3.6
			 */
			SEGGER_RTT_WriteString(0, "\r\nAS7263:\n\r");brieflyToggleEnablingSWD();
			AS7263loop: if (address <= 0x2B)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterAS7263(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceAS7262State.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceAS7263State.i2cBuffer[0]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();

						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto AS7263loop;
				}
			}

			break;
		}

		case kWarpSensorSCD30:
		{
			/*
			 *	SCD30: VDD 3.3--5.5
			 */
			SEGGER_RTT_WriteString(0, "\r\nSCD30:\n\r");brieflyToggleEnablingSWD();
			SCD30loop:
			for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
			{
				voltageTrace[i] = actualSssupplyMillivolts;
				status = readSensorRegisterSCD30(address+j);
				if (status == kWarpStatusOK)
				{
					nSuccesses++;
					if (actualSssupplyMillivolts > sssupplyMillivolts)
					{
						actualSssupplyMillivolts -= 100;
						enableSssupply(actualSssupplyMillivolts);
					}
					if (referenceByte == deviceSCD30State.i2cBuffer[0])
					{
						nCorrects++;
					}
					if (chatty)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
							address+j,
							deviceSCD30State.i2cBuffer[0]);brieflyToggleEnablingSWD();
					}
				}
				else if (status == kWarpStatusDeviceCommunicationFailed)
				{
					SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
						address+j);brieflyToggleEnablingSWD();
					nFailures++;
					if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
					{
						actualSssupplyMillivolts += 100;
						enableSssupply(actualSssupplyMillivolts);
					}
				}
				else if (status == kWarpStatusBadDeviceCommand)
				{
					nBadCommands++;
				}
				if (spinDelay > 0) OSA_TimeDelay(spinDelay);
			}
			if (autoIncrement)
			{
				address++;
				goto SCD30loop;
			}

			break;
		}

		case kWarpSensorL3GD20H:
		{
			/*
			 *	L3GD20H: VDD 2.2V -- 3.6V
			 */
			SEGGER_RTT_WriteString(0, "\r\nL3GD20H:\n\r");brieflyToggleEnablingSWD();
			L3GD20Hloop: if (address >= 0x0F && address <= 0x39)
			{
				for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
				{
					voltageTrace[i] = actualSssupplyMillivolts;
					status = readSensorRegisterL3GD20H(address+j);
					if (status == kWarpStatusOK)
					{
						nSuccesses++;
						if (actualSssupplyMillivolts > sssupplyMillivolts)
						{
							actualSssupplyMillivolts -= 100;
							enableSssupply(actualSssupplyMillivolts);
						}

						if (referenceByte == deviceL3GD20HState.i2cBuffer[0])
						{
							nCorrects++;
						}


						if (chatty)
						{
							SEGGER_RTT_printf(0, "\r0x%02x --> 0x%02x\n",
								address+j,
								deviceL3GD20HState.i2cBuffer[0]);brieflyToggleEnablingSWD();
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						SEGGER_RTT_printf(0, "\r0x%02x --> ----\n",
							address+j);brieflyToggleEnablingSWD();

						nFailures++;
						if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
						{
							actualSssupplyMillivolts += 100;
							enableSssupply(actualSssupplyMillivolts);
						}
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}

					if (spinDelay > 0) OSA_TimeDelay(spinDelay);
				}

				if (autoIncrement)
				{
					address++;
					goto L3GD20Hloop;
				}
			}

			break;
		}

		default:
		{
			SEGGER_RTT_printf(0, "\r\tInvalid warpSensorDevice [%d] passed to repeatRegisterReadForDeviceAndAddress.\n", warpSensorDevice);brieflyToggleEnablingSWD();
		}
	}

	if (warpSensorDevice != kWarpSensorADXL362)
	{
		disableI2Cpins();
	}

	/*
	 *	To make printing of stats robust, we switch to VLPR (assuming we are not already in VLPR).
	 *
	 *	As of circa issue-58 implementation, RTT printing when in RUN mode was flaky (achievable SWD speed too slow for buffer fill rate?)
	 */
	warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */);

	SEGGER_RTT_printf(0, "\r\n\t%d/%d success rate.\n", nSuccesses, (nSuccesses + nFailures));brieflyToggleEnablingSWD();
	SEGGER_RTT_printf(0, "\r\t%d/%d successes matched ref. value of 0x%02x.\n", nCorrects, nSuccesses, referenceByte);brieflyToggleEnablingSWD();
	SEGGER_RTT_printf(0, "\r\t%d bad commands.\n\n", nBadCommands);brieflyToggleEnablingSWD();
	SEGGER_RTT_printf(0, "\r\tVoltage trace:\n", nBadCommands);brieflyToggleEnablingSWD();

	for (int i = 0; i < readCount; i++)
	{
		SEGGER_RTT_printf(0, "\r\t\t%d\t%d\n", i, voltageTrace[i]);

		/*
		 *	To give debug interface time to catch up with the prints.
		 *
		 *	Even when we force CPU into VLPR mode, we need a longer delay
		 *	than the 10ms that we now use in brieflyToggleEnablingSWD(),
		 *	since RTT buffer will be filling quickly.
		 */		
		enableSWDDisablePTA1x2x3xGpio();
		OSA_TimeDelay(50);
		disableSWDEnablePTA1x2x3xGpio();
	}
}



int
char2int(int character)
{
	if (character >= '0' && character <= '9')
	{
		return character - '0';
	}

	if (character >= 'a' && character <= 'f')
	{
		return character - 'a' + 10;
	}

	if (character >= 'A' && character <= 'F')
	{
		return character - 'A' + 10;
	}

	return 0;
}



uint8_t
readHexByte(void)
{
	uint8_t		topNybble, bottomNybble;

	enableSWDDisablePTA1x2x3xGpio();
	topNybble = SEGGER_RTT_WaitKey();
	bottomNybble = SEGGER_RTT_WaitKey();
	disableSWDEnablePTA1x2x3xGpio();

	return (char2int(topNybble) << 4) + char2int(bottomNybble);
}



int
read4digits(void)
{
	uint8_t		digit1, digit2, digit3, digit4;
	
	enableSWDDisablePTA1x2x3xGpio();
	digit1 = SEGGER_RTT_WaitKey();
	digit2 = SEGGER_RTT_WaitKey();
	digit3 = SEGGER_RTT_WaitKey();
	digit4 = SEGGER_RTT_WaitKey();
	disableSWDEnablePTA1x2x3xGpio();

	return (digit1 - '0')*1000 + (digit2 - '0')*100 + (digit3 - '0')*10 + (digit4 - '0');
}



WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
	i2c_status_t	status;
	uint8_t		commandBuffer[1];
	uint8_t		payloadBuffer[1];
	i2c_device_t	i2cSlaveConfig =
			{
				.address = i2cAddress,
				.baudRate_kbps = gWarpI2cBaudRateKbps
			};

	commandBuffer[0] = commandByte;
	payloadBuffer[0] = payloadByte;
	enableI2Cpins(true /* pullupEnable*/);
	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBuffer,
						(sendPayloadByte ? 1 : 0),
						1000	/* timeout in milliseconds */);
	disableI2Cpins();

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength, bool driveI2cPinsHighToMatchSupply, bool driveI2cPinsLow)
{
	uint8_t		inBuffer[payloadLength];
	spi_status_t	status;
	
	enableSPIpins(driveI2cPinsHighToMatchSupply);
	status = SPI_DRV_MasterTransferBlocking(0		/* master instance */,
						NULL		/* spi_master_user_config_t */,
						payloadBytes,
						inBuffer,
						payloadLength	/* transfer size */,
						1000		/* timeout in microseconds (unlike I2C which is ms) */);
	disableSPIpins(driveI2cPinsLow);

	return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}



void
powerupAllSensors(void)
{
	WarpStatus	status;

	/*
	 *	BMX055mag
	 *
	 *	Write '1' to power control bit of register 0x4B. See page 134.
	 */
	status = writeByteToI2cDeviceRegister(	deviceBMX055magState.i2cAddress		/*	i2cAddress		*/,
						true					/*	sendCommandByte		*/,
						0x4B					/*	commandByte		*/,
						true					/*	sendPayloadByte		*/,
						(1 << 0)				/*	payloadByte		*/);
	if (status != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "\r\tPowerup command failed, code=%d, for BMX055mag @ 0x%02x.\n", status, deviceBMX055magState.i2cAddress);brieflyToggleEnablingSWD();
	}
}



void
activateAllLowPowerSensorModes(void)
{
	WarpStatus	status;



	/*
	 *	ADXL362:	See Power Control Register (Address: 0x2D, Reset: 0x00).
	 *
	 *	POR values are OK.
	 */



	/*
	 *	BMX055accel: At POR, device is in Normal mode. Move it to Deep Suspend mode.
	 *
	 *	Write '1' to deep suspend bit of register 0x11, and write '0' to suspend bit of register 0x11. See page 23.
	 */
	status = writeByteToI2cDeviceRegister(	deviceBMX055accelState.i2cAddress	/*	i2cAddress		*/,
						true					/*	sendCommandByte		*/,
						0x11					/*	commandByte		*/,
						true					/*	sendPayloadByte		*/,
						(1 << 5)				/*	payloadByte		*/);
	if (status != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "\r\tPowerdown command failed, code=%d, for BMX055accel @ 0x%02x.\n", status, deviceBMX055accelState.i2cAddress);brieflyToggleEnablingSWD();
	}


	/*
	 *	BMX055gyro: At POR, device is in Normal mode. Move it to Deep Suspend mode.
	 *
	 *	Write '1' to deep suspend bit of register 0x11. See page 81.
	 */
	status = writeByteToI2cDeviceRegister(	deviceBMX055gyroState.i2cAddress	/*	i2cAddress		*/,
						true					/*	sendCommandByte		*/,
						0x11					/*	commandByte		*/,
						true					/*	sendPayloadByte		*/,
						(1 << 5)				/*	payloadByte		*/);
	if (status != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "\r\tPowerdown command failed, code=%d, for BMX055gyro @ 0x%02x.\n", status, deviceBMX055gyroState.i2cAddress);brieflyToggleEnablingSWD();
	}



	/*
	 *	L3GD20H: See CTRL1 at 0x20 (page 36).
	 *
	 *	POR state seems to be powered down.
	 */
	status = writeByteToI2cDeviceRegister(	deviceL3GD20HState.i2cAddress	/*	i2cAddress		*/,
						true				/*	sendCommandByte		*/,
						0x20				/*	commandByte		*/,
						true				/*	sendPayloadByte		*/,
						0x00				/*	payloadByte		*/);
	if (status != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "\r\tPowerdown command failed, code=%d, for L3GD20H @ 0x%02x.\n", status, deviceL3GD20HState.i2cAddress);brieflyToggleEnablingSWD();
	}



	/*
	 *	TMP006B: Configuration Register at address 0x02. At POR, is in "Sensor and die continuous conversion" mode.
	 *
	 *	Set config register to 0x00 (see page 20).
	 */
	status = writeByteToI2cDeviceRegister(	deviceTMP006BState.i2cAddress	/*	i2cAddress		*/,
						true				/*	sendCommandByte		*/,
						0x02				/*	commandByte		*/,
						true				/*	sendPayloadByte		*/,
						0x00				/*	payloadByte		*/);
	if (status != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "\r\tPowerdown command failed, code=%d, for TMP006B @ 0x%02x.\n", status, deviceTMP006BState.i2cAddress);brieflyToggleEnablingSWD();
	}


	/*
	 *	PAN1326.
	 *
	 *	For now, simply hold its reset line low.
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinADXL362_CS_PAN1326_nSHUTD);
}