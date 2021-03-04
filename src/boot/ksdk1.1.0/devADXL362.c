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

#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devADXL362.h"

extern volatile WarpSPIDeviceState	deviceADXL362State;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;
extern uint8_t				gWarpSpiCommonSourceBuffer[];
extern uint8_t				gWarpSpiCommonSinkBuffer[];


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
initADXL362(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts)
{
	deviceADXL362State.chipSelectIoPinID		= chipSelectIoPinID;
	deviceADXL362State.spiSourceBuffer		= gWarpSpiCommonSourceBuffer;
	deviceADXL362State.spiSinkBuffer		= gWarpSpiCommonSinkBuffer;
	deviceADXL362State.spiBufferLength		= kWarpMemoryCommonSpiBufferBytes;
	deviceADXL362State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterADXL362(uint8_t command, uint8_t deviceRegister, uint8_t writeValue, int numberOfAdditionalDummyBytes)
{
	spi_status_t	status;


	warpScaleSupplyVoltage(deviceADXL362State.operatingVoltageMillivolts);

	/*
	 *	Appropriately restrict the total number of bytes shifted out
	 *	(and hence shifted in) to the size of the sink buffer.
	 */
	int		totalTransactionBytes = min(numberOfAdditionalDummyBytes + 3, kWarpMemoryCommonSpiBufferBytes);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

	/*
	 *	Populate the shift-out register with the read-register command,
	 *	followed by the register to be read, followed by the single byte
	 *	to be written.
	 */
	deviceADXL362State.spiSourceBuffer[0] = command;
	deviceADXL362State.spiSourceBuffer[1] = deviceRegister;
	deviceADXL362State.spiSourceBuffer[2] = writeValue;

	deviceADXL362State.spiSinkBuffer[0] = 0x00;
	deviceADXL362State.spiSinkBuffer[1] = 0x00;
	deviceADXL362State.spiSinkBuffer[2] = 0x00;

	/*
	 *	First, create a falling edge on chip-select.
	 */
	GPIO_DRV_SetPinOutput(deviceADXL362State.chipSelectIoPinID);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(deviceADXL362State.chipSelectIoPinID);

	/*
	 *	The result of the SPI transaction will be stored in deviceADXL362State.spiSinkBuffer.
	 *
	 *	Providing a spi_master_user_config_t is optional since it is already provided when we did
	 *	SPI_DRV_MasterConfigureBus(), so we pass in NULL. The "master instance" is always 0 for
	 *	the KL03 since there is only one SPI peripheral.
	 */
	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceADXL362State.spiSourceBuffer,
					(uint8_t * restrict)deviceADXL362State.spiSinkBuffer,
					totalTransactionBytes /* transfer size */,
					gWarpSpiTimeoutMicroseconds);
	warpDisableSPIpins();

	/*
	 *	Disengage the ADXL362
	 */
	GPIO_DRV_SetPinOutput(deviceADXL362State.chipSelectIoPinID);

	if (status != kStatus_SPI_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterADXL362(uint8_t deviceRegister, int numberOfBytes)
{
	/*
	 *	Since writeSensorRegisterADXL362() will already write one additional byte
	 *	after the R/W command designator and the register designator, only need
	 *	to ask it to do numberOfBytes - 1 additional dummy shifts out.
	 *
	 *	NOTE: writeSensorRegisterADXL362() will appropriately restrict the total
	 *	number of bytes shifted out (and hence shifted in) to the size of the
	 *	sink buffer.
	 */
	return writeSensorRegisterADXL362(0x0B /* command == read register */, deviceRegister, 0x00 /* writeValue */, numberOfBytes - 1);
}

WarpStatus
readFIFObytesADXL362(void)
{
	WarpStatus	status;


	warpScaleSupplyVoltage(deviceADXL362State.operatingVoltageMillivolts);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

	/*
	 *	Set MEASURE mode with AUTOSLEEP
	 */
	status = writeSensorRegisterADXL362(	0x0A	/*	command == write register		*/,
						0x2D	/*	The register to write			*/,
						0x06	/*	writeValue				*/,
						0	/*	number of additional dummy bytes	*/
					);
	if (status != kWarpStatusOK)
	{
	}

	/*
	 *	Set ODR mode to 400Hz
	 */
	status = writeSensorRegisterADXL362(	0x0A	/* command == write register			*/,
						0x2C	/* The register to write			*/,
						0x07	/* writeValue					*/,
						0	/* number of additional dummy bytes		*/
					);
	if (status != kWarpStatusOK)
	{
	}

	/*
	 *	Set FIFO mode to STREAM and AH bit to 1, also read temperature to FIFO
	 */
	status = writeSensorRegisterADXL362(	0x0A	/*	command == write register		*/,
						0x28	/*	The register to write			*/,
						0x0E	/*	writeValue				*/,
						0	/*	number of additional dummy bytes	*/
					);
	if (status != kWarpStatusOK)
	{
	}

	/*
	 *	Set FIFO SAMPLES to 0xFF (AH bit to 1 previously, so total FIFO samples is 512)
	 */
	status = writeSensorRegisterADXL362(	0x0A	/*	command == write register		*/,
						0x29	/*	The register to write			*/,
						0xFF	/*	writeValue				*/,
						0	/*	number of additional dummy bytes	*/
					);
	if (status != kWarpStatusOK)
	{
	}

	/*
	 *	Put the sensor in LOOP mode with ACT_EN and INACT_EN enabled
	 */
	status = writeSensorRegisterADXL362(	0x0A	/*	command == write register		*/,
						0x27	/*	ACT_INACT_CTL register			*/,
						0x35	/*	writeValue				*/,
						0	/*	numberOfAdditionalDummyBytes		*/
					);
	if (status != kWarpStatusOK)
	{
	}

	/*
	 *	Tune this delay so that the FIFO fill level printed below is ~kWarpMemoryCommonSpiBufferBytes entries
	 */
	OSA_TimeDelay(10);

	/*
	 *	Populate the shift-out register with the read-FIFO command.
	 */
	deviceADXL362State.spiSourceBuffer[0] = 0x0D;

	GPIO_DRV_SetPinOutput(deviceADXL362State.chipSelectIoPinID);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(deviceADXL362State.chipSelectIoPinID);

	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
						NULL /* spi_master_user_config_t */,
						(const uint8_t *restrict)deviceADXL362State.spiSourceBuffer,
						(uint8_t * restrict) deviceADXL362State.spiSinkBuffer,
						kWarpMemoryCommonSpiBufferBytes /* transfer size */,
						gWarpSpiTimeoutMicroseconds /* timeout in microseconds (unlike I2C which is ms) */);
	warpDisableSPIpins();

	GPIO_DRV_SetPinOutput(deviceADXL362State.chipSelectIoPinID);

	/*
	 *	Read and print the STATUS
	 */
	status = readSensorRegisterADXL362(0x0B, 1 /* numberOfBytes */);
	if (status != kWarpStatusOK)
	{
	}
	else
	{
		warpPrint("Status --> [0x%02x]\n",
				deviceADXL362State.spiSinkBuffer[2]);
	}

	/*
	 *	Read and print FIFO_ENTRIES_L
	 */
	status = readSensorRegisterADXL362(0x0C, 1 /* numberOfBytes */);
	if (status != kWarpStatusOK)
	{
	}
	else
	{
		warpPrint("FIFO_ENTRIES_L --> [0x%02x]\n",
				deviceADXL362State.spiSinkBuffer[2]);
	}

	/*
	 *	Read and print FIFO_ENTRIES_H
	 */
	status = readSensorRegisterADXL362(0x0D, 1 /* numberOfBytes */);
	if (status != kWarpStatusOK)
	{
	}
	else
	{
		warpPrint("FIFO_ENTRIES_H --> [0x%02x]\n",
				deviceADXL362State.spiSinkBuffer[2]);
	}

	for (int i = 1; i < kWarpMemoryCommonSpiBufferBytes; i += 8)
	{
		warpPrint("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				deviceADXL362State.spiSinkBuffer[i + 0],
				deviceADXL362State.spiSinkBuffer[i + 1],
				deviceADXL362State.spiSinkBuffer[i + 2],
				deviceADXL362State.spiSinkBuffer[i + 3],
				deviceADXL362State.spiSinkBuffer[i + 4],
				deviceADXL362State.spiSinkBuffer[i + 5],
				deviceADXL362State.spiSinkBuffer[i + 6],
				deviceADXL362State.spiSinkBuffer[i + 7]
			);
	}
	warpPrint("\n");

	if (status != kStatus_SPI_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataADXL362(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	status;

//readFIFObytesADXL362();

	/*
	 *	Set MEASURE mode with AUTOSLEEP
	 */
	status = writeSensorRegisterADXL362(	0x0A	/*	command == write register		*/,
						0x2D	/*	The register to write			*/,
						0x06	/*	writeValue				*/,
						0	/*	number of additional dummy bytes	*/
					);
	if (status != kWarpStatusOK)
	{
	}

	/*
	 *	Put the sensor in LOOP mode with ACT_EN and INACT_EN enabled
	 */
	status = writeSensorRegisterADXL362(	0x0A	/*	command == write register		*/,
						0x27	/*	ACT_INACT_CTL register			*/,
						0x35	/*	writeValue				*/,
						0	/*	numberOfAdditionalDummyBytes		*/
					);
	if (status != kWarpStatusOK)
	{
	}

	/*
	 *			Read X
	 */
	status = readSensorRegisterADXL362(kWarpSensorOutputRegisterADXL362XDATA_L, 2 /* numberOfBytes */);
	if (status != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		/*
		 *	Recall that the index [0] and [1] of the spiSinkBuffer are
		 *	bytes that were shifted in when sending out the instruction
		 *	and address bytes. We therefore look in indices [2] and [3].
		 */
		readSensorRegisterValueMSB = deviceADXL362State.spiSinkBuffer[2];
		readSensorRegisterValueLSB = deviceADXL362State.spiSinkBuffer[3];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

		/*
		 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
		 */
		readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}


	/*
	 *			Read Y
	 */
	status = readSensorRegisterADXL362(kWarpSensorOutputRegisterADXL362YDATA_L, 2 /* numberOfBytes */);
	if (status != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		/*
		 *	Recall that the index [0] and [1] of the spiSinkBuffer are
		 *	bytes that were shifted in when sending out the instruction
		 *	and address bytes. We therefore look in indices [2] and [3].
		 */
		readSensorRegisterValueMSB = deviceADXL362State.spiSinkBuffer[2];
		readSensorRegisterValueLSB = deviceADXL362State.spiSinkBuffer[3];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

		/*
		 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
		 */
		readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}


	/*
	 *			Read Z
	 */
	status = readSensorRegisterADXL362(kWarpSensorOutputRegisterADXL362ZDATA_L, 2 /* numberOfBytes */);
	if (status != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		/*
		 *	Recall that the index [0] and [1] of the spiSinkBuffer are
		 *	bytes that were shifted in when sending out the instruction
		 *	and address bytes. We therefore look in indices [2] and [3].
		 */
		readSensorRegisterValueMSB = deviceADXL362State.spiSinkBuffer[2];
		readSensorRegisterValueLSB = deviceADXL362State.spiSinkBuffer[3];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

		/*
		 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
		 */
		readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}


	/*
	 *			Read TEMP
	 */
	status = readSensorRegisterADXL362(kWarpSensorOutputRegisterADXL362TEMP_L, 2 /* numberOfBytes */);
	if (status != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		/*
		 *	Recall that the index [0] and [1] of the spiSinkBuffer are
		 *	bytes that were shifted in when sending out the instruction
		 *	and address bytes. We therefore look in indices [2] and [3].
		 */
		readSensorRegisterValueMSB = deviceADXL362State.spiSinkBuffer[2];
		readSensorRegisterValueLSB = deviceADXL362State.spiSinkBuffer[3];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

		/*
		 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
		 */
		readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

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