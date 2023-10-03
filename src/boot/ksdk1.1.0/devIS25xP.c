/*
	Authored 2020-2022. Phillip Stanley-Marbell, Orestis Kaparounakis.

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
#include "devIS25xP.h"

extern volatile WarpSPIDeviceState	deviceIS25xPState;
extern volatile uint32_t 			gWarpSpiTimeoutMicroseconds;
extern uint8_t 						gWarpSpiCommonSourceBuffer[];
extern uint8_t 						gWarpSpiCommonSinkBuffer[];
extern uint16_t 					gWarpBuffAddress;

uint8_t 	gFlashWriteLimit 		= 0x20;

uint8_t		deviceOpsBuffer[kWarpMemoryCommonSpiBufferBytes];

void initIS25xP(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts)
{
	deviceIS25xPState.chipSelectIoPinID 			= chipSelectIoPinID;
	deviceIS25xPState.spiSourceBuffer 				= gWarpSpiCommonSourceBuffer;
	deviceIS25xPState.spiSinkBuffer 				= gWarpSpiCommonSinkBuffer;
	deviceIS25xPState.spiBufferLength				= kWarpMemoryCommonSpiBufferBytes;
	deviceIS25xPState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	// PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	return;
}

WarpStatus
spiTransactionIS25xP(uint8_t ops[], size_t opCount)
{
	spi_status_t status;

	if (opCount > deviceIS25xPState.spiBufferLength)
	{
		return kWarpStatusBadDeviceCommand;
	}

	warpScaleSupplyVoltage(deviceIS25xPState.operatingVoltageMillivolts);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

	for (int i = 0; (i < opCount) && (i < deviceIS25xPState.spiBufferLength); i++)
	{
		deviceIS25xPState.spiSourceBuffer[i] = ops[i];
		deviceIS25xPState.spiSinkBuffer[i] = 0xFF;
	}

	/*
	 *	First, pulse the /CS to wake up device in case it is in Ultra-Deep Power-Down mode.
	 *	(See Section 10.2.1 of manual). The 50 milliseconds we use here is conservative:
	 *	the manual says the minimum time needed is 20ns. However, the datasheet also says:
	 *
	 *		"After the CS pin has been deasserted, the device will exit from the
	 *		Ultra-Deep Power-Down mode and return to the standby mode within a
	 *		maximum time of tXUDPD."
	 *
	 *	Since txudpd is 100us, we wait another millisecond before proceeding to assert /CS
	 *	again below.
	 */
	GPIO_DRV_ClearPinOutput(deviceIS25xPState.chipSelectIoPinID);
	OSA_TimeDelay(1);
	GPIO_DRV_SetPinOutput(deviceIS25xPState.chipSelectIoPinID);
	OSA_TimeDelay(1);

	/*
	 *	Next, create a falling edge on chip-select.
	 */
	GPIO_DRV_ClearPinOutput(deviceIS25xPState.chipSelectIoPinID);

	/*
	 *	The result of the SPI transaction will be stored in deviceADXL362State.spiSinkBuffer.
	 *	Providing a spi_master_user_config_t is optional since it is already provided when we did
	 *	SPI_DRV_MasterConfigureBus(), so we pass in NULL. The "master instance" is always 0 for
	 *	the KL03 since there is only one SPI peripheral.
	 */
	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0 /*	master instance			*/,
											NULL /*	spi_master_user_config_t	*/,
											(const uint8_t *restrict)deviceIS25xPState.spiSourceBuffer /*	source buffer			*/,
											(uint8_t *restrict)deviceIS25xPState.spiSinkBuffer /*	receive buffer			*/,
											opCount /*	transfer size			*/,
											gWarpSpiTimeoutMicroseconds);
	warpDisableSPIpins();

	/*
	 *	Deassert the AT45DB
	 */
	GPIO_DRV_SetPinOutput(deviceIS25xPState.chipSelectIoPinID);

	if (status != kStatus_SPI_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void enableIS25xPWrite()
{
	WarpStatus status;
	uint8_t ops[1] =
	{
			0x06, /* WREN */
	};
	status = spiTransactionIS25xP(ops, 1);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: Communication failed: %d", status);
	}
}

void disableIS25xPWrite()
{
	WarpStatus status;
	uint8_t ops[] =
	{
			0x04, /* WREN */
	};
	status = spiTransactionIS25xP(ops, 1);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: Communication failed: %d", status);
	}
}

WarpStatus
programSingleIS25xPWithoutOffsetUpdate(uint16_t* pageNumber_p, uint8_t* pageOffset_p, size_t nbyte, uint8_t *buf)
{
	// Assume we can fit into a single page.
	WarpStatus status;

	size_t nIterations = nbyte / gFlashWriteLimit;
	size_t excess = nbyte % gFlashWriteLimit;

	for (size_t i = 0; i < nIterations; i++)
	{
		status = programPageIS25xP(*pageNumber_p, *pageOffset_p, gFlashWriteLimit, buf + (i * gFlashWriteLimit));
		*pageOffset_p += gFlashWriteLimit;

		if (status != kWarpStatusOK)
		{
			warpPrint("\r\n\tError: programPageIS25xP failed");
			return status;
		}
	}

	if (excess > 0) {
		status = programPageIS25xP(*pageNumber_p, *pageOffset_p, excess, buf + (nIterations * gFlashWriteLimit));
		if (status != kWarpStatusOK)
		{
			warpPrint("\r\n\tError: programPageIS25xP failed");
			return status;
		}
		*pageOffset_p += excess;
	}

	return status;
}

WarpStatus
programMultipleIS25xPWithoutOffsetUpdate(uint16_t* pageNumber_p, uint8_t* pageOffset_p, size_t nbyte, uint8_t *buf)
{
	WarpStatus status;

	if (kWarpSizeIS25xPPageSizeBytes <= (size_t)(*pageOffset_p) + (size_t)(nbyte))
	{
		size_t nByteToWrite = kWarpSizeIS25xPPageSizeBytes - *pageOffset_p;
		status = programSingleIS25xPWithoutOffsetUpdate(pageNumber_p, pageOffset_p, nByteToWrite, buf);
		if (status != kWarpStatusOK)
		{
			warpPrint("\r\n\tError: programSingleIS25xPWithoutOffsetUpdate failed");
			return status;
		}
		*pageNumber_p += 1;
		*pageOffset_p = 0;

		status = programMultipleIS25xPWithoutOffsetUpdate(pageNumber_p, pageOffset_p, nbyte - nByteToWrite, buf + (nByteToWrite));
	} else
	{
		status = programSingleIS25xPWithoutOffsetUpdate(pageNumber_p, pageOffset_p, nbyte, buf);
		if (status != kWarpStatusOK)
		{
			warpPrint("\r\n\tError: programSingleIS25xPWithoutOffsetUpdate failed");
			return status;
		}
	}

	return kWarpStatusOK;
}

WarpStatus
writeToIS25xPFromEnd(size_t nbyte, uint8_t *buf)
{
	// assume that nbyte < 32
	WarpStatus status;
	uint8_t pageOffsetBuf[3];
	status = readMemoryIS25xP(kWarpIS25xPPageOffsetStoragePage, kWarpIS25xPPageOffsetStorageOffset, kWarpIS25xPPageOffsetStorageSize, pageOffsetBuf);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: readMemoryIS25xP failed");
		return status;
	}

	uint8_t pageOffset = pageOffsetBuf[2];
	uint16_t pageNumber = pageOffsetBuf[1] | pageOffsetBuf[0] << 8;

	status = programMultipleIS25xPWithoutOffsetUpdate(&pageNumber, &pageOffset, nbyte, buf);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: programMultipleIS25xPWithoutOffsetUpdate failed");
		return status;
	}

	status = programPageNumberAndOffset(pageNumber, pageOffset);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: programPageNumberAndOffset failed");
		return status;
	}

	return kWarpStatusOK;
}

void
byteOffsetToAddress(uint32_t byteOffset, uint8_t *address)
{
	address[2] = (uint8_t)(byteOffset);
	address[1] = (uint8_t)(byteOffset >>= 8);
	address[0] = (uint8_t)(byteOffset >>= 8);
}

WarpStatus
programPageNumberAndOffset(uint16_t pageNumber, uint8_t pageOffset)
{
	WarpStatus status;
	status = eraseSectorIS25xP(0);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError:raseSectorIS25xP failed");
		return status;
	}

	uint8_t address[3];
	address[1] = (uint8_t)(pageNumber);
	address[0] = (uint8_t)(pageNumber >> 8);
	address[2] = pageOffset;


	status = programPageIS25xP(0, 0, 3, address);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: programPageIS25xP failed");
		return status;
	}
	return kWarpStatusOK;
}

WarpStatus
resetIS25xP()
{
	WarpStatus status;
	status = chipEraseIS25xP();
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: chipEraseIS25xP failed");
		return status;
	}

	status = programPageNumberAndOffset(kWarpInitialPageNumberIS25xP, kWarpInitialPageOffsetIS25xP);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: programPageNumberAndOffset failed");
		return status;
	}

	return kWarpStatusOK;
}

// WarpStatus getCurrentPage

WarpStatus readAllMemoryIS25xP()
{
	WarpStatus status;

	uint8_t pageOffsetBuf[3];
	status = readMemoryIS25xP(kWarpIS25xPPageOffsetStoragePage, kWarpIS25xPPageOffsetStorageOffset, kWarpIS25xPPageOffsetStorageSize, pageOffsetBuf);
	if (status != kWarpStatusOK)
	{
		return status;
	}

	uint8_t pageOffset = pageOffsetBuf[2];
	uint16_t pageNumber = pageOffsetBuf[1] | pageOffsetBuf[0] << 8;

	warpPrint("\r\n\tPage number: %d\t page offset: %d\n", pageNumber, pageOffset);

	uint32_t startAddress = kWarpInitialPageOffsetIS25xP | kWarpInitialPageNumberIS25xP << 8;

	uint32_t endAddress = pageOffset | pageNumber << 8;

	size_t n_iterations = (endAddress - startAddress) / gFlashWriteLimit;
	size_t excess = (endAddress - startAddress) % gFlashWriteLimit;

	uint8_t ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x03; /* NORD */

	warpPrint("Data start\n");
	for (int i = 0; i < n_iterations; i++) {
		ops[3] = (uint8_t)(startAddress);
		ops[2] = (uint8_t)(startAddress >> 8);
		ops[1] = (uint8_t)(startAddress >> 16);
		status = spiTransactionIS25xP(ops, kWarpMemoryCommonSpiBufferBytes);
		if (status != kWarpStatusOK)
		{
			return status;
		}
		for (size_t i = 0; i < gFlashWriteLimit; i++) {
			warpPrint("%c", deviceIS25xPState.spiSinkBuffer[i + 4]);
		}

		startAddress += gFlashWriteLimit;
	}

	ops[3] = (uint8_t)(startAddress);
	ops[2] = (uint8_t)(startAddress >> 8);
	ops[1] = (uint8_t)(startAddress >> 16);
	status = spiTransactionIS25xP(ops, kWarpMemoryCommonSpiBufferBytes);
	if (status != kWarpStatusOK)
	{
		return status;
	}
	for (size_t i = 0; i < excess; i++) {
		warpPrint("%c", deviceIS25xPState.spiSinkBuffer[i + 4]);
	}
	warpPrint("Data end\n");
	return kWarpStatusOK;
}

WarpStatus
readMemoryIS25xP(uint16_t startPageNumber, uint8_t startPageOffset, size_t nbyte, void *buf)
{
	WarpStatus status;

	if (nbyte > kWarpSizeAT45DBPageSizeBytes)
	{
		return kWarpStatusBadDeviceCommand;
	}

	size_t 		nBytesBeingRead;
	uint16_t 	nextPageNumber;
	uint8_t 	nextPageOffset;
	// warpPrint("\nReading %d bytes from page %d, offset %d\n", nbyte, startPageNumber, startPageOffset);

	size_t nBytesRemainingInPage 	= kWarpSizeAT45DBPageSizeBytes - startPageOffset;
	size_t nBytesSpiLimit 			= kWarpMemoryCommonSpiBufferBytes - 4;
	// warpPrint("nBytesRemaining: %d, nByte: %d\n", nBytesRemainingInPage, nbyte);

	if (nBytesRemainingInPage < nbyte)
	{
		// warpPrint("nBytesRemainingInPage < nbyte\n");
		if (nBytesRemainingInPage > nBytesSpiLimit)
		{
			nBytesBeingRead = nBytesSpiLimit;
			nextPageNumber 	= startPageNumber;
			nextPageOffset 	= startPageOffset + nBytesSpiLimit;
		} else
		{
			nBytesBeingRead = nBytesRemainingInPage;
			nextPageNumber 	= startPageNumber + 1;
			nextPageOffset 	= 0;
		}
	} else
	{
		// warpPrint("nBytesRemainingInPage >= nbyte\n");
		if (nbyte > nBytesSpiLimit)
		{
			nBytesBeingRead = nBytesSpiLimit;
			nextPageNumber 	= startPageNumber;
			nextPageOffset 	= startPageOffset + nBytesSpiLimit;
		} else
		{
			nBytesBeingRead = nbyte;
			nextPageNumber 	= startPageNumber;
			nextPageOffset 	= startPageOffset + nbyte;
		}
	}
	// warpPrint("nBytesBeingRead: %d\n", nBytesBeingRead);

	size_t nBytesRemaining	= nbyte - nBytesBeingRead;
	// warpPrint("nBytesRemaining: %d\n", nBytesRemaining);

	deviceOpsBuffer[0] = 0x03; /* NORD */
	deviceOpsBuffer[2] = (uint8_t)(startPageNumber);
	deviceOpsBuffer[1] = (uint8_t)(startPageNumber >> 8);
	deviceOpsBuffer[3] = startPageOffset;

	status = spiTransactionIS25xP(deviceOpsBuffer, nBytesBeingRead + 4);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: communication failed");
		return status;
	}

	for (size_t i = 0; i < nBytesBeingRead; i++)
	{
		((uint8_t *)buf)[i] = deviceIS25xPState.spiSinkBuffer[i + 4];
	}

	if (nBytesRemaining == 0)
	{
		return kWarpStatusOK;

	}
	return readMemoryIS25xP(nextPageNumber, nextPageOffset, nBytesRemaining, (uint8_t *)buf + nBytesBeingRead);
}

WarpStatus
programPageIS25xP(uint16_t startPageAddress, uint8_t startPageOffset, size_t nbyte, uint8_t *buf)
{
	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x02; /* PP */
	ops[2] = (uint8_t)(startPageAddress);
	ops[1] = (uint8_t)(startPageAddress >>= 8);
	ops[3] = startPageOffset;

	for (size_t i = 0; i < nbyte; i++)
	{
		ops[i + 4] = buf[i];
	}

	WarpStatus status;

	enableIS25xPWrite();
	status = spiTransactionIS25xP(ops, nbyte + 4);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: communication failed");
		return status;
	}

	status = waitForWriteCompletion();
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: waitForWriteCompletion failed");
		return status;
	}

	disableIS25xPWrite();

	return status;
}


WarpStatus
eraseSectorIS25xP(uint32_t address)
{
	WarpStatus status;

	uint8_t ops[4] = {0};
	ops[0] = 0xD7; /* SER (SPI Mode) */
	ops[2] = (uint8_t)(address <<= 1);
	ops[1] = (uint8_t)(address >>= 8);

	ops[3] = 0x00;

	enableIS25xPWrite();
	status = spiTransactionIS25xP(ops, 4);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: communication failed");
		return status;
	}

	status = waitForWriteCompletion();
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: waitForWriteCompletion failed");
		return status;
	}
	return kWarpStatusOK;
}

WarpStatus
erase32kBlockIS25xP(uint32_t address)
{
	WarpStatus status;

	uint8_t ops[4] = {0};
	ops[0] = 0x52; /* BER32K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	status = spiTransactionIS25xP(ops, 4);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: communication failed");
		return status;
	}
	return kWarpStatusOK;
}

WarpStatus
erase64kBlockIS25xP(uint32_t address)
{
	WarpStatus status;

	uint8_t ops[4] = {0};
	ops[0] = 0xD8; /* BER64K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	status = spiTransactionIS25xP(ops, 4);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: communication failed");
		return status;
	}
	return kWarpStatusOK;
}

WarpStatus
chipEraseIS25xP()
{
	warpPrint("\n%s\n", "Erasing chip. This takes about 4s.");
	enableIS25xPWrite();
	uint8_t ops[1] = {0};

	ops[0] = 0xC7; /* CER (SPI Mode) */

	WarpStatus status;
	status = spiTransactionIS25xP(ops, 1);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: communication failed");
		return status;
	}
	status = waitForWriteCompletion();
	warpPrint("%s\n", "Chip erased");

	return status;
}

WarpStatus
flashStatusIS25xP()
{
	uint8_t ops4[2] = {
			/* Read Status Register */
			0x05, /* Byte0 */
			0x00, /* Dummy Byte1 */
	};
	WarpStatus status = spiTransactionIS25xP(ops4, 2 /* opCount */);
	if (status != kWarpStatusOK)
	{
		warpPrint("SPI transaction to read Flash ID failed...\n");
	}
	else
	{
		warpPrint("Status = [" BYTE_TO_BINARY_PATTERN "]\n",
							BYTE_TO_BINARY(deviceIS25xPState.spiSinkBuffer[1]));
	}
	return kWarpStatusOK;
}

WarpStatus
waitForWriteCompletion()
{
	uint8_t ops4[] = {
			/* Read Status Register */
			0x05, /* Byte0 */
			0x00, /* Dummy Byte1 */
	};

	WarpStatus status;
	status = spiTransactionIS25xP(ops4, sizeof(ops4) / sizeof(uint8_t) /* opCount */);
	if (status != kWarpStatusOK)
	{
		warpPrint("\r\n\tError: communication failed");
		return status;
	}

	while ((deviceIS25xPState.spiSinkBuffer[1] & 0x1))
	{
		OSA_TimeDelay(1);

		status = spiTransactionIS25xP(ops4, sizeof(ops4) / sizeof(uint8_t) /* opCount */);
		if (status != kWarpStatusOK)
		{
			warpPrint("\r\n\tError: communication failed");
			return status;
		}
	}
	return kWarpStatusOK;
}