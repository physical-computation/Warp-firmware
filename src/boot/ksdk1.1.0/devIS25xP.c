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
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;
extern uint8_t				gWarpSpiCommonSourceBuffer[];
extern uint8_t				gWarpSpiCommonSinkBuffer[];
extern uint8_t				gWarpWriteToFlash;


void
initIS25xP(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts)
{
	deviceIS25xPState.chipSelectIoPinID		= chipSelectIoPinID;
	deviceIS25xPState.spiSourceBuffer		= gWarpSpiCommonSourceBuffer;
	deviceIS25xPState.spiSinkBuffer			= gWarpSpiCommonSinkBuffer;
	deviceIS25xPState.spiBufferLength		= kWarpMemoryCommonSpiBufferBytes;
	deviceIS25xPState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	// PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	return;
}

WarpStatus
spiTransactionIS25xP(uint8_t ops[], size_t opCount)
{
	spi_status_t	status;

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
	 *	Create a falling edge on chip-select.
	 */
	// PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	GPIO_DRV_SetPinOutput(deviceIS25xPState.chipSelectIoPinID);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(deviceIS25xPState.chipSelectIoPinID);

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
					(const uint8_t * restrict)deviceIS25xPState.spiSourceBuffer,
					(uint8_t * restrict)deviceIS25xPState.spiSinkBuffer,
					opCount /* transfer size */,
					gWarpSpiTimeoutMicroseconds);
	warpDisableSPIpins();

	/*
	 *	Deassert the IS25xP
	 */
	GPIO_DRV_SetPinOutput(deviceIS25xPState.chipSelectIoPinID);

	if (status != kStatus_SPI_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}
void
enableIs25xPWrite() {
  WarpStatus status;
  uint8_t ops[] = {
      0x06, /* WREN */
  };
  status = spiTransactionIS25xP(ops, 1);
  if (status != kWarpStatusOK) {
    warpPrint("\r\n\tCommunication failed: %d", status);
  }
}
WarpStatus
ProgramIS25xP(size_t nbyte, uint8_t* buf) {
  // assume that nbyte < 60
  int writeToFlash = gWarpWriteToFlash;
  gWarpWriteToFlash = kWarpWriteToFlash;

  uint8_t pageOffsetBuf[3];
  readMemoryIS25xP(0, 3, pageOffsetBuf);

  uint8_t pageOffset = pageOffsetBuf[2];
  uint16_t pageNumber = pageOffsetBuf[1] | pageOffsetBuf[0] << 8;

  bool nextPageRequired = nbyte + pageOffset > kWarpSizeAT45DBPageSizeBytes;

  WarpStatus status;
  if (!nextPageRequired) {
    uint8_t fullBufSize = nbyte + pageOffset;
    uint8_t fullBuf[fullBufSize];

    if (pageOffset > 0) {
      status = readMemoryIS25xP(pageNumber, pageOffset, fullBuf);
      if (status != kWarpStatusOK) {
        warpPrint("Error: ReadIs25xP failed\n");
        return status;
      }
    }

    for (int i = 0; i < nbyte; i++) {
      fullBuf[pageOffset + i] = buf[i];
    }

    status = programPageIS25xP(pageNumber, fullBufSize, fullBuf);

    resetIS25xP(pageNumber, fullBufSize);
  }
  else {
    uint8_t firstBufSize = kWarpSizeAT45DBPageSizeBytes - pageOffset;
    uint8_t firstBuf[kWarpSizeAT45DBPageSizeBytes];

    if (pageOffset > 0) {
      status = readMemoryIS25xP(pageNumber, pageOffset, firstBuf);
      if (status != kWarpStatusOK) {
        warpPrint("Error: ReadIS25xP failed\n");
        return status;
      }
    }

    for (int i = 0; i < firstBufSize; i++) {
      firstBuf[pageOffset + i] = buf[i];
    }

    status = programPageIS25xP(pageNumber, kWarpSizeAT45DBPageSizeBytes, firstBuf);
    if (status != kWarpStatusOK) {
      warpPrint("Error: PageProgramIs25xP failed\n");
      return status;
    }

    uint8_t middleBufSize = nbyte - firstBufSize;
    size_t nIterations = middleBufSize / kWarpSizeAT45DBPageSizeBytes;

    size_t excess = middleBufSize % kWarpSizeAT45DBPageSizeBytes;
    uint8_t* middleBuf = buf + firstBufSize;

    for (int i = 0; i < nIterations; i++) {
      status = programPageIS25xP((pageNumber+=1), kWarpSizeAT45DBPageSizeBytes, middleBuf+(i*kWarpSizeAT45DBPageSizeBytes));
      if (status != kWarpStatusOK) {
        warpPrint("Error: PageProgramIs25xP failed\n");
        return status;
      }
    }

    status = programPageIS25xP((pageNumber+=1), excess, middleBuf+(nIterations*kWarpSizeAT45DBPageSizeBytes));
      if (status != kWarpStatusOK) {
        warpPrint("Error: PageProgramIs25xP failed\n");
        return status;
      }

    resetIS25xP(pageNumber, excess);
  }

  gWarpWriteToFlash = writeToFlash;
  return kWarpStatusOK;
}

void
resetIS25xP(uint32_t pageNumber, uint8_t pageOffset)
{
  enableIs25xPWrite();

  WarpStatus status;

  uint8_t initialNANDStartPosition[4];
  initialNANDStartPosition[2] = (uint8_t)(pageNumber);
  initialNANDStartPosition[1] = (uint8_t)(pageNumber>>=8);
  initialNANDStartPosition[0] = (uint8_t)(pageNumber>>=8);
  initialNANDStartPosition[3] = pageOffset;

  status = programPageIS25xP(0, 4, initialNANDStartPosition);
  if (status != kWarpStatusOK) {
     warpPrint("Error: PageProgramIS25xP failed\n");
  }
}

WarpStatus
readMemoryIS25xP(uint32_t pageNumber, size_t nbyte, void *  buf)
{

	WarpStatus	status;
  if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
  {
    return kWarpStatusBadDeviceCommand;
  }

  uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
  ops[0] = 0x03;	/* NORD */
  ops[3] = (uint8_t)(pageNumber);
	ops[2] = (uint8_t)(pageNumber >>= 8);
	ops[1] = (uint8_t)(pageNumber >>= 8);
	// ops[4] = 0x00;
	// ops[5] = 0x00;
	// ops[6] = 0x00;
	// ops[7] = 0x00;

  status = spiTransactionIS25xP(ops, nbyte+4);
	if (status != kWarpStatusOK) {
		return status;
	}

  for (size_t i = 0; i < nbyte; i++)
  {
    ((uint8_t*)buf)[i] = deviceIS25xPState.spiSinkBuffer[i+8];

  }

  return kWarpStatusOK;




	// WarpStatus	status;


	// if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	// {
	// 	return kWarpStatusBadDeviceCommand;
	// }

	// size_t nIterations = nbyte / (kWarpMemoryCommonSpiBufferBytes - 8);
	// size_t excessBytes = nbyte % (kWarpMemoryCommonSpiBufferBytes - 8);

	// for (size_t i = 0; i < nIterations; i++)
	// {
	// 	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	// 	ops[0] = 0x03;	/* NORD */
	// 	ops[3] = (uint8_t)(pageNumber);
	// 	ops[2] = (uint8_t)(pageNumber >>= 8);
	// 	ops[1] = (uint8_t)(pageNumber >>= 8);
	// 	ops[4] = 0x00;
	// 	ops[5] = 0x00;
	// 	ops[6] = 0x00;
	// 	ops[7] = 0x00;


	// 	// ops[2] = (uint8_t)(pageNumber <<= 1);
	// 	// ops[1] = (uint8_t)(pageNumber >>= 8);
	// 	// ops[3] = 0x00;
	// 	// ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	// 	// ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	// 	// ops[3] = (uint8_t)((startAddress & 0x000F));
	// 	// ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	// 	// ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	// 	// ops[3] = (uint8_t)((startAddress & 0x000F));

	// 	status = spiTransactionIS25xP(ops, nbyte+4);
	// 	if (status != kWarpStatusOK)
	// 	{
	// 		return status;
	// 	}

	// 	// for (size_t i = 0; i < nbyte; i++)
	// 	// {
	// 	// 	((uint8_t*)buf)[i] = deviceIS25xPState.spiSinkBuffer[i+4];
	// 	// }
	// 	for (size_t j = 0; j < kWarpMemoryCommonSpiBufferBytes - 8; j++)
	// 		{
	// 			((uint8_t*)buf)[i*(kWarpMemoryCommonSpiBufferBytes - 8) + j] = deviceIS25xPState.spiSinkBuffer[i+8];
	// 		}
	// }
	// for (size_t i = 0; i < nIterations; i++)
	// {
	// 	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	// 	ops[0] = 0x03;	/* NORD */
	// 	ops[3] = (uint8_t)(pageNumber);
	// 	ops[2] = (uint8_t)(pageNumber >>= 8);
	// 	ops[1] = (uint8_t)(pageNumber >>= 8);
	// 	// ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	// 	// ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	// 	// ops[3] = (uint8_t)((startAddress & 0x000F));


	// 	status = spiTransactionIS25xP(ops, kWarpMemoryCommonSpiBufferBytes);
	// 	if (status != kWarpStatusOK)
	// 	{
	// 		return status;
	// 	}

	// 	for (size_t j = 0; j < kWarpMemoryCommonSpiBufferBytes - 8; j++)
	// 	{
	// 		((uint8_t*)buf)[i*(kWarpMemoryCommonSpiBufferBytes - 8) + j] = deviceIS25xPState.spiSinkBuffer[i+8];
	// 	}

	// }

	// uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	// ops[0] = 0x03;	/* NORD */
	// ops[3] = (uint8_t)(pageNumber);
	// 	ops[2] = (uint8_t)(pageNumber >>= 8);
	// 	ops[1] = (uint8_t)(pageNumber >>= 8);
	// // ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	// // ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	// // ops[3] = (uint8_t)((startAddress & 0x000F));


	// status = spiTransactionIS25xP(ops, excessBytes + 8);
	// if (status != kWarpStatusOK)
	// {
	// 	return status;
	// }

	// for (size_t i = 0; i < excessBytes; i++)
	// {
	// 	((uint8_t*)buf)[nIterations*(kWarpMemoryCommonSpiBufferBytes - 8) + i] = deviceIS25xPState.spiSinkBuffer[i+8];
	// }

	// return kWarpStatusOK;

	// // WarpStatus	status;
	// // if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	// // {
	// // 	return kWarpStatusBadDeviceCommand;
	// // }

	// // uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	// // ops[0] = 0x03;	/* NORD */
	// // ops[2] = (uint8_t)(pageNumber <<= 1);
	// // ops[1] = (uint8_t)(pageNumber >>= 8);
	// // ops[3] = 0x00;

	// // status = spiTransactionIS25xP(ops, nbyte+4);
	// // if (status != kWarpStatusOK)
	// // {
	// // 	return status;
	// // }

	// // for (size_t i = 0; i < nbyte; i++)
	// // {
	// // 	((uint8_t*)buf)[i] = deviceIS25xPState.spiSinkBuffer[i+4];
	// // }

	// // return kWarpStatusOK;
}

WarpStatus
programPageIS25xP(uint32_t pageNumber, size_t nbyte, uint8_t *  buf)
{


	WarpStatus	status;

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x02;	/* NORD */
	ops[3] = (uint8_t)(pageNumber);
	ops[2] = (uint8_t)(pageNumber >>= 8);
	ops[1] = (uint8_t)(pageNumber >>= 8);

	if (status != kWarpStatusOK)
	{
		return status;
	}

	for (size_t i = 0; i < nbyte; i++)
	{
		ops[i+4] = buf[i];
	}
	return  spiTransactionIS25xP(ops, nbyte + 4);

}

WarpStatus
eraseSectorIS25xP(uint32_t address)
{
	uint8_t	ops[4] = {0};

	ops[0] = 0xD7;	/* SER (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionIS25xP(ops, 4);
}

WarpStatus
erase32kBlockIS25xP(uint32_t address)
{
	uint8_t	ops[4] = {0};

	ops[0] = 0x52;	/* BER32K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionIS25xP(ops, 4);
}

WarpStatus
erase64kBlockIS25xP(uint32_t address)
{
	uint8_t	ops[4] = {0};

	ops[0] = 0xD8;	/* BER64K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionIS25xP(ops, 4);
}

WarpStatus
chipEraseIS25xP()
{
	uint8_t	ops[1] = {0};

	ops[0] = 0xC7;	/* CER (SPI Mode) */

	return spiTransactionIS25xP(ops, 1);
}
