/*
	Authored 2021. Phillip Stanley-Marbell.

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
#include "devAT45DB.h"

extern volatile WarpSPIDeviceState	deviceAT45DBState;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;
extern uint8_t				gWarpSpiCommonSourceBuffer[];
extern uint8_t				gWarpSpiCommonSinkBuffer[];

 

void
initAT45DB(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts)
{
	deviceAT45DBState.chipSelectIoPinID		= chipSelectIoPinID;
	deviceAT45DBState.spiSourceBuffer		= gWarpSpiCommonSourceBuffer;
	deviceAT45DBState.spiSinkBuffer		= gWarpSpiCommonSinkBuffer;
	deviceAT45DBState.spiBufferLength		= kWarpMemoryCommonSpiBufferBytes;
	deviceAT45DBState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
spiTransactionAT45DB(WarpSPIDeviceState volatile *  deviceStatePointer, uint8_t ops[], size_t opCount)
{
	spi_status_t	status;


	warpScaleSupplyVoltage(deviceAT45DBState.operatingVoltageMillivolts);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

	for (int i = 0; (i < opCount) && (i < deviceAT45DBState.spiBufferLength); i++)
	{
		deviceAT45DBState.spiSourceBuffer[i] = ops[i];
		deviceAT45DBState.spiSinkBuffer[i] = 0xFF;
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
	GPIO_DRV_ClearPinOutput(deviceAT45DBState.chipSelectIoPinID);
	OSA_TimeDelay(1);
	GPIO_DRV_SetPinOutput(deviceAT45DBState.chipSelectIoPinID);
	OSA_TimeDelay(1);

	/*
	 *	Next, create a falling edge on chip-select.
	 */
	GPIO_DRV_ClearPinOutput(deviceAT45DBState.chipSelectIoPinID);

	/*
	 *	The result of the SPI transaction will be stored in deviceADXL362State.spiSinkBuffer.
	 *	Providing a spi_master_user_config_t is optional since it is already provided when we did
	 *	SPI_DRV_MasterConfigureBus(), so we pass in NULL. The "master instance" is always 0 for
	 *	the KL03 since there is only one SPI peripheral.
	 */
	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0							/*	master instance			*/,
					NULL								/*	spi_master_user_config_t	*/,
					(const uint8_t * restrict)deviceAT45DBState.spiSourceBuffer	/*	source buffer			*/,
					(uint8_t * restrict)deviceAT45DBState.spiSinkBuffer		/*	receive buffer			*/,
					opCount								/*	transfer size			*/,
					gWarpSpiTimeoutMicroseconds);
	warpDisableSPIpins();

	/*
	 *	Deassert the AT45DB
	 */
	GPIO_DRV_SetPinOutput(deviceAT45DBState.chipSelectIoPinID);

	if (status != kStatus_SPI_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}
WarpStatus
readBufferAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  bufd)
{
	WarpStatus	status;

	// if (NOT RUN MODE)
	// {
	// 	return kWarpStatusBadPowerModeSpecified;
	// }

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0xD4;	/* NORD */
	
	ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	ops[3] = (uint8_t)((startAddress & 0x000F));
	//ops[4] = (uint8_t)((startAddress & 0x0000));
	//warpPrint("op0 op1 op2 op3 %d, %d, %d, %d\n", ops[0], ops[1], ops[2], ops[3]);

	return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+4);

	if (status != kWarpStatusOK)
	{
		return status;
	}

	for (size_t i = 0; i < nbyte; i++)
	{
		//ops[0] = 0x0B;	/* NORD */
		//return spiTransactionAT45DB(&deviceAT45DBState, ops, 1);
		((uint8_t*)bufd)[i] = deviceAT45DBState.spiSinkBuffer[i+4];
	}
	// for (size_t i = 0; i < nbyte; i++)
	// {
	// 	warpPrint("%c", buf[i]);
	// 	OSA_TimeDelay(5);
	// }

	return kWarpStatusOK;
}
WarpStatus
readPageAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	WarpStatus	status;

	// if (NOT RUN MODE)
	// {
	// 	return kWarpStatusBadPowerModeSpecified;
	// }

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x1B;	/* NORD */
	
	ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	ops[3] = (uint8_t)((startAddress & 0x000F));
	//ops[4] = 0x00;
	//warpPrint("op0 op1 op2 op3 %d, %d, %d, %d\n", ops[0], ops[1], ops[2], ops[3]);

	return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+4);

	if (status != kWarpStatusOK)
	{
		return status;
	}

	for (size_t i = 0; i < nbyte; i++)
	{
		((uint8_t*)buf)[i] = deviceAT45DBState.spiSinkBuffer[i+4];
	}
	// for (size_t i = 0; i < nbyte; i++)
	// {
	// 	warpPrint("%c", buf[i]);
	// 	OSA_TimeDelay(5);
	// }

	return kWarpStatusOK;
}
WarpStatus
writeDataToBuffer(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	
	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x84;	/* PP */
	ops[1] = (uint8_t)((0x00 & 0x0F00) >> 2);
	ops[2] = (uint8_t)((0x00 & 0x00F0) >> 1);
	ops[3] = (uint8_t)((0x00 & 0x000F));
	for (size_t i = 0; i < nbyte; i++)
	{
		ops[i+4] = ((uint8_t*)buf)[i];
	}

	return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+4);

	// for (uint16_t i = 0; i < nbyte; i++)
    // {
    //     // Send the "Data Load" command and the current data byte to the chip
	// ops[0] = 0x53;	/* PP */
	// ops[1] = (uint8_t)((0x00 & 0x0F00) >> 2);
	// ops[2] = (uint8_t)((0x00 & 0x00F0) >> 1);
	// ops[3] = (uint8_t)((0x00 & 0x000F));
	// for (size_t i = 0; i < nbyte; i++)
	// {
	// 	ops[i+4] = ((uint8_t*)buf)[i];
	// }

	// return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+4);

        // Wait for the write to complete

        // Increment the address counter for the next byte
       // startAddress++;
    //}
	ops[0] = 0x83;	/* PP */
	ops[1] = (uint8_t)((0x00 & 0x0F00) >> 2);
	ops[2] = (uint8_t)((0x00 & 0x00F0) >> 1);
	ops[3] = (uint8_t)((0x00 & 0x000F));
	for (size_t i = 0; i < nbyte; i++)
	{
		ops[i+4] = ((uint8_t*)buf)[i];
	}

	return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+4);
}
WarpStatus 
writeDataFromBufferToPage(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x88;	/* PP */
	ops[1] = (uint8_t)((0x00 & 0x0F00) >> 2);
	ops[2] = (uint8_t)((0x00 & 0x00F0) >> 1);
	ops[3] = (uint8_t)((0x00 & 0x000F));
	for (size_t i = 0; i < nbyte; i++)
	{
		ops[i+4] = ((uint8_t*)buf)[i];
	}

	return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+4);
}

WarpStatus
eraseSectorAT45DB(uint32_t address)
{
	uint8_t	ops[4] = {0};

	ops[0] = 0x7C;	/* SER (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
}

WarpStatus
erase32kBlockAT45DB(uint32_t address)
{
	uint8_t	ops[4] = {0};

	ops[0] = 0x12;	/* BER32K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
}

WarpStatus
erasePageAT45DB(uint32_t address)
{
	uint8_t	ops[4] = {0};

	ops[0] = 0x81;	/* BER64K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
}

WarpStatus
chipEraseAT45DB()
{
	uint8_t	ops[4] = {0};

	ops[0] = 0xC7;	/* CER (SPI Mode) */
	ops[1] = 0x94;
	ops[2] = 0x80;
	ops[3] = 0x9A;

	return spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
}

 WarpStatus 
 at45db_bread(uint32_t sblock, size_t nblocks, uint8_t *buf)
{
    size_t nb;

    nb = readPageAT45DB(sblock,
                     nblocks, buf);

    return nb;
}
WarpStatus
send_command(uint8_t opcode, uint32_t address, uint8_t data) 
{
    //GPIO_DRV_ClearPinOutput(deviceAT45DBState.chipSelectIoPinID);
	uint8_t	ops[4] = {0};

    // Send the command and address bytes over SPI
	ops[0] = opcode;	/* BER64K (SPI Mode) */
	ops[1] = (uint8_t)((address >> 16) & 0xFF);
	ops[2] = (uint8_t)((address >> 8) & 0xFF);
	ops[3] = (uint8_t)(address & 0xFF);

    // Send the data byte over SPI
    return spiTransactionAT45DB(&deviceAT45DBState, ops, 4);

       //GPIO_DRV_SetPinOutput(deviceAT45DBState.chipSelectIoPinID);

}

WarpStatus write_to_buffer1(uint32_t address, uint8_t* data, uint16_t length)
 {
    // First, erase the sector containing the buffer
    //send_command(0x81, address & 0xFFFF0000, 0x50);

    //Then, write the data byte by byte to the buffer
    for (uint16_t i = 0; i < length; i++) {
        send_command(0x84, address + i, data[i]);
    }
}

void read_from_buffer1(uint32_t address, uint8_t* buffer, uint16_t length)
 {
    // Send the Read from Buffer 1 command to the chip
    send_command(0xD1, address & 0xFFFF0000, 0x00);

    // Read the data byte by byte from the buffer
	for (size_t i = 0; i < length; i++)
	{
		((uint8_t*)buffer)[i] = deviceAT45DBState.spiSinkBuffer[i+5];
	}
    // for (uint16_t i = 0; i < length; i++) {
    //     buffer[i] = spiTransactionAT45DB(&deviceAT45DBState, 0x00, 1);;
    // }
}

WarpStatus
MainMemoryPageReadAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	WarpStatus	status;


	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[8] = {0};
	ops[0] = 0xD2;	
	ops[1] = (uint8_t)((startAddress >> 16) & 0xFF);
	ops[2] = (uint8_t)((startAddress >> 8) & 0xFF);
	ops[3] = (uint8_t)((startAddress & 0xFF));
	ops[4] = 0x00;
	ops[5] = 0x00;
	ops[6] = 0x00;
	ops[7] = 0x00;

	spiTransactionAT45DB(&deviceAT45DBState, ops, 8);

	if (status != kWarpStatusOK)
	{
		return status;
	}

	for (size_t i = 0; i < nbyte; i++)
	{
		(buf)[i] = deviceAT45DBState.spiSinkBuffer[i+8];
	}
	

	return kWarpStatusOK;
}

WarpStatus
MainMemoryPageProgramAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	WarpStatus	status;


	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x82;
	ops[1] = (uint8_t)((startAddress >> 16) & 0xFF);
	ops[2] = (uint8_t)((startAddress >> 8) & 0xFF);
	ops[3] = (uint8_t)((startAddress & 0xFF));
	

	for (size_t i = 0; i < nbyte; i++)
	{
		ops[i+4] = ((uint8_t*)buf)[i];
	}

	return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+4);
	

	return kWarpStatusOK;
}
