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
//################################################################################################################
extern volatile WarpSPIDeviceState	deviceAT45DBState;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;
extern uint8_t				gWarpSpiCommonSourceBuffer[];
extern uint8_t				gWarpSpiCommonSinkBuffer[];
//################################################################################################################
/* Read commands */
#define AT45DB_RDMN          0xd2 /* Main Memory Page Read */
#define AT45DB_RDARRY        0xe8 /* Continuous Array Read (Legacy Command) */
#define AT45DB_RDARRAYLF     0x03 /* Continuous Array Read (Low Frequency) */
#define AT45DB_RDARRAYHF     0x0b /* Continuous Array Read (High Frequency) */
#define AT45DB_RDBF1LF       0xd1 /* Buffer 1 Read (Low Frequency) */
#define AT45DB_RDBF2LF       0xd3 /* Buffer 2 Read (Low Frequency) */
#define AT45DB_RDBF1         0xd4 /* Buffer 1 Read */
#define AT45DB_RDBF2         0xd6 /* Buffer 2 Read */

/* Program and Erase Commands */
#define AT45DB_WRBF1         0x84 /* Buffer 1 Write */
#define AT45DB_WRBF2         0x87 /* Buffer 2 Write */
#define AT45DB_BF1TOMNE      0x83 /* Buffer 1 to Main Memory Page Program with Built-in Erase */
#define AT45DB_BF2TOMNE      0x86 /* Buffer 2 to Main Memory Page Program with Built-in Erase */
#define AT45DB_BF1TOMN       0x88 /* Buffer 1 to Main Memory Page Program without Built-in Erase */
#define AT45DB_BF2TOMN       0x89 /* Buffer 2 to Main Memory Page Program without Built-in Erase  */
#define AT45DB_PGERASE       0x81 /* Page Erase */
#define AT45DB_BLKERASE      0x50 /* Block Erase */
#define AT45DB_SECTERASE     0x7c /* Sector Erase */
#define AT45DB_CHIPERASE1    0xc7 /* Chip Erase - byte 1 */
#define AT45DB_CHIPERASE2  0x94 /* Chip Erase - byte 2 */
#define AT45DB_CHIPERASE3  0x80 /* Chip Erase - byte 3 */
#define AT45DB_CHIPERASE4  0x9a /* Chip Erase - byte 4 */
#define AT45DB_MNTHRUBF1     0x82 /* Main Memory Page Program Through Buffer 1 */
#define AT45DB_MNTHRUBF2     0x85 /* Main Memory Page Program Through Buffer 2 */

/* Protection and Security Commands */
#define AT45DB_ENABPROT1     0x3d /* Enable Sector Protection - byte 1 */
#define AT45DB_ENABPROT2   0x2a /* Enable Sector Protection - byte 2 */
#define AT45DB_ENABPROT3   0x7f /* Enable Sector Protection - byte 3 */
#define AT45DB_ENABPROT4   0xa9 /* Enable Sector Protection - byte 4 */
#define AT45DB_DISABPROT1    0x3d /* Disable Sector Protection - byte 1 */
#define AT45DB_DISABPROT2  0x2a /* Disable Sector Protection - byte 2 */
#define AT45DB_DISABPROT3  0x7f /* Disable Sector Protection - byte 3 */
#define AT45DB_DISABPROT4  0x9a /* Disable Sector Protection - byte 4 */
#define AT45DB_ERASEPROT1    0x3d /* Erase Sector Protection Register - byte 1 */
#define AT45DB_ERASEPROT2  0x2a /* Erase Sector Protection Register - byte 2 */
#define AT45DB_ERASEPROT3  0x7f /* Erase Sector Protection Register - byte 3 */
#define AT45DB_ERASEPROT4  0xcf /* Erase Sector Protection Register - byte 4 */
#define AT45DB_PROGPROT1     0x3d /* Program Sector Protection Register - byte 1 */
#define AT45DB_PROGPROT2   0x2a /* Program Sector Protection Register - byte 2 */
#define AT45DB_PROGPROT3   0x7f /* Program Sector Protection Register - byte 3 */
#define AT45DB_PROGPROT4   0xfc /* Program Sector Protection Register - byte 4 */
#define AT45DB_RDPROT        0x32 /* Read Sector Protection Register */
#define AT45DB_LOCKDOWN1     0x3d /* Sector Lockdown - byte 1 */
#define AT45DB_LOCKDOWN2   0x2a /* Sector Lockdown - byte 2 */
#define AT45DB_LOCKDOWN3   0x7f /* Sector Lockdown - byte 3 */
#define AT45DB_LOCKDOWN4   0x30 /* Sector Lockdown - byte 4 */
#define AT45DB_RDLOCKDOWN    0x35 /* Read Sector Lockdown Register  */
#define AT45DB_PROGSEC1      0x9b /* Program Security Register - byte 1 */
#define AT45DB_PROGSEC2    0x00 /* Program Security Register - byte 2 */
#define AT45DB_PROGSEC3    0x00 /* Program Security Register - byte 3 */
#define AT45DB_PROGSEC4    0x00 /* Program Security Register - byte 4 */
#define AT45DB_RDSEC         0x77 /* Read Security Register */

/* Additional commands */
#define AT45DB_MNTOBF1XFR    0x53 /* Main Memory Page to Buffer 1 Transfer */
#define AT45DB_MNTOBF2XFR    0x55 /* Main Memory Page to Buffer 2 Transfer */
#define AT45DB_MNBF1CMP      0x60 /* Main Memory Page to Buffer 1 Compare  */
#define AT45DB_MNBF2CMP      0x61 /* Main Memory Page to Buffer 2 Compare */
#define AT45DB_AUTOWRBF1     0x58 /* Auto Page Rewrite through Buffer 1 */
#define AT45DB_AUTOWRBF2     0x59 /* Auto Page Rewrite through Buffer 2 */
#define AT45DB_PWRDOWN       0xb9 /* Deep Power-down */
#define AT45DB_RESUME        0xab /* Resume from Deep Power-down */
#define AT45DB_RDSR          0xd7 /* Status Register Read */
#define AT45DB_RDDEVID       0x9f /* Manufacturer and Device ID Read */

#define AT45DB_MANUFACTURER  0x1f /* Manufacturer ID: Atmel */
#define AT45DB_DEVID1_CAPMSK 0x1f /* Bits 0-4: Capacity */
#define AT45DB_DEVID1_1MBIT  0x02 /* xxx0 0010 = 1Mbit AT45DB011 */
#define AT45DB_DEVID1_2MBIT  0x03 /* xxx0 0012 = 2Mbit AT45DB021 */
#define AT45DB_DEVID1_4MBIT  0x04 /* xxx0 0100 = 4Mbit AT45DB041 */
#define AT45DB_DEVID1_8MBIT  0x05 /* xxx0 0101 = 8Mbit AT45DB081 */
#define AT45DB_DEVID1_16MBIT 0x06 /* xxx0 0110 = 16Mbit AT45DB161 */
#define AT45DB_DEVID1_32MBIT 0x07 /* xxx0 0111 = 32Mbit AT45DB321 */
#define AT45DB_DEVID1_64MBIT 0x08 /* xxx0 1000 = 32Mbit AT45DB641 */
#define AT45DB_DEVID1_FAMMSK 0xe0 /* Bits 5-7: Family */
#define AT45DB_DEVID1_DFLASH 0x20 /* 001x xxxx = Dataflash */
#define AT45DB_DEVID1_AT26DF 0x40 /* 010x xxxx = AT26DFxxx series (Not supported) */
#define AT45DB_DEVID2_VERMSK 0x1f /* Bits 0-4: MLC mask */
#define AT45DB_DEVID2_MLCMSK 0xe0 /* Bits 5-7: MLC mask */

/* Status register bit definitions */
#define AT45DB_SR_RDY       (1 << 7) /* Bit 7: RDY/ Not BUSY */
#define AT45DB_SR_COMP      (1 << 6) /* Bit 6: COMP */
#define AT45DB_SR_PROTECT   (1 << 1) /* Bit 1: PROTECT */
#define AT45DB_SR_PGSIZE    (1 << 0) /* Bit 0: PAGE_SIZE */

WarpStatus	AT45dbxx;
uint8_t Shift, PageSize, FlashSize_MBit, Pages;
//################################################################################################################
uint8_t AT45DB_readStatusRegister()
{
    // Send Read Status Register command
    uint8_t ops[2] = {0};
	ops[0] = 0xD7;
	ops[1] = 0x00;	
    
	WarpStatus status =  spiTransactionAT45DB(&deviceAT45DBState, ops, 2);
    return status;
}
//################################################################################################################
WarpStatus AT45dbxx_WaitBusy(void)
{
	uint8_t	status;
	uint8_t op[2] = {0};
	op[0] = 0xD7;
	op[1] = 0x00;
	//spiTransactionAT45DB(&deviceAT45DBState, op[0], 1);
	status =  spiTransactionAT45DB(&deviceAT45DBState, op, 2);  
	while((status & 0x80) == 0)
	{
		OSA_TimeDelay(1);
		//spiTransactionAT45DB(&deviceAT45DBState, op, 1);
		status =  spiTransactionAT45DB(&deviceAT45DBState, op, 2); 
	}
}
bool AT45DB641EIsBusy()
{
    uint8_t statusRegCommand[2] = {0xD7, 0}; // read status register command
	//warpPrint("status register %d", statusRegCommand[2]);
    uint8_t statusReg = 0;
    spiTransactionAT45DB(&deviceAT45DBState, statusRegCommand, sizeof(statusRegCommand)); // send the command
    spiTransactionAT45DB(&deviceAT45DBState, &statusReg, sizeof(statusReg)); // read the status register

    return (statusReg & 0x80); // return true if the device is busy, false otherwise
}
//################################################################################################################
WarpStatus AT45dbxx_Resume(void) 
{
	spiTransactionAT45DB(&deviceAT45DBState, AT45DB_RESUME, 1);		
}
//################################################################################################################
WarpStatus AT45dbxx_PowerDown(void) 
{
	spiTransactionAT45DB(&deviceAT45DBState, AT45DB_PWRDOWN, 1);	
}
//################################################################################################################
WarpStatus initAT45DB(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts)
{
	deviceAT45DBState.chipSelectIoPinID		= chipSelectIoPinID;
	deviceAT45DBState.spiSourceBuffer		= gWarpSpiCommonSourceBuffer;
	deviceAT45DBState.spiSinkBuffer		= gWarpSpiCommonSinkBuffer;
	deviceAT45DBState.spiBufferLength		= kWarpMemoryCommonSpiBufferBytes;
	deviceAT45DBState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	GPIO_DRV_SetPinOutput(deviceAT45DBState.chipSelectIoPinID);	
	
	OSA_TimeDelay(10);
	uint8_t Temp0 = 0, Temp1 = 0,Temp2=0;
	spiTransactionAT45DB(&deviceAT45DBState, 0x9f, 1);	
	Temp0=spiTransactionAT45DB(&deviceAT45DBState, 0xa5, 1);	
	Temp1=spiTransactionAT45DB(&deviceAT45DBState, 0xa5, 1);	
	Temp2=AT45DB_readStatusRegister();
	if(Temp0==0x1f)
	{
		switch	(Temp1&0x1f)
		{
			
			case 0x08:	//	AT45db641
				 FlashSize_MBit = 64;
				 Pages = 8192;
				if(Temp2&0x01)
				{
					 Shift = 0;
					 PageSize = 1024;				
				}
				else
				{
					 Shift = 11;
					 PageSize = 1056;				
				}
			break;			
		}			
		
		return true;
	}
	else
		return false;
}
WarpStatus initializeMemoryAT45DB() {
    uint8_t op1[32] = {0};
	op1[0] = 0x3D;
	op1[1] = 0x00;
	op1[2] = 0x00;
	op1[3] = 0x00;

    WarpStatus status1 =  spiTransactionAT45DB(&deviceAT45DBState, op1, 4);
	warpPrint("status1: %d\n", status1);

	uint8_t memoryBuffer[32] = {0};
	memoryBuffer[0] = 0x83;
	memoryBuffer[1] = 0x00;
	memoryBuffer[2] = 0x00;
	memoryBuffer[3] = 0x00;
    

    const uint16_t numDataBytes = 32;
    const uint16_t numMemBytes = numDataBytes + 8;
    for (uint16_t i = 0; i < numDataBytes; i += 2) {
        uint16_t value = i * 3;
        memoryBuffer[i + 8] = (value >> 8) & 0xFF; // MSB
        memoryBuffer[i + 9] = value & 0xFF; // LSB
    }
   
    WarpStatus status2 =  spiTransactionAT45DB(&deviceAT45DBState, memoryBuffer, numMemBytes + 4);
	warpPrint("status2: %d\n", status2);
    uint8_t op2[32] = {0};
	op2[0] = 0x83;
	op2[1] = 0x00;
	op2[2] = 0x00;
	op2[3] = 0x00;
    
	WarpStatus status3 =  spiTransactionAT45DB(&deviceAT45DBState, op2, numMemBytes + 4);
    warpPrint("status3: %d\n", status3);

    // Wait for the write to complete
    while (AT45DB_readStatusRegister() & 0x80) {};

    // Verify the data was written correctly
    uint8_t op3[32] = {0};
	op3[0] = 0x0B;
	op3[1] = 0x00;
	op3[2] = 0x00;
	op3[3] = 0x00;
    
	WarpStatus status4 =  spiTransactionAT45DB(&deviceAT45DBState, op3, 4);
    warpPrint("status4: %d\n", status4);
    bool success = true;
    for (uint16_t i = 0; i < numDataBytes; i += 2) {
        uint16_t expectedValue = i * 3;
        op3[i] = (0x00) << 8 | (0x00);
        if (op3[i] != expectedValue) {
            success = false;
            break;
        }
    }
    WarpStatus status5 =  spiTransactionAT45DB(&deviceAT45DBState, op3, numDataBytes + 4);

    warpPrint("status5: %d\n", status5);
}
//################################################################################################################
WarpStatus spiTransactionAT45DB(WarpSPIDeviceState volatile *  deviceStatePointer, uint8_t ops[], size_t opCount)
{
	spi_status_t	status;

	if (opCount > deviceAT45DBState.spiBufferLength)
	{
		return kWarpStatusBadDeviceCommand;
	}

	warpScaleSupplyVoltage(deviceAT45DBState.operatingVoltageMillivolts);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

	for (int i = 0; (i < opCount) && (i < deviceAT45DBState.spiBufferLength); i++)
	{
		deviceAT45DBState.spiSourceBuffer[i] = ops[i];
		//warpPrint(" a%d,., ", deviceAT45DBState.spiSourceBuffer[i+8]);
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
WarpStatus spiTransactionAT45DB1 (WarpSPIDeviceState volatile * deviceStatePointer, uint8_t * txBuffer, uint8_t * rxBuffer, size_t transactionLength)
{

	
    // uint8_t bufferTx[transactionLength];
    // uint8_t bufferRx[transactionLength];
	spi_status_t	status;
	warpScaleSupplyVoltage(deviceAT45DBState.operatingVoltageMillivolts);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

    for (uint8_t i = 0; i < transactionLength; i++)
    {
        deviceAT45DBState.spiSourceBuffer[i] = txBuffer[i];
        deviceAT45DBState.spiSinkBuffer[i] = 0;
    }
	GPIO_DRV_ClearPinOutput(deviceAT45DBState.chipSelectIoPinID);
	OSA_TimeDelay(1);
	GPIO_DRV_SetPinOutput(deviceAT45DBState.chipSelectIoPinID);
	OSA_TimeDelay(1);
    // Assert CS
    GPIO_DRV_ClearPinOutput(deviceAT45DBState.chipSelectIoPinID);
    
	warpEnableSPIpins();
    // Perform SPI transaction
    status = SPI_DRV_MasterTransferBlocking(0							/*	master instance			*/,
					NULL								/*	spi_master_user_config_t	*/,
					(const uint8_t * restrict)deviceAT45DBState.spiSourceBuffer	/*	source buffer			*/,
					(uint8_t * restrict)deviceAT45DBState.spiSinkBuffer		/*	receive buffer			*/,
					transactionLength								/*	transfer size			*/,
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


    // Deassert CS
    GPIO_DRV_SetPinOutput(deviceAT45DBState.chipSelectIoPinID);
    OSA_TimeDelay(1);



    for (uint8_t i = 0; i < transactionLength; i++)
    {
        deviceAT45DBState.spiSinkBuffer[i] = rxBuffer[i];
    }

    return kWarpStatusOK;
}
//################################################################################################################
WarpStatus chipEraseAT45DB()
{
	WarpStatus	status;
	uint8_t	ops[4] = {0};

	ops[0] = 0xC7;	/* CER (SPI Mode) */
	ops[1] = 0x94;
	ops[2] = 0x80;
	ops[3] = 0x9A;

	status =  spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
	return status;
}
//################################################################################################################
WarpStatus MainMemoryPageEraseAT45DB(uint32_t startAddress)
{
    WarpStatus status;
    startAddress = startAddress << Shift;
	AT45dbxx_Resume();
	AT45dbxx_WaitBusy();
    uint8_t ops[4] = {0};
    ops[0] = 0x81;  // Main Memory Page Erase with Built-in Erase (Page #)
    ops[1] = (uint8_t)((startAddress >> 16) & 0xFF);
    ops[2] = (uint8_t)((startAddress >> 8) & 0xFF);
    ops[3] = (uint8_t)(startAddress & 0xFF);
    
    status = spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
    
    return status;
}
//################################################################################################################
WarpStatus writeDataToBuffer(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
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
		ops[i+4] = (buf)[i];
	}

	return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+4);
}
//################################################################################################################
WarpStatus readBufferAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  bufd)
{
	WarpStatus	status;

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0xD4;	/* NORD */
	
	ops[1] = (uint8_t)((startAddress & 0x0F00) >> 2);
	ops[2] = (uint8_t)((startAddress & 0x00F0) >> 1);
	ops[3] = (uint8_t)((startAddress & 0x000F));
	
	status =  spiTransactionAT45DB(&deviceAT45DBState, ops, 4);

	if (status != kWarpStatusOK)
	{
		return status;
	}
	status = spiTransactionAT45DB(&deviceAT45DBState, bufd, nbyte);
	for (size_t i = 0; i < nbyte; i++)
	{
		//((uint8_t*)bufd)[i] = deviceAT45DBState.spiSinkBuffer[i+4];
		warpPrint("%d,", bufd[i]);
	}

	return kWarpStatusOK;
}
//################################################################################################################
WarpStatus writeDataFromBufferToPage(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
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
//################################################################################################################
WarpStatus readPageAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	WarpStatus	status, status1;

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[] = {0};
	ops[0] = 0x0B;	/* NORD */	
	ops[1] = (uint8_t)((startAddress >> 16) & 0xFF);
	ops[2] = (uint8_t)((startAddress >> 8) & 0xFF);
	ops[3] = (uint8_t)((startAddress & 0xFF));
	ops[4] = 0;
AT45dbxx_Resume();
AT45DB641EIsBusy();
	status =  spiTransactionAT45DB1(&deviceAT45DBState, ops, NULL, 5);

	if (status != kWarpStatusOK)
	{
		return status;
	}

	// for (size_t i = 0; i < nbyte; i++)
	// {
	// 	((uint8_t*)buf)[i] = deviceAT45DBState.spiSinkBuffer[i];
	// }
	status1 = spiTransactionAT45DB1(&deviceAT45DBState, buf, NULL, nbyte);
	AT45dbxx_PowerDown();
	if (status1 != kWarpStatusOK)
	{
		return status1;
	}
	for (size_t i = 0; i < 32; i++)
		{
			//warpPrint("%d,", buf[i]);

			warpPrint("%d, ",  buf[i]);						
		}
}
//################################################################################################################
WarpStatus MainMemoryPageProgramAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf, int r)
{
	WarpStatus	status;
	startAddress = startAddress << Shift;
	if(nbyte > PageSize)
		nbyte = PageSize;
	AT45dbxx_Resume();
	AT45DB641EIsBusy();
	uint8_t	ops[] = {0};
	ops[0] = 0x82;
	ops[1] = (uint8_t)((startAddress >> 16) & 0xFF);
	ops[2] = (uint8_t)((startAddress >> 8) & 0xFF);
	ops[3] = (uint8_t)((startAddress & 0xFF));
	spiTransactionAT45DB1(&deviceAT45DBState, ops, NULL, 4);
	spiTransactionAT45DB1(&deviceAT45DBState, buf, NULL, nbyte);
	// for (size_t i = 0; i < nbyte; i++)
	// {
	// 	ops[i+5] = buf[i];
	// }

	// status =  spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte+5);
	// return status;
AT45DB641EIsBusy();
}
//################################################################################################################
WarpStatus MainMemoryPageReadAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	WarpStatus	status;
	uint16_t pageIndex;

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}
	pageIndex = startAddress & 0x7FF;
	uint8_t	ops[6] = {0};
	ops[0] = 0xD2;	
	ops[1] = (uint8_t)((pageIndex >> 7) & 0xFF);
	ops[2] = (uint8_t)((pageIndex << 1) & 0xFF);
	ops[3] = (uint8_t)((startAddress & 0xFF));
	ops[4] = 0x00;
	ops[5] = 0x00;
	ops[6] = 0x00;
	ops[7] = 0x00;
	
    
	status =  spiTransactionAT45DB(&deviceAT45DBState, ops, 8);

	if (status != kWarpStatusOK)
    {
        return status;
    }

	

	
	// for (size_t i = 0; i < nbyte; i++)
	// {
	// 	buf[i] = deviceAT45DBState.spiSourceBuffer[i+7];		
		
	// 	//warpPrint(" a%d,., ", buf[i]);
	// }	
	// for (size_t i = 0; i < 32; i++)
	// {
	// 	warpPrint("%d, ", buf[i]);
	// 	OSA_TimeDelay(5);
	// }
	status = spiTransactionAT45DB(&deviceAT45DBState, buf, nbyte);
	
	
	
	return status;
}
//################################################################################################################
WarpStatus eraseSectorAT45DB(uint32_t address)
{
	uint8_t	ops[4] = {0};

	ops[0] = 0x7C;	/* SER (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
}
//################################################################################################################
WarpStatus erase32kBlockAT45DB(uint32_t address)
{
	uint8_t	ops[4] = {0};

	ops[0] = 0x12;	/* BER32K (SPI Mode) */
	ops[1] = (uint8_t)((address & 0x0F00) >> 2);
	ops[2] = (uint8_t)((address & 0x00F0) >> 1);
	ops[3] = (uint8_t)((address & 0x000F));

	return spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
}
//################################################################################################################
WarpStatus MemoryPageProgramAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
    WarpStatus status;

    // Step 1: Write buffer 1 to the main memory buffer using "Buffer 1 Write" command
    uint8_t bufferWrite1Command[4] = {0x84, 0, 0, 0}; // Buffer 1 write command
    spiTransactionAT45DB(&deviceAT45DBState, bufferWrite1Command, sizeof(bufferWrite1Command)); // send the command
    spiTransactionAT45DB(&deviceAT45DBState, buf, nbyte); // send the data to write

    // Step 2: Erase the main memory page using "Main Memory Page Erase" command
    uint8_t pageEraseCommand[4] = {0x81, (uint8_t)((startAddress >> 16) & 0xFF), (uint8_t)((startAddress >> 8) & 0xFF), (uint8_t)(startAddress & 0xFF)};
    spiTransactionAT45DB(&deviceAT45DBState, pageEraseCommand, sizeof(pageEraseCommand)); // send the command

    // Step 3: Write buffer 1 to the main memory using "Main Memory Page Program through Buffer 1 with Built-In Erase" command
    uint8_t pageProgramCommand[4] = {0x83, (uint8_t)((startAddress >> 16) & 0xFF), (uint8_t)((startAddress >> 8) & 0xFF), (uint8_t)(startAddress & 0xFF)};
    spiTransactionAT45DB(&deviceAT45DBState, pageProgramCommand, sizeof(pageProgramCommand)); // send the command

    // Step 4: Wait for the programming to complete
    while(AT45DB641EIsBusy());

    return status;
}
//################################################################################################################
WarpStatus BufferWriteAT45DB(uint8_t bufferNumber, uint16_t offset, uint16_t length, uint8_t *data)
{
    uint8_t command[] = {0};
    uint16_t pageIndex;
    WarpStatus status;
    
    /* Calculate the page index and offset within the page */
    pageIndex = (offset >> 8) & 0xFF;
    offset &= 0xFF;
    
    /* Send the buffer write command */
    command[0] = 0x84;
    command[1] = bufferNumber;
    command[2] = offset >> 8;
    command[3] = offset & 0xFF;
   
    // for (int i = 0; i < length;i++)
    // {
    //     command[i+4] = data[i];
    // }
    /* Send the data to be written to the buffer */
    status = spiTransactionAT45DB(&deviceAT45DBState, command, 4);
    if (status != kWarpStatusOK)
    {
        return status;
    }
    // /* Send the buffer to main memory transfer command */
    // command[0] = 0x83;
    // command[1] = bufferNumber;
    // command[2] = pageIndex;
    // command[3] = 0;
    // status = spiTransactionAT45DB(&deviceAT45DBState, command, 4);
    // if (status != kWarpStatusOK)
    // {
    //     return status;
    // }
    status = spiTransactionAT45DB(&deviceAT45DBState, data, length);
    if (status != kWarpStatusOK)
    {
        return status;
    }
    /* Wait for the operation to complete */
    // status = AT45DB641EIsBusy();
    // if (status != kWarpStatusOK)
    // {
    //     return status;
    // }
    return kWarpStatusOK;
}
//################################################################################################################
WarpStatus BufferToMainMemoryAT45DB(uint8_t bufferNumber, uint32_t offset, uint32_t length, uint16_t pageAddress)
{
    uint8_t command[4];
    WarpStatus status;
    
    /* Send the buffer to main memory transfer command */
    command[0] = 0x83;
    command[1] = bufferNumber;
    command[2] = (pageAddress >> 7) & 0xFE;   // Most significant 8 bits of page address
    command[3] = (pageAddress << 1) & 0xFF;   // Least significant 7 bits of page address, plus 1 zero bit
    status = spiTransactionAT45DB(&deviceAT45DBState, command, sizeof(command));
    if (status != kWarpStatusOK)
    {
        return status;
    }
    
    /* Wait for the operation to complete */
    // status = AT45DB641EIsBusy();
    // if (status != kWarpStatusOK)
    // {
    //     return status;
    // }
    
    return kWarpStatusOK;
}
//################################################################################################################
WarpStatus MainMemoryPageReadAT45DB1(uint16_t pageAddress, uint16_t length, uint8_t *data)
{
    uint8_t command[5];
    uint8_t *response;
    uint32_t responseLength;
    uint16_t pageIndex;
    WarpStatus status;
    
    /* Calculate the page index */
    pageIndex = pageAddress & 0x7FF;
    
    /* Send the main memory page read command */
    command[0] = 0xD2;
    command[1] = (pageIndex >> 7) & 0xFF;
    command[2] = (pageIndex << 1) & 0xFF;
    command[3] = 0x00;
    command[4] = 0x00;
	
	status = spiTransactionAT45DB1 (&responseLength, &command, NULL, 5);
	
    if (status != kWarpStatusOK)
    {
        return status;
    }
    
    /* Read the data from main memory */
    status = spiTransactionAT45DB1(&responseLength, NULL, &data, length);
    if (status != kWarpStatusOK)
    {
        return status;
    }
    
    return kWarpStatusOK;
}
//################################################################################################################
WarpStatus MainMemoryPageToBufferTransferAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	WarpStatus	status;
	uint16_t pageIndex;

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}
	pageIndex = startAddress & 0x7FF;
	uint8_t	ops[4] = {0};
	ops[0] = 0x53;	
	ops[1] = (uint8_t)((pageIndex >> 7) & 0xFF);
	ops[2] = (uint8_t)((pageIndex << 1) & 0xFF);
	ops[3] = (uint8_t)((startAddress & 0xFF));
	
	
    
	status =  spiTransactionAT45DB(&deviceAT45DBState, ops, 4);

	if (status != kWarpStatusOK)
    {
        return status;
    }

	
	status = spiTransactionAT45DB(&deviceAT45DBState, buf, nbyte);
	
	return status;
}
//################################################################################################################
WarpStatus mainMemoryPageToBufferAT45DB(uint8_t bufferNumber, uint16_t pageAddress)
{
    // Check for valid buffer number
    if ((bufferNumber != 1) && (bufferNumber != 2))
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    // Construct opcode based on buffer number
    uint8_t opcode = (bufferNumber == 1) ? 0x53 : 0x55;

    // Construct address bytes
    uint8_t addressBytes[3] = {
        (pageAddress >> 8) & 0x7F,  // 15 page address bits (PA14 - PA0)
        (pageAddress & 0xFF),
        0x00                        // 9 dummy bits
    };

    // Send command to AT45DB641E
    uint8_t commandBuffer[4] = {opcode, addressBytes[0], addressBytes[1], addressBytes[2]};
    WarpStatus status = spiTransactionAT45DB(&deviceAT45DBState, commandBuffer, 4);

    return status;
}
//################################################################################################################
WarpStatus bufferReadAT45DB(uint8_t bufferNumber, uint16_t bufferAddress, uint8_t* dataBuffer, uint16_t nbyte)
{
	WarpStatus status;
    // Check for valid buffer number
    if ((bufferNumber != 1) && (bufferNumber != 2))
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    // Construct opcode based on buffer number
    uint8_t opcode = (bufferNumber == 1) ? 0xD4 : 0xD6;

    // Construct address bytes
    uint8_t addressBytes[3] = {
        0x00,                       // 15 dummy bits
        (bufferAddress >> 8) & 0x01,// 9 buffer address bits (BFA8 - BFA0)
        (bufferAddress & 0xFF)
    };
	int8_t commandBuffer[4] = {0};
    // Send command to AT45DB641E
    commandBuffer[0] = opcode;
	commandBuffer[1] = addressBytes[0];
	commandBuffer[2] = addressBytes[1];
	commandBuffer[3] = addressBytes[2];
	
    WarpStatus status1 = spiTransactionAT45DB(&deviceAT45DBState, commandBuffer, 4);

	status = spiTransactionAT45DB(&deviceAT45DBState, dataBuffer, nbyte);

    return status;
}
//################################################################################################################

WarpStatus PageProgramAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf)
{
	
	WarpStatus	status;

	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}
	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0x82;	/* PP */
	ops[1] = (uint16_t)((startAddress & 0x0F00) >> 2);
	ops[2] = (uint16_t)((startAddress & 0x00F0) >> 1);
	ops[3] = (uint16_t)((startAddress & 0x000F));
	
	for (size_t i = 0; i < nbyte; i++)
	{
		ops[i+4] = ((uint8_t*)buf)[i];
		//warpPrint("after write %d\n", buf[i]);
	}
	
    
	return  spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte + 4);	

	
}
// WarpStatus PageProgramAT45DB(uint32_t startAddress, size_t nbyte, int16_t *buf)
// {
//     WarpStatus status;

//     if (nbyte > (kWarpMemoryCommonSpiBufferBytes - 4) / 2)
//     {
//         return kWarpStatusBadDeviceCommand;
//     }

//     uint8_t ops[kWarpMemoryCommonSpiBufferBytes] = {0};
//     ops[0] = 0x82; /* PP */
//     ops[1] = (uint16_t)((startAddress & 0x0F00) >> 2);
//     ops[2] = (uint16_t)((startAddress & 0x00F0) >> 1);
//     ops[3] = (uint16_t)((startAddress & 0x000F));

//     for (size_t i = 0; i < nbyte; i++)
//     {
        
//         ops[i*2 + 4] = (int16_t*)( buf[i] >> 8);
//         ops[i*2 + 5] = (uint8_t*) buf[i];
//         warpPrint("%d",  buf[i]);
//     }

//     return spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte * 2 + 4);
// }

WarpStatus
readmemoryAT45DB(uint32_t startAddress, size_t nbyte, void *  buf)
{
	WarpStatus	status;
	if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
	{
		return kWarpStatusBadDeviceCommand;
	}

	uint8_t	ops[kWarpMemoryCommonSpiBufferBytes] = {0};
	ops[0] = 0xD2;	/* NORD */
	ops[1] = (uint16_t)((startAddress & 0x0F00) >> 2);
	ops[2] = (uint16_t)((startAddress & 0x00F0) >> 1);
	ops[3] = (uint16_t)((startAddress & 0x000F));
	ops[4] = 0x00;
	ops[5] = 0x00;
	ops[6] = 0x00;
	ops[7] = 0x00;

	status = spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte + 8);	
	if (status != kWarpStatusOK)
	{
		return status;
	}

	for (size_t i = 0; i < nbyte; i++)
	{
		((uint8_t*)buf)[i] = deviceAT45DBState.spiSinkBuffer[i+8];		

	}

	return kWarpStatusOK;

}
// WarpStatus readmemoryAT45DB(uint32_t startAddress, size_t nbyte, void* buf)
// {
//     WarpStatus status;
//     if (nbyte > kWarpMemoryCommonSpiBufferBytes - 4)
//     {
//         return kWarpStatusBadDeviceCommand;
//     }

//     uint8_t ops[kWarpMemoryCommonSpiBufferBytes] = {0};
//     ops[0] = 0xD2; /* NORD */
//     ops[1] = (uint16_t)((startAddress & 0x0F00) >> 2);
//     ops[2] = (uint16_t)((startAddress & 0x00F0) >> 1);
//     ops[3] = (uint16_t)((startAddress & 0x000F));
//     ops[4] = 0x00;
//     ops[5] = 0x00;
//     ops[6] = 0x00;
//     ops[7] = 0x00;

//     status = spiTransactionAT45DB(&deviceAT45DBState, ops, nbyte + 8);    
//     if (status != kWarpStatusOK)
//     {
//         return status;
//     }

//     uint8_t* buf_uint8_t = (uint8_t*) buf;

//     for (size_t i = 0; i < nbyte; i++)
//     {
//         buf_uint8_t[i] = deviceAT45DBState.spiSinkBuffer[i + 8];
//     }

//     int16_t* buf_int16_t = (int16_t*) buf;

//     for (size_t i = 0; i < nbyte / 2; i++)
//     {
//         buf_int16_t[i] = (int16_t) (buf_uint8_t[2 * i] << 8) | buf_uint8_t[2 * i + 1];
//     }

//     return kWarpStatusOK;
// }


WarpStatus disablesectorprotection()
{
	WarpStatus	status;
	uint8_t	ops[4] = {0};

	ops[0] = 0x3D;	/* CER (SPI Mode) */
	ops[1] = 0x2A;
	ops[2] = 0x7F;
	ops[3] = 0x9A;

	status =  spiTransactionAT45DB(&deviceAT45DBState, ops, 4);
	return status;

}