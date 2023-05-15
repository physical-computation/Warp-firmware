/*
	Authored 2019. Vasileios Tsoutsouras.

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
#include "devISL23415.h"

extern volatile WarpSPIDeviceState	deviceISL23415State;
extern volatile uint32_t			gWarpSpiTimeoutMicroseconds;
extern uint8_t						gWarpSpiCommonSourceBuffer[];
extern uint8_t						gWarpSpiCommonSinkBuffer[];


void		
initISL23415(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts) 
{
	deviceISL23415State.chipSelectIoPinID			= chipSelectIoPinID;
	deviceISL23415State.spiSourceBuffer				= gWarpSpiCommonSourceBuffer;
	deviceISL23415State.spiSinkBuffer				= gWarpSpiCommonSinkBuffer;
	deviceISL23415State.spiBufferLength				= kWarpMemoryCommonSpiBufferBytes;
	deviceISL23415State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus	
readDeviceRegisterISL23415(uint8_t deviceRegister) 
{
	spi_status_t	status;


	warpScaleSupplyVoltage(deviceISL23415State.operatingVoltageMillivolts);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

	/*
	 *	Send the register/instruction, followed by a dummy byte (DCP will shift out value during the
	 *	send of this dummy byte), followed by NOP byte, followed by another dummy byte.
	 *
	 *	Read operations on the ISL23415, unlike write operations, require a NOP instruction to be
	 *	sent (after the two-byte register+dummy sequence used to indicate what register to read).
	 *	See Figure 28 of the ISL23415 manual.
	 */
	deviceISL23415State.spiSourceBuffer[0] = deviceRegister						/*	See Table 4 of manual	*/;
	deviceISL23415State.spiSourceBuffer[1] = 0x00							/*	Dummy byte		*/;
	deviceISL23415State.spiSourceBuffer[2] = kWarpSensorConfigurationRegisterISL23415nopInstruction	/*	NOP byte		*/;
	deviceISL23415State.spiSourceBuffer[3] = 0x00							/*	Dummy byte		*/;

	deviceISL23415State.spiSinkBuffer[0] = 0xFF;
	deviceISL23415State.spiSinkBuffer[1] = 0xFF;
	deviceISL23415State.spiSinkBuffer[2] = 0xFF;
	deviceISL23415State.spiSinkBuffer[3] = 0xFF;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_SPI_nCS);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kWarpPinISL23415_SPI_nCS);
	
	/*
	 *	The result of the SPI transaction will be stored in deviceISL23415State.spiSinkBuffer.
	 *
	 *	Providing a spi_master_user_config_t is optional since it is already provided when we did
	 *	SPI_DRV_MasterConfigureBus(), so we pass in NULL. The "master instance" is always 0 for
	 *	the KL03 since there is only one SPI peripheral.
	 */
	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(
					0								/*	master instance			*/,
					NULL								/*	spi_master_user_config_t	*/,
					(const uint8_t * restrict)deviceISL23415State.spiSourceBuffer,
					(uint8_t * restrict)deviceISL23415State.spiSinkBuffer,
					4								/*	reg ID + dummy + NOP + dummy	*/,
					gWarpSpiTimeoutMicroseconds);
	warpDisableSPIpins();

	/*
	 *	Drive /CS high
	 */
	OSA_TimeDelay(50);
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_SPI_nCS);

	if (status != kStatus_SPI_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus	
writeDeviceRegisterISL23415(uint8_t deviceRegister, uint8_t writeValue)
{
	spi_status_t	status;


	warpScaleSupplyVoltage(deviceISL23415State.operatingVoltageMillivolts);

	/*
	 *	First, configure chip select pins of the various SPI slave devices
	 *	as GPIO and drive all of them high.
	 */
	warpDeasserAllSPIchipSelects();

	/*
	 *	Configure the four ISL23415 DCPs over SPI.
	 *
	 *	It takes two bytes (16 bit clocks) to complete a write transaction. The ISL23415 at 1.8V can
	 *	operate at bit clocks of up to 5MHz and so can change value in principle at rates of up to 312.5k
	 *	times a second. This rate is in principle fast enough for changing  the pull-up resistor values
	 *	at I2C line speeds.
	 */

	/*
	 *	Send the register/instruction, followed by the payload byte.
	 *
	 *	Write operations on the ISL23415, unlike read operations, are two-byte sequences.
	 *	See Figure 27 of the ISL23415 manual.
	 */
	deviceISL23415State.spiSourceBuffer[0] = deviceRegister						/*	See Table 4 of manual	*/;
	deviceISL23415State.spiSourceBuffer[1] = writeValue						/*	Register value to set	*/;

	deviceISL23415State.spiSinkBuffer[0] = 0x00;
	deviceISL23415State.spiSinkBuffer[1] = 0x00;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_SPI_nCS);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kWarpPinISL23415_SPI_nCS);

	/*
	 *	The result of the SPI transaction will be stored in deviceISL23415State.spiSinkBuffer.
	 *
	 *	Providing a spi_master_user_config_t is optional since it is already provided when we did
	 *	SPI_DRV_MasterConfigureBus(), so we pass in NULL. The "master instance" is always 0 for
	 *	the KL03 since there is only one SPI peripheral.
	 */
	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(
					0								/*	master instance			*/,
					NULL								/*	spi_master_user_config_t	*/,
					(const uint8_t * restrict)deviceISL23415State.spiSourceBuffer,
					(uint8_t * restrict)deviceISL23415State.spiSinkBuffer,
					2								/*	reg ID + payload		*/,
					gWarpSpiTimeoutMicroseconds);
	warpDisableSPIpins();

	/*
	 *	Drive /CS high
	 */
	OSA_TimeDelay(50);
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_SPI_nCS);

	if (status != kStatus_SPI_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}
