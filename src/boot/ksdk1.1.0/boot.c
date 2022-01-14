/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: See git blame.

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
#include <stdarg.h>

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
#include "fsl_lpuart_driver.h"
#include "glaux.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"

// Include global variables and function prototypes from
// devSSD1331.h and adc16_Waterlevel.h
#include "devSSD1331.h"
#include "adc16_Waterlevel.h"

#define							kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define							kWarpConstantStringErrorSanity		"\rSanity check failed!"

#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
	#include "devMMA8451Q.h"
	volatile WarpI2CDeviceState			deviceMMA8451QState;
#endif

#if (WARP_BUILD_ENABLE_DEVINA219)
	#include "devINA219.h"
	volatile WarpI2CDeviceState			deviceINA219State;
#endif

volatile i2c_master_state_t				i2cMasterState;
volatile spi_master_state_t				spiMasterState;
volatile spi_master_user_config_t			spiUserConfig;
volatile lpuart_user_config_t				lpuartUserConfig;
volatile lpuart_state_t					lpuartState;


volatile bool						gWarpBooted				= false;
volatile uint32_t					gWarpI2cBaudRateKbps			= kWarpDefaultI2cBaudRateKbps;
volatile uint32_t					gWarpUartBaudRateBps			= kWarpDefaultUartBaudRateBps;
volatile uint32_t					gWarpSpiBaudRateKbps			= kWarpDefaultSpiBaudRateKbps;
volatile uint32_t					gWarpSleeptimeSeconds			= kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask					gWarpMode				= kWarpModeDisableAdcOnSleep;
volatile uint32_t					gWarpI2cTimeoutMilliseconds		= kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t					gWarpSpiTimeoutMicroseconds		= kWarpDefaultSpiTimeoutMicroseconds;
volatile uint32_t					gWarpUartTimeoutMilliseconds		= kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t					gWarpMenuPrintDelayMilliseconds		= kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t					gWarpSupplySettlingDelayMilliseconds	= kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t					gWarpCurrentSupplyVoltage		= kWarpDefaultSupplyVoltageMillivolts;
char							gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];

/*
 *	Since only one SPI transaction is ongoing at a time in our implementation
 */
uint8_t							gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

static void						sleepUntilReset(void);
static void						lowPowerPinStates(void);
static void						disableTPS62740(void);
static void						enableTPS62740(uint16_t voltageMillivolts);
static void						setTPS62740CommonControlLines(uint16_t voltageMillivolts);
static void						dumpProcessorState(void);
static void						repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
															bool autoIncrement, int chunkReadsPerAddress, bool chatty,
															int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
															uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
static int						char2int(int character);
static int						char2int(int character);
static uint8_t						readHexByte(void);
static int						read4digits(void);
static void						printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, bool loopForever);

/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus						writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus						writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);

void							warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);

/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
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
	gWarpSleeptimeSeconds++;
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
void
LLWU_IRQHandler(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *  notify, power_manager_callback_data_t *  dataPtr)
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
 *	Derived from KSDK power_manager_demo.c <<END
 */

void
sleepUntilReset(void)
{
	while (1)
	{

		warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);

		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
	}
}

void
enableLPUARTpins(void)
{
	/*
	 *	Enable UART CLOCK
	 */
	CLOCK_SYS_EnableLpuartClock(0);

	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

	/*
	 *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
	 */
	lpuartUserConfig.baudRate = gWarpUartBaudRateBps;
	lpuartUserConfig.parityMode = kLpuartParityDisabled;
	lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
	lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;
	lpuartUserConfig.clockSource = kClockLpuartSrcMcgIrClk;

	LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);
}

void
disableLPUARTpins(void)
{
	/*
	 *	LPUART deinit
	 */
	LPUART_DRV_Deinit(0);

	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

//TODO: we don't use flow-control
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Disable LPUART CLOCK
	*/
	CLOCK_SYS_DisableLpuartClock(0);
}


WarpStatus
sendBytesToUART(uint8_t *  bytes, size_t nbytes)
{
	lpuart_status_t	status;

	status = LPUART_DRV_SendDataBlocking(0, bytes, nbytes, gWarpUartTimeoutMilliseconds);
	if (status != 0)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


void
warpEnableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	kWarpPinSPI_MISO_UART_RTS_UART_RTS --> PTA6 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	kWarpPinSPI_MOSI_UART_CTS --> PTA7 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAlt3);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);
	#endif

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}



void
warpDisableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);

	/*	kWarpPinSPI_MISO_UART_RTS	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	kWarpPinSPI_MOSI_UART_CTS	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	#endif

//TODO: we don't use HW flow control so can remove these since we don't use the RTS/CTS
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	CLOCK_SYS_DisableSpiClock(0);
}



void
warpDeasserAllSPIchipSelects(void)
{

	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	#endif
}



void
debugPrintSPIsinkBuffer(void)
{
	for (int i = 0; i < kWarpMemoryCommonSpiBufferBytes; i++)
	{
		warpPrint("\tgWarpSpiCommonSinkBuffer[%d] = [0x%02X]\n", i, gWarpSpiCommonSinkBuffer[i]);
	}
	warpPrint("\n");
}



void
warpEnableI2Cpins(void)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	(ALT2 == I2C)
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	(ALT2 == I2C)
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}



void
warpDisableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	disabled
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	disabled
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	CLOCK_SYS_DisableI2cClock(0);
}


#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
	void
	lowPowerPinStates(void)
	{
		/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state, except for the
		 *	sacrificial pins (WLCSP package, Glaux) where we set them to disabled. We choose
		 *	to set non-disabled pins to '0'.
		 *
		 *	NOTE: Pin state "disabled" means default functionality is active.
		 */

		/*
		 *			PORT A
		 */
		/*
		 *	Leave PTA0/1/2 SWD pins in their default state (i.e., as SWD / Alt3).
		 *
		 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

		/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
		 *
		 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

		/*
		 *	Disable PTA5
		 *
		 *	NOTE: Enabling this significantly increases current draw
		 *	(from ~180uA to ~4mA) and we don't need the RTC on Glaux.
		 *
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

		/*
		 *	PTA6, PTA7, PTA8, and PTA9 on Glaux are SPI and sacrificial SPI.
		 *
		 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
		 *
		 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
		 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

		/*
		 *	NOTE: The KL03 has no PTA10 or PTA11
		 */

		/*
		 *	In Glaux, PTA12 is a sacrificial pin for SWD_RESET, so careful not to drive it.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);



		/*
		 *			PORT B
		 *
		 *	PTB0 is LED on Glaux. PTB1 is unused, and PTB2 is FLASH_!CS
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

		/*
		 *	PTB3 and PTB4 (I2C pins) are true open-drain and we
		 *	purposefully leave them disabled since they have pull-ups.
		 *	PTB5 is sacrificial for I2C_SDA, so disable.
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);

		/*
		 *	NOTE:
		 *
		 *	The KL03 has no PTB8, PTB9, or PTB12.  Additionally, the WLCSP package
		 *	we in Glaux has no PTB6, PTB7, PTB10, or PTB11.
		 */

		/*
		 *	In Glaux, PTB13 is a sacrificial pin for SWD_RESET, so careful not to drive it.
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);

		GPIO_DRV_SetPinOutput(kGlauxPinFlash_SPI_nCS);
		GPIO_DRV_ClearPinOutput(kGlauxPinLED);

		return;
	}
#else
	void
	lowPowerPinStates(void)
	{
		/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state. We choose
		 *	to set them all to '0' since it happens that the devices we want to keep
		 *	deactivated (SI4705) also need '0'.
		 */

		/*
		 *			PORT A
		 */
		/*
		 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

		/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
		 *
		 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

		/*
		 *	Disable PTA5
		 *
		 *	NOTE: Enabling this significantly increases current draw
		 *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
		 *
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

		/*
		 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
		 *
		 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
		 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

		/*
		 *	NOTE: The KL03 has no PTA10 or PTA11
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);


		/*
		 *			PORT B
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
	}
#endif


void
disableTPS62740(void)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_REGCTRL);
	#endif
}

void
enableTPS62740(uint16_t voltageMillivolts)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*
		 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
		 *
		 *	Setup:
		 *		PTB5/kWarpPinTPS62740_REGCTRL for GPIO
		 *		PTB6/kWarpPinTPS62740_VSEL4 for GPIO
		 *		PTB7/kWarpPinTPS62740_VSEL3 for GPIO
		 *		PTB10/kWarpPinTPS62740_VSEL2 for GPIO
		 *		PTB11/kWarpPinTPS62740_VSEL1 for GPIO
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

		setTPS62740CommonControlLines(voltageMillivolts);
		GPIO_DRV_SetPinOutput(kWarpPinTPS62740_REGCTRL);
	#endif
}

void
setTPS62740CommonControlLines(uint16_t voltageMillivolts)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		switch(voltageMillivolts)
		{
			case 1800:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 1900:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2000:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2100:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2200:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2300:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2400:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2500:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2600:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2700:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2800:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2900:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3000:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3100:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3200:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3300:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			/*
			 *	Should never happen, due to previous check in warpScaleSupplyVoltage()
			 */
			default:
			{
				warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
			}
		}

		/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
		 */
		OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
	#endif
}



void
warpScaleSupplyVoltage(uint16_t voltageMillivolts)
{
	if (voltageMillivolts == gWarpCurrentSupplyVoltage)
	{
		return;
	}

	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		if (voltageMillivolts >= 1800 && voltageMillivolts <= 3300)
		{
			enableTPS62740(voltageMillivolts);
			gWarpCurrentSupplyVoltage = voltageMillivolts;
		}
		else
		{
			warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
		}
	#endif
}



void
warpDisableSupplyVoltage(void)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		disableTPS62740();

		/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
		 */
		OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
	#endif
}


void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	WarpStatus	status = kWarpStatusOK;

	/*
	 *	Set all pins into low-power states. We don't just disable all pins,
	 *	as the various devices hanging off will be left in higher power draw
	 *	state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
	if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}

	status = warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
	if (status != kWarpStatusOK)
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPS, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}
}


/*
void
printPinDirections(void)
{
	warpPrint("I2C0_SDA:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SDA_UART_RX));
	OSA_TimeDelay(100);
	warpPrint("I2C0_SCL:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SCL_UART_TX));
	OSA_TimeDelay(100);
	warpPrint("SPI_MOSI:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MOSI_UART_CTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_MISO:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MISO_UART_RTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_SCK_I2C_PULLUP_EN:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_SCK_I2C_PULLUP_EN));
	OSA_TimeDelay(100);
	warpPrint("ADXL362_CS:%d\n", GPIO_DRV_GetPinDir(kWarpPinADXL362_CS));
	OSA_TimeDelay(100);
}
*/



void
dumpProcessorState(void)
{
	uint32_t	cpuClockFrequency;

	CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
	warpPrint("\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
	warpPrint("\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
	warpPrint("\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
	warpPrint("\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
	warpPrint("\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
	warpPrint("\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
	warpPrint("\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
	warpPrint("\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
	warpPrint("\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
	warpPrint("\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
	warpPrint("\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
	warpPrint("\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
	warpPrint("\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
	warpPrint("\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
}


void
printBootSplash(uint16_t gWarpCurrentSupplyVoltage, uint8_t menuRegisterAddress, WarpPowerManagerCallbackStructure *  powerManagerCallbackStructure)
{
	/*
	 *	We break up the prints with small delays to allow us to use small RTT print
	 *	buffers without overrunning them when at max CPU speed.
	 */
	warpPrint("\r\n\n\n\n[ *\t\t\t\tWarp (HW revision C) / Glaux (HW revision B)\t\t\t* ]\n");
	warpPrint("\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
	warpPrint("\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
			gWarpCurrentSupplyVoltage, menuRegisterAddress);
	warpPrint("\r\tI2C=%dkb/s,\tSPI=%dkb/s,\tUART=%db/s,\tI2C Pull-Up=%d\n\n",
			gWarpI2cBaudRateKbps, gWarpSpiBaudRateKbps, gWarpUartBaudRateBps);
	warpPrint("\r\tSIM->SCGC6=0x%02x\t\tRTC->SR=0x%02x\t\tRTC->TSR=0x%02x\n", SIM->SCGC6, RTC->SR, RTC->TSR);
	warpPrint("\r\tMCG_C1=0x%02x\t\t\tMCG_C2=0x%02x\t\tMCG_S=0x%02x\n", MCG_C1, MCG_C2, MCG_S);
	warpPrint("\r\tMCG_SC=0x%02x\t\t\tMCG_MC=0x%02x\t\tOSC_CR=0x%02x\n", MCG_SC, MCG_MC, OSC_CR);
	warpPrint("\r\tSMC_PMPROT=0x%02x\t\t\tSMC_PMCTRL=0x%02x\t\tSCB->SCR=0x%02x\n", SMC_PMPROT, SMC_PMCTRL, SCB->SCR);
	warpPrint("\r\tPMC_REGSC=0x%02x\t\t\tSIM_SCGC4=0x%02x\tRTC->TPR=0x%02x\n\n", PMC_REGSC, SIM_SCGC4, RTC->TPR);
	warpPrint("\r\t%ds in RTC Handler to-date,\t%d Pmgr Errors\n", gWarpSleeptimeSeconds, powerManagerCallbackStructure->errorCount);
}

void
blinkLED(int pin)
{
	GPIO_DRV_SetPinOutput(pin);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(pin);
	OSA_TimeDelay(200);

	return;
}

void
warpPrint(const char *fmt, ...)
{
	int	fmtlen;
	va_list	arg;

	/*
	 *	We use an ifdef rather than a C if to allow us to compile-out
	 *	all references to SEGGER_RTT_*printf if we don't want them.
	 *
	 *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
	 *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
	 *	also takes our print buffer which we will eventually send over
	 *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
	 *	2kB flash and removes the use of malloc so we can keep heap
	 *	allocation to zero.
	 */
	#if (WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF)
		/*
		 *	We can't use SEGGER_RTT_vprintf to format into a buffer
		 *	since SEGGER_RTT_vprintf formats directly into the special
		 *	RTT memory region to be picked up by the RTT / SWD mechanism...
		 */
		va_start(arg, fmt);
		fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
		va_end(arg);

		if (fmtlen < 0)
		{
			SEGGER_RTT_WriteString(0, gWarpEfmt);
			return;
		}

	#else

		SEGGER_RTT_WriteString(0, fmt);

	#endif

	return;
}

int
warpWaitKey(void)
{
	/*
	 *	SEGGER'S implementation assumes the result of result of
	 *	SEGGER_RTT_GetKey() is an int, so we play along.
	 */
	int		rttKey, bleChar = kWarpMiscMarkerForAbsentByte;

	/*
	 *	Set the UART buffer to 0xFF and then wait until either the
	 *	UART RX buffer changes or the RTT icoming key changes.
	 *
	 *	The check below on rttKey is exactly what SEGGER_RTT_WaitKey()
	 *	does in SEGGER_RTT.c.
	 */

	do
	{
		rttKey	= SEGGER_RTT_GetKey();

		/*
		 *	NOTE: We ignore all chars on BLE except '0'-'9', 'a'-'z'/'A'-Z'
		 */
		if (!(bleChar > 'a' && bleChar < 'z') && !(bleChar > 'A' && bleChar < 'Z') && !(bleChar > '0' && bleChar < '9'))
		{
			bleChar = kWarpMiscMarkerForAbsentByte;
		}
	} while ((rttKey < 0) && (bleChar == kWarpMiscMarkerForAbsentByte));

	return rttKey;
}

int
main(void)
{
	WarpStatus				status;
	uint8_t					key;
	WarpSensorDevice			menuTargetSensor		= kWarpSensorMMA8451Q;
	volatile WarpI2CDeviceState *		menuI2cDevice			= NULL;
	uint8_t					menuRegisterAddress		= 0x00;
	rtc_datetime_t				warpBootDate;
	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	/*
	 *	We use this as a template later below and change the .mode fields for the different other modes.
	 */
	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: POWER_SYS_SetMode() depends on this order
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

	WarpPowerManagerCallbackStructure		powerManagerCallbackStructure;

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
	 *	Set board crystal value (Warp revB and earlier).
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	/*
	 *	When booting to CSV stream, we wait to be up and running as soon as possible after
	 *	a reset (e.g., a reset due to waking from VLLS0)
	 */
	if (!WARP_BUILD_BOOT_TO_CSVSTREAM)
	{
		warpPrint("\n\n\n\rBooting Warp, in 3... ");
		OSA_TimeDelay(1000);
		warpPrint("2... ");
		OSA_TimeDelay(1000);
		warpPrint("1...\n\n\n\r");
		OSA_TimeDelay(1000);
	}

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Initialize RTC Driver (not needed on Glaux, but we enable it anyway for now
	 *	as that lets us use the current sleep routines). NOTE: We also don't seem to
	 *	be able to go to VLPR mode unless we enable the RTC.
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
	if (WARP_BUILD_BOOT_TO_VLPR)
	{
		warpPrint("About to switch CPU to VLPR mode... ");
		status = warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
		if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
		{
			warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR() failed...\n");
		}
		warpPrint("done.\n\r");
	}

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	warpPrint("About to GPIO_DRV_Init()... ");
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	warpPrint("done.\n");

	/*
	 *	Make sure the SWD pins, PTA0/1/2 SWD pins in their ALT3 state (i.e., as SWD).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	warpPrint("About to lowPowerPinStates()... ");
	lowPowerPinStates();
	warpPrint("done.\n");

	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST on Warp revB, kGlauxPinLED on Glaux)
	 */
	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		blinkLED(kGlauxPinLED);
		blinkLED(kGlauxPinLED);
		blinkLED(kGlauxPinLED);

		USED(disableTPS62740);
		USED(enableTPS62740);
		USED(setTPS62740CommonControlLines);
	#endif

	/*
	 *	Initialize all the sensors
	 */
	#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
//		initMMA8451Q(	0x1D	/* i2cAddress */,	&deviceMMA8451QState,		kWarpDefaultSupplyVoltageMillivoltsMMA8451Q	);
		initMMA8451Q(	0x1D	/* i2cAddress */,		kWarpDefaultSupplyVoltageMillivoltsMMA8451Q	);
	#endif

  // Sets the i2cAddress of the INA219 sensor
  initINA219(	0x40	/* i2cAddress */,		kWarpDefaultSupplyVoltageMillivoltsINA219	);

	#if (WARP_BUILD_ENABLE_DEVLPS25H)
		initLPS25H(	0x5C	/* i2cAddress */,	&deviceLPS25HState,		kWarpDefaultSupplyVoltageMillivoltsLPS25H	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVHDC1000)
		initHDC1000(	0x43	/* i2cAddress */,	&deviceHDC1000State,		kWarpDefaultSupplyVoltageMillivoltsHDC1000	);
	#endif

	#if (WARP_BUILD_DISABLE_SUPPLIES_BY_DEFAULT)
		/*
		*	Make sure sensor supplies are off.
		*
		*	(There's no point in calling activateAllLowPowerSensorModes())
		*/
		warpPrint("Disabling sensor supply... \n");
		warpDisableSupplyVoltage();
		warpPrint("done.\n");
	#endif

	/*
	 *	At this point, we consider the system "booted" and, e.g., warpPrint()s
	 *	will also be sent to the BLE if that is compiled in.
	 */
	gWarpBooted = true;
	warpPrint("Boot done.\n");

	#if (WARP_BUILD_BOOT_TO_CSVSTREAM)
		printBootSplash(gWarpCurrentSupplyVoltage, menuRegisterAddress, &powerManagerCallbackStructure);

		/*
		 *	Force to printAllSensors
		 */
		gWarpI2cBaudRateKbps = 300;

		if (!WARP_BUILD_BOOT_TO_VLPR)
		{
			status = warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */);
			if (status != kWarpStatusOK)
			{
				warpPrint("warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */)() failed...\n");
			}
		}

		warpScaleSupplyVoltage(3300);
		printAllSensors(true /* printHeadersAndCalibration */, true /* hexModeFlag */, 0 /* menuDelayBetweenEachRun */, true /* loopForever */);
		/*
		 *	Notreached
		 */
	#endif

	//////////////////////////////////////////////////////////////////////////////
	/////////////////Main Functionality gor my project////////////////////////////
	//////////////////////////////////////////////////////////////////////////////
  // Get values from the sensor that is connected to the ADC pin  and depending
	// on their values change the OLED screen
	// Once this is done 3000 times (can be changed here to the value you want)
	// Warp functionality resumes and can get readings from the INA219 sensor for example

	int k = 1;
  for (k = 1; k < 3000; k++){
		OSA_TimeDelay(500);
		// Main functionality Function
		// Gets value from the ADC pin and calls the change OLED colour functions
		// depending on the value
		ADC16_Blocking();
	}

  /////////////////////////////Warp Functionality Continues/////////////////////
	while (1)
	{
		/*
		 *	Do not, e.g., lowPowerPinStates() on each iteration, because we actually
		 *	want to use menu to progressiveley change the machine state with various
		 *	commands.
		 */
		printBootSplash(gWarpCurrentSupplyVoltage, menuRegisterAddress, &powerManagerCallbackStructure);

		warpPrint("\rSelect:\n");
		warpPrint("\r- 'a': set default sensor.\n");
		warpPrint("\r- 'b': set I2C baud rate.\n");
		warpPrint("\r- 'c': set SPI baud rate.\n");
		warpPrint("\r- 'd': set UART baud rate.\n");
		warpPrint("\r- 'e': set default register address.\n");
		warpPrint("\r- 'f': write byte to sensor.\n");
		warpPrint("\r- 'g': set default sensor supply voltage.\n");
		warpPrint("\r- 'h': powerdown command to all sensors.\n");
		warpPrint("\r- 'i': set pull-up enable value.\n");
		warpPrint("\r- 'j': repeat read reg 0x%02x on sensor #%d.\n", menuRegisterAddress, menuTargetSensor);
		warpPrint("\r- 'k': sleep until reset.\n");
		warpPrint("\r- 'l': send repeated byte on I2C.\n");
		warpPrint("\r- 'm': send repeated byte on SPI.\n");
		warpPrint("\r- 'n': enable sensor supply voltage.\n");
		warpPrint("\r- 'o': disable sensor supply voltage.\n");
		warpPrint("\r- 'p': switch to VLPR mode.\n");
		warpPrint("\r- 'r': switch to RUN mode.\n");
		warpPrint("\r- 's': power up all sensors.\n");
		warpPrint("\r- 't': dump processor state.\n");
		warpPrint("\r- 'u': set I2C address.\n");

		warpPrint("\r- 'x': disable SWD and spin for 10 secs.\n");
		warpPrint("\r- 'z': perpetually dump all sensor data.\n");

		warpPrint("\rEnter selection> ");
		key = warpWaitKey();

		switch (key)
		{
			/*
			 *		Select sensor
			 */
			case 'a':
			{
				warpPrint("\r\tSelect:\n");

				#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
					warpPrint("\r\t- '5' MMA8451Q			(0x00--0x31): 1.95V -- 3.6V\n");
				#else
					warpPrint("\r\t- '5' MMA8451Q			(0x00--0x31): 1.95V -- 3.6V (compiled out) \n");
				#endif

				#if (WARP_BUILD_ENABLE_DEVLPS25H)
					warpPrint("\r\t- '6' LPS25H			(0x08--0x24): 1.7V -- 3.6V\n");
				#else
					warpPrint("\r\t- '6' LPS25H			(0x08--0x24): 1.7V -- 3.6V (compiled out) \n");
				#endif

				#if (WARP_BUILD_ENABLE_DEVHDC1000)
					warpPrint("\r\t- '8' HDC1000			(0x00--0x1F): 3.0V -- 5.0V\n");
				#else
					warpPrint("\r\t- '8' HDC1000			(0x00--0x1F): 3.0V -- 5.0V (compiled out) \n");
				#endif

				#if (WARP_BUILD_ENABLE_DEVINA219)
					warpPrint("\r\t- 'p' INA219			(0x00--0x05): 3V -- 5.5V\n");
				#else
					warpPrint("\r\t- 'p' INA219			(0x00--0x05): 3V -- 5.5V (compiled out) \n");
				#endif

				warpPrint("\r\tEnter selection> ");
				key = warpWaitKey();

				switch(key)
				{

					#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
						case '5':
						{
							menuTargetSensor = kWarpSensorMMA8451Q;
							menuI2cDevice = &deviceMMA8451QState;
							break;
						}
					#endif
					// Pick 'p' to chose the INA219 sensor
				  #if (WARP_BUILD_ENABLE_DEVINA219)
						case 'p':
						{
							menuTargetSensor = kWarpSensorINA219;
							menuI2cDevice = &deviceINA219State;
							break;
						}
					#endif

					#if (WARP_BUILD_ENABLE_DEVHDC1000)
						case '8':
						{
							menuTargetSensor = kWarpSensorHDC1000;
							menuI2cDevice = &deviceHDC1000State;
							break;
						}
					#endif

					default:
					{
						warpPrint("\r\tInvalid selection '%c' !\n", key);
					}
				}

				break;
			}

			/*
			 *	Change default I2C baud rate
			 */
			case 'b':
			{
				warpPrint("\r\n\tSet I2C baud rate in kbps (e.g., '0001')> ");
				gWarpI2cBaudRateKbps = read4digits();

				/*
				 *	Round 9999kbps to 10Mbps
				 */
				if (gWarpI2cBaudRateKbps == 9999)
				{
					gWarpI2cBaudRateKbps = 10000;
				}

				warpPrint("\r\n\tI2C baud rate set to %d kb/s", gWarpI2cBaudRateKbps);

				break;
			}

			/*
			 *	Change default SPI baud rate
			 */
			case 'c':
			{
				warpPrint("\r\n\tSet SPI baud rate in kbps (e.g., '0001')> ");
				gWarpSpiBaudRateKbps = read4digits();

				/*
				 *	Round 9999kbps to 10Mbps
				 */
				if (gWarpSpiBaudRateKbps == 9999)
				{
					gWarpSpiBaudRateKbps = 10000;
				}

				warpPrint("\r\n\tSPI baud rate: %d kb/s", gWarpSpiBaudRateKbps);

				break;
			}

			/*
			 *	Change default UART baud rate
			 */
			case 'd':
			{
				warpPrint("\r\n\tSet UART baud rate in kbps (e.g., '0001')> ");
				gWarpUartBaudRateBps = read4digits();
				warpPrint("\r\n\tUART baud rate: %d kb/s", gWarpUartBaudRateBps);

				break;
			}

			/*
			 *	Set register address for subsequent operations
			 */
			case 'e':
			{
				warpPrint("\r\n\tEnter 2-nybble register hex address (e.g., '3e')> ");
				menuRegisterAddress = readHexByte();
				warpPrint("\r\n\tEntered [0x%02x].\n\n", menuRegisterAddress);

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


				USED(status);
				warpPrint("\r\n\tEnter I2C addr. (e.g., '0f') or '99' for SPI > ");
				i2cAddress = readHexByte();
				warpPrint("\r\n\tEntered [0x%02x].\n", i2cAddress);

				warpPrint("\r\n\tEnter hex byte to send (e.g., '0f')> ");
				payloadByte[0] = readHexByte();
				warpPrint("\r\n\tEntered [0x%02x].\n", payloadByte[0]);

				if (i2cAddress == 0x99)
				{
					#if (WARP_BUILD_ENABLE_DEVADXL362)

					#endif
				}
				else
				{
					i2c_device_t slave =
					{
						.address = i2cAddress,
						.baudRate_kbps = gWarpI2cBaudRateKbps
					};

					warpScaleSupplyVoltage(gWarpCurrentSupplyVoltage);
					warpEnableI2Cpins();

					commandByte[0] = menuRegisterAddress;
					i2cStatus = I2C_DRV_MasterSendDataBlocking(
											0 /* I2C instance */,
											&slave,
											commandByte,
											1,
											payloadByte,
											1,
											gWarpI2cTimeoutMilliseconds);
					if (i2cStatus != kStatus_I2C_Success)
					{
						warpPrint("\r\n\tI2C write failed, error %d.\n\n", i2cStatus);
					}
					warpDisableI2Cpins();
				}

				/*
				 *	NOTE: do not disable the supply here, because we typically want to build on the effect of this register write command.
				 */

				break;
			}

			/*
			 *	Configure default TPS62740 voltage
			 */
			case 'g':
			{
				warpPrint("\r\n\tOverride sensor supply voltage in mV (e.g., '1800')> ");
				gWarpCurrentSupplyVoltage = read4digits();
				warpPrint("\r\n\tOverride sensor supply voltage set to %d mV", gWarpCurrentSupplyVoltage);

				break;
			}

			case 'j':
			{
				bool		autoIncrement, chatty;
				int		spinDelay, repetitionsPerAddress, chunkReadsPerAddress;
				int		adaptiveSssupplyMaxMillivolts;
				uint8_t		referenceByte;

				warpPrint("\r\n\tAuto-increment from base address 0x%02x? ['0' | '1']> ", menuRegisterAddress);
				autoIncrement = warpWaitKey() - '0';

				warpPrint("\r\n\tChunk reads per address (e.g., '1')> ");
				chunkReadsPerAddress = warpWaitKey() - '0';

				warpPrint("\r\n\tChatty? ['0' | '1']> ");
				chatty = warpWaitKey() - '0';

				warpPrint("\r\n\tInter-operation spin delay in milliseconds (e.g., '0000')> ");
				spinDelay = read4digits();

				warpPrint("\r\n\tRepetitions per address (e.g., '0000')> ");
				repetitionsPerAddress = read4digits();

				warpPrint("\r\n\tMaximum voltage for adaptive supply (e.g., '0000')> ");
				adaptiveSssupplyMaxMillivolts = read4digits();

				warpPrint("\r\n\tReference byte for comparisons (e.g., '3e')> ");
				referenceByte = readHexByte();

				warpPrint("\r\n\tRepeating dev%d @ 0x%02x, reps=%d, pull=%d, delay=%dms:\n\n",
					menuTargetSensor, menuRegisterAddress, repetitionsPerAddress, spinDelay);

				repeatRegisterReadForDeviceAndAddress(	menuTargetSensor /*warpSensorDevice*/,
														menuRegisterAddress /*baseAddress */,
														autoIncrement /*autoIncrement*/,
														chunkReadsPerAddress,
														chatty,
														spinDelay,
														repetitionsPerAddress,
														gWarpCurrentSupplyVoltage,
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
				warpPrint("\r\n\tSleeping until system reset...\n");
				sleepUntilReset();

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

				warpPrint("\r\n\tNOTE: First power sensors and enable I2C\n\n");
				warpPrint("\r\n\tByte to send (e.g., 'F0')> ");
				outBuffer[0] = readHexByte();

				warpPrint("\r\n\tRepetitions (e.g., '0000')> ");
				repetitions = read4digits();

				if (key == 'l')
				{
					warpPrint("\r\n\tSending %d repetitions of [0x%02x] on I2C, sensor supply voltage=%dmV\n\n",
						repetitions, outBuffer[0], gWarpCurrentSupplyVoltage);
					for (int i = 0; i < repetitions; i++)
					{
						writeByteToI2cDeviceRegister(0xFF, true /* sedCommandByte */, outBuffer[0] /* commandByte */, false /* sendPayloadByte */, 0 /* payloadByte */);
					}
				}
				else
				{
					warpPrint("\r\n\tSending %d repetitions of [0x%02x] on SPI, sensor supply voltage=%dmV\n\n",
						repetitions, outBuffer[0], gWarpCurrentSupplyVoltage);
					for (int i = 0; i < repetitions; i++)
					{
						writeBytesToSpi(outBuffer /* payloadByte */, 1 /* payloadLength */);
					}
				}

				break;
			}


			/*
			 *	enable sensor supply voltage
			 */
			case 'n':
			{
				warpScaleSupplyVoltage(gWarpCurrentSupplyVoltage);
				break;
			}

			/*
			 *	disable SSSUPPLY
			 */
			case 'o':
			{
				warpDisableSupplyVoltage();
				break;
			}

			/*
			 *	Switch to VLPR
			 */
			case 'p':
			{
				status = warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */);
				if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
				{
					warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */)() failed...\n");
				}

				break;
			}

			/*
			 *	Switch to RUN
			 */
			case 'r':
			{
				warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */);
				if (status != kWarpStatusOK)
				{
					warpPrint("warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */)() failed...\n");
				}

				break;
			}

			case 't':
			{
				dumpProcessorState();
				break;
			}

			case 'u':
			{
				if (menuI2cDevice == NULL)
				{
					warpPrint("\r\n\tCannot set I2C address: First set the default I2C device.\n");
				}
				else
				{
					warpPrint("\r\n\tSet I2C address of the selected sensor(e.g., '1C')> ");
					uint8_t address = readHexByte();
					menuI2cDevice->i2cAddress = address;
				}

				break;
			}
#if (WARP_BUILD_ENABLE_DEVRV8803C7)
			case 'v':
			{
				warpPrint("\r\n\tSleeping for 3 seconds, then resetting\n");
				warpSetLowPowerMode(kWarpPowerModeVLLS0, 3 /* sleep seconds */);
				if (status != kWarpStatusOK)
				{
					warpPrint("warpSetLowPowerMode(kWarpPowerModeVLLS0, 3 /* sleep seconds : irrelevant here */)() failed...\n");
				}

				warpPrint("\r\n\tThis should never happen...\n");

				break;
			}
#endif
			/*
			 *	Simply spin for 10 seconds. Since the SWD pins should only be enabled when we are waiting for key at top of loop (or toggling after printf), during this time there should be no interference from the SWD.
			 */
			case 'x':
			{
				warpPrint("\r\n\tSpinning for 10 seconds...\n");
				OSA_TimeDelay(10000);
				warpPrint("\r\tDone.\n\n");

				break;
			}

			/*
			 *	Dump all the sensor data in one go
			 */
			case 'z':
			{
				bool		hexModeFlag;

				warpPrint("\r\n\tHex or converted mode? ('h' or 'c')> ");
				key = warpWaitKey();
				hexModeFlag = (key == 'h' ? 1 : 0);

				warpPrint("\r\n\tSet the time delay between each run in milliseconds (e.g., '1234')> ");
				uint16_t	menuDelayBetweenEachRun = read4digits();
				warpPrint("\r\n\tDelay between read batches set to %d milliseconds.\n\n", menuDelayBetweenEachRun);
				printAllSensors(true /* printHeadersAndCalibration */, hexModeFlag, menuDelayBetweenEachRun, true /* loopForever */);

				/*
				 *	Not reached (printAllSensors() does not return)
				 */
				warpDisableI2Cpins();

				break;
			}

			/*
			 *	Read bytes from Flash and print as hex
			 */
			case 'R':
			{
				warpPrint("\r\n\tStart address (e.g., '0000')> ");
				//xx = read4digits();

				warpPrint("\r\n\tEnd address (e.g., '0000')> ");
				//xx = read4digits();

				break;
			}

			/*
			 *	Write raw bytes read from console to Flash
			 */
			case 'F':
			{
				warpPrint("\r\n\tStart address (e.g., '0000')> ");
				//xx = read4digits();

				warpPrint("\r\n\tNumber of bytes to read from console (e.g., '0000')> ");
				//xx = read4digits();

				warpPrint("\r\n\tEnter [%d] raw bytes > ");

				break;
			}

			/*
			 *	Use data from Flash to program FPGA
			 */
			case 'P':
			{
				warpPrint("\r\n\tStart address (e.g., '0000')> ");
				//xx = read4digits();

				warpPrint("\r\n\tNumber of bytes to use (e.g., '0000')> ");
				//xx = read4digits();

				break;
			}


			/*
			 *	Ignore naked returns.
			 */
			case '\n':
			{
				warpPrint("\r\tPayloads make rockets more than just fireworks.");
				break;
			}

			default:
			{
				warpPrint("\r\tInvalid selection '%c' !\n", key);
			}
		}
	}

	return 0;
}



void
printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, bool loopForever)
{
	/*
	 *	A 32-bit counter gives us > 2 years of before it wraps, even if sampling at 60fps
	 */
	uint32_t	readingCount = 0;
	uint32_t	numberOfConfigErrors = 0;

	#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
	numberOfConfigErrors += configureSensorMMA8451Q(0x00,/* Payload: Disable FIFO */
					0x01/* Normal read 8bit, 800Hz, normal, active mode */
					);

	#endif

	#if (WARP_BUILD_ENABLE_DEVINA219)
  numberOfConfigErrors += configureSensorINA219();

  #endif

	#if (WARP_BUILD_ENABLE_DEVHDC1000)
	numberOfConfigErrors += writeSensorRegisterHDC1000(kWarpSensorConfigurationRegisterHDC1000Configuration,/* Configuration register	*/
					(0b1010000<<8),
					);
	#endif

	if (printHeadersAndCalibration)
	{
		warpPrint("Measurement number, RTC->TSR, RTC->TPR,\t\t");

		#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
			warpPrint(" MMA8451 x, MMA8451 y, MMA8451 z,");
		#endif

		#if (WARP_BUILD_ENABLE_DEVINA219)
			warpPrint("INA219 Current uA");
    #endif

		#if (WARP_BUILD_ENABLE_DEVHDC1000)
			warpPrint(" HDC1000 Temp, HDC1000 Hum,");
		#endif

		warpPrint(" RTC->TSR, RTC->TPR, # Config Errors");
		warpPrint("\n\n");
	}

	do
	{
		warpPrint("%12u, %12d, %6d,\t\t", readingCount, RTC->TSR, RTC->TPR);

		#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
			printSensorDataMMA8451Q(hexModeFlag);
		#endif

		// Prints the INA219 sensor data
		#if (WARP_BUILD_ENABLE_DEVINA219)
			printSensorDataINA219(hexModeFlag);
		#endif

		#if (WARP_BUILD_ENABLE_DEVHDC1000)
			printSensorDataHDC1000(hexModeFlag);
		#endif

		warpPrint(" %12d, %6d, %2u\n", RTC->TSR, RTC->TPR, numberOfConfigErrors);

		if (menuDelayBetweenEachRun > 0)
		{
			OSA_TimeDelay(menuDelayBetweenEachRun);
		}

		readingCount++;
	} while (loopForever);
}


void
loopForSensor(	const char *  tagString,
		WarpStatus  (* readSensorRegisterFunction)(uint8_t deviceRegister, int numberOfBytes),
		volatile WarpI2CDeviceState *  i2cDeviceState,
		volatile WarpSPIDeviceState *  spiDeviceState,
		uint8_t  baseAddress,
		uint8_t  minAddress,
		uint8_t  maxAddress,
		int  repetitionsPerAddress,
		int  chunkReadsPerAddress,
		int  spinDelay,
		bool  autoIncrement,
		uint16_t  sssupplyMillivolts,
		uint8_t  referenceByte,
		uint16_t adaptiveSssupplyMaxMillivolts,
		bool  chatty
		)
{
	WarpStatus		status;
	uint8_t			address = min(minAddress, baseAddress);
	int			readCount = repetitionsPerAddress + 1;
	int			nSuccesses = 0;
	int			nFailures = 0;
	int			nCorrects = 0;
	int			nBadCommands = 0;
	uint16_t		actualSssupplyMillivolts = sssupplyMillivolts;


	if (	(!spiDeviceState && !i2cDeviceState) ||
		(spiDeviceState && i2cDeviceState) )
	{
		warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
	}

	warpScaleSupplyVoltage(actualSssupplyMillivolts);
	warpPrint(tagString);

	/*
	 *	Keep on repeating until we are above the maxAddress, or just once if not autoIncrement-ing
	 *	This is checked for at the tail end of the loop.
	 */
	while (true)
	{
		for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
		{
			status = readSensorRegisterFunction(address+j, 1 /* numberOfBytes */);
			if (status == kWarpStatusOK)
			{
				nSuccesses++;
				if (actualSssupplyMillivolts > sssupplyMillivolts)
				{
					actualSssupplyMillivolts -= 100;
					warpScaleSupplyVoltage(actualSssupplyMillivolts);
				}

				if (spiDeviceState)
				{
					if (referenceByte == spiDeviceState->spiSinkBuffer[2])
					{
						nCorrects++;
					}

					if (chatty)
					{
						warpPrint("\r\t0x%02x --> [0x%02x 0x%02x 0x%02x]\n",
							address+j,
							spiDeviceState->spiSinkBuffer[0],
							spiDeviceState->spiSinkBuffer[1],
							spiDeviceState->spiSinkBuffer[2]);
					}
				}
				else
				{
					if (referenceByte == i2cDeviceState->i2cBuffer[0])
					{
						nCorrects++;
					}

					if (chatty)
					{
						warpPrint("\r\t0x%02x --> 0x%02x\n",
							address+j,
							i2cDeviceState->i2cBuffer[0]);
					}
				}
			}
			else if (status == kWarpStatusDeviceCommunicationFailed)
			{
				warpPrint("\r\t0x%02x --> ----\n",
					address+j);

				nFailures++;
				if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
				{
					actualSssupplyMillivolts += 100;
					warpScaleSupplyVoltage(actualSssupplyMillivolts);
				}
			}
			else if (status == kWarpStatusBadDeviceCommand)
			{
				nBadCommands++;
			}

			if (spinDelay > 0)
			{
				OSA_TimeDelay(spinDelay);
			}
		}

		if (autoIncrement)
		{
			address++;
		}

		if (address > maxAddress || !autoIncrement)
		{
			/*
			 *	We either iterated over all possible addresses, or were asked to do only
			 *	one address anyway (i.e. don't increment), so we're done.
			 */
			break;
		}
	}

	/*
	 *	We intersperse RTT_printfs with forced delays to allow us to use small
	 *	print buffers even in RUN mode.
	 */
	warpPrint("\r\n\t%d/%d success rate.\n", nSuccesses, (nSuccesses + nFailures));
	OSA_TimeDelay(50);
	warpPrint("\r\t%d/%d successes matched ref. value of 0x%02x.\n", nCorrects, nSuccesses, referenceByte);
	OSA_TimeDelay(50);
	warpPrint("\r\t%d bad commands.\n\n", nBadCommands);
	OSA_TimeDelay(50);


	return;
}

void
repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, bool autoIncrement, int chunkReadsPerAddress, bool chatty, int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts, uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte)
{
	switch (warpSensorDevice)
	{

		case kWarpSensorMMA8451Q:
		{
			/*
			 *	MMA8451Q: VDD 1.95--3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
				loopForSensor(	"\r\nMMA8451Q:\n\r",		/*	tagString			*/
						&readSensorRegisterMMA8451Q,	/*	readSensorRegisterFunction	*/
						&deviceMMA8451QState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x31,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tMMA8451Q Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		case kWarpSensorINA219:
		{
			/*
			 *	MMA8451Q: VDD 1.95--3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVINA219)
				loopForSensor(	"\r\nINA219:\n\r",		/*	tagString			*/
						&readSensorRegisterINA219,	/*	readSensorRegisterFunction	*/
						&deviceINA219State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x05,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tINA219 Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		case kWarpSensorHDC1000:
		{
			/*
			 *	HDC1000: VDD 3V--5V
			 */
			#if (WARP_BUILD_ENABLE_DEVHDC1000)
				loopForSensor(	"\r\nHDC1000:\n\r",		/*	tagString			*/
						&readSensorRegisterHDC1000,	/*	readSensorRegisterFunction	*/
						&deviceHDC1000State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x1F,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tHDC1000 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		default:
		{
			warpPrint("\r\tInvalid warpSensorDevice [%d] passed to repeatRegisterReadForDeviceAndAddress.\n", warpSensorDevice);
		}
	}

	if (warpSensorDevice != kWarpSensorADXL362)
	{
		warpDisableI2Cpins();
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

	topNybble = warpWaitKey();
	bottomNybble = warpWaitKey();

	return (char2int(topNybble) << 4) + char2int(bottomNybble);
}



int
read4digits(void)
{
	uint8_t		digit1, digit2, digit3, digit4;

	digit1 = warpWaitKey();
	digit2 = warpWaitKey();
	digit3 = warpWaitKey();
	digit4 = warpWaitKey();

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

	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBuffer,
						(sendPayloadByte ? 1 : 0),
						gWarpI2cTimeoutMilliseconds);

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
	uint8_t		inBuffer[payloadLength];
	spi_status_t	status;

	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0					/* master instance */,
						NULL					/* spi_master_user_config_t */,
						payloadBytes,
						inBuffer,
						payloadLength				/* transfer size */,
						gWarpSpiTimeoutMicroseconds		/* timeout in microseconds (unlike I2C which is ms) */);
	warpDisableSPIpins();

	return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}
