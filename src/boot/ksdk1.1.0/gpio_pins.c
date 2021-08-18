#include <stdint.h>
#include <stdlib.h>
#include "config.h"
#include "gpio_pins.h"
#include "device/fsl_device_registers.h"

/*
 *	Here, we configure all pins that we ever use as general-purpose output.
 *
 *	(A) See Section 12.1.1 "GPIO instantiation information" of KL03 family reference, KL03P24M48SF0RM.pdf
 *	for the default state of pins, pull capability, etc.:
 *
 *		PTA0 : pulled down at reset
 *		PTA2 : pulled up at reset
 *		PTA1 / RESET_b : pulled up at reset
 *		PTB5 : pulled up at reset
 *
 *	(B) See Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf) for additional hints on pin setup for low power.
 *
 *	(C) On Glaux, PTA3, PTA4, PTA5, PTA8, PTA12, PTB5, and PTB13 are
 *	either sacrifical or input so we don't configure them as GPIO.
 *
 *	**NOTE 1**:	The semantics is that pins that are excluded are disabled (TODO: double check).
 *
 *	**NOTE 2**:	Empirically, adding entries for pins which we want to leave disabled on Glaux
 *			(e.g., sacrificial pins) leads to higher power dissipation.
 *
 */

gpio_output_pin_user_config_t	outputPins[] = {
	/*
	 *	Set unused pins as outputs
	 */
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		{
			.pinName = kWarpPinBGX_nRST,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		/*	We only use these as SPI, despite being connected to RTS/CTS on revC
		{
			.pinName = kWarpPinSPI_MISO_UART_RTS,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinSPI_MOSI_UART_CTS,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		*/
		{
			.pinName = kWarpPinADXL362_SPI_nCS,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinAT45DB_SPI_nCS,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinISL23415_SPI_nCS,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		/*	Not GPIO, so don't configure it as GPIO
		{
			.pinName = kWarpPinSPI_SCK,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		*/
		{
			.pinName = kWarpPinFPGA_nCS,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinSI4705_nRST,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		/*	Not GPIO, so don't configure it as GPIO
		{
			.pinName = kWarpPinI2C0_SCL_UART_TX,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinI2C0_SDA_UART_RX,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		*/
		{
			.pinName = kWarpPinTPS62740_REGCTRL,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinTPS62740_VSEL4,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinTPS62740_VSEL3,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinTPS62740_VSEL2,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kWarpPinTPS62740_VSEL1,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		/*	Not GPIO, so don't configure it as GPIO
		{
			.pinName = kWarpPinCLKOUT32K,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		*/
	#else
		{
			.pinName = kGlauxPinLED,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
		{
			.pinName = kGlauxPinFlash_SPI_nCS,
			.config.outputLogic = 1,
			.config.slewRate = kPortSlowSlewRate,
			.config.driveStrength = kPortLowDriveStrength,
		},
	#endif

	{
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}
};

/*
 *	Configuration to be passed to GPIO_DRV_Init() to disable all pins.
 *
 *	NOTE: the type here is
 *
 *			gpio_input_pin_user_config_t
 *	not
 *
 *			gpio_output_pin_user_config_t
 *
 *	like the above.
 *
 *	On Warp (but not Glaux), PTB1 is tied to VBATT. Need to configure it as an input pin.
 *	On Glaux, PTA0 (SWD_CLK) is also used as the RTC interrupt line since it is also the
 *	LLWU_P7 source for the low-leakage wakeup unit.
 *
 *	NOTE: The semantics is that pins that are excluded are disabled (TODO: double check).
 */
gpio_input_pin_user_config_t	inputPins[] = {
	{
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}
};

/*
 *	This array is used to configure only the pins needed for LLWU wakeup. See AN4503 Section 3.1.6.
 *
 *	**NOTE**:	The semantics is that pins that are excluded are disabled (TODO: double check).
 */
gpio_input_pin_user_config_t	wakeupPins[] = {
	{
		.pinName = kWarpPinUnusedPTA0,
		.config.isPullEnable = true,
		.config.pullSelect = kPortPullUp,
		.config.isPassiveFilterEnabled = false,
		.config.interrupt = kPortIntDisabled,
	},
	{
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}
};
