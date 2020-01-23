#include <stdint.h>
#include <stdlib.h>
#include "gpio_pins.h"
#include "device/fsl_device_registers.h"



/*
 *	See Section 12.1.1 "GPIO instantiation information" of KL03 family reference, KL03P24M48SF0RM.pdf
 *	for the default state of pins, pull capability, etc.:
 *
 *		PTA0 : pulled down at reset
 *		PTA2 : pulled up at reset
 *		PTA1 / RESET_b : pulled up at reset
 *		PTB5 : pulled up at reset
 *
 *	See Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf) for additional hints on pin setup for low power
 *
 *	Here, we configure all pins that we ever use as general-purpose output.
 *
 *	Currently, this excludes kWarpPinKL03_VDD_ADC which we configure in inputPins
 *
 */



gpio_output_pin_user_config_t	outputPins[] = {
	/*
	 *	Set unused pins as outputs
	 */
	{
		.pinName = kWarpPinUnusedPTA0,				/*	Was kWarpPinLED1_TS5A3154_IN in Warp v2			*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinUnusedPTA1,				/*	Was kWarpPinLED2_TS5A3154_nEN in Warp v2		*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinUnusedPTA2,				/*	Was kWarpPinLED3_SI4705_nRST in Warp v2			*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740_VSEL1,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740_VSEL2,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740B_CTLEN,			/*	Was kWarpPinSPI_SCK_I2C_PULLUP_ENin Warp v2		*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinSPI_SCK,				/*	Was kWarpPinTPS82740A_CTLEN in Warp v2			*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740_VSEL3,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTS5A3154_IN,				/*	Was kWarpPinUnusedPTB6 in Warp v2			*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinSI4705_nRST,				/*	Was kWarpPinUnusedPTB7 in Warp v2			*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
	{
		.pinName = kWarpPinPAN1326_nSHUTD,			/*	Was kWarpPinUnusedPTB10 in Warp v2			*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinISL23415_nCS, 			/*	Was kWarpPinTPS82675_MODE in Warp v2			*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinCLKOUT32K,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
#endif
	{
		.pinName = kWarpPinADXL362_CS,				/*	Was kWarpPinADXL362_CS_PAN1326_nSHUTD in Warp v2	*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinI2C0_SCL,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinI2C0_SDA,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinSPI_MISO,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinSPI_MOSI,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740A_CTLEN,			/*	Was kWarpPinTPS82675_EN in Warp v2			*/
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
#ifdef WARP_FRDMKL03
	{
		.pinName = kWarpPinUnusedPTB2,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinPAN1323_nSHUTD,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinUnusedPTB4,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinUnusedPTB6,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinUnusedPTB7,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinUnusedPTB9,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
#endif
#ifdef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
	{
		.pinName = kWarpPinFRDMKL03LED_Red,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinFRDMKL03LED_Green,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
		{
		.pinName = kWarpPinFRDMKL03LED_Blue,
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
 *	PTB1 is tied to VBATT. Need to configure it as an input pin.
 *
 */
gpio_input_pin_user_config_t	inputPins[] = {
	{
		.pinName = kWarpPinKL03_VDD_ADC,
		.config.isPullEnable = true,
		.config.pullSelect = kPortPullUp,
		.config.isPassiveFilterEnabled = false,
		.config.interrupt = kPortIntDisabled,
	},
	{
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}
};
