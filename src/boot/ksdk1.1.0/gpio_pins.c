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
	{
		.pinName = kWarpPinI2C0_SDA,
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
		.pinName = kWarpPinSPI_MOSI,
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
		.pinName = kWarpPinSPI_SCK_I2C_PULLUP_EN,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82675_MODE,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740A_VSEL2,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinADXL362_CS_PAN1326_nSHUTD,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740A_CTLEN,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82675_EN,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740A_VSEL1,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS82740A_VSEL3,
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
	{
		.pinName = kWarpPinLED1_TS5A3154_IN,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinLED2_TS5A3154_nEN,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinLED3_SI4705_nRST,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	
	
	/*
	 *	Set unused pins as outputs
	 */
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
		.pinName = kWarpPinUnusedPTB10,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
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
