#ifndef __FSL_GPIO_PINS_H__
#define __FSL_GPIO_PINS_H__
#include "fsl_gpio_driver.h"

/*
 *	On Warp, these are the alternative functions we have chosen:
 *
 *	Pin Name			Default		Configuration
 *	=========================	========	=============
 *	PTA0/IRQ_0/LLWU_P7		SWD_CLK		ALT1
 *	PTA1/IRQ_1/LPTMR0_ALT1		RESET_b		ALT1
 *	PTA2				SWD_DIO		ALT1
 *	PTA3				EXTAL0		DEFAULT
 *	PTA4				XTAL0		DEFAULT
 *	PTA5/RTC_CLK_IN			DISABLED	ALT1
 *	PTA6				DISABLED	ALT3
 *	PTA7/IRQ_4			DISABLED	ALT3
 *	PTA8				ADC0_SE2	ALT1
 *	PTA9				ADC0_SE2	ALT1
 *	PTA12/IRQ_13/LPTMR0_ALT2	ADC0_SE0	ALT1
 *
 *	PTB0/IRQ_5/LLWU_P4		ADC0_SE9	ALT3
 *	PTB1/IRQ_6			ADC0_SE8	DEFAULT
 *	PTB2/IRQ_7			VREF_OUT	ALT1
 *	PTB3/IRQ_10			DISABLED	ALT2
 *	PTB4/IRQ_11			DISABLED	ALT2
 *	PTB5/IRQ_12			NMI_b		ALT1
  *	PTB6				DISABLED	ALT1
 *	PTB7				DISABLED	ALT1
 *	PTB10				DISABLED	ALT1
 *	PTB11				DISABLED	ALT1
 *	PTB13/CLKOUT32K			DISABLED	ALT1 
 */

enum _gpio_pins
{
	kWarpPinUnusedPTA0			= GPIO_MAKE_PIN(HW_GPIOA, 0),	/*	PTA0: Reserved for SWD CLK	*/
	kWarpPinUnusedPTA1			= GPIO_MAKE_PIN(HW_GPIOA, 1),	/*	PTA1: Reserved for SWD RESET_B	*/
	kWarpPinUnusedPTA2			= GPIO_MAKE_PIN(HW_GPIOA, 2),	/*	PTA2: Reserved for SWD DIO	*/
	kWarpPinEXTAL0				= GPIO_MAKE_PIN(HW_GPIOA, 3),	/*	PTA3: Reserved for EXTAL0	*/
	kWarpPinXTAL0				= GPIO_MAKE_PIN(HW_GPIOA, 4),	/*	PTA4: Reserved for XTAL0	*/

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		kWarpPinRTC_CLKIN		= GPIO_MAKE_PIN(HW_GPIOA, 5),
	#else
		kWarpPinBGX_nRST		= GPIO_MAKE_PIN(HW_GPIOA, 5),
	#endif

	kWarpPinSPI_MISO_UART_RTS		= GPIO_MAKE_PIN(HW_GPIOA, 6),
	kWarpPinSPI_MOSI_UART_CTS		= GPIO_MAKE_PIN(HW_GPIOA, 7),
	kWarpPinADXL362_SPI_nCS			= GPIO_MAKE_PIN(HW_GPIOA, 8),

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		kWarpPinSPI_SCK			= GPIO_MAKE_PIN(HW_GPIOA, 9),
	#else
		kWarpPinAT45DB_SPI_nCS		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	#endif

	kWarpPinISL23415_SPI_nCS		= GPIO_MAKE_PIN(HW_GPIOA, 12),

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		kGlauxPinLED			= GPIO_MAKE_PIN(HW_GPIOB, 0),
		kGlauxPinUnusedPTB1		= GPIO_MAKE_PIN(HW_GPIOB, 1),
	#else
		kWarpPinSPI_SCK			= GPIO_MAKE_PIN(HW_GPIOB, 0),
		kWarpPinFPGA_nCS		= GPIO_MAKE_PIN(HW_GPIOB, 1),
	#endif

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		kGlauxPinFlash_SPI_nCS		= GPIO_MAKE_PIN(HW_GPIOB, 2),
	#else
		kWarpPinSI4705_nRST		= GPIO_MAKE_PIN(HW_GPIOB, 2),
	#endif

	kWarpPinI2C0_SCL_UART_TX		= GPIO_MAKE_PIN(HW_GPIOB, 3),
	kWarpPinI2C0_SDA_UART_RX		= GPIO_MAKE_PIN(HW_GPIOB, 4),
	kWarpPinTPS62740_REGCTRL		= GPIO_MAKE_PIN(HW_GPIOB, 5),
	kWarpPinTPS62740_VSEL4			= GPIO_MAKE_PIN(HW_GPIOB, 6),
	kWarpPinTPS62740_VSEL3			= GPIO_MAKE_PIN(HW_GPIOB, 7),
	kWarpPinTPS62740_VSEL2			= GPIO_MAKE_PIN(HW_GPIOB, 10),
	kWarpPinTPS62740_VSEL1			= GPIO_MAKE_PIN(HW_GPIOB, 11),
	kWarpPinCLKOUT32K			= GPIO_MAKE_PIN(HW_GPIOB, 13),
};

extern gpio_input_pin_user_config_t	inputPins[];
extern gpio_output_pin_user_config_t	outputPins[];
extern gpio_input_pin_user_config_t	wakeupPins[];
#endif /* __FSL_GPIO_PINS_H__ */
