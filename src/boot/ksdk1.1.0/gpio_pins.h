#ifndef __FSL_GPIO_PINS_H__
#define __FSL_GPIO_PINS_H__

#include "fsl_gpio_driver.h"



/*
 *	On Warp, these are the alternative functions we have chosen:
 *
 *	Pin Name			Default		Configuration				
 *	=========================	========	=======================================
 *	PTA3				EXTAL0		DEFAULT		EXTAL0
 *	PTA4				XTAL0		DEFAULT		XTAL0
 *	PTA5/RTC_CLK_IN			DISABLED	ALT1 		PTA5
 *	PTA6				DISABLED	ALT3		SPI0_MISO
 *	PTB11				DISABLED	ALT1		PTB11
 *	PTA7/IRQ_4			DISABLED	ALT3		SPI0_MOSI
 *	PTB0/IRQ_5/LLWU_P4		ADC0_SE9	ALT1		PTB0/IRQ_5/LLWU_P4
 *	PTB1/IRQ_6			ADC0_SE8	DEFAULT		ADC0_SE8
 *	PTB2/IRQ_7			VREF_OUT	ALT1		PTB2/IRQ_7
 *	PTA8				ADC0_SE2	ALT1		PTA8
 *	PTA9				ADC0_SE2	ALT3		SPI0_SCK
 *	PTB3/IRQ_10			DISABLED	ALT2		I2C0_SCL
 *	PTB4/IRQ_11			DISABLED	ALT2		I2C0_SDA
 *	PTB5/IRQ_12			NMI_b		ALT1		PTB5/IRQ_12
 *	PTA12/IRQ_13/LPTMR0_ALT2	ADC0_SE0	ALT1		PTA12/IRQ_13/LPTMR0_ALT2
 *	PTB13/CLKOUT32K			DISABLED	ALT1		PTB13/CLKOUT32K
 *	PTA0/IRQ_0/LLWU_P7		SWD_CLK		ALT1		PTA0/ IRQ_0/LLWU_P7
 *	PTA1/IRQ_1/LPTMR0_ALT1		RESET_b		ALT1		PTA1/IRQ_1/LPTMR0_ALT1
 *	PTA2				SWD_DIO		ALT1		PTA2
 *
 */

enum _gpio_pins 
{
	kWarpPinKL03_VDD_ADC			= GPIO_MAKE_PIN(HW_GPIOB, 1),		/*	Warp KL03 VDD ADC	--> PTB1	*/
	kWarpPinTPS82675_MODE			= GPIO_MAKE_PIN(HW_GPIOB, 11),		/*	Warp TPS82675_MODE	--> PTB11	*/
	kWarpPinI2C0_SDA			= GPIO_MAKE_PIN(HW_GPIOB, 4),		/*	Warp KL03_I2C0_SDA	--> PTB4	*/
	kWarpPinI2C0_SCL			= GPIO_MAKE_PIN(HW_GPIOB, 3),		/*	Warp KL03_I2C0_SCL	--> PTB3	*/
	kWarpPinSPI_MOSI			= GPIO_MAKE_PIN(HW_GPIOA, 7),		/*	Warp KL03_SPI_MOSI	--> PTA7	*/
	kWarpPinSPI_MISO			= GPIO_MAKE_PIN(HW_GPIOA, 6),		/*	Warp KL03_SPI_MISO	--> PTA6	*/
	kWarpPinSPI_SCK_I2C_PULLUP_EN		= GPIO_MAKE_PIN(HW_GPIOA, 9),		/*	Warp KL03_SPI_SCK	--> PTA9	*/	
	kWarpPinTPS82740A_VSEL2			= GPIO_MAKE_PIN(HW_GPIOA, 8),		/*	Warp TPS82740A_VSEL2	--> PTA8	*/

	kWarpPinADXL362_CS_PAN1326_nSHUTD	= GPIO_MAKE_PIN(HW_GPIOB, 2),		/*	Warp ADXL362_CS		--> PTB2	*/
	kWarpPinTPS82740A_CTLEN			= GPIO_MAKE_PIN(HW_GPIOB, 0),		/*	Warp TPS82740A_(CTL/EN)	--> PTB0	*/
	kWarpPinTPS82675_EN			= GPIO_MAKE_PIN(HW_GPIOA, 12),		/*	Warp TPS82675_EN	--> PTA12	*/
	kWarpPinTPS82740A_VSEL1			= GPIO_MAKE_PIN(HW_GPIOA, 5),		/*	Warp TPS82740A_VSEL1	--> PTA5	*/
	kWarpPinTPS82740A_VSEL3			= GPIO_MAKE_PIN(HW_GPIOB, 5),		/*	Warp TPS82740A_VSEL3	--> PTB5	*/

	kWarpPinCLKOUT32K			= GPIO_MAKE_PIN(HW_GPIOB, 13),		/*	Warp KL03_CLKOUT32K	--> PTB13	*/
	kWarpPinLED1_TS5A3154_IN		= GPIO_MAKE_PIN(HW_GPIOA, 0),		/*	Warp LED1/TS5A3154_IN	--> PTA0	*/
	kWarpPinLED2_TS5A3154_nEN		= GPIO_MAKE_PIN(HW_GPIOA, 1),		/*	Warp LED2/TS5A3154_nEN	--> PTA1	*/
	kWarpPinLED3_SI4705_nRST		= GPIO_MAKE_PIN(HW_GPIOA, 2),		/*	Warp LED3/SI4705_nRST	--> PTA2	*/
	
	kWarpPinUnusedPTB6			= GPIO_MAKE_PIN(HW_GPIOB, 6),		/*	PTB6: unused				*/
	kWarpPinUnusedPTB7			= GPIO_MAKE_PIN(HW_GPIOB, 7),		/*	PTB7: unused				*/
	kWarpPinUnusedPTB10			= GPIO_MAKE_PIN(HW_GPIOB, 10),		/*	PTB10: unused				*/

	kWarpPinEXTAL0				= GPIO_MAKE_PIN(HW_GPIOA, 3),		/*	PTA3: EXTAL0				*/
	kWarpPinXTAL0				= GPIO_MAKE_PIN(HW_GPIOA, 4),		/*	PTA4: XTAL0				*/
};

extern gpio_input_pin_user_config_t	inputPins[];
extern gpio_output_pin_user_config_t	outputPins[];

#endif /* __FSL_GPIO_PINS_H__ */
