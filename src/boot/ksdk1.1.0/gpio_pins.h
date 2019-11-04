#ifndef __FSL_GPIO_PINS_H__
#define __FSL_GPIO_PINS_H__

#include "fsl_gpio_driver.h"

/*
 *	On Warp, these are the alternative functions we have chosen:
 *
 *	Pin Name			Default		Configuration				Revision Notes
 *	=========================	========	====================================================================================
 *	PTA0/IRQ_0/LLWU_P7		SWD_CLK		ALT1		PTA0/ IRQ_0/LLWU_P7
 *	PTA1/IRQ_1/LPTMR0_ALT1		RESET_b		ALT1		PTA1/IRQ_1/LPTMR0_ALT1
 *	PTA2				SWD_DIO		ALT1		PTA2
 *	PTA3				EXTAL0		DEFAULT		EXTAL0
 *	PTA4				XTAL0		DEFAULT		XTAL0
 *	PTA5/RTC_CLK_IN			DISABLED	ALT1 		PTA5
 *	PTA6				DISABLED	ALT3		SPI0_MISO
 *	PTA7/IRQ_4			DISABLED	ALT3		SPI0_MOSI
 *	PTA8				ADC0_SE2	ALT1		PTA8
 *	PTA9				ADC0_SE2	ALT1		PTA9			(was ALT3 + SPI0_SCK in Warp v2)
 *	PTA12/IRQ_13/LPTMR0_ALT2	ADC0_SE0	ALT1		PTA12/IRQ_13/LPTMR0_ALT2
 *
 *	PTB0/IRQ_5/LLWU_P4		ADC0_SE9	ALT3		SPI0_SCK		(was  ALT1 + PTB0/IRQ_5/LLWU_P4 in Warp v2)
 *	PTB1/IRQ_6			ADC0_SE8	DEFAULT		ADC0_SE8
 *	PTB2/IRQ_7			VREF_OUT	ALT1		PTB2/IRQ_7
 *	PTB3/IRQ_10			DISABLED	ALT2		I2C0_SCL
 *	PTB4/IRQ_11			DISABLED	ALT2		I2C0_SDA
 *	PTB5/IRQ_12			NMI_b		ALT1		PTB5/IRQ_12
  *	PTB6				DISABLED	ALT1		PTB6			(was unused in Warp v2)
 *	PTB7				DISABLED	ALT1		PTB7			(was unused in Warp v2)
 *	PTB10				DISABLED	ALT1		PTB10			(was unused in Warp v2)
 *	PTB11				DISABLED	ALT1		PTB11
 *	PTB13/CLKOUT32K			DISABLED	ALT1		PTB13/CLKOUT32K
 *
 *	Historical notes:
 *		A long time ago, before switching to SEGGER RTT for print, we repurposed PTB3/PTB4 for LPUART0_TX/RX for debug prints,
 *		in which case:
 *
 *	Pin Name			Default		Configuration				
 *	=========================	========	====================================================================================
 *	PTB3/IRQ_10			DISABLED	ALT3		LPUART0_TX
 *	PTB4/IRQ_11			DISABLED	ALT3		LPUART0_RX
 *	
 */

enum _gpio_pins 
{
	kWarpPinUnusedPTA0			= GPIO_MAKE_PIN(HW_GPIOA, 0),		/*	PTA0: Reserved for SWD CLK			(was LED1/TS5A3154_IN in Warp v2)			*/
	kWarpPinUnusedPTA1			= GPIO_MAKE_PIN(HW_GPIOA, 1),		/*	PTA1: Reserved for SWD RESET_B			(was LED2/TS5A3154_nEN in Warp v2)			*/
	kWarpPinUnusedPTA2			= GPIO_MAKE_PIN(HW_GPIOA, 2),		/*	PTA2: Reserved for SWD DIO			(was LED3/SI4705_nRST in Warp v2)			*/

	kWarpPinEXTAL0				= GPIO_MAKE_PIN(HW_GPIOA, 3),		/*	PTA3: EXTAL0												*/
	kWarpPinXTAL0				= GPIO_MAKE_PIN(HW_GPIOA, 4),		/*	PTA4: XTAL0												*/
	
	kWarpPinTPS82740_VSEL1			= GPIO_MAKE_PIN(HW_GPIOA, 5),		/*	Warp TPS82740_VSEL1	--> PTA5									*/
	kWarpPinTPS82740_VSEL2			= GPIO_MAKE_PIN(HW_GPIOA, 8),		/*	Warp TPS82740_VSEL2	--> PTA8									*/
	kWarpPinTPS82740B_CTLEN			= GPIO_MAKE_PIN(HW_GPIOA, 12),		/*	Warp kWarpPinTPS82740B_CTLEN --> PTA12 		(was kWarpPinTPS82675_EN in Warp v2)			*/

	kWarpPinSPI_SCK				= GPIO_MAKE_PIN(HW_GPIOB, 0),		/*	Warp kWarpPinSPI_SCK	--> PTB0		(was kWarpPinTPS82740A_CTLEN in Warp v2)		*/
	kWarpPinKL03_VDD_ADC			= GPIO_MAKE_PIN(HW_GPIOB, 1),		/*	Warp KL03 VDD ADC	--> PTB1									*/
	kWarpPinTPS82740_VSEL3			= GPIO_MAKE_PIN(HW_GPIOB, 5),		/*	Warp TPS82740_VSEL3	--> PTB5									*/
	kWarpPinTS5A3154_IN			= GPIO_MAKE_PIN(HW_GPIOB, 6),		/*	Warp TS5A3154_IN	--> PTB6		(was unused in Warp v2)					*/
	kWarpPinSI4705_nRST			= GPIO_MAKE_PIN(HW_GPIOB, 7),		/*	Warp SI4705_nRST	--> PTB7		(was unused in Warp v2)					*/

	kWarpPinPAN1326_nSHUTD			= GPIO_MAKE_PIN(HW_GPIOB, 10),		/*	Warp PAN1326_nSHUTD	--> PTB10		(was unused in Warp v2)					*/
	kWarpPinISL23415_nCS			= GPIO_MAKE_PIN(HW_GPIOB, 11),		/*	Warp ISL23415_nCS	--> PTB11		(was TPS82675_MODE in Warp v2)				*/
	kWarpPinCLKOUT32K			= GPIO_MAKE_PIN(HW_GPIOB, 13),		/*	Warp KL03_CLKOUT32K	--> PTB13									*/

	kWarpPinADXL362_CS			= GPIO_MAKE_PIN(HW_GPIOB, 2),		/*	Warp ADXL362_CS		--> PTB2		(was kWarpPinADXL362_CS_PAN1326_nSHUTD in Warp v2)	*/
	kWarpPinI2C0_SCL			= GPIO_MAKE_PIN(HW_GPIOB, 3),		/*	Warp KL03_I2C0_SCL	--> PTB3									*/
	kWarpPinI2C0_SDA			= GPIO_MAKE_PIN(HW_GPIOB, 4),		/*	Warp KL03_I2C0_SDA	--> PTB4									*/
	kWarpPinSPI_MISO			= GPIO_MAKE_PIN(HW_GPIOA, 6),		/*	Warp KL03_SPI_MISO	--> PTA6									*/
	kWarpPinSPI_MOSI			= GPIO_MAKE_PIN(HW_GPIOA, 7),		/*	Warp KL03_SPI_MOSI	--> PTA7									*/
	kWarpPinTPS82740A_CTLEN			= GPIO_MAKE_PIN(HW_GPIOA, 9),		/*	Warp kWarpPinTPS82740A_CTLEN --> PTA9		(was kWarpPinSPI_SCK_I2C_PULLUP_EN in Warp v2)		*/	
	kWarpPinLPUART_HCI_TX 			= GPIO_MAKE_PIN(HW_GPIOB, 3), 		/*	Warp KL03_LPUART_TX	--> PTB3									*/
	kWarpPinLPUART_HCI_RX 			= GPIO_MAKE_PIN(HW_GPIOB, 4),		/*	Warp KL03_LPUART_RX	--> PTB4									*/
	kWarpPinPAN1326_HCI_RTS 		= GPIO_MAKE_PIN(HW_GPIOA, 6),		/*	Warp kWarpPinPAN1326_HCI_RTS --> PTA6							*/
	kWarpPinPAN1326_HCI_CTS 		= GPIO_MAKE_PIN(HW_GPIOA, 7)		/*	Warp kWarpPinPAN1326_HCI_CTS	--> PTA7						*/
};

extern gpio_input_pin_user_config_t	inputPins[];
extern gpio_output_pin_user_config_t	outputPins[];

#endif /* __FSL_GPIO_PINS_H__ */
