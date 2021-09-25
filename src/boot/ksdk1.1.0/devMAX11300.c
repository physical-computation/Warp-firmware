/*
	Authored 2019-2020, Giorgio Mallia.

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


#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"



volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
	Override Warp firmware's use of these pins and define new aliases.
*/

enum
{
	kMAX11300PinMISO	= GPIO_MAKE_PIN(HW_GPIOA,	6),
	kMAX11300PinMOSI	= GPIO_MAKE_PIN(HW_GPIOA,	8),
	kMAX11300PinSCK		= GPIO_MAKE_PIN(HW_GPIOA,	9),
	kMAX11300PinCSn		= GPIO_MAKE_PIN(HW_GPIOB,	13),
};


spi_status_t	writeCommand(uint8_t Register_Byte, uint16_t Command_Bytes, bool W_R)	/* W_R = 0/1 (Write/Read) */ 
{
	spi_status_t status;
	
	/*
		Drive /CS low.
		Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	*/

	GPIO_DRV_SetPinOutput(kMAX11300PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kMAX11300PinCSn);

	uint8_t Byte1 = (Register_Byte << 1) | W_R;
	uint8_t Byte2 = Command_Bytes >> 8;
	uint8_t Byte3 = Command_Bytes & 0xff;

	payloadBytes[0] = Byte1;
	payloadBytes[1] = Byte2;
	payloadBytes[2] = Byte3;

	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
											NULL	/* spi_master_user_config_t*/,
											(const uint8_t * restrict)&payloadBytes[0],
											(uint8_t * restrict) &inBuffer[0],
											3	/* transfer size */,
											2000	/* timeout in microseconds */);

	/*
		Drive /CS high
	*/

	GPIO_DRV_SetPinOutput(kMAX11300PinCSn);

	return status;
}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

void readID(void){
	SEGGER_RTT_printf(0, "\r\t Address (R/W): 0x%x Commands: 0x%x 0x%x \n", payloadBytes[0], payloadBytes[1], payloadBytes[2]);
	OSA_TimeDelay(10);
	writeCommand(0x00, 0x0000, 1); /* READ DEVICE ID: 0x00 (1) [Read] */
	SEGGER_RTT_printf(0, "\r\t Bytes Received: 0x%x 0x%x 0x%x \n", inBuffer[0], inBuffer[1], inBuffer[2]);
	OSA_TimeDelay(10);
}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

int read1digit(int lowerLimit, int higherLimit){
	int number = -1;
	while (!(number >= lowerLimit && number <= higherLimit)){
		number = SEGGER_RTT_WaitKey() - '0';
	}
	return number;
}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

int read2digits(int lowerLimit, int higherLimit){
	int digit1 = 0;
	int digit2 = 0;
	int number = -1;
	while (!(number >= lowerLimit && number <= higherLimit)){
		digit1 = SEGGER_RTT_WaitKey() - '0';
		digit2 = SEGGER_RTT_WaitKey() - '0';
		number = digit1*10 + digit2;
	}
	return number;
}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

int read4digits(int lowerLimit, int higherLimit){
	int digit1 = 0;
	int digit2 = 0;
	int digit3 = 0;
	int digit4 = 0;
	int number = -1;
	while (!(number >= lowerLimit && number <= higherLimit)){
		digit1 = SEGGER_RTT_WaitKey() - '0';
		digit2 = SEGGER_RTT_WaitKey() - '0';
		digit3 = SEGGER_RTT_WaitKey() - '0';
		digit4 = SEGGER_RTT_WaitKey() - '0';
		number = digit1*1000 + digit2*100 + digit3*10 + digit4*1;
	}
	return number;
}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

int read5digits(int lowerLimit, int higherLimit){
	int digit1 = 0;
	int digit2 = 0;
	int digit3 = 0;
	int digit4 = 0;
	int digit5 = 0;
	int number = -1;
	while (!(number >= lowerLimit && number <= higherLimit)){
		digit1 = SEGGER_RTT_WaitKey() - '0';
		digit2 = SEGGER_RTT_WaitKey() - '0';
		digit3 = SEGGER_RTT_WaitKey() - '0';
		digit4 = SEGGER_RTT_WaitKey() - '0';
		digit5 = SEGGER_RTT_WaitKey() - '0';
		number = digit1*10000 + digit2*1000 + digit3*100 + digit4*10 + digit5*1;
	}
	return number;
}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

uint16_t assembleCommand(bool D[16]){
	uint16_t command = 0;
	for(int i=0; i<16; i++){
		command |= D[i] << i;
	}
	return command;
}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

void SPIscope(void){
	SEGGER_RTT_printf(0, "\r\t Address (R/W): 0x%x Commands: 0x%x 0x%x \n", payloadBytes[0], payloadBytes[1], payloadBytes[2]);
	OSA_TimeDelay(10);
	SEGGER_RTT_printf(0, "\r\t Bytes Received: 0x%x 0x%x 0x%x \n", inBuffer[0], inBuffer[1], inBuffer[2]);
	OSA_TimeDelay(10);
}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

uint16_t AssembleDAC_DAT(int data){

	bool D_C_DAC_DATA[12];
	for (int i = 0; i < 12; i++)
	{
		D_C_DAC_DATA[i] = (((data >> i) & 1) == 1);
	}

	bool D_C_DAC_DATA_C[16] = {D_C_DAC_DATA[0], D_C_DAC_DATA[1], D_C_DAC_DATA[2], D_C_DAC_DATA[3], D_C_DAC_DATA[4], D_C_DAC_DATA[5], D_C_DAC_DATA[6], D_C_DAC_DATA[7], D_C_DAC_DATA[8], D_C_DAC_DATA[9], D_C_DAC_DATA[10], D_C_DAC_DATA[11], 0, 0, 0, 0};

	uint16_t Scommand = assembleCommand(D_C_DAC_DATA_C);

	return Scommand;
}


/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

int read1char(char input[10]){

	char character = '0';
	while (!((character == input[0] || character == input[1]) || (character == input[2] || character == input[3]) || (character == input[4] || character == input[5]) || (character == input[6] || character == input[7]) || (character == input[8] || character == input[9]))){

		character = SEGGER_RTT_WaitKey();
	}

	int match = 0;

	if(character == input[0] || character == input[1]){
		match = 1;
	}
	else if(character == input[2] || character == input[3]){
		match = 2;
	}
	else if(character == input[4] || character == input[5]){
		match = 3;
	}
	else if(character == input[6] || character == input[7]){
		match = 4;
	}
	else if(character == input[8] || character == input[9]){
		match = 5;
	}

	return  match;

}

/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

int devMAX11300(void){

	/*
		Override Warp firmware's use of these pins.
		Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	*/

	PORT_HAL_SetMuxMode(PORTA_BASE, 6u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
		Override Warp firmware's use of these pins.
		Reconfigure to use as GPIO.
	*/

	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	uint16_t	Scommand1			=	0x0000;	/*	BRST = 0: Default address incrementing mode
													THSHD = 0: Thermal shutdown disabled
													ADCCONV = 00: 200kbps */
	
	uint16_t	Scommand2			=	0x0044;	/*	DACREF = 1: DAC Internal reference
													DACL = 1: Immediate update for DAC configured ports */

	uint16_t	Scommand3			=	0x0047;	/*	Scommand2 + ADCCL initialized as 11 */

	uint16_t	Scommand4_AS		=	0xC000;	/*	AS MODE CONFIGURATION COMMAND */
	uint16_t	Scommand4_DAC		=	0x5100;	/*	DAC MODE CONFIGURATION COMMAND */
	uint16_t	Scommand4_HI		=	0x0000;	/*	HIGH IMPEDANCE PORT MODE CONFIGURATION COMMAND */
	uint16_t	Scommand4_ADC		=	0x7120;	/*	ADC MODE CONFIGURATION COMMAND (Internal/DAC AVR */

	uint16_t	Scommand5			=	0x0020;	/* INTERRUPT REGISTER: DAC OVERCURRENT INTERRUPT 0000 0100 0000 0000 */

	uint8_t PORT_Config_Addr[20]	=	{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33};
	uint8_t ADC_DataPorts[20]		=	{0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53};
	uint8_t DAC_DataPorts[20]		=	{0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73};

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	SEGGER_RTT_WriteString(0, "\r\t Reading Device ID: \n");
	OSA_TimeDelay(10);

	readID();
	OSA_TimeDelay(1);
	SPIscope();

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	SEGGER_RTT_WriteString(0, "\r\t Select Port Configuration: Enter a 20-element vector. \n");
	OSA_TimeDelay(10);
	SEGGER_RTT_WriteString(0, "\r\t Options: Analogue switch (A), DAC Output (O), Waveform Generator (G), High impedance port (H) or ADC Input port (I): \n");
	OSA_TimeDelay(10);

	int AS_number = 0;
	int AS_port[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	int DAC_number = 0;
	int DAC_port[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	int ADC_number = 0;
	int ADC_port[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	int DAC_WAVE_number = 0;
	int DAC_WAVE_port = 0;

	int HI_number = 0;
	int HI_port[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	int length = 20;
	char input_values[10] = {'a', 'A', 'o', 'O', 'g', 'G', 'h', 'H', 'i', 'I'};
	int input = 0;

	int c = 0;
	while(c != 1){

		for(int i = 0; i < length; i++){
			input = read1char(input_values);
			
			if(input == 1){
				AS_number++;
				AS_port[AS_number] = i;
			}
			else if(input == 2){
				DAC_number++;
				DAC_port[DAC_number] = i;
			}
			else if(input == 3){
				DAC_WAVE_number++;
				DAC_WAVE_port = i;
			}
			else if(input == 4){
				HI_number++;
				HI_port[HI_number] = i;
			}
			else if(input == 5){
				ADC_number++;
				ADC_port[ADC_number] = i;
			}
		}

		if(DAC_WAVE_number > 1){
			SEGGER_RTT_WriteString(0, "\r\t Variable DAC Output ALREADY SELECTED \n");
			OSA_TimeDelay(10);
			
			AS_number = 0;
			ADC_number = 0;
			DAC_number = 0;
			HI_number = 0;
			DAC_WAVE_number = 0;
			DAC_WAVE_port = 0;
		}

		else{
			c = 1;
		}

	}

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	/* CONFIGURING All THE PORTS' FUNCTIONS */

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	SEGGER_RTT_WriteString(0, "\r\t Configuration Starts \n");
	OSA_TimeDelay(10);

	SEGGER_RTT_WriteString(0, "\r\t Configure BRST, THSHDN, ADCCONV \n");
	OSA_TimeDelay(10);

	writeCommand(0x10, Scommand1, 0); /* DEVICE CONTROL (0) [Write] */
	OSA_TimeDelay(1);
	SPIscope();

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	if(HI_number >= 1){
		SEGGER_RTT_WriteString(0, "\r\t HIs Configuration \n");
		OSA_TimeDelay(10);

		for (int k = 0; k < HI_number; k++){

			/* --- --- --- HI PORT CONFIGURATION */
			writeCommand(PORT_Config_Addr[HI_port[k]], Scommand4_HI, 0); /* PORT CONTROL - HI (0) [Write] */
			OSA_TimeDelay(1);
			/* SPIscope(); */
			/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */
		}
	}

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	if(DAC_number >= 1){
		
		SEGGER_RTT_WriteString(0, "\r\t DACREF, DACCTL Configuration \n");
		OSA_TimeDelay(10);
		
		writeCommand(0x10, Scommand2, 0); /*   DEVICE CONTROL (0) [Write]  */
		OSA_TimeDelay(1);
		SPIscope();
		
		SEGGER_RTT_WriteString(0, "\r\t DACs Configuration \n");
		OSA_TimeDelay(10);
		
		for (int k = 1; k <= DAC_number; k++){

			/* --- --- --- DAC PORT CONFIGURATION   */
			writeCommand(PORT_Config_Addr[DAC_port[k]], Scommand4_DAC, 0); /* PORT CONTROL - DAC (0) [Write]   */
			OSA_TimeDelay(1);
			/* SPIscope(); */
			/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */
		}
	}

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	if(DAC_WAVE_number == 1){

		SEGGER_RTT_WriteString(0, "\r\t WAVE DAC Configuration \n");
		OSA_TimeDelay(10);

		/* --- --- --- DAC PORT CONFIGURATION   */
		writeCommand(PORT_Config_Addr[DAC_WAVE_port], Scommand4_DAC, 0); /* PORT CONTROL - DAC (0) [Write] */
		OSA_TimeDelay(1);
		/* SPIscope(); */
		/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */
	}

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	if(ADC_number >= 1){

		SEGGER_RTT_WriteString(0, "\r\t ADCs Configuration \n");
		OSA_TimeDelay(10);

		for (int k = 1; k <= ADC_number; k++){

			/* --- --- --- ADC PORT CONFIGURATION */
			writeCommand(PORT_Config_Addr[ADC_port[k]], Scommand4_ADC, 0); /* PORT CONTROL - ADC (0) [Write] */
			OSA_TimeDelay(1);
			SPIscope(); OSA_TimeDelay(10);
			/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */
		}
	}

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	writeCommand(0x10, Scommand3, 0); /* DEVICE CONTROL (0) [Write]  - CONFIGURE ACCTL */
	OSA_TimeDelay(1);
	/* SPIscope(); */

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	if(AS_number >= 1){

		SEGGER_RTT_WriteString(0, "\r\t ASs Configuration \n");
		OSA_TimeDelay(10);

		for (int k = 0; k < AS_number; k++){
			/* --- --- --- AS PORT CONFIGURATION    */
			writeCommand(PORT_Config_Addr[AS_port[k]], Scommand4_AS, 0); /*    PORT CONTROL - AS (0) [Write]   */
			OSA_TimeDelay(1);
			/* SPIscope(); */
			/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */
		}
	}

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	/* --- --- --- CONFIGURE INTERRUPT CONTROL */
	writeCommand(0x11, Scommand5, 0); /* INTERRUPT CONTROL  (0) [Write] */
	OSA_TimeDelay(1);
	SPIscope();
	/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */

	SEGGER_RTT_WriteString(0, "\r\t End of Configuration \n");
	OSA_TimeDelay(10);

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	/* WRITING VALUES TO CONFIGURED DACs AND READ DATA FROM CONFIGURED ADCs */

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	if(DAC_number >= 1){

		SEGGER_RTT_printf(0, "\r\t DAC Configuration Starts. %d DACs selected. \n", DAC_number);
		OSA_TimeDelay(10);
		for (int k = 1; k <= DAC_number; k++){
			SEGGER_RTT_printf(0, "\r\t Select single DAC output value for DAC # %d : \n", k);
			OSA_TimeDelay(10);

			SEGGER_RTT_WriteString(0, "\r\t Enter a value between 00000mV and 10000mV: \n");
			OSA_TimeDelay(10);
			int value = read5digits(0, 10000);

			uint16_t VALUE_Command = AssembleDAC_DAT((value*4095/10000));

			/* --- --- --- CONFIGURE DAC STATIC OUTPUT VALUE */
			writeCommand(DAC_DataPorts[DAC_port[k]], VALUE_Command, 0); /* DAC CONTROL (0) [Write] */
			OSA_TimeDelay(1);
			/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */
		}	
	}

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	if(DAC_WAVE_number == 0){ /* IF NO WAVEFORM GENERATION LOOP IS REQUIRED, READ ALL ADCs DATA */

		if(ADC_number >= 1) {
			SEGGER_RTT_WriteString(0, "\r\t READING ADCs DATA \n");
			OSA_TimeDelay(10);

			for(int k = 1; k <= ADC_number; k++){
				/* --- --- ---  READ ADC REGISTER VALUE */
				writeCommand(ADC_DataPorts[ADC_port[k]], 0x0000, 1); /* ADC CONTROL (1) [Read] */
				OSA_TimeDelay(1);

				uint16_t ADC_DATA = ((inBuffer[1] << 8) | inBuffer[2]) & 0x0FFF;
				/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */

				SEGGER_RTT_printf(0, "\r\t ADC on PORT # %d selected: ADC DATA: %d [mV] \n", ADC_port[k], ADC_DATA*(10000/4096));
				OSA_TimeDelay(10);
			}
		}
	}
	
	else{

		int amplitude = 0;
		int period = 0;

		SEGGER_RTT_WriteString(0, "\r\t Variable DAC Output Configuration \n");
		OSA_TimeDelay(10);

		SEGGER_RTT_WriteString(0, "\r\t Select Number of Steps. Enter a value between 00000 and 10000: \n");
		OSA_TimeDelay(10);

		period = read5digits(0, 10000);

		SEGGER_RTT_WriteString(0, "\r\t Select amplitude. Enter a value between 00000mV and 10000mV: \n");
		OSA_TimeDelay(10);

		amplitude = read5digits(0, 10000);

		SEGGER_RTT_WriteString(0, "\r\t Waveform Generator Starts \n");
		OSA_TimeDelay(10);

		/* --- --- ---  CONFIGURE DAC STATIC OUTPUT VALUE */
		writeCommand(DAC_DataPorts[DAC_WAVE_port], Scommand4_DAC, 0); /* DAC CONTROL  (0) [Write] */
		OSA_TimeDelay(1);
		/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */

		SEGGER_RTT_printf(0, "\r\t DAC # %d selected. \n", DAC_WAVE_port);
		OSA_TimeDelay(10);

		int WAVE_INT = 0;

		for(int l = 0; l <= period; l++){

			float slope = (amplitude*4095/period/1000);
			WAVE_INT = l*slope/10;
			uint16_t WAVE_Command = AssembleDAC_DAT(WAVE_INT);

			/* --- --- ---  CONFIGURE DAC OUTPUT VALUE */
			writeCommand(DAC_DataPorts[DAC_WAVE_port], WAVE_Command, 0); /* DAC CONTROL  (0) [Write] */
			OSA_TimeDelay(1);
			/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- --- */

			/* FOR EACH LOOP ITERATION, READ ALL THE ADCs DATA */
			if(ADC_number >= 1){

				for (int k = 1; k <= ADC_number; k++){
					
					/* --- --- ---  READ ADC REGISTER VALUE */
					writeCommand(ADC_DataPorts[ADC_port[k]], 0x0000, 1); /* ADC CONTROL  (1) [Read] */
					OSA_TimeDelay(1);

					uint16_t ADC_DATA = ((inBuffer[1] << 8) | inBuffer[2]) & 0x0FFF;
					/* --- --- ---  --- --- ---  --- --- ---  --- --- ---  --- --- ---  */

					SEGGER_RTT_printf(0, "\r\t ADC on PORT # %d selected: ADC DATA: %d [mV] \n", ADC_port[k], ADC_DATA*(10000/4096));
					OSA_TimeDelay(10);
				}
			}
		}
	}

	/*		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		-		*/

	return 0;

	SEGGER_RTT_WriteString(0, "\r\t Test ENDS \n");
	OSA_TimeDelay(10);

}


