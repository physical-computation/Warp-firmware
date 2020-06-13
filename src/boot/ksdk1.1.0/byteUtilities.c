/*
	Authored 2016-2020. Phillip Stanley-Marbell, Youchao Wang AND James Meech.

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
#include "fsl_misc_utilities.h"

int16_t     
AMG8834ByteUtility(uint16_t	readSensorRegisterValueMSB, uint16_t readSensorRegisterValueLSB)
{
   /*
	*	Format is 12 bits with the highest-order bit being a sign (0 +ve, 1 -ve)
    */
    int16_t readSensorRegisterValueCombined	= ((readSensorRegisterValueMSB & 0x07) << 8) | (readSensorRegisterValueLSB & 0xFF);
	readSensorRegisterValueCombined *= ((readSensorRegisterValueMSB & (1 << 3)) == 0 ? 1 : -1); 
   /*
	*	Specification, page 14/26, says LSB counts for 0.25 C (1/4 C)
	*/
	readSensorRegisterValueCombined >>= 2;
    return readSensorRegisterValueCombined;
} 

int16_t 
BMX055AccelerometerByteUtility(uint16_t MSB, uint16_t LSB) 
{
	int16_t readSensorRegisterValueCombined = ((MSB & 0xFF) << 4) | (LSB >> 4);

	/*
	 *	Sign extend the 12-bit value based on knowledge that upper 4 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 11)) - (1 << 11);
	return readSensorRegisterValueCombined;
}

int16_t 
BMX055MagnetometerByteUtility(uint16_t MSB, uint16_t LSB) 
{
	int16_t readSensorRegisterValueCombined = ((MSB & 0xFF) << 5) | (LSB >> 3);

	/*
	 *	Sign extend the 13-bit value based on knowledge that upper 3 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 12)) - (1 << 12);
	return readSensorRegisterValueCombined;
}

uint16_t 
concatTwoBytes(uint16_t MSB, uint16_t LSB) 
{
	uint16_t  MSBandLSBCombined = ((MSB & 0xFF) << 8) | (LSB & 0xFF);
	return MSBandLSBCombined;
}

uint32_t 
concatTwoPointFiveBytes(uint16_t MSB, uint16_t LSB, uint16_t XLSB) 
{
	uint32_t  MSBandLSBandXLSBCombined =
			((MSB & 0xFF)  << 12) |
			((LSB & 0xFF)  << 4)  |
			((XLSB & 0xF0) >> 4);
	return MSBandLSBandXLSBCombined;
}

int16_t
MMA8451QAccelerometerByteUtility(uint16_t MSB, uint16_t LSB) 
{
	int16_t MSBandLSBCombined = ((MSB & 0xFF) << 6) | (LSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	MSBandLSBCombined = (MSBandLSBCombined ^ (1 << 13)) - (1 << 13); 
	return MSBandLSBCombined;
}


