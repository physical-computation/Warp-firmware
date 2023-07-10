/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

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
void		initBMX055accel(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
void		initBMX055gyro(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
void		initBMX055mag(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

WarpStatus	writeSensorRegisterBMX055accel(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue);
WarpStatus	writeSensorRegisterBMX055gyro(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue);
WarpStatus	writeSensorRegisterBMX055mag(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue);

WarpStatus	configureSensorBMX055accel(uint8_t payloadPMU_RANGE,
					uint8_t payloadACCD_HBW,
					uint16_t menuI2cPullupValue);
WarpStatus	configureSensorBMX055gyro(uint8_t payloadRANGE,
					uint8_t payloadBW,
					uint8_t payloadLPM1,
					uint8_t payloadRATE_HBW);
WarpStatus configureSensorBMX055mag(uint8_t payloadPowerCtrl,
					uint8_t payloadOpMode);

WarpStatus readSensorRegisterBMX055accel(uint8_t deviceRegister, int numberOfBytes);
WarpStatus readSensorRegisterBMX055gyro(uint8_t deviceRegister, int numberOfBytes);
WarpStatus readSensorRegisterBMX055mag(uint8_t deviceRegister, int numberOfBytes);

void 		printSensorDataBMX055accel(bool hexModeFlag);
void 		printSensorDataBMX055gyro(bool hexModeFlag);
void 		printSensorDataBMX055mag(bool hexModeFlag);

uint8_t 	appendSensorDataBMX055accel(uint8_t* buf);
uint8_t 	appendSensorDataBMX055gyro(uint8_t* buf);
uint8_t 	appendSensorDataBMX055mag(uint8_t* buf);

const uint8_t bytesPerMeasurementBMX055            = 16;
const uint8_t bytesPerReadingBMX055                = 2;
const uint8_t numberOfReadingsPerMeasurementBMX055 = 8;