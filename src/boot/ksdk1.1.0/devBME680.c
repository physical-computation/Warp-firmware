/*
	Authored 2016-2018. Phillip Stanley-Marbell, Youchao Wang, James Meech.

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
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_port_hal.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include <math.h>
//#define UINT_MAX 4294967295
#define BME680_CONCAT_BYTES(msb, lsb)	(((uint16_t)msb << 8) | (uint16_t)lsb)
#define BME680_CONCAT_BYTESxlsb(msb, xlsb)	(((uint32_t)msb << 4) | (uint32_t)xlsb)
float calcT(uint8_t temp_msb, uint8_t temp_lsb, uint8_t temp_xlsb, uint8_t CalibVals[]);
float calcH(uint8_t hum_msb, uint8_t hum_lsb, uint8_t CalibVals[], float temp_comp); 
//float calcP(uint8_t pres_msb, uint8_t pres_lsb, uint8_t pres_xlsb, uint8_t CalibVals[], float t_fine);
float findMean(float samples[], int size);
float findStandardDeviation(float samples[], float mean, int size);
float findCorrelationCoefficient(float samplesX[], float samplesY[], float meanX, float meanY, float standardDeviationX, float standardDeviationY, int size);
float Skewness(float samples[], float mean, float standardDeviation, int size);
float Kurtosis(float samples[], float mean, float standardDeviation, int size);
float findNewUncertainty(float samplesH[], float meanH, float sigmaH, float rhoHT, float sigmaT, int size);
float normalPDF(float h, float meanH, float sigmaH, float sigmaT, float rhoHT);
//float sqrt1(const float x);
//float exp(float x);
void Swap(float *a, float *b);
void BubbleSort(float *arr, int n);
//unsigned int lfsr113_Bits(void);
//void boxMueller(float mu, float sigma, float *z0, float *z1);
//float log(float x);
//float cos(float x);
//float sin(float x);

extern volatile WarpI2CDeviceState	deviceBME680State;
extern volatile uint8_t			deviceBME680CalibrationValues[];
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
/*
union {
    int i;
    float x;
} u;
*/
void
initBME680(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskPressure | kWarpTypeMaskTemperature);

	return;
}

WarpStatus
writeSensorRegisterBME680(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	if (deviceRegister > 0xFF)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterBME680(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);

	/*
	 *	We only check to see if it is past the config registers.
	 *
	 *	TODO: We should eventually numerate all the valid register addresses
	 *	(configuration, control, and calibration) here.
	 */
	if (deviceRegister > kWarpSensorConfigurationRegisterBME680CalibrationRegion2End)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBME680State.i2cBuffer,
							1,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


WarpStatus
configureSensorBME680(uint8_t payloadCtrl_Hum, uint8_t payloadCtrl_Meas, uint8_t payloadGas_0, uint16_t menuI2cPullupValue)
{
	uint8_t		reg, index = 0;
	WarpStatus	status1, status2, status3, status4 = 0;

	status1 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Hum,
							payloadCtrl_Hum,
							menuI2cPullupValue);

	status2 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas,
							payloadCtrl_Meas,
							menuI2cPullupValue);

	status3 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Gas_0,
							payloadGas_0,
							menuI2cPullupValue);


	/*
	 *	Read the calibration registers
	 */
	uint8_t addr = 0x89; 
	for (int i = 0; i < 25; i++)
	{
		status4 |= readSensorRegisterBME680(addr, 1 /* numberOfBytes */);
		deviceBME680CalibrationValues[i] = deviceBME680State.i2cBuffer[0];
		addr++;
	}
	addr = 0xE1;
	for (int i = 0; i < 16; i++)
	{
		status4 |= readSensorRegisterBME680(addr, 1 /* numberOfBytes */);
		deviceBME680CalibrationValues[i+25] = deviceBME680State.i2cBuffer[0];
		addr++;
	}

	return (status1 | status2 | status3 | status4);
}


void
printSensorDataBME680(bool hexModeFlag, uint16_t menuI2cPullupValue)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	uint16_t	readSensorRegisterValueXLSB;
	uint16_t 	unsignedRawAdcValueUint16_t;
	uint32_t	unsignedRawAdcValueUint32_t;
	WarpStatus	triggerStatus, i2cReadStatusMSB, i2cReadStatusLSB, i2cReadStatusXLSB;
    const int samplesPerDistribution= 30; 
	float temperatureDistribution[samplesPerDistribution];
	float humidityDistribution[samplesPerDistribution];
	//float pressureDistribution[samplesPerDistribution];
	float temperatureMean;
	//float pressureMean;
	float humidityMean; 
	float temperatureStandardDeviation;
	//float pressureStandardDeviation;
	float humidityStandardDeviation;
	float correlation; 
	float temperatureSkewness;
	float humiditySkewness; 
	float temperatureKurtosis;
	float humidityKurtosis; 
	float r0 = 0;
	float r1 = 0;
    for(int i = 0; i < samplesPerDistribution; i++)
	{
		/*
		*	First, trigger a measurement
		*/
		triggerStatus = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas,
								0b00100101,
								menuI2cPullupValue);

		i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_msb, 1);
		readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
		i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_lsb, 1);
		readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
		i2cReadStatusXLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_xlsb, 1);
		readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
		temperatureDistribution[i] = calcT(readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueXLSB, deviceBME680CalibrationValues);
		/*
		i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_msb, 1);
		readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
		i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_lsb, 1);
		readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
		i2cReadStatusXLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_xlsb, 1);
		readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
		pressureDistribution[i] = calcP(readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueXLSB, deviceBME680CalibrationValues, temperatureDistribution[i]);
		*/
		i2cReadStatusMSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680hum_msb, 1);
		readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
		i2cReadStatusLSB = readSensorRegisterBME680(kWarpSensorOutputRegisterBME680hum_lsb, 1);
		readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
		humidityDistribution[i] = calcH(readSensorRegisterValueMSB, readSensorRegisterValueLSB, deviceBME680CalibrationValues, temperatureDistribution[i]);
	}
	temperatureMean = findMean(temperatureDistribution, samplesPerDistribution);
	humidityMean = findMean(humidityDistribution, samplesPerDistribution);
	//pressureMean = findMean(pressureDistribution, samplesPerDistribution);
	temperatureStandardDeviation = findStandardDeviation(temperatureDistribution, temperatureMean, samplesPerDistribution);
	humidityStandardDeviation = findStandardDeviation(humidityDistribution, humidityMean, samplesPerDistribution); 
	temperatureSkewness = Skewness(temperatureDistribution, temperatureMean, temperatureStandardDeviation, samplesPerDistribution); 
	humiditySkewness = Skewness(humidityDistribution, humidityMean, humidityStandardDeviation, samplesPerDistribution); 
	temperatureKurtosis = Kurtosis(temperatureDistribution, temperatureMean, temperatureStandardDeviation, samplesPerDistribution); 
	humidityKurtosis = Kurtosis(humidityDistribution, humidityMean, humidityStandardDeviation, samplesPerDistribution); 
	if((abs(temperatureSkewness) < 2) && (abs(humiditySkewness) < 2) && (temperatureKurtosis < 7) && (humidityKurtosis < 7))
	{
		SEGGER_RTT_printf(0, " %u.%05u, ", (int)(temperatureMean), (int)((temperatureMean - (int)(temperatureMean))*100000)); 
		SEGGER_RTT_printf(0, "%u.%05u, ", (int)(temperatureStandardDeviation), (int)((temperatureStandardDeviation - (int)(temperatureStandardDeviation))*100000)); 
		SEGGER_RTT_printf(0, "%u.%05u, ", (int)(humidityMean), (int)((humidityMean - (int)(humidityMean))*100000)); 
		SEGGER_RTT_printf(0, "%u.%05u, ", (int)(humidityStandardDeviation), (int)((humidityStandardDeviation - (int)(humidityStandardDeviation))*100000));
		correlation = findCorrelationCoefficient(humidityDistribution, temperatureDistribution, humidityMean, temperatureMean, humidityStandardDeviation, temperatureStandardDeviation, samplesPerDistribution);
		BubbleSort(humidityDistribution, samplesPerDistribution);
		humidityStandardDeviation = findNewUncertainty(humidityDistribution, humidityMean, humidityStandardDeviation, correlation, temperatureStandardDeviation, samplesPerDistribution);
		SEGGER_RTT_printf(0, "%u.%05u, ", (int)(humidityStandardDeviation), (int)((humidityStandardDeviation - (int)(humidityStandardDeviation))*100000));

	}
}

float calcT(uint8_t temp_msb, uint8_t temp_lsb, uint8_t temp_xlsb, uint8_t CalibVals[]) 
	{ 	 	 	 
	// Put together the  temperature calibration parameters out of values from the calibration register
	uint16_t par_t1 =(uint16_t) (BME680_CONCAT_BYTES(CalibVals[34], CalibVals[33])); 
	int16_t  par_t2 =(uint16_t) (BME680_CONCAT_BYTES(CalibVals[2], CalibVals[1])); 	
	int8_t   par_t3 =(uint8_t)  CalibVals[3];   
	
	// Get the whole temperature value from the three registers it is spread across	 
	uint32_t temp_adc = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4); 
		 	 	 	 
	// Define variables to be used to calculate the compensated temperature
	float t_fine;
	float var1 = 0;
	float var2 = 0;
	float calc_temp = 0;

	/* calculate var1 data */
	var1  = ((((float)temp_adc / 16384.0f) - ((float)par_t1 / 1024.0f))
			* ((float)par_t2));

	/* calculate var2 data */
	var2  = (((((float)temp_adc / 131072.0f) - ((float)par_t1 / 8192.0f)) *
		(((float)temp_adc / 131072.0f) - ((float)par_t1 / 8192.0f))) *
		((float)par_t3 * 16.0f));

	/* t_fine value*/
	t_fine = (var1 + var2); 

	/* compensated temperature data*/
	calc_temp  = ((t_fine) / 5120.0f);
	// Store the calculated temperature in the array that was passed by reference
	return calc_temp; 
	} 

	float calcH(uint8_t hum_msb, uint8_t hum_lsb, uint8_t CalibVals[], float temp_comp)
	{ 

	int16_t hum_adc= BME680_CONCAT_BYTES(hum_msb,hum_lsb);   

	// Combine the componets from the calibration array to get the calibration parameters
	uint16_t par_h1 = (uint16_t)(((uint16_t) CalibVals[27] << 4) | (CalibVals[26] & 0x0F));
	uint16_t par_h2 = (uint16_t)(((uint16_t) CalibVals[25] << 4) | (CalibVals[26] >> 4));
	int8_t   par_h3 = (int8_t) CalibVals[28]; 
	int8_t   par_h4 = (int8_t) CalibVals[29];
	int8_t   par_h5 = (int8_t) CalibVals[30]; 
	uint8_t  par_h6 = (uint8_t) CalibVals[31];
	int8_t   par_h7 = (int8_t) CalibVals[32];

	// Define variables to be used to calculate the compensated humidity
	float calc_hum = 0; 
	float var1 = 0;
	float var2 = 0;
    float var3 = 0;
	float var4 = 0;

	
        // Calculate humidity 
	var1 = (float)((float)hum_adc) - (((float)par_h1 * 16.0f) + (((float)par_h3 / 2.0f)
		* temp_comp));

	var2 = var1 * ((float)(((float) par_h2 / 262144.0f) * (1.0f + (((float)par_h4 / 16384.0f)
		* temp_comp) + (((float)par_h5 / 1048576.0f) * temp_comp * temp_comp))));

	var3 = (float) par_h6 / 16384.0f;

	var4 = (float) par_h7 / 2097152.0f;


	calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
	// Humidity  can't be more than 100 % or less than 0 %
	if (calc_hum > 100.0f)
		calc_hum = 100.0f;
	else if (calc_hum < 0.0f)
		calc_hum = 0.0f;

	// Store the calculated temperature in the array that was passed by reference
	return calc_hum; 
	}
    
float findMean(float samples[], int size)
{ 
    float sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += samples[i];
    }
    sum = sum/size; 
    return sum; 
}

float findStandardDeviation(float samples[], float mean, int size)
{
    float sum = 0;
    for(int i = 0; i < size; i++) 
    {
        sum += (samples[i]-mean)*(samples[i]-mean);
    }
    sum = sqrt(sum/size); 
    return sum; 
}

float findCorrelationCoefficient(float samplesX[], float samplesY[], float meanX, float meanY, float standardDeviationX, float standardDeviationY, int size)
{ 
    float sumTop = 0;
    float sumBottom = 0;
    for(int i = 0; i < size; i++) 
    {
        sumTop += (samplesX[i]-meanX)*(samplesY[i]-meanY); 
    }
    sumBottom = sqrt(standardDeviationX*standardDeviationX*size*standardDeviationY*standardDeviationY*size);
    return sumTop/sumBottom; 
}

float Skewness(float samples[], float mean, float standardDeviation, int size)
{
  float Skewness = 0;
  for(int i = 0; i < size; i++)
  {
  Skewness += (samples[i] - mean)*(samples[i] - mean)*(samples[i] - mean);
  }
  Skewness = Skewness/(size*standardDeviation*standardDeviation*standardDeviation);
  return Skewness;
}

float Kurtosis(float samples[], float mean, float standardDeviation, int size)
{
  float Kurtosis = 0;
  for(int i = 0; i < size; i++)
  {
  Kurtosis += (samples[i] - mean)*(samples[i] - mean)*(samples[i] - mean)*(samples[i] - mean);
  }
  Kurtosis = Kurtosis/(size*standardDeviation*standardDeviation*standardDeviation*standardDeviation);
  return Kurtosis;
}

float findNewUncertainty(float samplesH[], float meanH, float sigmaH, float rhoHT, float sigmaT, int size)
{
    float A = 0;
    float b = 0;
    float h = 0;  
    
    for(int i = 0; i < size-1; i++)
    {
        b = samplesH[i+1] - samplesH[i];
        h = (normalPDF(samplesH[i], meanH, sigmaH, sigmaT, rhoHT) + normalPDF(samplesH[i+1], meanH, sigmaH, sigmaT, rhoHT))/2; 
        A += b*h; 
    }
    return 1.0/(A*sqrt(2*M_PI));
}

float normalPDF(float h, float meanH, float sigmaH, float sigmaT, float rhoHT) 
{
    float top = ((h-meanH)/sigmaH);
	top = top*top;
    float bottom = 2*(1-(rhoHT*rhoHT));
    float mult = 1/(2*M_PI*sigmaT*sigmaH*sqrt(1-(rhoHT*rhoHT)));
    return mult*exp(-(top/bottom));
} 

void Swap(float *a, float *b)
{
    float temp = *a;
    *a = *b;
    *b = temp;
}
//Function to sort the array
void BubbleSort(float *arr, int n)
{
    int i = 0, j = 0;
    for (i = 0; i < n-1; i++)
    {
        for (j = 0; j < n-i-1; j++)
        {
            if (arr[j] > arr[j+1])
            {
                Swap(&arr[j], &arr[j+1]);
            }
        }
    }
}
