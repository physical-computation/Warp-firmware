/*
	Authored 2019. Sam Willis. Additional contributions, 2019-onwards,
	see git log.

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

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_rtc_driver.h"
#include "fsl_i2c_master_driver.h"

#include "warp.h"
#include "devRV8803C7.h"

extern volatile WarpI2CDeviceState	deviceRV8803C7State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;


void
initRV8803C7(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceRV8803C7State.i2cAddress			= i2cAddress;
	deviceRV8803C7State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
readRTCRegisterRV8803C7(uint8_t deviceRegister, uint8_t *receiveData)
{
	/*
	 *	Read address in 'deviceRegister' into 'receiveData' over i2c from the RTC
	 */
	uint8_t		cmdBuff[1];
	i2c_status_t	status;

	if (deviceRegister > 0x2F) {
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave = {
		.address = deviceRV8803C7State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuff[0] = deviceRegister;

	warpScaleSupplyVoltage(deviceRV8803C7State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	status = I2C_DRV_MasterReceiveDataBlocking(0 /* I2C instance */,
							&slave,
							cmdBuff,
							1,
							receiveData,
							1,
							gWarpI2cTimeoutMilliseconds);
	warpDisableI2Cpins();

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readRTCRegistersRV8803C7(uint8_t deviceStartRegister, uint8_t nRegs, uint8_t *  receiveData)
{
	/*
	 *	Read 'nRegs' number of consecutive addresses from 'deviceStartRegister' into 'receiveData'
	 *	over i2c from the RTC
	 */
	uint8_t		cmdBuff[1];
	i2c_status_t	status;

	if (deviceStartRegister > 0x2F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave = {
		.address = deviceRV8803C7State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuff[0] = deviceStartRegister;

	warpScaleSupplyVoltage(deviceRV8803C7State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	status = I2C_DRV_MasterReceiveDataBlocking(0 /* I2C instance */,
							&slave,
							cmdBuff,
							1,
							receiveData,
							nRegs,
							gWarpI2cTimeoutMilliseconds);
	warpDisableI2Cpins();

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
writeRTCRegisterRV8803C7(uint8_t deviceRegister, uint8_t payload)
{
	/*
	 *	Write the value in 'payload' to the to the address in 'deviceRegister' over i2c to the RTC
	 */
	uint8_t		cmdBuff[1];
	uint8_t		txBuff[1];
	i2c_status_t	status;


	if (deviceRegister > 0x2F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave = {
		.address = deviceRV8803C7State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuff[0] = deviceRegister;
	txBuff[0] = payload;

	warpScaleSupplyVoltage(deviceRV8803C7State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	status = I2C_DRV_MasterSendDataBlocking(0 /* I2C instance */,
						&slave,
						cmdBuff,
						1,
						txBuff,
						1,
						gWarpI2cTimeoutMilliseconds);
	warpDisableI2Cpins();

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
writeRTCRegistersRV8803C7(uint8_t deviceStartRegister, uint8_t nRegs, uint8_t payload[])
{
	/*
	 *	Write to fill 'nRegs' number of consecutive registers, starting with the address at 'deviceStartRegister'
	 *	with the values in 'payload' over i2c to the RTC.
	 */
	uint8_t		cmdBuff[1];
	i2c_status_t	status;

	if (deviceStartRegister > 0x2F)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave = {
		.address = deviceRV8803C7State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuff[0] = deviceStartRegister;

	warpScaleSupplyVoltage(deviceRV8803C7State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	status = I2C_DRV_MasterSendDataBlocking(0 /* I2C instance */,
						&slave,
						cmdBuff,
						1,
						payload,
						nRegs,
						gWarpI2cTimeoutMilliseconds);
	warpDisableI2Cpins();

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

uint8_t
bin2bcd(uint8_t bin)
{
	/*
	 *	Convert a int to bcd format
	 */
	uint8_t		r, d, bcd;
	d = bin;
	bcd = 0;

	for (uint8_t n = 0; n<16; n+=4)
	{
		r = d % 10;
		d /= 10;
		bcd |= (r << n);
	}
	return bcd;
}

uint8_t
date2weekday(uint8_t day, uint8_t month, uint8_t year)
{
	/*
	 *	Returns day of week based on date
	 *	TODO replace with more understandable expression
	 */
	uint8_t	weekday  = (day += month < 3 ? year-- : year - 2, 23*month/9 + day + 4 + year/4- year/100 + year/400)%7;
	return	weekday;
}

WarpStatus
setRTCTimeRV8803C7(rtc_datetime_t *tm)
{
	/*
	 *	Set the time and date of the RV-8803-C7
	 */
	WarpStatus	ret;
	uint8_t		ctrl, flags;


	ret = readRTCRegisterRV8803C7(kWarpRV8803RegCtrl, &ctrl);

	/*
	 *	TODO: Is there special handling needed when ctrl < 0?
	 */
	if (ret | (ctrl < 0))
	{
		return ret;
	}

	/*
	 *	Stop the clock
	 */
	ctrl |= kWarpRV8803CtrlRESET;
	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegCtrl, ctrl);
	if (ret)
	{
		return ret;
	}

	/*
	 *	Program the time and date
	 */
	uint8_t weekday = date2weekday(tm->day, tm->month, tm->year);
	uint8_t date[7] = {
				bin2bcd(tm->second),
				bin2bcd(tm->minute),
				bin2bcd(tm->hour),
				1 << weekday,
				bin2bcd(tm->day),
				bin2bcd(tm->month + 1),
				bin2bcd(tm->year - 100)
	};

	ret = writeRTCRegistersRV8803C7(kWarpRV8803RegSec, 7, date);
	if (ret)
	{
		return ret;
	}

	/*
	 *	Restart the clock
	 */
	ctrl &= kWarpRV8803CtrlRESET;
	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegCtrl, ctrl);
	if (ret)
	{
		return ret;
	}

	ret = readRTCRegisterRV8803C7(kWarpRV8803RegFlag, &flags);

	/*
	 *	TODO: Is there special handling needed when flags < 0?
	 */
	if (ret | (flags < 0))
	{
		return ret;
	}

	flags &= ~(kWarpRV8803FlagV1F | kWarpRV8803FlagV2F);
	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegFlag, flags);


	return ret;
}

WarpStatus
setRTCCountdownRV8803C7(uint16_t countdown, WarpRV8803ExtTD clk_freq, bool interupt_enable)
{
	uint8_t		ext, flags, ctrl, ret;

	/*
	 *	Set the countdown timer and if it should cause the interupt pin of the RV8803 to activate
	 */
	if (countdown > 4095)
	{
		return kWarpStatusBadDeviceCommand;
	}

	ret = readRTCRegisterRV8803C7(kWarpRV8803RegExt, &ext);
	if (ret)
	{
		return ret;
	}

	ret = readRTCRegisterRV8803C7(kWarpRV8803RegFlag, &flags);
	if (ret)
	{
		return ret;
	}

	ret = readRTCRegisterRV8803C7(kWarpRV8803RegCtrl, &ctrl);
	if (ret)
	{
		return ret;
	}

	ext &= ~kWarpRV8803ExtTE;	/*	Stop countdown			*/
	ext &= kWarpRV8803ExtClrTD;	/*	Clear the timer clock frequency	*/
	ext |= clk_freq; 		/*	Set the timer clock frequency	*/

	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegExt, ext);
	if (ret)
	{
		return ret;
	}

	flags &= ~kWarpRV8803FlagTF; /* clear the timer flag */
	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegFlag, flags);
	if (ret)
	{
		return ret;
	}

	/*
	 *	Set the number of counts before the timer is triggered
	 */
	uint8_t	MSByte = (countdown >> 8) & 0xFF;
	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegTimerCounter1, MSByte);
	if (ret)
	{
		return ret;
	}

	uint8_t	LSByte = countdown & 0xFF;
	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegTimerCounter0, LSByte);
	if (ret)
	{
		return ret;
	}

	/*
	 *	Set the signal interrupt pin on countdown
	 */
	if (interupt_enable)
	{
		ctrl |= kWarpRV8803CtrlTIE;
	}
	else
	{
		ctrl &= ~kWarpRV8803CtrlTIE;
	}

	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegCtrl, ctrl);
	if (ret)
	{
		return ret;
	}

	/*
	 *	Start countdown again
	 */
	ext |= kWarpRV8803ExtTE;
	ret = writeRTCRegisterRV8803C7(kWarpRV8803RegExt, ext);


	return ret;
}


uint8_t
appendSensorDataRV8803C7(uint8_t* buf)
{
	uint8_t	tmpRV8803RegisterByte;
	WarpStatus status;
	uint8_t index = 0;

	status = readRTCRegisterRV8803C7(kWarpRV8803RegHour, &tmpRV8803RegisterByte);

	if (status != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		buf[index] = tmpRV8803RegisterByte;
		index += 1;
	}

	status = readRTCRegisterRV8803C7(kWarpRV8803RegMin, &tmpRV8803RegisterByte);

	if (status != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		buf[index] = tmpRV8803RegisterByte;
		index += 1;
	}

	status = readRTCRegisterRV8803C7(kWarpRV8803RegSec, &tmpRV8803RegisterByte);

	if (status != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;
	}
	else
	{
		buf[index] = tmpRV8803RegisterByte;
		index += 1;
	}


	return index;
}