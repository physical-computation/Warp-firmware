/*
	Authored 2019. Sam Willis.

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

//#include <stdlib.h>
//#include <stdint.h>

#include "devRV8803C7.h"
//#include "warp.h"

#include "fsl_i2c_master_driver.h"
//#include "fsl_rtc_driver.h"

#define RV8803_SEC 0x00
#define RV8803_MIN 0x01
#define RV8803_HOUR 0x02
#define RV8803_WEEKDAY 0x03
#define RV8803_DATE 0x04
#define RV8803_MONTH 0x05
#define RV8803_YEAR 0x06
#define RV8803_RAM 0x07
#define RV8803_MIN_ALARM 0x08
#define RV8803_HOUR_ALARM 0x09
#define RV8803_WEEKDAY_OR_DATE_ALARM 0x0A
#define RV8803_TIMER_COUNTER_0 0x0B
#define RV8803_TIMER_COUNTER_1 0x0C
#define RV8803_EXT 0x0D
#define RV8803_FLAG 0x0E
#define RV8803_CTRL 0x0F

#define BIT(n) (uint8_t)1U << n

#define RV8803_EXT_TD_CLR 0xFC
#define RV8803_EXT_FD_CLR 0xF3

#define RV8803_EXT_TE BIT(4)
#define RV8803_EXT_USEL BIT(5)
#define RV8803_EXT_WADA BIT(6)
#define RV8803_EXT_TEST BIT(7)

#define RV8803_FLAG_V1F	BIT(0)
#define RV8803_FLAG_V2F BIT(1)
#define RV8803_FLAG_EVF BIT(2)
#define RV8803_FLAG_AF BIT(3)
#define RV8803_FLAG_TF BIT(4)
#define RV8803_FLAG_UF BIT(5)

#define RV8803_CTRL_RESET BIT(0)

#define RV8803_CTRL_EIE BIT(2)
#define RV8803_CTRL_AIE BIT(3)
#define RV8803_CTRL_TIE BIT(4)
#define RV8803_CTRL_UIE BIT(5)

extern volatile WarpI2CDeviceState deviceRV8803C7State;
extern volatile uint32_t gWarpI2cBaudRateKbps;
extern volatile uint32_t gWarpI2cTimeoutMilliseconds;

void initRV8803C7(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer) {
  deviceStatePointer->i2cAddress = i2cAddress;
  return;
}

WarpStatus readRTCRegisterRV8803C7(uint8_t deviceRegister, uint8_t *receiveData) {
  // read address in 'deviceRegister' into 'receiveData' over i2c from the RTC
  uint8_t cmdBuff[1];
  i2c_status_t status;

  if (deviceRegister > 0xFF) {
    return kWarpStatusBadDeviceCommand;
  }


  i2c_device_t slave = {
                        .address = deviceRV8803C7State.i2cAddress,
                        .baudRate_kbps = gWarpI2cBaudRateKbps
  };

  cmdBuff[0] = deviceRegister;
  status = I2C_DRV_MasterReceiveDataBlocking(0 /* I2C instance */,
                                             &slave,
                                             cmdBuff,
                                             1,
                                             receiveData,
                                             1,
                                             gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success) {
    return kWarpStatusDeviceCommunicationFailed;
  }

	return kWarpStatusOK;
}

WarpStatus readRTCRegistersRV8803C7(uint8_t deviceStartRegister, uint8_t nRegs, uint8_t *receiveData) {
  // read 'nRegs' number of consecutive addresses from 'deviceStartRegister' into 'receiveData' over i2c from the RTC
  uint8_t cmdBuff[1];
  i2c_status_t status;

  if (deviceStartRegister > 0xFF) {
    return kWarpStatusBadDeviceCommand;
  }

  i2c_device_t slave = {
                        .address = deviceRV8803C7State.i2cAddress,
                        .baudRate_kbps = gWarpI2cBaudRateKbps
  };

  cmdBuff[0] = deviceStartRegister;
  status = I2C_DRV_MasterReceiveDataBlocking(0 /* I2C instance */,
                                             &slave,
                                             cmdBuff,
                                             1,
                                             receiveData,
                                             nRegs,
                                             gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success) {
    return kWarpStatusDeviceCommunicationFailed;
  }

	return kWarpStatusOK;
}

WarpStatus writeRTCRegisterRV8803C7(uint8_t deviceRegister, uint8_t payload) {
  // write the value in 'payload' to the to the address in 'deviceRegister' over i2c to the RTC
  uint8_t cmdBuff[1], txBuff[1];
  i2c_status_t status;

  if (deviceRegister > 0xFF) {
    return kWarpStatusBadDeviceCommand;
  }

  i2c_device_t slave = {
                        .address = deviceRV8803C7State.i2cAddress,
                        .baudRate_kbps = gWarpI2cBaudRateKbps
  };

  cmdBuff[0] = deviceRegister;
	txBuff[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(0 /* I2C instance */,
                                          &slave,
                                          cmdBuff,
                                          1,
                                          txBuff,
                                          1,
                                          gWarpI2cTimeoutMilliseconds);

  if (status != kStatus_I2C_Success) {
    return kWarpStatusDeviceCommunicationFailed;
  }

	return kWarpStatusOK;
}

WarpStatus writeRTCRegistersRV8803C7(uint8_t deviceStartRegister, uint8_t nRegs, uint8_t payload[]) {
  // write to fill 'nRegs' number of consecutive registers, starting with the address at 'deviceStartRegister'
  // with the values in 'payload' over i2c to the RTC.
  uint8_t cmdBuff[1];
  i2c_status_t status;

  if (deviceStartRegister > 0xFF) {
    return kWarpStatusBadDeviceCommand;
  }

  i2c_device_t slave = {
                        .address = deviceRV8803C7State.i2cAddress,
                        .baudRate_kbps = gWarpI2cBaudRateKbps
  };

  cmdBuff[0] = deviceStartRegister;
	status = I2C_DRV_MasterSendDataBlocking(0 /* I2C instance */,
                                          &slave,
                                          cmdBuff,
                                          1,
                                          payload,
                                          nRegs,
                                          gWarpI2cTimeoutMilliseconds);

  if (status != kStatus_I2C_Success) {
    return kWarpStatusDeviceCommunicationFailed;
  }

	return kWarpStatusOK;
}

uint8_t bin2bcd(uint8_t bin) {
  // convert a int to bcd format
  uint8_t r, d, bcd;
  d = bin;
  bcd = 0;
  for (uint8_t n = 0; n<16; n+=4) {
    r = d % 10;
    d /= 10;
    bcd |= (r << n);
  }
  return bcd;
}

//typedef enum { SUNDAY, MONDAY, TUESDAY, WEDNESDAY, THURSDAY, FRIDAY, SATURDAY
//} WEEKDAY;
uint8_t date2weekday(uint8_t day, uint8_t month, uint8_t year) {
  // returns day of week based on date, TODO replace with more understandable expression
  uint8_t weekday  = (day += month < 3 ? year-- : year - 2, 23*month/9 + day + 4 + year/4- year/100 + year/400)%7;
  return weekday;
}

WarpStatus setRTCTimeRV8803C7(rtc_datetime_t *tm) {
  // set the time and date of the RV-8803-C7
  WarpStatus ret;
	uint8_t ctrl, flags;
  ret = readRTCRegisterRV8803C7(RV8803_CTRL, &ctrl);
	if (ret | ctrl < 0) // check what I need to do if ctrl < 0
		return ret;

  /* Stop the clock */
  ret = writeRTCRegisterRV8803C7(RV8803_CTRL, ctrl | RV8803_CTRL_RESET);
	if (ret)
		return ret;

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

  ret = writeRTCRegistersRV8803C7(RV8803_SEC, 7, date);
	if (ret)
		return ret;

	/* Restart the clock */
  ret = writeRTCRegisterRV8803C7(RV8803_CTRL, ctrl & ~RV8803_CTRL_RESET);
	if (ret)
		return ret;

  ret = readRTCRegisterRV8803C7(RV8803_FLAG, &flags);
	if (ret | flags < 0) // check what I need to do if flags < 0
		return ret;


  ret = writeRTCRegisterRV8803C7(RV8803_FLAG, flags & ~(RV8803_FLAG_V1F | RV8803_FLAG_V2F));
	return ret;
}

WarpStatus setRTCCountdownRV8803C7(uint16_t countdown, RV8803_EXT_TD_t clk_freq, bool interupt_enable) {
  // set the countdown timer, and if it should cause the interupt pin of the RV8803 to activate
  if (countdown > 4095)
    return 1;

  uint8_t ext, flags, ctrl, ret;
  ret = readRTCRegisterRV8803C7(RV8803_EXT, &ext);
  if (ret)
    return ret;

  ret = readRTCRegisterRV8803C7(RV8803_FLAG, &flags);
  if (ret)
    return ret;

  ret = readRTCRegisterRV8803C7(RV8803_CTRL, &ctrl);
  if (ret)
    return ret;

  // stop countdown
  ext &= ~RV8803_EXT_TE;
  ret = writeRTCRegisterRV8803C7(RV8803_EXT, ext);
  if (ret)
    return ret;

  // set the timer clock frequency
  ext &= RV8803_EXT_TD_CLR;
  ext |= clk_freq;
  ret = writeRTCRegisterRV8803C7(RV8803_EXT, ext);
  if (ret)
    return ret;

  // clear the timer flag
  flags &= ~RV8803_FLAG_TF;
  ret = writeRTCRegisterRV8803C7(RV8803_FLAG, flags);
  if (ret)
    return ret;

  uint8_t LSByte = countdown & 0xFF;
  ret = writeRTCRegisterRV8803C7(RV8803_TIMER_COUNTER_0, LSByte);
  if (ret)
    return ret;

  uint8_t MSByte = (countdown >> 8) & 0xFF;
  ret = writeRTCRegisterRV8803C7(RV8803_TIMER_COUNTER_1, MSByte);
  if (ret)
    return ret;

  // set the signal interrupt pin on countdown
  if (interupt_enable)
    ctrl |= RV8803_CTRL_TIE;
  else
    ctrl &= ~RV8803_CTRL_TIE;
  ret = writeRTCRegisterRV8803C7(RV8803_CTRL, ctrl);
  if (ret)
    return ret;

  // start countdown again
  ext |= RV8803_EXT_TE;
  ret = writeRTCRegisterRV8803C7(RV8803_EXT, ext);
  return ret;
}
