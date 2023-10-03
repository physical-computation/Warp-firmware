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

typedef enum
{
	kWarpRV8803ExtTD_4kHZ	= 0,
	kWarpRV8803ExtTD_64HZ	= 1,
	kWarpRV8803ExtTD_1HZ	= 2,
	kWarpRV8803ExtTD_60S	= 3
} WarpRV8803ExtTD;

typedef enum
{
	kWarpRV8803ExtFD_32kHZ	= 0,
	kWarpRV8803ExtFD_1kHZ	= 4,
	kWarpRV8803ExtFD_1HZ	= 8
} WarpRV8803ExtFD;

typedef enum
{
	kWarpRV8803RegSec					= 0x00,
	kWarpRV8803RegMin					= 0x01,
	kWarpRV8803RegHour					= 0x02,
	kWarpRV8803RegWeekday				= 0x03,
	kWarpRV8803RegDate					= 0x04,
	kWarpRV8803RegMonth					= 0x05,
	kWarpRV8803RegYear					= 0x06,
	kWarpRV8803RegRAM					= 0x07,
	kWarpRV8803RegMinAlarm				= 0x08,
	kWarpRV8803RegHourAlarm				= 0x09,
	kWarpRV8803RegWeekdayOrDateAlarm	= 0x0A,
	kWarpRV8803RegTimerCounter0			= 0x0B,
	kWarpRV8803RegTimerCounter1			= 0x0C,
	kWarpRV8803RegExt					= 0x0D,
	kWarpRV8803RegFlag					= 0x0E,
	kWarpRV8803RegCtrl					= 0x0F,
} WarpRV8803Reg;

#define BIT(n) (uint8_t)1U << n

typedef enum
{
	kWarpRV8803ExtClrTD			= 0xFC,
	kWarpRV8803ExtClrFD			= 0xF3,
} WarpRV8803ExtClr;

typedef enum
{
	kWarpRV8803ExtTE			= BIT(4),
	kWarpRV8803ExtUSEL			= BIT(5),
	kWarpRV8803ExtWADA			= BIT(6),
	kWarpRV8803ExtTEST			= BIT(7),
} WarpRV8803ExtFlag;

typedef enum
{
	kWarpRV8803FlagV1F			= BIT(0),
	kWarpRV8803FlagV2F			= BIT(1),
	kWarpRV8803FlagEVF			= BIT(2),
	kWarpRV8803FlagAF			= BIT(3),
	kWarpRV8803FlagTF			= BIT(4),
	kWarpRV8803FlagUF			= BIT(5),
} WarpRV8803FlagFlag;

typedef enum
{
	kWarpRV8803CtrlRESET		= BIT(0),
	kWarpRV8803CtrlEIE			= BIT(2),
	kWarpRV8803CtrlAIE			= BIT(3),
	kWarpRV8803CtrlTIE			= BIT(4),
	kWarpRV8803CtrlUIE			= BIT(5),
} WarpRV8803CtrlFlag;

void		initRV8803C7(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readRTCRegisterRV8803C7(uint8_t deviceRegister, uint8_t *receiveData);
WarpStatus	readRTCRegistersRV8803C7(uint8_t deviceRegister, uint8_t nRegs, uint8_t receiveData[]);
WarpStatus	writeRTCRegisterRV8803C7(uint8_t deviceStartRegister, uint8_t payload);
WarpStatus	writeRTCRegistersRV8803C7(uint8_t deviceStartRegister, uint8_t nRegs, uint8_t payload[]);
WarpStatus	setRTCTimeRV8803C7(rtc_datetime_t *  tm);
WarpStatus	setRTCCountdownRV8803C7(uint16_t countdown, WarpRV8803ExtTD clk_freq, bool interupt_enable);

uint8_t appendSensorDataRV8803C7(uint8_t* buf);

const uint8_t bytesPerMeasurementRV8803C7				= 3;
const uint8_t bytesPerReadingRV8803C7					= 1;
const uint8_t numberOfReadingsPerMeasurementRV8803C7 	= 3;
/*
 *	TODO: Implement other functions:
 *
 *		handle_irq
 *		gettime
 *		time_update_irq_enable
 *		set_countdown
 *		get_countdown
 *		countdown_irq_enable
 *		getalarm
 *		setalarm
 *		larm_irq_enable
 */
