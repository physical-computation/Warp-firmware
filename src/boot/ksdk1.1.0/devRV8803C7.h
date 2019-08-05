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

#ifndef WARP_BUILD_ENABLE_DEVRV8803C7
#define WARP_BUILD_ENABLE_DEVRV8803C7
#endif

typedef enum {TD_4kHZ=0, TD_64HZ=1, TD_1HZ=2, TD_60S=3} WarpRV8803ExtTD_t;
typedef enum {FD_32kHZ=0, FD_1kHZ=4, FD_1HZ=8} WarpRV8803ExtFD_t;

void initRV8803C7(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer);

WarpStatus readRTCRegisterRV8803C7(uint8_t deviceRegister, uint8_t *receiveData);
WarpStatus readRTCRegistersRV8803C7(uint8_t deviceRegister, uint8_t nRegs, uint8_t receiveData[]);

WarpStatus writeRTCRegisterRV8803C7(uint8_t deviceStartRegister, uint8_t payload);
WarpStatus writeRTCRegistersRV8803C7(uint8_t deviceStartRegister, uint8_t nRegs, uint8_t payload[]);

WarpStatus setRTCTimeRV8803C7(rtc_datetime_t *tm);
WarpStatus setRTCCountdownRV8803C7(uint16_t countdown, WarpRV8803ExtTD_t clk_freq, bool interupt_enable);

/*
 *	TODO: Impalement other functions
 *	handle_irq
 *	gettime
 *	time_update_irq_enable
 *	set_countdown
 *	get_countdown
 *	countdown_irq_enable
 *	getalarm
 *	setalarm
 *	alarm_irq_enable
 */
