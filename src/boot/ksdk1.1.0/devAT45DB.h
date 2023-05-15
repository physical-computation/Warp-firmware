/*
	Authored 2021. Phillip Stanley-Marbell.

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

WarpStatus	initAT45DB(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts);
WarpStatus 	spiTransactionAT45DB(WarpSPIDeviceState volatile *  deviceStatePointer, uint8_t ops[], size_t opCount);
void 		enableAT45DBWrite();
WarpStatus 	saveToAT45DBFromEnd(size_t nbyte, uint8_t* buf);
void 	setAT45DBStartOffset(uint16_t pageNumber, uint8_t pageOffset);
WarpStatus 	readMemoryAT45DB(uint16_t pageNumber, size_t nbyte, void *  buf);
WarpStatus 	pageProgramAT45DB(uint16_t startAddress, size_t nbyte, uint8_t *  buf);
WarpStatus 	eraseSectorAT45DB(uint32_t address);
WarpStatus 	erase32kBlockAT45DB(uint32_t address);
WarpStatus 	chipEraseAT45DB();


