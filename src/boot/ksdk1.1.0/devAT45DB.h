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

#define kWarpSizeAT45DBBufferSize 		256
#define kWarpInitialPageNumberAT45DB	1
#define kWarpInitialPageOffsetAT45DB	0
#define kWarpInitialBufferOffsetAT45DB	0

const uint16_t	kWarpAT45DBPageOffsetStoragePage	= 0;
const size_t	kWarpAT45DBPageOffsetStorageSize	= 3;

typedef enum
{
	bufferNumber1AT45DB,
	bufferNumber2AT45DB,
} BufferNumberAT45DB;

void		enableAT45DBWrite();
WarpStatus	initAT45DB(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts);
WarpStatus	spiTransactionAT45DB(WarpSPIDeviceState volatile* deviceStatePointer, uint8_t ops[], size_t opCount);
WarpStatus	saveToAT45DBFromEnd(size_t nbyte, uint8_t* buf);
WarpStatus	setAT45DBStartPosition(uint16_t pageNumber, uint8_t pageOffset);
WarpStatus	readMemoryAT45DB(uint16_t pageNumber, size_t nbyte, void* buf);
WarpStatus	pageProgramAT45DB(uint16_t startAddress, size_t nbyte, uint8_t* buf);
WarpStatus	writeToAT45DBFromEndBuffered(size_t nbyte, uint8_t* buf);

WarpStatus	resetAT45DB();
WarpStatus	waitForDeviceReady();
WarpStatus	initiateChipEraseAndWaitAT45DB();
WarpStatus	initiateChipEraseAT45DB();

WarpStatus	configurePageSize();
WarpStatus	savePagePositionAT45DB();
WarpStatus	writeBufferAndSavePagePositionAT45DB();

WarpStatus	writeToBufferAT45DB(BufferNumberAT45DB buffer, uint8_t address, size_t nbyte, uint8_t *  buf);
WarpStatus	bufferToMainMemoryWriteAT45DB(BufferNumberAT45DB buffer, uint16_t pageNumber);
WarpStatus	bufferToMainMemoryWritePageAT45DB(BufferNumberAT45DB buffer);
WarpStatus	loadMainMemoryPageToBuffer(BufferNumberAT45DB buffer, uint16_t address);
