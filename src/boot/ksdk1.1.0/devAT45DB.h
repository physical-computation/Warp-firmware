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
WarpStatus spiTransactionAT45DB(WarpSPIDeviceState volatile *  deviceStatePointer, uint8_t ops[], size_t opCount);
WarpStatus writeDataToBuffer(uint32_t startAddress, size_t nbyte, uint8_t *  buf);
WarpStatus readBufferAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf);
WarpStatus writeDataFromBufferToPage(uint32_t startAddress, size_t nbyte, uint8_t *  buf);
WarpStatus readPageAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf);
WarpStatus MainMemoryPageProgramAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf, int r);
WarpStatus MainMemoryPageReadAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf);
WarpStatus MainMemoryPageEraseAT45DB(uint32_t startAddress);
WarpStatus eraseSectorAT45DB(uint32_t address);
WarpStatus erase32kBlockAT45DB(uint32_t address);
WarpStatus chipEraseAT45DB();
WarpStatus MemoryPageProgramAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf);
bool AT45DB641EIsBusy();
WarpStatus initializeMemoryAT45DB();
uint8_t AT45DB_readStatusRegister();
WarpStatus BufferWriteAT45DB(uint8_t bufferNumber, uint16_t offset, uint16_t length, uint8_t *data);
WarpStatus BufferToMainMemoryAT45DB(uint8_t bufferNumber, uint32_t offset, uint32_t length, uint16_t pageIndex);
WarpStatus MainMemoryPageReadAT45DB1(uint16_t pageAddress, uint16_t length, uint8_t *data);
WarpStatus spiTransactionAT45DB1 (WarpSPIDeviceState volatile * deviceStatePointer, uint8_t * txBuffer, uint8_t * rxBuffer, size_t transactionLength);
WarpStatus MainMemoryPageToBufferTransferAT45DB(uint32_t startAddress, size_t nbyte, uint8_t *  buf);
WarpStatus mainMemoryPageToBufferAT45DB(uint8_t bufferNumber, uint16_t pageAddress);
WarpStatus bufferReadAT45DB(uint8_t bufferNumber, uint16_t bufferAddress, uint8_t* dataBuffer, uint16_t nbyte);
WarpStatus AT45dbxx_WaitBusy(void);
WarpStatus AT45dbxx_Resume(void);
WarpStatus AT45dbxx_PowerDown(void);
WarpStatus programPageIS25xP(uint32_t startAddress, size_t nbyte, void *  buf);
// WarpStatus readmemoryAT45DB(uint32_t startAddress, size_t nbyte, void *  buf);
WarpStatus readmemoryAT45DB(uint16_t startAddress, size_t nbyte, void * buf);
WarpStatus PageProgramAT45DB(uint16_t startAddress, size_t nbyte, uint8_t *  buf);
WarpStatus disablesectorprotection();


