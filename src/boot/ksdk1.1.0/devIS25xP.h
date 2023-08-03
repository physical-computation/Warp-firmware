/*
	Authored 2020-2022. Phillip Stanley-Marbell, Orestis Kaparounakis.

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

#define kWarpInitialPageNumberIS25xP		0x10
#define kWarpInitialPageOffsetIS25xP		0x00

const uint16_t 	kWarpIS25xPPageOffsetStoragePage 	= 0;
const uint8_t 	kWarpIS25xPPageOffsetStorageOffset 	= 0;
const size_t 	kWarpIS25xPPageOffsetStorageSize	= 3;

void		initIS25xP(int chipSelectIoPinID, uint16_t operatingVoltageMillivolts);

/**
 * @brief Perform a transaction with the Flash device.
 *
 * This function performs a blocking transaction with the Flash device using the
 * provided operation command bytes.
 *
 * @param ops Array pointing to bytes making up the command.
 * @param opCount The number of bytes making up the command (including dummies for receive).
 */
WarpStatus	spiTransactionIS25xP(uint8_t ops[], size_t opCount);

/**
 * @brief Perform a contiguous read of the flash memory starting from an offset
 * address.
 *
 * This function performs a blocking read of the device.
 *
 * @param startAddress Flash memory address to start reading from. Width is actually 24 bits.
 * @param nbyte Number of bytes to read from the flash memory.
 * @param buf Buffer to place the read values in. Must fit `nbyte` bytes.
 */
WarpStatus 	readAllMemoryIS25xP();
WarpStatus 	readMemoryIS25xP(uint16_t startPageNumber, uint8_t startPageOffset, size_t nbyte, void* buf);
WarpStatus 	programPageIS25xP(uint16_t startPageAddress, uint8_t startPageOffset,  size_t nbyte, uint8_t* buf);
WarpStatus 	eraseSectorIS25xP(uint32_t address);
WarpStatus 	erase32kBlockIS25xP(uint32_t address);
WarpStatus 	erase64kBlockIS25xP(uint32_t address);
WarpStatus 	chipEraseIS25xP();
WarpStatus 	programPageNumberAndOffset(uint16_t pageNumber, uint8_t pageOffset);
WarpStatus 	resetIS25xP();
WarpStatus 	writeToIS25xPFromEnd(size_t nbyte, uint8_t* buf);
void 		enableIS25xPWrite();
void 		disableIS25xPWrite();
WarpStatus 	flashStatusIS25xP();
WarpStatus 	waitForWriteCompletion();