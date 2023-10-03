## Overview
The Warp firmware is intended to be a demonstration and meassurement environment for testing the Warp hardware.

It provides facilities that we use to perform tests on the hardware such as activating the different programmable voltage regulator output voltages (16 different supply voltage levels),
activating the programmable I2C pull-ups to different values (65536 different settings), changing the I2C and SPI bit rate, changing the Cortex-M0 clock frequency,
and so on. Having a menu interface allows us to perform various experiments without having to re-compile and load firmware to the system for each experiment.

The Warp firmware is a tool for experimentation. You can also use it as a baseline for building real applications by modifying it to remove the menu-driven functionality and linking in only the sensor drivers you need.

The core of the firmware is in `warp-kl03-ksdk1.1-boot.c`. The drivers for the individual sensors are in `devXXX.c` for sensor `XXX`. For example,
`devADXL362.c` for the ADXL362 3-axis accelerometer. The section below briefly describes all the source files in this directory.


## Source File Descriptions

##### `CMakeLists.txt`
This is the CMake configuration file. Edit this to change the default size of the stack and heap.


##### `SEGGER_RTT.*`
This is the implementation of the SEGGER Real-Time Terminal interface. Do not modify.

##### `SEGGER_RTT_Conf.h`
Configuration file for SEGGER Real-Time Terminal interface. You can increase the size of `BUFFER_SIZE_UP` to reduce text in the menu being trimmed.

##### `SEGGER_RTT_printf.c`
Implementation of the SEGGER Real-Time Terminal interface formatted I/O routines. Do not modify.

##### `devADXL362.*`
Driver for Analog devices ADXL362.

##### `devAMG8834.*`
Driver for AMG8834.

##### `devAS7262.*`
Driver for AS7262.

##### `devAS7263.*`
Driver for AS7263.

##### `devAS726x.h`
Header file with definitions used by both `devAS7262.*` and `devAS7263.*`.

##### `devBME680.*`
Driver for BME680.

##### `devBMX055.*`
Driver for BMX055.

##### `devCCS811.*`
Driver for CCS811.

##### `devHDC1000.*`
Driver forHDC1000 .

##### `devIS25WP128.*`
Driver for IS25WP128.

##### `devISL23415.*`
Driver for ISL23415.

##### `devL3GD20H.*`
Driver for L3GD20H.

##### `devLPS25H.*`
Driver for LPS25H.

##### `devMAG3110.*`
Driver for MAG3110.

##### `devMMA8451Q.*`
Driver for MMA8451Q.

##### `devPAN1326.*`
Driver for PAN1326.

##### `devSI4705.*`
Driver for SI4705.

##### `devSI7021.*`
Driver for SI7021.

##### `devTCS34725.*`
Driver for TCS34725.

##### `gpio_pins.c`
Definition of I/O pin configurations using the KSDK `gpio_output_pin_user_config_t` structure.

##### `gpio_pins.h`
Definition of I/O pin mappings and aliases for different I/O pins to symbolic names relevant to the Warp hardware design, via `GPIO_MAKE_PIN()`.

##### `startup_MKL03Z4.S`
Initialization assembler.

##### `warp-kl03-ksdk1.1-boot.c`
The core of the implementation. This puts together the processor initialization with a menu interface that triggers the individual sensor drivers based on commands entered at the menu.
You can modify `warp-kl03-ksdk1.1-boot.c` to achieve a custom firmware implementation using the following steps:

1.  Remove the definitions for all the `WarpI2CDeviceState` and `WarpSPIDeviceState` structures for the sensors you are not using.

2.  Remove the function `repeatRegisterReadForDeviceAndAddress()` since that references all the sensors in the full Warp platform.

3.  Remove `activateAllLowPowerSensorModes()` and `powerupAllSensors()` since those assume they have access to all the sensors on the Warp platform.

4.  Modify `main()` to replace the menu (see the `while (1)` [loop](https://github.com/physical-computation/Warp-firmware/blob/ea3fac66e0cd85546b71134538f8d8f6ce1741f3/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c#L1107)) with the specific operations you want the hardware to perform after initialization.

You can inspect the baseline firmware to see what functions are called when you enter commands at the menu. You can then use the underlying functionality that is already implemented to implement your own custom tasks.


##### `warp-kl03-ksdk1.1-powermodes.c`
Implements functionality related to enabling the different low-power modes of the KL03.

##### `warp.h`
Constant and data structure definitions.

## Flash protocol
_This is currently valid only for the Warp variant, since this was written only for the AT45DB_

This section describes the protocol that is used to store measurements into flash. Let us first discuss some terminology.

The Warp hardware comes with a selection of _sensors_ which can be activated by setting the appropriate flag in `config.h`. An example of a sensor is the `MAG3110`. Each sensor produces a number of _readings_. Each such reading consists of a signed integer; the maximum size of a reading will depend on the sensor.

A collection of such readings from the activated sensors is a _measurement_. A measurement is an ordered sequence of readings from all the activated sensors. The `writeAllSensorsToFlash` function in `boot.c` generates such a measurement, and writes them to flash.

When printing to `stdout`, the `printSensorData<sensor>` function of the sensor (found in `dev<sensor>.h`) is called. Here `<sensor>` is `MAG3110` for example. This function simply sequentially prints out readings as characters. Since we don't need to do any thing further, this process is fairly straightforward.

In order to write to flash, the `writeAllSensorsToFlash` function creates an array of bytes that corresponds to the measurement. For each activated sensor `<sensor>`, its `appendSensorData<sensor>` function is called to append its readings to a buffer.

The purpose of writing to flash is to eventually read back and print out the measurements in a form that is similar to if we had printed to `stdout` in the first place. If we kept the configuration of active sensors the same, we know exactly how many bytes correspond to a measurement, and the byte configuration of readings from any active sensor, allowing us to sequentially decode measurements easily.

However, for ease of use, we have made it is possible to write to flash with different configurations of the sensors, across multiple calls to `writeAllSensorsToFlash`. This means that, when reading from the flash, we need to know which sensors were active when a particular measurement was written. This is done by first creating a 16b bitfield in front of each measurement which encodes the active sensor configuration.

We can see that this is done in `writeAllSensorsToFlash`, as soon as an active sensor is configured. For example, the following is an excerpt from `boot.c`:
```C
#if (WARP_BUILD_ENABLE_DEVMAG3110)
	numberOfConfigErrors += configureSensorMAG3110(
		0x00, /*	Payload: DR 000, OS 00, 80Hz, ADC 1280, Full 16bit, standby mode
							 to set up register*/
		0xA0, /*	Payload: AUTO_MRST_EN enable, RAW value without offset */
		0x10);

	sensorBitField = sensorBitField | 0b1000000;
#endif
```
Note `sensorBitField = sensorBitField | 0b100000;`, which activates the bit that corresponds to the `MAG3110` sensor.

When reading, this bitfield is first decoded to know exactly which sensors were active, and therefore how many bytes make up this measurement. Once those bytes have been read and decoded, the next measurement is decoded, read and printed out in the same way.

While this protocol makes writing and reading to flash more useful, it also means that any one adding new sensors need to follow the following guidelines to make sure that the protocol is not violated.

### The structure of the sensor configuration bitfield.
In the current version,  13 out of the 16 bits are used. They correspond to the following sensors being active.

* `0b0000000000000001`: 	the measurement number,
* `0b0000000000000010`: 	the `RTC->TSR` time stamp,
* `0b0000000000000100`: 	the `RTC->TPR` time stamp,
* `0b0000000000001000`:	the `ADXL362` sensor,
* `0b0000000000010000`:	the `AMG8834` sensor,
* `0b0000000000100000`:	the `MMA8451Q` sensor,
* `0b0000000001000000`:	the `MAG3110` sensor,
* `0b0000000010000000`:	the `L3GD20H` sensor,
* `0b0000000100000000`:	the `BME680` sensor,
* `0b0000001000000000`:	the `BMX055` sensor,
* `0b0000010000000000`:	the `CCS811` sensor,
* `0b0000100000000000`:	the `HDC1000` sensor,
* `0b1000000000000000`:	the number of config errors.

*As an important note, this bitfield represents the order in which the readings from the sensors are used to create a measurement.* The first element represents the least significant bit. For example, if the bitfield is `0b00000000100101111`, then the following readings are expected:

1) 4B of the measurement number,
2) 4B of the `RTC->TSR` time stamp,
3) 4B of the `RTC->TPR` time stamp,
4) 8B (4 readings of 2B) of `ADXL362` sensor readings,
5) 6B (3 readings of 2B) of `MMA8451Q` sensor readings, and
6) 12B (3 readings of 4B)of `BME680` sensor readings.

### Writing a new sensor
When writing a new sensor, the following guidelines should be followed in addition to other common sense steps:

1) Write both a `printSensorData<sensor>` and `appendSensorData<sensor>` function. The former should print out the readings to `stdout`, and the latter should append the readings to a buffer. The function signatures should be as follows; look at the existing sensors for examples of what should be inside them:
	```C
	void printSensorData<sensor>(bool hexModeFlag);
	uint8_t appendSensorData<sensor>(uint8_t* buf);
	```

	The output of `appendSensorData<sensor>` is the number of bytes that were appended by the function.

2) Inside `dev<sensor>.h`, define the following 3 constant variables, with their appropriate values. As a sanity check, `numberOfReadingsPerMeasurement<sensor> * bytesPerReading<sensor> = bytesPerMeasurement<sensor>`.
	```C
	const uint8_t bytesPerMeasurement<sensor> = <appropriate value>;
	const uint8_t bytesPerReading<sensor> = <appropriate value>;
	const uint8_t numberOfReadingsPerMeasurement<sensor> = <appropriate value>;
	```

3) Decide on which free bit of the bitfield can be assigned to this sensor. Then, in `printAllSensors`, add the following line right after its configuration:
	```C
	sensorBitField = sensorBitField | 0b<bitfield value>;
	```

4) Following the pattern of the `writeAllSensorsToFlash` function, use the `appnedSensorData<sensor>` functions to append the readings to the buffer, respectively. *Note that you must do this in the order that is specified by the bitfield.* Otherwise, the data won't be stored in the correct order, and will be impossible to decode when reading from flash.

	When writing the `appendSensorData<sensor>` function, you must use the following template:
	```C
	bytesWrittenIndex += appendSensorData<sensor>(flashWriteBuf + bytesWrittenIndex);
	```
	This is to ensure that the buffer is appended to correctly.



5) Add a decoding case to the `writeAllSensorsToFlash` function in `boot.c`, in the correct order. For example, if the new sensor is designated the 13th bit, then the following should be added AFTER the 12th case:
	```C
	if (sensorBitField & 0b1000000000000) {
		numberOfSensorsFound++;
		if (numberOfSensorsFound - 1 == sensorIndex) {
		*sizePerReading = bytesPerReading<sensor>;
		*numberOfReadings = numberOfReadingsPerMeasurement<sensor>;
		return;
		}
	}
	```
	Note that `bytesPerReading<sensor>` and `numberOfReadingsPerMeasurement<sensor>` are the constants defined in Step 2.