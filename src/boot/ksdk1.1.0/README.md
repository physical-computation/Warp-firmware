## Overview
The Warp firmware is intended to be a demonstration and meassurement environment for testing the Warp hardware. 
It provides facilities that we use to perform tests on the hardware such as activating the different programmable voltage regulators,
activating the programmable I2C pull-ups to different values, changing the I2C and SPI bit rate, changing the Cortex-M0 clock frequency,
and so on. It is a tool for experimentation. You can use it as a baseline for building real applications by modifying it to remove the
menu-driven functionality and linking in only the sensor drivers you need.

The core of the firmware is in `warp-kl03-ksdk1.1-boot.c`. The drivers for the individual sensors are in `devXXX.c` for sensor `XXX`. For example,
`devADXL362.c` for the ADXL362 3-axis accelerometer. The section below briefly describes all the source files in this directory. 


## Source File Descriptions

##### CMakeLists.txt
This is the CMake configuration file. Edit this to change the default size of the stack and heap.


##### SEGGER_RTT.*
This is the implementation of the SEGGER Real-Time Terminal interface. Do not modify.

##### SEGGER_RTT_Conf.h
Configuration file for SEGGER Real-Time Terminal interface. You can increase the size of `BUFFER_SIZE_UP` to reduce text in the menu being trimmed.

##### SEGGER_RTT_printf.c
Implementation of the SEGGER Real-Time Terminal interface formatted I/O routines. Do not modify.

##### devADXL362.*
Driver for Analog devices ADXL362.

##### devAMG8834.*
Driver for AMG8834.

##### devAS7262.*
Driver for AS7262.

##### devAS7263.*
Driver for AS7263.

##### devAS726x.h
Header file with definitions used by both `devAS7262.*` and `devAS7263.*`.

##### devBME680.*
Driver for BME680.

##### devBMX055.*
Driver for BMX055.

##### devCCS811.*
Driver for CCS811.

##### devHDC1000.8
Driver forHDC1000 .

##### devIS25WP128.*
Driver for IS25WP128.

##### devISL23415.*
Driver for ISL23415.

##### devL3GD20H.*
Driver for L3GD20H.

##### devLPS25H.*
Driver for LPS25H.

##### devMAG3110.*
Driver for MAG3110.

##### devMMA8451Q.*
Driver for MMA8451Q.

##### devPAN1326.*
Driver for PAN1326.

##### devSI4705.*
Driver for SI4705.

##### devSI7021.*
Driver for SI7021.

##### devTCS34725.*
Driver for TCS34725.

##### gpio_pins.c
Definition of I/O pin mappings.

##### gpio_pins.h
Definition of I/O pin mappings.

##### startup_MKL03Z4.S
Initialization assembler.

##### warp-kl03-ksdk1.1-boot.c
The core of the implementation. This puts together the processor initialization with a menu interface that triggers the individual sensor drivers.
You can modify `warp-kl03-ksdk1.1-boot.c` to achieve a custom firmware implementation using the following steps:

1.  Remove the definitions for all the `WarpI2CDeviceState` and `WarpSPIDeviceState` structures for the sensors you are not using.

2.  Remove the function `repeatRegisterReadForDeviceAndAddress()` since that references all the sensors in the full Warp platform.

3.  Remove `activateAllLowPowerSensorModes()` and `powerupAllSensors()` since those assume they have access to all the sensors on the Warp platform.

4.  Modify `main()` to replace the menu (see the `while (1)` [loop](https://github.com/physical-computation/Warp-firmware/blob/ea3fac66e0cd85546b71134538f8d8f6ce1741f3/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c#L1107) with the specific operations you want the hardware to perform after initialization.


##### warp-kl03-ksdk1.1-powermodes.c
Implements functionality related to enabling the different low-power modes of the KL03.

##### warp.h
Constant and data structure definitions.
