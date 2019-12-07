# Baseline firmware for the [Warp](https://github.com/physical-computation/Warp-hardware) family of hardware platforms
This is the firmware for the [Warp hardware](https://github.com/physical-computation/Warp-hardware) and its publicly available and unpublished derivatives. This firmware also runs on the Freescale/NXP FRDM KL03 evaluation board which we use for teaching at the University of Cambridge. When running on platforms other than Warp, only the sensors available in the corresponding hardware platform are accessible.

**Prerequisites:** You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake` (installed, e.g., via `apt-get` on Linux or via [MacPorts](https://www.macports.org) on macOS). You will also need an installed copy of the SEGGER [JLink commander](https://www.segger.com/downloads/jlink/), `JlinkExe`, which is available for Linux, macOS, and Windows (here are direct links for downloading it for [macOS](https://www.segger.com/downloads/jlink/JLink_MacOSX.pkg), and [Linux tgz 64-bit](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.tgz)).

## 1.  Compiling the Warp firmware
First, make sure the environment variable `ARMGCC_DIR` is set correctly (you can check whether this is set correctly, e.g., via `echo $ARMGCC_DIR`; if this is unfamiliar, see [here](http://homepages.uc.edu/~thomam/Intro_Unix_Text/Env_Vars.html) or [here](https://www2.cs.duke.edu/csl/docs/csh.html)). If your `arm-none-eabi-gcc` is in `/usr/local/bin/arm-none-eabi-gcc`, then you want to set  `ARMGCC_DIR` to `/usr/local`. If your shell is `tcsh`:

	setenv ARMGCC_DIR <full path to the directory containing bin/arm-none-eabi-gcc>

Alternatively, if your shell is `bash`

	export ARMGCC_DIR=<full path to the directory containing bin/arm-none-eabi-gcc>

(You can check what your shell is, e.g., via `echo $SHELL`.) Second, edit the jlink command file, `tools/scripts/jlink.commands` to include the correct path.

Third, you should be able to build the Warp firmware by

	cd build/ksdk1.1/
	./build.sh

This copies the files from `Warp/src/boot/ksdk1.1.0/` into the KSDK tree, builds, and converts the binary to SREC. See 	`Warp/src/boot/ksdk1.1.0/README.md` for more. _When editing source, edit the files in `Warp/src/boot/ksdk1.1.0/`, not the files in the build location, since the latter are overwritten during each build._

Fourth, you will need two terminal windows. In one shell window, run the firmware downloader:

	JLinkExe -device MKL03Z32XXX4 -if SWD -speed 100000 -CommanderScript ../../tools/scripts/jlink.commands

In the other shell window, launch the JLink RTT client<sup>&nbsp;<a href="#Notes">See note 1 below</a></sup>:

	JLinkRTTClient

## 2. Using the Warp firmware on the Freescale FRDMKL03 Board
The SEGGER firmware allows you to use SEGGER’s JLink software to load your own firmware to the board, even without using their specialized JLink programming cables. You can find the SEGGER firmware at the SEGGER Page for [OpenSDA firmware](https://www.segger.com/products/debug-probes/j-link/models/other-j-links/opensda-sda-v2/).

To build the Warp firmware for the FRDM KL03, you will need to uncomment the `#define WARP_FRDMKL03` define in `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c`. When building for the FRDMKL03 board, you can also disable drivers for sensors that are not on the FRDMKL03 (i.e., disable all sensors except the MMA8451Q). The full set of diffs is:
```diff
diff --git a/src/boot/ksdk1.1.0/CMakeLists.txt b/src/boot/ksdk1.1.0/CMakeLists.txt
index 5cd6996..197e0a5 100755
--- a/src/boot/ksdk1.1.0/CMakeLists.txt
+++ b/src/boot/ksdk1.1.0/CMakeLists.txt
@@ -89,19 +89,19 @@ ADD_EXECUTABLE(Warp
     "${ProjDirPath}/../../../../platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S"
     "${ProjDirPath}/../../src/warp-kl03-ksdk1.1-boot.c"
     "${ProjDirPath}/../../src/warp-kl03-ksdk1.1-powermodes.c"
-    "${ProjDirPath}/../../src/devBMX055.c"
+#    "${ProjDirPath}/../../src/devBMX055.c"
 #    "${ProjDirPath}/../../src/devADXL362.c"
     "${ProjDirPath}/../../src/devMMA8451Q.c"
 #    "${ProjDirPath}/../../src/devLPS25H.c"
-    "${ProjDirPath}/../../src/devHDC1000.c"
-    "${ProjDirPath}/../../src/devMAG3110.c"
+#    "${ProjDirPath}/../../src/devHDC1000.c"
+#    "${ProjDirPath}/../../src/devMAG3110.c"
 #    "${ProjDirPath}/../../src/devSI7021.c"
-    "${ProjDirPath}/../../src/devL3GD20H.c"
-    "${ProjDirPath}/../../src/devBME680.c"
+#    "${ProjDirPath}/../../src/devL3GD20H.c"
+#    "${ProjDirPath}/../../src/devBME680.c"
 #    "${ProjDirPath}/../../src/devTCS34725.c"
 #    "${ProjDirPath}/../../src/devSI4705.c"
-    "${ProjDirPath}/../../src/devCCS811.c"
-    "${ProjDirPath}/../../src/devAMG8834.c"
+#    "${ProjDirPath}/../../src/devCCS811.c"
+#    "${ProjDirPath}/../../src/devAMG8834.c"
 #    "${ProjDirPath}/../../src/devRV8803C7.c"
 #    "${ProjDirPath}/../../src/devPAN1326.c"
 #    "${ProjDirPath}/../../src/devAS7262.c"
diff --git a/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c b/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c
index 87a27e1..42ce458 100755
--- a/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c
+++ b/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c
@@ -55,7 +55,7 @@
 #include "SEGGER_RTT.h"
 #include "warp.h"
 
-//#define WARP_FRDMKL03
+#define WARP_FRDMKL03
```


## 3.  Editing the firmware
The firmware is currently all in `src/boot/ksdk1.1.0/`, in particular, see `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c` and the per-sensor drivers in `src/boot/ksdk1.1.0/dev*.[c,h]`.

The firmware builds on the Kinetis SDK. You can find more documentation on the Kinetis SDK in the document [doc/Kinetis SDK v.1.1 API Reference Manual.pdf](https://github.com/physical-computation/Warp-firmware/blob/master/doc/Kinetis%20SDK%20v.1.1%20API%20Reference%20Manual.pdf).

The firmware is designed for the Warp hardware platform, but will also run on the Freeacale FRDM KL03 development board. In that case, the only driver which is relevant is the one for the MMA8451Q. For more details about the structure of the firmware, see [src/boot/ksdk1.1.0/README.md](src/boot/ksdk1.1.0/README.md).

## 4.  Interacting with the boot menu
When the firmware boots, you will be dropped into a menu with a rich set of commands. The Warp boot menu allows you to conduct most of the experiments you will likely need without modifying the firmware:
````
[ *				W	a	r	p	(rev. b)			* ]
[  				      Cambridge / Physcomplab   				  ]

	Supply=0mV,	Default Target Read Register=0x00
	I2C=200kb/s,	SPI=200kb/s,	UART=1kb/s,	I2C Pull-Up=32768

	SIM->SCGC6=0x20000001		RTC->SR=0x10		RTC->TSR=0x5687132B
	MCG_C1=0x42			MCG_C2=0x00		MCG_S=0x06
	MCG_SC=0x00			MCG_MC=0x00		OSC_CR=0x00
	SMC_PMPROT=0x22			SMC_PMCTRL=0x40		SCB->SCR=0x00
	PMC_REGSC=0x00			SIM_SCGC4=0xF0000030	RTC->TPR=0xEE9

	0s in RTC Handler to-date,	0 Pmgr Errors
Select:
- 'a': set default sensor.
- 'b': set I2C baud rate.
- 'c': set SPI baud rate.
- 'd': set UART baud rate.
- 'e': set default register address.
- 'f': write byte to sensor.
- 'g': set default SSSUPPLY.
- 'h': powerdown command to all sensors.
- 'i': set pull-up enable value.
- 'j': repeat read reg 0x00 on sensor #3.
- 'k': sleep until reset.
- 'l': send repeated byte on I2C.
- 'm': send repeated byte on SPI.
- 'n': enable SSSUPPLY.
- 'o': disable SSSUPPLY.
- 'p': switch to VLPR mode.
- 'r': switch to RUN mode.
- 's': power up all sensors.
- 't': dump processor state.
- 'u': set I2C address.
- 'x': disable SWD and spin for 10 secs.
- 'z': dump all sensors data.
Enter selection> 
````
### Double echo characters
By default on Unix, you will likely see characters you enter shown twice. To avoid this, do the following:
- Make sure you are running `bash` (and not `csh`)
- Execute `stty -echo` at the command line in the terminal window in which you will run the `JLinkRTTClient`.

### Introduction to using the menu
You can probe around the menu to figure out what to do. In brief, you will likely want:

1. Menu item `b` to set the I2C baud rate.

2. Menu item `r` to switch the processor from low-power mode (2MHz) to "run" mode (48MHz).

3. Menu item `g` to set sensor supply voltage.

4. Menu item `n` to turn on the voltage regulators.

5. Menu item `z` to repeatedly read from all the sensors whose drivers are compiled into the build.

*NOTE: In many cases, the menu expects you to type a fixed number of characters (e.g., 0000 or 0009 for zero and nine)<sup>&nbsp;<a href="#Notes">See note 1 below</a></sup>. If using the `JLinkRTTClient`, the menu interface eats your characters as you type them, and you should not hit RETURN after typing in text. On the other hand, if using `telnet` you have to hit return.*

### Example 1: Dump all registers for a single sensor
-	`b` (set the I2C baud rate to `0300` for 300 kb/s).
-	`g` (set sensor supply voltage to `3000` for 3000mV sensor supply voltage).
-	`n` (turn on the sensor supply regulators).
-	`j` (submenu for initiating a fixed number of repeated reads from a sensor):
````
Enter selection> j

    Auto-increment from base address 0x01? ['0' | '1']> 0
    Chunk reads per address (e.g., '1')> 1
    Chatty? ['0' | '1']> 1
    Inter-operation spin delay in milliseconds (e.g., '0000')> 0000
    Repetitions per address (e.g., '0000')> 0000
    Maximum voltage for adaptive supply (e.g., '0000')> 2500
    Reference byte for comparisons (e.g., '3e')> 00
````

### Example 2: Stream data from all sensors
This will perpetually stream data from the 90+ sensor dimensions at a rate of about 90-tuples per second. Use the following command sequence:
-	`b` (set the I2C baud rate to `0300` for 300 kb/s).
-	`r` (enable 48MHz "run" mode for the processor).
-	`g` (set sensor supply voltage to `3000` for 3000mV sensor supply voltage).
-	`n` (turn on the sensor supply regulators).
-	`z` (start to stream data from all sensors that can run at the chosen voltage and baud rate).

## 5.  To update your fork
From your local clone:

	git remote add upstream https://github.com/physical-computation/Warp-firmware.git
	git fetch upstream
	git pull upstream master

----

### If you use Warp in your research, please cite it as:
Phillip Stanley-Marbell and Martin Rinard. “A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation”. ArXiv e-prints (2018). arXiv:1804.09241.
**BibTeX:**
```
@ARTICLE{1804.09241,
author = {Stanley-Marbell, Phillip and Rinard, Martin},
title = {A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation},
journal = {ArXiv e-prints},
archivePrefix = {arXiv},
eprint = {1804.09241},
year = 2018,
}
```
Phillip Stanley-Marbell and Martin Rinard. “Warp: A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation”. IEEE Micro (2019), doi 10.1109/MM.2019.2951004.
**BibTeX:**
```
@ARTICLE{10.1109/MM.2019.2951004,
author = {Stanley-Marbell, Phillip and Rinard, Martin},
title = {Warp: A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation},
doi = {10.1109/MM.2019.2951004},
journal = {IEEE Micro},
year = 2019,
}
```
### Acknowledgements
This research is supported by an Alan Turing Institute award TU/B/000096 under EPSRC grant EP/N510129/1, by Royal Society grant RG170136, and by EPSRC grants EP/P001246/1 and EP/R022534/1.

----
### Notes
<sup>1</sup>&nbsp; On some Unix platforms, the `JLinkRTTClient` has a double echo of characters you type in. You can prevent this by configuring your terminal program to not echo the characters you type. To achieve this on `bash`, use `stty -echo` from the terminal. Alternatively, rather than using the `JLinkRTTClient`, you can use a `telnet` program: `telnet localhost 19021`. This avoids the JLink RTT Client's "double echo" behavior but you will then need a carriage return (&crarr;) for your input to be sent to the board. Also see [Python SEGGER RTT library from Square, Inc.](https://github.com/square/pylink/blob/master/examples/rtt.py) (thanks to [Thomas Garry](https://github.com/tidge27) for the pointer).
