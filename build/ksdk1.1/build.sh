#!/bin/sh
	
	# Uncomment this to set ARMGCC_DIR correctly: export ARMGCC_DIR=<full path to arm-gcc directory>

	mkdir -p work
	mkdir -p work/boards/Warp
	mkdir -p work/demos/Warp/src


	cp -r ../../tools/sdk/ksdk1.1.0/*				work
	cp ../../src/boot/ksdk1.1.0/SEGGER*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c		work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-powermodes.c	work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/warp.h				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/CMakeLists.txt			work/demos/Warp/armgcc/Warp/
	cp ../../src/boot/ksdk1.1.0/startup_MKL03Z4.S			work/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
	cp ../../src/boot/ksdk1.1.0/gpio_pins.c				work/boards/Warp
	cp ../../src/boot/ksdk1.1.0/gpio_pins.h				work/boards/Warp
	cd work/lib/ksdk_platform_lib/armgcc/KL03Z4 && ./clean.sh; ./build_release.sh
	cd ../../../../demos/Warp/armgcc/Warp && ./clean.sh; ./build_release.sh
	echo "\n\nNow, run\n\n\t/Applications/SEGGER/JLink/JLinkExe -device MKL03Z32XXX4 -if SWD -speed 4000 -CommanderScript ../../tools/scripts/jlink.commands\n\n"

