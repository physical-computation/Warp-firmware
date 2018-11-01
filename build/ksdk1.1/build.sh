#!/bin/sh
	
	# Uncomment this to set ARMGCC_DIR correctly: export ARMGCC_DIR=<full path to arm-gcc directory>

	mkdir -p work
	mkdir -p work/boards/Warp
	mkdir -p work/demos/Warp/src
	mkdir -p work/demos/Warp/src/btstack

	cp -r ../../tools/sdk/ksdk1.1.0/*				work
	cp ../../src/boot/ksdk1.1.0/SEGGER*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c		work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-powermodes.c	work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/warp.h				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devBMX055.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devADXL362.*			work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devMMA8451Q.*			work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devLPS25H.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devHDC1000.*			work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devMAG3110.*			work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devSI7021.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devL3GD20H.*			work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devBME680.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devTCS34725.*			work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devSI4705.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devCCS811.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devAMG8834.*			work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devAS7262.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devAS7263.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devAS726x.*				work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/devPAN1326.*			work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/CMakeLists.txt			work/demos/Warp/armgcc/Warp/
	cp ../../src/boot/ksdk1.1.0/startup_MKL03Z4.S		work/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
	cp ../../src/boot/ksdk1.1.0/gpio_pins.c				work/boards/Warp
	cp ../../src/boot/ksdk1.1.0/gpio_pins.h				work/boards/Warp

	# Files needed for Bluetooth
	## user files
	cp ../../src/boot/ksdk1.1.0/devCC2564C.*						work/demos/Warp/src/btstack/
	cp -r ../../src/boot/ksdk1.1.0/btstack/*     					work/demos/Warp/src/
	cp ../../src/boot/ksdk1.1.0/btstack/btstack_config.h     		work/demos/Warp/src/btstack/
	cp ../../src/boot/ksdk1.1.0/btstack/btstack_config.h     		work/demos/Warp/src/ble #FIXME:
	cp ../../src/boot/ksdk1.1.0/btstack/btstack_config.h     		work/demos/Warp/src/classic #FIXME:
	cp ../../src/boot/ksdk1.1.0/btstack/led_counter.*     			work/demos/Warp/src/btstack/
	cp ../../src/boot/ksdk1.1.0/btstack/hal_led.*     				work/demos/Warp/src/btstack/
	cp ../../src/boot/ksdk1.1.0/btstack/hal_tick.c     				work/demos/Warp/src/btstack/
	cp ../../src/boot/ksdk1.1.0/btstack/hal_cpu.*     				work/demos/Warp/src/btstack/
	cp ../../src/boot/ksdk1.1.0/btstack/hal_uart_dma.*     			work/demos/Warp/src/btstack/
	cp ../../src/boot/ksdk1.1.0/btstack/btstack_main.h     			work/demos/Warp/src/btstack/

	## chipset
	cp ../../src/btstack/chipset/cc256x/btstack_chipset_cc256x.h 	work/demos/Warp/src/btstack/
	cp ../../src/btstack/chipset/cc256x/btstack_chipset_cc256x.c     work/demos/Warp/src/btstack/
	cp ../../src/btstack/chipset/cc256x/ant_cmd.c		     		work/demos/Warp/src/btstack/
	cp ../../src/btstack/chipset/cc256x/ant_cmd.h                    work/demos/Warp/src/btstack/
	cp ../../src/btstack/chipset/cc256x/cc2564cInit.c                work/demos/Warp/src/btstack/
	
	## bt stack
	cp -r ../../src/btstack/platform/embedded/btstack_run_loop_embedded.*	work/demos/Warp/src/btstack/
	cp -r ../../src/btstack/platform/embedded/btstack_uart_block_embedded.*	work/demos/Warp/src/btstack/
	cp -r ../../src/btstack/platform/embedded/hal_cpu.h			work/demos/Warp/src/btstack/
	cp -r ../../src/btstack/platform/embedded/hal_led.h			work/demos/Warp/src/btstack/
	cp -r ../../src/btstack/platform/embedded/hal_tick.h			work/demos/Warp/src/btstack/
	cp -r ../../src/btstack/platform/embedded/hal_uart_dma.h		work/demos/Warp/src/btstack/
	

	cp -r ../../src/btstack/src/*                    				work/demos/Warp/src/btstack/

	# cp -r ../../../btstack/src/*                    				work/demos/Warp/src/
	#a bit of a hack to get it to compile properly FIXME:
	# cp -r ../../../btstack/src/*.c                    				work/demos/Warp/src/classic
	# cp -r ../../../btstack/src/*.h                    				work/demos/Warp/src/classic
	# cp -r ../../../btstack/src/*.c                    				work/demos/Warp/src/ble
	# cp -r ../../../btstack/src/*.h                    				work/demos/Warp/src/ble

	cd work/lib/ksdk_platform_lib/armgcc/KL03Z4 && ./clean.sh; ./build_release.sh
	cd ../../../../demos/Warp/armgcc/Warp && ./clean.sh; ./build_release.sh
	echo "\n\nNow, run\n\n\t/Applications/SEGGER/JLink/JLinkExe -device MKL03Z32XXX4 -if SWD -speed 4000 -CommanderScript ../../tools/scripts/jlink.commands\n\n"

