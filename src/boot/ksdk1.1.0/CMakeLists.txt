INCLUDE(CMakeForceCompiler)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)


# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

# DEBUG LINK FILE
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${ProjDirPath}/../../../../platform/linker/MKL03Z4/gcc/MKL03Z32xxx4_flash.ld  -static")

# RELEASE LINK FILE
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/../../../../platform/linker/MKL03Z4/gcc/MKL03Z32xxx4_flash.ld  -static")

# DEBUG ASM FLAGS
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g  -mcpu=cortex-m0plus  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# DEBUG C FLAGS
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g  -mcpu=cortex-m0plus  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99 -fshort-enums")

# DEBUG LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g  --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs  -Xlinker --defsym=__stack_size__=0x300  -Xlinker --defsym=__heap_size__=0x00")

# RELEASE ASM FLAGS
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mcpu=cortex-m0plus  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# RELEASE C FLAGS
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mcpu=cortex-m0plus  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99 -fshort-enums")

# RELEASE LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs  -Xlinker --defsym=__stack_size__=0x300  -Xlinker --defsym=__heap_size__=0x00")

# ASM MACRO
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -DDEBUG")

# C MACRO
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -DDEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -DCPU_MKL03Z32VFK4")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -DFRDM_KL03Z48M")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -DFREEDOM")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DNDEBUG")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DCPU_MKL03Z32VFK4")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DFRDM_KL03Z48M")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DFREEDOM")

# CXX MACRO

# INCLUDE_DIRECTORIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/utilities/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/osa/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/CMSIS/Include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/CMSIS/Include/device)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/startup/MKL03Z4)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/hal/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/drivers/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/system/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../boards/Warp)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/utilities/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/osa/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/CMSIS/Include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/CMSIS/Include/device)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/startup/MKL03Z4)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/hal/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/drivers/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/system/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../boards/Warp)
ENDIF()

# ADD_EXECUTABLE
ADD_EXECUTABLE(Warp 
    "${ProjDirPath}/../../../../platform/utilities/src/fsl_misc_utilities.c"
    "${ProjDirPath}/../../../../platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S"
    "${ProjDirPath}/../../src/warp-kl03-ksdk1.1-boot.c"
    "${ProjDirPath}/../../src/warp-kl03-ksdk1.1-powermodes.c"
    "${ProjDirPath}/../../src/devBMX055.c"
#    "${ProjDirPath}/../../src/devADXL362.c"
    "${ProjDirPath}/../../src/devMMA8451Q.c"
#    "${ProjDirPath}/../../src/devLPS25H.c"
    "${ProjDirPath}/../../src/devHDC1000.c"
    "${ProjDirPath}/../../src/devMAG3110.c"
#    "${ProjDirPath}/../../src/devSI7021.c"
    "${ProjDirPath}/../../src/devL3GD20H.c"
    "${ProjDirPath}/../../src/devBME680.c"
#    "${ProjDirPath}/../../src/devTCS34725.c"
#    "${ProjDirPath}/../../src/devSI4705.c"
    "${ProjDirPath}/../../src/devCCS811.c"
    "${ProjDirPath}/../../src/devAMG8834.c"
#    "${ProjDirPath}/../../src/devRV8803C7.c"
#    "${ProjDirPath}/../../src/devPAN1326.c"
#    "${ProjDirPath}/../../src/devAS7262.c"
#    "${ProjDirPath}/../../src/devAS7263.c"
    "${ProjDirPath}/../../src/SEGGER_RTT.c"
    "${ProjDirPath}/../../src/SEGGER_RTT_printf.c"
    "${ProjDirPath}/../../../../platform/drivers/src/i2c/fsl_i2c_irq.c"
    "${ProjDirPath}/../../../../platform/drivers/src/spi/fsl_spi_irq.c"
    "${ProjDirPath}/../../../../platform/startup/MKL03Z4/system_MKL03Z4.c"
    "${ProjDirPath}/../../../../platform/startup/startup.c"
    "${ProjDirPath}/../../../../platform/startup/startup.h"
    "${ProjDirPath}/../../../../boards/Warp/gpio_pins.c"
    "${ProjDirPath}/../../../../boards/Warp/gpio_pins.h"
)
SET_TARGET_PROPERTIES(Warp PROPERTIES OUTPUT_NAME "Warp.elf")

TARGET_LINK_LIBRARIES(Warp -Wl,--start-group)
# LIBRARIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    TARGET_LINK_LIBRARIES(Warp ${ProjDirPath}/../../../../lib/ksdk_platform_lib/armgcc/KL03Z4/debug/libksdk_platform.a)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
    TARGET_LINK_LIBRARIES(Warp ${ProjDirPath}/../../../../lib/ksdk_platform_lib/armgcc/KL03Z4/release/libksdk_platform.a)
ENDIF()

# SYSTEM LIBRARIES
TARGET_LINK_LIBRARIES(Warp m)
TARGET_LINK_LIBRARIES(Warp c)
TARGET_LINK_LIBRARIES(Warp gcc)
TARGET_LINK_LIBRARIES(Warp nosys)
TARGET_LINK_LIBRARIES(Warp -Wl,--end-group)

# MAP FILE
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG}  -Xlinker -Map=debug/Warp.map")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE}  -Xlinker -Map=release/Warp.map")

# BIN AND HEX
ADD_CUSTOM_COMMAND(TARGET Warp POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/Warp.elf ${EXECUTABLE_OUTPUT_PATH}/Warp.hex)
ADD_CUSTOM_COMMAND(TARGET Warp POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/Warp.elf ${EXECUTABLE_OUTPUT_PATH}/Warp.bin)
ADD_CUSTOM_COMMAND(TARGET Warp POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Osrec ${EXECUTABLE_OUTPUT_PATH}/Warp.elf ${EXECUTABLE_OUTPUT_PATH}/Warp.srec)
