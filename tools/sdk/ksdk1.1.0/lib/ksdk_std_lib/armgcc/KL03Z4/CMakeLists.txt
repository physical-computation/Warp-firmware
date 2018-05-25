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

# DEBUG ASM FLAGS
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -g  -mcpu=cortex-m0plus  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99 ")

# DEBUG C FLAGS
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g  -mcpu=cortex-m0plus  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# RELEASE ASM FLAGS
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE}  -mcpu=cortex-m0plus  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99 ")

# RELEASE C FLAGS
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mcpu=cortex-m0plus  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

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
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../utilities/stdlib/include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../utilities/stdlib/stdio)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/utilities/src)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../utilities/stdlib/include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../utilities/stdlib/stdio)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../platform/utilities/src)
ENDIF()

# ADD_LIBRARY
ADD_LIBRARY(KsdkStdLib STATIC
    "${ProjDirPath}/../../../../utilities/stdlib/include/strings.h"
    "${ProjDirPath}/../../../../utilities/stdlib/include/std_prv.h"
    "${ProjDirPath}/../../../../utilities/stdlib/include/stdio.h"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/buf_prv.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/buf_prv.h"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/err.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/fclose.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/fflush.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/fopen.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/fread.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/fwrite.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/get.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/pos.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/print.c"
    "${ProjDirPath}/../../../../platform/utilities/src/print_scan.c"
    "${ProjDirPath}/../../../../platform/utilities/src/print_scan.h"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/put.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/scan.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/stdio.c"
    "${ProjDirPath}/../../../../utilities/stdlib/stdio/std_prv.c"
    "${ProjDirPath}/../../../../utilities/stdlib/strings/strings.c"
)

SET_TARGET_PROPERTIES(KsdkStdLib PROPERTIES OUTPUT_NAME "libksdk_std.a")
