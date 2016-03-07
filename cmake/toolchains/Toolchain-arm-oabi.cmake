# defines:
#
# NM
# OBJCOPY
# LD
# CXX_COMPILER
# C_COMPILER
# CMAKE_SYSTEM_NAME
# CMAKE_SYSTEM_VERSION
# GENROMFS
# LINKER_FLAGS
# CMAKE_EXE_LINKER_FLAGS
# CMAKE_FIND_ROOT_PATH
# CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
# CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
# CMAKE_FIND_ROOT_PATH_MODE_INCLUDE

include(CMakeForceCompiler)

if ("$ENV{TS7500_TOOLCHAIN_ROOT}" STREQUAL "")
        message(FATAL_ERROR "TS7500_TOOLCHAIN_ROOT not set")
else()
        set(TS7500_TOOLS_ROOT $ENV{TS7500_TOOLCHAIN_ROOT})
endif()

if ("$ENV{TS7500_ARM_SYSROOT}" STREQUAL "")
        message(FATAL_ERROR "TS7500_ARM_SYSROOT not set")
else()
        set(TS7500_TOOLS_ROOT $ENV{TS7500_ARM_SYSROOT})
endif()

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
find_program(C_COMPILER arm-unknown-linux-gnu-gcc
	PATHS ${TS7500_TOOLS_ROOT}/arm-unknown-linux-gnu/bin
	NO_DEFAULT_PATH
	)

if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find arm-unknown-linux-gnu-gcc compiler")
endif()
cmake_force_c_compiler(${C_COMPILER} GNU)

find_program(CXX_COMPILER arm-unknown-linux-gnu-g++
	PATHS ${TS7500_TOOLS_ROOT}/arm-unknown-linux-gnu/bin
	NO_DEFAULT_PATH
	)

if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find arm-unknown-linux-gnu-g++ compiler")
endif()
cmake_force_cxx_compiler(${CXX_COMPILER} GNU)

# compiler tools
foreach(tool objcopy nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} arm-unknown-linux-gnu-${tool}
		PATHS ${TS7500_TOOLS_ROOT}/arm-unknown-linux-gnu/bin
		NO_DEFAULT_PATH
		)
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find arm-unknwon-linux-gnu-${tool}")
	endif()
endforeach()

# os tools
foreach(tool echo patch grep rm mkdir nm genromfs cp touch make unzip)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${TOOL}")
	endif()
endforeach()

set(C_FLAGS "--sysroot=${TS7500_ARM_SYSROOT}")
set(LINKER_FLAGS "-Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS})
set(CMAKE_C_FLAGS ${C_FLAGS})
set(CMAKE_CXX_LINKER_FLAGS ${C_FLAGS})

# where is the target environment
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
