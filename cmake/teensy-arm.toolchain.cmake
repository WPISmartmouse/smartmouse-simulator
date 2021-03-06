# Copyright (c) 2015, Pierre-Andre Saulais <pasaulais@free.fr>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set(TRIPLE "arm-none-eabi")

find_program(ARDUINO_EXECUTABLE NAMES "arduino" HINTS /opt DOC "Path to the arduinoYou  executable" NO_DEFAULT_PATH)

file(GLOB ARDUINO_ROOT /opt/arduino-1.*)
if (NOT ARDUINO_ROOT)
  message(FATAL_ERROR "arduino-1.* folder not found anywhere in /opt")
endif()

set(ARDUINO_LIB_ROOT "${ARDUINO_ROOT}/hardware/teensy/avr/libraries" CACHE PATH "arduino libraries directory")

if (NOT EXISTS ${ARDUINO_LIB_ROOT})
  message(FATAL_ERROR "${ARDUINO_LIB_ROOT} does not exist")
endif()

set(TEENSY_CORES_ROOT "${ARDUINO_ROOT}/hardware/teensy/avr/cores" CACHE PATH "Teensy cores")
set(ARDUINO_LIB_ROOT "${ARDUINO_ROOT}/hardware/teensy/avr/libraries" CACHE PATH "arduino libraries directory")
set(TEENSY_ROOT "${TEENSY_CORES_ROOT}/teensy3")
set(TOOLCHAIN_ROOT "${ARDUINO_ROOT}/hardware/tools/arm")
set(ARDUINO_VERSION "108" CACHE STRING "Version of the Arduino SDK")
set(TEENSYDUINO_VERSION "139" CACHE STRING "Version of the Teensyduino SDK")
set(TEENSY_MODEL "MK66FX1M0")
string(TOLOWER ${TEENSY_MODEL} MCU)

set(TEENSY_FREQUENCY "180" CACHE STRING "Frequency of the Teensy MCU (Mhz)")
set_property(CACHE TEENSY_FREQUENCY PROPERTY STRINGS 180 96 72 48 24 16 8 4 2)

set(TEENSY_USB_MODE "SERIAL" CACHE STRING "What kind of USB device the Teensy should emulate")
set_property(CACHE TEENSY_USB_MODE PROPERTY STRINGS SERIAL HID SERIAL_HID MIDI RAWHID FLIGHTSIM)

if(WIN32)
    set(TOOL_OS_SUFFIX .exe)
else(WIN32)
    set(TOOL_OS_SUFFIX )
endif(WIN32)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_COMPILER "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-gcc${TOOL_OS_SUFFIX}" CACHE PATH "gcc" FORCE)
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-g++${TOOL_OS_SUFFIX}" CACHE PATH "g++" FORCE)
set(CMAKE_AR "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-gcc-ar${TOOL_OS_SUFFIX}" CACHE PATH "archive" FORCE)
set(CMAKE_LINKER "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-ld${TOOL_OS_SUFFIX}" CACHE PATH "linker" FORCE)
set(CMAKE_NM "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-nm${TOOL_OS_SUFFIX}" CACHE PATH "nm" FORCE)
set(CMAKE_OBJCOPY "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-objcopy${TOOL_OS_SUFFIX}" CACHE PATH "objcopy" FORCE)
set(CMAKE_OBJDUMP "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-objdump${TOOL_OS_SUFFIX}" CACHE PATH "objdump" FORCE)
set(CMAKE_STRIP "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-strip${TOOL_OS_SUFFIX}" CACHE PATH "strip" FORCE)

include_directories("${TEENSY_ROOT}")

set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb")
set(BASE_FLAGS "-Os -nostdlib -ffunction-sections -fdata-sections ${TARGET_FLAGS}")

set(CMAKE_C_FLAGS "${BASE_FLAGS} -DTIME_T=1421620748" CACHE STRING "c flags") # XXX Generate TIME_T dynamically.
set(CMAKE_CXX_FLAGS "${BASE_FLAGS} -fno-exceptions -fno-rtti -felide-constructors -std=gnu++0x" CACHE STRING "c++ flags")

set(LINKER_DIRS "${TOOLCHAIN_ROOT}/")
set(LINKER_FLAGS "-Os -Wl,--gc-sections ${TARGET_FLAGS} -DUSB_SERIAL -T${TEENSY_ROOT}/${MCU}.ld -L${LINKER_DIRS}" )
set(LINKER_LIBS "-larm_cortexM4l_math -lm -lstdc++ -lc -lsupc++" )
set(CMAKE_SHARED_LINKER_FLAGS "${LINKER_FLAGS}" CACHE STRING "linker flags" FORCE)
set(CMAKE_MODULE_LINKER_FLAGS "${LINKER_FLAGS}" CACHE STRING "linker flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS}" CACHE STRING "linker flags" FORCE)

# Do not pass flags like '-ffunction-sections -fdata-sections' to the linker.
# This causes undefined symbol errors when linking.
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> -o <TARGET>  <OBJECTS> <LINK_LIBRARIES> ${LINKER_LIBS}" CACHE STRING "Linker command line" FORCE)

add_definitions("-DARDUINO=${ARDUINO_VERSION}")
add_definitions("-DTEENSYDUINO=${TEENSYDUINO_VERSION}")
add_definitions("-D__${TEENSY_MODEL}__")
add_definitions(-DLAYOUT_US_ENGLISH)
add_definitions(-DUSB_VID=null)
add_definitions(-DUSB_PID=null)
add_definitions(-MMD)
