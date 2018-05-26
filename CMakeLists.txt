cmake_minimum_required(VERSION 3.1)
project(smartmouse-simulator)

option(REAL "target teensy" OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Testing -- only valid on sim
if (NOT ${REAL})
    add_subdirectory(googletest)
endif ()

include(cmake/Teensy.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Arduino
if (CMAKE_HOST_WIN32)
    set(ARDUINO_ROOT "C:/PROGRA~2/Arduino" CACHE PATH "Path to the Arduino directory")
else ()
    set(ARDUINO_ROOT "/opt/arduino-1.8.4" CACHE PATH "Path to the Arduino directory")
endif ()
set(ARDUINO_LIB_ROOT "${ARDUINO_ROOT}/hardware/teensy/avr/libraries" CACHE PATH "arduino libraries directory")

add_subdirectory(src)
add_subdirectory(robots)
