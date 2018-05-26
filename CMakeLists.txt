cmake_minimum_required(VERSION 3.1)
project(smartmouse-simulator)

option(REAL "target teensy" OFF)

# this is a hack to we can find the "Teensy-config.cmake" file
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# for finding arduino libraries we need to build
if (CMAKE_HOST_WIN32)
    set(ARDUINO_ROOT "C:/PROGRA~2/Arduino" CACHE PATH "Path to the Arduino directory")
else()
    set(ARDUINO_ROOT "/opt/arduino-1.8.4" CACHE PATH "Path to the Arduino directory")
endif()
set(ARDUINO_LIB_ROOT "${ARDUINO_ROOT}/hardware/teensy/avr/libraries" CACHE PATH "arduino libraries directory")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(src)
add_subdirectory(robots)
