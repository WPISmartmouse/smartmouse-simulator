cmake_minimum_required(VERSION 3.1)
project(smartmouse-simulator)

option(REAL "target teensy" OFF)

# this is a hack to we can find the "Teensy-config.cmake" file
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(src)
add_subdirectory(robots)
