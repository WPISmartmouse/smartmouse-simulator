cmake_minimum_required(VERSION 3.1)
project(smartmouse-simulator)

option(REAL "target teensy" OFF)
option(BUILD_TESTS "build tests" ON)
option(ADDRESS_SANITIZER "use address sanitizer" OFF)
option(UNDEFINED_SANITIZER "use undefined behavior sanitizer" OFF)
option(THREAD_SANITIZER "use thread sanitizer" OFF)
option(LEAK_SANITIZER "use leak sanitizer" OFF)

macro(sanitize TARGET)
    if (${ADDRESS_SANITIZER})
        target_link_libraries(${TARGET} asan)
        target_compile_options(${TARGET} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
    elseif (${UNDEFINED_SANITIZER})
        target_link_libraries(${TARGET} ubsan)
        target_compile_options(${TARGET} PRIVATE -fsanitize=undefined)
    elseif (${THREAD_SANITIZER})
        target_link_libraries(${TARGET} tsan)
        target_compile_options(${TARGET} PRIVATE -fsanitize=thread)
    elseif (${LEAK_SANITIZER})
        target_link_libraries(${TARGET} lsan)
        target_compile_options(${TARGET} PRIVATE -fsanitize=leak)
    endif ()
endmacro()

find_package(Eigen3 REQUIRED)

# Testing -- only valid on sim
if (${REAL})
    set(BUILD_TESTS OFF)
endif ()

if (${BUILD_TESTS})
    enable_testing()
    add_subdirectory(googletest)
endif ()

include(cmake/Teensy.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

# Arduino
if (CMAKE_HOST_WIN32)
    set(ARDUINO_ROOT "C:/PROGRA~2/Arduino" CACHE PATH "Path to the Arduino directory")
else ()
    set(ARDUINO_ROOT "/opt/arduino-1.8.4" CACHE PATH "Path to the Arduino directory")
endif ()
set(ARDUINO_LIB_ROOT "${ARDUINO_ROOT}/hardware/teensy/avr/libraries" CACHE PATH "arduino libraries directory")

add_subdirectory(src)
add_subdirectory(robots)
