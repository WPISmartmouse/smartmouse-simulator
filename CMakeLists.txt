cmake_minimum_required(VERSION 3.10)
project(smartmouse-simulator)

# This is used by the rops tool
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")

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

find_package(Eigen3 REQUIRED NO_MODULE)

# Testing -- only valid on sim
if (${REAL})
    add_definitions(-DREAL -DNO_IMPLICIT_EXTERN_C)
    set(BUILD_TESTS OFF)
endif ()

if (${BUILD_TESTS})
    enable_testing()
    include(GoogleTest)
    add_subdirectory(googletest)
endif ()

include(cmake/Teensy.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 --coverage -pg")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(src)
add_subdirectory(robots)
