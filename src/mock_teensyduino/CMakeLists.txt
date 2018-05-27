set(ARDUINO_LIBS SPI)

if (${REAL})

    file(GLOB_RECURSE TEENSY_C_CORE_FILES ${TEENSY_ROOT}/*.c)
    file(GLOB_RECURSE TEENSY_CXX_CORE_FILES ${TEENSY_ROOT}/*.cpp)
    set(TARGET_FLAGS "-DUSB_SERIAL -DF_CPU=${TEENSY_FREQUENCY}000000 ${TEENSY_FLAGS}")
    add_library(teensyduino arduino/arduino_hal.cpp ${TEENSY_C_CORE_FILES} ${TEENSY_CXX_CORE_FILES})
    target_include_directories(teensyduino PUBLIC arduino)
    target_compile_definitions(teensyduino PUBLIC ${TARGET_FLAGS})

    foreach (ARDUINO_LIB ${ARDUINO_LIBS})
        set(LIB_DIR "${ARDUINO_LIB_ROOT}/${ARDUINO_LIB}")
        file(GLOB SRCS "${LIB_DIR}" "${LIB_DIR}/*.cpp")
        add_library(${ARDUINO_LIB} ${SRCS})
        target_include_directories(${ARDUINO_LIB} PUBLIC ${LIB_DIR})
        target_link_libraries(${ARDUINO_LIB} teensyduino)
    endforeach ()
else ()
    add_library(teensyduino sim/sim_hal.cpp)
    target_include_directories(teensyduino PUBLIC sim)
    target_link_libraries(teensyduino core)

    foreach (ARDUINO_LIB ${ARDUINO_LIBS})
        set(LIB_DIR sim/${ARDUINO_LIB})
        file(GLOB SRCS "${LIB_DIR}" "${LIB_DIR}/*.cpp")
        add_library(${ARDUINO_LIB} ${SRCS})
        target_include_directories(${ARDUINO_LIB} PUBLIC ${LIB_DIR})
        target_link_libraries(${ARDUINO_LIB} teensyduino)
    endforeach ()
endif ()

add_library(hal common/hal.cpp)
target_include_directories(hal PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
        $<INSTALL_INTERFACE:common/include>)
target_link_libraries(hal PUBLIC core teensyduino)