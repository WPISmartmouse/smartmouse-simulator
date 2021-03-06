set(ARDUINO_LIBS SPI)

if(${REAL})
  file(GLOB_RECURSE TEENSY_C_CORE_FILES ${TEENSY_ROOT}/*.c)
  file(GLOB_RECURSE TEENSY_CXX_CORE_FILES ${TEENSY_ROOT}/*.cpp)
  set(TARGET_FLAGS "-DUSB_SERIAL -DF_CPU=${TEENSY_FREQUENCY}000000")
  add_library(hal
              arduino/arduino_hal.cpp
              ${TEENSY_C_CORE_FILES}
              ${TEENSY_CXX_CORE_FILES})
  target_include_directories(
    hal
    PUBLIC arduino $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
           $<INSTALL_INTERFACE:common/include>)
  target_compile_definitions(hal PUBLIC ${TARGET_FLAGS})
  target_compile_options(hal PUBLIC ${TEENSY_FLAGS})

  foreach(ARDUINO_LIB ${ARDUINO_LIBS})
    set(LIB_DIR "${ARDUINO_LIB_ROOT}/${ARDUINO_LIB}")
    file(GLOB SRCS "${LIB_DIR}" "${LIB_DIR}/*.cpp")
    add_library(${ARDUINO_LIB} ${SRCS})
    target_include_directories(${ARDUINO_LIB} PUBLIC ${LIB_DIR})
    target_link_libraries(${ARDUINO_LIB} hal)
  endforeach()
else()
  add_library(hal common/util.cpp sim/sim_hal.cpp)
  target_include_directories(
    hal
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sim>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
           $<INSTALL_INTERFACE:sim>
           $<INSTALL_INTERFACE:common/include>)
  target_link_libraries(hal core fmt::fmt)

  foreach(ARDUINO_LIB ${ARDUINO_LIBS})
    set(LIB_DIR sim/${ARDUINO_LIB})
    file(GLOB SRCS "${LIB_DIR}" "${LIB_DIR}/*.cpp")
    add_library(${ARDUINO_LIB} ${SRCS})
    target_include_directories(${ARDUINO_LIB} PUBLIC ${LIB_DIR})
    target_link_libraries(${ARDUINO_LIB} hal)
  endforeach()
endif()
