project(commanduino VERSION 0.1)

file(GLOB SRCS *.cpp)

add_library(commanduino ${SRCS})
target_include_directories(
  commanduino
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(commanduino hal)

if(${BUILD_TESTS})
  add_subdirectory(test)
endif()
