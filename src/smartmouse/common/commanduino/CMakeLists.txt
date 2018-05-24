file(GLOB SRCS *.cpp)
file(GLOB HDRS *.h)

add_library(commanduino ${SRCS} ${HDRS})
target_include_directories(commanduino PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
