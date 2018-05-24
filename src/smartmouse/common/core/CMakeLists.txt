file(GLOB CORE_SRCS *.cpp)
file(GLOB CORE_HDRS *.h)

add_library(core ${CORE_SRCS} ${CORE_HDRS})