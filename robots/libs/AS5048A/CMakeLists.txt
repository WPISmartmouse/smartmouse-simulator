add_library(AS5048A as5048a.cpp as5048a.h)
target_link_libraries(AS5048A PUBLIC hal SPI)
target_include_directories(AS5048A PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})