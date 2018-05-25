add_library(AS5048A AS5048A.cpp AS5048A.h)
target_link_libraries(AS5048A PUBLIC hal)
target_include_directories(AS5048A PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})