file(GLOB COMMAND_SRCS commands/*.cpp)
file(GLOB COMMAND_HDRS commands/*.h)
add_library(commands ${COMMAND_SRCS} ${COMMAND_HDRS})
target_include_directories(commands PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(commands core commanduino kinematic_controller hal AS5048A)

add_library(smartmouse_2018 main.cpp)
target_link_libraries(smartmouse_2018 core commanduino kinematic_controller hal commands AS5048A)
