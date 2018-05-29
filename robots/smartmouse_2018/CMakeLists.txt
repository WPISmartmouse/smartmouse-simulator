file(GLOB COMMAND_SRCS commands/*.cpp)
add_library(commands ${COMMAND_SRCS})
target_include_directories(commands PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(commands commanduino kinematic_controller AS5048A hal core)

add_library(smartmouse_2018 smartmouse_2018_plugin.cpp smartmouse_2018_robot.cpp)
target_link_libraries(smartmouse_2018 commands commanduino kinematic_controller hal core)

if (${REAL})
    add_teensy_executable(teensy_main teensy_main.cpp)
    target_link_libraries(teensy_main smartmouse_2018)
endif ()

