file(GLOB COMMAND_SRCS commands/*.cpp)
file(GLOB COMMAND_HDRS commands/*.h)
add_library(commands ${COMMAND_SRCS} ${COMMAND_HDRS})
target_include_directories(commands PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# To get the real robot code versus the sim robot code, you just link against a different HAL library
# All the HAL libraries have the same API because they share headers!
if (${REAL})
    target_link_libraries(commands core commanduino kinematic_controller arduino_hal)

    add_library(smartmouse_2018_arduino main.cpp)
    target_link_libraries(smartmouse_2018_arduino core commanduino kinematic_controller arduino_hal commands)
else ()
    target_link_libraries(commands core commanduino kinematic_controller sim_hal)

    add_library(smartmouse_2018_sim main.cpp)
    target_link_libraries(smartmouse_2018_sim core commanduino kinematic_controller sim_hal commands)
endif ()
