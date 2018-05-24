
# To get the real robot code versus the sim robot code, you just link against a different HAL library
# All the HAL libraries have the same API because they share headers!
if (${REAL})
    add_library(smartmouse_2018_arduino main.cpp)
    target_link_libraries(smartmouse_2018_arduino common arduino_hal)
else ()
    add_library(smartmouse_2018_sim main.cpp)
    target_link_libraries(smartmouse_2018_sim common simulation_hal)
endif ()
