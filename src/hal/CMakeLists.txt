if (${REAL})
    # FIXME: this probably shouldn't go here
    include(Teensy)
    add_library(arduino_hal common_hal.cpp arduino/arduino_hal.cpp hal.h)
    target_include_directories(arduino_hal PRIVATE .)
elseif ()
    add_library(sim_hal common_hal.cpp sim/sim_hal.cpp hal.h)
    target_include_directories(sim_hal PRIVATE .)
endif ()
