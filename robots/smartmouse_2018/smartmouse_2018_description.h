#pragma once

#include <core/msgs.h>

const auto smartmouse_2018_description = ssim::RobotDescription{
    .footprint = {{0, 0},
                  {0, 1}},
    .left_wheel = {},
    .right_wheel = {},
    .cog = {},
    .left_motor = {},
    .right_motor = {},
    .left_encoder = {},
    .right_encoder = {},
    .sensors = {},
    .digital_outputs = {},
    .digital_inputs = {},
    .analog_outputs = {},
    .analog_inputs = {},
    .leds = {}
};
