#pragma once

#include <core/msgs.h>

const auto smartmouse_2018_description = ssim::RobotDescription{
    .footprint = {
        {.x = -0.035, .y = 0.035},
        {.x = 0.040, .y =  0.035},
        {.x = 0.055, .y =  0.015},
        {.x = 0.055, .y =  -0.015},
        {.x = 0.040, .y =  -0.035},
        {.x = -0.035, .y = -0.035},
        {.x = -0.035, .y = 0.035}
    },
    .left_wheel = {
        .position = {0.0, -0.0275, 0.021},
        .radius = 0.0145,
        .thickness = 0.0127,
        .u_static = 0,
    },
    .right_wheel = {
        .position = {0.0, 0.0275, -0.021},
        .radius = 0.0145,
        .thickness = 0.0127,
        .u_static = 0,
    },
    .cog = {0.0, 0.0, 0.01},
    .left_motor = {
        .pin_1 = 5,
        .pin_2 = 6,
        .u_kinetic = 8.0,
        .u_static = 10.0,
        .J = 0.000658,
        .b = 0.0000012615,
        .R = 5,
        .L = 0.58,
        .K = 0.0787,
    },
    .right_motor = {
        .pin_1 = 8,
        .pin_2 = 7,
        .u_kinetic = 8.0,
        .u_static = 10.0,
        .J = 0.000658,
        .b = 0.0000012615,
        .R = 5,
        .L = 0.58,
        .K = 0.0787,
    },
    .left_encoder = {
        .cs_pin = 9,
        .n_bits = 14,
    },
    .right_encoder = {
        .cs_pin = 9,
        .n_bits = 14,
    },
    .sensors = {
        .adc_bits = 14,
        .front = {
            .p = {
                .x = 0.055,
                .y = 0.0,
                .theta = 0.0,
            },
            .min_range_m = 0,
            .max_range_m = 0,
            .beam_angle_rad = 0,
            .a = 1.304576,
            .b = 0.034583,
            .c = 295.613524,
            .d = 0
        },
        .gerald_left = {
            .p = {
                .x = 0.040,
                .y = -0.022,
                .theta = -0.785398,
            },
            .min_range_m = 0,
            .max_range_m = 0,
            .beam_angle_rad = 0,
            .a = 1.195759,
            .b = 0.020983,
            .c = 531.855338,
            .d = 0
        },
        .gerald_right = {
            .p = {
                .x = 0.040,
                .y = 0.022,
                .theta = 0.785398,
            },
            .min_range_m = 0,
            .max_range_m = 0,
            .beam_angle_rad = 0,
            .a = 1.187237,
            .b = 0.020162,
            .c = 658.131265,
            .d = 0
        },
        .front_left = {
            .p = {
                .x = 0.035,
                .y = -0.026,
                .theta = -1.3446,
            },
            .min_range_m = 0,
            .max_range_m = 0,
            .beam_angle_rad = 0,
            .a = 1.197716,
            .b = 0.021642,
            .c = 370.645580,
            .d = 0
        },
        .front_right = {
            .p = {
                .x = 0.035,
                .y = 0.026,
                .theta = 1.3446,
            },
            .min_range_m = 0,
            .max_range_m = 0,
            .beam_angle_rad = 0,
            .a = 1.168262,
            .b = 0.017652,
            .c = 506.691267,
            .d = 0
        },
        .back_left = {
            .p = {
                .x = -0.0256,
                .y = -0.03,
                .theta = -1.48353,
            },
            .min_range_m = 0,
            .max_range_m = 0,
            .beam_angle_rad = 0,
            .a = 1.240250,
            .b = 0.028287,
            .c = 380.648090,
            .d = 0
        },
        .back_right = {
            .p = {
                .x = -0.0256,
                .y = 0.03,
                .theta = 1.48353,
            },
            .min_range_m = 0,
            .max_range_m = 0,
            .beam_angle_rad = 0,
            .a = 1.235639,
            .b = 0.026170,
            .c = 388.385273,
            .d = 0
        }
    },
    .digital_outputs = {},
    .digital_inputs = {},
    .analog_outputs = {},
    .analog_inputs = {},
    .leds = {
        {.pin=32, .r=255, .g=0, .b=0},
        {.pin=25, .r=0, .g=255, .b=255},
        {.pin=26, .r=255, .g=0, .b=255},
        {.pin=27, .r=255, .g=255, .b=0},
        {.pin=28, .r=0, .g=255, .b=255},
        {.pin=29, .r=255, .g=0, .b=255},
        {.pin=30, .r=255, .g=255, .b=0},
        {.pin=31, .r=0, .g=255, .b=255},
    },
    .battery_pin = 66,
    .button_pin = 23
};
