#include <hal/hal.h>

#include "smartmouse_2018_description.h"

const ssim::SensorDescription FRONT_LEFT_SENSOR = {
    .p = {.x = 0.035, .y = -0.026, .theta = -1.3446},
    .min_range_m = 0,
    .max_range_m = 0,
    .beam_angle_rad = 0,
    .a = 1.304576,
    .b = 0.034583,
    .c = 295.613524,
    .calibration_offset = 0,
    .calibration_distance = 0.05621,
    .adc_bits = 14
};
const ssim::SensorDescription GERALD_LEFT_SENSOR = {
    .p = {.x = 0.040, .y = -0.022, .theta = -0.785398},
    .min_range_m = 0,
    .max_range_m = 0,
    .beam_angle_rad = 0,
    .a = 1.195759,
    .b = 0.020983,
    .c = 531.855338,
    .calibration_offset = 0,
    .calibration_distance = 0.05220,
    .adc_bits = 14
};
const ssim::SensorDescription GERALD_RIGHT_SENSOR = {
    .p = {.x = 0.040, .y = 0.022, .theta = 0.785398},
    .min_range_m = 0,
    .max_range_m = 0,
    .beam_angle_rad = 0,
    .a = 1.187237,
    .b = 0.020162,
    .c = 658.131265,
    .calibration_offset = 0,
    .calibration_distance = 0.06385,
    .adc_bits = 14
};
const ssim::SensorDescription FRONT_RIGHT_SENSOR = {
    .p = {.x = 0.035, .y = 0.026, .theta = 1.3446},
    .min_range_m = 0,
    .max_range_m = 0,
    .beam_angle_rad = 0,
    .a = 1.197716,
    .b = 0.021642,
    .c = 370.645580,
    .calibration_offset = 0,
    .calibration_distance = 0.10000,
    .adc_bits = 14
};
const ssim::SensorDescription FRONT_SENSOR = {
    .p = {.x = 0.055, .y = 0.0, .theta = 0.0},
    .min_range_m = 0,
    .max_range_m = 0,
    .beam_angle_rad = 0,
    .a = 1.168262,
    .b = 0.017652,
    .c = 506.691267,
    .calibration_offset = 0,
    .calibration_distance = 0.05959,
    .adc_bits = 14
};
const ssim::SensorDescription BACK_LEFT_SENSOR = {
    .p = {.x = -0.0256, .y = -0.03, .theta = -1.48353},
    .min_range_m = 0,
    .max_range_m = 0,
    .beam_angle_rad = 0,
    .a = 1.240250,
    .b = 0.028287,
    .c = 380.648090,
    .calibration_offset = 0,
    .calibration_distance = 0.07390,
    .adc_bits = 14
};
const ssim::SensorDescription BACK_RIGHT_SENSOR = {
    .p = {.x = -0.0256, .y = 0.03, .theta = 1.48353},
    .min_range_m = 0,
    .max_range_m = 0,
    .beam_angle_rad = 0,
    .a = 1.235639,
    .b = 0.026170,
    .c = 388.385273,
    .calibration_offset = 0,
    .calibration_distance = 0.06928,
    .adc_bits = 14
};

// NOTE: This variable can be manipulated by the server!
// We don't need to guard it because the server and the robot plugin run synchronously
ssim::RobotDescription ssim::global_robot_description {
    .footprint = {
        {.x = -0.035, .y = 0.035},
        {.x = 0.040, .y =  0.035},
        {.x = 0.055, .y =  0.015},
        {.x = 0.055, .y =  -0.015},
        {.x = 0.040, .y =  -0.035},
        {.x = -0.035, .y = -0.035},
        {.x = -0.035, .y = 0.035}
    },
    .wheels = {
        .left_wheel_position = {0.0, -0.0275, 0.021},
        .right_wheel_position = {0.0, 0.0275, -0.021},
        .radius = 0.0145,
        .thickness = 0.0127,
        .u_static = 0,
    },
    .cog = {0.0, 0.0, 0.01},
    .left_motor = {
        .u_kinetic = 8.0,
        .u_static = 10.0,
        .J = 0.000658,
        .b = 0.0000012615,
        .R = 5,
        .L = 0.58,
        .K = 0.0787,
    },
    .right_motor = {
        .u_kinetic = 8.0,
        .u_static = 10.0,
        .J = 0.000658,
        .b = 0.0000012615,
        .R = 5,
        .L = 0.58,
        .K = 0.0787,
    },
    .left_encoder = {
        .n_bits = 14,
    },
    .right_encoder = {
        .n_bits = 14,
    },
    .sensors = {
        FRONT_SENSOR,
        FRONT_LEFT_SENSOR,
        FRONT_RIGHT_SENSOR,
        BACK_LEFT_SENSOR,
        BACK_RIGHT_SENSOR,
        GERALD_LEFT_SENSOR,
        GERALD_RIGHT_SENSOR
    },
    .pin_map = {
        {23, ssim::PinVariant{ssim::DigitalInputDescription{}}},
        {25, ssim::PinVariant{ssim::LEDDescription{.r=255, .g=0, .b=0}}},
        {66, ssim::PinVariant{ssim::AnalogInputDescription{}}},
    },
    .track_width_cu = TRACK_WIDTH_CU,
    .min_abstract_force = MIN_ABSTRACT_FORCE,
    .min_speed_cups = MIN_SPEED_CUPS,
    .max_speed_cups = ssim::toCellUnits(0.72),
};
