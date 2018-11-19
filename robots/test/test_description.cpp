#include <hal/hal.h>

// NOTE: This variable can be manipulated by the server!
// We don't need to guard it because the server and the robot plugin run synchronously
ssim::RobotDescription ssim::robot_description {
    .footprint = {
        {.x = 0.02, .y = 0.02},
        {.x = -0.02, .y = 0.02},
        {.x = -0.02, .y = -0.02},
        {.x = 0.02, .y = -0.02},
        {.x = 0.02, .y = 0.02}
    },
    .wheels = {
        .left_wheel_position = {0.0, 0.0, 0.02},
        .right_wheel_position = {0.0, 0.0, -0.02},
        .radius = 0.01,
        .thickness = 0.01,
        .u_static = 0,
    },
    .cog = {0.0, 0.0, 0.0},
    .left_motor = {
    },
    .right_motor = {
    },
    .left_encoder = {
    },
    .right_encoder = {
    },
    .sensors = {
    },
    .pin_map = {
    },
    .track_width_cu = .04,
    .min_abstract_force = 0,
    .min_speed_cups = 0,
    .max_speed_cups = 1,
    .system_clock = {},
    .battery = {},
};
