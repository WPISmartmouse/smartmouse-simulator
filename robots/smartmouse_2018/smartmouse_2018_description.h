#pragma once

#include <math.h>
#include <core/maze.h>
#include <core/msgs.h>

constexpr unsigned int BACK_LEFT_ANALOG_PIN = 33;
constexpr unsigned int FRONT_RIGHT_ANALOG_PIN = 34;
constexpr unsigned int FRONT_ANALOG_PIN = 35;
constexpr unsigned int GERALD_LEFT_ANALOG_PIN = 36;
constexpr unsigned int FRONT_LEFT_ANALOG_PIN = 37;
constexpr unsigned int GERALD_RIGHT_ANALOG_PIN = 38;
constexpr unsigned int BACK_RIGHT_ANALOG_PIN = 39;
constexpr unsigned int BATTERY_ANALOG_PIN = 66;

// TODO: remove these and use new SPI encoders
constexpr unsigned int MOSI = 11;
constexpr unsigned int LEFT_ENCODER_CS = 9;
constexpr unsigned int RIGHT_ENCODER_CS = 10;

constexpr unsigned int MOTOR_LEFT_A1 = 5;
constexpr unsigned int MOTOR_LEFT_A2 = 6;
constexpr unsigned int MOTOR_RIGHT_B1 = 8;
constexpr unsigned int MOTOR_RIGHT_B2 = 7;


constexpr unsigned int LED_1 = 25;
constexpr unsigned int LED_2 = 26;
constexpr unsigned int LED_3 = 27;
constexpr unsigned int LED_4 = 28;
constexpr unsigned int LED_5 = 29;
constexpr unsigned int LED_6 = 30;
constexpr unsigned int LED_7 = 31;
constexpr unsigned int SYS_LED = 32;

constexpr unsigned int FRONT_LEFT_ENABLE_PIN = 21;
constexpr unsigned int GERALD_LEFT_ENABLE_PIN = 20;
constexpr unsigned int GERALD_RIGHT_ENABLE_PIN = 22;
constexpr unsigned int FRONT_RIGHT_ENABLE_PIN = 19;
constexpr unsigned int FRONT_ENABLE_PIN = 18;
constexpr unsigned int BACK_LEFT_ENABLE_PIN = 2;
constexpr unsigned int BACK_RIGHT_ENABLE_PIN = 4;

constexpr unsigned int BUTTON_PIN = 23;

extern const ssim::SensorDescription FRONT_LEFT_SENSOR;
extern const ssim::SensorDescription GERALD_LEFT_SENSOR;
extern const ssim::SensorDescription GERALD_RIGHT_SENSOR;
extern const ssim::SensorDescription FRONT_RIGHT_SENSOR;
extern const ssim::SensorDescription FRONT_SENSOR;
extern const ssim::SensorDescription BACK_LEFT_SENSOR;
extern const ssim::SensorDescription BACK_RIGHT_SENSOR;

constexpr double MAX_HARDWARE_SPEED_MPS = 0.90;
constexpr double MIN_SPEED_MPS = 0.0145;
constexpr double FRONT_WALL_THRESHOLD = 0.12;
constexpr double SIDE_WALL_THRESHOLD = 0.20;
constexpr double GERALD_WALL_THRESHOLD = 0.15;
constexpr double WALL_CHANGED_THRESHOLD = 0.02;
constexpr double USE_FRONT_WALL_FOR_POSE = 0.09;
constexpr double ROT_TOLERANCE = 0.025;
constexpr double TRACK_WIDTH_M = 0.053;
constexpr double ANALOG_MAX_DIST_M = 0.20;
constexpr double ANALOG_MIN_DIST_M = 0.015;
constexpr double WHEEL_RAD = 0.0145;
constexpr double MIN_ABSTRACT_FORCE = 12;
constexpr int TICKS_PER_REVOLUTION = 16384;
constexpr double RAD_PER_TICK = 2 * M_PI / TICKS_PER_REVOLUTION;

constexpr double TRACK_WIDTH_CU = ssim::toCellUnits(TRACK_WIDTH_M);
constexpr double MAX_HARDWARE_SPEED_CUPS = ssim::toCellUnits(MAX_HARDWARE_SPEED_MPS);
constexpr double MIN_SPEED_CUPS = ssim::toCellUnits(MIN_SPEED_MPS);
constexpr double ANALOG_MAX_DIST_CU = ssim::toCellUnits(ANALOG_MAX_DIST_M);
constexpr double ANALOG_MIN_DIST_CU = ssim::toCellUnits(ANALOG_MIN_DIST_M);
