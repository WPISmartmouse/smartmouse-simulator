#pragma once

#include <cstdint>
#include <array>
#include <vector>

#include <core/math.h>

namespace ssim {

struct Point2 {
  double x;
  double y;
};

struct Point3 {
  double x;
  double y;
  double z;
};

struct RowColYaw {
  double row;
  double col;
  double yaw;
};

struct XYTheta {
  double x;
  double y;
  double theta;
};

struct SensorDescription {
  XYTheta p;
  double min_range_m;
  double max_range_m;
  double beam_angle_rad;
  double a;
  double b;
  double c;
  double d;
};

struct SensorsDescription {
  unsigned int adc_bits;
  SensorDescription front;
  SensorDescription gerald_left;
  SensorDescription gerald_right;
  SensorDescription front_left;
  SensorDescription front_right;
  SensorDescription back_left;
  SensorDescription back_right;
};

struct DigitalInputDescription {
  unsigned int pin;
};

struct DigitalOutputDescription {
  unsigned int pin;
};

struct AnalogInputDescription {
  unsigned int pin;
  unsigned int n_bits;
};

struct AnalogOutputDescription {
  unsigned int pin;
  unsigned int n_bits;
};

struct EncoderDescription {
  unsigned int cs_pin;
  unsigned int n_bits;
};

struct LEDDescription {
  unsigned int pin = 0;
  int r = 0;
  int g = 0;
  int b = 0;
};

struct MotorDescription {
  double pin_1;
  double pin_2;
  double u_kinetic;
  double u_static;
  double J;
  double b;
  double R;
  double L;
  double K;
};

struct WheelDescription {
  Point3 position;
  double radius;
  double thickness;
  double u_static;
};

struct RobotDescription {
  std::vector<Point2> footprint;
  WheelDescription left_wheel;
  WheelDescription right_wheel;
  Point3 cog;
  MotorDescription left_motor;
  MotorDescription right_motor;
  EncoderDescription left_encoder;
  EncoderDescription right_encoder;
  SensorsDescription sensors;
  std::vector<DigitalOutputDescription> digital_outputs;
  std::vector<DigitalInputDescription> digital_inputs;
  std::vector<AnalogOutputDescription> analog_outputs;
  std::vector<AnalogInputDescription> analog_inputs;
  std::vector<LEDDescription> leds;
  unsigned int battery_pin;
  unsigned int button_pin;
};

struct WheelPhysicsState {
  double theta; // radians
  double omega; // radians/second
  double alpha;
  double current; // amperes
};

struct RobotSimState {
  WheelPhysicsState left_wheel;
  WheelPhysicsState right_wheel;
  RowColYaw p; // meters
  RowColYaw v; // meters/second
  RowColYaw a; // meters/second^2
  int32_t front_left_adc;
  int32_t front_right_adc;
  int32_t gerald_left_adc;
  int32_t gerald_right_adc;
  int32_t back_left_adc;
  int32_t back_right_adc;
  int32_t front_adc;
  double front_left_m;
  double front_right_m;
  double gerald_left_m;
  double gerald_right_m;
  double back_left_m;
  double back_right_m;
  double front_m;
};

struct Command {
  int32_t abstract_force = 0;
};

struct RobotCommand {
  Command left;
  Command right;
};

struct WorldStatistics {
  unsigned long step = 0;
  int32_t time_s = 0;
  int32_t time_ns = 0;
  double real_time_factor = 0.0; // RTF acutally acheived
};

} // namespace ssim
