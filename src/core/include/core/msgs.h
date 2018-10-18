#pragma once

#include <cstdint>
#include <array>
#include <vector>

#include <core/math.h>

namespace ssim {

class RowColYaw {
 public:
  double row;
  double col;
  double yaw;
};

class XYTheta {
 public:
  double x;
  double y;
  double theta;
};

class SensorDescription {
 public:
  XYTheta p;
  double min_range_m;
  double max_range_m;
  double beam_angle_rad;
  double a;
  double b;
  double c;
  double d;
};

class SensorsDescription {
 public:
  SensorDescription front;
  SensorDescription gerald_left;
  SensorDescription gerald_right;
  SensorDescription front_left;
  SensorDescription front_right;
  SensorDescription back_left;
  SensorDescription back_right;
};

class DigitalInputDescription {
 public:
  unsigned int pin;
};

class DigitalOutputDescription {
 public:
  unsigned int pin;
};

class AnalogInputDescription {
 public:
  unsigned int pin;
  unsigned int n_bits;
};

class AnalogOutputDescription {
 public:
  unsigned int pin;
  unsigned int n_bits;
};

class EncoderDescription {
 public:
  unsigned int cs_pin;
  unsigned int n_bits;
};

class LEDDescription {
 public:
  unsigned int pin = 0;
  int r = 0;
  int g = 0;
  int b = 0;
};

class MotorDescription {
 public:
  double u_kinetic;
  double u_static;
  double J;
  double b;
  double R;
  double L;
  double K;
};

class WheelDescription {
 public:
  Eigen::Vector2d pose;
  double radius;
  double thickness;
  double u_static;
};

class RobotDescription {
 public:
  std::vector<Eigen::Vector2d> footprint;
  WheelDescription left_wheel;
  WheelDescription right_wheel;
  Eigen::Vector3d cog;
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
};

class WheelPhysicsState {
 public:
  double theta; // radians
  double omega; // radians/second
  double alpha;
  double current; // amperes
};

class RobotSimState {
 public:
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

class Command {
 public:
  int32_t abstract_force = 0;
};

class RobotCommand {
 public:
  Command left;
  Command right;
};

class WorldStatistics {
 public:
  unsigned long step = 0;
  int32_t time_s = 0;
  int32_t time_ns = 0;
  double real_time_factor = 0.0; // RTF acutally acheived
};

} // namespace ssim
