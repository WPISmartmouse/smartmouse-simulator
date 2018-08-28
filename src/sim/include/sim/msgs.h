#pragma once

#include <optional>
#include <cstdint>
#include <array>
#include <vector>

#include <sim/time.h>

namespace ssim {

class Vector2d {
 public:
  double x;
  double y;
};

class Vector3d {
 public:
  double x;
  double y;
  double z;
};

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

class WorldStatistics {
 public:
  unsigned long step = 0;
  Time sim_time;
  double real_time_factor = 0.0; // RTF acutally acheived
};

class ServerControl {
 public:
  std::optional<bool> pause;
  std::optional<bool> stationary;
  std::optional<uint32_t> step;
  std::optional<bool> quit;
  std::optional<bool> reset_time;
  std::optional<bool> reset_robot;
  std::optional<double> reset_row;
  std::optional<double> reset_col;
  std::optional<double> reset_yaw;
  std::optional<std::string> author;
  std::optional<bool> toggle_play_pause;
};

class PIDConstants {
 public:
  std::optional<double> kP;
  std::optional<double> kI;
  std::optional<double> kD;
  std::optional<double> kFFOffset;
  std::optional<double> kFFScale;
};

class PhysicsConfig {
 public:
  std::optional<uint32_t> ns_of_sim_per_step;
  std::optional<double> real_time_factor; // desired RTF
};

class WheelDescription {
 public:
  Vector2d pose;
  double radius;
  double thickness;
  double u_static;
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

class RobotDescription {
 public:
  std::vector<Vector2d> footprint;
  WheelDescription left_wheel;
  WheelDescription right_wheel;
  Vector3d cog;
  MotorDescription motor;
  SensorsDescription sensors;
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
  Time stamp;
};


/// A series of funtions to convert various things into the above messages
RobotDescription convert(std::ifstream const &fs);

} // namespace ssim
