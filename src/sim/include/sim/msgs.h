#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <optional>

#include <Eigen/Eigen>

#include <core/node.h>
#include <core/math.h>
#include <core/msgs.h>

namespace ssim {

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

class PIDSetpoints {
 public:
  std::optional<double> left_setpoints_cups;
  std::optional<double> right_setpoints_cups;
};

class PhysicsConfig {
 public:
  std::optional<uint32_t> ns_of_sim_per_step;
  std::optional<double> real_time_factor; // desired RTF
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
  MotorDescription motor;
  SensorsDescription sensors;
};

} // namespace ssim
