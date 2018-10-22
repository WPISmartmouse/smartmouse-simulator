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

class PhysicsConfig {
 public:
  std::optional<uint32_t> ns_of_sim_per_step;
  std::optional<double> real_time_factor; // desired RTF
};

} // namespace ssim
