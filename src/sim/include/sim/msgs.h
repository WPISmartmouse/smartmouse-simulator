#pragma once

#include <sim/time.h>

namespace ssim {

struct WallPoints {
};
struct WorldStatistics {
  unsigned long step = 0;
  Time sim_time;
  double real_time_factor = 0; // RTF acutally acheived
};

struct ServerControl {
};

struct PhysicsConfig {
};

struct Maze {
};

struct RobotCommand {
};

struct RobotDescription {
};

struct RobotSimState {
};

struct SensorDescription {
};

typedef std::vector<ssim::WallPoints> maze_walls_t[ssim::SIZE][ssim::SIZE];

} // namespace ssim
