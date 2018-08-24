#include <core/maze.h>

#include <kinematic_controller/robot_config.h>

namespace ssim {

double MAX_SPEED_MPS = 0.72;
double MAX_SPEED_CUPS = toCellUnits(MAX_SPEED_MPS);
bool WALL_SMASH = false;

} // namespace ssim
