#pragma once

#include <core/maze.h>
#include <core/msgs.h>
#include <core/plugin.h>

namespace ssim {

extern RobotDescription robot_description;
extern RobotPlugin *global_plugin;

inline double cellsToRad(double x) {
  return x * UNIT_DIST_M / robot_description.wheels.radius;
}

inline double meterToRad(double x) {
  return x / robot_description.wheels.radius;
}

inline double radToMeters(double x) {
  return x * robot_description.wheels.radius;
}

inline double radToCU(double x) {
  return x * robot_description.wheels.radius / UNIT_DIST_M;
}

}
