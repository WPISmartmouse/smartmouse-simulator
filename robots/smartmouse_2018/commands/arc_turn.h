// TODO: finish unit conversion

#pragma once

#include <commanduino/commanduino.h>
#include "../../../src/core/include/core/direction.h"
#include <core/abstract_maze.h>
#include <kinematic_controller/RobotConfig.h>

#include "smartmouse_2018_robot.h"

class ArcTurn : public Command {
public:
  ArcTurn(Smartmouse2018Robot &robot, ssim::Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  Smartmouse2018Robot &robot;
  ssim::Direction dir;

  ssim::GlobalPose curPose;
  unsigned int curCol;
  unsigned int curRow;
  ssim::Direction curDir;

  ssim::GlobalPose startPose;
  unsigned int startCol;
  unsigned int startRow;

  double dYaw;
  double goalYaw;

  double vtc_x;
  double vtc_y;

  constexpr static double speed_scale = 0.75;
  double SLOW_ARC_SPEED = speed_scale * (ssim::MAX_SPEED_CUPS / (ssim::HALF_UNIT_DIST + (ssim::TRACK_WIDTH_M / 2))) *
                          (ssim::HALF_UNIT_DIST - (ssim::TRACK_WIDTH_M / 2));
  double FAST_ARC_SPEED = speed_scale * (ssim::MAX_SPEED_CUPS);

  constexpr static double kp_turn = 3.00;
  constexpr static double ang_weight = 1.00;
  constexpr static double arc_weight = 1.00;

  double pose_dist(ssim::GlobalPose pose, double x, double y);
};
