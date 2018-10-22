// TODO: finish unit conversion

#pragma once

#include <commanduino/commanduino.h>
#include <core/direction.h>
#include <hal/util.h>
#include <core/maze.h>
#include <smartmouse_2018_description.h>

#include "smartmouse_2018_robot.h"

class ArcTurn : public Command {
 public:
  explicit ArcTurn(Smartmouse2018Robot &robot, ssim::Direction dir);

  void initialize() override;

  void execute() override;

  bool isFinished() override;

  void end() override;

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
  double slow_arc_speed;
  double fast_arc_speed;

  constexpr static double kp_turn = 3.00;
  constexpr static double ang_weight = 1.00;
  constexpr static double arc_weight = 1.00;

  double pose_dist(ssim::GlobalPose pose, double x, double y);
};
