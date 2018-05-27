#pragma once

#include <commanduino/commanduino.h>
#include "../../../src/core/include/core/direction.h"
#include <kinematic_controller/VelocityProfile.h>

#include "smartmouse_2018_robot.h"

class TurnInPlace : public Command {
public:
  TurnInPlace(Smartmouse2018Robot &robot, ssim::Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  Smartmouse2018Robot &robot;
  bool left_turn;
  ssim::Direction dir;
  double goal_yaw;
  double yaw_error;


  const static double kP;
  ssim::VelocityProfile *profile;
};

