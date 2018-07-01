#pragma once

#include <commanduino/commanduino.h>
#include "../../../src/core/include/core/direction.h"
#include "../../../src/kinematic_controller/include/kinematic_controller/velocity_profile.h"

#include "smartmouse_2018_robot.h"

class TurnInPlace : public Command {
public:
  TurnInPlace(Smartmouse2018Robot &robot, ssim::Direction dir);

  void initialize() override;

  void execute() override;

  bool isFinished() override;

  void end() override;

private:
  Smartmouse2018Robot &robot;
  bool left_turn;
  ssim::Direction dir;
  double goal_yaw;
  double yaw_error;


  const static double kP;
  ssim::VelocityProfile *profile;
};

