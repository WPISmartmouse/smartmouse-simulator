#pragma once

#include <commanduino/commanduino.h>
#include <core/pose.h>
#include "../../../src/kinematic_controller/include/kinematic_controller/velocity_profile.h"

#include "smartmouse_2018_robot.h"

class Forward : public Command {
public:
  explicit Forward(Smartmouse2018Robot &robot);

  void initialize() override;

  void execute() override;

  bool isFinished() override;

  void end() override;

private:
  ssim::GlobalPose start;
  Smartmouse2018Robot &robot;
  ssim::VelocityProfile *profile;
};

