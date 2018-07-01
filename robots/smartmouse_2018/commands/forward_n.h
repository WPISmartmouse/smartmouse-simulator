#pragma once

#include <commanduino/commanduino.h>
#include <core/pose.h>
#include "../../../src/kinematic_controller/include/kinematic_controller/velocity_profile.h"

#include "smartmouse_2018_robot.h"

class ForwardN : public Command {
public:
  explicit ForwardN(Smartmouse2018Robot &robot, unsigned int n);

  void initialize() override;

  void execute() override;

  bool isFinished() override;

  void end() override;

private:

  ssim::GlobalPose start;
  Smartmouse2018Robot &robot;
  unsigned int n;
  ssim::VelocityProfile *profile;
};

