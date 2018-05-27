#pragma once

#include <commanduino/commanduino.h>
#include "../../../src/kinematic_controller/include/kinematic_controller/velocity_profile.h"

#include "smartmouse_2018_robot.h"

class ForwardToCenter : public Command {
public:
  ForwardToCenter(Smartmouse2018Robot &robot);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  constexpr static double kPFwd = 5;

  ssim::GlobalPose start;
  Smartmouse2018Robot &robot;
  ssim::VelocityProfile *profile;
};

