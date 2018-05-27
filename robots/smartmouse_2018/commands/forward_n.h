#pragma once

#include <commanduino/commanduino.h>
#include <core/Pose.h>
#include <kinematic_controller/VelocityProfile.h>

#include "smartmouse_2018_robot.h"

class ForwardN : public Command {
public:
  explicit ForwardN(Smartmouse2018Robot &robot, unsigned int n);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  ssim::GlobalPose start;
  Smartmouse2018Robot &robot;
  unsigned int n;
  ssim::VelocityProfile *profile;
};

