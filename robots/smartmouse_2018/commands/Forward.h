#pragma once

#include "CommanDuino.h"
#include "Pose.h"
#include "VelocityProfile.h"

#include "smartmouse_2018_robot.h"

class Forward : public Command {
public:
  explicit Forward(Smartmouse2018Robot &robot);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  ssim::GlobalPose start;
  Smartmouse2018Robot &robot;
  ssim::VelocityProfile *profile;
};
