#pragma once

#include "CommanDuino.h"
#include "Direction.h"

#include "smartmouse_2018_robot.h"

class Turn : public CommandGroup {
public:
  explicit Turn(Smartmouse2018Robot &robot, ssim::Direction dir);

  void initialize();

private:
  Smartmouse2018Robot &robot;
  ssim::Direction dir;
};

