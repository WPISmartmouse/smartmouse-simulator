#pragma once

#include <commanduino/commanduino.h>
#include <core/direction.h>

#include "smartmouse_2018_robot.h"

class Turn : public CommandGroup {
public:
  explicit Turn(Smartmouse2018Robot &robot, ssim::Direction dir);

  void initialize() override;

private:
  Smartmouse2018Robot &robot;
  ssim::Direction dir;
};

