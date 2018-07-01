#pragma once

#include <commanduino/commanduino.h>

#include "smartmouse_2018_robot.h"

class ReturnToStart : public CommandGroup {
public:
  explicit ReturnToStart(Smartmouse2018Robot &robot);

  void initialize() override;

  bool isFinished() override;

private:
  ssim::route_t pathToStart;
  int index;
  Smartmouse2018Robot &robot;

};
