#pragma once

#include <commanduino/commanduino.h>

#include "smartmouse_2018_robot.h"

class Stop : public Command {
public:
  explicit Stop(Smartmouse2018Robot &robot);

  Stop(Smartmouse2018Robot &robot, unsigned long stop_time);

  void initialize() override;

  bool isFinished() override;

  void end() override;

private:
  Smartmouse2018Robot &robot;
  unsigned long stop_time;
};
