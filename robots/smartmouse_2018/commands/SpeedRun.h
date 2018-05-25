#pragma once

#include "CommanDuino.h"

#include "smartmouse_2018_robot.h"

class SpeedRun : public CommandGroup {
public:
  explicit SpeedRun(Smartmouse2018Robot &robot);

  void initialize();

  bool isFinished();

private:
  Smartmouse2018Robot &robot;
  ssim::route_t *path;
  unsigned long index;
};
