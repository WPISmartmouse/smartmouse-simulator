#pragma once

#include "CommanDuino.h"

#include "smartmouse_2018_robot.h"

class WaitForStart : public CommandGroup {
public:
  explicit WaitForStart(Smartmouse2018Robot &robot);

  void initialize();
  void execute();
  bool isFinished();
  void end();

private:
  Smartmouse2018Robot &robot;
  double speed;
};
