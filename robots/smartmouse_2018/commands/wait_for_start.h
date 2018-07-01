#pragma once

#include <commanduino/commanduino.h>

#include "smartmouse_2018_robot.h"

class WaitForStart : public CommandGroup {
public:
  explicit WaitForStart(Smartmouse2018Robot &robot);

  void initialize() override;
  void execute() override;
  bool isFinished() override;
  void end() override;

private:
  Smartmouse2018Robot &robot;
  double speed;
};
