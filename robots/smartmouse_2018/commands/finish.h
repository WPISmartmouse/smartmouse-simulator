#pragma once

#include <commanduino/commanduino.h>

#include "smartmouse_2018_robot.h"

class Finish : public Command {
public:
  explicit Finish(Smartmouse2018Robot &robot);

  void initialize() override;

  void execute() override;

  bool isFinished() override;

private:
  const unsigned int BLINK_TIME = 50;

  Smartmouse2018Robot &robot;
  unsigned long t;
  unsigned int pin_id;
  bool on;
};
