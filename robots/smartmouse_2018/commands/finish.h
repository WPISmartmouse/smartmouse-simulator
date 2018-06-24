#pragma once

#include <commanduino/commanduino.h>

#include "smartmouse_2018_robot.h"

class Finish : public Command {
public:
  Finish(Smartmouse2018Robot &robot);

  void initialize();
  void execute();
  bool isFinished();

private:
  const unsigned int BLINK_TIME = 50;

  Smartmouse2018Robot &robot;
  unsigned long t;
  uint8_t pin_id;
  bool on;
};
