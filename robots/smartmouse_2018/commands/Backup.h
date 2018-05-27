#pragma once

#include <commanduino/CommanDuino.h>

#include "smartmouse_2018_robot.h"

class Backup : public Command {
public:
  Backup(Smartmouse2018Robot &robot);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  Smartmouse2018Robot &robot;
};

