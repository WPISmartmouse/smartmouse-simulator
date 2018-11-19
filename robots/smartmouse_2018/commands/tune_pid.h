#pragma once

#include <commanduino/commanduino.h>
#include <smartmouse_2018_robot.h>

class TunePID : public Command {
public:
  explicit TunePID(Smartmouse2018Robot &robot);

  void execute() override;

  bool isFinished() override;

  void end() override;

 private:
  Smartmouse2018Robot &robot_;
};
