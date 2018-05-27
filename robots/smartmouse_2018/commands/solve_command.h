#pragma once

#include <commanduino/commanduino.h>
#include <core/Solver.h>

#include "smartmouse_2018_robot.h"

class SolveCommand : public CommandGroup {
public:
  SolveCommand(Smartmouse2018Robot &robot, ssim::Solver *solver);

  void initialize();

  bool isFinished();

private:
  Smartmouse2018Robot &robot;
  ssim::Solver *solver;
  static constexpr int MAX_RUNS = 4;
  int runs;
};
