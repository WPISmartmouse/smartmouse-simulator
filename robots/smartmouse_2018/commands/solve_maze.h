#pragma once

#include <commanduino/commanduino.h>
#include "../../../src/core/include/core/solver.h"

#include "smartmouse_2018_robot.h"

class SolveMaze : public CommandGroup {
public:
  SolveMaze(Smartmouse2018Robot &robot, ssim::Solver *solver, ssim::Solver::Goal goal);

  void initialize() override;

  bool isFinished() override;

  void end() override;

private:
  Smartmouse2018Robot &robot;
  ssim::Solver *solver;
  int movements;
  ssim::Solver::Goal goal;
  bool solved;

  bool at_center;
};
