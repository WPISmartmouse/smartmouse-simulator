#pragma once

#include "CommanDuino.h"
#include "Solver.h"

#include "smartmouse_2018_robot.h"

class SolveMaze : public CommandGroup {
public:
  SolveMaze(Smartmouse2018Robot &robot, ssim::Solver *solver, ssim::Solver::Goal goal);

  void initialize();

  bool isFinished();

  void end();

private:
  Smartmouse2018Robot &robot;
  ssim::Solver *solver;
  int movements;
  ssim::Solver::Goal goal;
  bool solved;

  bool at_center;
};
