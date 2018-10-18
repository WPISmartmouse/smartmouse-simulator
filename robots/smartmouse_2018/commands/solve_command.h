#pragma once

#include <commanduino/commanduino.h>
#include <core/solver.h>

class SolveCommand : public CommandGroup {
public:
  SolveCommand(Smartmouse2018Robot &robot, ssim::Solver *solver);

  void initialize() override;

  bool isFinished() override;

private:
  Smartmouse2018Robot &robot;
  ssim::Solver *solver;
  static constexpr int MAX_RUNS = 4;
  int runs;
};
