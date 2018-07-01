#include "commands/turn.h"
#include "commands/forward_to_center.h"
#include "commands/turn_in_place.h"
#include "commands/solve_command.h"
#include "commands/wait_for_start.h"
#include "commands/stop.h"
#include "commands/finish.h"
#include "commands/solve_maze.h"

SolveCommand::SolveCommand(Smartmouse2018Robot &robot, ssim::Solver *solver) : CommandGroup("SolveGroup"),
                                                   robot(robot),
                                                   solver(solver) {}


void SolveCommand::initialize() {
  runs = 0;
  add<WaitForStart>(robot);
  solver->setup();
  add<Stop>(robot, 1000);
  add<SolveMaze>(robot, solver, ssim::Solver::Goal::CENTER);
  add<Finish>(robot);
  add<SolveMaze>(robot, solver, ssim::Solver::Goal::START);
}

bool SolveCommand::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    runs++;

    // we could check for some finished button here
    if (runs == MAX_RUNS) {
      return true;
    }

    add<Stop>(robot, 200);
    add<WaitForStart>(robot);
    add<SolveMaze>(robot, solver, ssim::Solver::Goal::CENTER);
    add<Finish>(robot);
    add<SolveMaze>(robot, solver, ssim::Solver::Goal::START);
    return false;
  }

  return false;
}
