#include "commands/Turn.h"
#include "commands/ForwardToCenter.h"
#include "commands/TurnInPlace.h"
#include "commands/SolveCommand.h"
#include "commands/WaitForStart.h"
#include "commands/Stop.h"
#include "commands/Finish.h"
#include "commands/SolveMaze.h"

SolveCommand::SolveCommand(Smartmouse2018Robot &robot, ssim::Solver *solver) : CommandGroup("SolveGroup"), robot(robot),
                                                                               solver(solver) {}


void SolveCommand::initialize() {
  runs = 0;
  addSequential(new WaitForStart(robot));
  solver->setup();
  addSequential(new Stop(robot, 1000));
  addSequential(new SolveMaze(robot, solver, ssim::Solver::Goal::CENTER));
  addSequential(new Finish(robot, solver->mouse->maze));
  addSequential(new SolveMaze(robot, solver, ssim::Solver::Goal::START));
}

bool SolveCommand::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    runs++;

    // we could check for some finished button here
    if (runs == MAX_RUNS) {
      return true;
    }

    addSequential(new Stop(robot, 200));
    addSequential(new WaitForStart(robot));
    addSequential(new SolveMaze(robot, solver, ssim::Solver::Goal::CENTER));
    addSequential(new Finish(robot, solver->mouse->maze));
    addSequential(new SolveMaze(robot, solver, ssim::Solver::Goal::START));
    return false;
  }

  return false;
}
