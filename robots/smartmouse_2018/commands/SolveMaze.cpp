#include <hal/hal.h>

#include <commands/SolveMaze.h>
#include <commands/ForwardN.h>
#include <commands/Turn.h>
#include <commands/ForwardToCenter.h>
#include <commands/TurnInPlace.h>

SolveMaze::SolveMaze(Smartmouse2018Robot &robot, ssim::Solver *solver, ssim::Solver::Goal goal) : CommandGroup("solve"),
                                                                                                  robot(robot),
                                                                                                  solver(solver),
                                                                                                  movements(0),
                                                                                                  goal(goal),
                                                                                                  solved(false),
                                                                                                  at_center(false) {}

void SolveMaze::initialize() {
  solved = false;
  at_center = false;
  solver->setGoal(goal);
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      ssim::motion_primitive_t prim = solver->planNextStep();

      if (prim.d == ssim::Direction::INVALID || !solver->isSolvable()) {
        solved = false;
        return true;
      }

      if (prim.d == solver->mouse->getDir()) {
        addSequential(new ForwardN(robot, prim.n));
      } else {
        addSequential(new Turn(robot, prim.d));
      }

      movements++;
    } else if (!at_center) {
      addSequential(new ForwardToCenter(robot));
      if (goal == ssim::Solver::Goal::START) {
        addSequential(new TurnInPlace(robot, ssim::Direction::E));
      }
      at_center = true;
      solved = true;
      return false;
    } else {
      return true;
    }

    return false;
  }

  return false;
}

void SolveMaze::end() {
  ssim::print("solve time (seconds): %lu\r\n", getTime() / 1000ul);
  solver->teardown();
}
