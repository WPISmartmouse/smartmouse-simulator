#include <hal/util.h>

#include <commands/solve_maze.h>
#include <commands/forward_n.h>
#include <commands/turn.h>
#include <commands/forward_to_center.h>
#include <commands/turn_in_place.h>

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
      ssim::MotionPrimitive prim = solver->planNextStep();

      if (!solver->isSolvable()) {
        solved = false;
        return true;
      }

      if (prim.d == solver->mouse->getDir()) {
        add<ForwardN>(robot, prim.n);
      } else {
        add<Turn>(robot, prim.d);
      }

      movements++;
    } else if (!at_center) {
      add<ForwardToCenter>(robot);
      if (goal == ssim::Solver::Goal::START) {
        add<TurnInPlace>(robot, ssim::Direction::E);
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
