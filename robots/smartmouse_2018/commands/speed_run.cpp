#include "commands/speed_run.h"
#include "commands/wait_for_start.h"
#include "commands/turn.h"
#include "commands/forward.h"

SpeedRun::SpeedRun(Smartmouse2018Robot &robot) : CommandGroup("speed"), robot(robot) {}

void SpeedRun::initialize() {
  index = 0;
  path = &robot.maze.fastest_route;
}

bool SpeedRun::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool returned = robot.getRow() == ssim::SIZE / 2
                    && robot.getCol() == ssim::SIZE / 2;

    if (!returned) {
      ssim::motion_primitive_t prim = path->at(index++);
      add<Turn>(robot, prim.d);
      add<Forward>(robot);
    } else {
      return true;
    }

    return false;
  }

  return false;
}
