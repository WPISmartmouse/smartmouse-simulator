#include "commands/return_to_start.h"
#include "commands/forward.h"
#include "commands/turn.h"

ReturnToStart::ReturnToStart(Smartmouse2018Robot &robot) : CommandGroup("return"), robot(robot) {
}

void ReturnToStart::initialize() {
  // plan path from center to origin
  robot.maze.flood_fill_from_point(&pathToStart, ssim::CENTER, ssim::ORIGIN);
  index = 0;
}

bool ReturnToStart::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool returned = (robot.getRowCol() == ssim::ORIGIN);

    if (!returned) {
      ssim::MotionPrimitive prim = pathToStart[index++];
      add<Turn>(robot, prim.d);
      add<Forward>(robot);
    } else {
      return true;
    }

    return false;
  }

  return false;
}
