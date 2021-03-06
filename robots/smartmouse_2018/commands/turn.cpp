#include <smartmouse_2018_description.h>

#include "commands/turn.h"
#include "commands/turn_in_place.h"
#include "commands/forward.h"
#include "commands/arc_turn.h"
#include "commands/forward_to_center.h"
#include "commands/wall_smash.h"
#include "commands/backup.h"

Turn::Turn(Smartmouse2018Robot &robot, ssim::Direction dir) : CommandGroup("RealTurnGroup"), robot(robot), dir(dir) {}

void Turn::initialize() {
  // if we want a logical 180 turn, we do full stop then turn.
  if (opposite_direction(robot.getDir()) == dir) {
    add<ForwardToCenter>(robot); // slowly stop
    add<TurnInPlace>(robot, dir);
    add<Forward>(robot);
  } else if (robot.getDir() != dir) {
    if (robot.wall_smash) {
      add<WallSmash>(robot);
      add<TurnInPlace>(robot, dir);
      add<Forward>(robot);
    }
    else {
      add<ForwardToCenter>(robot);
      add<TurnInPlace>(robot, dir);
      add<Forward>(robot);
    }
  }
}

