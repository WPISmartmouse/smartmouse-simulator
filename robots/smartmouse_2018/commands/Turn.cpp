#include "commands/Turn.h"
#include "commands/TurnInPlace.h"
#include "commands/Forward.h"
#include "commands/ArcTurn.h"
#include "commands/ForwardToCenter.h"
#include "commands/WallSmash.h"
#include "commands/Backup.h"

Turn::Turn(Smartmouse2018Robot &robot, ssim::Direction dir) : CommandGroup("RealTurnGroup"), robot(robot), dir(dir) {}

void Turn::initialize() {
  // if we want a logical 180 turn, we do full stop then turn.
  if (opposite_direction(robot.getDir()) == dir) {
    addSequential(new ForwardToCenter(robot)); // slowly stop
    addSequential(new TurnInPlace(robot, dir));
    addSequential(new Forward(robot));
  } else if (robot.getDir() != dir) {
    if (ssim::WALL_SMASH) {
      addSequential(new WallSmash(robot)) ;
      addSequential(new TurnInPlace(robot, dir));
      addSequential(new Forward(robot));
    }
    else {
      addSequential(new ForwardToCenter(robot));
      addSequential(new TurnInPlace(robot, dir));
      addSequential(new Forward(robot));
    }
  }
}

