#pragma once

#include <core/maze.h>
#include <core/fwd_msgs.h>

namespace ssim {

class RobotPlugin {
 public:
  virtual void Setup() = 0;

  virtual void Loop() = 0;

  virtual void OnWorldStats(WorldStatistics const &msg) {};

  virtual void OnPhysics(PhysicsConfig const &msg) {};

  virtual void OnServerControl(ServerControl const &msg) {};

  virtual void OnRobotState(RobotSimState const &msg) {};

  virtual void OnRobotCommand(RobotCommand const &msg) {};

  virtual void OnRobot(RobotDescription const &msg) {};

  virtual void OnMaze(AbstractMaze const &msg) {};

  virtual void OnPIDConstants(PIDConstants const &msg) {};

  virtual void OnPIDSetpoints(PIDSetpoints const &msg) {};
};

}
