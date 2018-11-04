#pragma once

#include <core/maze.h>
#include <core/msgs.h>

namespace ssim {

class RobotPlugin {
 public:
  virtual void Setup() {};

  virtual void Step() {};

  virtual void OnPIDConstants(PIDConstants const &/*msg*/) {};

  virtual void OnPIDSetpoints(PIDSetpoints const &/*msg*/) {};
};

} // namespace ssim
