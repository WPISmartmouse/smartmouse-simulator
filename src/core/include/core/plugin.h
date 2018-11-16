#pragma once

#include <core/maze.h>
#include <core/msgs.h>

namespace ssim {

class RobotPlugin {
 public:
  virtual void Setup() {};

  virtual void Step() {};

  Debug const &get_debug() const noexcept;

 private:
  Debug debug_;

};

} // namespace ssim
