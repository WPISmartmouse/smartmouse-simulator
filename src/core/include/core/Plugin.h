#pragma once

namespace ssim {

class RobotPlugin {
public:
  virtual void Setup() = 0;

  virtual void Loop() = 0;
};

}