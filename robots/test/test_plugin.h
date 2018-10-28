#include <Arduino.h>
#include <core/mouse.h>
#include <core/flood.h>
#include <core/plugin.h>

class TestPlugin : public ssim::RobotPlugin {

 public:
  void Setup() override;

  void Step() override;

};

