#include <Arduino.h>
#include <commanduino/commanduino.h>
#include <core/mouse.h>
#include <core/flood.h>
#include <core/plugin.h>

#include "smartmouse_2018_robot.h"

class Smartmouse2018Plugin : public ssim::RobotPlugin {

 public:
  std::unique_ptr<Command> root_command = {nullptr};
  Smartmouse2018Robot robot;
  long last_t_us = millis();
  long last_blink_us = millis();
  bool done = false;
  bool on = true;
  bool paused = false;

  void Setup() override;

  void Step() override;

};

