
#include <commanduino/commanduino.h>
#include <core/mouse.h>
#include <core/plugin.h>

#include "smartmouse_2018_plugin.h"

#include "smartmouse_2018_description.h"


void Smartmouse2018Plugin::Setup()  {
//  auto root_command = std::make_unique<SolveCommand>(robot, new ssim::Flood(&robot));

  last_t_us = micros();
  last_blink_us = micros();

}

void Smartmouse2018Plugin::Step() {
  Smartmouse2018Robot::checkVoltage();

  long now_us = micros();
  double dt_us = now_us - last_t_us;

  if (now_us - last_blink_us > 1000000) {
    last_blink_us = now_us;
    digitalWrite(smartmouse_2018_description.leds[0].pin, static_cast <uint8_t>(on));
    on = !on;
  }

  // minimum period of main loop
  if (dt_us < 1500) {
    return;
  }

  if (not paused and not done) {
    done = root_command->run();
  } else {
    robot.setSpeedCps(0, 0);
    digitalWrite(smartmouse_2018_description.leds[0].pin, 1);
    digitalWrite(smartmouse_2018_description.leds[2].pin, 1);
    digitalWrite(smartmouse_2018_description.leds[4].pin, 1);
    digitalWrite(smartmouse_2018_description.leds[6].pin, 1);
  }

  auto dt_s = dt_us / 1e6;

  last_t_us = now_us;
}

ssim::RobotDescription const *get_description() {
  return &smartmouse_2018_description;
}

ssim::RobotPlugin *get_plugin() {
  return new Smartmouse2018Plugin();
}
