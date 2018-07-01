#include <Arduino.h>
#include <commanduino/commanduino.h>
#include <core/mouse.h>
#include <core/flood.h>
#include <core/plugin.h>

#include "commands/solve_command.h"
#include "smartmouse_2018_robot.h"

#include "smartmouse_2018_plugin.h"


void Smartmouse2018Main::Setup() {
  robot.setup();

  auto root = std::make_unique<SolveCommand>(robot, new ssim::Flood(&robot));

  last_t_us = micros();
  last_blink_us = micros();

}

void Smartmouse2018Main::Loop() {
  Smartmouse2018Robot::checkVoltage();

  long now_us = micros();
  double dt_us = now_us - last_t_us;

  if (now_us - last_blink_us > 1000000) {
    last_blink_us = now_us;
    digitalWrite(Smartmouse2018Robot::SYS_LED, static_cast <uint8_t>(on));
    on = !on;
  }

// minimum period of main loop
  if (dt_us < 1500) {
    return;
  }

  if (not paused and not done) {
    done = root->run();
  } else {
    robot.setSpeedCps(0, 0);
    digitalWrite(Smartmouse2018Robot::SYS_LED, 1);
    digitalWrite(Smartmouse2018Robot::LED_2, 1);
    digitalWrite(Smartmouse2018Robot::LED_4, 1);
    digitalWrite(Smartmouse2018Robot::LED_6, 1);
  }

  auto dt_s = dt_us / 1e6;

  robot.
      run(dt_s);

  last_t_us = now_us;
}

