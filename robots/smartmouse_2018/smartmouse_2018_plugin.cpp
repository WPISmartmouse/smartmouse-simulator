
#include <commands/solve_command.h>
#include <commanduino/commanduino.h>
#include <core/mouse.h>
#include <core/plugin.h>

#include "smartmouse_2018_plugin.h"
#include "smartmouse_2018_description.h"


void Smartmouse2018Plugin::Setup()  {
  root_command = std::make_unique<SolveCommand>(robot, new ssim::Flood(&robot));

  last_t_us = micros();
  last_blink_us = micros();

  robot.Setup();
}

void Smartmouse2018Plugin::Step() {
  Smartmouse2018Robot::checkVoltage();

  long now_us = micros();
  double dt_us = now_us - last_t_us;

  if (now_us - last_blink_us > 1000000) {
    last_blink_us = now_us;
    digitalWrite(SYS_LED, static_cast <uint8_t>(on));
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
    digitalWrite(LED_1, 1);
    digitalWrite(LED_3, 1);
    digitalWrite(LED_5, 1);
    digitalWrite(LED_7, 1);
  }

  auto dt_s = dt_us / 1e6;

  robot.Step(dt_s);

  last_t_us = now_us;
}

ssim::RobotPlugin *ssim::global_plugin = new Smartmouse2018Plugin();
