#include <Arduino.h>
#include "CommanDuino.h"
#include "mouse.h"
#include "Flood.h"
#include "Plugin.h"

#include "commands/SolveCommand.h"
#include "smartmouse_2018_robot.h"

class Smartmouse2018Main : public ssim::RobotPlugin {

public:
  Scheduler *scheduler;
  Smartmouse2018Robot robot;
  long last_t_us, last_blink_us;
  bool done = false;
  bool on = true;
  bool paused = false;

  void Setup() override {
    robot.setup();

    scheduler = new Scheduler(new SolveCommand(robot, new ssim::Flood(&robot)));

    last_t_us = micros();
    last_blink_us = micros();

  }

  void Loop() override {
    Smartmouse2018Robot::checkVoltage();

    long now_us = micros();
    double dt_us = now_us - last_t_us;

    if (now_us - last_blink_us > 1000000) {
      last_blink_us = now_us;
      digitalWrite(Smartmouse2018Robot::SYS_LED, static_cast<uint8_t>(on));
      on = !on;
    }

    // minimum period of main loop
    if (dt_us < 1500) {
      return;
    }

    if (not paused and not done) {
      done = scheduler->run();
    } else {
      robot.setSpeedCps(0, 0);
      digitalWrite(Smartmouse2018Robot::SYS_LED, 1);
      digitalWrite(Smartmouse2018Robot::LED_2, 1);
      digitalWrite(Smartmouse2018Robot::LED_4, 1);
      digitalWrite(Smartmouse2018Robot::LED_6, 1);
    }

    auto dt_s = dt_us / 1e6;

    robot.run(dt_s);

    last_t_us = now_us;
  }

} robot_plugin;

