#include <Arduino.h>
#include <core/mouse.h>

#include "commands/stop.h"

Stop::Stop(Smartmouse2018Robot &robot) : Command("end"), robot(robot), stop_time(10000) {}

Stop::Stop(Smartmouse2018Robot &robot, unsigned long stop_time) : Command("end"), robot(robot), stop_time(stop_time) {}

void Stop::initialize() {
  setTimeout(stop_time);
  robot.setSpeedCps(0, 0);
  digitalWrite(Smartmouse2018Robot::LED_7, 1);
}

bool Stop::isFinished() {
  return isTimedOut();
}

void Stop::end() {
  digitalWrite(Smartmouse2018Robot::LED_7, 0);
}
