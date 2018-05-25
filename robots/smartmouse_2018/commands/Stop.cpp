#include "hal.h"
#include "mouse.h"

#include "commands/Stop.h"

Stop::Stop(Smartmouse2018Robot &robot) : Command("end"), robot(robot), stop_time(10000) {}

Stop::Stop(Smartmouse2018Robot &robot, unsigned long stop_time) : Command("end"), robot(robot), stop_time(stop_time) {}

void Stop::initialize() {
  setTimeout(stop_time);
  robot.setSpeedCps(0, 0);
  ssim::digitalWrite(Smartmouse2018Robot::LED_7, 1);
}

bool Stop::isFinished() {
  return isTimedOut();
}

void Stop::end() {
  ssim::digitalWrite(Smartmouse2018Robot::LED_7, 0);
}
