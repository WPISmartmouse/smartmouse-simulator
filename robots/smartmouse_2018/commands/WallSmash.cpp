#include "hal.h"

#include "commands/WallSmash.h"

WallSmash::WallSmash(Smartmouse2018Robot &robot) : Command("WallSmash"), robot(robot) {}

void WallSmash::initialize() {
  setTimeout(1000);
  ssim::digitalWrite(Smartmouse2018Robot::LED_5, 1);
}

void WallSmash::execute() {
  robot.setSpeedCps(1, 1);
}

bool WallSmash::isFinished() {
  return isTimedOut();
}

void WallSmash::end() {
  ssim::digitalWrite(Smartmouse2018Robot::LED_5, 0);
}
