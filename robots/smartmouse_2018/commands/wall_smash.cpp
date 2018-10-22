#include <Arduino.h>

#include "commands/wall_smash.h"
#include "smartmouse_2018_description.h"

WallSmash::WallSmash(Smartmouse2018Robot &robot) : Command("WallSmash"), robot(robot) {}

void WallSmash::initialize() {
  setTimeout(1000);
  digitalWrite(LED_5, 1);
}

void WallSmash::execute() {
  robot.setSpeedCps(1, 1);
}

bool WallSmash::isFinished() {
  return isTimedOut();
}

void WallSmash::end() {
  digitalWrite(LED_5, 0);
}

