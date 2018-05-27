#include <Arduino.h>

#include <kinematic_controller/robot.h>

#include "commands/Backup.h"

Backup::Backup(Smartmouse2018Robot &robot) : Command("Backup"), robot(robot) {}

void Backup::initialize() {
  setTimeout(300);
  digitalWrite(Smartmouse2018Robot::LED_6, 1);
}

void Backup::execute() {
  robot.setSpeedCps(-0.75, -0.75);
}

bool Backup::isFinished() {
  return isTimedOut();
}

void Backup::end() {
  robot.kinematic_controller.reset_fwd_to_center();
  digitalWrite(Smartmouse2018Robot::LED_6, 0);
}

