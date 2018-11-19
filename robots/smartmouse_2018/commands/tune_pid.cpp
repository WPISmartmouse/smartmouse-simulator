#include <hal/util.h>
#include <Arduino.h>
#include <core/mouse.h>

#include "commands/tune_pid.h"
#include "tune_pid.h"

#include <smartmouse_2018_description.h>

TunePID::TunePID(Smartmouse2018Robot &robot) : Command("tune pid"), robot_(robot) {}

void TunePID::execute() {
  auto const &msg = Serial1.read<ssim::Debug>();
  robot_.kinematic_controller.setParams(msg.kP, msg.kI, msg.kD, msg.kFFScale, msg.kFFOffset);
  robot_.setSpeedCps(msg.left_setpoint_cups, msg.right_setpoint_cups);
}

bool TunePID::isFinished() {
  return !digitalRead(BUTTON_PIN);
}

void TunePID::end() {
  robot_.setSpeedCps(0, 0);
}

