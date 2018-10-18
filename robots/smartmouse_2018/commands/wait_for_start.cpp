#include <cstdint>

#include <Arduino.h>
#include <smartmouse_2018_description.h>

#include "commands/wait_for_start.h"

WaitForStart::WaitForStart(Smartmouse2018Robot &robot) : CommandGroup("wait_calibrate"), robot(robot), speed(0) {}

void WaitForStart::initialize() {
  robot.left_encoder.ResetPosition();
  robot.right_encoder.ResetPosition();
  robot.kinematic_controller.enabled = false;
}

void WaitForStart::execute() {
  CommandGroup::execute();

  // Set the max speed of the robot based on the left wheel
  double percent_speed =
      static_cast<double>(robot.left_encoder.getUnsignedRotation()) / ssim::TICKS_PER_REVOLUTION;
  speed = percent_speed * ssim::MAX_HARDWARE_SPEED_MPS;
  auto right = robot.right_encoder.getUnsignedRotation();
  if (right < ssim::TICKS_PER_REVOLUTION / 3) {
    ssim::WALL_SMASH = false;
  } else if (right < ssim::TICKS_PER_REVOLUTION * 2 / 3) {
    ssim::WALL_SMASH = false;
  } else {
    ssim::WALL_SMASH = true;
  }


//  digitalWrite(Smartmouse2018Robot::LED_1, static_cast<uint8_t>(GlobalProgramSettings.dead_reckoning));
  digitalWrite(smartmouse_2018_description.leds[2].pin, static_cast<uint8_t>(ssim::WALL_SMASH));

  constexpr uint8_t NUM_LEDS = 6;
  uint8_t light_up_until_led_index = static_cast<uint8_t >(percent_speed * NUM_LEDS);
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    if (i < light_up_until_led_index) {
      digitalWrite(smartmouse_2018_description.leds[7].pin, 1);
    } else {
      digitalWrite(smartmouse_2018_description.leds[7].pin - i, 0);
    }
  }
}

bool WaitForStart::isFinished() {
  if (CommandGroup::isFinished()) {
    return !digitalRead(smartmouse_2018_description.button_pin);
  } else {
    return false;
  }
}

void WaitForStart::end() {
  // FIXME:
  ssim::MAX_SPEED_MPS = std::max(speed, 0.15);
//  GlobalProgramSettings.dead_reckoning = false;
//  smartmouse::kc::MAX_SPEED_MPS = 0.15;
  ssim::MAX_SPEED_CUPS = ssim::toCellUnits(ssim::MAX_SPEED_MPS);
  for (uint8_t i = 0; i < 7; i++) {
    digitalWrite(smartmouse_2018_description.leds[7].pin - i, 0);
  }

  robot.resetToStartPose();
  robot.kinematic_controller.enabled = true;
}
