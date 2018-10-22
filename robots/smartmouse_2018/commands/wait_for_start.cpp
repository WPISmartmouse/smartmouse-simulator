#include <cstdint>

#include <Arduino.h>

#include "commands/wait_for_start.h"
#include <smartmouse_2018_description.h>

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
      static_cast<double>(robot.left_encoder.getUnsignedRotation()) / TICKS_PER_REVOLUTION;
  speed = percent_speed * MAX_HARDWARE_SPEED_MPS;
  auto right = robot.right_encoder.getUnsignedRotation();
  if (right < TICKS_PER_REVOLUTION / 3) {
    robot.wall_smash = false;
  } else if (right < TICKS_PER_REVOLUTION * 2 / 3) {
    robot.wall_smash = false;
  } else {
    robot.wall_smash = true;
  }


//  digitalWrite(Smartmouse2018Robot::LED_1, static_cast<uint8_t>(GlobalProgramSettings.dead_reckoning));
  digitalWrite(LED_2, static_cast<uint8_t>(robot.wall_smash));

  constexpr uint8_t NUM_LEDS = 6;
  uint8_t light_up_until_led_index = static_cast<uint8_t >(percent_speed * NUM_LEDS);
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    if (i < light_up_until_led_index) {
      digitalWrite(LED_7, 1);
    } else {
      digitalWrite(LED_7 - i, 0);
    }
  }
}

bool WaitForStart::isFinished() {
  if (CommandGroup::isFinished()) {
    return !digitalRead(BUTTON_PIN);
  } else {
    return false;
  }
}

void WaitForStart::end() {
  robot.max_speed_mps = std::max(speed, 0.15);
  robot.max_speed_cups = ssim::toCellUnits(robot.max_speed_mps);
  for (uint8_t i = 0; i < 7; i++) {
    digitalWrite(LED_7 - i, 0);
  }

  robot.resetToStartPose();
  robot.kinematic_controller.enabled = true;
}
