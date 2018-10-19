#include <hal/hal.h>

#include "smartmouse_2018_robot.h"
#include "smartmouse_2018_description.h"

Smartmouse2018Robot::Smartmouse2018Robot() : left_encoder(smartmouse_2018_description.left_encoder.cs_pin),
                                             right_encoder(smartmouse_2018_description.right_encoder.cs_pin) {}

void Smartmouse2018Robot::resetToStartPose() {
  reset(); // resets row, col, and dir
  left_encoder.ResetPosition();
  right_encoder.ResetPosition();
  kinematic_controller.reset_col_to(0.5);
  kinematic_controller.reset_row_to(0.5);
  kinematic_controller.reset_yaw_to(dir_to_yaw(dir));
}

double Smartmouse2018Robot::checkVoltage() {
  // 3.2v is max and 2.7v is min
  int a = analogRead(smartmouse_2018_description.battery_pin);
  double voltage = a / std::pow(2, 13) * 3.3;

  if (voltage < 2.7) {
    ssim::print("VOLTAGE [%f] IS TOO LOW. CHARGE THE BATTERY!!!\r\n", voltage);
  } else if (voltage > 3.3) {
    ssim::print("VOLTAGE [%f] IS TOO HIGH. SHE'S GONNA BLOW!!!\r\n", voltage);
  }

  return voltage;
}

ssim::SensorReading Smartmouse2018Robot::checkWalls() {
  return ssim::SensorReading(0, 0);
}
