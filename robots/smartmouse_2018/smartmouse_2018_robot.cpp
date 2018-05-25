#include "common/hal.h"

#include "smartmouse_2018_robot.h"

void Smartmouse2018Robot::resetToStartPose() {
  reset(); // resets row, col, and dir
  left_encoder.ResetPosition();
  right_encoder.ResetPosition();
  kinematic_controller.reset_col_to(0.5);
  kinematic_controller.reset_row_to(0.5);
  kinematic_controller.reset_yaw_to(dir_to_yaw(dir));
}
