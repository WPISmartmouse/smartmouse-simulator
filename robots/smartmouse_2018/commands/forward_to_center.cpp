#include <Arduino.h>
#include "commands/forward_to_center.h"

#include "smartmouse_2018_description.h"

ForwardToCenter::ForwardToCenter(Smartmouse2018Robot &robot) : Command("FwdToCenter"), robot(robot), profile(nullptr) {}

void ForwardToCenter::initialize() {
  setTimeout(5000);
  start = robot.getGlobalPose();
  const double goal_disp = ssim::Robot::fwdDispToCenter(robot);
  const double v0 = robot.kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = 0.0;
  profile = new ssim::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(LED_3, 1);
}

void ForwardToCenter::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(robot, t_s);

  auto disp_error = profile->dispError();
  if (l < .01 and r < .01 and 0 < disp_error and disp_error < 0.25) {
    l += kPFwd * disp_error;
    r += kPFwd * disp_error;
  }

  robot.setSpeedCps(l, r);
}

bool ForwardToCenter::isFinished() {
  return profile->dispError() <= 0.001 or isTimedOut();
}

void ForwardToCenter::end() {
  digitalWrite(LED_3, 0);
}
