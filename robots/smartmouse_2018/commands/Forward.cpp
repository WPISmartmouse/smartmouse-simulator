#include <Arduino.h>
#include <kinematic_controller/KinematicController.h>

#include "commands/Forward.h"

Forward::Forward(Smartmouse2018Robot &robot) : Command("Forward"), robot(robot), profile(nullptr) {}

void Forward::initialize() {
  setTimeout(3000);
  start = robot.getGlobalPose();
  const double goal_disp = ssim::Robot::dispToNextEdge(robot);
  const double v0 = robot.kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = ssim::kVf_cps;
  profile = new ssim::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(Smartmouse2018Robot::LED_1, 1);
}

void Forward::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(robot, t_s);
  robot.setSpeedCps(l, r);
}

bool Forward::isFinished() {
  return profile->dispError() <= 0 or isTimedOut();
}

void Forward::end() {
  digitalWrite(Smartmouse2018Robot::LED_1, 0);
}

