#include <Arduino.h>
#include <kinematic_controller/kinematic_controller.h>

#include <commands/forward.h>
#include "smartmouse_2018_description.h"

Forward::Forward(Smartmouse2018Robot &robot) : Command("Forward"), robot(robot), profile(nullptr) {}

void Forward::initialize() {
  setTimeout(3000);
  start = robot.getGlobalPose();
  const double goal_disp = ssim::Robot::dispToNextEdge(robot);
  const double v0 = robot.kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = ssim::kVf_cps;
  profile = new ssim::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(smartmouse_2018_description.leds[1].pin, 1);
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
  digitalWrite(smartmouse_2018_description.leds[1].pin, 0);
}

