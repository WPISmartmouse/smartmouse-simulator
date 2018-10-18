#include <Arduino.h>
#include "commands/forward_n.h"

#include "smartmouse_2018_description.h"

ForwardN::ForwardN(Smartmouse2018Robot &robot, unsigned int n) : Command("Forward"), robot(robot), n(n),
                                                                 profile(nullptr) {}

void ForwardN::initialize() {
  start = robot.getGlobalPose();
  const double goal_disp = ssim::Robot::dispToNthEdge(robot, n);
  const double v0 = robot.kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = ssim::kVf_cps;
  profile = new ssim::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(smartmouse_2018_description.leds[4].pin, 1);
}

void ForwardN::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(robot, t_s);
  robot.setSpeedCps(l, r);
}

bool ForwardN::isFinished() {
  return profile->dispError() <= 0;
}

void ForwardN::end() {
  digitalWrite(smartmouse_2018_description.leds[4].pin, 0);
}

