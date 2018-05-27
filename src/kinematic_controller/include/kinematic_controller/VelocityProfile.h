#pragma once

#include <core/pose.h>

#include <kinematic_controller/robot.h>
#include <kinematic_controller/RobotConfig.h>
#include <kinematic_controller/VelocityProfileTiming.h>

namespace ssim {

constexpr double kVf_cps = 0.5;

class VelocityProfile {
 public:

  VelocityProfile(GlobalPose start_pose, const VelocityProfileTiming timing);

  std::pair<double, double> drive_straight_wheel_velocities(Robot &robot, double t_s);

  constexpr static double kPWall = 3;
  constexpr static double kPYaw = -1.2;

  double dispError();

  double compute_forward_velocity(double t_s);

 private:
  const VelocityProfileTiming timing;

  double disp = 0;
  GlobalPose start_pose;

};

} // namespace ssim

