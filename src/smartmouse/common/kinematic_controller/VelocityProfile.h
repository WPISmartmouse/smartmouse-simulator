#pragma once

#include "mouse.h"
#include "Pose.h"

#include "RobotConfig.h"
#include "VelocityProfileTiming.h"

namespace ssim {

constexpr double kVf_cps = 0.5;

class VelocityProfile {
 public:

  VelocityProfile(GlobalPose start_pose, const VelocityProfileTiming timing);

  std::pair<double, double> drive_straight_wheel_velocities(Mouse &mouse, double t_s);

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

