#pragma once

#include <core/mouse.h>

#include <kinematic_controller/kinematic_controller.h>

namespace ssim {

/// Class to combine Mouse (inherited) and KinematicController (member)
class Robot : public Mouse {

public:

  static double dispToNextEdge(Robot &robot);

  static double dispToNthEdge(Robot &robot, unsigned int n);

  static double fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose);

  static double fwdDispToCenter(Robot &robot);

  static GlobalPose poseOfToNthEdge(Robot &robot, unsigned int n);

  static double sidewaysDispToCenter(Robot &robot);

  Robot() = default;

  std::pair<double, double> getWheelVelocitiesCPS();

  void setSpeedCps(double left_speed_cps, double right_speed_cps);

  void reset_fwd_to_center();

  ssim::GlobalPose getGlobalPose();

  ssim::LocalPose getLocalPose();

  KinematicController kinematic_controller;

};

} // namespace ssim
