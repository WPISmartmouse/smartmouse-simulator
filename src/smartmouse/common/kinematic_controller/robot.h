#pragma once

#include "mouse.h"

#include "KinematicController.h"

namespace ssim {

class Robot : public Mouse {

public:

  static double dispToNextEdge(Robot &robot);

  static double dispToNthEdge(Robot &robot, unsigned int n);

  static double fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose);

  static double fwdDispToCenter(Robot &robot);

  static GlobalPose poseOfToNthEdge(Robot &robot, unsigned int n);

  static double sidewaysDispToCenter(Robot &robot);

  KinematicController kinematic_controller;

  std::pair<double, double> getWheelVelocitiesCPS();

  void setSpeedCps(double left_speed_cps, double right_speed_cps);

  ssim::GlobalPose getGlobalPose();

  ssim::LocalPose getLocalPose();

};

} // namespace ssim
