#pragma once

namespace ssim {

struct LocalPose {
  double to_left = 0, to_back = 0;
  double yaw_from_straight = 0;

  LocalPose() = default;
  LocalPose(double to_left, double to_back);
  LocalPose(double to_left, double to_back, double yaw_from_straight);
};

struct GlobalPose {
  double col = 0, row = 0;
  double yaw = 0;

  GlobalPose() = default;
  GlobalPose(double col, double row);
  GlobalPose(double col, double row, double yaw);
};

struct GlobalState {
  GlobalPose pose;
  double velocity = 0;

  GlobalState() = default;
  GlobalState(GlobalPose pose, double velocity);
};

} // namespace ssim
