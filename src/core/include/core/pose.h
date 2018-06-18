#pragma once

namespace ssim {

struct LocalPose {
  double to_left, to_back;
  double yaw_from_straight;

  LocalPose() = default;
  LocalPose(double to_left, double to_back);
  LocalPose(double to_left, double to_back, double yaw_from_straight);
};

struct GlobalPose {
  double col, row;
  double yaw;

  GlobalPose() = default;
  GlobalPose(double col, double row);
  GlobalPose(double col, double row, double yaw);
};

struct GlobalState {
  GlobalPose pose;
  double velocity;

  GlobalState() = default;
  GlobalState(GlobalPose pose, double velocity);
};

} // namespace ssim
