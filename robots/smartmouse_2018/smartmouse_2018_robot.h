#pragma once

#include <core/msgs.h>
#include <hal/hal.h>
#include <as5048a.h>

#include <kinematic_controller/robot.h>

template<typename T>
struct RangeData {
  T gerald_left;
  T gerald_right;
  T front_left;
  T front_right;
  T back_left;
  T back_right;
  T front;

  std::string to_string() {
    std::stringstream ss;
    ss << back_left << ", "
       << front_left << ","
       << gerald_left << ","
       << front << ","
       << gerald_right << ","
       << front_right << ","
       << back_right << ",";
    return ss.str();
  }
};

class Smartmouse2018Robot : public ssim::Robot {

 public:

  static double checkVoltage();

  Smartmouse2018Robot();

  ~Smartmouse2018Robot() = default;

  ssim::SensorReading checkWalls() override;

  void Setup() override;

  void Step(double dt_s) override;

  ssim::GlobalPose estimate_pose() const override;

  std::tuple<double, double, bool> estimate_local_pose(RangeData<double> range_data) const;

  std::vector<double> ReadSensors();

  void resetToStartPose();

  AS5048A left_encoder;
  AS5048A right_encoder;
  RangeData<double> range_data_m;

  bool sense_left_wall = false;
  bool sense_right_wall = false;
  bool ignoring_left = false;
  bool ignoring_right = false;
  double acceleration_cellpss = 0.0;
  double dt_s = 0.0;
  bool enable_sensor_pose_estimate = true;

  double max_speed_mps = ssim::global_robot_description.min_speed_cups;
  bool wall_smash = false;
  double max_speed_cups = ssim::global_robot_description.max_speed_cups;
};
