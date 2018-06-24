#pragma once

#include <tuple>
#include <utility>

#include <core/mouse.h>
#include <core/pose.h>

#include <kinematic_controller/regulated_motor.h>
#include <kinematic_controller/robot_config.h>
#include <kinematic_controller/trajectory_planner.h>

namespace ssim {

constexpr std::pair<double, double> from_sensors_to_wall(SensorPose s1,
                                                         SensorPose s2,
                                                         double s1_dist_m,
                                                         double s2_dist_m) {
  const double d1x = cos(s1.angle) * s1_dist_m + s1.x;
  const double d1y = sin(s1.angle) * s1_dist_m + s1.y;
  const double d2x = cos(s2.angle) * s2_dist_m + s2.x;
  const double d2y = sin(s2.angle) * s2_dist_m + s2.y;
  const double yaw = -atan2(d2y - d1y, d2x - d1x);
  // distance from point to line:
  // https://en.wikipedia.org/w/index.php?title=Distance_from_a_point_to_a_line&oldid=828284744
  // where x_0 and y_0 are equal to 0
  // the fabs is required because the numerator represents an area
  const double dist = fabs(d2x * d1y - d2y * d1x) / sqrt(pow(d2y - d1y, 2) + pow(d2x - d1x, 2));
  return {dist, yaw};
};

const std::pair<double, double> from_sensors_to_left_wall(SensorPose s1,
                                                          SensorPose s2,
                                                          double s1_dist_m,
                                                          double s2_dist_m);

const std::pair<double, double> from_sensors_to_right_wall(SensorPose s1,
                                                           SensorPose s2,
                                                           double s1_dist_m,
                                                           double s2_dist_m);

class KinematicController {
public:
  static const double kPWall;
  static const double kDWall;
  static const double kPYaw;

  static GlobalPose forwardKinematics(double vl, double vr, double yaw, double dt);

  KinematicController() = default;

  void planTraj(Waypoints waypoints);

  std::tuple<double, double, bool> estimate_pose(RangeData<double> range_data, const Mouse &mouse);

  GlobalPose getGlobalPose();

  LocalPose getLocalPose(const Mouse &mouse);

  std::pair<double, double> getWheelVelocitiesCPS();

  bool isStopped();

  void reset_fwd_to_center(const Mouse &mouse);

  void reset_col_to(double new_col);

  void reset_row_to(double new_row);

  void reset_yaw_to(double new_yaw);

  std::pair<double, double>
  run(double dt_s, double left_angle_rad, double right_angle_rad, RangeData<double> range_data, const Mouse &mouse);

  void setAccelerationCpss(double acceleration_mpss);

  void setSpeedCps(double left_setpoint_mps, double right_setpoint_mps);

  void setParams(double kP, double kI, double kD, double ff_scale, double ff_offset);

  double getCurrentForwardSpeedCUPS();

  RegulatedMotor left_motor;
  RegulatedMotor right_motor;

  bool enable_sensor_pose_estimate = true;
  bool enabled = true;
  bool kinematics_enabled = true;
  bool sense_left_wall = false;
  bool sense_right_wall = false;

private:
  bool initialized = false;
  bool ignoring_left = false;
  bool ignoring_right = false;

  GlobalPose current_pose_estimate_cu;
  static const double kDropSafety;
  double acceleration_cellpss;
  double dt_s;
};

} // namespace ssim
