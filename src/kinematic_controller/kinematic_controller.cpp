#include <cmath>
#include <algorithm>
#include <iostream>
#include <tuple>

#include <core/math.h>
#include <core/mouse.h>

#include <kinematic_controller/kinematic_controller.h>

namespace ssim {

const double KinematicController::kDropSafety = 0.8;

GlobalPose KinematicController::getGlobalPose() {
  return current_pose_estimate_cu;
}

LocalPose KinematicController::getLocalPose(const Mouse &mouse) {
  LocalPose local_pose_estimate;
  local_pose_estimate.yaw_from_straight =
      yaw_diff(dir_to_yaw(mouse.getDir()), current_pose_estimate_cu.yaw);
  switch (mouse.getDir()) {
    case Direction::N: {
      local_pose_estimate.to_back = ceil(current_pose_estimate_cu.row) - current_pose_estimate_cu.row;
      local_pose_estimate.to_left = current_pose_estimate_cu.col - floor(current_pose_estimate_cu.col);
      break;
    }
    case Direction::S: {
      local_pose_estimate.to_back = current_pose_estimate_cu.row - floor(current_pose_estimate_cu.row);
      local_pose_estimate.to_left = ceil(current_pose_estimate_cu.col) - current_pose_estimate_cu.col;
      break;
    }
    case Direction::E: {
      local_pose_estimate.to_back = current_pose_estimate_cu.col - floor(current_pose_estimate_cu.col);
      local_pose_estimate.to_left = current_pose_estimate_cu.row - floor(current_pose_estimate_cu.row);
      break;
    }
    case Direction::W: {
      local_pose_estimate.to_back = ceil(current_pose_estimate_cu.col) - current_pose_estimate_cu.col;
      local_pose_estimate.to_left = ceil(current_pose_estimate_cu.row) - current_pose_estimate_cu.row;
      break;
    }
    default:
      exit(-1);
  }
  return local_pose_estimate;
}

std::pair<double, double> KinematicController::getWheelVelocitiesCPS() {
  std::pair<double, double> vels;
  vels.first = radToCU(left_motor.velocity_rps);
  vels.second = radToCU(right_motor.velocity_rps);
  return vels;
};

bool KinematicController::isStopped() {
  return left_motor.isStopped() && left_motor.isStopped();
}

void KinematicController::reset_col_to(double new_col) {
  current_pose_estimate_cu.col = new_col;
}

void KinematicController::reset_row_to(double new_row) {
  current_pose_estimate_cu.row = new_row;
}

void KinematicController::reset_yaw_to(double new_yaw) {
  current_pose_estimate_cu.yaw = new_yaw;
}

void KinematicController::reset_fwd_to_center(const Mouse &mouse) {
  auto dir = mouse.getDir();
  switch (dir) {
    case Direction::N: {
      current_pose_estimate_cu.row = mouse.getRow() + 0.5;
    }
    case Direction::S: {
      current_pose_estimate_cu.row = mouse.getRow() - 0.5;
    }
    case Direction::E: {
      current_pose_estimate_cu.col = mouse.getCol() + 0.5;
    }
    case Direction::W: {
      current_pose_estimate_cu.col = mouse.getCol() - 0.5;
    }
    default: {
      exit(-1);
    }
  }

}

std::pair<double, double>
KinematicController::run(double dt_s, double left_angle_rad, double right_angle_rad, RangeData<double> range_data, const Mouse &mouse) {
  static std::pair<double, double> abstract_forces;

  if (!initialized) {
    initialized = true;
    abstract_forces.first = 0;
    abstract_forces.second = 0;
    return abstract_forces;
  }

  // save the cycle time so other functions can use is
  // this prevents us from passing dt all over the place
  this->dt_s = dt_s;

  if (enabled) {
    if (kinematics_enabled) {
      // equations based on docs/dynamics_model.pdf
      double vl_cu = radToCU(left_motor.velocity_rps);
      double vr_cu = radToCU(right_motor.velocity_rps);

      GlobalPose d_pose_cu;
      d_pose_cu = forwardKinematics(vl_cu, vr_cu, current_pose_estimate_cu.yaw, dt_s);
      current_pose_estimate_cu.col += d_pose_cu.col;
      current_pose_estimate_cu.row += d_pose_cu.row;
      current_pose_estimate_cu.yaw += d_pose_cu.yaw;
      wrapAngleRadInPlace(&current_pose_estimate_cu.yaw);

      // given odometry estimate, improve estimate using sensors, then update odometry estimate to match our best estimate
      // offset_cu is measured from center wall line left of the robot (it's in the local pose frame)
      double est_yaw, offset_m;
      bool no_walls;
      std::tie(est_yaw, offset_m, no_walls) = estimate_pose(range_data, mouse);
      double offset_cu = toCellUnits(offset_m);

      if (enable_sensor_pose_estimate and not no_walls) {
        // if there is a wall in front of us and too close, the front side sensors will reflect off of it and ruin
        // out angle estimate
        if (range_data.front > 0.080) {
          current_pose_estimate_cu.yaw = est_yaw;
        }

        double d_wall_front_cu = 0;
        bool wall_in_front = false;
        // FIXME:
        if (range_data.front < USE_FRONT_WALL_FOR_POSE) {
          double yaw_error = yaw_diff(current_pose_estimate_cu.yaw, dir_to_yaw(mouse.getDir()));
          double d_wall_front_m = cos(yaw_error) * range_data.front + FRONT_ANALOG_X;
          d_wall_front_cu = toCellUnits(d_wall_front_m);
          wall_in_front = true;
        }

        switch (mouse.getDir()) {
          case Direction::N: {
            current_pose_estimate_cu.col = mouse.getCol() + offset_cu;
            if (wall_in_front) {
              current_pose_estimate_cu.row = mouse.getRow() + d_wall_front_cu + HALF_WALL_THICKNESS_CU;
            }
            break;
          }
          case Direction::S: {
            current_pose_estimate_cu.col = mouse.getCol() + 1 - offset_cu;
            if (wall_in_front) {
              current_pose_estimate_cu.row = mouse.getRow() + 1 - d_wall_front_cu - HALF_WALL_THICKNESS_CU;
            }
            break;
          }
          case Direction::E: {
            current_pose_estimate_cu.row = mouse.getRow() + offset_cu;
            if (wall_in_front) {
              current_pose_estimate_cu.col = mouse.getCol() + 1 - d_wall_front_cu - HALF_WALL_THICKNESS_CU;
            }
            break;
          }
          case Direction::W: {
            current_pose_estimate_cu.row = mouse.getRow() + 1 - offset_cu;
            if (wall_in_front) {
              current_pose_estimate_cu.col = mouse.getCol() + d_wall_front_cu + HALF_WALL_THICKNESS_CU;
            }
            break;
          }
          default:
            break;
        }
      }
    }

    // run PID, which will update the velocities of the wheels
    abstract_forces.first = left_motor.runPid(dt_s, left_angle_rad);
    abstract_forces.second = right_motor.runPid(dt_s, right_angle_rad);
  } else {
    abstract_forces.first = 0;
    abstract_forces.second = 0;
  }

  return abstract_forces;
}

GlobalPose KinematicController::forwardKinematics(double vl_cups, double vr_cups, double yaw_rad, double dt) {
  double dcol_cu, drow_cu, dtheta_rad;
  double dyawdt = (vl_cups - vr_cups) / TRACK_WIDTH_CU;
  double R = TRACK_WIDTH_CU * (vr_cups + vl_cups) / (2 * (vl_cups - vr_cups));

  if (fabs(vl_cups) < 1e-5 && fabs(vr_cups) < 1e-5) {
    // this means we're stopped, so ignore it
    dcol_cu = 0;
    drow_cu = 0;
    dtheta_rad = 0;
  } else if (fabs(vl_cups - vr_cups) < 1e-5) {
    // going perfectly straight is a special condition
    dcol_cu = dt * (vr_cups + vl_cups) / 2 * cos(yaw_rad);
    drow_cu = dt * (vr_cups + vl_cups) / 2 * sin(yaw_rad);
    dtheta_rad = 0;
  } else {
    double dtheta_about_icc = dyawdt * dt; //eq 11
    dcol_cu = R * (sin(dtheta_about_icc + yaw_rad) - sin(yaw_rad)); //eq 28
    drow_cu = -R * (cos(dtheta_about_icc + yaw_rad) - cos(yaw_rad)); //eq 29
    dtheta_rad = dyawdt * dt;
  }

  return GlobalPose(dcol_cu, drow_cu, dtheta_rad);
}

std::tuple<double, double, bool> KinematicController::estimate_pose(RangeData<double> range_data, const Mouse &mouse) {
  double d_to_wall_left, yaw_left;
  double d_to_wall_right, yaw_right;
  std::tie(d_to_wall_left, yaw_left) = from_sensors_to_left_wall(BACK_LEFT,
                                                                 FRONT_LEFT,
                                                                 range_data.back_left,
                                                                 range_data.front_left);
  std::tie(d_to_wall_right, yaw_right) = from_sensors_to_right_wall(BACK_RIGHT,
                                                                    FRONT_RIGHT,
                                                                    range_data.back_right,
                                                                    range_data.front_right);

  sense_left_wall = range_data.front_left < SIDE_WALL_THRESHOLD &&
                    range_data.back_left < SIDE_WALL_THRESHOLD;
  sense_right_wall = range_data.front_right < SIDE_WALL_THRESHOLD &&
                     range_data.back_right < SIDE_WALL_THRESHOLD;


  // check for walls that will fall off in the near future (geralds!)
  if (range_data.gerald_left > GERALD_WALL_THRESHOLD) {
    sense_left_wall = false;
  }

  if (range_data.gerald_right > GERALD_WALL_THRESHOLD) {
    sense_right_wall = false;
  }

  bool left_wall_logical = mouse.isWallInDirection(left_of_dir(mouse.getDir()));
  bool right_wall_logical = mouse.isWallInDirection(right_of_dir(mouse.getDir()));

  // consider the "logical" state of walls AND actual range reading
  double yaw = 0;
  double offset = 0;
  bool ignore_walls = true;
  if (sense_left_wall && left_wall_logical) { // wall is on left
    offset = d_to_wall_left + HALF_WALL_THICKNESS_M;
    yaw = dir_to_yaw(mouse.getDir()) + yaw_left;
    ignore_walls = false;
  } else if (sense_right_wall && right_wall_logical) { // wall is on right
    // d_to_wall_right should be positive here for this to be correct
    offset = UNIT_DIST_M - d_to_wall_right - HALF_WALL_THICKNESS_M;
    yaw = dir_to_yaw(mouse.getDir()) + yaw_right;
    ignore_walls = false;
  } else { // we're too far from any walls, use our pose estimation
    ignore_walls = true;
  }

  return std::tuple<double, double, bool>(yaw, offset, ignore_walls);
};

void KinematicController::setAccelerationCpss(double acceleration_cpss) {
  this->acceleration_cellpss = acceleration_cpss;
  left_motor.setAccelerationCpss(acceleration_cpss);
  right_motor.setAccelerationCpss(acceleration_cpss);
}

void KinematicController::setSpeedCps(double left_setpoint_cps,
                                      double right_setpoint_cps) {
  left_motor.setSetpointCps(left_setpoint_cps);
  right_motor.setSetpointCps(right_setpoint_cps);
}

void KinematicController::planTraj(Waypoints waypoints) {
  TrajectoryPlanner planner(waypoints);
  planner.plan();
}

void KinematicController::setParams(double kP, double kI, double kD, double ff_scale, double ff_offset) {
  left_motor.setParams(kP, kI, kD, ff_scale, ff_offset);
  right_motor.setParams(kP, kI, kD, ff_scale, ff_offset);
}

double KinematicController::getCurrentForwardSpeedCUPS() {
  return radToCU((left_motor.velocity_rps + right_motor.velocity_rps) / 2);
}

const std::pair<double, double> from_sensors_to_left_wall(SensorPose s1,
                                                          SensorPose s2,
                                                          double s1_dist_m,
                                                          double s2_dist_m) {
  double dist;
  double yaw;
  std::tie(dist, yaw) = from_sensors_to_wall(s1, s2, s1_dist_m, s2_dist_m);
  return {dist, yaw};
};

const std::pair<double, double> from_sensors_to_right_wall(SensorPose s1,
                                                           SensorPose s2,
                                                           double s1_dist_m,
                                                           double s2_dist_m) {
  double dist;
  double yaw;
  std::tie(dist, yaw) = from_sensors_to_wall(s1, s2, s1_dist_m, s2_dist_m);
  return {dist, yaw};
};

} // namespace ssim
