#include <cmath>
#include <tuple>

#include <core/math.h>
#include <core/mouse.h>
#include <hal/hal.h>
#include <hal/util.h>
#include <kinematic_controller/kinematic_controller.h>
#include <kinematic_controller/robot.h>

namespace ssim {

GlobalPose KinematicController::getGlobalPose() const {
  return current_pose_estimate_cu;
}

LocalPose KinematicController::getLocalPose(const Mouse &mouse) {
  LocalPose local_pose_estimate;
  local_pose_estimate.yaw_from_straight =
      yaw_diff(current_pose_estimate_cu.yaw, dir_to_yaw(mouse.getDir()));
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
  vels.first = ssim::radToCU(left_motor.velocity_rps);
  vels.second = ssim::radToCU(right_motor.velocity_rps);
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
      current_pose_estimate_cu.row = mouse.getRow().Double() + 0.5;
      break;
    }
    case Direction::S: {
      current_pose_estimate_cu.row = mouse.getRow().Double() - 0.5;
      break;
    }
    case Direction::E: {
      current_pose_estimate_cu.col = mouse.getCol().Double() + 0.5;
      break;
    }
    case Direction::W: {
      current_pose_estimate_cu.col = mouse.getCol().Double() - 0.5;
      break;
    }
    default: {
      exit(-1);
    }
  }

}

std::pair<double, double>
KinematicController::run(double dt_s, double left_angle_rad, double right_angle_rad, const Robot &robot) {
  static std::pair<double, double> abstract_forces;

  if (!initialized) {
    initialized = true;
    abstract_forces.first = 0;
    abstract_forces.second = 0;
    return abstract_forces;
  }

  if (enabled) {
    if (kinematics_enabled) {
      // equations based on docs/dynamics_model.pdf
      double vl_cu = ssim::radToCU(left_motor.velocity_rps);
      double vr_cu = ssim::radToCU(right_motor.velocity_rps);

      GlobalPose d_pose_cu;
      d_pose_cu = forwardKinematics(vl_cu, vr_cu, current_pose_estimate_cu.yaw, dt_s);
      current_pose_estimate_cu.col += d_pose_cu.col;
      current_pose_estimate_cu.row += d_pose_cu.row;
      current_pose_estimate_cu.yaw += d_pose_cu.yaw;
      wrapAngleRadInPlace(current_pose_estimate_cu.yaw);

      // given odometry estimate, improve estimate using sensors, then update odometry estimate to match our best estimate
      // offset_cu is measured from center wall line left of the robot (it's in the local pose frame)
      current_pose_estimate_cu = robot.estimate_pose();
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
  double dyawdt = (vl_cups - vr_cups) / robot_description.track_width_cu;
  double R = robot_description.track_width_cu * (vr_cups + vl_cups) / (2 * (vl_cups - vr_cups));

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


void KinematicController::setAccelerationCpss(double acceleration_cpss) {
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
  return ssim::radToCU((left_motor.velocity_rps + right_motor.velocity_rps) / 2);
}

const std::pair<double, double> from_sensors_to_left_wall(XYTheta s1, XYTheta s2, double s1_dist_m, double s2_dist_m) {
  double dist;
  double yaw;
  std::tie(dist, yaw) = from_sensors_to_wall(s1, s2, s1_dist_m, s2_dist_m);
  return {dist, yaw};
};

const std::pair<double, double> from_sensors_to_right_wall(XYTheta s1, XYTheta s2, double s1_dist_m, double s2_dist_m) {
  double dist;
  double yaw;
  std::tie(dist, yaw) = from_sensors_to_wall(s1, s2, s1_dist_m, s2_dist_m);
  return {dist, yaw};
};

} // namespace ssim
