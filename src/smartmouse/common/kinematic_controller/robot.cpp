#include "robot.h"

namespace ssim {

double Robot::fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose) {
  switch (dir) {
    case Direction::N:
      return start_pose.row - current_pose.row;
    case Direction::E:
      return current_pose.col - start_pose.col;
    case Direction::S:
      return current_pose.row - start_pose.row;
    case Direction::W:
      return start_pose.col - current_pose.col;
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

double Robot::dispToNextEdge(Robot &robot) {
  GlobalPose current_pose = robot.getGlobalPose();
  Direction dir = robot.getDir();

  switch (dir) {
    case Direction::N: {
      return current_pose.row - robot.getRow();
    }
    case Direction::S: {
      return robot.getRow() + 1 - current_pose.row;
    }
    case Direction::E: {
      return robot.getCol() + 1 - current_pose.col;
    }
    case Direction::W: {
      return current_pose.col - robot.getCol();
    }
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

double Robot::dispToNthEdge(Robot &robot, unsigned int n) {
  // give the displacement to the nth edge like above...
  return dispToNextEdge(robot) + (n - 1);
}

GlobalPose Robot::poseOfToNthEdge(Robot &robot, unsigned int n) {
  // give the displacement to the nth edge like above...
  double disp = dispToNthEdge(robot, n);
  GlobalPose pose = robot.getGlobalPose();
  pose.col += cos(pose.yaw) * disp;
  pose.row += sin(pose.yaw) * disp;

  return pose;
}

double Robot::sidewaysDispToCenter(Robot &robot) {
  // local y is sideways, increasing from left to right
  return robot.getLocalPose().to_left - 0.5;
}

double Robot::fwdDispToCenter(Robot &robot) {
  switch (robot.getDir()) {
    case Direction::N: {
      return robot.getGlobalPose().row - robot.getRow() + 0.5;
    }
    case Direction::S: {
      return (robot.getRow() + 0.5) - robot.getGlobalPose().row;
    }
    case Direction::E: {
      return robot.getCol() + 0.5 - robot.getGlobalPose().col;
    }
    case Direction::W: {
      return robot.getGlobalPose().col - (robot.getCol() + 0.5);
    }
    default:
      exit(-1);
  }
}


void Robot::setSpeedCps(double left_speed_cps, double right_speed_cps) {
  kinematic_controller.setSpeedCps(left_speed_cps, right_speed_cps);
}

ssim::GlobalPose Robot::getGlobalPose() {
  return kinematic_controller.getGlobalPose();
}

ssim::LocalPose Robot::getLocalPose() {
  return kinematic_controller.getLocalPose();
}

std::pair<double, double> Robot::getWheelVelocitiesCPS() {
  return kinematic_controller.getWheelVelocitiesCPS();
}

};
