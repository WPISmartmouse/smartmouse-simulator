// TODO: finish unit conversion

#include "commands/ArcTurn.h"

#include "math_util.h"
#include "mouse.h"


ArcTurn::ArcTurn(Smartmouse2018Robot &robot, ssim::Direction dir) : Command("ArcTurn"), robot(robot), dir(dir) {}

void ArcTurn::initialize() {
  curPose = robot.getGlobalPose();
  curCol = robot.getCol();
  curRow = robot.getRow();
  curDir = robot.getDir();

  startPose = curPose;
  startCol = curCol;
  startRow = curRow;

  goalYaw = dir_to_yaw(dir);

  // determine vtc of arc
  vtc_x = curCol * ssim::UNIT_DIST_M + ssim::HALF_UNIT_DIST;
  vtc_y = curRow * ssim::UNIT_DIST_M + ssim::HALF_UNIT_DIST;
  switch (curDir) {
    case ssim::Direction::N: {
      vtc_y += ssim::HALF_UNIT_DIST;
      if (dir == ssim::Direction::E) {
        vtc_x += ssim::HALF_UNIT_DIST;
      } else {
        vtc_x -= ssim::HALF_UNIT_DIST;
      }
      break;
    }
    case ssim::Direction::E: {
      vtc_x -= ssim::HALF_UNIT_DIST;
      if (dir == ssim::Direction::S) {
        vtc_y += ssim::HALF_UNIT_DIST;
      } else {
        vtc_y -= ssim::HALF_UNIT_DIST;
      }
      break;
    }
    case ssim::Direction::S: {
      vtc_y -= ssim::HALF_UNIT_DIST;
      if (dir == ssim::Direction::W) {
        vtc_x -= ssim::HALF_UNIT_DIST;
      } else {
        vtc_x += ssim::HALF_UNIT_DIST;
      }
      break;
    }
    case ssim::Direction::W: {
      vtc_x += ssim::HALF_UNIT_DIST;
      if (dir == ssim::Direction::N) {
        vtc_y -= ssim::HALF_UNIT_DIST;
      } else {
        vtc_y += ssim::HALF_UNIT_DIST;
      }
      break;
    }
    default:
      exit(0);
  }
}

void ArcTurn::execute() {

  double cur_x = fabs(curPose.row - vtc_x);
  double cur_y = fabs(curPose.col - vtc_y);

  double dAngle;
  if ((dir == ssim::Direction::N) || (dir == ssim::Direction::S)) {
    dAngle = atanf(fabs(cur_x / cur_y));
  } else {
    dAngle = atanf(fabs(cur_y / cur_x));
  }

  double ang_error =
      ssim::yaw_diff(dAngle, fabs(ssim::yaw_diff(curPose.yaw, dir_to_yaw(curDir))));
  double arc_error = (ssim::HALF_UNIT_DIST / pose_dist(curPose, vtc_x, vtc_y)) - 1;
  double corr = (ang_error * ang_weight) + (arc_error * arc_weight);

  double fast_speed = FAST_ARC_SPEED * ((corr * -kp_turn) + 1);
  double slow_speed = SLOW_ARC_SPEED * ((corr * kp_turn) + 1);

  if (right_of_dir(curDir) == dir) {
    robot.setSpeedCps(fast_speed, slow_speed);
  } else {
    robot.setSpeedCps(slow_speed, fast_speed);
  }
}

bool ArcTurn::isFinished() {
  curPose = robot.getGlobalPose();
  curCol = robot.getCol();
  curRow = robot.getRow();
  curDir = robot.getDir();

  dYaw = ssim::yaw_diff(curPose.yaw, goalYaw);
  return (curCol != startCol) || (curRow != startRow);
}

void ArcTurn::end() {
  robot.internalTurnToFace(dir);
}

double ArcTurn::pose_dist(ssim::GlobalPose pose, double x, double y) {
  return sqrtf(powf(pose.col - x, 2) + powf(pose.row - y, 2));
}
