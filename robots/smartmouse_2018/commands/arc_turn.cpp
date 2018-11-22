// TODO: finish unit conversion

#include "commands/arc_turn.h"

#include <core/math.h>
#include <core/mouse.h>


ArcTurn::ArcTurn(Smartmouse2018Robot &robot, ssim::Direction dir) : Command("ArcTurn"), robot(robot), dir(dir) {}

void ArcTurn::initialize() {
  curPose = robot.getGlobalPose();
  current_row_col = robot.getRowCol();
  curDir = robot.getDir();

  startPose = curPose;
  start = current_row_col;

  goalYaw = dir_to_yaw(dir);

  // determine vtc of arc
  vtc_x = current_row_col.row.Double() * ssim::UNIT_DIST_M + ssim::HALF_UNIT_DIST;
  vtc_y = current_row_col.col.Double() * ssim::UNIT_DIST_M + ssim::HALF_UNIT_DIST;
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

  slow_arc_speed = speed_scale * (robot.max_speed_cups / (ssim::HALF_UNIT_DIST + (TRACK_WIDTH_CU / 2))) *
                   (ssim::HALF_UNIT_DIST - (TRACK_WIDTH_CU / 2));
  fast_arc_speed = speed_scale * (robot.max_speed_cups);
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
      ssim::yaw_diff(fabs(ssim::yaw_diff(dir_to_yaw(curDir), curPose.yaw)), dAngle);
  double arc_error = (ssim::HALF_UNIT_DIST / pose_dist(curPose, vtc_x, vtc_y)) - 1;
  double corr = (ang_error * ang_weight) + (arc_error * arc_weight);

  double fast_speed = fast_arc_speed * ((corr * -kp_turn) + 1);
  double slow_speed = slow_arc_speed * ((corr * kp_turn) + 1);

  if (right_of_dir(curDir) == dir) {
    robot.setSpeedCps(fast_speed, slow_speed);
  } else {
    robot.setSpeedCps(slow_speed, fast_speed);
  }
}

bool ArcTurn::isFinished() {
  curPose = robot.getGlobalPose();
  current_row_col = robot.getRowCol();
  curDir = robot.getDir();

  dYaw = ssim::yaw_diff(goalYaw, curPose.yaw);
  return current_row_col != start;
}

void ArcTurn::end() {
  robot.internalTurnToFace(dir);
}

double ArcTurn::pose_dist(ssim::GlobalPose pose, double x, double y) {
  return sqrtf(powf(pose.col - x, 2) + powf(pose.row - y, 2));
}
