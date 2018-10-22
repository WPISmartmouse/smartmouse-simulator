#include <tuple>

#include <Arduino.h>
#include <core/math.h>

#include "commands/turn_in_place.h"
#include "smartmouse_2018_description.h"

const double TurnInPlace::kP = 0.9;

TurnInPlace::TurnInPlace(Smartmouse2018Robot &robot, ssim::Direction dir) : Command("RealTurnInPlace"), robot(robot),
                                                                            left_turn(false), dir(dir), goal_yaw(0),
                                                                            yaw_error(0) {}

void TurnInPlace::initialize() {
  robot.enable_sensor_pose_estimate = false;
  digitalWrite(LED_2, 1);
  setTimeout(5000);
  goal_yaw = dir_to_yaw(dir);
  const auto start = robot.getGlobalPose();
  yaw_error = ssim::yaw_diff(start.yaw, goal_yaw);
  const double turn_arc_length = fabs(yaw_error) * TRACK_WIDTH_CU / 2;
  left_turn = (yaw_error < 0);
  profile = new ssim::VelocityProfile(start, {turn_arc_length, 0, 0});
}

void TurnInPlace::execute() {
  double t_s = getTime() / 1000.0;
  double fwd_v = profile->compute_forward_velocity(t_s);
  double vl = 0, vr = 0;
  if (left_turn) {
    vl = -fwd_v;
    vr = fwd_v;
  } else {
    vl = fwd_v;
    vr = -fwd_v;
  }

  if (vl < .01 and vr < .01 and fabs(yaw_error) > ROT_TOLERANCE) {
    vl += kP * yaw_error;
    vr -= kP * yaw_error;
  }

  robot.setSpeedCps(vl, vr);

  // when we get close to aligned, there might be a wall we can use to better estimate our angle
  // this allows us to use that
  if (fabs(yaw_error) < ROT_TOLERANCE * 4 && robot.enable_sensor_pose_estimate) {
    robot.enable_sensor_pose_estimate = true;
    // FIXME: this is kind of a hack. It's needed because DriveStraight checks dir in order to compute
    // FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
    robot.internalTurnToFace(dir);
  }
}

bool TurnInPlace::isFinished() {
  double current_yaw = robot.getGlobalPose().yaw;
  yaw_error = ssim::yaw_diff(current_yaw, goal_yaw);
  double vl_cps, vr_cps;
  std::tie(vl_cps, vr_cps) = robot.getWheelVelocitiesCPS();
  return isTimedOut()
         || ((fabs(yaw_error) < ROT_TOLERANCE) && fabs(vl_cps) <= MIN_SPEED_CUPS
             && fabs(vr_cps) < MIN_SPEED_CUPS);
}

void TurnInPlace::end() {
  robot.enable_sensor_pose_estimate = true;
  digitalWrite(LED_2, 0);
  robot.internalTurnToFace(dir);
}

