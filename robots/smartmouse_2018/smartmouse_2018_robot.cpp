#include <hal/util.h>
#include <hal/hal.h>

#include "smartmouse_2018_robot.h"
#include "smartmouse_2018_description.h"


Smartmouse2018Robot::Smartmouse2018Robot() : left_encoder(LEFT_ENCODER_CS),
                                             right_encoder(RIGHT_ENCODER_CS) {}

std::tuple<double, double, bool> Smartmouse2018Robot::estimate_local_pose(RangeData<double> range_data) const {
  const auto[d_to_wall_left, yaw_left] = ssim::from_sensors_to_left_wall(BACK_LEFT_SENSOR.p,
                                                                         FRONT_LEFT_SENSOR.p,
                                                                         range_data.back_left,
                                                                         range_data.front_left);
  const auto[d_to_wall_right, yaw_right] = ssim::from_sensors_to_right_wall(BACK_RIGHT_SENSOR.p,
                                                                            FRONT_RIGHT_SENSOR.p,
                                                                            range_data.back_right,
                                                                            range_data.front_right);

  auto sense_left_wall = range_data.front_left < SIDE_WALL_THRESHOLD &&
                         range_data.back_left < SIDE_WALL_THRESHOLD;
  auto sense_right_wall = range_data.front_right < SIDE_WALL_THRESHOLD &&
                          range_data.back_right < SIDE_WALL_THRESHOLD;


  // check for walls that will fall off in the near future (geralds!)
  if (range_data.gerald_left > GERALD_WALL_THRESHOLD) {
    sense_left_wall = false;
  }

  if (range_data.gerald_right > GERALD_WALL_THRESHOLD) {
    sense_right_wall = false;
  }

  bool left_wall_logical = isWallInDirection(left_of_dir(getDir()));
  bool right_wall_logical = isWallInDirection(right_of_dir(getDir()));

  // consider the "logical" state of walls AND actual range reading
  double yaw = 0;
  double offset = 0;
  bool ignore_walls = true;
  if (sense_left_wall && left_wall_logical) { // wall is on left
    offset = d_to_wall_left + ssim::HALF_WALL_THICKNESS_M;
    yaw = dir_to_yaw(getDir()) + yaw_left;
    ignore_walls = false;
  } else if (sense_right_wall && right_wall_logical) { // wall is on right
    // d_to_wall_right should be positive here for this to be correct
    offset = ssim::UNIT_DIST_M - d_to_wall_right - ssim::HALF_WALL_THICKNESS_M;
    yaw = dir_to_yaw(getDir()) + yaw_right;
    ignore_walls = false;
  } else { // we're too far from any walls, use our pose estimation
    ignore_walls = true;
  }

  return std::tuple<double, double, bool>(yaw, offset, ignore_walls);
};

ssim::GlobalPose Smartmouse2018Robot::estimate_pose() const {
  const auto[est_yaw, offset_m, no_walls] = estimate_local_pose(range_data_m);
  ssim::GlobalPose new_pose_estimate_cu = kinematic_controller.getGlobalPose();
  double offset_cu = ssim::toCellUnits(offset_m);

  if (enable_sensor_pose_estimate and not no_walls) {
    // if there is a wall in front of us and too close, the front side sensors will reflect off of it and ruin
    // out angle estimate
    if (range_data_m.front > 0.080) {
      new_pose_estimate_cu.yaw = est_yaw;
    }

    double d_wall_front_cu = 0;
    bool wall_in_front = false;
    // FIXME:
    if (range_data_m.front < USE_FRONT_WALL_FOR_POSE) {
      double yaw_error = ssim::yaw_diff(new_pose_estimate_cu.yaw, dir_to_yaw(getDir()));
      double d_wall_front_m = cos(yaw_error) * range_data_m.front + FRONT_SENSOR.p.x;
      d_wall_front_cu = ssim::toCellUnits(d_wall_front_m);
      wall_in_front = true;
    }

    switch (getDir()) {
      case ssim::Direction::N: {
        new_pose_estimate_cu.col = getCol() + offset_cu;
        if (wall_in_front) {
          new_pose_estimate_cu.row = getRow() + d_wall_front_cu + ssim::HALF_WALL_THICKNESS_CU;
        }
        break;
      }
      case ssim::Direction::S: {
        new_pose_estimate_cu.col = getCol() + 1 - offset_cu;
        if (wall_in_front) {
          new_pose_estimate_cu.row = getRow() + 1 - d_wall_front_cu - ssim::HALF_WALL_THICKNESS_CU;
        }
        break;
      }
      case ssim::Direction::E: {
        new_pose_estimate_cu.row = getRow() + offset_cu;
        if (wall_in_front) {
          new_pose_estimate_cu.col = getCol() + 1 - d_wall_front_cu - ssim::HALF_WALL_THICKNESS_CU;
        }
        break;
      }
      case ssim::Direction::W: {
        new_pose_estimate_cu.row = getRow() + 1 - offset_cu;
        if (wall_in_front) {
          new_pose_estimate_cu.col = getCol() + d_wall_front_cu + ssim::HALF_WALL_THICKNESS_CU;
        }
        break;
      }
      default:
        break;
    }
  }

  return new_pose_estimate_cu;
}

void Smartmouse2018Robot::resetToStartPose() {
  reset(); // resets row, col, and dir
  left_encoder.ResetPosition();
  right_encoder.ResetPosition();
  kinematic_controller.reset_col_to(0.5);
  kinematic_controller.reset_row_to(0.5);
  kinematic_controller.reset_yaw_to(dir_to_yaw(dir));
}

double Smartmouse2018Robot::checkVoltage() {
  // 3.2v is max and 2.7v is min
  int a = analogRead(BATTERY_ANALOG_PIN);
  double voltage = a / std::pow(2, 13) * 3.3;

  if (voltage < 2.7) {
    ssim::print("VOLTAGE [%f] IS TOO LOW. CHARGE THE BATTERY!!!\r\n", voltage);
  } else if (voltage > 3.3) {
    ssim::print("VOLTAGE [%f] IS TOO HIGH. SHE'S GONNA BLOW!!!\r\n", voltage);
  }

  return voltage;
}

ssim::SensorReading Smartmouse2018Robot::checkWalls() {
  return ssim::SensorReading(0, 0);
}

void Smartmouse2018Robot::Setup() {
  pinMode(LED_1, OUTPUT);
}

void Smartmouse2018Robot::Step(double dt_s) {
  const auto left_angle_rad = left_encoder.getRotation() * 2 * M_PI / ssim::global_robot_description.left_encoder.ticks;
  const auto right_angle_rad = right_encoder.getRotation() * 2 * M_PI / ssim::global_robot_description.right_encoder.ticks;

  const auto range_data_m = ReadSensors();

  const auto[abstract_left_force, abstract_right_force] = kinematic_controller.run(dt_s, left_angle_rad,
                                                                                   right_angle_rad,
                                                                                   *this);

  // THIS IS SUPER IMPORTANT!
  // update row/col information
  row = static_cast<unsigned int>(kinematic_controller.getGlobalPose().row);
  col = static_cast<unsigned int>(kinematic_controller.getGlobalPose().col);

  if (abstract_left_force < 0) {
    analogWrite(MOTOR_LEFT_A1, 0);
    analogWrite(MOTOR_LEFT_A2, static_cast<unsigned int>(-abstract_left_force));
  } else {
    analogWrite(MOTOR_LEFT_A1, static_cast<unsigned int>(abstract_left_force));
    analogWrite(MOTOR_LEFT_A2, 0);
  }

  if (abstract_right_force < 0) {
    analogWrite(MOTOR_RIGHT_B2, 0);
    analogWrite(MOTOR_RIGHT_B1, static_cast<unsigned int>(-abstract_right_force));
  } else {
    analogWrite(MOTOR_RIGHT_B2, static_cast<unsigned int>(abstract_right_force));
    analogWrite(MOTOR_RIGHT_B1, 0);
  }
}

std::vector<double> Smartmouse2018Robot::ReadSensors() {
  RangeData<double> range_data_adc{};

  digitalWriteFast(BACK_LEFT_ENABLE_PIN, HIGH);
  digitalWriteFast(BACK_RIGHT_ENABLE_PIN, HIGH);
  digitalWriteFast(FRONT_ENABLE_PIN, HIGH);
  delayMicroseconds(100);
  range_data_adc.back_left = analogRead(BACK_LEFT_ANALOG_PIN);
  range_data_adc.back_right = analogRead(BACK_RIGHT_ANALOG_PIN);
  range_data_adc.front = analogRead(FRONT_ANALOG_PIN);
  digitalWriteFast(BACK_RIGHT_ENABLE_PIN, LOW);
  digitalWriteFast(BACK_LEFT_ENABLE_PIN, LOW);
  digitalWriteFast(FRONT_ENABLE_PIN, LOW);

  digitalWriteFast(FRONT_LEFT_ENABLE_PIN, HIGH);
  digitalWriteFast(FRONT_RIGHT_ENABLE_PIN, HIGH);
  delayMicroseconds(100);
  range_data_adc.front_left = analogRead(FRONT_LEFT_ANALOG_PIN);
  range_data_adc.front_right = analogRead(FRONT_RIGHT_ANALOG_PIN);
  digitalWriteFast(FRONT_LEFT_ENABLE_PIN, LOW);
  digitalWriteFast(FRONT_RIGHT_ENABLE_PIN, LOW);

  digitalWriteFast(GERALD_LEFT_ENABLE_PIN, HIGH);
  digitalWriteFast(GERALD_RIGHT_ENABLE_PIN, HIGH);
  delayMicroseconds(100);
  range_data_adc.gerald_left = analogRead(GERALD_LEFT_ANALOG_PIN);
  range_data_adc.gerald_right = analogRead(GERALD_RIGHT_ANALOG_PIN);
  digitalWriteFast(GERALD_RIGHT_ENABLE_PIN, LOW);
  digitalWriteFast(GERALD_LEFT_ENABLE_PIN, LOW);

  std::vector<double> range_data_m;
  range_data_m.push_back(FRONT_SENSOR.to_meters(range_data_adc.front));
  range_data_m.push_back(FRONT_LEFT_SENSOR.to_meters(range_data_adc.front_left));
  range_data_m.push_back(FRONT_RIGHT_SENSOR.to_meters(range_data_adc.front_right));
  range_data_m.push_back(BACK_LEFT_SENSOR.to_meters(range_data_adc.back_left));
  range_data_m.push_back(BACK_RIGHT_SENSOR.to_meters(range_data_adc.back_right));
  range_data_m.push_back(GERALD_LEFT_SENSOR.to_meters(range_data_adc.gerald_left));
  range_data_m.push_back(GERALD_RIGHT_SENSOR.to_meters(range_data_adc.gerald_right));

  return range_data_m;
}

