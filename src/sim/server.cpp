#include <optional>
#include <chrono>
#include <iostream>
#include <thread>
#include <numeric>

#include <fmt/format.h>
#include <core/math.h>
#include <hal/hal.h>
#include <hal/util.h>
#include <kinematic_controller/kinematic_controller.h>
#include <sim/widgets/maze_widget.h>
#include <sim/ray_tracing.h>
#include <sim/server.h>

namespace ssim {

using namespace std::chrono_literals;
using ul_micros = std::chrono::duration<unsigned long, std::micro>;
using dbl_nanos = std::chrono::duration<double, std::nano>;
using dbl_s = std::chrono::duration<double>;

double ComputeSensorDistToWall(AbstractMaze const &maze, SensorDescription const &sensor, RowColYaw const robot_pose,
                               unsigned int const max_cells_to_check) {
  double min_range_m = sensor.max_range_m;
  // TODO: make a toCellUnits that is vectorized and operates in-place on sense.p
  double const sensor_col = toCellUnits(sensor.p.x);
  double const sensor_row = toCellUnits(sensor.p.y);
  double const robot_theta = robot_pose.yaw;
  Eigen::Vector3d s_origin_3d{sensor_col, sensor_row, 1};
  Eigen::Matrix3d tf;
  tf << cos(robot_theta), -sin(robot_theta), robot_pose.col, sin(robot_theta), cos(
      robot_theta), robot_pose.row, 0, 0, 1;
  s_origin_3d = tf * s_origin_3d;
  Eigen::Vector2d s_origin{s_origin_3d(0), s_origin_3d(1)};
  Eigen::Vector2d s_direction{cos(robot_theta + sensor.p.theta), sin(robot_theta + sensor.p.theta)};

  // iterate over the lines of walls that are nearby
  auto row = static_cast<unsigned int>(robot_pose.row);
  auto col = static_cast<unsigned int>(robot_pose.col);
  unsigned int const min_r = row > max_cells_to_check ? std::max(0u, row - max_cells_to_check) : 0u;
  unsigned int const max_r = std::min(SIZE, row + max_cells_to_check);
  unsigned int const min_c = col > max_cells_to_check ? std::max(0u, col - max_cells_to_check) : 0u;
  unsigned int const max_c = std::min(SIZE, col + max_cells_to_check);
  for (unsigned int r = min_r; r < max_r; r++) {
    for (unsigned int c = min_c; c < max_c; c++) {
      for (auto const d : wise_enum::range<Direction>) {
        if (maze.is_wall({r, c}, d.value)) {
          auto const wall = WallToCoordinates(r, c, d.value);
          std::array<Line2d, 4> lines{
              Line2d{wall.c1, wall.r1, wall.c2, wall.r1},
              Line2d{wall.c2, wall.r1, wall.c2, wall.r2},
              Line2d{wall.c2, wall.r2, wall.c1, wall.r2},
              Line2d{wall.c1, wall.r2, wall.c1, wall.r1}
          };
          for (auto const &line : lines) {
            auto const range_cu = RayTracing::distance_to_wall(line, s_origin, s_direction);
            if (range_cu) {
              auto const range_m = toMeters(range_cu.value());
              if (range_m < min_range_m) {
                min_range_m = range_m;
              }
            }
          }
        }
      }
    }
  }

  min_range_m = std::max(min_range_m, sensor.min_range_m);

  return min_range_m;
}


void Server::thread_run() {
  ResetRobot(0.5, 0.5, 0);
  ComputeMaxSensorRange();

  {
    auto it = robot_description.pin_map.find(robot_description.battery.pin);
    if (it == robot_description.pin_map.cend()) {
      throw std::runtime_error{
          fmt::format("Battery pin {} not found in pin map", robot_description.battery.pin)};
    }
    auto analog_input = std::get_if<ssim::AnalogInputDescription>(&it->second);
    if (!analog_input) {
      throw std::runtime_error{
          fmt::format("Battery pin {} is not an AnalogInput pin", robot_description.battery.pin)};
    }
    analog_input->adc_value =
        static_cast<int>(robot_description.battery.max_voltage / robot_description.battery.volts_per_bit);
  }

  global_plugin->Setup();

  while (true) {
    auto const start_step_time = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock, dbl_nanos> desired_end_time;
    {
      std::lock_guard<std::mutex> lock(message_mutex);
      // Handle stepping forward in time
      auto const desired_step_time_ns = ns_of_sim_per_step_ / real_time_factor_;
      desired_end_time = start_step_time + desired_step_time_ns;

      // special case when update_rate is zero, like on startup.
      if (desired_step_time_ns.count() == 0.0) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      if (quit_) {
        break;
      }

      if (pause_at_steps_ > 0 && pause_at_steps_ == steps_) {
        pause_at_steps_ = 0;
        pause_ = true;
        std::this_thread::sleep_for(1ms);
        continue;
      }

      if (pause_) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      // Update the world and step the robot controller
      Step();
    }

    // Sleep
    auto const end_step_time = std::chrono::steady_clock::now();
    auto const sleep_time = desired_end_time - end_step_time;
    if (sleep_time.count() > 0) {
      std::this_thread::sleep_for(sleep_time);
    }

    auto const actual_end_step_time = std::chrono::steady_clock::now();
    double rtf = std::chrono::duration_cast<dbl_s>(actual_end_step_time - start_step_time).count();

    ssim::WorldStatistics world_stats;
    world_stats.sim_time_ns = std::chrono::nanoseconds(ns_of_sim_per_step_ * steps_);
    world_stats.real_time_factor = rtf;
    world_stats.step = steps_;
    emit WorldStatsChanged(world_stats);
  }

  emit finished();
}

void Server::Step() {
  // update sim time
  robot_description.system_clock.sim_time += ns_of_sim_per_step_;

  // perform physics simulation
  const auto dt_s = std::chrono::duration_cast<dbl_s>(ns_of_sim_per_step_).count();
  SimulateStep(dt_s);

  // run the robot code
  global_plugin->Step();
  auto const &debug = global_plugin->get_debug();

  // emit the information to the client
  emit RobotSimStateChanged(state_);
  emit DebugChanged(debug);

  // increment step counter
  ++steps_;
}

void Server::SimulateStep(double dt) {
  // TODO: Implement friction
  // TODO: use left/right motors seperately
  double const u_k = robot_description.left_motor.u_kinetic;
  double const u_s = robot_description.left_motor.u_static;

  double const motor_J = robot_description.left_motor.J;
  double const motor_b = robot_description.left_motor.b;
  double const motor_K = robot_description.left_motor.K;
  double const motor_R = robot_description.left_motor.R;
  double const motor_L = robot_description.left_motor.L;

  // use the cmd abstract forces, apply our dynamics model, update robot state
  double col = state_.p.col;
  double row = state_.p.row;
  double yaw = state_.p.yaw;
  double v_col = state_.v.col;
  double v_row = state_.v.row;
  double dyawdt = state_.v.yaw;
  double tl = state_.left_wheel.theta;
  double wl = state_.left_wheel.omega;
  double il = state_.left_wheel.current;
  double tr = state_.right_wheel.theta;
  double wr = state_.right_wheel.omega;
  double ir = state_.right_wheel.current;

  double vl = radToMeters(wl);
  double vr = radToMeters(wr);

  double new_al = il * motor_K / motor_J - motor_b * wl / motor_J; // equation 36
  double new_ar = ir * motor_K / motor_J - motor_b * wr / motor_J; // equation 39
  if (wl < 1e-3 && new_al < u_s) {
    new_al = 0;
  } else if (new_al < u_k) {
    new_al = 0;
  }
  if (wr < 1e-3 && new_ar < u_s) {
    new_ar = 0;
  } else if (new_ar < u_k) {
    new_ar = 0;
  }

  double new_wl = wl + new_al * dt;
  double new_wr = wr + new_ar * dt;
  double new_tl = tl + new_wl * dt + 0.5 * new_al * dt * dt;
  double new_tr = tr + new_wr * dt + 0.5 * new_ar * dt * dt;

  auto const left_pin_1 = robot_description.pin_map.at(robot_description.left_motor.pin_1);
  auto const left_analog_1 = std::get_if<ssim::MotorPinDescription>(&left_pin_1);
  auto const left_pin_2 = robot_description.pin_map.at(robot_description.left_motor.pin_2);
  auto const left_analog_2 = std::get_if<ssim::MotorPinDescription>(&left_pin_2);
  auto const left_abstract_force = [&]() {
    if (left_analog_1->value > 0 and left_analog_2->value == 0) {
      return left_analog_1->value;
    } else if (left_analog_2->value > 0 and left_analog_1->value == 0) {
      return left_analog_2->value;
    } else if (left_analog_2->value == 0 and left_analog_1->value == 0) {
      return 0u;
    } else {
      throw std::runtime_error(
          fmt::format("pin 1 and 2 cannot have values {} and {}", left_analog_1->value, left_analog_2->value));
    }
  }();

  auto const right_pin_1 = robot_description.pin_map.at(robot_description.right_motor.pin_1);
  auto const right_analog_1 = std::get_if<ssim::MotorPinDescription>(&right_pin_1);
  auto const right_pin_2 = robot_description.pin_map.at(robot_description.right_motor.pin_2);
  auto const right_analog_2 = std::get_if<ssim::MotorPinDescription>(&right_pin_2);
  auto const right_abstract_force = [&]() {
    if (right_analog_1->value > 0 and right_analog_2->value == 0) {
      return right_analog_1->value;
    } else if (right_analog_2->value > 0 and right_analog_1->value == 0) {
      return right_analog_2->value;
    } else if (right_analog_2->value == 0 and right_analog_1->value == 0) {
      return 0u;
    } else {
      throw std::runtime_error(
          fmt::format("pin 1 and 2 cannot have values {} and {}", right_analog_1->value, right_analog_2->value));
    }
  }();

  double const kVRef = 5.0;
  double voltage_l = (left_abstract_force * kVRef) / 255.0;
  double voltage_r = (right_abstract_force * kVRef) / 255.0;
  double new_il = il + dt * (voltage_l - motor_K * wl - motor_R * il) / motor_L;
  double new_ir = ir + dt * (voltage_r - motor_K * wr - motor_R * ir) / motor_L;

  double vl_cups = toCellUnits(vl);
  double vr_cups = toCellUnits(vr);
  GlobalPose d_pose = KinematicController::forwardKinematics(vl_cups, vr_cups, yaw, dt);

  // update row and col position
  double new_col = col + d_pose.col;
  double new_row = row + d_pose.row;

  // update col and row speed and acceleration
  double new_v_col = (new_col - col) / dt;
  double new_v_row = (new_row - row) / dt;
  double new_a_col = (new_v_col - v_col) / dt;
  double new_a_row = (new_v_row - v_row) / dt;

  // update yaw, omega, and alpha
  double new_yaw = yaw + d_pose.yaw; //eq 27
  double new_dyawdt = yaw_diff(new_yaw, yaw) / dt;
  double new_a = (new_dyawdt - dyawdt) / dt;

  // handle wrap-around of theta
  wrapAngleRadInPlace(new_yaw);

  // Ray trace to find distance to walls
  // iterate over every line segment in the maze (all edges of all walls)
  // find the intersection of that wall with each sensor
  // if the intersection exists, and the distance is the shortest range for that sensor, replace the current range
  for (auto &sensor : robot_description.sensors) {
    // take the actual distance and the angle and reverse-calculate the ADC value
    double const d = ComputeSensorDistToWall(maze_, sensor, state_.p, max_cells_to_check_);
    const auto it = ssim::robot_description.pin_map.find(sensor.adc_pin);
    if (it == ssim::robot_description.pin_map.cend()) {
      throw std::invalid_argument{fmt::format("pin {} is not in the pin map.", sensor.adc_pin)};
    }

    auto analog_input = std::get_if<ssim::AnalogInputDescription>(&it->second);
    if (!analog_input) {
      throw std::invalid_argument{fmt::format("pin {} is not an AnalogInput pin", sensor.adc_pin)};
    }

    analog_input->adc_value = sensor.to_adc(d);
  }

  if (!stationary_) {
    state_.p.col = new_col;
    state_.p.row = new_row;
    state_.p.yaw = new_yaw;
    state_.v.col = new_v_col;
    state_.v.row = new_v_row;
    state_.v.yaw = new_dyawdt;
    state_.a.col = new_a_col;
    state_.a.row = new_a_row;
    state_.a.yaw = new_a;
  }

  state_.left_wheel.abstract_force = left_abstract_force;
  state_.left_wheel.theta = new_tl;
  state_.left_wheel.omega = new_wl;
  state_.left_wheel.alpha = new_al;
  state_.left_wheel.current = new_il;
  state_.right_wheel.abstract_force = right_abstract_force;
  state_.right_wheel.theta = new_tr;
  state_.right_wheel.omega = new_wr;
  state_.right_wheel.alpha = new_ar;
  state_.right_wheel.current = new_ir;

  // set state variables within the global robot description
  robot_description.left_encoder.ticks = rad_to_ticks(state_.left_wheel.theta, robot_description.left_encoder.n_bits);
  robot_description.right_encoder.ticks = rad_to_ticks(state_.right_wheel.theta,
                                                       robot_description.right_encoder.n_bits);
}

void Server::ResetTime() {
  steps_ = 0UL;
  pause_at_steps_ = 0ul;
  emit Redraw();
}

void Server::ResetRobot(double reset_col, double reset_row, double reset_yaw) {
  state_.p.col = reset_col;
  state_.p.row = reset_row;
  state_.p.yaw = reset_yaw;
  state_.v.col = 0;
  state_.v.row = 0;
  state_.v.yaw = 0;
  state_.a.col = 0;
  state_.a.row = 0;
  state_.a.yaw = 0;
  state_.left_wheel.theta = 0;
  state_.left_wheel.omega = 0;
  state_.left_wheel.current = 0;
  state_.right_wheel.theta = 0;
  state_.right_wheel.omega = 0;
  state_.right_wheel.current = 0;
  emit RobotSimStateChanged(state_);
  emit Redraw();
}

void Server::OnServerControl(ServerControl const server_control) {
  std::lock_guard<std::mutex> lock(message_mutex);
  if (server_control.pause) {
    pause_ = server_control.pause.value();
  } else if (server_control.toggle_play_pause) {
    pause_ = !pause_;
  }
  if (server_control.stationary) {
    stationary_ = server_control.stationary.value();
  }
  if (server_control.quit) {
    quit_ = server_control.quit.value();
  }
  if (server_control.step) {
    pause_ = false;
    pause_at_steps_ = steps_ + server_control.step.value();
  }
  if (server_control.reset_time) {
    ResetTime();
  }
  if (server_control.reset_robot) {
    double reset_col = 0.5;
    double reset_row = 0.5;
    double reset_yaw = 0;
    if (server_control.reset_col) {
      reset_col = server_control.reset_col.value();
    }
    if (server_control.reset_row) {
      reset_row = server_control.reset_row.value();
    }
    if (server_control.reset_yaw) {
      reset_yaw = server_control.reset_yaw.value();
    }

    ResetRobot(reset_col, reset_row, reset_yaw);
  }
}

void Server::OnPhysics(PhysicsConfig const config) {
  std::lock_guard<std::mutex> lock(message_mutex);
  if (config.ns_of_sim_per_step) {
    ns_of_sim_per_step_ = std::chrono::nanoseconds(config.ns_of_sim_per_step.value());
  }
  if (config.real_time_factor) {
    if (config.real_time_factor >= 1e-3 && config.real_time_factor <= 10) {
      real_time_factor_ = config.real_time_factor.value();
    }
  }
}

void Server::OnMaze(AbstractMaze const maze) {
  std::lock_guard<std::mutex> lock(message_mutex);
  maze_ = maze;
}

void Server::ComputeMaxSensorRange() {
  auto const max_range = std::accumulate(robot_description.sensors.cbegin(),
                                         robot_description.sensors.cend(), 0.0,
                                         [&](double range, SensorDescription const &sensor) {
                                           return std::max(range,
                                                           ComputeSensorRange(
                                                               sensor));
                                         });

  max_cells_to_check_ = (unsigned int) std::ceil(toCellUnits(max_range));
}

double Server::ComputeSensorRange(const SensorDescription sensor) const {
  double sensor_x = sensor.p.x;
  double sensor_y = sensor.p.x;
  double range_x = sensor_x + cos(sensor.p.theta) * sensor.max_range_m;
  double range_y = sensor_y + sin(sensor.p.theta) * sensor.max_range_m;
  return std::hypot(range_x, range_y);
}

} // namespace ssim

// Force MOC to run on the header file
#include <sim/moc_server.cpp>
