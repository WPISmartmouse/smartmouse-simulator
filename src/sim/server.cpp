#include <optional>

// this should be specific to each robot and somehow pluginable or loadable
//#include <common/kinematic_controller/RobotConfig.h>
#include <core/math.h>
#include <kinematic_controller/kinematic_controller.h>
#include <sim/ray_tracing.h>
#include <sim/server.h>

namespace ssim {

Server::Server()
    : sim_time_(Time::Zero),
      steps_(0ul),
      pause_(true),
      stationary_(false),
      quit_(false),
      connected_(false),
      ns_of_sim_per_step_(1000000u),
      pause_at_steps_(0),
      real_time_factor_(1),
      mouse_set_(false),
      max_cells_to_check_(0) {
  ResetRobot(0.5, 0.5, 0);
}

void Server::Start() {
  thread_ = new std::thread(std::bind(&Server::RunLoop, this));
}

void Server::RunLoop() {
  connected_ = true;
  bool done = false;
  while (!done) {
    done = Run();
  }
}

bool Server::Run() {
  Time update_rate = Time(0, ns_of_sim_per_step_);
  Time start_step_time = Time::GetWallTime();
  Time desired_step_time = update_rate / real_time_factor_;
  Time desired_end_time = start_step_time + desired_step_time;

  // special case when update_rate is zero, like on startup.
  if (desired_step_time == 0) {
    Time::MSleep(1);
    return false;
  }

  if (quit_) {
    return true;
  }

  if (pause_at_steps_ > 0 && pause_at_steps_ == steps_) {
    pause_at_steps_ = 0;
    pause_ = true;
    Time::MSleep(1);
    return false;
  }

  if (pause_) {
    Time::MSleep(1);
    return false;
  }

  Step();

  Time end_step_time = Time::GetWallTime();
  if (end_step_time > desired_end_time) {
    // FIXME: do proper logging control
    // std::cout << "step took too long. Skipping sleep." << std::endl;
  } else {
    // FIXME: fudge factor makes sleep time more accurate, because we are often not woken up in time
    Time sleep_time = (desired_end_time - end_step_time) - 5e-5;
    Time::Sleep(sleep_time);
  }

  Time actual_end_step_time = Time::GetWallTime();
  double rtf = update_rate.Double() / (actual_end_step_time - start_step_time).Double();

  // This will send a message the GUI so it can update
  PublishInternalState();

  // announce completion of this step
  PublishWorldStats(rtf);

  return false;
}

void Server::Step() {
  // update sim time
  auto dt = Time(0, ns_of_sim_per_step_);
  sim_time_ += dt;

  if (mouse_set_) {
    UpdateRobotState(dt.Double());
  }

  // increment step counter
  ++steps_;
}

void Server::UpdateRobotState(double dt) {
  // TODO: Implement friction
  const double u_k = mouse_.motor.u_kinetic;
  const double u_s = mouse_.motor.u_static;

  const double motor_J = mouse_.motor.J;
  const double motor_b = mouse_.motor.b;
  const double motor_K = mouse_.motor.K;
  const double motor_R = mouse_.motor.R;
  const double motor_L = mouse_.motor.L;

  // use the cmd abstract forces, apply our dynamics model, update robot state
  double col = robot_state_.p.col;
  double row = robot_state_.p.row;
  double yaw = robot_state_.p.yaw;
  double v_col = robot_state_.v.col;
  double v_row = robot_state_.v.row;
  double dyawdt = robot_state_.v.yaw;
  double tl = robot_state_.left_wheel.theta;
  double wl = robot_state_.left_wheel.omega;
  double il = robot_state_.left_wheel.current;
  double tr = robot_state_.right_wheel.theta;
  double wr = robot_state_.right_wheel.omega;
  double ir = robot_state_.right_wheel.current;

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
  double new_tl = tl + new_wl * dt + 1 / 2 * new_al * dt * dt;
  double new_tr = tr + new_wr * dt + 1 / 2 * new_ar * dt * dt;

  const double kVRef = 5.0;
  double voltage_l = (cmd_.left.abstract_force * kVRef) / 255.0;
  double voltage_r = (cmd_.right.abstract_force * kVRef) / 255.0;
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
  wrapAngleRadInPlace(&new_yaw);

  // Ray trace to find distance to walls
  // iterate over every line segment in the maze (all edges of all walls)
  // find the intersection of that wall with each sensor
  // if the intersection exists, and the distance is the shortest range for that sensor, replace the current range
  robot_state_.front_m = ComputeSensorDistToWall(mouse_.sensors.front);
  robot_state_.front_left_m = ComputeSensorDistToWall(mouse_.sensors.front_left);
  robot_state_.front_right_m = ComputeSensorDistToWall(mouse_.sensors.front_right);
  robot_state_.gerald_left_m = ComputeSensorDistToWall(mouse_.sensors.gerald_left);
  robot_state_.gerald_right_m = ComputeSensorDistToWall(mouse_.sensors.gerald_right);
  robot_state_.back_left_m = ComputeSensorDistToWall(mouse_.sensors.back_left);
  robot_state_.back_right_m = ComputeSensorDistToWall(mouse_.sensors.back_right);

  robot_state_.front_adc = ComputeADCValue(mouse_.sensors.front);
  robot_state_.front_left_adc = ComputeADCValue(mouse_.sensors.front_left);
  robot_state_.front_right_adc = ComputeADCValue(mouse_.sensors.front_right);
  robot_state_.gerald_left_adc = ComputeADCValue(mouse_.sensors.gerald_left);
  robot_state_.gerald_right_adc = ComputeADCValue(mouse_.sensors.gerald_right);
  robot_state_.back_left_adc = ComputeADCValue(mouse_.sensors.back_left);
  robot_state_.back_right_adc = ComputeADCValue(mouse_.sensors.back_right);

  if (!stationary_) {
    robot_state_.p.col = new_col;
    robot_state_.p.row = new_row;
    robot_state_.p.yaw = new_yaw;
    robot_state_.v.col = new_v_col;
    robot_state_.v.row = new_v_row;
    robot_state_.v.yaw = new_dyawdt;
    robot_state_.a.col = new_a_col;
    robot_state_.a.row = new_a_row;
    robot_state_.a.yaw = new_a;
  }
  robot_state_.left_wheel.theta = new_tl;
  robot_state_.left_wheel.omega = new_wl;
  robot_state_.left_wheel.alpha = new_al;
  robot_state_.left_wheel.current = new_il;
  robot_state_.right_wheel.theta = new_tr;
  robot_state_.right_wheel.omega = new_wr;
  robot_state_.right_wheel.alpha = new_ar;
  robot_state_.right_wheel.current = new_ir;
}

void Server::ResetTime() {
  sim_time_ = Time::Zero;
  steps_ = 0UL;
  pause_at_steps_ = 0ul;

  PublishInternalState();
  PublishWorldStats(0);
}

void Server::ResetRobot(double reset_col, double reset_row, double reset_yaw) {
  robot_state_.p.col = reset_col;
  robot_state_.p.row = reset_row;
  robot_state_.p.yaw = reset_yaw;
  robot_state_.v.col = 0;
  robot_state_.v.row = 0;
  robot_state_.v.yaw = 0;
  robot_state_.a.col = 0;
  robot_state_.a.row = 0;
  robot_state_.a.yaw = 0;
  robot_state_.left_wheel.theta = 0;
  robot_state_.left_wheel.omega = 0;
  robot_state_.left_wheel.current = 0;
  robot_state_.right_wheel.theta = 0;
  robot_state_.right_wheel.omega = 0;
  robot_state_.right_wheel.current = 0;
  cmd_.left.abstract_force = 0;
  cmd_.right.abstract_force = 0;

  PublishInternalState();
}

void Server::PublishInternalState() {
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnRobotState(robot_state_); });
}

void Server::PublishWorldStats(double rtf) {
  WorldStatistics world_statistics;
  world_statistics.step = steps_;
  world_statistics.time_s = sim_time_.sec;
  world_statistics.time_ns = sim_time_.nsec;
  world_statistics.real_time_factor = rtf;

  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnWorldStats(world_statistics); });
}

void Server::OnServerControl(const ServerControl &server_control) {
  if (server_control.pause) {
    pause_ = server_control.pause.value();
  } else if (server_control.toggle_play_pause) {
    ServerControl play_pause_msg;
    play_pause_msg.pause = !pause_;
    std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(play_pause_msg); });
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

void Server::OnPhysics(PhysicsConfig const &config) {
  if (config.ns_of_sim_per_step) {
    ns_of_sim_per_step_ = config.ns_of_sim_per_step.value();
  }
  if (config.real_time_factor) {
    if (config.real_time_factor >= 1e-3 && config.real_time_factor <= 10) {
      real_time_factor_ = config.real_time_factor.value();
    }
  }
}

void Server::OnMaze(AbstractMaze const &maze) {
  maze_ = maze;
}

void Server::OnRobotCommand(RobotCommand const &cmd) {
  cmd_ = cmd;
}

void Server::OnRobotDescription(RobotDescription const &description) {
  mouse_ = description;
  ComputeMaxSensorRange();
  mouse_set_ = true;
}

void Server::Join() {
  thread_->join();
}

bool Server::IsConnected() {
  return connected_;
}

unsigned int Server::getNsOfSimPerStep() const {
  return ns_of_sim_per_step_;
}

int Server::ComputeADCValue(SensorDescription sensor) {
  // take the actual distance and the angle and reverse-calculate the ADC value
  double d = ComputeSensorDistToWall(sensor);
  IRModel model{sensor.a, sensor.b, sensor.c, sensor.d};
  return model.toADC(d);
}

double Server::ComputeSensorDistToWall(SensorDescription sensor) {
  double min_range = ANALOG_MAX_DIST_CU;
  // TODO: make a toCellUnits that is vectorized and operates in-place on sense.p
  double sensor_col = toCellUnits(sensor.p.x);
  double sensor_row = toCellUnits(sensor.p.y);
  double robot_theta = robot_state_.p.yaw;
  Eigen::Vector3d s_origin_3d{sensor_col, sensor_row, 1};
  Eigen::Matrix3d tf;
  tf << cos(robot_theta), -sin(robot_theta), robot_state_.p.col,
      sin(robot_theta), cos(robot_theta), robot_state_.p.row,
      0, 0, 1;
  s_origin_3d = tf * s_origin_3d;
  Eigen::Vector2d s_origin{s_origin_3d(0), s_origin_3d(1)};
  Eigen::Vector2d s_direction{cos(robot_theta + sensor.p.theta), sin(robot_theta + sensor.p.theta)};

  // iterate over the lines of walls that are nearby
  int row = (int) robot_state_.p.row;
  int col = (int) robot_state_.p.col;
  unsigned int min_r = (unsigned int) std::max(0, row - (int) max_cells_to_check_);
  unsigned int max_r = std::min(SIZE, row + max_cells_to_check_);
  unsigned int min_c = (unsigned int) std::max(0, col - (int) max_cells_to_check_);
  unsigned int max_c = std::min(SIZE, col + max_cells_to_check_);
  for (unsigned int r = min_r; r < max_r; r++) {
    for (unsigned int c = min_c; c < max_c; c++) {
      // get the walls at r/c
      Node *n = nullptr;
      if (maze_.get_node(&n, r, c) != 0) {
        // FIXME make exits do something clever
        exit(-1);
      }

      auto lines = Convert(*n);
      for (const auto &line : lines) {
        auto range = RayTracing::distance_to_wall(line, s_origin, s_direction);
        if (range && *range < min_range) {
          min_range = *range;
        }
      }
    }
  }

  if (min_range < ANALOG_MIN_DIST_CU) {
    min_range = ANALOG_MIN_DIST_CU;
  }

  return toMeters(min_range);
}

void Server::ComputeMaxSensorRange() {
  double max_range = 0;

  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors.front));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors.front_left));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors.front_right));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors.gerald_left));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors.gerald_right));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors.back_left));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors.back_right));

  max_cells_to_check_ = (unsigned int) std::ceil(toCellUnits(max_range));
}

double Server::ComputeSensorRange(const SensorDescription sensor) {
  double sensor_x = sensor.p.x;
  double sensor_y = sensor.p.x;
  double range_x = sensor_x + cos(sensor.p.theta) * ANALOG_MAX_DIST_M;
  double range_y = sensor_y + sin(sensor.p.theta) * ANALOG_MAX_DIST_M;
  return std::hypot(range_x, range_y);
}

} // namespace ssim
