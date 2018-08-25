#pragma once

#include <thread>
#include <mutex>

#include <core/maze.h>

#include <sim/msgs.h>
#include <sim/time.h>

namespace ssim {

class Server {

 public:
  Server();
  void Start();
  void RunLoop();
  bool Run();
  void Step();
  void Join();
  bool IsConnected();
  unsigned int getNsOfSimPerStep() const;

  std::thread *thread_;
 private:
  void OnServerControl(const ssim::ServerControl &msg);
  void OnPhysics(const ssim::PhysicsConfig &msg);
  void OnMaze(const ssim::Maze &msg);
  void OnRobotCommand(const ssim::RobotCommand &msg);
  void OnRobotDescription(const ssim::RobotDescription &msg);

  void UpdateRobotState(double dt);
  void ResetRobot(double reset_col, double reset_row, double reset_yaw);
  void ResetTime();
  void PublishInternalState();
  void PublishWorldStats(double rtf);
  void ComputeMaxSensorRange();
  double ComputeSensorRange(const ssim::SensorDescription sensor);

  int ComputeADCValue(ssim::SensorDescription sensor);
  double ComputeSensorDistToWall(ssim::SensorDescription sensor);

  Time sim_time_;
  unsigned long steps_ = 0UL;
  std::mutex physics_mutex_;
  bool pause_ = true;
  bool static_;
  bool quit_ = false;
  bool connected_ = false;
  unsigned int ns_of_sim_per_step_ = 1000000u;
  unsigned long pause_at_steps_ = 0ul;
  double real_time_factor_ = 1.0;
  ssim::maze_walls_t maze_walls_;
  ssim::RobotCommand cmd_;
  ssim::RobotDescription mouse_;
  ssim::RobotSimState robot_state_;

  bool mouse_set_;
  unsigned int max_cells_to_check_;
};
}
