#pragma once

#include <thread>

#include <common/core/AbstractMaze.h>
#include <common/Time.h>

namespace ssim {

struct WorldStats {
  unsigned long step = 0;
  Time
};

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
  void OnServerControl(const smartmouse::msgs::ServerControl &msg);
  void OnPhysics(const smartmouse::msgs::PhysicsConfig &msg);
  void OnMaze(const smartmouse::msgs::Maze &msg);
  void OnRobotCommand(const smartmouse::msgs::RobotCommand &msg);
  void OnRobotDescription(const smartmouse::msgs::RobotDescription &msg);

  void UpdateRobotState(double dt);
  void ResetRobot(double reset_col, double reset_row, double reset_yaw);
  void ResetTime();
  void PublishInternalState();
  void PublishWorldStats(double rtf);
  void ComputeMaxSensorRange();
  double ComputeSensorRange(const smartmouse::msgs::SensorDescription sensor);

  int ComputeADCValue(smartmouse::msgs::SensorDescription sensor);
  double ComputeSensorDistToWall(smartmouse::msgs::SensorDescription sensor);

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
  bool mouse_set_;
  unsigned int max_cells_to_check_;
};
}
