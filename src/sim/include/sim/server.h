#pragma once

#include <queue>
#include <functional>
#include <thread>
#include <mutex>

#include <core/maze.h>
#include <core/plugin.h>

#include <sim/conversions.h>
#include <sim/time.h>

namespace ssim {

class Server {

 public:
  Server();

  ~Server();

  void Enqueue();

 private:
  void Start();

  void RunLoop();

  bool Run();

  void Step();

  void Join();

  unsigned int getNsOfSimPerStep() const;

  void OnServerControl(ServerControl const &server_control);

  void OnPhysics(PhysicsConfig const &config);

  void OnMaze(AbstractMaze const &maze);

  void OnRobotCommand(RobotCommand const &cmd);

  void OnRobotDescription(RobotDescription const &description);

  void UpdateRobotState(double dt);

  void ResetRobot(double reset_col, double reset_row, double reset_yaw);

  void ResetTime();

  void PublishInternalState();

  void PublishWorldStats(double rtf);

  void ComputeMaxSensorRange();

  double ComputeSensorRange(SensorDescription sensor);

  int ComputeADCValue(SensorDescription sensor);

  double ComputeSensorDistToWall(SensorDescription sensor);

  Time sim_time_;
  unsigned long steps_ = 0UL;
  bool pause_ = true;
  bool stationary_;
  bool quit_ = false;
  unsigned int ns_of_sim_per_step_ = 1000000u;
  unsigned long pause_at_steps_ = 0ul;
  double real_time_factor_ = 1.0;
  AbstractMaze maze_;
  RobotCommand cmd_;
  RobotDescription mouse_;
  RobotSimState robot_state_;

  bool mouse_set_;
  unsigned int max_cells_to_check_;

  RobotPlugin plugin_;

  std::thread *thread_;

  std::queue<std::function<void()>> queue_;
};

} // namespace ssim
