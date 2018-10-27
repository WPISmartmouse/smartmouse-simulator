#pragma once

#include <QtCore/QThread>

#include <core/maze.h>
#include <core/plugin.h>
#include <sim/conversions.h>

namespace ssim {

class Server : public QThread {
  Q_OBJECT

 public slots:
  void run() override;

  void OnServerControl(ServerControl server_control);

  void OnPhysics(PhysicsConfig config);

  void OnMaze(AbstractMaze maze);

  void OnRobotCommand(RobotCommand cmd);

  void OnPIDConstants(PIDConstants msg);

  void OnPIDSetpoints(PIDSetpoints msg);

 signals:

  void UpdateState(RobotSimState state);

  void UpdateWorldStats(WorldStatistics world_stats);

 private:
  void Step();

  void SimulateStep(double dt);

  void ResetRobot(double reset_col, double reset_row, double reset_yaw);

  void ResetTime();

  void ComputeMaxSensorRange();

  double ComputeSensorRange(SensorDescription sensor);

  double ComputeSensorDistToWall(SensorDescription sensor);

  unsigned long steps_ = 0UL;
  bool pause_ = true;
  bool stationary_ = false;
  bool quit_ = false;
  std::chrono::nanoseconds ns_of_sim_per_step_{1000000};
  unsigned int max_cells_to_check_ = 0;
  unsigned long pause_at_steps_ = 0ul;
  double real_time_factor_ = 1.0;
  AbstractMaze maze_;
  RobotCommand cmd_;
  RobotSimState state_;
};

} // namespace ssim
