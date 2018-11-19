#pragma once

#include <mutex>

#include <core/maze.h>
#include <core/msgs.h>
#include <sim/msgs.h>
#include <core/plugin.h>

namespace ssim {

class Server : public QObject {
 Q_OBJECT

 public slots:

  void thread_run();

  void OnServerControl(ServerControl server_control);

  void OnPhysics(PhysicsConfig config);

  void OnMaze(AbstractMaze maze);

 signals:

  void finished();

  void Redraw();

  void RobotSimStateChanged(RobotSimState state);

  void DebugChanged(Debug debug);

  void WorldStatsChanged(WorldStatistics world_stats);

 private:
  void Step();

  void SimulateStep(double dt);

  void ResetRobot(double reset_col, double reset_row, double reset_yaw);

  void ResetTime();

  void ComputeMaxSensorRange();

  double ComputeSensorRange(SensorDescription sensor);

  double ComputeSensorDistToWall(SensorDescription sensor);

  unsigned long steps_ = 0UL;
  bool pause_ = false;
  bool stationary_ = false;
  bool quit_ = false;
  std::chrono::nanoseconds ns_of_sim_per_step_{1'000'000};
  unsigned int max_cells_to_check_ = 0;
  unsigned long pause_at_steps_ = 0ul;
  double real_time_factor_ = 1.0;
  AbstractMaze maze_;
  RobotSimState state_;
  std::mutex message_mutex;
};

} // namespace ssim
