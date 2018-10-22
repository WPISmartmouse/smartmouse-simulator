#pragma once

#include <time.h>

#include <QtCore/QSettings>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QShortcut>
#include <QtCore/QFileInfo>

#include <sim/widgets/maze_widget.h>
#include <sim/widgets/state_widget.h>
#include <core/plugin.h>
#include <sim/server.h>
#include <sim/conversions.h>

namespace Ui {
class MainWindow;
}

namespace ssim {

class Client : public QMainWindow {
 Q_OBJECT

 public:
  static const int kRestartCode = 1337;

  Client(Server &server, QMainWindow *parent = nullptr);

  void closeEvent(QCloseEvent *event) override;

 public slots:

  void Exit();

 private slots:

  void Restart();

  void TogglePlayPause();

  void SetStatic();

  void ResetMouse();

  void ResetTime();

  void Step();

  void LoadNewMaze();

  void ShowSourceCode();

  void ShowKeyboardShortcuts();

  void ShowWiki();

  void RealTimeFactorChanged(double real_time_factor);

  void StepCountChanged(int step_time_ms);

  void TimePerStepMsChanged(int step_time_ms);

  void SendRobotCmd();

  void SendTeleportCmd();

  void PublishPIDConstants();

  void PublishPIDSetpoints();

 signals:

  void SetRealTime(QString str);

  void SetTime(QString str);

 private:
  void ConfigureGui();

  void LoadDefaultMaze();

  void LoadRandomMaze();

  void RestoreSettings();

  void SaveSettings();

  void OnWorldStats(WorldStatistics const &msg);

  void OnPhysics(PhysicsConfig const &msg);

  void OnServerControl(ServerControl const &msg);

  unsigned int step_count_ = 1u;
  QSettings *settings_;
  QString maze_files_dir_;
  QString default_maze_file_name_;
  Ui::MainWindow *ui_;
  MazeWidget maze_widget_;
  StateWidget state_widget_;
  QShortcut *shortcut;
  Server &server_;
};

} // namespace ssim
