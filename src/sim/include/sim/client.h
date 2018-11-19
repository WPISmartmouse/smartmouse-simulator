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

namespace Ui {
class MainWindow;
}

namespace ssim {

class Client : public QMainWindow {
 Q_OBJECT

 public:
  static const int kRestartCode = 1337;

  explicit Client(Server *server, QMainWindow *parent = nullptr);

  void closeEvent(QCloseEvent *event) override;

 public slots:

  void Exit();

 private slots:

  void Restart();

  void LoadNewMaze();

  void LoadRandomMaze();

  void ShowSourceCode();

  void ShowKeyboardShortcuts();

  void ShowWiki();

  void SendTeleportCmd();

  void RealTimeFactorChanged(double real_time_factor);

  void StepCountChanged(int step_time_ms);

  void TimePerStepMsChanged(int step_time_ms);

  void PIDConstantsSpinboxChanged();

  void PIDSetpointsSpinboxChanged();

  void TogglePlayPause();

  void SetStatic();

  void ResetMouse();

  void ResetTime();

  void Step();

  void OnWorldStats(WorldStatistics msg);

  void OnFinished();

 signals:

  void PhysicsChanged(PhysicsConfig msg);

  void MazeChanged(AbstractMaze msg);

  void ServerChanged(ServerControl msg);

  void SetRealTime(QString str);

  void SetTime(QString str);

 private:
  void ConfigureGui();

  void LoadDefaultMaze();

  void RestoreSettings();

  void SaveSettings();

  unsigned int step_count_ = 1u;
  QSettings *settings_;
  QString maze_files_dir_;
  QString default_maze_file_name_;
  Ui::MainWindow *ui_;
  MazeWidget *maze_widget_;
  StateWidget *state_widget_;
  QShortcut *shortcut;
  bool restart_ = false;
};

} // namespace ssim
