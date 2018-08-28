#include <fstream>
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#include <QtWidgets/QAction>
#include <QtWidgets/QSpinBox>

#include <sim/server.h>
#include <sim/client.h>
#include <QFileDialog>
#include <QCloseEvent>

#include "ui_mainwindow.h"

namespace ssim {

Client::Client(QMainWindow *parent) :
    QMainWindow(parent), ui_(new Ui::MainWindow) {
  ui_->setupUi(this);

  ConfigureGui();
  RestoreSettings();

  // publish the initial configuration
  PhysicsConfig initial_physics_config;
  initial_physics_config.ns_of_sim_per_step = 1000000u;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnPhysics(initial_physics_config); });

  // publish initial config of the server
  ServerControl initial_ServerControl;
  initial_ServerControl.pause = false;
  initial_ServerControl.reset_robot = true;
  initial_ServerControl.reset_time = true;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(initial_ServerControl); });
}

void Client::Exit() {
  SaveSettings();
  ServerControl quit_msg;
  quit_msg.quit = true;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(quit_msg); });
  QApplication::exit(0);
}

void Client::Restart() {
  SaveSettings();
  QApplication::exit(kRestartCode);
}

void Client::TogglePlayPause() {
  ServerControl msg;
  msg.toggle_play_pause = true;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(msg); });
}

void Client::SetStatic() {
  ServerControl static_msg;
  static_msg.stationary = ui_->static_checkbox->isChecked();
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(static_msg); });
}

void Client::Step() {
  ServerControl step_msg;
  step_msg.step = step_count_;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(step_msg); });
}

void Client::ResetMouse() {
  ServerControl reset_msg;
  reset_msg.reset_robot = true;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(reset_msg); });
}

void Client::ResetTime() {
  ServerControl reset_msg;
  reset_msg.reset_time = true;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(reset_msg); });
}

void Client::RealTimeFactorChanged(double real_time_factor) {
  PhysicsConfig rtf_msg;
  rtf_msg.real_time_factor = real_time_factor;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnPhysics(rtf_msg); });
}

void Client::StepCountChanged(int step_time_ms) {
  if (step_time_ms > 0) {
    step_count_ = (unsigned int) step_time_ms;
  }
}

void Client::TimePerStepMsChanged(int step_time_ms) {
  PhysicsConfig time_per_step_msg;
  time_per_step_msg.ns_of_sim_per_step = step_time_ms * 1000000u;
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnPhysics(time_per_step_msg); });
}

void Client::OnWorldStats(const WorldStatistics &msg) {
  Time time(msg.sim_time);
  emit SetRealTime(QString::number(msg.real_time_factor, 'f', 4));
  emit SetTime(QString::fromStdString(time.FormattedString()));
}

void Client::OnPhysics(const PhysicsConfig &msg) {
  if (msg.ns_of_sim_per_step) {
    ui_->ms_per_step_spinner->setValue(msg.ns_of_sim_per_step.value() / 1000000);
  }
  if (msg.real_time_factor) {
    ui_->real_time_factor_spinner->setValue(msg.real_time_factor.value());
  }
}

void Client::OnServerControl(const ServerControl &msg) {
  if (msg.pause) {
    if (msg.pause.value()) {
      ui_->play_button->setText("Play");
    } else {
      ui_->play_button->setText("Pause");
    }
  }
}

void Client::ShowWiki() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/Smartmouse_2018/wiki", QUrl::TolerantMode));
}

void Client::ShowSourceCode() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/Smartmouse_2018", QUrl::TolerantMode));
}

void Client::ShowKeyboardShortcuts() {
  QDesktopServices::openUrl(QUrl("https://github.com/WPISmartMouse/Smartmouse_2018/wiki/Simulator-Hotkeys",
                                 QUrl::TolerantMode));
}

void Client::LoadNewMouse() {
  QString file_name = QFileDialog::getOpenFileName(this, tr("Open Mouse"), mouse_files_dir_, tr("Mouse Files (*.ms)"));

  if (!file_name.isEmpty()) {
    QFileInfo file_info(file_name);
    mouse_files_dir_ = file_info.dir().absolutePath();
    default_mouse_file_name_ = file_name;
    settings_->setValue("gui/default_mouse_file_name", default_mouse_file_name_);
    settings_->setValue("gui/mouse_files_directory", mouse_files_dir_);

    std::ifstream fs;
    fs.open(file_info.absoluteFilePath().toStdString(), std::fstream::in);

    RobotDescription robot_description_msg = convert(fs);
    std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnRobot(robot_description_msg); });
    ui_->mouse_file_name_label->setText(file_info.fileName());
  }
}

void Client::LoadNewMaze() {
  QString file_name = QFileDialog::getOpenFileName(this, tr("Open Maze"), maze_files_dir_, tr("Maze Files (*.mz)"));

  if (!file_name.isEmpty()) {
    QFileInfo file_info(file_name);
    maze_files_dir_ = file_info.dir().absolutePath();
    default_maze_file_name_ = file_name;
    settings_->setValue("gui/default_maze_file_name", default_maze_file_name_);
    settings_->setValue("gui/maze_files_directory", maze_files_dir_);

    std::ifstream fs;
    fs.open(file_info.absoluteFilePath().toStdString(), std::fstream::in);
    AbstractMaze maze(fs);
    std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnMaze(maze); });
    ui_->maze_file_name_label->setText(file_info.fileName());
  }
}

void Client::LoadRandomMaze() {
  const AbstractMaze &maze = AbstractMaze::gen_random_legal_maze();
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnMaze(maze); });
}

void Client::LoadDefaultMouse() {
  if (!default_mouse_file_name_.isEmpty()) {
    QFileInfo file_info(default_mouse_file_name_);

    std::ifstream fs;
    std::string mouse_filename = file_info.absoluteFilePath().toStdString();
    fs.open(mouse_filename, std::fstream::in);
    if (fs.good()) {
      RobotDescription mouse_msg = convert(fs);
      std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnRobot(mouse_msg); });
      ui_->mouse_file_name_label->setText(file_info.fileName());
    } else {
      std::cout << "default mouse file [" << mouse_filename << "] not found\n";
    }
  } else {
    std::cout << "no default mouse\n";
    // TODO: handle this case
  }
}

void Client::LoadDefaultMaze() {
  if (!default_maze_file_name_.isEmpty()) {
    QFileInfo file_info(default_maze_file_name_);

    std::ifstream fs;
    std::string maze_filename = file_info.absoluteFilePath().toStdString();
    fs.open(maze_filename, std::fstream::in);
    if (fs.good()) {
      const AbstractMaze maze(fs);
      std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnMaze(maze); });
      ui_->maze_file_name_label->setText(file_info.fileName());
    } else {
      std::cout << "default mouse file [" << maze_filename << "] not found. Loading random maze.\n";
      LoadRandomMaze();
    }
  } else {
    std::cout << "No default maze. Loading random maze\n";
    LoadRandomMaze();
  }
}

void Client::SendRobotCmd() {
  RobotCommand cmd;
  cmd.left.abstract_force = ui_->left_f_spinbox->value();
  cmd.right.abstract_force = ui_->right_f_spinbox->value();
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnRobotCommand(cmd); });
}

void Client::SendTeleportCmd() {
  ServerControl cmd;
  cmd.reset_robot = true;
  cmd.reset_col = ui_->teleport_column_spinbox->value();
  cmd.reset_row = ui_->teleport_row_spinbox->value();
  cmd.reset_yaw = ui_->teleport_yaw_spinbox->value();
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnServerControl(cmd); });
}

void Client::PublishPIDConstants() {
  PIDConstants msg;
  msg.kP = ui_->kp_spinbox->value();
  msg.kI = ui_->ki_spinbox->value();
  msg.kD = ui_->kd_spinbox->value();
  msg.kFFOffset = ui_->kff_offset_spinbox->value();
  msg.kFFScale = ui_->kff_scale_spinbox->value();
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnPIDConstants(msg); });
}

void Client::PublishPIDSetpoints() {
  Vector2d msg;
  msg.x = ui_->left_setpoint_spinbox->value();
  msg.y = ui_->right_setpoint_spinbox->value();
  std::for_each(plugins.begin(), plugins.end(), [&](auto &plugin) { plugin.OnPIDSetpoints(msg); });
}

void Client::ConfigureGui() {
//  maze_widget_ = new MazeWidget();
//  ui_->gui_tabs->addTab(maze_widget_, maze_widget_->GetTabName());
//  state_widget_ = new StateWidget();
//  ui_->main_splitter->addWidget(state_widget_);
  ui_->info_tabs->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Expanding);
  ui_->info_tabs->setMaximumWidth(300);

  shortcut = new QShortcut(QKeySequence(tr("Space", "Toggle Play/Pause")), this);
  connect(shortcut, &QShortcut::activated, this, &Client::TogglePlayPause);

  connect(ui_->load_maze_button, &QPushButton::clicked, this, &Client::LoadNewMaze);
  connect(ui_->load_mouse_button, &QPushButton::clicked, this, &Client::LoadNewMouse);
  connect(ui_->random_maze_button, &QPushButton::clicked, this, &Client::LoadRandomMaze);
  connect(ui_->refresh_mouse_button, &QPushButton::clicked, this, &Client::LoadDefaultMouse);
  connect(ui_->actionExit, &QAction::triggered, this, &Client::Exit);
  connect(ui_->actionRestart, &QAction::triggered, this, &Client::Restart);
  connect(ui_->actionReset_Mouse, &QAction::triggered, this, &Client::ResetMouse);
  connect(ui_->actionReset_Time, &QAction::triggered, this, &Client::ResetTime);
  connect(ui_->actionSourceCode, &QAction::triggered, this, &Client::ShowSourceCode);
  connect(ui_->actionWiki, &QAction::triggered, this, &Client::ShowWiki);
  connect(ui_->actionKeyboard_Shortcuts, &QAction::triggered, this, &Client::ShowKeyboardShortcuts);
  connect(ui_->play_button, &QPushButton::clicked, this, &Client::TogglePlayPause);
  connect(ui_->step_button, &QPushButton::clicked, this, &Client::Step);
  // Casting is to handle overloaded slot valueChanged. Don't overload slots!
  connect(ui_->static_checkbox, &QCheckBox::stateChanged, this, &Client::SetStatic);
  connect(ui_->real_time_factor_spinner,
          static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          this,
          &Client::RealTimeFactorChanged);
  connect(ui_->step_spinner,
          static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          this,
          &Client::StepCountChanged);
  connect(ui_->ms_per_step_spinner,
          static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          this,
          &Client::TimePerStepMsChanged);
  QObject::connect(this, &Client::SetRealTime, ui_->real_time_value_label, &QLabel::setText);
  QObject::connect(this, &Client::SetTime, ui_->time_value_label, &QLabel::setText);
  connect(ui_->send_command_button, &QPushButton::clicked, this, &Client::SendRobotCmd);
  connect(ui_->teleport_button, &QPushButton::clicked, this, &Client::SendTeleportCmd);
  connect(ui_->publish_constants_button, &QPushButton::clicked, this, &Client::PublishPIDConstants);
  connect(ui_->publish_setpoints_button, &QPushButton::clicked, this, &Client::PublishPIDSetpoints);

  QFile styleFile(":/style.qss");
  styleFile.open(QFile::ReadOnly);
  QString style(styleFile.readAll());
  this->setStyleSheet(style);
}

void Client::closeEvent(QCloseEvent *event) {
  SaveSettings();
  event->accept();
}

void Client::SaveSettings() {
  settings_->setValue("gui/main_splitter", ui_->main_splitter->saveState());
  settings_->setValue("gui/info_tabs", ui_->info_tabs->currentIndex());
  settings_->setValue("gui/static_", ui_->static_checkbox->isChecked());
  settings_->setValue("gui/real_time_value", ui_->real_time_factor_spinner->value());
  settings_->setValue("gui/kp", ui_->kp_spinbox->value());
  settings_->setValue("gui/ki", ui_->ki_spinbox->value());
  settings_->setValue("gui/kd", ui_->kd_spinbox->value());
  settings_->setValue("gui/kff_offset", ui_->kff_offset_spinbox->value());
  settings_->setValue("gui/kff_scale", ui_->kff_scale_spinbox->value());
}

void Client::RestoreSettings() {
  QCoreApplication::setOrganizationName("WPISmartmouse");
  QCoreApplication::setOrganizationDomain("smartmouse.com");
  QCoreApplication::setApplicationName("SmartmouseSim");
  settings_ = new QSettings();

  const QByteArray splitter_state = settings_->value("gui/main_splitter").toByteArray();
  if (!splitter_state.isEmpty()) {
    ui_->main_splitter->restoreState(splitter_state);
  }

  const int info_tab_index = settings_->value("gui/info_tabs").toInt();
  ui_->info_tabs->setCurrentIndex(info_tab_index);

  maze_files_dir_ = settings_->value("gui/maze_files_directory").toString();
  default_maze_file_name_ = settings_->value("gui/default_maze_file_name").toString();
  LoadDefaultMaze();

  mouse_files_dir_ = settings_->value("gui/mouse_files_directory").toString();
  default_mouse_file_name_ = settings_->value("gui/default_mouse_file_name").toString();
  LoadDefaultMouse();

  ui_->static_checkbox->setChecked(settings_->value("gui/static_").toBool());

  ui_->real_time_factor_spinner->setValue(settings_->value("gui/real_time_value").toDouble());
  ui_->kp_spinbox->setValue(settings_->value("gui/kp").toDouble());
  ui_->ki_spinbox->setValue(settings_->value("gui/ki").toDouble());
  ui_->kd_spinbox->setValue(settings_->value("gui/kd").toDouble());
  ui_->kff_offset_spinbox->setValue(settings_->value("gui/kff_offset").toDouble());
  ui_->kff_scale_spinbox->setValue(settings_->value("gui/kff_scale").toDouble());
}

} // namespace ssim
