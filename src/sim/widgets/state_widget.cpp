#include <sstream>
#include <QtWidgets/QHBoxLayout>

#include <sim/widgets/state_widget.h>
#include <core/maze.h>
#include <hal/hal.h>

#include "ui_statewidget.h"

namespace ssim {

StateWidget::StateWidget(QWidget *parent) : QWidget(parent), AbstractTab(), ui_(new Ui::StateWidget) {
  ui_->setupUi(this);

  pid_widget_ = new PIDPlotWidget();
  control_widget_ = new ControlPlotWidget();
  sensor_widget_ = new SensorWidget();
  ui_->charts_tabs->addTab(pid_widget_, pid_widget_->GetTabName());
  ui_->charts_tabs->addTab(control_widget_, control_widget_->GetTabName());
  ui_->charts_tabs->addTab(sensor_widget_, sensor_widget_->GetTabName());

  connect(this, SIGNAL(SetLeftVelocity(QString)), ui_->left_velocity_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetRightVelocity(QString)), ui_->right_velocity_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetLeftCurrent(QString)), ui_->left_current_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetRightCurrent(QString)), ui_->right_current_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetLeftAcceleration(QString)), ui_->left_acceleration_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetRightAcceleration(QString)), ui_->right_acceleration_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetLeftForce(QString)), ui_->left_abstract_force_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetRightForce(QString)), ui_->right_abstract_force_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetRow(QString)), ui_->row_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetCol(QString)), ui_->column_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetDir(QString)), ui_->direction_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetEstimatedCol(QString)), ui_->estimated_col_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetEstimatedRow(QString)), ui_->estimated_row_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetEstimatedYaw(QString)), ui_->estimated_yaw_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetTrueCol(QString)), ui_->true_col_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetTrueRow(QString)), ui_->true_row_edit, SLOT(setText(QString)));
  connect(this, SIGNAL(SetTrueYaw(QString)), ui_->true_yaw_edit, SLOT(setText(QString)));
}


void StateWidget::OnRobotSimState(RobotSimState msg) {
  auto const &p = msg.p;
  this->SetLeftVelocity(QString::asprintf("%0.3f c/s", ssim::radToCU(msg.left_wheel.omega)));
  this->SetRightVelocity(QString::asprintf("%0.3f c/s", ssim::radToCU(msg.right_wheel.omega)));
  this->SetLeftAcceleration(QString::asprintf("%0.3f c/s^2", ssim::radToCU(msg.left_wheel.alpha)));
  this->SetRightAcceleration(QString::asprintf("%0.3f c/s^2", ssim::radToCU(msg.right_wheel.alpha)));
  this->SetLeftCurrent(QString::asprintf("%0.3f mA", msg.left_wheel.current * 1000));
  this->SetRightCurrent(QString::asprintf("%0.3f mA", msg.right_wheel.current * 1000));
  this->SetTrueCol(QString::asprintf("%0.3f (%0.1f cm)", p.col, ssim::toMeters(p.col) * 100));
  this->SetTrueRow(QString::asprintf("%0.3f (%0.1f cm)", p.row, ssim::toMeters(p.row) * 100));
  this->SetTrueYaw(QString::asprintf("%0.1f deg", p.yaw * 180 / M_PI));

  // compute x and y with respect to the top left square
  char col_str[14];
  snprintf(col_str, 14, "%i", (int) p.col);
  char row_str[14];
  snprintf(row_str, 14, "%i", (int) p.row);
  this->SetCol(col_str);
  this->SetRow(row_str);
  this->SetDir(QChar(yaw_to_char(p.yaw)));
  
  auto p = msg.position_cu();
  this->SetEstimatedCol(QString::asprintf("%0.3f (%0.1f cm)", p.col, ssim::toMeters(p.col) * 100));
  this->SetEstimatedRow(QString::asprintf("%0.3f (%0.1f cm)", p.row, ssim::toMeters(p.row) * 100));
  this->SetEstimatedYaw(QString::asprintf("%0.1f deg", p.yaw * 180 / M_PI));
}

void StateWidget::RobotCommandCallback(const RobotCommand msg) {
  emit SetLeftForce(QString::asprintf("%3i / 255", msg.left.abstract_force));
  emit SetRightForce(QString::asprintf("%3i / 255", msg.right.abstract_force));
}

const QString StateWidget::GetTabName() {
  return QString("State Widget");
}

} // namespace ssim

// Force MOC to run on the header file
#include <sim/widgets/moc_state_widget.cpp>
