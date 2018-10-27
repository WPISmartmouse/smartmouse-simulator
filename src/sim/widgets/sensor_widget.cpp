#include <core/msgs.h>
#include <sim/widgets/sensor_widget.h>

#include "ui_sensorwidget.h"

namespace ssim {

SensorWidget::SensorWidget() : AbstractTab(), ui_(new Ui::SensorWidget) {
  ui_->setupUi(this);
  connect(this, SIGNAL(SetTrueBackLeft(QString)), ui_->true_back_left_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueFrontLeft(QString)), ui_->true_front_left_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueGeraldLeft(QString)), ui_->true_gerald_left_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueFront(QString)), ui_->true_front_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueGeraldRight(QString)), ui_->true_gerald_right_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueFrontRight(QString)), ui_->true_front_right_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueBackRight(QString)), ui_->true_back_right_edit, SLOT(setText(QString)), Qt::QueuedConnection);

  connect(this, SIGNAL(SetEstimatedBackLeft(QString)), ui_->estimated_back_left_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedFrontLeft(QString)), ui_->estimated_front_left_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedGeraldLeft(QString)), ui_->estimated_gerald_left_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedFront(QString)), ui_->estimated_front_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedGeraldRight(QString)), ui_->estimated_gerald_right_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedFrontRight(QString)), ui_->estimated_front_right_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedBackRight(QString)), ui_->estimated_back_right_edit, SLOT(setText(QString)), Qt::QueuedConnection);
}

const QString SensorWidget::GetTabName() {
  return QString("Range Sensors");
}

//void SensorWidget::RobotSimStateCallback(const ssim::RobotSimState &msg) {
//  for (auto const & sensor : msg.sensors)
//  SetTrueBackLeft(QString::number(msg.back_left_m));
//  SetTrueFrontLeft(QString::number(msg.front_left_m));
//  SetTrueGeraldLeft(QString::number(msg.gerald_left_m));
//  SetTrueFront(QString::number(msg.front_m));
//  SetTrueBackRight(QString::number(msg.back_right_m));
//  SetTrueFrontRight(QString::number(msg.front_right_m));
//  SetTrueGeraldRight(QString::number(msg.gerald_right_m));
//}

//void SensorWidget::DebugStateCallback(const smartmouse::msgs::DebugState &msg) {
//  SetEstimatedBackLeft(QString::number(msg.back_left_m));
//  SetEstimatedFrontLeft(QString::number(msg.front_left_m));
//  SetEstimatedGeraldLeft(QString::number(msg.gerald_left_m));
//  SetEstimatedFront(QString::number(msg.front_m));
//  SetEstimatedBackRight(QString::number(msg.back_right_m));
//  SetEstimatedFrontRight(QString::number(msg.front_right_m));
//  SetEstimatedGeraldRight(QString::number(msg.gerald_right_m));
//}
}

// Force MOC to run on the header file
#include <sim/widgets/moc_sensor_widget.cpp>
