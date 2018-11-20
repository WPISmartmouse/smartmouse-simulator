#include <core/msgs.h>
#include <sim/widgets/sensor_widget.h>

#include "ui_sensorwidget.h"

namespace ssim {

SensorWidget::SensorWidget() : AbstractTab(), ui_(new Ui::SensorWidget) {
  ui_->setupUi(this);
  connect(this, SIGNAL(SetTrueBackLeft(QString)), ui_->true_back_left_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueFrontLeft(QString)), ui_->true_front_left_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueGeraldLeft(QString)), ui_->true_gerald_left_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueFront(QString)), ui_->true_front_edit, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueGeraldRight(QString)), ui_->true_gerald_right_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueFrontRight(QString)), ui_->true_front_right_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetTrueBackRight(QString)), ui_->true_back_right_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);

  connect(this, SIGNAL(SetEstimatedBackLeft(QString)), ui_->estimated_back_left_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedFrontLeft(QString)), ui_->estimated_front_left_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedGeraldLeft(QString)), ui_->estimated_gerald_left_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedFront(QString)), ui_->estimated_front_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedGeraldRight(QString)), ui_->estimated_gerald_right_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedFrontRight(QString)), ui_->estimated_front_right_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(SetEstimatedBackRight(QString)), ui_->estimated_back_right_edit, SLOT(setText(QString)),
          Qt::QueuedConnection);
}

const QString SensorWidget::GetTabName() {
  return QString("Range Sensors");
}

void SensorWidget::OnRobotSimState(ssim::RobotSimState const &msg) {
}

void SensorWidget::OnDebug(Debug const &msg) {
  for (auto const &pair : msg.sensor_ranges) {
    auto const &name = pair.first;
    auto const range_m = pair.second;
//    SetTrueBackLeft(QString::number()); ??
  }
}

} // namespace ssim

// Force MOC to run on the header file
#include <sim/widgets/moc_sensor_widget.cpp>
