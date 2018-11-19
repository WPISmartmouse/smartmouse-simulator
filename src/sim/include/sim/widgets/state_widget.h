#pragma once

#include <QtGui/QPainter>
#include <QLineEdit>
#include <QTextEdit>
#include <QScrollArea>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtWidgets/QLabel>

#include <core/msgs.h>
#include <sim/widgets/abstract_tab.h>
#include <sim/widgets/pid_plot_widget.h>
#include <sim/widgets/control_plot_widget.h>
#include <sim/widgets/sensor_widget.h>

namespace Ui {
class StateWidget;
}

namespace ssim {

class Client;

class StateWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  explicit StateWidget(QWidget *parent);

  const QString GetTabName() override;

  public slots:

  void OnRobotSimState(RobotSimState msg);

  void OnDebug(Debug msg);

 signals:

  void SetLeftVelocity(QString str);

  void SetRightVelocity(QString str);

  void SetLeftCurrent(QString str);

  void SetRightCurrent(QString str);

  void SetLeftAcceleration(QString str);

  void SetRightAcceleration(QString str);

  void SetLeftForce(QString str);

  void SetRightForce(QString str);

  void SetRow(QString str);

  void SetCol(QString str);

  void SetDir(QString str);

  void SetTrueCol(QString str);

  void SetTrueRow(QString str);

  void SetTrueYaw(QString str);

  void SetEstimatedCol(QString str);

  void SetEstimatedRow(QString str);

  void SetEstimatedYaw(QString str);

  void HighlightCol();

  void HighlightRow();

  void HighlightYaw();

 private:

  Ui::StateWidget *ui_;
  PIDPlotWidget *pid_widget_;
  ControlPlotWidget *control_widget_;
  SensorWidget *sensor_widget_;
};

} // namespace ssim
