#pragma once

#include <list>

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_draw.h>
#include <QtWidgets/QPushButton>

#include <sim/widgets/abstract_tab.h>
#include <sim/widgets/pid_series_data.h>

namespace Ui {

class control_plot_widget;
}

namespace ssim {

class ControlPlotWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  ControlPlotWidget();

  void Clear();

  void Screenshot();

  const QString GetTabName() override;

  void ControlCallback(const smartmouse::msgs::RobotCommand &msg);

 signals:

  void Replot();

 private slots:

  void LeftChecked();

  void RightChecked();

 private:
  Ui::control_plot_widget *ui_;
  QwtPlot *plot_;
  PlotSeriesData *left_actual_;
  PlotSeriesData *right_actual_;
  const unsigned int capacity_;
};

} // namespace ssim
