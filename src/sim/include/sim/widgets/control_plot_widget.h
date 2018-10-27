#pragma once

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_draw.h>

#include <core/msgs.h>
#include <sim/widgets/abstract_tab.h>
#include <sim/widgets/pid_series_data.h>

namespace Ui {

class ControlPlotWidget;
}

namespace ssim {

class ControlPlotWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  explicit ControlPlotWidget();

  void Clear();

  void Screenshot();

  const QString GetTabName() override;

  void ControlCallback(const RobotCommand msg);

 signals:

  void Replot();

 private slots:

  void LeftChecked();

  void RightChecked();

 private:
  Ui::ControlPlotWidget *ui_;
  QwtPlot *plot_;
  PlotSeriesData *left_actual_;
  PlotSeriesData *right_actual_;
  const unsigned int capacity_;
};

} // namespace ssim
