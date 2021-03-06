#pragma once

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_draw.h>

#include <core/msgs.h>
#include <sim/widgets/abstract_tab.h>
#include <sim/widgets/pid_series_data.h>

namespace Ui {
class PIDPlotWidget;
}

namespace ssim {

class PIDPlotWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  explicit PIDPlotWidget();

  void Clear();

  void Screenshot();

  const QString GetTabName() override;

  void OnDebug(const Debug &msg);

  void OnRobotSimState(const RobotSimState &msg);

 signals:

  void Replot();

 private slots:

  void LeftChecked();

  void LeftSetpointChecked();

  void RightChecked();

  void RightSetpointChecked();

 private:
  Ui::PIDPlotWidget *ui_;
  QwtPlot *plot_;
  PlotSeriesData *left_setpoint_;
  PlotSeriesData *left_actual_;
  PlotSeriesData *right_setpoint_;
  PlotSeriesData *right_actual_;
  const unsigned int capacity_;
};

} // namespace ssim
