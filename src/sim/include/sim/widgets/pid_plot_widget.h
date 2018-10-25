#pragma once

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_draw.h>

#include <core/msgs.h>
#include <sim/widgets/abstract_tab.h>
#include <sim/widgets/pid_series_data.h>

namespace Ui {

class PidPlotWidget;
}

namespace ssim {

class Client;

class PIDPlotWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  explicit PIDPlotWidget(Client &client);

  void Clear();

  void Screenshot();

  const QString GetTabName() override;

  void PIDSetpointsCallback(const ssim::PIDSetpoints &msg);

  void PIDConstantsCallback(const ssim::PIDConstants &msg);

 signals:

  void Replot();

 private slots:

  void LeftChecked();

  void LeftSetpointChecked();

  void RightChecked();

  void RightSetpointChecked();

 private:
  Ui::PidPlotWidget *ui_;
  QwtPlot *plot_;
  PlotSeriesData *left_setpoint_;
  PlotSeriesData *left_actual_;
  PlotSeriesData *right_setpoint_;
  PlotSeriesData *right_actual_;
  const unsigned int capacity_;
  Client &client_;
};

} // namespace ssim
