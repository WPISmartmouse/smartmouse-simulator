#pragma once

#include <QtWidgets/QWidget>
#include <sim/widgets/abstract_tab.h>

namespace Ui {
class SensorWidget;
}

namespace ssim {

class Client;

class SensorWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  explicit SensorWidget(Client &client);

  const QString GetTabName() override;

 signals:

  void SetTrueBackLeft(QString str);

  void SetTrueFrontLeft(QString str);

  void SetTrueGeraldLeft(QString str);

  void SetTrueFront(QString str);

  void SetTrueGeraldRight(QString str);

  void SetTrueFrontRight(QString str);

  void SetTrueBackRight(QString str);

  void SetEstimatedBackLeft(QString str);

  void SetEstimatedFrontLeft(QString str);

  void SetEstimatedGeraldLeft(QString str);

  void SetEstimatedFront(QString str);

  void SetEstimatedGeraldRight(QString str);

  void SetEstimatedFrontRight(QString str);

  void SetEstimatedBackRight(QString str);

 private:
  Ui::SensorWidget *ui_;
  Client &client_;
};

} // namespace ssim
