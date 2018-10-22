#pragma once

#include <QtWidgets>
#include <QtGui/QPaintEvent>

#include <core/maze.h>
#include <core/msgs.h>
#include <sim/widgets/abstract_tab.h>

namespace ssim {

class MazeWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  MazeWidget();

  void OnMaze(ssim::AbstractMaze const &msg);
  void OnRobotDescription(ssim::RobotDescription const &msg);
  void OnRobotSimState(ssim::RobotSimState const &msg);

  void paintEvent(QPaintEvent *event);

  const QString GetTabName() override;

 signals:
  void MyUpdate();

 private:
  void PaintWalls(QPainter &painter, QTransform tf);
  void PaintMouse(QPainter &painter, QTransform tf);

  static const int kPaddingPx;
  static const QBrush kRobotBrush;
  static QBrush kWallBrush;

  bool mouse_set_;
};

} // namespace ssim
