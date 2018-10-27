#pragma once

#include <QtWidgets>
#include <QtGui/QPaintEvent>

#include <core/maze.h>
#include <core/msgs.h>
#include <sim/widgets/abstract_tab.h>

namespace ssim {

WallCoordinates WallToCoordinates(double r, double c);

class MazeWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  explicit MazeWidget(QWidget *parent);

  void OnMaze(ssim::AbstractMaze const &msg);

  void OnRobotSimState(ssim::RobotSimState const &msg);

  void paintEvent(QPaintEvent *event);

  const QString GetTabName() override;

 private:
  void PaintWalls(QPainter &painter, QTransform tf);

  void PaintMouse(QPainter &painter, QTransform tf);

  static const int kPaddingPx;
  static const QBrush kRobotBrush;
  static QBrush kWallBrush;

  ssim::RobotSimState robot_sim_state_;
  ssim::AbstractMaze maze_;
  bool mouse_set_;
};

} // namespace ssim
