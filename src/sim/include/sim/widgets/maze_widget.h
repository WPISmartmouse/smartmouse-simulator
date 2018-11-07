#pragma once

#include <QtWidgets/QWidget>
#include <QtGui/QPaintEvent>

#include <core/maze.h>
#include <core/msgs.h>
#include <sim/widgets/abstract_tab.h>

namespace ssim {

WallCoordinates WallToCoordinates(double r, double c, Direction dir);

class MazeWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  explicit MazeWidget(QWidget *parent);

  void OnMaze(AbstractMaze msg);

  void OnRobotSimState(RobotSimState msg);

  void OnRedraw();

  void paintEvent(QPaintEvent *event);

  const QString GetTabName() override;

 private:
  void PaintWalls(QPainter &painter, QTransform tf);

  void PaintMouse(QPainter &painter, QTransform tf);

  static const int kPaddingPx;
  static const QBrush kRobotBrush;
  static QBrush kWallBrush;

  RobotSimState robot_sim_state_;
  AbstractMaze maze_;
};

} // namespace ssim
