#pragma once

#include <QtWidgets>
#include <QtGui/QPaintEvent>

#include <core/maze.h>
#include <core/msgs.h>
#include <sim/widgets/abstract_tab.h>

namespace ssim {

WallCoordinates WallToCoordinates(double const r, double const c, Direction const dir);

class MazeWidget : public QWidget, public AbstractTab {
 Q_OBJECT

 public:
  explicit MazeWidget(QWidget *parent);

  void OnMaze(AbstractMaze msg);

  void OnRobotSimState(RobotSimState msg);

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
