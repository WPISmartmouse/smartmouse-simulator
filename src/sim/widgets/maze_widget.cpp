#include <QtWidgets/QApplication>
#include <QtGui/QPainter>

#include <sim/widgets/maze_widget.h>
#include <hal/hal.h>
#include <hal/util.h>
#include <thread>

namespace ssim {

const int MazeWidget::kPaddingPx = 24;
const QBrush MazeWidget::kRobotBrush = QBrush(QColor("#F57C00"));
QBrush MazeWidget::kWallBrush = QBrush(Qt::red);

WallCoordinates WallToCoordinates(double const r, double const c, Direction const dir) {
  double c1 = 0, r1 = 0, c2 = 0, r2 = 0;
  switch (dir) {
    case Direction::N: {
      c1 = c - HALF_WALL_THICKNESS_CU;
      r1 = r - HALF_WALL_THICKNESS_CU;
      c2 = c + 1 + HALF_WALL_THICKNESS_CU;
      r2 = r + HALF_WALL_THICKNESS_CU;
      break;
    }
    case Direction::S: {
      c1 = c - HALF_WALL_THICKNESS_CU;
      r1 = r + 1 - HALF_WALL_THICKNESS_CU;
      c2 = c + 1 + HALF_WALL_THICKNESS_CU;
      r2 = r + 1 + HALF_WALL_THICKNESS_CU;
      break;
    }
    case Direction::E: {
      c1 = c + 1 - HALF_WALL_THICKNESS_CU;
      r1 = r - HALF_WALL_THICKNESS_CU;
      c2 = c + 1 + HALF_WALL_THICKNESS_CU;
      r2 = r + 1 + HALF_WALL_THICKNESS_CU;
      break;
    }
    case Direction::W: {
      c1 = c - HALF_WALL_THICKNESS_CU;
      r1 = r - HALF_WALL_THICKNESS_CU;
      c2 = c + HALF_WALL_THICKNESS_CU;
      r2 = r + 1 + HALF_WALL_THICKNESS_CU;
      break;
    }
    default: {
      throw std::runtime_error("Invalid Direction");
    }
  }

  return {.r1=r1, .c1=c1, .r2=r2, .c2=c2};
}


MazeWidget::MazeWidget(QWidget *parent) : QWidget(parent), AbstractTab() {
  setSizePolicy(QSizePolicy::Policy::MinimumExpanding, QSizePolicy::Policy::MinimumExpanding);
}

void MazeWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  QTransform tf;
  {

    QRect g = this->geometry();

    int const w = std::min(g.width(), g.height()) - kPaddingPx;
    double const cell_units_to_pixels = w / SIZE_CU;

    int const origin_col = (g.width() - w) / 2;
    int const origin_row = (g.height() - w) / 2;

    tf.translate(origin_col, origin_row);
    tf = tf.scale(cell_units_to_pixels, cell_units_to_pixels);
  }

  // draw the background
  QRectF const base = QRectF(0, 0, SIZE_CU, SIZE_CU);
  painter.fillRect(tf.mapRect(base), QApplication::palette().background());

  // Draw the thin-line grid over the whole maze
  for (unsigned int i = 0; i <= SIZE; i++) {
    QLineF const h_line(0, i, SIZE_CU, i);
    painter.setPen(QApplication::palette().light().color());
    painter.drawLine(tf.map(h_line));

    QLineF const v_line(i, 0, i, SIZE_CU);
    painter.drawLine(tf.map(v_line));
  }

  PaintWalls(painter, tf);
  PaintMouse(painter, tf);
}

void MazeWidget::PaintMouse(QPainter &painter, QTransform tf) {
  QPainterPath footprint;
  for (auto pt : global_robot_description.footprint) {
    footprint.lineTo(pt.x, pt.y);
  }

  auto const &left_wheel_pose = global_robot_description.wheels.left_wheel_position;
  auto const &right_wheel_pose = global_robot_description.wheels.right_wheel_position;
  double const lwx = left_wheel_pose.x;
  double const lwy = left_wheel_pose.y;
  double const rwx = right_wheel_pose.x;
  double const rwy = right_wheel_pose.y;
  double const radius = global_robot_description.wheels.radius;
  double const thickness = global_robot_description.wheels.thickness;

  QPainterPath left_wheel_path;
  left_wheel_path.moveTo(lwx - radius, lwy - thickness / 2);
  left_wheel_path.lineTo(lwx - radius, lwy + thickness / 2);
  left_wheel_path.lineTo(lwx + radius, lwy + thickness / 2);
  left_wheel_path.lineTo(lwx + radius, lwy - thickness / 2);

  QPainterPath right_wheel_path;
  right_wheel_path.moveTo(rwx - radius, rwy - thickness / 2);
  right_wheel_path.lineTo(rwx - radius, rwy + thickness / 2);
  right_wheel_path.lineTo(rwx + radius, rwy + thickness / 2);
  right_wheel_path.lineTo(rwx + radius, rwy - thickness / 2);

  tf.translate(robot_sim_state_.p.col, robot_sim_state_.p.row);
  tf.rotateRadians(robot_sim_state_.p.yaw, Qt::ZAxis);
  tf.scale(1 / UNIT_DIST_M, 1 / UNIT_DIST_M);

  painter.setPen(QPen(Qt::black));
  painter.fillPath(tf.map(footprint), kRobotBrush);
  painter.fillPath(tf.map(left_wheel_path), QBrush(Qt::black));
  painter.fillPath(tf.map(right_wheel_path), QBrush(Qt::black));

  for (auto sensor : global_robot_description.sensors) {
    auto const sensor_pose = sensor.p;
    // FIXME: This really should not be here
    double const sensor_range = sensor.to_meters(sensor.adc_value);
    QTransform line_tf(tf);
    line_tf.translate(sensor_pose.x, sensor_pose.y);
    line_tf.rotateRadians(sensor_pose.theta, Qt::ZAxis);

    // draw the ray to the wall
    QLineF const line(0, 0, sensor_range, 0);
    painter.setPen(QPen(Qt::black));
    painter.drawLine(line_tf.map(line));
  }
}

void MazeWidget::PaintWalls(QPainter &painter, QTransform tf) {
  for (unsigned int row = 0; row < SIZE; row++) {
    for (unsigned int col = 0; col < SIZE; col++) {
      for (auto d = Direction::First; d < Direction::Last; d++) {
        if (maze_.is_wall(row, col, d)) {
          auto wall = WallToCoordinates(row, col, d);
          QPainterPath wall_path;
          wall_path.moveTo(wall.c1, wall.r1);
          wall_path.lineTo(wall.c2, wall.r1);
          wall_path.lineTo(wall.c2, wall.r2);
          wall_path.lineTo(wall.c1, wall.r2);
          wall_path.lineTo(wall.c1, wall.r1);
          painter.fillPath(tf.map(wall_path), kWallBrush);
        }
      }
    }
  }
}

const QString MazeWidget::GetTabName() {
  return QString("Maze View");
}

void MazeWidget::OnMaze(AbstractMaze maze) {
  maze_ = maze;
  emit update();
}

void MazeWidget::OnRobotSimState(RobotSimState state) {
  robot_sim_state_ = state;

  emit update();
}

} // namespace ssim

// Force MOC to run on the header file
#include <sim/widgets/moc_maze_widget.cpp>
