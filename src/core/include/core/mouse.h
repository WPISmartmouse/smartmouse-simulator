#pragma once

#include <sstream>
#include <cmath>

#include <core/direction.h>
#include <core/maze.h>
#include <core/pose.h>

namespace ssim {

/** \brief represents a mouse
 * don't ever change the row/col of a mouse directly. This prevents it from working on the real robot
 * use forward and turnToFace to move the mouse around.
 * Once those functions work on the real robot it will port over fluidly
 */
class Mouse {

public:

  Mouse() = default;

  explicit Mouse(AbstractMaze maze);

  virtual ~Mouse() = default;

  void reset();

  MazeIndex getCol() const;

  MazeIndex getRow() const;

  RowCol getRowCol() const;

  Direction getDir() const;

  void internalTurnToFace(Direction dir);

  void internalForward();

  /** creates one long string of the mouse in the maze in ascii */
  void maze_mouse_string(char *) const;

  virtual SensorReading checkWalls() = 0;

  /** doesn't simply read sensors. Check the internal maze structure */
  bool isWallInDirection(Direction d) const;

  AbstractMaze maze;

protected:
  RowCol row_col;
  Direction dir = Direction::E;
};

} // namespace ssim
