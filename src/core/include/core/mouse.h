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

  /** \brief return the current column.
   * Guaranteed to be between 0 and MAZE_SIZE
   * \return current column
   */
  unsigned int getCol() const;

  /** \brief return the current row.
   * Guaranteed to be between 0 and MAZE_SIZE
   * \return current row
   */
  unsigned int getRow() const;

  /** \brief return the current direction.
   * \return current direction
   */
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
  unsigned int row = 0;
  unsigned int col = 0;
  Direction dir = Direction::E;
};

} // namespace ssim
