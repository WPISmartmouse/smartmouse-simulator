#pragma once

#include <sstream>
#include <cmath>

#include "Direction.h"
#include "AbstractMaze.h"
#include "Pose.h"

namespace ssim {

template<typename T>
struct RangeData {
  T gerald_left;
  T gerald_right;
  T front_left;
  T front_right;
  T back_left;
  T back_right;
  T front;

  std::string to_string() {
    std::stringstream ss;
    ss << back_left << ", "
       << front_left << ","
       << gerald_left << ","
       << front << ","
       << gerald_right << ","
       << front_right << ","
       << back_right << ",";
    return ss.str();
  }
};

/** \brief depresents a mouse
 * don't ever change the row/col of a mouse directly. This prevents it from working on the real robot
 * use forward and turnToFace to move the mouse around.
 * Once those functions work on the real robot it will port over fluidly
 */
class Mouse {

public:

  Mouse();

  Mouse(AbstractMaze *maze);

  Mouse(unsigned int starting_row, unsigned int starting_col);

  Mouse(AbstractMaze *maze, unsigned int starting_row, unsigned int starting_col);

  void reset();

  /** \brief return the current column.
   * Guaranteed to be between 0 and MAZE_SIZE
   * \return current column
   */
  unsigned int getCol();

  /** \brief return the current row.
   * Guaranteed to be between 0 and MAZE_SIZE
   * \return current row
   */
  unsigned int getRow();

  /** \brief return the current direction.
   * \return current direction
   */
  Direction getDir();

  void mark_mouse_position_visited();

  void internalTurnToFace(Direction dir);

  void internalForward();

  /** creates one long string of the mouse in the maze in ascii */
  void maze_mouse_string(char *) const;

  virtual SensorReading checkWalls() = 0;

  /** doesn't simply read sensors. Check the internal maze structure */
  bool isWallInDirection(Direction d);

  AbstractMaze *maze;

protected:
  unsigned int row, col;
  Direction dir;
};

} // namespace ssim
