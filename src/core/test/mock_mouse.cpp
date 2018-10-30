#include <core/test/mock_mouse.h>

MockMouse::MockMouse(ssim::AbstractMaze &maze) : Mouse(maze) {
}

void MockMouse::reset_to(unsigned int const r, unsigned int const c) {
  reset();
  this->row_col.row = r;
  this->row_col.col = c;
}

ssim::SensorReading MockMouse::checkWalls() {
  ssim::SensorReading sr(getRow(), getCol());
  sr.walls[static_cast<int>(ssim::Direction::N)] = maze.is_wall(getRowCol(), ssim::Direction::N);
  sr.walls[static_cast<int>(ssim::Direction::S)] = maze.is_wall(getRowCol(), ssim::Direction::S);
  sr.walls[static_cast<int>(ssim::Direction::E)] = maze.is_wall(getRowCol(), ssim::Direction::E);
  sr.walls[static_cast<int>(ssim::Direction::W)] = maze.is_wall(getRowCol(), ssim::Direction::W);

  return sr;
}
