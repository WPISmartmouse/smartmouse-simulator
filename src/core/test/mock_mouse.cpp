#include <core/test/mock_mouse.h>

MockMouse::MockMouse(ssim::AbstractMaze &maze) : Mouse(maze) {
}

ssim::SensorReading MockMouse::checkWalls() {
  ssim::Node *n;
  auto valid = maze.get_node(&n, getRow(), getCol());

  if (valid == ssim::Node::OUT_OF_BOUNDS) {
    return {getRow(), getCol()};
  }

  ssim::SensorReading sr(getRow(), getCol());
  sr.walls[static_cast<int>(ssim::Direction::N)] = n->wall(ssim::Direction::N);
  sr.walls[static_cast<int>(ssim::Direction::S)] = n->wall(ssim::Direction::S);
  sr.walls[static_cast<int>(ssim::Direction::E)] = n->wall(ssim::Direction::E);
  sr.walls[static_cast<int>(ssim::Direction::W)] = n->wall(ssim::Direction::W);

  return sr;
}
