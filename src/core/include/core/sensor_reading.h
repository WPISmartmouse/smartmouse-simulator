#pragma once

#include <array>

#include "maze_index.h"
#include "node.h"
#include "direction.h"

namespace ssim {

class SensorReading {
public:
  SensorReading(MazeIndex row, MazeIndex col);

  bool isWall(Direction dir) const;

  std::array<bool, 4> walls;

  RowCol row_col;
};

} // namespace ssim
