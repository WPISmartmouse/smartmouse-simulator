#pragma once

#include <array>

#include "direction.h"

namespace ssim {

class SensorReading {
public:
  SensorReading(int row, int col);

  SensorReading(int row, int col, bool *walls);

  std::array<bool, 4> walls;

  bool isWall(Direction dir) const;

  unsigned int const row, col;
};

} // namespace ssim
