#pragma once

#include <array>

#include "direction.h"

namespace ssim {

class SensorReading {
public:
  SensorReading(unsigned int row, unsigned int col);

  std::array<bool, 4> walls;

  bool isWall(Direction dir) const;

  unsigned int const row, col;
};

} // namespace ssim
