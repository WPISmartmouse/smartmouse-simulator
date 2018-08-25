#pragma once

#include <array>

#include "direction.h"

namespace ssim {

class SensorReading {
public:
  SensorReading(unsigned int row, unsigned int col);

  bool isWall(Direction dir) const;

  std::array<bool, 4> walls;

  unsigned int const row, col;
};

} // namespace ssim
