#pragma once

#include <array>

#include <core/Direction.h>

namespace ssim {

class SensorReading {
public:
  SensorReading(int row, int col);

  SensorReading(int row, int col, bool *walls);

  std::array<bool, 4> walls;

  bool isWall(Direction dir);

  const unsigned int row, col;
};

} // namespace ssim
