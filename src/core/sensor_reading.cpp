#include <core/sensor_reading.h>

namespace ssim {

SensorReading::SensorReading(MazeIndex row, MazeIndex col) : walls({false}), row_col(row, col) {}

bool SensorReading::isWall(Direction const dir) const {
  return walls[static_cast<int>(dir)];
}

} // namespace ssim
