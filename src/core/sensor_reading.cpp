#include <core/sensor_reading.h>

namespace ssim {

SensorReading::SensorReading(unsigned int row, unsigned int col) : walls({false}), row(row), col(col) {}

bool SensorReading::isWall(Direction const dir) const {
  return walls[static_cast<int>(dir)];
}

} // namespace ssim
