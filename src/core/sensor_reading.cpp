#include <core/sensor_reading.h>

namespace ssim {

SensorReading::SensorReading(int const row, int const col) :
    walls({false, false, false, false}),
    row(row),
    col(col) {}

SensorReading::SensorReading(int const row, int const col, bool *const walls) :
    walls({walls[0], walls[1], walls[2], walls[3]}),
    row(row),
    col(col) {}

bool SensorReading::isWall(Direction const dir) const {
  return walls[static_cast<int>(dir)];
}

} // namespace ssim
