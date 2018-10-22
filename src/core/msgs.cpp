#include <core/msgs.h>

namespace ssim {

double SensorDescription::to_adc(double meters) const {
  return static_cast<unsigned int>(pow(a - meters, 1 / b) + c + calibration_offset);
}

double SensorDescription::to_meters(double const adc_value) const {
  const auto d = a - std::pow(adc_value - c - calibration_offset, b);
  if (std::isnan(d)) {
    return max_range_m;
  } else if (d > max_range_m) {
    return max_range_m;
  } else if (d < min_range_m) {
    return min_range_m;
  } else {
    return d;
  }
}

void SensorDescription::calibrate(const int adc_reading) {
  calibration_offset = static_cast<int8_t>(adc_reading - to_adc(calibration_distance));
}

} // namespace ssim

