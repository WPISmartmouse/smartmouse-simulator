#include <cmath>

#include <kinematic_controller/IRModel.h>
#include <kinematic_controller/RobotConfig.h>

namespace ssim {

IRModel::IRModel(double a, double b, double c, double d) : a(a), b(b), c(c), CALIBRATION_DISTANCE(d) {}

double IRModel::toMeters(int adc) const {
  double d = a - std::pow(adc - c - adc_offset, b);
  if (std::isnan(d)) {
    return ANALOG_MAX_DIST_M;
  } else if (d > ANALOG_MAX_DIST_M) {
    return ANALOG_MAX_DIST_M;
  } else if (d < ANALOG_MIN_DIST_M) {
    return ANALOG_MIN_DIST_M;
  } else {
    return d;
  }
}

int IRModel::toADC(double distance_m) const {
  return static_cast<int>(pow(a - distance_m, 1 / b) + c + adc_offset);
}

void IRModel::calibrate(const int adc_reading) {
  adc_offset = static_cast<int8_t>(adc_reading - toADC(CALIBRATION_DISTANCE));
}

} // namespace ssim
