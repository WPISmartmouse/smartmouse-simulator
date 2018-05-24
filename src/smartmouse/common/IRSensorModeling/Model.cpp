#include <cmath>

#include <smartmouse/common/IRSensorModeling/Model.h>
///#include <smartmouse/common/KinematicController/RobotConfig.h>

namespace smartmouse {

namespace ir {

Model::Model(double a, double b, double c, double d) : a(a), b(b), c(c), CALIBRATION_DISTANCE(d) {}

double Model::toMeters(int adc) const {
  double d = a - std::pow(adc - c - adc_offset, b);
  if (std::isnan(d)) {
    return smartmouse::kc::ANALOG_MAX_DIST_M;
  } else if (d > smartmouse::kc::ANALOG_MAX_DIST_M) {
    return smartmouse::kc::ANALOG_MAX_DIST_M;
  } else if (d < smartmouse::kc::ANALOG_MIN_DIST_M) {
    return smartmouse::kc::ANALOG_MIN_DIST_M;
  } else {
    return d;
  }
}

int Model::toADC(double distance_m) const {
  return static_cast<int>(pow(a - distance_m, 1 / b) + c + adc_offset);
}

void Model::calibrate(const int adc_reading) {
  adc_offset = static_cast<int8_t>(adc_reading - toADC(CALIBRATION_DISTANCE));
}

}
}
