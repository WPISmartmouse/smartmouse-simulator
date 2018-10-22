#pragma once

#include <cstdint>
#include <variant>
#include <string>
#include <array>
#include <map>
#include <vector>

#include <core/math.h>

namespace ssim {

struct Point2 {
  double x;
  double y;
};

struct Point3 {
  double x;
  double y;
  double z;
};

struct RowColYaw {
  double row;
  double col;
  double yaw;
};

struct XYTheta {
  double x;
  double y;
  double theta;
};


struct SensorDescription {
/**
 * We model our sensors as follows:
 *
 * distance in meters = a - (adc - c - adc_offset)^b;
 */
  XYTheta const p;
  double const min_range_m;
  double const max_range_m;
  double const beam_angle_rad;
  double const a;
  double const b;
  double const c;
  double calibration_offset;
  double const calibration_distance;
  unsigned int const adc_bits;
  unsigned int adc_value;

  // We don't use the adc_value in our class because on real hardware that won't be set!
  double to_meters(double adc_value) const;
  double to_adc(double meters) const;
  void calibrate(int adc_reading);
};

struct DigitalInputDescription {
  bool state;
};

struct DigitalOutputDescription {
  bool state;
};

struct AnalogInputDescription {
  unsigned int const n_bits;
  unsigned int adc_value;
};

struct AnalogOutputDescription {
  unsigned int const n_bits;
  unsigned int adc_value;
};

struct EncoderDescription {
  unsigned int const n_bits;
  unsigned int ticks;
};

struct LEDDescription {
  int const r;
  int const g;
  int const b;
  bool state;
};

enum class MotorPinType {
  A,
  B
};

struct MotorPinDescription {
  MotorPinType const motor_pin_type;
  unsigned int value;
};

struct MotorDescription {
  double const u_kinetic;
  double const u_static;
  double const J;
  double const b;
  double const R;
  double const L;
  double const K;
};

struct WheelsDescription {
  Point3 const left_wheel_position;
  Point3 const right_wheel_position;
  double const radius;
  double const thickness;
  double const u_static;
};

using PinVariant = std::variant<AnalogInputDescription,
    AnalogOutputDescription,
    DigitalInputDescription,
    DigitalOutputDescription,
    LEDDescription,
    MotorPinDescription>;

struct RobotDescription {
  std::vector<Point2> const footprint;
  WheelsDescription const wheels;
  Point3 const cog;
  MotorDescription left_motor;
  MotorDescription right_motor;
  EncoderDescription left_encoder;
  EncoderDescription right_encoder;
  std::vector<SensorDescription> sensors;
  std::map<int, PinVariant> pin_map;
  double const track_width_cu;
  double const min_abstract_force;
  double const min_speed_cups;
  double const max_speed_cups;
};

struct WheelPhysicsState {
  double theta; // radians
  double omega; // radians/second
  double alpha;
  double current; // amperes
};

struct RobotSimState {
  WheelPhysicsState left_wheel;
  WheelPhysicsState right_wheel;
  RowColYaw p; // meters
  RowColYaw v; // meters/second
  RowColYaw a; // meters/second^2
};

struct Command {
  int32_t abstract_force = 0;
};

struct RobotCommand {
  Command left;
  Command right;
};

struct WorldStatistics {
  unsigned long step = 0;
  int32_t time_s = 0;
  int32_t time_ns = 0;
  double real_time_factor = 0.0; // RTF acutally acheived
};

class PIDConstants {
 public:
  std::optional<double> kP;
  std::optional<double> kI;
  std::optional<double> kD;
  std::optional<double> kFFOffset;
  std::optional<double> kFFScale;
};

class PIDSetpoints {
 public:
  std::optional<double> left_setpoints_cups;
  std::optional<double> right_setpoints_cups;
};

} // namespace ssim
