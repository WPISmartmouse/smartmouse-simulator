#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include <core/math.h>

namespace ssim {

struct Point2 {
  double x = 0;
  double y = 0;
};

struct Point3 {
  double x = 0;
  double y = 0;
  double z = 0;
};

struct RowColYaw {
  double row = 0;
  double col = 0;
  double yaw = 0;
};

struct XYTheta {
  double x = 0;
  double y = 0;
  double theta = 0;
};

struct WallCoordinates {
  double r1 = 0;
  double c1 = 0;
  double r2 = 0;
  double c2 = 0;
};


struct SensorDescription {
/**
 * We model our sensors as follows:
 *
 * distance in meters = a - (adc - c - adc_offset)^b;
 */
  XYTheta const p;
  double const min_range_m = 0;
  double const max_range_m = 0;
  double const beam_angle_rad = 0;
  double const a = 0;
  double const b = 0;
  double const c = 0;
  double calibration_offset = 0;
  double const calibration_distance = 0;
  unsigned int const adc_bits = 0;
  unsigned int adc_value = 0;

  // We don't use the adc_value in our class because on real hardware that won't be set!
  double to_meters(double adc_value) const;
  unsigned int to_adc(double meters) const;
  void calibrate(int adc_reading);
};

struct DigitalInputDescription {
  bool state = false;
};

struct DigitalOutputDescription {
  bool state = false;
};

struct AnalogInputDescription {
  unsigned int const n_bits = 0;
  unsigned int adc_value = 0;
};

struct AnalogOutputDescription {
  unsigned int const n_bits = 0;
  unsigned int adc_value = 0;
};

struct EncoderDescription {
  unsigned int const n_bits = 0;
  unsigned int ticks = 0;
};

struct LEDDescription {
  int const r = 0;
  int const g = 0;
  int const b = 0;
  bool state = false;
};

enum class MotorPinType {
  A,
  B
};

struct MotorPinDescription {
  MotorPinType const motor_pin_type;
  unsigned int value = 0;
};

struct MotorDescription {
  double const u_kinetic = 0;
  double const u_static = 0;
  double const J = 0;
  double const b = 0;
  double const R = 0;
  double const L = 0;
  double const K = 0;
  unsigned int const pin_1 = 0;
  unsigned int const pin_2 = 0;
};

struct WheelsDescription {
  Point3 const left_wheel_position;
  Point3 const right_wheel_position;
  double const radius = 0;
  double const thickness = 0;
  double const u_static = 0;
};

struct SystemClock {
  std::chrono::nanoseconds sim_time;
};

struct BatteryDescription {
  double const max_voltage = 0;
  double const volts_per_bit = 0;
  // TODO: come up with a way to ensure that this pin is in the pin_map
  int const pin = 0;
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
  double const track_width_cu = 0;
  double const min_abstract_force = 0;
  double const min_speed_cups = 0;
  double const max_speed_cups = 0;
  SystemClock system_clock;
  BatteryDescription battery;
};

struct WheelPhysicsState {
  double theta; // radian = 0s
  double omega; // radians/secon = 0d
  double alpha = 0;
  double current; // ampere = 0s
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
  std::chrono::nanoseconds sim_time_ns = {};
  double real_time_factor = 0.0; // RTF acutally achieved
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
