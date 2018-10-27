#include <chrono>
#include <iostream>
#include <thread>

#include <fmt/format.h>
#include <hal/util.h>
#include <hal/hal.h>
#include "Arduino.h"

void digitalWrite(unsigned int pin, bool high) {
  // TODO: implement me
}

bool digitalRead(unsigned int pin) {
  // TODO: implement me
  return false;
}

void analogWrite(unsigned int pin, unsigned int value) {
  const auto it = ssim::global_robot_description.pin_map.find(pin);
  if (it == ssim::global_robot_description.pin_map.cend()) {
    throw std::runtime_error{fmt::format("pin {} is not in the pin map.", pin)};
  }

  const auto analog_output = std::get_if<ssim::AnalogOutputDescription>(&it->second);
  const auto motor_pin = std::get_if<ssim::MotorPinDescription>(&it->second);
  if (analog_output)
  {
    analog_output->adc_value = value;
  }
  else if (motor_pin)
  {
    motor_pin->value = value;
  }
  else
  {
    throw std::runtime_error{fmt::format("pin {0} is not an AnalogInput or a MotorPinDescription", pin)};
  }
}

unsigned int analogRead(unsigned int pin) {
  const auto it = ssim::global_robot_description.pin_map.find(pin);
  if (it == ssim::global_robot_description.pin_map.cend()) {
    throw std::runtime_error{fmt::format("pin {} is not in the pin map.", pin)};
  }

  auto analog_input = std::get_if<ssim::AnalogInputDescription>(&it->second);
  if (!analog_input) {
    throw std::runtime_error{fmt::format("pin {0} is not an AnalogInput", pin)};
  }

  return analog_input->adc_value;
}

void pinMode(unsigned int pin, unsigned int mode) {
  // TODO: implement me
}

long micros() {
  using ul_micro = std::chrono::duration<unsigned long, std::micro>;
  const auto micros = std::chrono::duration_cast<ul_micro>(
      ssim::global_robot_description.system_clock.sim_time);
  return micros.count();
}

void delayMicroseconds(unsigned long long micros) {
  std::this_thread::sleep_for(std::chrono::microseconds(micros));
}

unsigned long millis() {
  using ul_milli = std::chrono::duration<unsigned long, std::milli>;
  const auto millis = std::chrono::duration_cast<ul_milli>(
      ssim::global_robot_description.system_clock.sim_time);
  return millis.count();
}

void digitalWriteFast(unsigned int pin, bool high) {
  digitalWrite(pin, high);
}

void CoutSerial::print(std::string const &s) {
  std::cout << s;
}

void CoutSerial::println(std::string const &s) {
  std::cout << s << std::endl;
}

void CoutSerial::print(word w, unsigned int mode) {
  print(std::to_string(w));
}

void NopSerial::print(std::string const &s) {}

void NopSerial::println(std::string const &s) {}

CoutSerial Serial;
NopSerial Serial1;
