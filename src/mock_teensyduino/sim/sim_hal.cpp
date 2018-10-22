#include <iostream>
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
  // TODO: implement me
}

unsigned int analogRead(unsigned int pin) {
  // TODO: implement me
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
  // TODO: get simulation time in micros
  return 0;
}

void delayMicroseconds(unsigned long long micros) {
  // TODO: sleep
}

long millis() {
  // TODO: get simulation time in millis
  return 0;
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
