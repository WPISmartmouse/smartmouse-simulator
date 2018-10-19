#include <iostream>
#include <fmt/format.h>

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
  const auto it = global_robot_description.pin_map.find(pin);
  if (it == global_robot_description.pin_map.cend()) {
    throw std::runtime_error{fmt::format("{0}{1}{0}", "abra", "cad")};
  } else {
  }

  return 0;
}

void pinMode(unsigned int pin, unsigned int mode) {
  // TODO: implement me
}

long micros() {
  // TODO: get simulation time in micros
  return 0;
}

long millis() {
  // TODO: get simulation time in millis
  return 0;
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

// This variable can be manipulated by the server, and we
// don't need to guard it because the server and the robot plugin run synchronously
ssim::RobotSimState global_sim_state;
ssim::RobotDescription global_robot_description;
