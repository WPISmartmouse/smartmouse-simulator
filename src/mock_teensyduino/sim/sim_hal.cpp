#include <chrono>
#include <variant>
#include <iostream>
#include <thread>

#include <fmt/format.h>
#include <hal/util.h>
#include <hal/hal.h>
#include "Arduino.h"

void digitalWrite(unsigned int pin, bool state) {
  const auto it = ssim::global_robot_description.pin_map.find(pin);
  if (it == ssim::global_robot_description.pin_map.cend()) {
    throw std::invalid_argument{fmt::format("pin {} is not in the pin map.", pin)};
  }

  auto digital_output = std::get_if<ssim::DigitalOutputDescription>(&it->second);
  auto led = std::get_if<ssim::LEDDescription>(&it->second);
  if (digital_output) {
    digital_output->state = state;
  }
  else if (led) {
    led->state = state;
  } else {
    throw std::invalid_argument{fmt::format("pin {0} is not a DigitalOutput or LED", pin)};
  }
}

bool digitalRead(unsigned int pin) {
  auto const it = ssim::global_robot_description.pin_map.find(pin);
  if (it == ssim::global_robot_description.pin_map.cend()) {
    throw std::invalid_argument{fmt::format("pin {} is not in the pin map.", pin)};
  }

  auto const digital_input = std::get_if<ssim::DigitalInputDescription>(&it->second);
  if (digital_input) {
    return digital_input->state;
  } else {
    throw std::invalid_argument{fmt::format("pin {} is not a DigitalInput", pin)};
  }
}

void analogWrite(unsigned int pin, unsigned int value) {
  auto const it = ssim::global_robot_description.pin_map.find(pin);
  if (it == ssim::global_robot_description.pin_map.cend()) {
    throw std::invalid_argument{fmt::format("pin {} is not in the pin map.", pin)};
  }

  auto const analog_output = std::get_if<ssim::AnalogOutputDescription>(&it->second);
  auto const motor_pin = std::get_if<ssim::MotorPinDescription>(&it->second);
  if (analog_output) {
    analog_output->adc_value = value;
  } else if (motor_pin) {
    motor_pin->value = value;
  } else {
    throw std::invalid_argument{fmt::format("pin {} is not an AnalogInput or a MotorPinDescription", pin)};
  }
}

unsigned int analogRead(unsigned int pin) {
  const auto it = ssim::global_robot_description.pin_map.find(pin);
  if (it == ssim::global_robot_description.pin_map.cend()) {
    throw std::invalid_argument{fmt::format("pin {} is not in the pin map.", pin)};
  }

  auto analog_input = std::get_if<ssim::AnalogInputDescription>(&it->second);
  if (!analog_input) {
    throw std::invalid_argument{fmt::format("pin {} is not an AnalogInput pin", pin)};
  }

  return analog_input->adc_value;
}

void pinMode(unsigned int pin, unsigned int mode) {
  const auto it = ssim::global_robot_description.pin_map.find(pin);
  if (it == ssim::global_robot_description.pin_map.cend()) {
    throw std::invalid_argument{fmt::format("Tried to set pin mode on pin {}, which is not in the pin map.", pin)};
  }

  auto digital_input = std::holds_alternative<ssim::DigitalInputDescription>(it->second);
  auto digital_output = std::holds_alternative<ssim::DigitalOutputDescription>(it->second);
  auto led = std::holds_alternative<ssim::LEDDescription>(it->second);

  switch (mode) {
    case INPUT:
      if (!(digital_input)) {
        throw std::invalid_argument{fmt::format("can't set INPUT mode on pin {} as it's not a digital input or LED", pin)};
      }
      break;
    case OUTPUT:
      if (!(digital_output or led)) {
        throw std::invalid_argument{fmt::format("can't set OUTPUT mode on pin {} as it's not a digital input or LED", pin)};
      }
      break;
    case INPUT_PULLUP:
      if (!(digital_input)) {
        throw std::invalid_argument{fmt::format("can't set INPUT_PULLUP mode on pin {} as it's not a digital input or LED", pin)};
      }
      break;
    case INPUT_PULLDOWN:
      if (!(digital_input)) {
        throw std::invalid_argument{fmt::format("can't set INPUT_PULLDOWN mode on pin {} as it's not a digital input or LED", pin)};
      }
      break;
    case OUTPUT_OPENDRAIN:
      if (!(digital_output or led)) {
        throw std::invalid_argument{fmt::format("can't set OUTPUT_OPENDRAIN mode on pin {} as it's not a digital input or LED", pin)};
      }
      break;
    case INPUT_DISABLE:
      if (!(digital_input)) {
        throw std::invalid_argument{fmt::format("can't set INPUT_DISABLE mode on pin {} as it's not a digital input or LED", pin)};
      }
      break;
    default:
      throw std::invalid_argument{fmt::format("unknown pin mode {} on pin {}", mode, pin)};
  }
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

void CoutSerial::print(word w, unsigned int /*mode*/) {
  print(std::to_string(w));
}

void NopSerial::print(std::string const &/*s*/) {}

void NopSerial::println(std::string const &/*s*/) {}

CoutSerial Serial;
NopSerial Serial1;
