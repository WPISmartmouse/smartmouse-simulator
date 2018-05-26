#include <iostream>

#include "Arduino.h"

void digitalWrite(unsigned int pin, bool high) {
  // TODO: implement digital for LEDs by doing a lookup into a table of pins that are registered as LEDs
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
  return 0;
}

void pinMode(unsigned int pin, unsigned int mode) {
  // TODO: implement me
}

void CoutSerial::print(const std::string &s) {
  std::cout << s;
}

void NopSerial::print(const std::string &s) {}
