#pragma once

#include <string>
#include <sstream>
#include <cstdint>
#include <core/msgs.h>
#include <core/plugin.h>

typedef uint8_t byte;
typedef unsigned int word;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define OUTPUT_OPENDRAIN 4
#define INPUT_DISABLE 5
#define LSBFIRST 0
#define MSBFIRST 1
#define _BV(n) (1<<(n))
#define CHANGE 4
#define FALLING 2
#define RISING 3
#define BIN 2
#define HEX 16

void digitalWriteFast(unsigned int pin, bool state);

void digitalWrite(unsigned int pin, bool state);

bool digitalRead(unsigned int pin);

void analogWrite(unsigned int pin, unsigned int value);

unsigned int analogRead(unsigned int pin);

void delayMicroseconds(unsigned long long micros);

long micros();

unsigned long millis();

void pinMode(unsigned int pin, unsigned int mode);

class CoutSerial {
 public:
  void println(const std::string &);

  void print(const std::string &);

  void print(word, unsigned int);
};

class DebugSerial {
 public:
  void print(const std::string &);

  void println(const std::string &);

  template <typename T>
  T read() {
    // look through the list of messages and gives me the one with the right T?
    // no idea how to implement this correctly...
  }

};

extern CoutSerial Serial;
// this is going to be our mechanism for passing data from the robot to the simulator
// but we will do it in a way that could also pass data to a bluetooth transmitter or something
// so it's juts going to be serialized data or text
extern DebugSerial Serial1;
