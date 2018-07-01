#pragma once

#include <string>
#include <cstdint>


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

void digitalWrite(unsigned int pin, bool high);

bool digitalRead(unsigned int pin);

void analogWrite(unsigned int pin, unsigned int value);

unsigned int analogRead(unsigned int pin);

long micros();

long millis();

void pinMode(unsigned int pin, unsigned int mode);

class CoutSerial {
public:
  void println(const std::string &);
  void print(const std::string &);
  void print(word, unsigned int);
};

class NopSerial {
public:
  void print(const std::string &);
  void println(const std::string &);
};

extern CoutSerial Serial;
extern NopSerial Serial1;
