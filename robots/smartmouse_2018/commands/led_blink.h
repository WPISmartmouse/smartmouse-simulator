#pragma once

#include <cstdint>

#include <commanduino/commanduino.h>

class LEDBlink : public Command {
public:
  LEDBlink(const uint8_t led_pin, unsigned long blink_time);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  const uint8_t led_pin;
  bool off;
  unsigned long blink_time;
};
