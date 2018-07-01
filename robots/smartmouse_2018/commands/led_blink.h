#pragma once

#include <cstdint>

#include <commanduino/commanduino.h>

class LEDBlink : public Command {
public:
  LEDBlink( uint8_t led_pin, unsigned long blink_time);

  void initialize() override;

  void execute() override;

  bool isFinished() override;

  void end() override;

private:
  const uint8_t led_pin;
  bool off;
  unsigned long blink_time;
};
