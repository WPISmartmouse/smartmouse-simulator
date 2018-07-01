#pragma once

#include <commanduino/commanduino.h>

class Delay : public Command {
public:
  explicit Delay(int timeout);

  void initialize() override;

  void execute() override;

  bool isFinished() override;

  void end() override;

private:
  int timeout;
};
