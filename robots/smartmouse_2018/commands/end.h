#pragma once

#include <commanduino/commanduino.h>

class End : public Command {
public:
  End();

  bool isFinished();

  void end();
};
