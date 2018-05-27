#pragma once

#include <commanduino/CommandDuino.h>

template<typename CommandType, typename... CommandArgs>
class repeat_command : public CommandGroup {
public:
  repeat_command (int count, CommandArgs... args) : CommandGroup("RepeatCommand") {
    for (int i = 0; i < count; i++) {
      addSequential(new CommandType(args...));
    }
  }
};
