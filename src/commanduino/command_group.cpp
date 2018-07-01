#include <commanduino/command_group.h>

CommandGroup::CommandGroup(std::string const &name) : Command(name) {}

void CommandGroup::initialize() {}

void CommandGroup::_initialize() {
  iterator = commands.begin();
  Command::_initialize();
}

void CommandGroup::execute() {}

void CommandGroup::_execute() {
  // run the current command
  bool done = iterator->get()->run();

  // if it's done, remove it and advance the iterator
  if (done) {
    iterator = commands.erase(iterator);
  }
}

void CommandGroup::end() {}

bool CommandGroup::isFinished() {
  return commands.empty();
}
