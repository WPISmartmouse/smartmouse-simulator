#include <Arduino.h>

#include <commanduino/command.h>

Command::Command(std::string const &name) : name(name) {
}

bool Command::run() {
  bool finished = false;

  if (!initialized) {
    initialize();
    _initialize();
    initialized = true;
  } else if (isFinished()) {
    finished = true;
    end();
    _end();
  } else {
    execute();
    _execute();
  }

  return finished;
}

void Command::setTimeout(unsigned long timeout) {
  this->timeout = timeout;
}

unsigned long Command::getTime() const {
  return millis() - startTime;
}

bool Command::isTimedOut() const {
  return getTime() > timeout;
}

bool Command::isRunning() const {
  return running;
}

void Command::initialize() {}

void Command::_initialize() {
  running = true;
  startTime = millis();
}

void Command::execute() {}

void Command::_execute() {}

void Command::end() {}

void Command::_end() {
  running = false;
}

bool Command::operator!=(const Command &other) const {
  return this->name != other.name;
}
