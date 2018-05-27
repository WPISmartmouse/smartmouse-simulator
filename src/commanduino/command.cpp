#include <Arduino.h>

#include <commanduino/command.h>

Command::Command() : name("unnamed"), initialized(false), running(false), timeout(0), startTime(0) {}

Command::Command(const char *name) : name(name),
                                     initialized(false), running(false), timeout(0), startTime(0) {}

Command::~Command() {}

bool Command::cycle() {
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

unsigned long Command::getTime() {
  return millis() - startTime;
}

bool Command::isTimedOut() {
  return getTime() > timeout;
}

bool Command::isRunning() {
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

bool Command::operator!=(const Command &other) {
  return this->name != other.name;
}