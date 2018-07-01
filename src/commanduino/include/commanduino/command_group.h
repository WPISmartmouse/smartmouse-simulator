#pragma once

#include <list>
#include <memory>

#include <commanduino/command.h>

/** \brief grouping commands is a useful abstraction.
 * Commands groups execute commands in parallel or series
 */
class CommandGroup : public Command {
public:
  explicit CommandGroup(std::string const &name);

  CommandGroup() = default;

  ~CommandGroup() override = default;

  template<typename T, class... Args>
  void add(Args &&... args) {
    commands.push_back(std::move(std::make_unique<T>(args...)));
  }

  void _initialize() override;

  void _execute() override;

  void initialize() override;

  void execute() override;

  void end() override;

  bool isFinished() override;

  std::list<std::unique_ptr<Command>> commands;

private:

  std::list<std::unique_ptr<Command>>::iterator iterator;

};
