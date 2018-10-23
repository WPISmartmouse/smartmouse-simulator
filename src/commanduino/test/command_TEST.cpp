#include <chrono>
#include <cstdlib>
#include <utility>

#include <hal/util.h>
#include <hal/hal.h>
#include <gtest/gtest.h>

#include <commanduino/commanduino.h>
#include <core/msgs.h>

ssim::RobotDescription ssim::global_robot_description = {};

class MockCommand : public Command {
 public:
  bool mock_is_initialized = false;

  MockCommand() : Command("Mock Command") {}

  ~MockCommand() override = default;

  void initialize() override {
    ssim::print("initializing MockCommand %p \n", this);
    mock_is_initialized = true;
  }

  bool isFinished() override {
    ssim::print("isFinished MockCommand %p \n", this);
    return true;
  }
};

class MockTimedCommand : public Command {
public:
  MockTimedCommand() : Command("Mock Command") {}

  ~MockTimedCommand() override = default;

  void initialize() override {
    setTimeout(1);
  }

  bool isFinished() override {
    return isTimedOut();
  }
};

class MockCommandGroup : public CommandGroup {
public:
  MockCommandGroup() : CommandGroup("Mock Command Group") {
    // add two fake commands
    add<MockCommand>();
    add<MockCommand>();
  }

  ~MockCommandGroup() override = default;

  void initialize() override {
    ssim::print("initialize MockCommandGroup %p \n", this);
  }
};

TEST(CommandTest, command_test) {
  MockCommand cmd;

  EXPECT_FALSE(cmd.mock_is_initialized);
  EXPECT_FALSE(cmd.isRunning());

  auto done = cmd.run();
  EXPECT_TRUE(cmd.mock_is_initialized);
  EXPECT_FALSE(done);
  EXPECT_TRUE(cmd.isRunning());

  done = cmd.run();
  EXPECT_TRUE(done);
  EXPECT_FALSE(cmd.isRunning());
}

TEST(CommandGroupTest, command_group_test) {
  MockCommandGroup cmd;

  // initialize the MockCommand group
  auto done = cmd.run();
  EXPECT_FALSE(done);

  // initialize the first MockCommand
  done = cmd.run();
  EXPECT_FALSE(done);

  // finish the first MockCommand
  done = cmd.run();
  EXPECT_FALSE(done);

  // initialize the second MockCommand
  done = cmd.run();
  EXPECT_FALSE(done);

  // finish the second MockCommand
  done = cmd.run();
  EXPECT_FALSE(done);

  // check that it's done now
  done = cmd.run();
  EXPECT_TRUE(done);
}

