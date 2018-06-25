#include <cstdlib>

#include <gtest/gtest.h>

#include <core/direction.h>
#include <core/mouse.h>
#include <core/abstract_maze.h>

class MockMouse : public ssim::Mouse {

public:
  ssim::SensorReading checkWalls() override {
    return {0, 0};
  }

  MockMouse() = default;
  ~MockMouse() = default;

  explicit MockMouse(ssim::AbstractMaze &maze) : Mouse(maze) {
  }
};

TEST(MouseTest, movement_test) {
  MockMouse m;
  EXPECT_EQ(m.getRow(), 0);
  EXPECT_EQ(m.getCol(), 0);
  EXPECT_EQ(m.getDir(), ssim::Direction::E);

  m.internalForward();
  EXPECT_EQ(m.getRow(), 0);
  EXPECT_EQ(m.getCol(), 1);
  EXPECT_EQ(m.getDir(), ssim::Direction::E);

  m.internalTurnToFace(ssim::Direction::S);
  EXPECT_EQ(m.getRow(), 0);
  EXPECT_EQ(m.getCol(), 1);
  EXPECT_EQ(m.getDir(), ssim::Direction::S);

  m.internalForward();
  EXPECT_EQ(m.getRow(), 1);
  EXPECT_EQ(m.getCol(), 1);
  EXPECT_EQ(m.getDir(), ssim::Direction::S);

  m.reset();
  EXPECT_EQ(m.getRow(), 0);
  EXPECT_EQ(m.getCol(), 0);
  EXPECT_EQ(m.getDir(), ssim::Direction::E);
}

TEST(Mouse_Test, wall_test) {
  auto ssim_env = std::getenv("SSIM");

  if (!ssim_env) {
    std::cerr << "SSIM environment variable not set\n";
    return;
  }

  auto project_root = std::string(ssim_env);
  // TODO: implement this with std::fs when we upgrade to gcc 8.0
  std::ifstream fs(project_root + "/mazes/tests/test.mz");
  ssim::AbstractMaze maze(fs);
  MockMouse m(maze);
  EXPECT_TRUE(m.isWallInDirection(ssim::Direction::W));
  EXPECT_TRUE(m.isWallInDirection(ssim::Direction::S));
  EXPECT_TRUE(m.isWallInDirection(ssim::Direction::N));
  EXPECT_FALSE(m.isWallInDirection(ssim::Direction::E));
  EXPECT_TRUE(false);
}
