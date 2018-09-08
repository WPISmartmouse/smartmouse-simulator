#include <cstdlib>

#include <gtest/gtest.h>

#include <core/direction.h>
#include <core/mouse.h>
#include <core/maze.h>

#include <core/test/mock_mouse.h>

TEST(MouseTest, movement_test) {
  MockMouse m;
  EXPECT_EQ(m.getRow(), 0u);
  EXPECT_EQ(m.getCol(), 0u);
  EXPECT_EQ(m.getDir(), ssim::Direction::E);

  m.internalForward();
  EXPECT_EQ(m.getRow(), 0u);
  EXPECT_EQ(m.getCol(), 1u);
  EXPECT_EQ(m.getDir(), ssim::Direction::E);

  m.internalTurnToFace(ssim::Direction::S);
  EXPECT_EQ(m.getRow(), 0u);
  EXPECT_EQ(m.getCol(), 1u);
  EXPECT_EQ(m.getDir(), ssim::Direction::S);

  m.internalForward();
  EXPECT_EQ(m.getRow(), 1u);
  EXPECT_EQ(m.getCol(), 1u);
  EXPECT_EQ(m.getDir(), ssim::Direction::S);

  m.reset();
  EXPECT_EQ(m.getRow(), 0u);
  EXPECT_EQ(m.getCol(), 0u);
  EXPECT_EQ(m.getDir(), ssim::Direction::E);
}

TEST(MouseTest, wall_test) {
  auto ssim_env = std::getenv("SSIM");

  ASSERT_TRUE(ssim_env);

  auto project_root = std::string(ssim_env);
  // TODO: implement this with std::fs when we upgrade to gcc 8.0
  std::ifstream fs(project_root + "/mazes/tests/test.mz");
  ssim::AbstractMaze maze(fs);
  MockMouse m(maze);
  EXPECT_TRUE(m.isWallInDirection(ssim::Direction::W));
  EXPECT_TRUE(m.isWallInDirection(ssim::Direction::S));
  EXPECT_TRUE(m.isWallInDirection(ssim::Direction::N));
  EXPECT_FALSE(m.isWallInDirection(ssim::Direction::E));
}

