#include "gtest/gtest.h"

#include <core/node.h>
#include <core/maze.h>

TEST(NodeTest, MazeSizeTest) {
  static_assert(ssim::SIZE == 16, "You must have the ssim::SIZE set to 16 to run tests.");
}

TEST(NodeTest, ConstructorTest) {
  const ssim::Node n(ssim::MazeIndex{3}, ssim::MazeIndex{4});
  EXPECT_EQ(n.Row(), ssim::MazeIndex{3u});
  EXPECT_EQ(n.Col(), ssim::MazeIndex{4u});
}

TEST(NodeTest, NoArgConstructorTest) {
  const ssim::Node n;
  EXPECT_EQ(n.Row(), ssim::MazeIndex{0u});
  EXPECT_EQ(n.Col(), ssim::MazeIndex{0u});
}
