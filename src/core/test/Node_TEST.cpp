#include "gtest/gtest.h"

#include <core/node.h>
#include <core/maze.h>

TEST(NodeTest, MazeSizeTest) {
  static_assert(ssim::SIZE == 16, "You must have the ssim::SIZE set to 16 to run tests.");
}

TEST(NodeTest, ConstructorTest) {
  const ssim::Node n(3, 4);
  EXPECT_EQ(n.row(), 3);
  EXPECT_EQ(n.col(), 4);
}

TEST(NodeTest, NoArgConstructorTest) {
  const ssim::Node n;
  EXPECT_EQ(n.row(), 0);
  EXPECT_EQ(n.col(), 0);
}
