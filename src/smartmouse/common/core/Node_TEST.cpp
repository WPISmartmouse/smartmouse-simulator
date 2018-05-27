#include "gtest/gtest.h"

#include "Node.h"
#include "AbstractMaze.h"

TEST(NodeTest, MazeSizeTest) {
  static_assert(ssim::SIZE == 16, "You must have the ssim::SIZE set to 16 to run tests.");
}

TEST(NodeTest, ConstructorTest) {
  ssim::Node n(3, 4);
  EXPECT_EQ(n.row(), 3);
  EXPECT_EQ(n.col(), 4);
}

TEST(NodeTest, NoArgConstructorTest) {
  ssim::Node n;
  EXPECT_EQ(n.row(), 0);
  EXPECT_EQ(n.col(), 0);
}
