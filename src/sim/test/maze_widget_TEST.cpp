#include <gtest/gtest.h>

#include <sim/widgets/maze_widget.h>

TEST(FloodTest, wall_coordinates) {
  auto const &wall = ssim::WallToCoordinates(0, 0, ssim::Direction::N);
  EXPECT_EQ(wall.c1, -ssim::HALF_WALL_THICKNESS_CU);
  EXPECT_EQ(wall.c2, 1 + ssim::HALF_WALL_THICKNESS_CU);
  EXPECT_EQ(wall.r1, -ssim::HALF_WALL_THICKNESS_CU);
  EXPECT_EQ(wall.r2, ssim::HALF_WALL_THICKNESS_CU);
}
