#include "gtest/gtest.h"

#include <math.h>
#include <core/maze.h>

TEST(MazeTest, insert_motion_primitive_back) {
  ssim::route_t route;
  ssim::motion_primitive_t E1{.n=1, .d=ssim::Direction::E};
  ssim::motion_primitive_t W2{.n=2, .d=ssim::Direction::W};

  ssim::insert_motion_primitive_back(&route, E1);
  EXPECT_EQ(ssim::route_to_string(route), "1E");

  ssim::insert_motion_primitive_back(&route, E1);
  EXPECT_EQ(ssim::route_to_string(route), "2E");

  ssim::insert_motion_primitive_back(&route, W2);
  EXPECT_EQ(ssim::route_to_string(route), "2E2W");
}
