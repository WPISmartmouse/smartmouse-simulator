#include "gtest/gtest.h"

#include <math.h>
#include <core/maze.h>

TEST(MazeTest, insert_motion_primitive_back) {
  ssim::Route route;
  ssim::MotionPrimitive E1{.n=1, .d=ssim::Direction::E};
  ssim::MotionPrimitive W2{.n=2, .d=ssim::Direction::W};

  ssim::insert_motion_primitive_back(&route, E1);
  EXPECT_EQ(ssim::route_to_string(route), "1E");

  ssim::insert_motion_primitive_back(&route, E1);
  EXPECT_EQ(ssim::route_to_string(route), "2E");

  ssim::insert_motion_primitive_back(&route, W2);
  EXPECT_EQ(ssim::route_to_string(route), "2E2W");
}

TEST(MazeTest, empty_maze) {
  auto ssim_env = std::getenv("SSIM");

  EXPECT_TRUE(ssim_env);

  for (auto j = 0; j < 10; ++j) {
    auto project_root = std::string(ssim_env);
    std::ifstream fs(project_root + "/mazes/empty.mz");
    ssim::AbstractMaze maze(fs);

    for (unsigned int i = 0; i < ssim::SIZE; ++i) {
      ASSERT_TRUE(maze.is_wall({0, i}, ssim::Direction::N));
      ASSERT_TRUE(maze.is_wall({i, ssim::SIZE-1}, ssim::Direction::E));
      ASSERT_TRUE(maze.is_wall({ssim::SIZE-1, i}, ssim::Direction::S));
      ASSERT_TRUE(maze.is_wall({i, 0}, ssim::Direction::W));
    }
  }
}

TEST(MazeTest, add_works_both_ways) {
  ssim::AbstractMaze mz;
  mz.remove_all_walls();
  mz.add_wall({0, 0}, ssim::Direction::S);
  EXPECT_TRUE(mz.is_wall({0, 0}, ssim::Direction::S));
  EXPECT_TRUE(mz.is_wall({1, 0}, ssim::Direction::N));
}
