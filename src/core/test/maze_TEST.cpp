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
      ASSERT_EQ(maze.nodes[0][i]->neighbors[static_cast<int>(ssim::Direction::N)], nullptr);
      ASSERT_EQ(maze.nodes[i][ssim::SIZE - 1]->neighbors[static_cast<int>(ssim::Direction::E)], nullptr);
      ASSERT_EQ(maze.nodes[ssim::SIZE - 1][i]->neighbors[static_cast<int>(ssim::Direction::S)], nullptr);
      ASSERT_EQ(maze.nodes[i][0]->neighbors[static_cast<int>(ssim::Direction::W)], nullptr);
    }
  }
}
