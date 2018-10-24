#include <gtest/gtest.h>

#include <core/maze.h>
#include <core/flood.h>

#include "include/core/test/mock_mouse.h"

TEST(FloodTest, Maze2017) {
  auto ssim_env = std::getenv("SSIM");

  ASSERT_TRUE(ssim_env);

  auto project_root = std::string(ssim_env);
  std::ifstream fs(project_root + "/mazes/tests/2017.mz");
  ssim::AbstractMaze maze(fs);

  MockMouse mouse(maze);
  ssim::Flood flood(&mouse);

  flood.setup();
  auto route = flood.solve();

  ASSERT_TRUE(flood.isSolvable());
  auto constexpr expected_route_str = "3E1S3E1N1E2S4E1N2W1N3E1S2E1S1W1S2E3S1W1N2W2N7W1N2W1S2W2S1E1N4E1S1W1S1E1S2W1S2"
                                      "W1S2E1S1E2N1E1S2E3S4W1N1W2S1E2S5E4N1E1S1E1N1E3S1E6N2W1N1W3N2W2S1E2S1W";
  ASSERT_EQ(ssim::route_to_string(route), expected_route_str);

  flood.setGoal(ssim::Flood::Goal::START);
  route = flood.solve();

  ASSERT_TRUE(flood.isSolvable());
  auto constexpr expected_route_reverse_str = "3E1S3E1N1E2S4E1N2W1N3E1S2E1S1W1S2E4S1W1S3W1N1W3N2W2S1E2S1W";
  ASSERT_EQ(ssim::route_to_string(route), expected_route_reverse_str);
}

TEST(FloodTest, Maze2016) {
  auto ssim_env = std::getenv("SSIM");

  ASSERT_TRUE(ssim_env);

  auto project_root = std::string(ssim_env);
  std::ifstream fs(project_root + "/mazes/tests/2016.mz");
  ssim::AbstractMaze maze(fs);

  MockMouse mouse(maze);
  ssim::Flood flood(&mouse);

  flood.setup();
  auto route = flood.solve();

  ASSERT_TRUE(flood.isSolvable());
  auto constexpr expected_route_str = "2E2S3E1N2E1S1E2S1E1S1E1N1E1S1E1S1E1N1E2S3W2S1W2N1W1S1W";
  ASSERT_EQ(ssim::route_to_string(route), expected_route_str);

  flood.setGoal(ssim::Flood::Goal::START);
  route = flood.solve();

  ASSERT_TRUE(flood.isSolvable());
  auto constexpr expected_route_reverse_str = "2E2S3E1N2E3S1W1S2W1S1W1S3E1N3E2S1W";
  ASSERT_EQ(ssim::route_to_string(route), expected_route_reverse_str);
}

TEST(FloodTest, RandomMaze) {
  srand(0);
  for (auto i = 0; i < 50; i++) {
    auto maze = ssim::AbstractMaze::gen_random_legal_maze();
    MockMouse mouse(maze);
    ssim::Flood flood(&mouse);

    flood.setup();
    auto route = flood.solve();

    ASSERT_TRUE(flood.isSolvable());

    // The shortest possible route is directly E then S
    // which contains the same number of steps as the maze size
    ASSERT_GE(ssim::expanded_route_length(route), ssim::SIZE);
  }
}
