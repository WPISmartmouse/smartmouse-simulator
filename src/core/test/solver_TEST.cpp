#include <gtest/gtest.h>

#include <core/maze.h>
#include <core/flood.h>

#include "include/core/test/mock_mouse.h"

namespace ssim {
void print_maze_str(const AbstractMaze &maze, char *buff) {
  char *b = buff;
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      if (maze.is_wall({i, j}, Direction::W)) {
        strncpy(b++, "|", 1);
        if (maze.is_wall({i, j}, Direction::S)) {
          strncpy(b++, "_", 1);
        } else {
          strncpy(b++, " ", 1);
        }
      } else {
        strcpy(b++, "_");
        if (maze.is_wall({i, j}, Direction::S)) {
          strncpy(b++, "_", 1);
        } else {
          strncpy(b++, " ", 1);
        }
      }
    }
    *(b++) = '|';
    *(b++) = '\n';
  }
  b++;
  *b = '\0';
}
}

TEST(FloodTest, Maze2017) {
  auto ssim_env = std::getenv("SSIM");

  EXPECT_TRUE(ssim_env);

  auto project_root = std::string(ssim_env);
  std::ifstream fs(project_root + "/mazes/tests/2017.mz");
  ssim::AbstractMaze maze(fs);

  MockMouse mouse(maze);
  ssim::Flood flood(&mouse);

  flood.setup();
  auto route = flood.solve();

  // On the first run through we will get a suboptimal path
  EXPECT_TRUE(flood.isSolvable());

  // We find a better path
  mouse.reset();
  route = flood.solve();

  EXPECT_TRUE(flood.isSolvable());

  // We now find the optimal path
  mouse.reset();
  route = flood.solve();

  EXPECT_TRUE(flood.isSolvable());
  auto constexpr expected_route_str = "3E1S3E1N1E2S4E1N2W1N3E1S2E1S1W1S2E4S1W1S3W1N1W3N2W2S1E2S1W";
  EXPECT_EQ(ssim::route_to_string(route), expected_route_str);

  // Without resetting, go in reverse!
  flood.setGoal(ssim::Flood::Goal::START);
  route = flood.solve();

  EXPECT_TRUE(flood.isSolvable());
  auto constexpr expected_route_reverse_str = "1E2N1W2N2E3S1E1S3E1N1E4N2W1N1E1N2W1N3W1S2E1S4W2N1W1S3W1N3W";
  EXPECT_EQ(ssim::route_to_string(route), expected_route_reverse_str);
}

TEST(FloodTest, Maze2016) {
  auto ssim_env = std::getenv("SSIM");

  EXPECT_TRUE(ssim_env);

  auto project_root = std::string(ssim_env);
  std::ifstream fs(project_root + "/mazes/tests/2016.mz");
  ssim::AbstractMaze maze(fs);

  MockMouse mouse(maze);
  ssim::Flood flood(&mouse);

  flood.setup();
  auto route = flood.solve();
  flood.setGoal(ssim::Flood::Goal::START);
  route = flood.solve();
  flood.setGoal(ssim::Flood::Goal::CENTER);
  route = flood.solve();
  flood.setGoal(ssim::Flood::Goal::START);
  route = flood.solve();
  flood.setGoal(ssim::Flood::Goal::CENTER);
  route = flood.solve();

  EXPECT_TRUE(flood.isSolvable());
  auto constexpr expected_route_str = "2E2S1E1S1E1S1W1S1W2S4E1N3E2S1W";
  EXPECT_EQ(ssim::route_to_string(route), expected_route_str);

  flood.setGoal(ssim::Flood::Goal::START);
  route = flood.solve();

  EXPECT_TRUE(flood.isSolvable());
  auto constexpr expected_route_reverse_str = "1E2N3W1S4W2N1E1N1E1N1W1N1W2N2W";
  EXPECT_EQ(ssim::route_to_string(route), expected_route_reverse_str);
}

TEST(FloodTest, invalid) {
  auto ssim_env = std::getenv("SSIM");

  EXPECT_TRUE(ssim_env);

  auto project_root = std::string(ssim_env);
  std::ifstream fs(project_root + "/mazes/tests/impossible.mz");
  ssim::AbstractMaze maze(fs);

  MockMouse mouse(maze);
  ssim::Flood flood(&mouse);

  flood.setup();
  flood.setGoal(ssim::Flood::Goal::START);
  auto route = flood.solve();
  EXPECT_EQ(route.size(), 0u);

  mouse.reset_to(ssim::MazeIndex{1}, ssim::IDX_0);
  flood.setup();
  flood.setGoal(ssim::Flood::Goal::CENTER);

  EXPECT_EQ(route.size(), 0u);
}

TEST(FloodTest, RandomMaze) {
  srand(0);
  for (auto i = 0; i < 50; i++) {
    auto maze = ssim::AbstractMaze::gen_random_legal_maze();
    MockMouse mouse(maze);
    ssim::Flood flood(&mouse);

    flood.setup();
    auto route = flood.solve();

    EXPECT_TRUE(flood.isSolvable());

    // The shortest possible route is directly E then S
    // which contains the same number of steps as the maze size
    ASSERT_GE(ssim::expanded_route_length(route), ssim::SIZE - 2);
  }
}
