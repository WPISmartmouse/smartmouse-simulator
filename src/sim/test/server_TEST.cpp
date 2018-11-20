#include <gtest/gtest.h>

#include <QtCore/QObject>

#include <sim/server.h>

TEST(ServerTest, create_test) {
  ssim::Server server;
}

TEST(ServerTest, sensor_dist) {
  auto const ssim_env = std::getenv("SSIM");

  EXPECT_TRUE(ssim_env);

  auto const project_root = std::string(ssim_env);
  std::ifstream fs(project_root + "/mazes/tests/test.mz");
 ssim::AbstractMaze const maze(fs);
  ssim::SensorDescription const sensor{
      .p = {.x=0, .y=0, .theta=M_PI_2},
      .min_range_m = 0.0,
      .max_range_m = 1.0,
      .beam_angle_rad = 0,
      .a = 0,
      .b = 0,
      .c = 0,
      .calibration_offset = 0,
      .calibration_distance = 0,
      .adc_bits = 0,
      .adc_pin = 0
  };
  ssim::RowColYaw robot_pose{.row=0.5, .col=0.5, .yaw=0};
  auto const d = ssim::ComputeSensorDistToWall(maze, sensor, robot_pose, 1);
  EXPECT_NEAR(d, ssim::toMeters(0.5 - ssim::HALF_WALL_THICKNESS_CU), 1e-6);
}
