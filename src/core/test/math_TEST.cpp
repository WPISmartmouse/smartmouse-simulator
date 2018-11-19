#include "gtest/gtest.h"

#include <math.h>

#include <core/math.h>
#include <core/maze.h>

TEST(MathTest, yaw_diff) {
  {
    auto const d = ssim::yaw_diff(3, 1);
    EXPECT_DOUBLE_EQ(d, 2.0);
  }
  {
    auto const d = ssim::yaw_diff(1, -1);
    EXPECT_DOUBLE_EQ(d, 2.0);
  }
  {
    auto const d = ssim::yaw_diff(M_PI, 0);
    EXPECT_DOUBLE_EQ(d, M_PI);
  }
  {
    auto const d = ssim::yaw_diff(0, M_PI);
    EXPECT_DOUBLE_EQ(d, -M_PI);
  }
  {
    auto const d = ssim::yaw_diff(-M_PI, M_PI);
    EXPECT_DOUBLE_EQ(d, 0);
  }
  {
    auto const d = ssim::yaw_diff(M_PI, -M_PI);
    EXPECT_DOUBLE_EQ(d, 0);
  }
  {
    auto const d = ssim::yaw_diff(M_PI + 0.1, -M_PI);
    EXPECT_NEAR(d, -0.1, 1e-6);
  }
  {
    auto const d = ssim::yaw_diff(-M_PI, M_PI + 0.1);
    EXPECT_NEAR(d, 0.1, 1e-6);
  }
  {
    auto const d = ssim::yaw_diff(M_PI, -M_PI - 0.1);
    EXPECT_NEAR(d, -0.1, 1e-6);
  }
  {
    auto const d = ssim::yaw_diff(-M_PI - 0.1, M_PI);
    EXPECT_NEAR(d, 0.1, 1e-6);
  }
}

TEST(MathTest, rad_to_deg) {
  EXPECT_EQ(ssim::rad_to_deg(0), 0);
  EXPECT_EQ(ssim::rad_to_deg(M_PI), 180);
  EXPECT_EQ(ssim::rad_to_deg(M_PI / 2.0), 90);
  EXPECT_EQ(ssim::rad_to_deg(-M_PI / 2.0), -90);
}

TEST(MathTest, wrap_angle) {
  EXPECT_NEAR(ssim::wrapAngleRad(2 * M_PI + 0.1), 0.1, 1e-6);
  EXPECT_NEAR(ssim::wrapAngleRad(2), 2, 1e-6);
}

TEST(MathTest, wrap_angle_in_place) {
  EXPECT_NEAR(ssim::wrapAngleRad(2 * M_PI + 0.1), 0.1, 1e-6);
  EXPECT_NEAR(ssim::wrapAngleRad(2), 2, 1e-6);
}

TEST(MathTest, intersection) {
  using ssim::HALF_WALL_THICKNESS_CU;
  ssim::Line2d l1{1 - HALF_WALL_THICKNESS_CU, -HALF_WALL_THICKNESS_CU, 1 + HALF_WALL_THICKNESS_CU,
                  1 + HALF_WALL_THICKNESS_CU};
  ssim::Line2d l2{0.8, 0.5, 3.0, 0.5};
  Eigen::Vector2d ignore;
  EXPECT_TRUE(l1.Intersect(l2).intersects);
}
