#include "gtest/gtest.h"

#include <math.h>
#include <core/direction.h>

TEST(DirectionTest, opposite_direction) {
  EXPECT_EQ(ssim::opposite_direction('N'), 'S');
  EXPECT_EQ(ssim::opposite_direction('S'), 'N');
  EXPECT_EQ(ssim::opposite_direction('W'), 'E');
  EXPECT_EQ(ssim::opposite_direction('E'), 'W');
  EXPECT_EQ(ssim::opposite_direction('x'), '\0');

  EXPECT_EQ(ssim::opposite_direction(ssim::Direction::N), ssim::Direction::S);
  EXPECT_EQ(ssim::opposite_direction(ssim::Direction::S), ssim::Direction::N);
  EXPECT_EQ(ssim::opposite_direction(ssim::Direction::W), ssim::Direction::E);
  EXPECT_EQ(ssim::opposite_direction(ssim::Direction::E), ssim::Direction::W);
}

TEST(DirectionTest, left_of_direction) {
  EXPECT_EQ(ssim::left_of_dir(ssim::Direction::N), ssim::Direction::W);
  EXPECT_EQ(ssim::left_of_dir(ssim::Direction::E), ssim::Direction::N);
  EXPECT_EQ(ssim::left_of_dir(ssim::Direction::S), ssim::Direction::E);
  EXPECT_EQ(ssim::left_of_dir(ssim::Direction::W), ssim::Direction::S);
}

TEST(DirectionTest, right_of_direction) {
  EXPECT_EQ(ssim::right_of_dir(ssim::Direction::N), ssim::Direction::E);
  EXPECT_EQ(ssim::right_of_dir(ssim::Direction::E), ssim::Direction::S);
  EXPECT_EQ(ssim::right_of_dir(ssim::Direction::S), ssim::Direction::W);
  EXPECT_EQ(ssim::right_of_dir(ssim::Direction::W), ssim::Direction::N);
}

TEST(DirectionTest, int_to_dir) {
  EXPECT_EQ(ssim::int_to_dir(0), ssim::Direction::N);
  EXPECT_EQ(ssim::int_to_dir(1), ssim::Direction::E);
  EXPECT_EQ(ssim::int_to_dir(2), ssim::Direction::S);
  EXPECT_EQ(ssim::int_to_dir(3), ssim::Direction::W);
  EXPECT_THROW(ssim::int_to_dir(-3), std::invalid_argument);
}

TEST(DirectionTest, yaw_to_char) {
  EXPECT_EQ(ssim::yaw_to_char(0.0), 'E');
  EXPECT_EQ(ssim::yaw_to_char(M_PI_4), 'E');
  EXPECT_EQ(ssim::yaw_to_char(3*M_PI_4), 'S');
  EXPECT_EQ(ssim::yaw_to_char(M_PI), 'W');
  EXPECT_EQ(ssim::yaw_to_char(-3*M_PI_4), 'W');
  EXPECT_EQ(ssim::yaw_to_char(-M_PI_2), 'N');
  EXPECT_EQ(ssim::yaw_to_char(-M_PI_4), 'N');
}

TEST(DirectionTest, yaw_to_dir) {
  EXPECT_EQ(ssim::yaw_to_dir(0.0), ssim::Direction::E);
  EXPECT_EQ(ssim::yaw_to_dir(M_PI_4), ssim::Direction::E);
  EXPECT_EQ(ssim::yaw_to_dir(M_PI_2), ssim::Direction::S);
  EXPECT_EQ(ssim::yaw_to_dir(3 * M_PI_4), ssim::Direction::S);
  EXPECT_EQ(ssim::yaw_to_dir(M_PI), ssim::Direction::W);
  EXPECT_EQ(ssim::yaw_to_dir(-3 * M_PI_4), ssim::Direction::W);
  EXPECT_EQ(ssim::yaw_to_dir(-M_PI_2), ssim::Direction::N);
  EXPECT_EQ(ssim::yaw_to_dir(-M_PI_4), ssim::Direction::N);
}

TEST(DirectionTest, dir_to_yaw) {
  EXPECT_EQ(ssim::dir_to_yaw(ssim::Direction::E), 0.0);
  EXPECT_EQ(ssim::dir_to_yaw(ssim::Direction::S), M_PI_2);
  EXPECT_EQ(ssim::dir_to_yaw(ssim::Direction::W), M_PI);
  EXPECT_EQ(ssim::dir_to_yaw(ssim::Direction::N), -M_PI_2);
}

TEST(DirectionTest, dir_to_char) {
  EXPECT_EQ(ssim::dir_to_char(ssim::Direction::N), 'N');
  EXPECT_EQ(ssim::dir_to_char(ssim::Direction::S), 'S');
  EXPECT_EQ(ssim::dir_to_char(ssim::Direction::W), 'W');
  EXPECT_EQ(ssim::dir_to_char(ssim::Direction::E), 'E');
}

TEST(DirectionTest, char_to_dir) {
  EXPECT_EQ(ssim::char_to_dir('N'), ssim::Direction::N);
  EXPECT_EQ(ssim::char_to_dir('S'), ssim::Direction::S);
  EXPECT_EQ(ssim::char_to_dir('W'), ssim::Direction::W);
  EXPECT_EQ(ssim::char_to_dir('E'), ssim::Direction::E);
  EXPECT_THROW(ssim::char_to_dir('x'), std::invalid_argument);
}
