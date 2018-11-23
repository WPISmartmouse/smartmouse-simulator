#include "gtest/gtest.h"

#include <math.h>
#include <core/maze_index.h>

TEST(MazeIndexTest, initialize) {
  EXPECT_NO_THROW(ssim::MazeIndex row{0});
  EXPECT_NO_THROW(ssim::MazeIndex row{ssim::SIZE - 1});
  EXPECT_THROW(ssim::MazeIndex row{ssim::SIZE}, std::invalid_argument);
  EXPECT_THROW(ssim::MazeIndex row{ssim::SIZE + 1}, std::invalid_argument);
}

TEST(MazeIndexTest, increment) {
  ssim::MazeIndex row{0};
  EXPECT_NO_THROW(row + ssim::MazeIndex{1});
  EXPECT_NO_THROW(row + ssim::MazeIndex{ssim::SIZE - 1};);
  EXPECT_THROW(row + ssim::MazeIndex{ssim::SIZE}, std::invalid_argument);
}


TEST(MazeIndexTest, increment_by_unsigned_int) {
  ssim::MazeIndex row{0};
  EXPECT_NO_THROW(row + 1);
  EXPECT_NO_THROW(row + (ssim::SIZE - 1));
  EXPECT_THROW(row + ssim::SIZE, std::invalid_argument);
  EXPECT_THROW(row + ssim::SIZE + 1, std::invalid_argument);
}

TEST(MazeIndexTest, decrement) {
  ssim::MazeIndex row{0};
  EXPECT_THROW(row - ssim::MazeIndex{1}, std::invalid_argument);

  row = ssim::MazeIndex{1};
  EXPECT_NO_THROW(row - ssim::MazeIndex{1});

  row = ssim::MazeIndex{3};
  EXPECT_NO_THROW(row - ssim::MazeIndex{1});
}


TEST(MazeIndexTest, decrement_by_unsigned_int) {
  ssim::MazeIndex row{1};
  EXPECT_NO_THROW(row - 1);
  EXPECT_THROW(row - 2, std::invalid_argument);

  row = ssim::MazeIndex{1};
  EXPECT_NO_THROW(row - 1);

  row = ssim::MazeIndex{ssim::SIZE - 1};
  EXPECT_NO_THROW(row - (ssim::SIZE - 1));
  EXPECT_THROW(row - ssim::SIZE, std::invalid_argument);
}

TEST(MazeIndexTest, prefix_increment) {
  ssim::MazeIndex row{0};
  EXPECT_NO_THROW(row++);
  EXPECT_EQ(row.value, 1u);
  EXPECT_NO_THROW(row++);
  EXPECT_EQ(row.value, 2u);

  row = ssim::MazeIndex{ssim::SIZE - 2u};
  EXPECT_NO_THROW(row++);
  EXPECT_EQ(row.value, ssim::SIZE - 1u);
  EXPECT_THROW(row++, std::invalid_argument);
}

