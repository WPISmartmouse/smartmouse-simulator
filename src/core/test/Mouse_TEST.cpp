#include "gtest/gtest.h"

#include <core/direction.h>
#include <core/mouse.h>

class MockMouse : public ssim::Mouse {

  ssim::SensorReading checkWalls () override {
    return {0, 0};
  }
};

TEST(MouseTest, movement_test) {
  MockMouse m;
  EXPECT_EQ(m.getRow(), 0);
  EXPECT_EQ(m.getCol(), 0);
  EXPECT_EQ(m.getDir(), ssim::Direction::E);

  m.internalForward();
  EXPECT_EQ(m.getRow(), 0);
  EXPECT_EQ(m.getCol(), 1);

}
