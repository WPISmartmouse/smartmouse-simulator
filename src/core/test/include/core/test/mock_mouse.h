#include <core/mouse.h>
#include <core/maze.h>

class MockMouse : public ssim::Mouse {

public:
  ssim::SensorReading checkWalls() override;
  MockMouse() = default;
  ~MockMouse() override = default;
  void reset_to(unsigned int r, unsigned int c);
  explicit MockMouse(ssim::AbstractMaze &maze);
};
