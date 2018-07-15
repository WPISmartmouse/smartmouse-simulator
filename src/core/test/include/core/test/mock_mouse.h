#include <core/mouse.h>
#include <core/maze.h>

class MockMouse : public ssim::Mouse {

public:
  ssim::SensorReading checkWalls() override;
  MockMouse() = default;
  ~MockMouse() override = default;
  explicit MockMouse(ssim::AbstractMaze &maze);
};
