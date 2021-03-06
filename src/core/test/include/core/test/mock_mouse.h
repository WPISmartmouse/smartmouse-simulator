#include <core/mouse.h>
#include <core/maze.h>
#include <core/maze_index.h>

class MockMouse : public ssim::Mouse {

public:
  ssim::SensorReading checkWalls() override;
  MockMouse() = default;
  ~MockMouse() override = default;
  void reset_to(ssim::MazeIndex r, ssim::MazeIndex c);
  explicit MockMouse(ssim::AbstractMaze &maze);
};
