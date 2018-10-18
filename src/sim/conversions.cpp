#include <core/json.h>
#include <core/maze.h>
#include <sim/conversions.h>

namespace ssim {

std::array<Line2d, 16> Convert(Node const &node) {
  // TODO optimize for each specific configuration of walls so that we don't have to check all 16 lines every time
  double r = node.row();
  double c = node.row();
  return {
      // each line is made of two (col/row points) like {pt 1 col, pt 1 row, pt 2 col, pt 2 row}
      // north wall lines
      Line2d{c, r, c, r + WALL_THICKNESS_CU},
      {c, r, c + 1, r},
      {c + 1, r, c, r + WALL_THICKNESS_CU},
      {c + 1, r, c + 1, r + WALL_THICKNESS_CU},
      // east wall lines
      {c + 1 - WALL_THICKNESS_CU, r, c + 1 - WALL_THICKNESS_CU, r + 1},
      {c + 1 - WALL_THICKNESS_CU, r, c + 1, r},
      {c + 1, r, c + 1, r + 1},
      {c + 1 - WALL_THICKNESS_CU, r + 1, c + 1, r + 1},
      // south wall lines
      {c, r + 1 - WALL_THICKNESS_CU, c, r + 1},
      {c, r + 1 - WALL_THICKNESS_CU, c + 1, r + 1 - WALL_THICKNESS_CU},
      {c + 1, r + 1 - WALL_THICKNESS_CU, c + 1, r + 1},
      {c, r + 1, c + 1, r + 1},
      // west wall lines
      {c, r, c, r + 1},
      {c, r, c + WALL_THICKNESS_CU, r},
      {c + WALL_THICKNESS_CU, r, c + WALL_THICKNESS_CU, r + 1},
      {c, r + 1, c + WALL_THICKNESS_CU, r + 1}
  };
}

} // namespace msgs
