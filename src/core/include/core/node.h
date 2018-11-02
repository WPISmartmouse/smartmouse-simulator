#pragma once

#include <limits>
#include <array>

namespace ssim {

struct RowCol {
  bool operator==(const RowCol &other) const;

  unsigned int row;
  unsigned int col;
};

enum class WallEnum {
  NoWall,
  PerimeterWall,
  Wall
};

/**
 * \brief holds its location & neighbors, as well as a bool for indicating if it has been discovered
 * you don't need to free nodes in a maze, just use free_maze however, be sure to free nodes allocated not in mazes
 * visited is meant for ACTUALLY visiting, known is just used for searching/solving
 */
class Node {
 public:
  Node(RowCol row_col);

  Node(unsigned int row, unsigned int col);

  Node() = default;

  bool operator==(Node const &other) const;

  bool operator!=(Node const &other) const;

  unsigned int Row() const;

  unsigned int Col() const;

  RowCol GetRowCol() const;

  void Reset();

  int weight = std::numeric_limits<decltype(weight)>::max(); // used for flood-fill
  int distance = 0; // used for A-star
  bool known = false;
  bool visited = false;
  std::array<WallEnum, 4> walls = {WallEnum::NoWall, WallEnum::NoWall, WallEnum::NoWall, WallEnum::NoWall};

 private:
  RowCol row_col = {0};
};

} // namespace ssim
