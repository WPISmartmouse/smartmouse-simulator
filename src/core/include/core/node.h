#pragma once

#include <limits>
#include <array>

#include "maze_index.h"

namespace ssim {

struct RowCol {

  RowCol() = default;

  constexpr RowCol(unsigned int const row, unsigned int const col) : row(row), col(col) {}

  constexpr RowCol(MazeIndex row, MazeIndex col) : row(row), col(col) {}

  bool operator==(const RowCol &other) const;

  bool operator!=(const RowCol &other) const;

  MazeIndex row{0u};
  MazeIndex col{0u};
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

  Node(MazeIndex row, MazeIndex col);

  Node() = default;

  bool operator==(Node const &other) const;

  bool operator!=(Node const &other) const;

  MazeIndex Row() const;

  MazeIndex Col() const;

  RowCol GetRowCol() const;

  void Reset();

  int weight = std::numeric_limits<decltype(weight)>::max(); // used for flood-fill
  int distance = 0; // used for A-star
  bool known = false;
  bool visited = false;
  std::array<WallEnum, 4> walls = {WallEnum::NoWall, WallEnum::NoWall, WallEnum::NoWall, WallEnum::NoWall};

 private:
  RowCol row_col;
};

constexpr RowCol const ORIGIN{0, 0};

} // namespace ssim
