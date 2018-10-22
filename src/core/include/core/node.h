#pragma once

#include <array>
#include <limits>

#include <core/direction.h>

namespace ssim {

/**
 * \brief holds its location & neighbors, as well as a bool for indicating if it has been discovered
 * you don't need to free nodes in a maze, just use free_maze however, be sure to free nodes allocated not in mazes
 * visited is meant for ACTUALLY visiting, known is just used for searching/solving
 */
class Node {
public:
  int weight = std::numeric_limits<decltype(weight)>::max(); //used for flood-fill
  int distance = 0; //used for A-star
  bool known = false;
  bool visited = false;

  //if you want to iterate over neighbors, just increment the pointer to north
  std::array<Node *, 4> neighbors = {};

  int const static OUT_OF_BOUNDS;

  /** \brief intializes a node */
  Node(unsigned int row, unsigned int col);

  Node() = default;

  unsigned int row() const;

  unsigned int col() const;

  /** \brief get the neighbor in the given direction */
  Node *neighbor(Direction dir) const;

  bool wall(Direction dir) const;

  void assign_weights_to_neighbors(const Node *goal, int weight, bool *success);

private:
  unsigned int r = 0;
  unsigned int c = 0;
};

} // namespace ssim
