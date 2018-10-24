#include <limits>

#include <core/node.h>

namespace ssim {

int const Node::OUT_OF_BOUNDS = -2;

Node::Node(unsigned int row, unsigned int col) : r(row), c(col) {}

unsigned int Node::row() const {
  return r;
}

unsigned int Node::col() const {
  return c;
}

Node *Node::neighbor(Direction const dir) const {
  switch (dir) {
    case Direction::N:
      return neighbors[0];
    case Direction::E:
      return neighbors[1];
    case Direction::S:
      return neighbors[2];
    case Direction::W:
      return neighbors[3];
    default:
      throw std::invalid_argument("neighbor requested in an invalid direction");
  }
}

bool Node::wall(Direction const dir) const {
  return neighbor(dir) == nullptr;
}

void Node::assign_weights_to_neighbors(Node const *const goal, int const weight, bool *const success) {
  //check all nodes that are unvisited, or would be given a lower weight
  if (!this->known || weight < this->weight) {
    //don't visit it again unless you find a shorter path
    this->known = true;

    //check if path to goal node was found
    if (this == goal) {
      *success = true;
    }

    //update weight
    this->weight = weight;

    //recursive call to explore each neighbors
    int i;
    for (i = 0; i < 4; i++) {
      if (this->neighbors[i]) {
        this->neighbors[i]->assign_weights_to_neighbors(goal, weight + 1, success);
      }
    }
  }
}

} // namespace ssim
