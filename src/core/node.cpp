#include <core/node.h>

namespace ssim {

Node::Node(unsigned int row, unsigned int col) : r(row), c(col) {}

bool Node::operator==(Node const &other) const {
  return Row() == other.Row() and Col() == other.Col();
}

void Node::Reset() {
  weight = std::numeric_limits<decltype(weight)>::max();
  distance = 0;
  known = false;
  visited = false;
}

unsigned int Node::Row() const {
  return r;
}

unsigned int Node::Col() const {
  return c;
}

bool Node::operator!=(Node const &other) const {
  return !(operator==(other));
}

} // namespace ssim
