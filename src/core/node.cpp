#include <core/node.h>

namespace ssim {

Node::Node(RowCol row_col) : row_col(row_col) {}

Node::Node(unsigned int const row, unsigned int const col) : row_col{row, col} {}

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
  return row_col.row;
}

unsigned int Node::Col() const {
  return row_col.col;
}

bool Node::operator!=(Node const &other) const {
  return !(operator==(other));
}

bool RowCol::operator==(const RowCol &other) const {
  return row == other.row and col == other.col;
}
} // namespace ssim
