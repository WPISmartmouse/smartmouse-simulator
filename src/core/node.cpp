#include <core/node.h>

namespace ssim {

Node::Node(RowCol row_col) : row_col(row_col) {}

Node::Node(MazeIndex const row, MazeIndex const col) : row_col{row, col} {}

bool Node::operator==(Node const &other) const {
  // TODO replace all && || with and or
  return Row() == other.Row() and Col() == other.Col();
}

bool Node::operator!=(Node const &other) const {
  return !(operator==(other));
}

void Node::Reset() {
  weight = std::numeric_limits<decltype(weight)>::max();
  distance = 0;
  known = false;
  visited = false;
}

MazeIndex Node::Row() const {
  return row_col.row;
}

MazeIndex Node::Col() const {
  return row_col.col;
}

RowCol Node::GetRowCol() const {
  return row_col;
}

bool RowCol::operator==(const RowCol &other) const {
  return row == other.row and col == other.col;
}

bool RowCol::operator!=(const RowCol &other) const {
  return row != other.row or col != other.col;
}

} // namespace ssim
