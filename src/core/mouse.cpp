#include <cstdlib>
#include <cstring>

#include <core/mouse.h>
#include <iostream>

namespace ssim {

Mouse::Mouse(AbstractMaze maze) : maze(maze) {}

Mouse::Mouse(AbstractMaze maze, unsigned int starting_row, unsigned int starting_col) : maze(maze), row(starting_row),
                                                                                         col(starting_col),
                                                                                         dir(Direction::E) {}

void Mouse::reset() {
  row = 0;
  col = 0;
  dir = Direction::E;
}

unsigned int Mouse::getRow() const {
  return row;
}

unsigned int Mouse::getCol() const {
  return col;
}

Direction Mouse::getDir() const {
  return dir;
}

void Mouse::internalTurnToFace(Direction dir) {
  this->dir = dir;
}

void Mouse::internalForward() {
  switch (dir) {
    case Direction::N:
      row--;
      break;
    case Direction::E:
      col++;
      break;
    case Direction::S:
      row++;
      break;
    case Direction::W:
      col--;
      break;
    default:
      break;
  }
}

bool Mouse::isWallInDirection(Direction d)  const {
  Node *mouse_node = nullptr;
  if ((maze.get_node(&mouse_node, row, col) == Node::OUT_OF_BOUNDS)) {
    return false;
  }
  bool is_wall = mouse_node->neighbor(d) == nullptr;
  return is_wall;
}

void Mouse::maze_mouse_string(char *buff) const {
  char *b = buff;
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      Node *n = maze.nodes[i][j];
      if (n->neighbor(Direction::W) == NULL) {
        strcpy(b++, "|");
      } else {
        strcpy(b++, "_");
      }

      if (row == i && col == j) {
        switch (dir) {
          case Direction::N:
            strcpy(b++, "^");
            break;
          case Direction::S:
            strcpy(b++, "v");
            break;
          case Direction::E:
            strcpy(b++, ">");
            break;
          case Direction::W:
            strcpy(b++, "<");
            break;
          default:
            strcpy(b++, "o");
            break;
        }
      } else if (n->neighbor(Direction::S) == NULL) {
        strcpy(b++, "_");
      } else {
        strcpy(b++, " ");
      }
    }
    *(b++) = '|';
    *(b++) = '\n';
  }
  b++;
  *b = '\0';
}

} // namespace ssim

