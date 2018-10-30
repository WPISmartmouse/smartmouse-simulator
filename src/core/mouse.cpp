#include <cstdlib>
#include <cstring>

#include <core/mouse.h>
#include <iostream>

namespace ssim {

Mouse::Mouse(AbstractMaze maze) : maze(maze) {}

void Mouse::reset() {
  row_col = {0};
  dir = Direction::E;
}

RowCol Mouse::getRowCol() const {
  return row_col;
}

unsigned int Mouse::getRow() const {
  return row_col.row;
}

unsigned int Mouse::getCol() const {
  return row_col.col;
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
      row_col.row--;
      break;
    case Direction::E:
      row_col.col++;
      break;
    case Direction::S:
      row_col.row++;
      break;
    case Direction::W:
      row_col.col--;
      break;
    default:
      break;
  }
}

bool Mouse::isWallInDirection(Direction d) const {
  return maze.is_wall(row_col, d);
}

void Mouse::maze_mouse_string(char *buff) const {
  char *b = buff;
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      if (maze.is_wall({i, j}, Direction::W)) {
        strcpy(b++, "|");
      } else {
        strcpy(b++, "_");
      }

      if (row_col.row == i && row_col.col == j) {
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
      } else if (maze.is_wall({i, j}, Direction::W)) {
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

