#include <cstdarg>
#include <vector>
#include <cstring>

#include "Arduino.h"

#include <core/mouse.h>

#include <hal/hal.h>

namespace ssim {

void println(const std::string &str) {
  Serial.println(str.c_str());
  Serial1.println(str.c_str());
}

void print(const std::string &str) {
  Serial.print(str.c_str());
  Serial1.print(str.c_str());
}

void print(const char *fmt, ...) {
  char buf[1024]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt);
  vsnprintf(buf, sizeof(buf), (const char *) fmt, args); // for the rest of the world
  va_end(args);
  Serial.print(buf);
  Serial1.print(buf);
}

void csv_print(std::vector<double> values) {
  char buf[1024];
  char *b = (char *) buf;
  for (double value : values) {
    int c = sprintf(b, "%0.3f, ", value);
    b += c;
  }
  sprintf(b, "\r\n");
  Serial.print(buf);
  Serial1.print(buf);
}

void print_maze_mouse(const Mouse &mouse) {
  char buff[BUFF_SIZE];
  mouse.maze_mouse_string(buff);
  print(buff);
}

void print_maze_str(const AbstractMaze &maze, char *buff) {
  char *b = buff;
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      Node *n = maze.nodes[i][j];
      if (n->neighbor(Direction::W) == nullptr) {
        strncpy(b++, "|", 1);
        if (n->neighbor(Direction::S) == nullptr) {
          strncpy(b++, "_", 1);
        } else {
          strncpy(b++, " ", 1);
        }
      } else {
        strcpy(b++, "_");
        if (n->neighbor(Direction::S) == nullptr) {
          strncpy(b++, "_", 1);
        } else {
          strncpy(b++, " ", 1);
        }
      }
    }
    *(b++) = '|';
    *(b++) = '\n';
  }
  b++;
  *b = '\0';
}

void print_maze(AbstractMaze const &maze) {
  char buff[BUFF_SIZE];
  print_maze_str(maze, buff);
  print(buff);
}

void print_neighbor_maze(const AbstractMaze &maze) {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      for (Direction d = Direction::First; d < Direction::Last; d++) {
        bool wall = (maze.nodes[i][j]->neighbor(d) == nullptr);
        print("%i", wall);
      }
      print(" ");
    }
    print("\r\n");
  }
}

void print_weight_maze(const AbstractMaze &maze) {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      int w = maze.nodes[i][j]->weight;
      print("%03u ", w);
    }
    print("\r\n");
  }
}

void print_dist_maze(const AbstractMaze &maze) {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      Node *n = maze.nodes[i][j];
      int d = n->distance;
      if (d < 10) {
        print("  %d ", d);
      } else if (d < 100) {
        print(" %d ", d);
      } else {
        print("%d ", d);
      }
    }
    print("\r\n");
  }
}

void print_pointer_maze(const AbstractMaze &maze) {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      print("%p ", maze.nodes[i][j]);
    }
    print("\r\n");
  }
}
}
