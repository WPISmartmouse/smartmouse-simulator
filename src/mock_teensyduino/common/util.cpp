#include <cstdarg>
#include <vector>

#include <Arduino.h>

#include <core/mouse.h>
#include <hal/util.h>

namespace ssim {

void println(std::string const &str) {
  Serial.println(str);
  Serial1.println(str);
}

void print(std::string const &str) {
  Serial.print(str);
  Serial1.print(str);
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
      if (maze.is_wall(i, j, Direction::W)) {
        strncpy(b++, "|", 1);
        if (maze.is_wall(i, j, Direction::S)) {
          strncpy(b++, "_", 1);
        } else {
          strncpy(b++, " ", 1);
        }
      } else {
        strcpy(b++, "_");
        if (maze.is_wall(i, j, Direction::S)) {
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
        bool wall = maze.is_wall(i, j, d);
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
      int w = maze.get_node(i, j).weight;
      print("%03u ", w);
    }
    print("\r\n");
  }
}

void print_dist_maze(const AbstractMaze &maze) {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      int d = maze.get_node(i, j).distance;
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

} // namespace ssim
