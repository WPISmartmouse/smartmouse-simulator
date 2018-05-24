#include "hal.h"

namespace ssim {

global_program_settings_t GlobalProgramSettings;

// FIXME: I forget what this was for
extern "C" {
int _getpid() { return -1; }
int _kill(int pid, int sig) { return -1; }
void _write_r() {}
}

void print_maze_mouse(const AbstractMouse &mouse) {
  char buff[smartmouse::maze::BUFF_SIZE];
  mouse.maze_mouse_string(buff);
  print(buff);
}

void print_maze_str(char *buff) {
  char *b = buff;
  unsigned int i, j;
  for (i = 0; i < smartmouse::maze::SIZE; i++) {
    for (j = 0; j < smartmouse::maze::SIZE; j++) {
      Node *n = nodes[i][j];
      if (n->neighbor(Direction::W) == nullptr) {
        strcpy(b++, "|");
        if (n->neighbor(Direction::S) == nullptr) {
          strcpy(b++, "_");
        } else {
          strcpy(b++, " ");
        }
      } else {
        strcpy(b++, "_");
        if (n->neighbor(Direction::S) == nullptr) {
          strcpy(b++, "_");
        } else {
          strcpy(b++, " ");
        }
      }
    }
    *(b++) = '|';
    *(b++) = '\n';
  }
  b++;
  *b = '\0';
}

void print_maze() {
  char buff[smartmouse::maze::BUFF_SIZE];
  print_maze_str(buff);
  print(buff);
}

void print_neighbor_maze() {
  unsigned int i, j;
  for (i = 0; i < smartmouse::maze::SIZE; i++) {
    for (j = 0; j < smartmouse::maze::SIZE; j++) {
      for (Direction d = Direction::First; d < Direction::Last; d++) {
        bool wall = (nodes[i][j]->neighbor(d) == nullptr);
        print("%i", wall);
      }
      print(" ");
    }
    print("\r\n");
  }
}

void print_weight_maze() {
  unsigned int i, j;
  for (i = 0; i < smartmouse::maze::SIZE; i++) {
    for (j = 0; j < smartmouse::maze::SIZE; j++) {
      int w = nodes[i][j]->weight;
      print("%03u ", w);
    }
    print("\r\n");
  }
}

void print_dist_maze() {
  unsigned int i, j;
  for (i = 0; i < smartmouse::maze::SIZE; i++) {
    for (j = 0; j < smartmouse::maze::SIZE; j++) {
      Node *n = nodes[i][j];
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

void print_pointer_maze() {
  unsigned int i, j;
  for (i = 0; i < smartmouse::maze::SIZE; i++) {
    for (j = 0; j < smartmouse::maze::SIZE; j++) {
      print("%p ", nodes[i][j]);
    }
    print("\r\n");
  }
}
}
