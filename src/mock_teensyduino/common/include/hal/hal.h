#pragma once

#include <vector>

#include <core/maze.h>

namespace ssim {

void println(std::string const &);

void print(std::string const &);

void print(char const *fmt, ...);

void csv_print(std::vector<double> values);

void print_maze_mouse();

void print_maze(AbstractMaze const &maze);

void print_maze_str(char *buff);

void print_pointer_maze();

/** prints each node as a list of booleans
 * EX)  0010 would mean on wall South 1011 would mean walls to the North, South, and West
*/
void print_neighbor_maze();

void print_weight_maze();

void print_dist_maze();

// FIXME: this 25 should be a parameter
#define print_slow(fmt, ...) {static unsigned long __idx=0;__idx++; if (__idx%25==0) {print(fmt, __VA_ARGS__);}}

}
