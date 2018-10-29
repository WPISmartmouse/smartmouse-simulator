/** maze.h
 * \brief Contains functions for creating and using the AbstractMaze and Node structs
 */
#pragma once

#include <functional>
#include <stdio.h>
#include <unordered_set>
#include <sstream>
#include <tuple>
#include <vector>

#ifndef ARDUINO

#include <fstream>

#endif

#include "sensor_reading.h"
#include "node.h"
#include "direction.h"

namespace ssim {
struct Wall {
  Wall(unsigned int row, unsigned int col, Direction d);

  Wall() = default;

  unsigned int Row() const;
  unsigned int Col() const;
  Direction Dir() const;

  bool operator==(const Wall &other) const;

 private:
  unsigned int row = 0;
  unsigned int col = 0;
  Direction dir = Direction::N;

};
}

namespace std {
template<>
struct hash<ssim::Wall> {
  size_t operator()(const ssim::Wall &x) const {
    return (x.Row() << 8) + x.Col();
  }
};
}

namespace ssim {

struct MotionPrimitive {
  uint8_t n;
  Direction d;

  std::string to_string() {
    std::stringstream ss;
    ss << (int) n << dir_to_char(d);
    return ss.str();
  }
};

using Route = std::vector<MotionPrimitive>;

std::string route_to_string(Route const &route);

unsigned int expanded_route_length(Route const &route);

void insert_motion_primitive_front(Route *route, MotionPrimitive prim);

void insert_motion_primitive_back(Route *route, MotionPrimitive prim);

unsigned int constexpr static SIZE = 16;
unsigned long const BUFF_SIZE = (SIZE * 2 + 3) * SIZE;
unsigned int constexpr static CENTER = SIZE / 2;
double constexpr static UNIT_DIST_M = 0.18;
double constexpr static WALL_THICKNESS_M = 0.012;
double constexpr static HALF_WALL_THICKNESS_M = WALL_THICKNESS_M / 2.0;
double constexpr static HALF_UNIT_DIST = UNIT_DIST_M / 2.0;
double constexpr static SIZE_M = SIZE * UNIT_DIST_M;

double constexpr toMeters(double cu) noexcept {
  return cu * UNIT_DIST_M;
}

double constexpr toCellUnits(double meters) noexcept {
  static_assert(UNIT_DIST_M > 0, "UNIT_DIST_M must be greater than zero.");
  return meters / UNIT_DIST_M;
}

double constexpr WALL_THICKNESS_CU = toCellUnits(WALL_THICKNESS_M);
double constexpr HALF_WALL_THICKNESS_CU = toCellUnits(HALF_WALL_THICKNESS_M);
double constexpr SIZE_CU = toCellUnits(SIZE_M);

class AbstractMaze {

#ifndef REAL
 public:
  AbstractMaze(std::ifstream &fs);

#endif

 public:

  AbstractMaze();

  static AbstractMaze gen_random_legal_maze();

  static void random_walk(AbstractMaze &maze, unsigned int row, unsigned int col);

  static std::tuple<bool, unsigned int, unsigned int> step(unsigned int row, unsigned int col, Direction d);

  bool operator==(AbstractMaze const &other) const;

  bool out_of_bounds(unsigned int row, unsigned int col) const;

  Node get_node(unsigned int r, unsigned int c) const;

  Node get_node_in_direction(unsigned int row, unsigned int col, Direction dir) const;

  Route truncate_route(unsigned int row, unsigned int col, Direction dir, Route route) const;

  void reset();

  bool is_perimeter(unsigned int row, unsigned int col, Direction dir) const;

  bool is_wall(unsigned int row, unsigned int col, Direction dir) const;

  void add_all_walls();

  void add_wall(unsigned int row, unsigned int col, Direction dir);

  void remove_all_walls();

  void remove_all_walls(unsigned int row, unsigned int col);

  void remove_wall(unsigned int row, unsigned int col, Direction dir);

  void mark_position_visited(unsigned int row, unsigned int col);

  void assign_weights_to_neighbors(Node n, Node goal, int weight, bool &goal_found);

  bool flood_fill_from_point(Route *path, unsigned int r0, unsigned int c0, unsigned int r1, unsigned int c1);

  bool flood_fill_from_origin(Route *path, unsigned int r1, unsigned int c1);

  bool flood_fill_from_origin_to_center(Route *path);

  bool flood_fill(Route *path, unsigned int r0, unsigned int c0, unsigned int r1, unsigned int c1);

  void update(SensorReading sr);

  Route fastest_route = {};
  bool solved = false;

 private:
  std::unordered_set<Wall> walls;
  std::unordered_set<Wall> perimeter;
  std::array<std::array<Node, SIZE>, SIZE> nodes; // array of node pointers
};

}

