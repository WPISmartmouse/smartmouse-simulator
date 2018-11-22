/** maze.h
 * \brief Contains functions for creating and using the AbstractMaze and Node structs
 */
#pragma once

#include <functional>
#include <stdio.h>
#include <sstream>
#include <tuple>
#include <vector>

#ifndef ARDUINO

#include <fstream>

#endif

#include "maze_index.h"
#include "sensor_reading.h"
#include "node.h"
#include "direction.h"

namespace ssim {

struct MotionPrimitive {
  uint8_t n = 0;
  Direction d = Direction::N;

  std::string to_string() {
    std::stringstream ss;
    ss << (int) n << dir_to_char(d);
    return ss.str();
  }
};

struct StepResult {
  bool const valid = false;
  RowCol const row_col;
};

using Route = std::vector<MotionPrimitive>;

std::string route_to_string(Route const &route);

unsigned int expanded_route_length(Route const &route);

void insert_motion_primitive_front(Route *route, MotionPrimitive prim);

void insert_motion_primitive_back(Route *route, MotionPrimitive prim);

unsigned long const BUFF_SIZE = (SIZE * 2 + 3) * SIZE;
constexpr auto static CENTER = RowCol{SIZE/2, SIZE/2};
constexpr double static UNIT_DIST_M = 0.18;
constexpr double static WALL_THICKNESS_M = 0.012;
constexpr double static HALF_WALL_THICKNESS_M = WALL_THICKNESS_M / 2.0;
constexpr double static HALF_UNIT_DIST = UNIT_DIST_M / 2.0;
constexpr double static SIZE_M = SIZE * UNIT_DIST_M;

constexpr double toMeters(double cu) noexcept {
  return cu * UNIT_DIST_M;
}

constexpr double toCellUnits(double meters) noexcept {
  static_assert(UNIT_DIST_M > 0, "UNIT_DIST_M must be greater than zero.");
  return meters / UNIT_DIST_M;
}

constexpr double WALL_THICKNESS_CU = toCellUnits(WALL_THICKNESS_M);
constexpr double HALF_WALL_THICKNESS_CU = toCellUnits(HALF_WALL_THICKNESS_M);
constexpr double SIZE_CU = toCellUnits(SIZE_M);

class AbstractMaze {

#ifndef REAL
 public:
  AbstractMaze(std::ifstream &fs);

#endif

 public:

  AbstractMaze();

  static AbstractMaze gen_random_legal_maze();

  static void random_walk(AbstractMaze &maze, RowCol start);

  static StepResult const step(RowCol start, Direction d);

  Node get_node(RowCol row_col) const;

  Route truncate_route(RowCol start, Route route) const;

  void reset();

  bool is_perimeter(RowCol row_col, Direction dir) const;

  bool is_wall(RowCol row_col, Direction dir) const;

  void add_all_walls();

  void add_wall(RowCol row_col, Direction dir);

  void remove_all_walls();

  void remove_wall(RowCol row_col, Direction dir);

  void remove_wall_if_exists(RowCol row_col, Direction dir);

  void mark_position_visited(RowCol row_col);

  void assign_weights_to_neighbors(RowCol rc, RowCol goal, int weight, bool &goal_found);

  bool flood_fill_from_point(Route *path, RowCol start, RowCol goal);

  bool flood_fill_from_origin(Route *path, RowCol goal);

  bool flood_fill_from_origin_to_center(Route *path);

  bool flood_fill(Route *path, RowCol start, RowCol goal);

  void update(SensorReading sr);

  Route fastest_route = {};
  bool solved = false;

 private:

  Node &get_mutable_node(RowCol row_col);

  std::array<std::array<Node, SIZE>, SIZE> nodes;
};

}

