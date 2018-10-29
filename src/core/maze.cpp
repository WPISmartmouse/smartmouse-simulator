#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <random>
#include <sstream>
#include <string>
#include <iostream>

#include <fmt/format.h>

#include <core/maze.h>

namespace ssim {

bool Wall::operator==(const Wall &other) const {
  return row == other.row and col == other.col and dir == other.dir;
}

Wall::Wall(unsigned int row, unsigned int col, Direction d) : row(row), col(col), dir(d) {}

unsigned int Wall::Row() const {
  return row;
}

unsigned int Wall::Col() const {
  return col;
}

Direction Wall::Dir() const {
  return dir;
}

AbstractMaze::AbstractMaze() {
  for (unsigned int i = 0; i < SIZE; i++) {
    perimeter.emplace(0, i, Direction::N);
    perimeter.emplace(SIZE - 1, i, Direction::S);
    perimeter.emplace(i, SIZE - 1, Direction::E);
    perimeter.emplace(i, 0, Direction::W);
  }
}

#ifndef REAL

AbstractMaze::AbstractMaze(std::ifstream &fs) : AbstractMaze() {
  std::string line;

  add_all_walls();

  //look West and South to connect any nodes
  for (unsigned int i = 0; i < SIZE; i++) {
    std::getline(fs, line);

    if (!fs.good()) {
      std::cerr << "getline failed\n";
      return;
    }

    unsigned int charPos = 0;
    for (unsigned int j = 0; j < SIZE; j++) {
      if (line.at(charPos) != '|') {
        remove_wall(i, j, Direction::W);
      }
      charPos++;
      if (line.at(charPos) != '_') {
        remove_wall(i, j, Direction::S);
      }
      charPos++;
    }
  }
}

#endif

void AbstractMaze::random_walk(AbstractMaze &maze, unsigned int const row, unsigned int col) {
  static std::random_device rd;
  static std::mt19937 g(rd());

  maze.mark_position_visited(row, col);

  // shuffle directions
  std::vector<Direction> dirs = {Direction::N, Direction::E, Direction::S, Direction::W};
  std::shuffle(dirs.begin(), dirs.end(), g);

  for (auto d : dirs) {
    auto n = maze.get_node_in_direction(row, col, d);
    if (!n.visited) {
      maze.remove_wall(row, col, d);
      random_walk(maze, row, col);
    }
  }
}

std::tuple<bool, unsigned int, unsigned int>
AbstractMaze::step(unsigned int const row, unsigned int const col, Direction const d) {
  using t = std::tuple<bool, unsigned int, unsigned int>;
  switch (d) {
    case Direction::N:
      return row >= 0 ? t{true, row - 1, col} : t{false, 0, 0};
    case Direction::S:
      return row < SIZE - 1 ? t{true, row + 1, col} : t{false, 0, 0};
    case Direction::E:
      return col < SIZE - 1 ? t{true, row, col + 1} : t{false, 0, 0};
    case Direction::W:
      return col > 0 ? t{true, row, col - 1} : t{false, 0, 0};
    default:
      throw std::invalid_argument(fmt::format("direction {} is invalid", dir_to_char(d)));
  }
}

bool AbstractMaze::operator==(AbstractMaze const &other) const {
  return walls == other.walls;
}

bool AbstractMaze::out_of_bounds(unsigned int row, unsigned int col) const {
  return col < 0 or col >= SIZE or row < 0 or row >= SIZE;
}

Node AbstractMaze::get_node(unsigned int const row, unsigned int const col) const {
  if (out_of_bounds(row, col)) {
    throw std::invalid_argument(fmt::format("get_node: row {} or col {} is out of bounds", row, col));
  }
  return nodes[row][col];
}

Node AbstractMaze::get_node_in_direction(unsigned int const row, unsigned int const col, Direction const dir) const {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  const auto[valid, row_in_dir, col_in_dir] = step(row, col, dir);
  if (!valid) {
    throw std::invalid_argument(
        fmt::format("get_node_in_dir: row {} or col {} in direction {} is out of bounds", row, col, dir_to_char(dir)));
  }
  return get_node(row_in_dir, col_in_dir);
}

bool AbstractMaze::is_perimeter(unsigned int row, unsigned int col, Direction dir) const {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  if (row == 0 and dir == Direction::N) {
    return true;
  } else if (row == SIZE - 1 and dir == Direction::S) {
    return true;
  } else if (col == 0 and dir == Direction::W) {
    return true;
  } else if (col == SIZE - 1 and dir == Direction::E) {
    return true;
  }
  return false;
}

bool AbstractMaze::is_wall(unsigned int const row, unsigned int const col, Direction const dir) const {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  auto const wall_it = walls.find({.row=row, .col=col, .dir=dir});
  auto const perimeter_it = perimeter.find({.row=row, .col=col, .dir=dir});
  return wall_it != walls.cend() or perimeter_it != perimeter.cend();
}

void AbstractMaze::reset() {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      nodes[i][j].Reset();
    }
  }
}

void AbstractMaze::add_wall(unsigned int const row, unsigned int const col, Direction const dir) {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  if (out_of_bounds(row, col)) {
    throw std::invalid_argument(fmt::format("add_wall: row {} or col {} is out of bounds", row, col));
  }

  if (!is_perimeter(row, col, dir)) {
    walls.emplace(row, col, dir);
  }

  // Add the wall from the other side if that's possible
  const auto[valid, row_in_dir, col_in_dir] = step(row, col, dir);
  if (valid and !is_perimeter(row_in_dir, col_in_dir, opposite_direction(dir))) {
    walls.emplace(row_in_dir, col_in_dir, opposite_direction(dir));
  }
}

void AbstractMaze::remove_wall(unsigned int const row, unsigned int const col, Direction const dir) {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  {
    if (is_perimeter(row, col, dir)) {
      throw std::invalid_argument(
          fmt::format("row {} or col {} direction {} is a perimeter, which cannot be remove", row, col,
                      dir_to_char(dir)));

    }
    const auto it = walls.find({.row=row, .col=col, .dir=dir});
    if (it == walls.cend()) {
      throw std::invalid_argument(
          fmt::format("remove_wall: row {} col {} dir {} not in walls", row, col, dir_to_char(dir)));
    }
    walls.erase(it);
  }

  // Remove the wall from the other side if that's possible
  const auto[valid, row_in_dir, col_in_dir] = step(row, col, dir);
  if (valid) {
    const auto it = walls.find({.row=row_in_dir, .col=col_in_dir, .dir=opposite_direction(dir)});
    if (it != walls.cend()) {
      walls.erase(it);
    }
  }
}

void AbstractMaze::remove_wall_if_exists(unsigned int const row, unsigned int const col, ssim::Direction const dir) {
  try {
    remove_wall(row, col, dir);
  }
  catch (std::invalid_argument const &e) {
  }
}

void AbstractMaze::remove_all_walls() {
  walls.clear();
}

void AbstractMaze::add_all_walls() {
  for (unsigned int row = 0; row < SIZE; row++) {
    for (unsigned int col = 0; col < SIZE; col++) {
      for (Direction d = Direction::First; d != Direction::Last; d++) {
        if (!is_perimeter(row, col, d)) {
          walls.emplace(row, col, d);
        }
      }
    }
  }
}

void AbstractMaze::assign_weights_to_neighbors(Node n, Node goal, int weight, bool &goal_found) {
  // check all nodes that are unvisited, or would be given a lower weight
  if (!n.known || weight < n.weight) {
    //don't visit it again unless you find a shorter path
    n.known = true;

    // check if path to goal node was found
    if (n == goal) {
      goal_found = true;
    }

    // update weight
    n.weight = weight;

    // recursive call to explore each neighbors
    for (Direction d = Direction::First; d != Direction::Last; d++) {
      const auto[valid, row_in_dir, col_in_dir] = step(n.Row(), n.Col(), d);
      if (!valid) {
        auto next_n = get_node(row_in_dir, col_in_dir);
        assign_weights_to_neighbors(next_n, goal, weight + 1, goal_found);
      }
    }
  }
}

bool
AbstractMaze::flood_fill_from_point(Route *const path, unsigned int const r0, unsigned int const c0,
                                    unsigned int const r1, unsigned int const c1) {
  return flood_fill(path, r0, c0, r1, c1);
}

bool AbstractMaze::flood_fill_from_origin(Route *const path, unsigned int const r1, unsigned int const c1) {
  return flood_fill(path, 0, 0, r1, c1);
}

bool AbstractMaze::flood_fill_from_origin_to_center(Route *const path) {
  return flood_fill(path, 0, 0, SIZE / 2, SIZE / 2);
}

bool AbstractMaze::flood_fill(Route *const path, unsigned int const r0, unsigned int const c0, unsigned int const r1,
                              unsigned int const c1) {
  Node n;
  Node goal = nodes[r1][c1];

  // in case the maze has already been solved, reset all weight and known values
  reset();

  if (r0 == r1 && c0 == c1) {
    return true;
  }

  //explore all neighbors of the current node starting  with a weight of 1
  //return 1 means path to goal was found
  bool solvable = true;

  //start at the goal
  n = goal;

  // recursively visits all neighbors
  bool goal_found = false;
  assign_weights_to_neighbors(n, goal, 0, goal_found);
  path->clear();

  //if we solved the maze, traverse from goal back to root and record what direction is shortest
  while (n != nodes[r0][c0] && solvable) {
    Node min_node = n;
    Direction min_dir = Direction::N;

    //find the neighbor with the lowest weight and go there,  that is the fastest route
    Direction d;
    bool deadend = true;
    for (d = Direction::First; d < Direction::Last; d++) {
      const auto[valid, row_in_dir, col_in_dir] = step(n.Row(), n.Col(), d);
      if (valid) {
        auto next_n = get_node(row_in_dir, col_in_dir);
        if (next_n.weight < min_node.weight) {
          min_node = next_n;
          min_dir = opposite_direction(d);
          deadend = false;
        }
      }
    }

    if (deadend) {
      solvable = false;
    }

    n = min_node;

    insert_motion_primitive_front(path, {1, min_dir});
  }

  return solvable;
}

void AbstractMaze::mark_position_visited(unsigned int const row, unsigned int const col) {
  nodes[row][col].visited = true;
}

AbstractMaze AbstractMaze::gen_random_legal_maze() {
  AbstractMaze maze;

  std::random_device rd;
  std::mt19937 g(rd());

  // start at center and move out, marking visited nodes as we go
  maze.mark_position_visited(SIZE / 2, SIZE / 2);
  maze.mark_position_visited(SIZE / 2 - 1, SIZE / 2);
  maze.mark_position_visited(SIZE / 2, SIZE / 2 - 1);
  maze.mark_position_visited(SIZE / 2 - 1, SIZE / 2 - 1);

  // pick std::random start node of the four possible ones;
  unsigned int starting_row = SIZE / 2 - g() % 2;
  unsigned int starting_col = SIZE / 2 - g() % 2;
  random_walk(maze, starting_row, starting_col);

  // knock down some more randomly
  unsigned int i = 0;
  while (i < SIZE * SIZE / 5) {
    unsigned int const row = g() % (SIZE - 2) + 1;
    unsigned int const col = g() % (SIZE - 2) + 1;
    Direction dir = int_to_dir(g() % 4);

    // check if that's a valid wall to knock down
    bool can_delete = false;

    switch (dir) {
      case Direction::N:
        if ((maze.is_wall(row, col, Direction::W) or maze.is_wall(row, col - 1, Direction::N) or
             maze.is_wall(row - 1, col, Direction::W)) &&
            (maze.is_wall(row, col, Direction::E) or maze.is_wall(row, col + 1, Direction::N) or
             maze.is_wall(row - 1, col, Direction::E))) {
          can_delete = true;
        }
        break;
      case Direction::E:
        if ((maze.is_wall(row, col, Direction::N) or maze.is_wall(row, col + 1, Direction::N) or
             maze.is_wall(row - 1, col, Direction::E)) &&
            (maze.is_wall(row, col, Direction::S) or maze.is_wall(row, col + 1, Direction::S) or
             maze.is_wall(row + 1, col, Direction::E))) {
          can_delete = true;
        }
        break;
      case Direction::S:
        if ((maze.is_wall(row, col, Direction::W) or maze.is_wall(row, col - 1, Direction::S) or
             maze.is_wall(row + 1, col, Direction::W)) &&
            (maze.is_wall(row, col, Direction::E) or maze.is_wall(row, col + 1, Direction::S) or
             maze.is_wall(row + 1, col, Direction::E))) {
          can_delete = true;
        }
        break;
      case Direction::W:
        if ((maze.is_wall(row, col, Direction::N) or maze.is_wall(row, col - 1, Direction::N) or
             maze.is_wall(row - 1, col, Direction::W)) &&
            (maze.is_wall(row, col, Direction::S) or maze.is_wall(row, col - 1, Direction::S) or
             maze.is_wall(row + 1, col, Direction::W))) {
          can_delete = true;
        }
        break;
      default:
        break;
    }

    if (can_delete) {
      maze.remove_wall(row, col, dir);
      i++;
    }
  }

  // knock down center square
  maze.remove_wall(SIZE / 2, SIZE / 2, Direction::N);
  maze.remove_wall(SIZE / 2, SIZE / 2, Direction::W);
  maze.remove_wall(SIZE / 2 - 1, SIZE / 2 - 1, Direction::S);
  maze.remove_wall(SIZE / 2 - 1, SIZE / 2 - 1, Direction::E);

  return maze;
}

std::string route_to_string(Route const &route) {
  std::stringstream ss;
  if (route.empty()) {
    ss << "empty";
  }

  for (MotionPrimitive prim : route) {
    ss << (int) prim.n << dir_to_char(prim.d);
  }

  return ss.str();
}

unsigned int expanded_route_length(Route const &route) {
  return std::accumulate(route.begin(), route.end(), 0u,
                         [](unsigned int accum, const MotionPrimitive &prim) { return accum + prim.n; });
}

void insert_motion_primitive_back(Route *route, MotionPrimitive const prim) {
  if (!route->empty() && prim.d == route->back().d) {
    route->back().n += prim.n;
  } else {
    route->insert(route->cend(), prim);
  }
}

void insert_motion_primitive_front(Route *route, MotionPrimitive const prim) {
  if (!route->empty() && prim.d == route->front().d) {
    route->front().n += prim.n;
  } else {
    route->insert(route->cbegin(), prim);
  }
}

Route AbstractMaze::truncate_route(unsigned int const row, unsigned int const col, Direction const dir,
                                   Route const route) const {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  Route trunc;
  auto r = row;
  auto c = col;
  bool done = false;
  bool valid;
  for (MotionPrimitive prim : route) {
    for (unsigned int i = 0; i < prim.n; i++) {
      // check if this move is valid
      std::tie(valid, r, c) = step(r, c, prim.d);
      if (!valid) {
        done = true;
        break;
      }

      insert_motion_primitive_back(&trunc, {1, prim.d});
    }
    if (done) {
      break;
    }
  }
  return trunc;
}

void AbstractMaze::update(SensorReading const sr) {
  for (Direction d = Direction::First; d < Direction::Last; d++) {
    //if a wall exists in that direction, add a wall
    //if no wall exists in that direction remove it
    if (sr.isWall(d)) {
      add_wall(sr.row, sr.col, d);
    } else {
      remove_wall_if_exists(sr.row, sr.col, d);
    }
  }
}

} // namespace ssim
