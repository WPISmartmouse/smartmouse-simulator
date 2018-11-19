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

AbstractMaze::AbstractMaze() {
  // Set the row and column of all the nodes
  for (unsigned int row = 0; row < SIZE; ++row) {
    for (unsigned int col = 0; col < SIZE; ++col) {
      // This will construct new nodes!
      nodes[row][col] = RowCol{row, col};
    }
  }

  for (unsigned int i = 0; i < SIZE; i++) {
    nodes[0][i].walls[static_cast<int>(Direction::N)] = WallEnum::PerimeterWall;
    nodes[SIZE - 1][i].walls[static_cast<int>(Direction::S)] = WallEnum::PerimeterWall;
    nodes[i][SIZE - 1].walls[static_cast<int>(Direction::E)] = WallEnum::PerimeterWall;
    nodes[i][0].walls[static_cast<int>(Direction::W)] = WallEnum::PerimeterWall;
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
        remove_wall({i, j}, Direction::W);
      }
      charPos++;
      if (line.at(charPos) != '_') {
        remove_wall({i, j}, Direction::S);
      }
      charPos++;
    }
  }
}

#endif

void AbstractMaze::random_walk(AbstractMaze &maze, RowCol const row_col) {
  static std::random_device rd;
  static std::mt19937 g(rd());

  maze.mark_position_visited(row_col);

  // shuffle directions
  std::vector<Direction> dirs = {Direction::N, Direction::E, Direction::S, Direction::W};
  std::shuffle(dirs.begin(), dirs.end(), g);

  for (auto d : dirs) {
    auto const[valid, next_row_col] = step(row_col, d);
    if (valid) {
      auto next_n = maze.get_node(next_row_col);
      if (!next_n.visited) {
        maze.remove_wall_if_exists(row_col, d);
        random_walk(maze, next_row_col);
      }
    }
  }
}

StepResult const AbstractMaze::step(RowCol const row_col, Direction const d) {
  switch (d) {
    case Direction::N:
      return row_col.row > 0 ? StepResult{true, {row_col.row - 1, row_col.col}} : StepResult{false, {0, 0}};
    case Direction::S:
      return row_col.row < SIZE - 1 ? StepResult{true, {row_col.row + 1, row_col.col}} : StepResult{false, {0, 0}};
    case Direction::E:
      return row_col.col < SIZE - 1 ? StepResult{true, {row_col.row, row_col.col + 1}} : StepResult{false, {0, 0}};
    case Direction::W:
      return row_col.col > 0 ? StepResult{true, {row_col.row, row_col.col - 1}} : StepResult{false, {0, 0}};
    default:
      throw std::invalid_argument(fmt::format("direction {} is invalid", dir_to_char(d)));
  }
}

bool AbstractMaze::out_of_bounds(RowCol const row_col) const {
  return row_col.col >= SIZE or row_col.row >= SIZE;
}

Node AbstractMaze::get_node(RowCol const row_col) const {
  if (out_of_bounds(row_col)) {
    throw std::invalid_argument(fmt::format("get_node: row {} or col {} is out of bounds", row_col.row, row_col.col));
  }
  return nodes[row_col.row][row_col.col];
}

Node AbstractMaze::get_node_in_direction(RowCol const row_col, Direction const dir) const {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  auto const[valid, new_row_col] = step(row_col, dir);
  if (!valid) {
    throw std::invalid_argument(
        fmt::format("get_node_in_dir: row {} or col {} in direction {} is out of bounds", new_row_col.row,
                    new_row_col.row, dir_to_char(dir)));
  }
  return get_node(new_row_col);
}

bool AbstractMaze::is_perimeter(RowCol const row_col, Direction dir) const {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  if (row_col.row == 0 and dir == Direction::N) {
    return true;
  } else if (row_col.row == SIZE - 1 and dir == Direction::S) {
    return true;
  } else if (row_col.col == 0 and dir == Direction::W) {
    return true;
  } else if (row_col.col == SIZE - 1 and dir == Direction::E) {
    return true;
  }
  return false;
}

bool AbstractMaze::is_wall(RowCol const row_col, Direction const dir) const {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  auto const is_wall = nodes[row_col.row][row_col.col].walls[static_cast<int>(dir)];
  switch (is_wall) {
    case WallEnum::Wall:
      return true;
    case WallEnum::PerimeterWall:
      return true;
    case WallEnum::NoWall:
      return false;
  }
  throw std::runtime_error("wall enum case not handled");
}

void AbstractMaze::reset() {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      nodes[i][j].Reset();
    }
  }
}

void AbstractMaze::add_wall(RowCol const row_col, Direction const dir) {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  if (out_of_bounds(row_col)) {
    throw std::invalid_argument(fmt::format("add_wall: row {} or col {} is out of bounds", row_col.row, row_col.col));
  }

  if (!is_perimeter(row_col, dir)) {
    nodes[row_col.row][row_col.col].walls[static_cast<int>(dir)] = WallEnum::Wall;
  }

  // Add the wall from the other side if that's possible
  auto const[valid, new_row_col] = step(row_col, dir);
  if (valid and !is_perimeter(new_row_col, opposite_direction(dir))) {
    nodes[new_row_col.row][new_row_col.col].walls[static_cast<int>(opposite_direction(dir))] = WallEnum::PerimeterWall;
  }
}

void AbstractMaze::remove_wall(RowCol const row_col, Direction const dir) {
  if (out_of_bounds(row_col)) {
    throw std::invalid_argument(fmt::format("cannot remove wall out of bounds: {}, {}", row_col.row, row_col.col));
  }

  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Direction::Last");
  }

  {
    if (is_perimeter(row_col, dir)) {
      throw std::invalid_argument(
          fmt::format("row {} or col {} direction {} is a perimeter, which cannot be remove", row_col.row, row_col.col,
                      dir_to_char(dir)));

    }
    if (nodes[row_col.row][row_col.col].walls[static_cast<int>(dir)] == WallEnum::NoWall) {
      throw std::invalid_argument(
          fmt::format("remove_wall: row {} col {} dir {} not in walls", row_col.row, row_col.col, dir_to_char(dir)));
    }
    nodes[row_col.row][row_col.col].walls[static_cast<int>(dir)] = WallEnum::NoWall;
  }

  // Remove the wall from the other side if that's possible
  auto const[valid, new_row_col] = step(row_col, dir);
  if (valid) {
    nodes[new_row_col.row][new_row_col.col].walls[static_cast<int>(opposite_direction(dir))] = WallEnum::NoWall;
  }
}

void AbstractMaze::remove_wall_if_exists(RowCol const row_col, ssim::Direction const dir) {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  {
    if (is_perimeter(row_col, dir)) {
      throw std::invalid_argument(
          fmt::format("row {} or col {} direction {} is a perimeter, which cannot be remove", row_col.row, row_col.col,
                      dir_to_char(dir)));

    }

    if (nodes[row_col.row][row_col.col].walls[static_cast<int>(dir)] == WallEnum::NoWall) {
      // this case is fine. The whole point of this function is to ignore this
      return;
    }
    nodes[row_col.row][row_col.col].walls[static_cast<int>(dir)] = WallEnum::NoWall;
  }

  // Remove the wall from the other side if that's possible
  auto const[valid, new_row_col] = step(row_col, dir);
  if (valid) {
    nodes[new_row_col.row][new_row_col.col].walls[static_cast<int>(opposite_direction(dir))] = WallEnum::NoWall;
  }
}

void AbstractMaze::remove_all_walls() {
  for (unsigned int row = 0; row < SIZE; row++) {
    for (unsigned int col = 0; col < SIZE; col++) {
      for (Direction d = Direction::First; d != Direction::Last; d++) {
        if (!is_perimeter({row, col}, d)) {
          nodes[row][col].walls[static_cast<int>(d)] = WallEnum::NoWall;
        }
      }
    }
  }
}

void AbstractMaze::add_all_walls() {
  for (unsigned int row = 0; row < SIZE; row++) {
    for (unsigned int col = 0; col < SIZE; col++) {
      for (Direction d = Direction::First; d != Direction::Last; d++) {
        if (!is_perimeter({row, col}, d)) {
          nodes[row][col].walls[static_cast<int>(d)] = WallEnum::Wall;
        }
      }
    }
  }
}

void
AbstractMaze::assign_weights_to_neighbors(RowCol const start, RowCol const goal, int const weight, bool &goal_found) {
  // stop early if we find that a node has higher weight than our goal
  if (weight > nodes[goal.row][goal.col].weight) {
    return;
  }

  // check all nodes that are unvisited, or would be given a lower weight
  auto &n = nodes[start.row][start.col];
  if (!n.known || weight < n.weight) {

    // don't visit it again unless you find a shorter path
    n.known = true;

    // check if path to goal node was found
    if (n == goal) {
      goal_found = true;
    }

    // update weight
    n.weight = weight;

    // recursive call to explore each neighbors
    for (Direction d = Direction::First; d != Direction::Last; d++) {
      auto const[valid, new_row_col] = step(start, d);
      auto const wall = is_wall(start, d);
      if (valid and not wall) {
        assign_weights_to_neighbors(new_row_col, goal, weight + 1, goal_found);
      }
    }
  }
}

bool
AbstractMaze::flood_fill_from_point(Route *const path, RowCol const start, RowCol const goal) {
  return flood_fill(path, start, goal);
}

bool AbstractMaze::flood_fill_from_origin(Route *const path, RowCol const goal) {
  return flood_fill(path, {0, 0}, goal);
}

bool AbstractMaze::flood_fill_from_origin_to_center(Route *const path) {
  return flood_fill(path, {0, 0}, {SIZE / 2, SIZE / 2});
}

bool AbstractMaze::flood_fill(Route *const path, RowCol const start, RowCol const goal) {
  // in case the maze has already been solved, reset all weight and known values
  reset();

  if (start == goal) {
    return true;
  }

  //explore all neighbors of the current node starting  with a weight of 1
  //return 1 means path to goal was found
  bool solvable = true;

  // recursively visits all neighbors
  bool goal_found = false;
  assign_weights_to_neighbors(start, goal, 0, goal_found);
  path->clear();

  // if we solved the maze, traverse from goal back to root and record what direction is shortest
  Node n = nodes[goal.row][goal.col];
  while (n != nodes[start.row][start.col] && solvable) {
    Node min_node = n;
    Direction min_dir = Direction::N;

    //find the neighbor with the lowest weight and go there,  that is the fastest route
    Direction d;
    bool deadend = true;
    for (d = Direction::First; d < Direction::Last; d++) {
      RowCol const current_rc = n.GetRowCol();
      auto const[valid, new_row_col] = step(current_rc, d);
      auto const wall = is_wall(current_rc, d);
      if (valid and not wall) {
        auto next_n = get_node(new_row_col);
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

void AbstractMaze::mark_position_visited(RowCol const row_col) {
  nodes[row_col.row][row_col.col].visited = true;
}

AbstractMaze AbstractMaze::gen_random_legal_maze() {
  AbstractMaze maze;

  maze.add_all_walls();

  static std::random_device rd;
  static std::mt19937 g(rd());
  static std::uniform_int_distribution<int> uid(1, 16);

  // start at center and move out, marking visited nodes as we go
  maze.mark_position_visited({SIZE / 2, SIZE / 2});
  maze.mark_position_visited({SIZE / 2 - 1, SIZE / 2});
  maze.mark_position_visited({SIZE / 2, SIZE / 2 - 1});
  maze.mark_position_visited({SIZE / 2 - 1, SIZE / 2 - 1});

  // pick std::random start node of the four possible ones;
  unsigned int starting_row = SIZE / 2 - uid(g) % 2;
  unsigned int starting_col = SIZE / 2 - uid(g) % 2;
  random_walk(maze, {starting_row, starting_col});

  // knock down some more randomly
  unsigned int i = 0;
  while (i < SIZE * SIZE / 5) {
    unsigned int const row = uid(g) % (SIZE - 2) + 1;
    unsigned int const col = uid(g) % (SIZE - 2) + 1;
    Direction dir = int_to_dir(uid(g) % 4);

    // check if that's a valid wall to knock down
    bool can_delete = false;

    switch (dir) {
      case Direction::N:
        if ((maze.is_wall({row, col}, Direction::W) or maze.is_wall({row, col - 1}, Direction::N) or
             maze.is_wall({row - 1, col}, Direction::W)) &&
            (maze.is_wall({row, col}, Direction::E) or maze.is_wall({row, col + 1}, Direction::N) or
             maze.is_wall({row - 1, col}, Direction::E))) {
          can_delete = true;
        }
        break;
      case Direction::E:
        if ((maze.is_wall({row, col}, Direction::N) or maze.is_wall({row, col + 1}, Direction::N) or
             maze.is_wall({row - 1, col}, Direction::E)) &&
            (maze.is_wall({row, col}, Direction::S) or maze.is_wall({row, col + 1}, Direction::S) or
             maze.is_wall({row + 1, col}, Direction::E))) {
          can_delete = true;
        }
        break;
      case Direction::S:
        if ((maze.is_wall({row, col}, Direction::W) or maze.is_wall({row, col - 1}, Direction::S) or
             maze.is_wall({row + 1, col}, Direction::W)) &&
            (maze.is_wall({row, col}, Direction::E) or maze.is_wall({row, col + 1}, Direction::S) or
             maze.is_wall({row + 1, col}, Direction::E))) {
          can_delete = true;
        }
        break;
      case Direction::W:
        if ((maze.is_wall({row, col}, Direction::N) or maze.is_wall({row, col - 1}, Direction::N) or
             maze.is_wall({row - 1, col}, Direction::W)) &&
            (maze.is_wall({row, col}, Direction::S) or maze.is_wall({row, col - 1}, Direction::S) or
             maze.is_wall({row + 1, col}, Direction::W))) {
          can_delete = true;
        }
        break;
      default:
        break;
    }

    if (can_delete) {
      maze.remove_wall_if_exists({row, col}, dir);
      i++;
    }
  }

  // knock down center square
  maze.remove_wall_if_exists({SIZE / 2, SIZE / 2}, Direction::N);
  maze.remove_wall_if_exists({SIZE / 2, SIZE / 2}, Direction::W);
  maze.remove_wall_if_exists({SIZE / 2 - 1, SIZE / 2 - 1}, Direction::S);
  maze.remove_wall_if_exists({SIZE / 2 - 1, SIZE / 2 - 1}, Direction::E);

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

Route AbstractMaze::truncate_route(RowCol const row_col, Direction const dir,
                                   Route const route) const {
  if (dir == Direction::Last) {
    throw std::invalid_argument("Invalid direction Last");
  }

  Route trunc;
  bool done = false;
  RowCol current_rc = row_col;
  for (MotionPrimitive prim : route) {
    for (unsigned int i = 0; i < prim.n; i++) {
      // check if this move is valid
      auto[valid, new_row_col] = step(current_rc, prim.d);
      auto const wall = is_wall(current_rc, prim.d);
      if (!valid or wall) {
        done = true;
        break;
      }

      // make the move
      current_rc = new_row_col;

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
      add_wall({sr.row, sr.col}, d);
    } else {
      remove_wall_if_exists({sr.row, sr.col}, d);
    }
  }
}

} // namespace ssim
