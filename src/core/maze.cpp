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

void for_each_cell_and_dir(std::function<void(MazeIndex, MazeIndex, Direction)> f) {
  MazeIndex row, col;
  for (; row <= IDX_MAX; row++) {
    for (; col <= IDX_MAX; col++) {
      for (auto const d : wise_enum::range<Direction>) {
        f(row, col, d.value);
      }
    }
  }
}

void for_each_cell(std::function<void(MazeIndex, MazeIndex)> f) {
  MazeIndex row, col;
  for (; row <= IDX_MAX; row++) {
    for (; col <= IDX_MAX; col++) {
      f(row, col);
    }
  }
}

AbstractMaze::AbstractMaze() {
  // TODO remove all direct indexing with ints/unsigned ints
  // TODO remove all for loops over cells
  // Set the row and column of all the nodes
  for_each_cell([&](MazeIndex row, MazeIndex col) {
    auto &n = this->get_mutable_node({row, col});
    n = RowCol{row, col};

  });

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
      return row_col.row > IDX_0 ? StepResult{true, {row_col.row - 1, row_col.col}} : StepResult{false, {0, 0}};
    case Direction::S:
      return row_col.row < IDX_MAX ? StepResult{true, {row_col.row + 1, row_col.col}} : StepResult{false, {0, 0}};
    case Direction::E:
      return row_col.col < IDX_MAX ? StepResult{true, {row_col.row, row_col.col + 1}} : StepResult{false, {0, 0}};
    case Direction::W:
      return row_col.col > IDX_0 ? StepResult{true, {row_col.row, row_col.col - 1}} : StepResult{false, {0, 0}};
  }

  return {false, {0, 0}};
}

Node AbstractMaze::get_node(RowCol const row_col) const {
  return nodes[row_col.row.value][row_col.col.value];
}

Node &AbstractMaze::get_mutable_node(RowCol const row_col) {
  return nodes[row_col.row.value][row_col.col.value];
}

bool AbstractMaze::is_perimeter(RowCol const row_col, Direction dir) const {
  if (row_col.row == IDX_0 and dir == Direction::N) {
    return true;
  } else if (row_col.row == IDX_MAX and dir == Direction::S) {
    return true;
  } else if (row_col.col == IDX_0 and dir == Direction::W) {
    return true;
  } else if (row_col.col == IDX_MAX and dir == Direction::E) {
    return true;
  }
  return false;
}

bool AbstractMaze::is_wall(RowCol const row_col, Direction const dir) const {
  auto const n = get_node(row_col);
  auto const is_wall = n.walls[static_cast<int>(dir)];
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
  if (!is_perimeter(row_col, dir)) {
    auto &n = get_mutable_node(row_col);
    n.walls[static_cast<int>(dir)] = WallEnum::Wall;
  }

  // Add the wall from the other side if that's possible
  auto const[valid, new_row_col] = step(row_col, dir);
  if (valid and !is_perimeter(new_row_col, opposite_direction(dir))) {
    auto &new_n = get_mutable_node(new_row_col);
    new_n.walls[static_cast<int>(opposite_direction(dir))] = WallEnum::PerimeterWall;
  }
}

void AbstractMaze::remove_wall(RowCol const row_col, Direction const dir) {
  if (is_perimeter(row_col, dir)) {
    throw std::invalid_argument(
        fmt::format("row {} or col {} direction {} is a perimeter, which cannot be remove", row_col.row.value,
                    row_col.col.value, dir_to_char(dir)));
  }

  auto &n = get_mutable_node(row_col);
  if (n.walls[static_cast<int>(dir)] == WallEnum::NoWall) {
    throw std::invalid_argument(
        fmt::format("remove_wall: row {} col {} dir {} not in walls", row_col.row.value, row_col.col.value,
                    dir_to_char(dir)));
  }
  n.walls[static_cast<int>(dir)] = WallEnum::NoWall;

  // Remove the wall from the other side if that's possible
  auto const[valid, new_row_col] = step(row_col, dir);
  if (valid) {
    auto &new_n = get_mutable_node(new_row_col);
    new_n.walls[static_cast<int>(opposite_direction(dir))] = WallEnum::NoWall;
  }
}

void AbstractMaze::remove_wall_if_exists(RowCol const row_col, ssim::Direction const dir) {
  {
    if (is_perimeter(row_col, dir)) {
      throw std::invalid_argument(
          fmt::format("row {} or col {} direction {} is a perimeter, which cannot be remove", row_col.row.value,
                      row_col.col.value, dir_to_char(dir)));
    }


    auto &n = get_mutable_node(row_col);
    if (n.walls[static_cast<int>(dir)] == WallEnum::NoWall) {
      // this case is fine. The whole point of this function is to ignore this
      return;
    }
    n.walls[static_cast<int>(dir)] = WallEnum::NoWall;
  }

  // Remove the wall from the other side if that's possible
  auto const[valid, new_row_col] = step(row_col, dir);
  if (valid) {
    auto &new_n = get_mutable_node(new_row_col);
    new_n.walls[static_cast<int>(opposite_direction(dir))] = WallEnum::NoWall;
  }
}

void AbstractMaze::remove_all_walls() {
  for (unsigned int row = 0; row < SIZE; row++) {
    for (unsigned int col = 0; col < SIZE; col++) {
      for (auto const d : wise_enum::range<Direction>) {
        if (!is_perimeter({row, col}, d.value)) {
          nodes[row][col].walls[static_cast<int>(d.value)] = WallEnum::NoWall;
        }
      }
    }
  }
}

void AbstractMaze::add_all_walls() {
  for (unsigned int row = 0; row < SIZE; row++) {
    for (unsigned int col = 0; col < SIZE; col++) {
      for (auto const d : wise_enum::range<Direction>) {
        if (!is_perimeter({row, col}, d.value)) {
          nodes[row][col].walls[static_cast<int>(d.value)] = WallEnum::Wall;
        }
      }
    }
  }
}

void
AbstractMaze::assign_weights_to_neighbors(RowCol const start, RowCol const goal, int const weight, bool &goal_found) {
  // stop early if we find that a node has higher weight than our goal
  if (weight > get_node(goal).weight) {
    return;
  }

  // check all nodes that are unvisited, or would be given a lower weight
  auto &n = get_mutable_node(start);
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
    for (auto const d : wise_enum::range<Direction>) {
      auto const[valid, new_row_col] = step(start, d.value);
      auto const wall = is_wall(start, d.value);
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
  Node n = get_node(goal);
  auto const &start_node = get_node(start);
  while (n != start_node && solvable) {
    Node min_node = n;
    Direction min_dir = Direction::N;

    //find the neighbor with the lowest weight and go there,  that is the fastest route
    bool deadend = true;
    for (auto const d : wise_enum::range<Direction>) {
      RowCol const current_rc = n.GetRowCol();
      auto const[valid, new_row_col] = step(current_rc, d.value);
      auto const wall = is_wall(current_rc, d.value);
      if (valid and not wall) {
        auto next_n = get_node(new_row_col);
        if (next_n.weight < min_node.weight) {
          min_node = next_n;
          min_dir = opposite_direction(d.value);
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
  get_mutable_node(row_col).visited = true;
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

Route AbstractMaze::truncate_route(RowCol const row_col, Route const route) const {
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
  for (auto const d : wise_enum::range<Direction>) {
    //if a wall exists in that direction, add a wall
    //if no wall exists in that direction remove it
    if (sr.isWall(d.value)) {
      add_wall(sr.row_col, d.value);
    } else {
      remove_wall_if_exists(sr.row_col, d.value);
    }
  }
}

} // namespace ssim
