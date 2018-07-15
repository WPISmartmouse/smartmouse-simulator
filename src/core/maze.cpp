#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <random>
#include <sstream>
#include <string>
#include <iostream>

#include <core/abstract_maze.h>

namespace ssim {

AbstractMaze::AbstractMaze() : solved(false) {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      nodes[i][j] = new Node(i, j);
    }
  }
}

AbstractMaze::AbstractMaze(AbstractMaze const &m) : AbstractMaze() {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      nodes[i][j]->distance = m.nodes[i][j]->distance;
      nodes[i][j]->known = m.nodes[i][j]->known;
      nodes[i][j]->neighbors = m.nodes[i][j]->neighbors;
    }
  }
}

#ifndef REAL
AbstractMaze::AbstractMaze(std::ifstream &fs) : AbstractMaze() {
  std::string line;

  //look West and North to connect any nodes
  for (unsigned int i = 0; i < SIZE; i++) {
    std::getline(fs, line);

    if (!fs.good()) {
      std::cerr << "getline failed\n";
      return;
    }

    unsigned int charPos = 0;
    for (unsigned int j = 0; j < SIZE; j++) {
      if (line.at(charPos) != '|') {
        connect_neighbor(i, j, Direction::W);
      }
      charPos++;
      if (line.at(charPos) != '_') {
        connect_neighbor(i, j, Direction::S);
      }
      charPos++;
    }
  }
  printf("\n");
}
#endif

int AbstractMaze::get_node(Node **const out, const unsigned int row, const unsigned int col) const {
  if (col < 0 || col >= SIZE || row < 0 || row >= SIZE) {
    return Node::OUT_OF_BOUNDS;
  }

  if (nodes[row][col] == nullptr) {
    return -1;
  }
  *out = nodes[row][col];
  return 0;
}

int AbstractMaze::get_node_in_direction(Node **const out, const unsigned int row, const unsigned int col,
                                        const Direction dir) const {
  switch (dir) {
    case Direction::N:
      return get_node(out, row - 1, col);
      break;
    case Direction::E:
      return get_node(out, row, col + 1);
      break;
    case Direction::S:
      return get_node(out, row + 1, col);
      break;
    case Direction::W:
      return get_node(out, row, col - 1);
      break;
    default:
      return -1;
  }
}

void AbstractMaze::reset() {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      nodes[i][j]->weight = -1;
      nodes[i][j]->known = false;
    }
  }
}

void AbstractMaze::update(SensorReading const sr) {
  for (Direction d = Direction::First; d < Direction::Last; d++) {
    //if a wall exists in that direction, add a wall
    if (sr.isWall(d)) {
      disconnect_neighbor(sr.row, sr.col, d);
    }
      //if no wall exists in that direction remove it
    else {
      //getting the previously unconnected neighbor represents adding a wall
      //make sure to update both the node and the node it now connects to
      connect_neighbor(sr.row, sr.col, d);
    }
  }
}

bool
AbstractMaze::flood_fill_from_point(route_t *const path, unsigned int const r0, unsigned int const c0,
                                    unsigned int const r1, unsigned int const c1) {
  return flood_fill(path, r0, c0, r1, c1);
}

bool AbstractMaze::flood_fill_from_origin(route_t *const path, unsigned int const r1, unsigned int const c1) {
  return flood_fill(path, 0, 0, r1, c1);
}

bool AbstractMaze::flood_fill_from_origin_to_center(route_t *const path) {
  return flood_fill(path, 0, 0, SIZE / 2, SIZE / 2);
}

bool AbstractMaze::flood_fill(route_t *const path, unsigned int const r0, unsigned int const c0, unsigned int const r1,
                              unsigned int const c1) {
  Node *n;
  Node *goal = nodes[r1][c1];

  //incase the maze has already been solved,  reset all weight and known values
  reset();

  if (r0 == r1 && c0 == c1) {
    return true;
  }

  //explore all neighbors of the current node starting  with a weight of 1
  //return 1 means path to goal was found
  bool success = false;
  bool solvable = true;

  //start at the goal
  n = goal;

  //recursively visits all neighbors
  nodes[r0][c0]->assign_weights_to_neighbors(n, 0, &success);
  path->clear();

  //if we solved the maze,  traverse from goal back to root and record what direction is shortest
  while (n != nodes[r0][c0] && solvable) {
    Node *min_node = n;
    Direction min_dir = Direction::N;

    //find the neighbor with the lowest weight and go there,  that is the fastest route
    Direction d;
    bool deadend = true;
    for (d = Direction::First; d < Direction::Last; d++) {
      if (n->neighbor(d) != nullptr) {
        if (n->neighbor(d)->weight < min_node->weight) {
          min_node = n->neighbor(d);
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

void AbstractMaze::disconnect_neighbor(unsigned int const row, unsigned int const col, Direction const dir) {
  Node *n1 = nullptr;
  int n1_status = get_node(&n1, row, col);
  Node *n2 = nullptr;
  int n2_status = get_node_in_direction(&n2, row, col, dir);

  if (n1_status != Node::OUT_OF_BOUNDS) {
    n1->neighbors[static_cast<int>(dir)] = nullptr;
  }

  if (n2_status != Node::OUT_OF_BOUNDS) {
    n2->neighbors[static_cast<int>(opposite_direction(dir))] = nullptr;
  }
}

void AbstractMaze::connect_neighbor(unsigned int const row, unsigned int const col, Direction const dir) {
  Node *n1 = nullptr;
  int n1_status = get_node(&n1, row, col);
  Node *n2 = nullptr;
  int n2_status = get_node_in_direction(&n2, row, col, dir);

  Direction opposite = opposite_direction(dir);

  if ((n1_status != Node::OUT_OF_BOUNDS) && (n2_status != Node::OUT_OF_BOUNDS)) {
    n1->neighbors[static_cast<int>(dir)] = n2;
    n2->neighbors[static_cast<int>(opposite)] = n1;
  }
}

void AbstractMaze::connect_all_neighbors(unsigned int const row, unsigned int const col) {
  for (Direction d = Direction::First; d < Direction::Last; d++) {
    connect_neighbor(row, col, d);
  }
}

void AbstractMaze::connect_all_neighbors_in_maze() {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      connect_all_neighbors(i, j);
    }
  }
}

void AbstractMaze::mark_position_visited(unsigned int const row, unsigned int const col) {
  nodes[row][col]->visited = true;
}

void AbstractMaze::mark_origin_known() {
  nodes[0][0]->known = true;
}

AbstractMaze AbstractMaze::gen_random_legal_maze() {
  AbstractMaze maze;

  // start at center and move out, marking visited nodes as we go
  maze.mark_position_visited(SIZE / 2, SIZE / 2);
  maze.mark_position_visited(SIZE / 2 - 1, SIZE / 2);
  maze.mark_position_visited(SIZE / 2, SIZE / 2 - 1);
  maze.mark_position_visited(SIZE / 2 - 1, SIZE / 2 - 1);

  // pick std::random start node of the four possible ones;
  unsigned int starting_row = SIZE / 2 - std::rand() % 2;
  unsigned int starting_col = SIZE / 2 - std::rand() % 2;
  Node *start_node = nullptr;
  maze.get_node(&start_node, starting_row, starting_col);
  make_connections(&maze, start_node);

  // knock down some more randomly
  unsigned int i = 0;
  while (i < SIZE * SIZE / 5) {
    unsigned int row = std::rand() % (SIZE - 2) + 1;
    unsigned int col = std::rand() % (SIZE - 2) + 1;
    int d = std::rand() % 4;
    Direction dir = int_to_dir(d);

    // check if that's a valid wall to knock down
    bool can_delete = false;

    switch (dir) {
      case Direction::N: {
        Node *left = nullptr;
        Node *right = nullptr;
        Node *above = nullptr;
        maze.get_node(&left, row, col - 1);
        maze.get_node(&right, row, col + 1);
        maze.get_node(&above, row - 1, col);
        if ((left->wall(Direction::N) || left->wall(Direction::E) || above->wall(Direction::W)) &&
            (right->wall(Direction::N) || right->wall(Direction::W) || above->wall(Direction::E))) {
          can_delete = true;
        }
      }
        break;
      case Direction::E: {
        Node *below = nullptr;
        Node *right = nullptr;
        Node *above = nullptr;
        maze.get_node(&below, row + 1, col);
        maze.get_node(&right, row, col + 1);
        maze.get_node(&above, row - 1, col);
        if ((above->wall(Direction::S) || above->wall(Direction::E) || right->wall(Direction::N)) &&
            (below->wall(Direction::N) || below->wall(Direction::E) || right->wall(Direction::S))) {
          can_delete = true;
        }
      }
        break;
      case Direction::S: {
        Node *left = nullptr;
        Node *right = nullptr;
        Node *below = nullptr;
        maze.get_node(&left, row, col - 1);
        maze.get_node(&right, row, col + 1);
        maze.get_node(&below, row + 1, col);
        if ((left->wall(Direction::S) || left->wall(Direction::E) || below->wall(Direction::W)) &&
            (right->wall(Direction::S) || right->wall(Direction::W) || below->wall(Direction::E))) {
          can_delete = true;
        }
      }
        break;
      case Direction::W: {
        Node *below = nullptr;
        Node *left = nullptr;
        Node *above = nullptr;
        maze.get_node(&below, row + 1, col);
        maze.get_node(&left, row, col - 1);
        maze.get_node(&above, row - 1, col);
        if ((above->wall(Direction::S) || above->wall(Direction::W) || left->wall(Direction::N)) &&
            (below->wall(Direction::N) || below->wall(Direction::W) || left->wall(Direction::S))) {
          can_delete = true;
        }
      }
        break;
      default:
        break;
    }

    if (can_delete) {
      maze.connect_neighbor(row, col, dir);
      i++;
    }
  }

  // knock down center square
  maze.connect_neighbor(SIZE / 2, SIZE / 2, Direction::N);
  maze.connect_neighbor(SIZE / 2, SIZE / 2, Direction::W);
  maze.connect_neighbor(SIZE / 2 - 1, SIZE / 2 - 1, Direction::S);
  maze.connect_neighbor(SIZE / 2 - 1, SIZE / 2 - 1, Direction::E);

  return maze;
}

void AbstractMaze::make_connections(AbstractMaze *maze, Node *node) {
  static std::random_device rd;
  static std::mt19937 g(rd());

  unsigned int r = node->row();
  unsigned int c = node->col();
  maze->mark_position_visited(r, c);

  // shuffle directions
  std::vector<Direction> dirs = {Direction::N, Direction::E, Direction::S, Direction::W};
  std::shuffle(dirs.begin(), dirs.end(), g);

  for (auto d : dirs) {
    Node *neighbor;
    int result = maze->get_node_in_direction(&neighbor, r, c, d);
    if (result != Node::OUT_OF_BOUNDS && result != -1) {
      if (!neighbor->visited) {
        maze->connect_neighbor(r, c, d);
        make_connections(maze, neighbor);
      }
    }
  }
}

std::string route_to_string(route_t const &route) {
  std::stringstream ss;
  if (route.empty()) {
    ss << "empty";
  }

  for (motion_primitive_t prim : route) {
    ss << (int) prim.n << dir_to_char(prim.d);
  }

  return ss.str();
}

unsigned int expanded_route_length(route_t const &route) {
  return std::accumulate(route.begin(), route.end(), 0u,
                         [](unsigned int accum, const motion_primitive_t &prim) { return accum + prim.n; });
}

void insert_motion_primitive_back(route_t *route, motion_primitive_t const prim) {
  if (!route->empty() && prim.d == route->back().d) {
    route->back().n += prim.n;
  } else {
    route->insert(route->cend(), prim);
  }
}

void insert_motion_primitive_front(route_t *route, motion_primitive_t const prim) {
  if (!route->empty() && prim.d == route->front().d) {
    route->front().n += prim.n;
  } else {
    route->insert(route->cbegin(), prim);
  }
}

route_t
AbstractMaze::truncate(unsigned int const row, unsigned int const col, Direction const dir, route_t const route) const {
  route_t trunc;
  Node *n = nodes[row][col];
  bool done = false;
  for (motion_primitive_t prim : route) {
    for (unsigned int i = 0; i < prim.n; i++) {
      // check if this move is valid
      if (n->wall(prim.d)) {
        done = true;
        break;
      }

      // if it's valid, add and simulate the move
      n = n->neighbor(prim.d);
      insert_motion_primitive_back(&trunc, {1, prim.d});
    }
    if (done) {
      break;
    }
  }
  return trunc;
}

bool AbstractMaze::operator==(AbstractMaze const &other) const {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      for (Direction d = Direction::First; d != Direction::Last; d++) {
        auto n1 = static_cast<bool>(nodes[i][j]->neighbor(d));
        auto n2 = static_cast<bool>(other.nodes[i][j]->neighbor(d));
        if (n1 ^ n2) {
          return false;
        }
      }
    }
  }
  return true;
}

AbstractMaze::~AbstractMaze() {
  unsigned int i, j;
  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      delete nodes[i][j];
    }
  }
}

} // namespace ssim
