#include <exception>

#include <core/flood.h>

namespace ssim {

Flood::Flood(Mouse *mouse) : Solver(mouse), done(false), goal(Goal::CENTER), solved(false) {}

void Flood::setup() {
  mouse->reset();
  mouse->maze.reset();
  no_wall_maze.remove_all_walls();
  all_wall_maze.add_all_walls();
  all_wall_maze.remove_wall(CENTER, Direction::W);
  all_wall_maze.remove_wall(CENTER, Direction::N);
  all_wall_maze.remove_wall({SIZE/2 - 1, SIZE/2 - 1}, Direction::E);
  all_wall_maze.remove_wall({SIZE/2 - 1, SIZE/2 - 1}, Direction::S);
  goal = Solver::Goal::CENTER;
}

void Flood::setGoal(Solver::Goal goal) {
  this->goal = goal;
}

MotionPrimitive Flood::planNextStep() {
  // mark the nodes visited in both the mazes
  no_wall_maze.mark_position_visited(mouse->getRowCol());
  all_wall_maze.mark_position_visited(mouse->getRowCol());

  // check left right back and front sides
  // eventually this will return values from sensors
  SensorReading sr = mouse->checkWalls();

  // update the mazes base on that reading
  no_wall_maze.update(sr);
  all_wall_maze.update(sr);

  // solve flood fill on the two mazes from mouse to goal
  switch (goal) {
    case Solver::Goal::CENTER: {
      solvable = no_wall_maze.flood_fill_from_point(&no_wall_path, mouse->getRowCol(), CENTER);
      all_wall_maze.flood_fill_from_point(&all_wall_path, mouse->getRowCol(), CENTER);
      break;
    }
    case Solver::Goal::START: {
      solvable = no_wall_maze.flood_fill_from_point(&no_wall_path, mouse->getRowCol(), {0, 0});
      all_wall_maze.flood_fill_from_point(&all_wall_path, mouse->getRowCol(), {0, 0});
      break;
    }
  }

  no_wall_maze.fastest_route = no_wall_path;

  // Walk along the no_wall_path as far as possible in the all_wall_maze
  // This will results in the longest path where we know there are no walls
  Route const nextPath = all_wall_maze.truncate_route(mouse->getRowCol(), no_wall_path);
  if (nextPath.empty()) {
    throw std::length_error("Plan length was zero because the first step in the path was deemed impossible.");
  } else {
    return nextPath.at(0);
  }
}

Route Flood::solve() {
  Route route;
  while (!isFinished()) {
    auto const step = planNextStep();
    insert_motion_primitive_back(&route, step);
    mouse->internalTurnToFace(step.d);
    for (auto i = 0; i < step.n; ++i) {
      mouse->internalForward();
    }
  }

  return route;
}

bool Flood::isFinished() {
  auto const r = mouse->getRow();
  auto const c = mouse->getCol();
  MazeIndex const C{SIZE / 2};
  switch (goal)
  {
    case Solver::Goal::CENTER:
      return !solvable || ((r >= C - 1 && r <= C) && (c >= C - 1 && c <= C));
    case Solver::Goal::START:
      return !solvable || (r == IDX_0 && c == IDX_0);
    default:
      throw std::invalid_argument("The Flood solver does not support the requested goal");
  }
}

} // namespace ssim
