#include "Solver.h"

namespace ssim {
Solver::Solver(Mouse *mouse) : solvable(true), mouse(mouse) {}

bool Solver::isSolvable() {
  return solvable;
}

} // namespace ssim
