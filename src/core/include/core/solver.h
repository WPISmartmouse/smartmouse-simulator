#pragma once

#include <core/mouse.h>

namespace ssim {

class Solver {
public:
  enum class Goal {
    CENTER,
    START
  };

  explicit Solver(Mouse *mouse);

  virtual void setup() = 0;

  virtual MotionPrimitive planNextStep() = 0;

  virtual bool isFinished() = 0;

  virtual Route solve() = 0;

  virtual void teardown() {};

  virtual void setGoal(Goal goal) = 0;

  bool isSolvable();

  bool solvable;

  // Non-Owning pointer -- we will not free this memory, because we don't own it.
  Mouse *mouse;
};

} // namespace ssim
