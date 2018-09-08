#pragma once

#include <optional>

#include <Eigen/Eigen>

#include <core/math.h>

namespace ssim {

class RayTracing {
 public:
  static std::optional<double> distance_to_wall(Line2d const &wall, Eigen::Vector2d const &pt, Eigen::Vector2d u);
};

} // namespace ssim
