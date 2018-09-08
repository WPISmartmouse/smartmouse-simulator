#pragma once

#include <eigen3/Eigen/Eigen>

#include <experimental/optional>
#include <core/msgs.h>

namespace ssim {

class RayTracing {
 public:
  static std::experimental::optional<double>
  distance_to_wall(Line2d const &wall, Eigen::Vector2d const &pt, Eigen::Vector2d u);
};

} // namespace ssim
