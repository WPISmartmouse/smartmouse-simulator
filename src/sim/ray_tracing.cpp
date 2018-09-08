#include <core/maze.h>
#include <sim/ray_tracing.h>

namespace ssim {

std::optional<double> RayTracing::distance_to_wall(Line2d const &wall, Eigen::Vector2d const &pt, Eigen::Vector2d u) {
  // project along u the size of the maze
  Eigen::Vector2d u_proj(u);
  u_proj *= SIZE_M;

  Eigen::Vector2d intersection_point;
  Line2d sensor_ray(pt, pt + u_proj);
  bool intersects = wall.Intersect(sensor_ray, intersection_point);

  if (intersects) {
    double dist = Distance(pt, intersection_point);
    return std::optional<double>(dist);
  } else {
    return std::optional<double>();
  }
}

} // namespace ssim
