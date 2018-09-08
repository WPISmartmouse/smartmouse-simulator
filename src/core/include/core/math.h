#pragma once

#include <cmath>
#include <limits>

#include <eigen3/Eigen/Eigen>

namespace ssim {

/** \brief Computes the signed shorted angle between y2 and y1. Check CommonTest.cpp to see examples
 *
 * @param y1 the second angle in the subtraction
 * @param y2 the first angle in the subtraction
 * @return the signed shorted angle between y2 and y1.
 */
constexpr double yaw_diff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}

constexpr double rad_to_deg(double rad) {
  return rad * 180 / (2 * M_PI);
}

// SOURCE: https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
// Floating-point modulo
// The result (the remainder) has same sign as the divisor.
// Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
template<typename T>
constexpr T MyMod(T x, T y) {
  static_assert(!std::numeric_limits<T>::is_exact, "Mod: floating-point type expected");

  if (0. == y)
    return x;

  double m = x - y * floor(x / y);

  // handle boundary cases resulted from floating-point cut off:

  if (y > 0)              // modulo range: [0..y)
  {
    if (m >= y)           // Mod(-1e-16             , 360.    ): m= 360.
      return 0;

    if (m < 0) {
      if (y + m == y)
        return 0; // just in case...
      else
        return y + m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
    }
  } else                    // modulo range: (y..0]
  {
    if (m <= y)           // Mod(1e-16              , -360.   ): m= -360.
      return 0;

    if (m > 0) {
      if (y + m == y)
        return 0; // just in case...
      else
        return y + m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
    }
  }

  return m;
}

// wrap [rad] angle to [-PI..PI)
constexpr double wrapAngleRad(double angle_rad) {
  return MyMod(angle_rad + M_PI, 2 * M_PI) - M_PI;
}

constexpr void wrapAngleRadInPlace(double *angle_rad) {
  *angle_rad = MyMod(*angle_rad + M_PI, 2 * M_PI) - M_PI;
}

class Line2d {

 public:
  Line2d(Eigen::Vector2d const &p1, Eigen::Vector2d const &p2);

  Line2d(double pt1_x, double pt1_y, double pt2_x, double pt2_y);

  bool Intersect(Line2d const &_line, Eigen::Vector2d &_pt, double _epsilon = 1e-6) const;

  double CrossProduct(Line2d const &_line) const;

  bool Within(Eigen::Vector2d const &_pt, double _epsilon = 1e-6) const;

 private:
  std::array<Eigen::Vector2d, 2> pts;
};

double Distance(Eigen::Vector2d p0, Eigen::Vector2d p1);


} // namespace ssim
