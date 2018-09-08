#include <core/math.h>

namespace ssim {


Line2d::Line2d(Eigen::Vector2d const &p1, Eigen::Vector2d const &p2) : pts{p1, p2} {}

Line2d::Line2d(double pt1_x, double pt1_y, double pt2_x, double pt2_y) {
  pts[0] = {pt1_x, pt1_y};
  pts[1] = {pt2_x, pt2_y};
}

bool Line2d::Intersect(Line2d const &_line, Eigen::Vector2d &_pt, double _epsilon) const {
  double d = this->CrossProduct(_line);

  // d is zero if the two line are collinear. Must check special cases.
  if (d <= _epsilon) {
    // Check if _line's starting point is on the line.
    if (this->Within(_line.pts[0], _epsilon)) {
      _pt = _line.pts[0];
      return true;
    }
      // Check if _line's ending point is on the line.
    else if (this->Within(_line.pts[1], _epsilon)) {
      _pt = _line.pts[1];
      return true;
    }
      // Other wise return false.
    else
      return false;
  }

  _pt(0) =
      (_line.pts[0](0) - _line.pts[1](0)) * (this->pts[0](0) * this->pts[1](1) - this->pts[0](1) * this->pts[1](0)) -
      (this->pts[0](0) - this->pts[1](0)) * (_line.pts[0](0) * _line.pts[1](1) - _line.pts[0](1) * _line.pts[1](0));

  _pt(1) =
      (_line.pts[0](1) - _line.pts[1](1)) * (this->pts[0](0) * this->pts[1](1) - this->pts[0](1) * this->pts[1](0)) -
      (this->pts[0](1) - this->pts[1](1)) * (_line.pts[0](0) * _line.pts[1](1) - _line.pts[0](1) * _line.pts[1](0));

  _pt /= d;

  if (_pt(0) < std::min(this->pts[0](0), this->pts[1](0)) || _pt(0) > std::max(this->pts[0](0), this->pts[1](0)) ||
      _pt(0) < std::min(_line.pts[0](0), _line.pts[1](0)) || _pt(0) > std::max(_line.pts[0](0), _line.pts[1](0))) {
    return false;
  }

  if (_pt(1) < std::min(this->pts[0](1), this->pts[1](1)) || _pt(1) > std::max(this->pts[0](1), this->pts[1](1)) ||
      _pt(1) < std::min(_line.pts[0](1), _line.pts[1](1)) || _pt(1) > std::max(_line.pts[0](1), _line.pts[1](1))) {
    return false;
  }

  return true;
}

double Line2d::CrossProduct(Line2d const &_line) const {
  return (this->pts[0](0) - this->pts[1](0)) * (_line.pts[0](1) - _line.pts[1](1)) -
         (this->pts[0](1) - this->pts[1](1)) * (_line.pts[0](0) - _line.pts[1](0));
}

bool Line2d::Within(Eigen::Vector2d const &_pt, double _epsilon) const {
  return _pt(0) <= std::max(this->pts[0](0), this->pts[1](0)) + _epsilon &&
         _pt(0) >= std::min(this->pts[0](0), this->pts[1](0)) - _epsilon &&
         _pt(1) <= std::max(this->pts[0](1), this->pts[1](1)) + _epsilon &&
         _pt(1) >= std::min(this->pts[0](1), this->pts[1](1)) - _epsilon;
}

double Distance(Eigen::Vector2d p0, Eigen::Vector2d p1) {
  return sqrt((p1(0) - p0(0)) * (p1(0) - p0(0)) + (p1(1) - p0(1)) * (p1(1) - p0(1)));
}

} // namespace ssim
