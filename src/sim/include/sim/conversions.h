#pragma once

#include <core/node.h>
#include <sim/msgs.h>

namespace ssim {

/// A series of funtions to convert various things into the above messages
RobotDescription Convert(std::ifstream const &fs);
std::array<Line2d, 16> Convert(Node const &node);

} // namespace ssim
