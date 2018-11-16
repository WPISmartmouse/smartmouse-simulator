#include <core/plugin.h>

namespace ssim {

Debug const &RobotPlugin::get_debug() const noexcept {
  return debug_;
}

} // namespace ssim
