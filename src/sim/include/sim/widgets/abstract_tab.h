#pragma once

#include <QtCore/QString>

namespace ssim {

class AbstractTab {

  virtual const QString GetTabName() = 0;

};

} // namespace ssim
