#include <hal/hal.h>
#include <core/mouse.h>

#include "commands/end.h"

End::End() : Command("end") {}

bool End::isFinished() {
  return true;
}

void End::end() {
  ssim::print("DONE.\r\n");
}
