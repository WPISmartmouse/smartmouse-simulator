#include "hal.h"
#include "mouse.h"

#include "commands/End.h"

End::End() : Command("end") {}

bool End::isFinished() {
  return true;
}

void End::end() {
  ssim::print("DONE.\r\n");
}
