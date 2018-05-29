#include "Arduino.h"

namespace ssim {

}

int main() {
  setup();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
  while (true) {
    loop();
  }
#pragma GCC diagnostic pop
  return 0;
}
