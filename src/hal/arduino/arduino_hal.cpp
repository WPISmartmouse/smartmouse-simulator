#include <Arduino.h>

namespace ssim {

void digitalWrite(unsigned int pin, bool high) {
  ::digitalWrite(pin, high);
}

}
