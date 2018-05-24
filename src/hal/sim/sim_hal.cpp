#include <cstdarg>
#include <vector>
#include <cstdio>

#include "hal.h"

namespace ssim {

void digitalWrite(unsigned int pin, bool high) {
  // TODO: implement digital for LEDs by doing a lookup into a table of pins that are registered as LEDs
}
void print(const char *fmt, ...) {
  char buf[1024]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt);
  vsnprintf(buf, sizeof(buf), (const char *) fmt, args); // for the rest of the world
  va_end(args);
  printf("%s", buf);
}

void csv_print(std::vector<double> values) {
  char buf[1024];
  char *b = (char *) buf;
  for (double value : values) {
    int c = sprintf(b, "%0.3f, ", value);
    b += c;
  }
  sprintf(b, "\r\n");
  printf("%s", buf);
}

}
