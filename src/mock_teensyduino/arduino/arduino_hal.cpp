#include "Arduino.h"

namespace ssim {

}

extern "C" {
int _getpid() { return -1; }
int _kill(int pid, int sig) { return -1; }
}

