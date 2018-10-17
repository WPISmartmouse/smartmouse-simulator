#include "smartmouse_2018_plugin.h"

Smartmouse2018Main robot_main;

int main() {
  robot_main.Setup();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
  while (true) {
    robot_main.Step();
  }
#pragma GCC diagnostic pop
}
