#include <hal/hal.h>

#include "test_plugin.h"

void TestPlugin::Setup()  {
}

void TestPlugin::Step() {
}

ssim::RobotPlugin *ssim::global_plugin = new TestPlugin();
