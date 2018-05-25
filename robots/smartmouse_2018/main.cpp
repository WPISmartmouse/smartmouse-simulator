#include "CommanDuino.h"
#include "mouse.h"
#include "Flood.h"
#include "Plugin.h"
#include "hal.h"

#include "commands/SolveCommand.h"

class Smartmouse2018 : public ssim::RobotPlugin {

public:
  Scheduler *scheduler;
  ssim::Mouse *mouse;
  unsigned long last_t_us, last_blink_us;
  bool done = false;
  bool on = true;
  bool paused = false;

  void Setup() {
    mouse = ssim::Mouse::inst();
    mouse->setup();

    scheduler = new Scheduler(new SolveCommand(new ssim::Flood(mouse)));

    last_t_us = timer.programTimeMs();
    last_blink_us = timer.programTimeMs();

  }

  void Loop() {
    ssim::Mouse::checkVoltage();

    unsigned long now_us = timer.programTimeUs();
    double dt_us = now_us - last_t_us;

    if (now_us - last_blink_us > 1000000) {
      last_blink_us = now_us;
      ssim::digitalWrite(ssim::Mouse::SYS_LED, static_cast<uint8_t>(on));
      on = !on;
    }

    // minimum period of main loop
    if (dt_us < 1500) {
      return;
    }

    if (not paused and not done) {
      done = scheduler->run();
    } else {
      mouse->setSpeedCps(0, 0);
      ssim::digitalWrite(Mouse::SYS_LED, 1);
      ssim::digitalWrite(Mouse::LED_2, 1);
      ssim::digitalWrite(Mouse::LED_4, 1);
      ssim::digitalWrite(Mouse::LED_6, 1);
    }

    auto dt_s = dt_us / 1e6;

    mouse->run(dt_s);

    last_t_us = now_us;
  }

};
