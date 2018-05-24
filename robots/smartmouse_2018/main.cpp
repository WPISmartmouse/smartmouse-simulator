//#include <Timer.h>
#include <smartmouse/common/commanduino/CommanDuino.h>
#include <smartmouse/common/commands/SolveCommand.h>
#include <smartmouse/common/core/AbstractMouse.h>
#include <smartmouse/common/core/Flood.h>
#include <smartmouse/Plugin.h>
#include <smartmouse/hal/hal.h>

namespace ssim {

class Smartmouse2018 : public RobotPlugin {

public:
  Scheduler *scheduler;
  AbstractMouse *mouse;
  unsigned long last_t_us, last_blink_us;
  bool done = false;
  bool on = true;
  bool paused = false;

  Setup() {
    mouse = AbastractMouse::inst();
    mouse->setup();

    GlobalProgramSettings.quiet = false;

    scheduler = new Scheduler(new SolveCommand(new Flood(mouse)));

    last_t_us = timer.programTimeMs();
    last_blink_us = timer.programTimeMs();

  }

  void Loop() {
    AbastractMouse::checkVoltage();

    unsigned long now_us = timer.programTimeUs();
    double dt_us = now_us - last_t_us;

    if (now_us - last_blink_us > 1000000) {
      last_blink_us = now_us;
      digitalWrite(AbastractMouse::SYS_LED, static_cast<uint8_t>(on));
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
      digitalWrite(AbastractMouse::SYS_LED, 1);
      digitalWrite(AbastractMouse::LED_2, 1);
      digitalWrite(AbastractMouse::LED_4, 1);
      digitalWrite(AbastractMouse::LED_6, 1);
    }

    auto dt_s = dt_us / 1e6;

    mouse->run(dt_s);

    last_t_us = now_us;
  }

};

}
