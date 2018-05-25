#include "hal.h"
#include "commands/Finish.h"

Finish::Finish(Smartmouse2018Robot &robot, ssim::AbstractMaze *maze) : Command("finish"), maze(maze), robot(robot),
                                                                       t(0), pin_id(0), on(false) {
}

void Finish::initialize() {
  robot.setSpeedCps(0, 0);
  setTimeout(2000);
  t = getTime();
  pin_id = Smartmouse2018Robot::LED_1;
  on = true;
}

void Finish::execute() {
  if (getTime() - t > BLINK_TIME) {

    on = !on;
    t = getTime();

    if (on) {
      ssim::digitalWrite(pin_id, 1);
    } else {
      ssim::digitalWrite(pin_id, 0);
      pin_id++;
      if (pin_id > Smartmouse2018Robot::LED_1) {
        pin_id = Smartmouse2018Robot::LED_2;
      }
    }
  }
}

bool Finish::isFinished() {
  return isTimedOut();
}