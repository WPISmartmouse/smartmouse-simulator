#pragma once

#include <core/msgs.h>
#include <as5048a.h>

#include <kinematic_controller/robot.h>

class Smartmouse2018Robot : public ssim::Robot {

public:

  Smartmouse2018Robot();

  void resetToStartPose();

  ssim::SensorReading checkWalls() override;

  void Setup();

  void Run(double dt_s);

  static double checkVoltage();

 private:
  AS5048A left_encoder;
  AS5048A right_encoder;
};
