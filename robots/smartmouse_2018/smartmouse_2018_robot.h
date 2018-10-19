#pragma once

#include <core/msgs.h>
#include <as5048a.h>

#include <kinematic_controller/robot.h>

class Smartmouse2018Robot : public ssim::Robot {

 public:

  Smartmouse2018Robot();

  void resetToStartPose();

  ssim::SensorReading checkWalls() override;

  static double checkVoltage();

  AS5048A left_encoder;
  AS5048A right_encoder;
};
