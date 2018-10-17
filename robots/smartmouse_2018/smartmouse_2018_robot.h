#pragma once

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
};
