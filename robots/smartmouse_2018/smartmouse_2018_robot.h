#pragma once

#include <AS5048A.h>

#include <kinematic_controller/robot.h>

class Smartmouse2018Robot : public ssim::Robot {

public:

  static const unsigned int BACK_LEFT_ANALOG_PIN = 14;
  static const unsigned int FRONT_RIGHT_ANALOG_PIN = 15;
  static const unsigned int FRONT_ANALOG_PIN = 16;
  static const unsigned int GERALD_LEFT_ANALOG_PIN = 17;
  static const unsigned int FRONT_LEFT_ANALOG_PIN = 18;
  static const unsigned int GERALD_RIGHT_ANALOG_PIN = 19;
  static const unsigned int BACK_RIGHT_ANALOG_PIN = 20;
  static const unsigned int BATTERY_ANALOG_PIN = 21;

  // TODO: remove these and use new SPI encoders?
  static const unsigned int MOSI = 11;
  static const unsigned int LEFT_ENCODER_CS = 9;
  static const unsigned int RIGHT_ENCODER_CS = 10;

  static const uint8_t MOTOR_LEFT_A1 = 5;
  static const uint8_t MOTOR_LEFT_A2 = 6;
  static const uint8_t MOTOR_RIGHT_B1 = 8;
  static const uint8_t MOTOR_RIGHT_B2 = 7;

  static const uint8_t LED_1 = 25;
  static const uint8_t LED_2 = 26;
  static const uint8_t LED_3 = 27;
  static const uint8_t LED_4 = 28;
  static const uint8_t LED_5 = 29;
  static const uint8_t LED_6 = 30;
  static const uint8_t LED_7 = 31;
  static const uint8_t SYS_LED = 32;

  static const uint8_t FRONT_LEFT_ENABLE_PIN = 21;
  static const uint8_t GERALD_LEFT_ENABLE_PIN = 20;
  static const uint8_t GERALD_RIGHT_ENABLE_PIN = 22;
  static const uint8_t FRONT_RIGHT_ENABLE_PIN = 19;
  static const uint8_t FRONT_ENABLE_PIN = 18;
  static const uint8_t BACK_LEFT_ENABLE_PIN = 2;
  static const uint8_t BACK_RIGHT_ENABLE_PIN = 4;

  static const uint8_t BUTTON_PIN = 23;

  AS5048A left_encoder, right_encoder;

  Smartmouse2018Robot();

  void resetToStartPose();

  ssim::SensorReading checkWalls() override;

  void setup();

  void run(double dt_s);


  static double checkVoltage();
};
