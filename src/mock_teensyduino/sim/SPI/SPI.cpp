#include <hal/hal.h>

#include "SPI.h"

SPI_t SPI;

SPISettings::SPISettings(int, int, int) {}

void SPI_t::begin() {

}

void SPI_t::end() {

}

void SPI_t::beginTransaction(SPISettings) {

}

void SPI_t::endTransaction() {

}

byte SPI_t::transfer(byte b) {
  static unsigned int index = 0;
  if (b == 0x0) {
    if (index == 0) {
      return ssim::robot_description.left_encoder.ticks;
    }
    else {
      index = 0;
      return ssim::robot_description.right_encoder.ticks;
    }
  }
}
