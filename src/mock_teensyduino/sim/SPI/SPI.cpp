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
  return 0;
}
