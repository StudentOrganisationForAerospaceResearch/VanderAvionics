#pragma once

extern SPI_HandleTypeDef hspi2;

void readBarometerTask(void const* arg);
uint16_t readCalibrationCoefficient(const uint8_t PROM_READ_CMD);

