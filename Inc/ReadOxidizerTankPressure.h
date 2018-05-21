#pragma once

extern ADC_HandleTypeDef hadc2;

void readOxidizerTankPressureTask(void const* arg);

uint16_t getAverageAdcReading();