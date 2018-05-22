#pragma once

extern ADC_HandleTypeDef hadc1;

void readCombustionTankPressureTask(void const* arg);

uint16_t getAverageAdc1Reading();