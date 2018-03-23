#pragma once

extern UART_HandleTypeDef huart1;
extern uint8_t apogeeDetected;

void monitorForLaunchTask(void const* arg);