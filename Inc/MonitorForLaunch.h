#pragma once

extern UART_HandleTypeDef huart1;
extern uint8_t drogueParachuteLaunched;

void monitorForLaunchTask(void const* arg);