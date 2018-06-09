#pragma once

extern UART_HandleTypeDef huart1;

void engineControlTask(void const* arg);
extern uint8_t launchCmdReceived;
