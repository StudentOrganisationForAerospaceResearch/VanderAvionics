#pragma once

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

extern int ventValveIsOpen;
extern int injectionValveIsOpen;
extern uint8_t launchSystemsRxChar;

void transmitDataTask(void const* arg);
