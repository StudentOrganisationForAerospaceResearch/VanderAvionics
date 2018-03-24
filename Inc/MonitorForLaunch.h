#pragma once

#include "FlightPhase.h"

extern UART_HandleTypeDef huart1;
extern FlightPhase currentFlightPhase;

void monitorForLaunchTask(void const* arg);