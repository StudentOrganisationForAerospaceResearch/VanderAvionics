#pragma once

#include "FlightPhase.h"

extern FlightPhase currentFlightPhase;

void monitorForEmergencyShutoffTask(void const* arg);