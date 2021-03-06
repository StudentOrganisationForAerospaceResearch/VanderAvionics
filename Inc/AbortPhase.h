#pragma once

extern uint8_t launchCmdReceived;
extern uint8_t systemIsArmed;
extern uint8_t pulseVentValveRequested;
extern uint8_t abortCmdReceived;
extern uint8_t resetAvionicsCmdReceived;
const extern int32_t HEARTBEAT_TIMEOUT;
extern int32_t heartbeatTimer;


void abortPhaseTask(void const* arg);
