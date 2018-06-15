#pragma once

extern uint8_t launchCmdReceived;
extern uint8_t abortCmdReceived;

void abortPhaseTask(void const* arg);
