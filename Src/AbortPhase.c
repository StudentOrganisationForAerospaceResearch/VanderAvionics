#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "AbortPhase.h"

static const int PRELAUNCH_PHASE_PERIOD = 50;

void abortPhaseTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();

    int abortDetected = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);
        if(currentFlightPhase != ABORT && !abortDetected) {
            continue;
        }

        // This ensures the program does not leave abort phase
        // if it was ever triggered
        abortDetected = 1;
        currentFlightPhase = ABORT;
    }
}
