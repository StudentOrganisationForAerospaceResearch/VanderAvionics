#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "AbortPhase.h"
#include "FlightPhase.h"

static const int PRELAUNCH_PHASE_PERIOD = 50;

void abortPhaseTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);

        if (getCurrentFlightPhase() == ABORT)
        {
            continue;
        }

        // close injection valve
        // open vent valve
    }
}
