#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "AbortPhase.h"
#include "FlightPhase.h"
#include "EngineControl.h"

static const int ABORT_PHASE_TASK_PERIOD = 50;
static const int VENT_VALVE_PULSE_PERIOD = 3000;
static const int ABORT_INJECTION_DELAY = 2 * 60 * 1000; // 5 min, * 60 sec/min * 1000msec/sec

void abortPhaseTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();
    uint32_t timeInAbort = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, ABORT_PHASE_TASK_PERIOD);

        if (getCurrentFlightPhase() == ABORT)
        {
            // pulse vent valve
            for (;;)
            {
                openVentValve();
                osDelay(VENT_VALVE_PULSE_PERIOD);
                closeVentValve();
                osDelay(VENT_VALVE_PULSE_PERIOD);
            }

            // overflow is purposefully not handled
            // that would be an extremely long time and not likely to happen
            timeInAbort += ABORT_PHASE_TASK_PERIOD;

            // open injection valve after a certain amount of time being in abort
            if (timeInAbort > ABORT_INJECTION_DELAY)
            {
                openInjectionValve();
            }
            else
            {
                closeInjectionValve();
            }

        }
        else
        {
            timeInAbort = 0;
        }
    }
}
