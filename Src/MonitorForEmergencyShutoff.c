#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForEmergencyShutoff.h"

static int MONITOR_FOR_EMERGENCY_SHUTOFF_PERIOD = 1000;

void monitorForEmergencyShutoffTask(void const* arg)
{
    osMutexId* canHandleMutex = (osMutexId*) arg;

    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_EMERGENCY_SHUTOFF_PERIOD);
    }
}
