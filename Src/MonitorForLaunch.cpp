#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForLaunch.h"

static int MONITOR_FOR_LAUNCH_PERIOD = 1000;

void monitorForLaunchTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();
    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_LAUNCH_PERIOD);
    }
}
