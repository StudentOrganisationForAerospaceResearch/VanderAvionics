#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForParachutes.h"

#include "Data.h"

static int MONITOR_FOR_PARACHUTES_PERIOD = 1000;

void monitorForParachutesTask(void const* arg)
{
    MonitorForParachuteData* data = (MonitorForParachuteData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);
    }
}
