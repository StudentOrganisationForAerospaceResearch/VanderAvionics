#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "LogData.h"

static int LOG_DATA_PERIOD = 1000;

void logData(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();
    for (;;)
    {
        osDelayUntil(&prevWakeTime, LOG_DATA_PERIOD);
    }
}
