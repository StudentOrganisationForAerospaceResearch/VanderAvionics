#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "LogData.h"

#include "Data.h"

static int LOG_DATA_PERIOD = 1000;

void logDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, LOG_DATA_PERIOD);
    }
}
