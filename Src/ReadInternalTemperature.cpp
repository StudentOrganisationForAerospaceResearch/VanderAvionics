#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadInternalTemperature.h"

static int READ_INTERNAL_TEMPERATURE_PERIOD = 1000;

void readInternalTemperatureTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();
    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_INTERNAL_TEMPERATURE_PERIOD);
    }
}
