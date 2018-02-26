#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadPressure.h"

static int READ_PRESSURE_PERIOD = 1000;

void readPressureTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();
    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_PRESSURE_PERIOD);
    }
}
