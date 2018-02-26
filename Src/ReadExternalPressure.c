#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadExternalPressure.h"

#include "Data.h"

static int READ_EXTERNAL_PRESSURE_PERIOD = 1000;

void readExternalPressureTask(void const* arg)
{
    ExternalPressureData* data = (ExternalPressureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_EXTERNAL_PRESSURE_PERIOD);
    }
}
