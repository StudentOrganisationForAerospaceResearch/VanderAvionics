#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadGps.h"

static int READ_GPS_PERIOD = 1000;

void readGpsTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();
    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_GPS_PERIOD);
    }
}
