#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "TransmitData.h"

#include "Data.h"

static int TRANSMIT_DATA_PERIOD = 1000;

void transmitDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, TRANSMIT_DATA_PERIOD);
    }
}
