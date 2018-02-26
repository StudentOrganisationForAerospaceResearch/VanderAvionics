#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadOxidizerTankPressure.h"

#include "Data.h"

static int READ_OXIDIZER_TANK_PRESSURE_PERIOD = 1000;

void readOxidizerTankPressureTask(void const* arg)
{
    OxidizerTankData* data = (OxidizerTankData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_OXIDIZER_TANK_PRESSURE_PERIOD);
    }
}
