#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadOxidizerTankConditions.h"

#include "Data.h"

static int READ_OXIDIZER_TANK_CONDITIONS_PERIOD = 1000;

void readOxidizerTankConditionsTask(void const* arg)
{
    OxidizerTankConditionsData* data = (OxidizerTankConditionsData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_OXIDIZER_TANK_CONDITIONS_PERIOD);
    }
}
