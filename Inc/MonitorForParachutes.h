#pragma once

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"

#include "cmsis_os.h"

void monitorForParachutesTask(void const* arg)
{
    for (;;)
    {
        osDelay(1);
    }
}
