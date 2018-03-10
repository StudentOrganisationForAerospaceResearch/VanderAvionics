#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadAccelGyroMagnetism.h"

#include "Data.h"

static int READ_ACCEL_GYRO_MAGNETISM = 1000;

void readAccelGyroMagnetismTask(void const* arg)
{
    AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_ACCEL_GYRO_MAGNETISM);
    }
}
