#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForParachutes.h"
#include "Data.h"
#include "main.h"

static int MONITOR_FOR_PARACHUTES_PERIOD = 1000;
uint8_t drogueParachuteLaunched = 0;

float readAccel(AccelGyroMagnetismData* data)
{
    osMutexWait(data->mutex_, 0);
    float accelX = data->accelX_;
    float accelY = data->accelY_;
    float accelZ = data->accelZ_;
    osMutexRelease(data->mutex_);

    float accelMagnitude =
        sqrt(
            accelX * accelX +
            accelY * accelY +
            accelZ * accelZ
        );

    return accelMagnitude;
}

float readPressure(ExternalPressureData* data)
{
    osMutexWait(data->mutex_, 0);
    int pressure = data->externalPressure_;
    osMutexRelease(data->mutex_);

    return (float)pressure;
}

void filterSensors(float current_accel, float current_pressure, float positionVector[3])
{
    // TODO
    positionVector[0] = 0.f;
    positionVector[1] = 0.f;
    positionVector[2] = 0.f;
}

int detectApogee(float positionVector[3])
{
    // TODO
    return 0;
}

void ejectDrogueParachute()
{
    // TODO
}

void ejectMainParachute()
{
    // TODO
}

void monitorForParachutesTask(void const* arg)
{
    uint32_t prevWakeTime = osKernelSysTick();

    MonitorForParachuteData* data = (MonitorForParachuteData*) arg;
    AccelGyroMagnetismData* accelGyroMagnetismData = data->accelGyroMagnetismData_;
    ExternalPressureData* externalPressureData = data->externalPressureData_;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        float currentAccel = readAccel(accelGyroMagnetismData);
        float currentPressure = readPressure(externalPressureData);

        float positionVector[3];
        filterSensors(currentAccel, currentPressure, positionVector);

        if (!drogueParachuteLaunched)
        {
            if (detectApogee(positionVector))
            {
                ejectDrogueParachute();
                drogueParachuteLaunched = 1;
            }
        }
        else
        {
            // TODO
            // detect 4600 ft above sea level and launch
            if (0)
            {
                ejectMainParachute();
                osThreadSuspend(osThreadGetId()); // kill thread
                break;
            }

        }
    }

    for (;;)
    {
        // do nothing incase thread doesn't die
    }
}
